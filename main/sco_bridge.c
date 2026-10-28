/*
 * SCO audio bridge — see sco_bridge.h.
 *
 * Adapted from repo/firmware hfp_client_mgr.c (outgoing ring + 7.5 ms TX
 * pump + HWM trim) and audio_bridge.c (resampling via audio_codec.c),
 * reduced to what the WebRTC session needs.
 */

#include "sco_bridge.h"

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"

#include "config_manager.h"
#include "esp_hf_client_legacy_api.h"
#include "nvs.h"

#include "audio_codec.h"

static const char *TAG = "sco";

/* ---------- adaptive downlink consumer (P1) ----------

 * The SCO/air (BT controller) audio clock and the server RTP clock differ by
 * tens of ppm. A fixed-rate reader lets that drift pile up in the ring: the
 * level pins at the HWM (trims = hard frame drops, the wired-call stutter)
 * or runs dry (pad holes). This consumer still emits exactly the byte count
 * the BT stack asks per cb, but pulls the ring at 1.0 +/- MAX_DELTA per cb,
 * steered by the level error vs SPK_TARGET_MS: level high -> consume faster,
 * low -> slower. Within +/-0.4 % the change is inaudible.
 */
#define SPK_RATIO_ONE   (1u << 16)
#define SPK_MAX_DELTA   262u                  /* 65536*0.004 = 0.4 %   */
#define SPK_SPAN_MS     120
#define SPK_DEADBAND_MS 150                   /* no resampling within +-150 ms */
#define SPK_TRIM_MARGIN_MS 40                 /* shed silence above target+40 ms */
#define SPK_TRIM_MAX_SAMP  480                /* <= 30 ms per callback           */
#define SPK_QUIET_ABS      120                /* "silent" = |sample| below this  */                   /* level span -> scale    */
#ifndef SPK_TARGET_MS
#define SPK_TARGET_MS   SPK_PREFILL_MS        /* default aim = prefill  */
#endif
static uint32_t s_spk_off_q16;                /* fractional consume pos */

/* Scratch for CVSD resampling: 20 ms @ 16 kHz = 320 samples, bursts 2x. */
#define SCRATCH_MAX 512

/* ---------- negotiated link state ---------- */

/* sco_codec_t (SCO_CODEC_CVSD / SCO_CODEC_MSBC) comes from audio_codec.h */

static volatile sco_codec_t s_codec = SCO_CODEC_MSBC;
static volatile bool s_sco_active = false;

/* ---------- rings (PSRAM, byte granular, power of 2) ---------- */

/* 16 KB per ring in PSRAM: mSBC needs 32 B/ms, i.e. 280 ms HWM + headroom
 * (the old 8 KB capped the spk budget at 256 ms and everything above
 * 200 ms was trimmed into audible frame drops at every turn onset). */
#define RING_SIZE   16384u
#define RING_MASK   (RING_SIZE - 1u)

static uint8_t *s_mic_buf = NULL;
static uint16_t s_mic_wr = 0, s_mic_rd = 0;
static portMUX_TYPE s_mic_mux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t *s_spk_buf = NULL;
static uint16_t s_spk_wr = 0, s_spk_rd = 0;
static portMUX_TYPE s_spk_mux = portMUX_INITIALIZER_UNLOCKED;

/* 16-bit LE PCM sample fetch from a byte ring, wrap-safe (ESP32 is LE,
 * PSRAM reads are cheap; called ≤120x/cb). */
static inline int16_t spk_fetch_sample(uint16_t rd, int32_t k)
{
    uint16_t idx = (uint16_t)((rd + (uint16_t)(k * 2)) & RING_MASK);
    uint16_t idx1 = (uint16_t)((idx + 1) & RING_MASK);
    union { uint8_t b[2]; int16_t v; } u;
    u.b[0] = s_spk_buf[idx];
    u.b[1] = s_spk_buf[idx1];
    return u.v;
}

/* P0 inter-timing diagnostics state (reset in sco_open). */
static int64_t s_last_cb_us;
static int64_t s_last_write_us;
static bool    s_last_write_first = true;

/* HWM: keep at most this much queued audio, trim above. The wired-call
 * W-log showed upstream delivery holes of 100-150 ms (wrgap60/150) at
 * every robot-turn onset: a 200 ms HWM with a 60 ms cushion both starved
 * (pad holes mid-word) and trimmed (burst against HWM) in the same
 * window. 360 ms of headroom above the 200 ms aim turns those bursts into
 * pure latency, which the silent gaps between turns drain back. */
#define SPK_HWM_MS  360u
#define MIC_HWM_MS  160u/* Prefill at SCO open: measured downlink loss is ~0.5% (WiFi + rare UDP
 * reorders), i.e. ~5 ms of deficit per second — a 150 ms cushion covers a
 * 30 s call end to end and absorbs send-side pacing jitter bursts. */
#ifdef CONFIG_GW_NET_ETHERNET
/* Wired delivery is *pacing-jittered*, not steady: the peer loop hands
 * frames over in bursts with 100-150 ms holes at turn starts, so the
 * wired build needs the same cushion (still larger in effect — user
 * latency budget allows it and onset clarity profits). */
#define SPK_PREFILL_MS 200u
#else
/* WiFi + SCO coexistence: eSCO reserved slots carve the airtime into
 * blackout windows (ring swings -1..380 ms, 30-50% packets lost inside).
 * NOTE: prefill 400 + coex-prefer-WIFI tested 2026-10-28 → phone played
 * NOISE and the pump consumption read 2.3x normal — reverted to 150 until
 * the interaction is understood (suspect: controller HCI request pattern
 * changes / partial-read underflow path). */
#define SPK_PREFILL_MS 150u
#endif

/* ---------- pump timer ---------- */

static esp_timer_handle_t s_pump_timer = NULL;
static volatile bool s_pump_running = false;

/* TX is clocked by the controller: like the IDF hfp_hf example, request the
 * next outgoing frame from the incoming SCO callback, which fires in lock-
 * step with the phone's SCO slots. A free-running esp_timer drifts against
 * that clock and jitters (callback gaps up to 33 ms seen), starving or
 * double-feeding the controller; the phone's mSBC PLC then adds the grain.
 * The timer stays only as a fallback when no incoming data shows up. */
#define PUMP_FALLBACK_US 20000
static volatile int64_t s_last_in_us;

static void pump_cb(void *arg)
{
    (void)arg;
    if (s_sco_active && esp_timer_get_time() - s_last_in_us > PUMP_FALLBACK_US) {
        esp_hf_client_outgoing_data_ready();
    }
}

/* ---------- stats ---------- */

static sco_bridge_stats_t s_stats;
static bool s_auto_answer = false;

/* ---------- HFP link management (production parity) ---------- */

#define BT_DEVICE_NAME_DEFAULT "blu-qwen-webrtc"   /* menuconfig default       */
#define LINK_CHECK_PERIOD_MS  8000u   /* let the phone win the initial connect */
#define CONNECT_WATCHDOG_MS   20000u  /* force-clear a hung CONNECTING state */
#define ANSWER_DELAY_MS       200u    /* first check; real wait is for voice_ready */
#define ANSWER_MAX_WAIT_MS    9000u

static volatile bool s_slc_up = false;
static volatile bool s_hfp_connecting = false;
static volatile bool s_call_active = false;
static volatile bool s_call_setup_incoming = false;
static volatile bool s_call_setup_any = false;     /* incoming/outgoing/alerting */
static volatile bool s_call_wanted = false;        /* any call in progress or ringing */
static volatile bool s_voice_ready = false;        /* server voice channel is up */
static volatile int64_t s_ring_start_us = 0;
static SemaphoreHandle_t s_call_sem = NULL;        /* given on call start */

/* Recompute "is a call happening"; wake the session task on the rising edge. */
static void call_state_update(void)
{
    bool wanted = s_call_active || s_call_setup_any;
    if (wanted && !s_call_wanted) {
        s_ring_start_us = esp_timer_get_time();
        s_call_wanted = true;
        if (s_call_sem != NULL) {
            xSemaphoreGive(s_call_sem);
        }
    } else if (!wanted) {
        s_call_wanted = false;
    }
}

bool sco_bridge_call_wanted(void)
{
    return s_call_wanted;
}

bool sco_bridge_wait_call(uint32_t timeout_ms)
{
    if (s_call_sem == NULL) {
        return false;
    }
    TickType_t t = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(s_call_sem, t) == pdTRUE && s_call_wanted;
}

void sco_bridge_set_voice_ready(bool ready)
{
    s_voice_ready = ready;
}
static int64_t s_connect_start_us = 0;

static esp_bd_addr_t s_peer_addr;
static bool s_peer_valid = false;

static esp_timer_handle_t s_link_timer = NULL;
static esp_timer_handle_t s_answer_timer = NULL;

static void link_timer_cb(void *arg)
{
    (void)arg;
    if (s_slc_up || !s_peer_valid) {
        return;
    }
    if (s_hfp_connecting) {
        /* Watchdog: a stuck CONNECTING (BT stack hang) is force-cleared so
         * the next tick can retry. */
        int64_t now_us = esp_timer_get_time();
        if (now_us - s_connect_start_us > (int64_t)CONNECT_WATCHDOG_MS * 1000) {
            ESP_LOGW(TAG, "HFP connect stuck %d ms, forcing disconnect",
                     (int)CONNECT_WATCHDOG_MS);
            esp_hf_client_disconnect(s_peer_addr);
        }
        return;
    }
    ESP_LOGI(TAG, "HFP (re)connecting to bonded phone");
    if (esp_hf_client_connect(s_peer_addr) == ESP_OK) {
        s_hfp_connecting = true;
        s_connect_start_us = esp_timer_get_time();
    }
}

static void answer_timer_cb(void *arg)
{
    (void)arg;
    if (s_call_setup_incoming && !s_call_active) {
        /* Hold the answer until the server voice channel is up, so the first
         * words are not lost; give up waiting after ANSWER_MAX_WAIT_MS. */
        if (!s_voice_ready &&
            esp_timer_get_time() - s_ring_start_us < (int64_t)ANSWER_MAX_WAIT_MS * 1000) {
            esp_timer_start_once(s_answer_timer, 200 * 1000);
            return;
        }
        ESP_LOGI(TAG, "Auto-answering incoming call (voice %s)", s_voice_ready ? "ready" : "NOT ready");
        esp_hf_client_answer_call();
    }
}

/* Load the peer address written by the production firmware (namespace
 * hfp_mgr, key peer_addr) so the dev board reconnects to the already
 * bonded phone. */
static bool load_peer_from_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open("hfp_mgr", NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    size_t len = sizeof(s_peer_addr);
    esp_err_t err = nvs_get_blob(h, "peer_addr", s_peer_addr, &len);
    nvs_close(h);
    if (err == ESP_OK && len == sizeof(s_peer_addr)) {
        ESP_LOGI(TAG, "peer from NVS: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(s_peer_addr));
        return true;
    }
    return false;
}

/* Persist the currently connected peer so the next boot pages the phone
 * that actually talked to us (same key the production firmware uses). */
static void save_peer_to_nvs(const esp_bd_addr_t addr)
{
    nvs_handle_t h;
    if (nvs_open("hfp_mgr", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    if (nvs_set_blob(h, "peer_addr", addr, sizeof(esp_bd_addr_t)) == ESP_OK) {
        nvs_commit(h);
    }
    nvs_close(h);
}

/* ---------- GAP callback: accept pairing initiated by the phone ---------- */

static void gap_event_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "paired with %s (" ESP_BD_ADDR_STR ")",
                     (const char *)param->auth_cmpl.device_name,
                     ESP_BD_ADDR_HEX(param->auth_cmpl.bda));
        } else {
            ESP_LOGW(TAG, "pairing failed, status=%d", (int)param->auth_cmpl.stat);
        }
        break;
    case ESP_BT_GAP_PIN_REQ_EVT: {
        esp_bt_pin_code_t pin = {0};
        pin[0] = '0'; pin[1] = '0'; pin[2] = '0'; pin[3] = '0';
        esp_bt_gap_pin_reply(param->pin_req.bda, true,
                             param->pin_req.min_16_digit ? 16 : 4, pin);
        break;
    }
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "SSP confirm %" PRIu32 ", accepting", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "SSP passkey: %" PRIu32, param->key_notif.passkey);
        break;
    default:
        break;
    }
}

/* ---------- ring helpers ---------- */

static inline uint16_t ring_used(uint16_t wr, uint16_t rd)
{
    return (uint16_t)((wr - rd) & RING_MASK);
}

/* Copy `len` bytes out of a ring and advance *rd. Caller holds the mux and
 * has verified used >= len. */
static void ring_read(uint8_t *ring, uint8_t *dst, uint16_t len, uint16_t *rd)
{
    uint16_t to_end = (uint16_t)(RING_SIZE - *rd);
    if (len <= to_end) {
        memcpy(dst, ring + *rd, len);
    } else {
        memcpy(dst, ring + *rd, to_end);
        memcpy(dst + to_end, ring, len - to_end);
    }
    *rd = (uint16_t)((*rd + len) & RING_MASK);
}

/* Write into a ring with over-HWM trim; returns trimmed bytes. */
static uint16_t ring_write(uint8_t *ring, uint16_t *wr, uint16_t *rd,
                           portMUX_TYPE *mux, const uint8_t *data, size_t len,
                           uint16_t hwm)
{
    if (ring == NULL || data == NULL || len == 0 || len > RING_MASK) {
        return 0;
    }
    uint16_t trim = 0;
    portENTER_CRITICAL(mux);
    uint16_t used = ring_used(*wr, *rd);
    if (used > hwm) {
        trim = (uint16_t)(used - hwm);
        *rd = (uint16_t)((*rd + trim) & RING_MASK);
        used = hwm;
    }
    uint16_t free_space = (uint16_t)(RING_SIZE - 1u - used);
    size_t copy = (len > free_space) ? free_space : len;
    if (copy > 0) {
        uint16_t to_end = (uint16_t)(RING_SIZE - *wr);
        if (copy <= to_end) {
            memcpy(ring + *wr, data, copy);
        } else {
            memcpy(ring + *wr, data, to_end);
            memcpy(ring, data + to_end, copy - to_end);
        }
        *wr = (uint16_t)((*wr + copy) & RING_MASK);
    }
    portEXIT_CRITICAL(mux);
    return trim;
}

static unsigned bytes_per_ms(sco_codec_t codec)
{
    return (codec == SCO_CODEC_MSBC) ? 32u : 16u;   /* 16 kHz / 8 kHz × int16 */
}

/* ---------- public: uplink pull ---------- */

bool sco_bridge_mic_read16k(int16_t *out, size_t samples)
{
    if (!s_sco_active || s_mic_buf == NULL || out == NULL || samples == 0) {
        s_stats.mic_empty++;
        return false;
    }

    if (s_codec == SCO_CODEC_MSBC) {
        size_t need = samples * 2;
        bool ok = false;
        portENTER_CRITICAL(&s_mic_mux);
        if (ring_used(s_mic_wr, s_mic_rd) >= need) {
            ring_read(s_mic_buf, (uint8_t *)out, (uint16_t)need, &s_mic_rd);
            ok = true;
        }
        portEXIT_CRITICAL(&s_mic_mux);
        if (ok) {
            s_stats.mic_frames++;
        } else {
            s_stats.mic_empty++;
        }
        return ok;
    }

    /* CVSD: pull half as many 8 kHz samples and upsample outside the lock. */
    size_t need8 = samples / 2;
    static int16_t s_scratch8[SCRATCH_MAX];
    if (need8 == 0 || need8 > SCRATCH_MAX) {
        s_stats.mic_empty++;
        return false;
    }
    bool ok = false;
    portENTER_CRITICAL(&s_mic_mux);
    if (ring_used(s_mic_wr, s_mic_rd) >= need8 * 2) {
        ring_read(s_mic_buf, (uint8_t *)s_scratch8, (uint16_t)(need8 * 2), &s_mic_rd);
        ok = true;
    }
    portEXIT_CRITICAL(&s_mic_mux);
    if (!ok) {
        s_stats.mic_empty++;
        return false;
    }
    resample_upsample(s_scratch8, out, need8);
    s_stats.mic_frames++;
    return true;
}

/* ---------- public: downlink push ---------- */

void sco_bridge_spk_write16k(const int16_t *pcm, size_t samples)
{
    if (!s_sco_active || s_spk_buf == NULL || pcm == NULL || samples == 0) {
        return;
    }

    /* P0: delivery-arrival spectrum (esp_peer handoff is bursty; these three
     * buckets separate "server burst" from "peer-loop holes"). */
    {
        int64_t now_us = esp_timer_get_time();
        if (!s_last_write_first) {
            int64_t gap = now_us - s_last_write_us;
            if (gap >= 30000 && gap < 60000) s_stats.write_gap_30++;
            else if (gap >= 60000 && gap < 150000) s_stats.write_gap_60++;
            else if (gap >= 150000) s_stats.write_gap_150++;
        }
        s_last_write_first = false;
        s_last_write_us = now_us;
    }

    unsigned rate_bytes_per_ms = bytes_per_ms(s_codec);
    if (s_codec == SCO_CODEC_MSBC) {
        uint16_t trim = ring_write(s_spk_buf, &s_spk_wr, &s_spk_rd, &s_spk_mux,
                                   (const uint8_t *)pcm, samples * 2,
                                   (uint16_t)(rate_bytes_per_ms * SPK_HWM_MS));
        if (trim > 0) {
            s_stats.trims++;
            s_stats.trim_bytes += trim;
            int lvl = sco_bridge_spk_level_ms();
            if (lvl > s_stats.trim_max_level) {
                s_stats.trim_max_level = lvl;
            }
        }
        s_stats.spk_frames += samples / 320u;   /* ~20 ms frames */
        return;
    }

    /* CVSD: downsample (even sample count only). */
    static int16_t s_scratch8[SCRATCH_MAX];
    size_t even = samples & ~((size_t)1);
    if (even == 0 || even / 2 > SCRATCH_MAX) {
        return;
    }
    size_t out8 = resample_downsample(pcm, s_scratch8, even);
    if (out8 == 0) {
        return;
    }
    uint16_t trim = ring_write(s_spk_buf, &s_spk_wr, &s_spk_rd, &s_spk_mux,
                               (const uint8_t *)s_scratch8, out8 * 2,
                               (uint16_t)(rate_bytes_per_ms * SPK_HWM_MS));
    if (trim > 0) {
        s_stats.trims++;
        s_stats.trim_bytes += trim;
        int lvl = sco_bridge_spk_level_ms();
        if (lvl > s_stats.trim_max_level) {
            s_stats.trim_max_level = lvl;
        }
    }
    s_stats.spk_frames += samples / 320u;
}

bool sco_bridge_sco_active(void)
{
    return s_sco_active;
}

int sco_bridge_spk_level_ms(void)
{
    if (!s_sco_active || s_spk_buf == NULL) {
        return -1;
    }
    portENTER_CRITICAL(&s_spk_mux);
    uint16_t used = ring_used(s_spk_wr, s_spk_rd);
    portEXIT_CRITICAL(&s_spk_mux);
    return (int)(used / bytes_per_ms(s_codec));
}

void sco_bridge_get_stats(sco_bridge_stats_t *out)
{
    if (out != NULL) {
        portENTER_CRITICAL(&s_spk_mux);
        *out = s_stats;
        portEXIT_CRITICAL(&s_spk_mux);
    }
}

/* ---------- HFP data callbacks (BTC task context) ---------- */

static void incoming_data_cb(const uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0) {
        return;
    }
    s_last_in_us = esp_timer_get_time();
    if (s_sco_active) {
        esp_hf_client_outgoing_data_ready();
    }
    /* Buffer is owned by the stack and only valid during the call: copy. */
    ring_write(s_mic_buf, &s_mic_wr, &s_mic_rd, &s_mic_mux, buf, len,
               (uint16_t)(bytes_per_ms(s_codec) * MIC_HWM_MS));
}

/* ---------- P1b: drip-drop (overflow guard, NOT a smoothdown tool) ----------
 *
 * W-log verdict after the first burn: at every robot-turn onset the ring
 * surges to HWM (late frames + PLC ghost frames arrive as one burst), and
 * a low drip threshold (err > 24 ms) then shred real speech at up to 16
 * samples/cb for seconds — that was the audible "muffled first seconds".
 *
 * New contract: drain authority is reserved for the overflow danger band
 * only (level approaching HWM, where the alternative is a hard trim). Bumps
 * below the guard line are *latency*, not loss: ring naturally drains back
 * to target during the silent gaps between turns. This trades ≤100 ms of
 * transient latency for zero mid-speech shredding.
 */
#define DRIP_MIN_ERR_MS   96    /* only drain in the HWM danger band      */
#define DRIP_MAX_SKIP     12    /* ~0.75 ms of 16 kHz samples per cb      */
#define DRIP_FADE         8     /* samples (~0.5 ms) of edge melt         */
static int16_t s_spk_last_out;  /* previous emitted sample (edge anchor) */

#define SPK_ONSET_MS          90    /* rebuffer depth after a dry ring     */
#define SPK_ONSET_TIMEOUT_MS  150   /* max hold once data starts arriving  */
static bool    s_spk_gate;          /* true = holding playback to rebuffer */
static int64_t s_spk_gate_since_us; /* when data first appeared in the gate */

static uint32_t outgoing_data_cb(uint8_t *buf, uint32_t len)
{
    s_stats.sco_cb_calls++;
    if (len < s_stats.cb_len_min || s_stats.cb_len_min == 0) {
        s_stats.cb_len_min = len;
    }
    if (len > s_stats.cb_len_max) {
        s_stats.cb_len_max = len;
    }

    int64_t now_us = esp_timer_get_time();
    if (s_last_cb_us) {
        int64_t gap = now_us - s_last_cb_us;
        if (gap < 0) gap = 0;
        if (gap > 10000) s_stats.cb_gap_10ms++;
        if ((uint32_t)gap > s_stats.cb_gap_max_us) s_stats.cb_gap_max_us = (uint32_t)gap;
    }
    s_last_cb_us = now_us;

    uint32_t copy = 0;
    portENTER_CRITICAL(&s_spk_mux);
    uint16_t avail = ring_used(s_spk_wr, s_spk_rd);

    /* Onset/rebuffer gate: after the ring ran dry (turn start or mid-speech
     * underrun) hold playback until ONSET_MS is queued, else the first frames
     * play straight into bursty delivery and stutter. Timeout bounds latency. */
    if (s_spk_gate) {
        if (avail == 0) {
            s_spk_gate_since_us = 0;
        } else {
            if (s_spk_gate_since_us == 0) {
                s_spk_gate_since_us = now_us;
            }
            if (avail >= bytes_per_ms(s_codec) * SPK_ONSET_MS ||
                now_us - s_spk_gate_since_us >= (int64_t)SPK_ONSET_TIMEOUT_MS * 1000) {
                s_spk_gate = false;
            }
        }
        if (s_spk_gate) {
            avail = 0;   /* emit silence without consuming the ring */
        }
    }

    if (s_codec == SCO_CODEC_MSBC && len >= 4 && (len & 1) == 0 && avail > 0) {
        /* Level-error -> consume-ratio (aim back at SPK_TARGET_MS). */
        int32_t level_ms = (int32_t)(avail / bytes_per_ms(s_codec));
        int32_t err_ms = level_ms - (int32_t)SPK_TARGET_MS;
        /* Deadband: a fractional read offset is a time-varying linear-
         * interpolation low-pass (strongest at frac=0.5). Capture showed the
         * output losing ~60 % of the decoded stream's HF energy at turn
         * onsets, where the level error pushes the ratio off 1.0 and frac
         * sweeps -> buzzy "kaka". Inside the deadband read at exactly 1.0
         * with zero offset (bit-exact); only steer when truly off target. */
        int32_t eff_err = 0;
        if (err_ms > (int32_t)SPK_DEADBAND_MS)       eff_err = err_ms - (int32_t)SPK_DEADBAND_MS;
        else if (err_ms < -(int32_t)SPK_DEADBAND_MS) eff_err = err_ms + (int32_t)SPK_DEADBAND_MS;
        int32_t adj = (int32_t)(((int64_t)eff_err * (int32_t)SPK_MAX_DELTA) / SPK_SPAN_MS);
        if (adj > (int32_t)SPK_MAX_DELTA) adj = (int32_t)SPK_MAX_DELTA;
        if (adj < -(int32_t)SPK_MAX_DELTA) adj = -(int32_t)SPK_MAX_DELTA;
        uint32_t ratio = SPK_RATIO_ONE + (uint32_t)adj;

        s_stats.level_sum_ms += level_ms;
        s_stats.level_n++;

        /* Interpolating fractional read: emits len/2 samples from the ring
         * while advancing the read pointer at `ratio`. */
        size_t out_samp = len / 2;
        uint16_t rd = s_spk_rd;
        uint16_t used_samp = avail / 2;
        uint32_t off = s_spk_off_q16;
        if (adj == 0) {
            off = 0;   /* unity read: no interpolation filtering */
        }

        /* Level trim: only ever delete audio that is silent. Sample-wise drip
         * drops inside speech (the old behaviour; 100+ events per 10 s once
         * the level hovers at the danger line) are audible as grain, so the
         * excess above target is shed in one go while the upcoming samples
         * are near-silent (pauses between phrases are plentiful). */
        size_t drip = 0;
        if (err_ms > (int32_t)SPK_TRIM_MARGIN_MS && used_samp > out_samp + 8) {
            size_t want = (size_t)(err_ms - (int32_t)SPK_TRIM_MARGIN_MS) * 16u;   /* samples */
            if (want > SPK_TRIM_MAX_SAMP) want = SPK_TRIM_MAX_SAMP;
            if (want > used_samp - out_samp - 8) want = used_samp - out_samp - 8;
            bool quiet = true;
            for (size_t k = 0; k < out_samp + want && quiet; k += 2) {
                int32_t v = spk_fetch_sample(rd, (int32_t)k);
                if (v > SPK_QUIET_ABS || v < -SPK_QUIET_ABS) quiet = false;
            }
            if (quiet && want > 0) {
                drip = want;
                rd = (uint16_t)(rd + drip * 2);
                used_samp -= (uint16_t)drip;
                s_stats.drop_events++;
                s_stats.drop_bytes += (uint32_t)(drip * 2);
            }
        }

        int16_t *outbuf = (int16_t *)buf;
        size_t i;
        for (i = 0; i < out_samp; i++) {
            while (off >= SPK_RATIO_ONE) {
                off -= SPK_RATIO_ONE;
                rd = (uint16_t)(rd + 2);
                used_samp--;
            }
            if (used_samp < 2) {
                break;
            }
            int32_t a = spk_fetch_sample(rd, 0);
            int32_t b = spk_fetch_sample(rd, 1);
            int32_t frac = (int32_t)(off >> 2);   /* 0..16383 */
            int32_t v = a + (((b - a) * frac) >> 14);
            if (drip > 0 && i < DRIP_FADE) {
                /* Melt the skip edge: fade from the last emitted sample
                 * into the post-skip content over ~0.5 ms. */
                int32_t w_v = i + 1;
                v = (s_spk_last_out * (DRIP_FADE - i) + v * w_v) / DRIP_FADE;
            }
            outbuf[i] = (int16_t)v;
            s_spk_last_out = (int16_t)v;
            off += ratio;
        }
        s_spk_off_q16 = off;
        s_spk_rd = rd;
        copy = i * 2;
        s_stats.consumed_bytes += copy;
    } else if (avail > 0 && len > 0) {
        uint32_t n = (avail < len) ? avail : len;
        ring_read(s_spk_buf, buf, (uint16_t)n, &s_spk_rd);
        copy = n;
        s_stats.consumed_bytes += copy;
    }
    portEXIT_CRITICAL(&s_spk_mux);

    if (s_codec == SCO_CODEC_MSBC) {
        if (copy < len) memset(buf + copy, 0, len - copy);
    }
    if (copy < len) {
        if (!s_spk_gate) {
            s_spk_gate = true;   /* ran dry: rebuffer before resuming */
            s_spk_gate_since_us = 0;
        }
        s_stats.pad_frames++;
        memset(buf + copy, 0, len - copy);
    }
    return len;
}

/* ---------- SCO open/close ---------- */

static void sco_open(sco_codec_t codec)
{
    if (s_mic_buf == NULL) {
        s_mic_buf = heap_caps_malloc(RING_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (s_spk_buf == NULL) {
        s_spk_buf = heap_caps_malloc(RING_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (s_mic_buf == NULL || s_spk_buf == NULL) {
        ESP_LOGE(TAG, "no PSRAM for SCO rings, audio dropped");
        return;
    }
    s_codec = codec;
    portENTER_CRITICAL(&s_mic_mux);
    s_mic_wr = s_mic_rd = 0;
    portEXIT_CRITICAL(&s_mic_mux);
    portENTER_CRITICAL(&s_spk_mux);
    s_spk_wr = s_spk_rd = 0;
    s_spk_off_q16 = 0;
    s_spk_last_out = 0;
    s_spk_gate = false;   /* open prefill already provides the cushion */
    s_spk_gate_since_us = 0;
    portEXIT_CRITICAL(&s_spk_mux);
    codec_resampler_reset();
    s_last_cb_us = 0;
    s_last_write_first = true;

    /* Prefill SPK_PREFILL_MS of silence so the pump has a jitter cushion
     * before the first RTP frame lands (see SPK_PREFILL_MS rationale). */
    static int16_t silence[320];
    memset(silence, 0, sizeof(silence));
    {
        const uint16_t chunk = (codec == SCO_CODEC_MSBC)
            ? (uint16_t)sizeof(silence) : (uint16_t)(sizeof(silence) / 2);
        const unsigned total =
            (unsigned)(bytes_per_ms(codec) * SPK_PREFILL_MS + chunk - 1) / chunk;
        for (unsigned i = 0; i < total; i++) {
            ring_write(s_spk_buf, &s_spk_wr, &s_spk_rd, &s_spk_mux,
                       (const uint8_t *)silence, chunk,
                       (uint16_t)(bytes_per_ms(codec) * SPK_HWM_MS));
        }
    }

    memset(&s_stats, 0, sizeof(s_stats));
    s_sco_active = true;
    if (s_pump_timer != NULL && !s_pump_running) {
        s_pump_running = true;
        esp_timer_start_periodic(s_pump_timer, 7500);   /* one mSBC frame */
    }
    ESP_LOGI(TAG, "SCO open codec=%s (%u Hz)",
             codec == SCO_CODEC_MSBC ? "mSBC" : "CVSD",
             codec == SCO_CODEC_MSBC ? 16000u : 8000u);
}

static void sco_close(void)
{
    if (!s_sco_active) {
        return;
    }
    s_sco_active = false;
    s_pump_running = false;
    if (s_pump_timer != NULL) {
        esp_timer_stop(s_pump_timer);
    }
    ESP_LOGI(TAG, "SCO close: mic=%lu/%lu spk=%lu pad=%lu trim=%lu cb=%lu",
             (unsigned long)s_stats.mic_frames, (unsigned long)s_stats.mic_empty,
             (unsigned long)s_stats.spk_frames, (unsigned long)s_stats.pad_frames,
             (unsigned long)s_stats.trims, (unsigned long)s_stats.sco_cb_calls);
}

/* ---------- HFP event callback (BTC task context) ---------- */

static void hfp_event_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param)
{
    switch (event) {
    case ESP_HF_CLIENT_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "HFP conn state=%d peer=" ESP_BD_ADDR_STR,
                 (int)param->conn_stat.state,
                 ESP_BD_ADDR_HEX(param->conn_stat.remote_bda));
        switch (param->conn_stat.state) {
        case ESP_HF_CLIENT_CONNECTION_STATE_CONNECTING:
            s_hfp_connecting = true;
            s_connect_start_us = esp_timer_get_time();
            break;
        case ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED:
            s_hfp_connecting = false;
            s_slc_up = true;
            s_call_active = false;
            s_call_setup_incoming = false;
            s_call_setup_any = false;
            call_state_update();
            /* Adopt this phone as the reconnect target from now on (the
             * NVS record may be stale, e.g. the phone re-paired). */
            memcpy(s_peer_addr, param->conn_stat.remote_bda, sizeof(esp_bd_addr_t));
            s_peer_valid = true;
            save_peer_to_nvs(s_peer_addr);
            ESP_LOGI(TAG, "phone connected (SLC up), peer adopted");
            break;
        case ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTED:
            s_hfp_connecting = false;
            s_slc_up = false;
            s_call_active = false;
            s_call_setup_incoming = false;
            s_call_setup_any = false;
            call_state_update();
            /* Be visible for the phone to connect on its own as well. */
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            ESP_LOGI(TAG, "phone disconnected, retrying in %u ms", LINK_CHECK_PERIOD_MS);
            break;
        default:
            break;
        }
        break;
    case ESP_HF_CLIENT_AUDIO_STATE_EVT:
        if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
            sco_open(SCO_CODEC_MSBC);
        } else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED) {
            sco_open(SCO_CODEC_CVSD);
        } else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED) {
            sco_close();
        }
        break;
    case ESP_HF_CLIENT_CIND_CALL_EVT:
        ESP_LOGI(TAG, "call indicator: %d", (int)param->call.status);
        s_call_active = (param->call.status != ESP_HF_CALL_STATUS_NO_CALLS);
        call_state_update();
        break;
    case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
        ESP_LOGI(TAG, "call setup: %d", (int)param->call_setup.status);
        s_call_setup_incoming =
            (param->call_setup.status == ESP_HF_CALL_SETUP_STATUS_INCOMING);
        s_call_setup_any = (param->call_setup.status != ESP_HF_CALL_SETUP_STATUS_IDLE);
        call_state_update();
        if (s_auto_answer && s_call_setup_incoming && s_answer_timer != NULL) {
            esp_timer_start_once(s_answer_timer, ANSWER_DELAY_MS * 1000);
        }
        break;
    case ESP_HF_CLIENT_CLIP_EVT:
        ESP_LOGI(TAG, "incoming call from: %s", param->clip.number);
        break;
    default:
        break;   /* other indicators: not on the bring-up path */
    }
}

/* ---------- init ---------- */

void sco_bridge_init(void)
{
    s_call_sem = xSemaphoreCreateBinary();
    /* Register order mirrors production hfp_client_init(): the event callback
     * goes in before esp_hf_client_init(), the data callbacks after. */
    ESP_ERROR_CHECK(esp_hf_client_register_callback(hfp_event_cb));
    ESP_ERROR_CHECK(esp_hf_client_init());
    ESP_ERROR_CHECK(esp_hf_client_register_data_callback(incoming_data_cb,
                                                         outgoing_data_cb));
    /* GAP events carry the pairing handshake; auto-accept SSP so the
     * phone can bond with us unattended. */
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_event_cb));

    /* Be visible/connectable: the phone can then find us for pairing and
     * auto-connect to the bond on its own. */
    const system_config_t *cfg = config_get();
    const char *dev_name = cfg->bt_device_name[0] ? cfg->bt_device_name
                                                  : BT_DEVICE_NAME_DEFAULT;
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(dev_name));
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    ESP_LOGI(TAG, "BT device \"%s\", connectable+discoverable", dev_name);

    const esp_timer_create_args_t timer_args = {
        .callback = pump_cb,
        .name = "sco_pump",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_pump_timer));

    /* Peer selection: a runtime/config override ("aa:bb:cc:dd:ee:ff",
     * BLE namespace "bt", key "peer"; menuconfig default CONFIG_GW_BT_PEER_ADDR)
     * wins, otherwise fall back to the address the production firmware bonded
     * and stored in NVS. Empty string matches strlen() < 17. */
    const char *peer = cfg->bt_peer_addr;
    if (strlen(peer) >= 17) {
        if (sscanf(peer, "%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx",
                   &s_peer_addr[0], &s_peer_addr[1], &s_peer_addr[2],
                   &s_peer_addr[3], &s_peer_addr[4], &s_peer_addr[5]) == 6) {
            s_peer_valid = true;
            ESP_LOGI(TAG, "peer from config: %s", peer);
        } else {
            ESP_LOGW(TAG, "configured peer addr malformed, trying NVS");
        }
    }
    if (!s_peer_valid) {
        s_peer_valid = load_peer_from_nvs();
    }
    if (!s_peer_valid) {
        ESP_LOGW(TAG, "no bonded peer known; waiting for the phone to connect");
    }

    /* Periodic link check: reconnects to the bonded phone (device-side
     * delay gives the phone's own auto-connect the first shot), with a
     * watchdog for stuck CONNECTING states. */
    const esp_timer_create_args_t link_args = {
        .callback = link_timer_cb,
        .name = "hfp_link",
    };
    ESP_ERROR_CHECK(esp_timer_create(&link_args, &s_link_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_link_timer,
                                             LINK_CHECK_PERIOD_MS * 1000));

    const esp_timer_create_args_t answer_args = {
        .callback = answer_timer_cb,
        .name = "hfp_answer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&answer_args, &s_answer_timer));
    s_auto_answer = cfg->bt_auto_answer;
}
