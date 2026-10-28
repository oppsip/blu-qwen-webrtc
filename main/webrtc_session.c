/*
 * WebRTC session: esp_peer PeerConnection + WHIP signaling + Opus media +
 * 'txt' DataChannel control events (with chunk reassembly).
 *
 * Device-side contract (verified in P1, see shared/gateway/webrtc-browser.mjs):
 *  - offer: one sendrecv audio m-line + one application m-line
 *  - server creates the 'txt' channel; client adds a placeholder channel
 *  - no session.hello from the client; server emits session.created/updated
 */

#include "webrtc_session.h"
#include "whip_client.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "cJSON.h"

#include "esp_peer.h"
#include "esp_peer_default.h"

#include "sco_bridge.h"
#include "psram_task.h"

static const char *TAG = "webrtc";

#define SDP_MAX      8192
#define CHUNK_TYPE   "qwaudio.transport.chunk"
#define CHUNK_SIZE   (8 * 1024)
#define MESSAGE_MAX  (512 * 1024)

typedef struct {
    webrtc_session_t *session;
    esp_peer_handle_t peer;
    QueueHandle_t events;      /* JSON text frames reassembled, pending parse */
    char *sdp;                 /* local SDP from on_msg */
    volatile bool sdp_ready;
    volatile bool connected;
    volatile bool failed;
    volatile bool txt_open;

    /* chunk reassembly state */
    char    chunk_id[80];
    uint8_t chunk_total;
    uint8_t chunk_index;
    char   *chunk_buf;         /* MESSAGE_MAX + 1 */
    uint32_t chunk_used;

    int16_t pcm[CODEC_FRAME_SAMPLES];   /* decode scratch */
} session_ctx_t;

static session_ctx_t s_ctx;

/* TEMP-P2-DIAG: per-window downlink reception diagnostics */
static volatile uint32_t s_diag_rxd;        /* on_audio_data callbacks        */
static volatile uint32_t s_diag_rxd_bytes;  /* encoded bytes received         */
static volatile uint32_t s_diag_dec_fail;   /* opus decode failures           */
static volatile int      s_diag_ring_min = 1 << 20;
static volatile int      s_diag_ring_max = -1;
static volatile uint32_t s_diag_gap_events;     /* pts deltas != 20 (ms) */
static volatile uint32_t s_diag_gap_missing;    /* estimated packets missing */
static volatile int32_t  s_diag_pts_prev = -1;  /* last inbound pts */
static volatile int32_t  s_diag_pts_last2 = -1; /* pts before that */
static volatile int      s_diag_pts_zero;       /* inbound frames with pts==0 */

/*
 * App-level packet-loss concealment (PLC): the espressif decoder lib exposes
 * no opus PLC, and a lost 20 ms frame otherwise becomes a hard zero-fill by
 * the SCO pump (~25% of out frames were silence mid-call). Conceal by
 * repeating the previous decoded frame with exponential fade + decay.
 */
#define PLC_MAX_MISSING 3                      /* lighter ghost fill: W-log   */
                                               /* showed 120 ms of synthesized
                                                * frames plus the late real
                                                * frames overfilled the ring at
                                                * every turn onset            */
#define PLC_FADE        128                    /* 8 ms cross-fade window    */
static int16_t s_plc_last[CODEC_FRAME_SAMPLES];   /* previous decoded frame */
static int16_t s_plc_tail[PLC_FADE];              /* its end, for blending  */
static bool    s_plc_last_used = false;
static int32_t s_plc_active_gain;   /* last ghost gain (0 = silence fill)  */
static volatile uint32_t s_diag_conceal;        /* concealed frames synthesized */

/* ---------- esp_peer callbacks (peer task context) ---------- */

/* The server sends Opus hybrid fullband (TOC cfg 13/15). The on-chip decoder
 * runs at 16 kHz and renders hybrid frames grainy (browser decoding the same
 * stream at 48 kHz is clean). Ask the encoder for wideband-or-less via the
 * standard Opus fmtp so it switches to pure SILK. */
#define OPUS_FMTP_ADD "maxplaybackrate=16000;sprop-maxcapturerate=16000;stereo=0;sprop-stereo=0;useinbandfec=0"
static void sdp_limit_opus_rate(char *sdp, size_t cap)
{
    char *rtp = strstr(sdp, "a=rtpmap:111 ");
    if (rtp == NULL) {
        return;
    }
    char *fmtp = strstr(sdp, "a=fmtp:111 ");
    size_t len = strlen(sdp);
    if (fmtp != NULL) {
        if (strstr(fmtp, "maxplaybackrate") != NULL && strstr(fmtp, "maxplaybackrate") < strstr(fmtp, "\r\n")) {
            return;
        }
        char *eol = strstr(fmtp, "\r\n");
        if (eol == NULL || len + sizeof(OPUS_FMTP_ADD) + 2 >= cap) {
            return;
        }
        size_t add = sizeof(OPUS_FMTP_ADD);   /* ';' + string */
        memmove(eol + add, eol, strlen(eol) + 1);
        eol[0] = ';';
        memcpy(eol + 1, OPUS_FMTP_ADD, add - 1);
    } else {
        char *eol = strstr(rtp, "\r\n");
        const char *pre = "a=fmtp:111 ";
        size_t add = 2 + strlen(pre) + strlen(OPUS_FMTP_ADD);
        if (eol == NULL || len + add >= cap) {
            return;
        }
        eol += 2;
        memmove(eol + add, eol, strlen(eol) + 1);
        memcpy(eol, pre, strlen(pre));
        memcpy(eol + strlen(pre), OPUS_FMTP_ADD, strlen(OPUS_FMTP_ADD));
        memcpy(eol + strlen(pre) + strlen(OPUS_FMTP_ADD), "\r\n", 2);
    }
}

static int on_state(esp_peer_state_t state, void *ctx)
{
    session_ctx_t *c = (session_ctx_t *)ctx;
    ESP_LOGI(TAG, "peer state=%d", (int)state);
    if (state == ESP_PEER_STATE_CONNECTED || state == ESP_PEER_STATE_DATA_CHANNEL_CONNECTED ||
        state == ESP_PEER_STATE_DATA_CHANNEL_OPENED) {
        c->connected = true;
    }
    if (state == ESP_PEER_STATE_CONNECT_FAILED || state == ESP_PEER_STATE_DISCONNECTED) {
        c->failed = true;
    }
    return 0;
}

static int on_msg(esp_peer_msg_t *info, void *ctx)
{
    session_ctx_t *c = (session_ctx_t *)ctx;
    if (info->type == ESP_PEER_MSG_TYPE_SDP && info->data != NULL && info->size > 0 &&
        (size_t)info->size < SDP_MAX) {
        memcpy(c->sdp, info->data, (size_t)info->size);
        c->sdp[info->size] = '\0';
        c->sdp_ready = true;
        ESP_LOGI(TAG, "local SDP ready: %d bytes", info->size);
    }
    return 0;
}

static int on_audio_data(esp_peer_audio_frame_t *frame, void *ctx)
{
    session_ctx_t *c = (session_ctx_t *)ctx;
    webrtc_session_t *s = c->session;
    s_diag_rxd++;
    s_diag_rxd_bytes += frame->size;
    if (frame->pts == 0) {
        s_diag_pts_zero++;
    } else {
        int32_t d = (int32_t)(frame->pts - s_diag_pts_prev);   /* signed wraparound-safe */
        if (s_diag_pts_prev >= 0 && d != 20 && d > 0) {
            int missing = d / 20 - 1;
            s_diag_gap_events++;
            s_diag_gap_missing += (uint32_t)missing;
            /* Conceal the hole before the (late) real audio arrives. */
            if (s_plc_last_used) {
                if (missing > PLC_MAX_MISSING) {
                    missing = PLC_MAX_MISSING;
                }
                /* Turn-boundary vs mid-speech hole: at turn boundaries the
                 * server elides silence from its pts timeline, so the "hole"
                 * is legal silence — filling it with decayed repeats of the
                 * previous utterance tail echoes old speech over the new
                 * response onset (the residual muffled-start artifact).
                 * Quiet pre-hole tail -> fill silence, not ghosts. */
                int64_t tail_energy = 0;
                for (int i = 0; i < PLC_FADE; i++) {
                    tail_energy += (int64_t)s_plc_tail[i] * s_plc_tail[i];
                }
                bool tail_quiet = (tail_energy < (int64_t)PLC_FADE * 200 * 200);  /* RMS < 200 */
                int32_t gain = tail_quiet ? 0 : 128;   /* Q7: 100% -> decays */
                for (int m = 0; m < missing; m++) {
                    int16_t out[CODEC_FRAME_SAMPLES];
                    if (gain == 0) {
                        memset(out, 0, sizeof(out));
                        s_plc_active_gain = 0;   /* hard, silence is silence */
                    } else {
                        for (int i = 0; i < CODEC_FRAME_SAMPLES; i++) {
                            int32_t v = s_plc_last[i] * gain / 128;
                            if (v > 32676) v = 32676;
                            if (v < -32676) v = -32676;
                            out[i] = (int16_t)v;
                        }
                        if (m == 0) {
                            /* P1: cross-fade the first 8 ms from the previous
                             * frame end into the repeated content — removes the
                             * waveform step at the loss edge (the harshest
                             * artifact of a hard repeat). */
                            for (int i = 0; i < PLC_FADE; i++) {
                                int32_t w_head = out[i] * (i + 1);
                                int32_t w_tail = s_plc_tail[i] * (PLC_FADE - 1 - i);
                                out[i] = (int16_t)((w_head + w_tail) / PLC_FADE);
                            }
                        }
                        /* Source itself decays so the repeated pattern fades in
                         * level and shape; below ~20% it plateaus to near-sil. */
                        for (int i = 0; i < CODEC_FRAME_SAMPLES; i++) {
                            s_plc_last[i] = out[i];
                        }
                        int32_t nxt = gain * 300 / 400;   /* -25 % per frame */
                        gain = nxt < 24 ? 24 : nxt;
                        s_plc_active_gain = gain;  /* remember for the re-entry fade */
                    }
                    sco_bridge_spk_write16k(out, CODEC_FRAME_SAMPLES);
                    s_diag_conceal++;
                }
            }
        }
        s_diag_pts_last2 = s_diag_pts_prev;   /* TEMP-P2-DIAG */
        s_diag_pts_prev = (int32_t)frame->pts;
    }

    /* Opus packet -> PCM16. Only whole frames are pushed. */
    int samples = media_codec_decode(&s->codec, frame->data, (size_t)frame->size,
                                     c->pcm, CODEC_FRAME_SAMPLES);
    if (samples > 0) {
        s->downlink_frames++;
        if (s_plc_active_gain > 0 && samples == CODEC_FRAME_SAMPLES) {
            /* Re-entry after ghost fill: fade the real frame's first 8 ms
             * from the ghost's decaying level up to full — no a-level step
             * at the ghost/speech boundary. */
            int32_t g0 = s_plc_active_gain;   /* Q7 */
            for (int i = 0; i < PLC_FADE; i++) {
                int32_t g = (g0 * (PLC_FADE - 1 - i) + 128 * (i + 1)) / PLC_FADE;
                int32_t v = c->pcm[i] * g / 128;
                c->pcm[i] = (int16_t)v;
            }
            s_plc_active_gain = 0;
        }
        /* Feed the phone (SCO out ring); no-op outside a call. */
        sco_bridge_spk_write16k(c->pcm, (size_t)samples);
        int lvl = sco_bridge_spk_level_ms();
        if (lvl > s_diag_ring_max) s_diag_ring_max = lvl;
        if (lvl < s_diag_ring_min) s_diag_ring_min = lvl;
        /* Keep the latest frame as PLC source; its end as the cross-fade anchor. */
        if (samples == CODEC_FRAME_SAMPLES) {
            memcpy(s_plc_last, c->pcm, sizeof(s_plc_last));
            memcpy(s_plc_tail, c->pcm + CODEC_FRAME_SAMPLES - PLC_FADE, sizeof(s_plc_tail));
            s_plc_last_used = true;
        }
    } else {
        s_diag_dec_fail++;
    }
    return 0;
}

static int on_channel_open(esp_peer_data_channel_info_t *ch, void *ctx)
{
    session_ctx_t *c = (session_ctx_t *)ctx;
    if (ch->label && strcmp(ch->label, "txt") == 0) {
        c->txt_open = true;
        ESP_LOGI(TAG, "txt control channel open");
    }
    return 0;
}

/* Reassemble 'qwaudio.transport.chunk' frames; deliver complete JSON to queue. */
static void handle_txt_frame(session_ctx_t *c, const char *text, size_t len)
{
    webrtc_session_t *s = c->session;
    if (len >= MESSAGE_MAX) {
        ESP_LOGW(TAG, "txt frame too large (%u)", (unsigned)len);
        return;
    }
    /* Fast path: not a chunk header */
    if (strncmp(text, "{\"type\":\"" CHUNK_TYPE, 9 + sizeof(CHUNK_TYPE) - 1) != 0) {
        char *copy = malloc(len + 1);
        if (copy == NULL) {
            return;
        }
        memcpy(copy, text, len);
        copy[len] = '\0';
        if (xQueueSend(c->events, &copy, 0) != pdTRUE) {
            free(copy);
            ESP_LOGW(TAG, "event queue full");
        }
        s->control_rx_events++;
        return;
    }

    s->control_rx_chunks++;
    cJSON *root = cJSON_ParseWithLength(text, len);
    if (root == NULL) {
        goto chunk_reset;
    }
    cJSON *id = cJSON_GetObjectItem(root, "id");
    cJSON *index = cJSON_GetObjectItem(root, "index");
    cJSON *total = cJSON_GetObjectItem(root, "total");
    cJSON *data = cJSON_GetObjectItem(root, "data");
    if (!cJSON_IsString(id) || !cJSON_IsNumber(index) || !cJSON_IsNumber(total) ||
        !cJSON_IsString(data)) {
        cJSON_Delete(root);
        goto chunk_reset;
    }
    {
        const char *cid = id->valuestring;
        int idx = index->valueint;
        int tot = total->valueint;
        const char *part = data->valuestring;
        if (tot < 2 || tot > MESSAGE_MAX / CHUNK_SIZE || idx < 0 || idx >= tot) {
            cJSON_Delete(root);
            goto chunk_reset;
        }
        if (c->chunk_index == 0) {
            snprintf(c->chunk_id, sizeof(c->chunk_id), "%s", cid);
            c->chunk_total = (uint8_t)tot;
            c->chunk_used = 0;
        }
        if (strcmp(c->chunk_id, cid) != 0 || c->chunk_total != (uint8_t)tot ||
            c->chunk_index != (uint8_t)idx) {
            cJSON_Delete(root);
            goto chunk_reset;
        }
        size_t plen = strlen(part);
        if (c->chunk_used + plen >= MESSAGE_MAX) {
            cJSON_Delete(root);
            goto chunk_reset;
        }
        memcpy(c->chunk_buf + c->chunk_used, part, plen);
        c->chunk_used += plen;
        c->chunk_index++;
        if ((int)c->chunk_index == tot) {
            c->chunk_buf[c->chunk_used] = '\0';
            char *copy = malloc(c->chunk_used + 1);
            if (copy != NULL) {
                memcpy(copy, c->chunk_buf, c->chunk_used + 1);
                if (xQueueSend(c->events, &copy, 0) != pdTRUE) {
                    free(copy);
                } else {
                    s->control_rx_events++;
                }
            }
            c->chunk_index = 0;
            c->chunk_used = 0;
        }
        cJSON_Delete(root);
        return;
    }
chunk_reset:
    c->chunk_index = 0;
    c->chunk_used = 0;
    ESP_LOGW(TAG, "bad chunk frame, reassembly reset");
}

static int on_data(esp_peer_data_frame_t *frame, void *ctx)
{
    session_ctx_t *c = (session_ctx_t *)ctx;
    if (frame->type == ESP_PEER_DATA_CHANNEL_STRING && frame->data != NULL && frame->size > 0) {
        handle_txt_frame(c, (const char *)frame->data, (size_t)frame->size);
    }
    return 0;
}

/* ---------- peer main loop task ---------- */

static void peer_task(void *arg)
{
    session_ctx_t *c = (session_ctx_t *)arg;
    while (!c->failed && !(c->session->state == WEBRTC_SESSION_CLOSED)) {
        esp_peer_main_loop(c->peer);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "peer loop exit");
    vTaskDelete(NULL);
}

/* ---------- answer SDP preprocessing ----------
 *
 * The gateway sits behind a 1:1 NAT, so its answer advertises host candidates
 * with LAN addresses. Rewrite them to the public address and prune the rest:
 *  - IPv4 UDP host candidates are rewritten to GW_RTC_PUBLIC_IP and deduped
 *    by port (esp_peer's ICE agent caps remote candidates at 10; the raw
 *    answer carries 16+)
 *  - TCP and IPv6 candidates are dropped
 *  - a=mid is moved right after the m= line: node-webrtc emits candidates
 *    before a=mid and libpeer's extractor then reports mid=255
 */

#define SDP_MAX_SECTIONS 8
#define SDP_MAX_CANDS    8

/* Start and length of field n (0-based, space separated) of a line. */
static const char *sdp_field(const char *s, int n, size_t *len)
{
    const char *p = s;
    for (int i = 0; i < n; i++) {
        p = strchr(p, ' ');
        if (p == NULL) {
            return NULL;
        }
        p++;
    }
    const char *e = strchr(p, ' ');
    *len = (e != NULL) ? (size_t)(e - p) : strlen(p);
    return p;
}

/* Log m=/a=mid/a=candidate lines for bring-up diagnostics. */
static void sdp_log_key_lines(const char *tag, const char *sdp, size_t len)
{
    int logged = 0;
    size_t pos = 0;
    while (pos < len && logged < 24) {
        size_t eol = pos;
        while (eol < len && sdp[eol] != '\n') {
            eol++;
        }
        size_t ll = eol - pos;
        if (ll > 0 && sdp[pos + ll - 1] == '\r') {
            ll--;
        }
        if ((ll >= 2 && strncmp(&sdp[pos], "m=", 2) == 0) ||
            (ll >= 5 && strncmp(&sdp[pos], "a=mid", 5) == 0) ||
            (ll >= 11 && strncmp(&sdp[pos], "a=candidate", 11) == 0)) {
            ESP_LOGI(tag, "  %.*s", (int)ll, &sdp[pos]);
            logged++;
        }
        pos = eol + 1;
    }
}

/*
 * Runtime override of the answer-SDP public IP (ble config "pip" value);
 * empty buffer means the menuconfig default (CONFIG_GW_RTC_PUBLIC_IP).
 */
static char s_rtc_public_ip[48];

void webrtc_session_set_public_ip(const char *ip)
{
    strlcpy(s_rtc_public_ip, ip ? ip : "", sizeof(s_rtc_public_ip));
}

static const char *s_public_ip(void)
{
    return s_rtc_public_ip[0] ? s_rtc_public_ip : CONFIG_GW_RTC_PUBLIC_IP;
}

/*
 * Rewrite the answer into out (out_cap must have headroom, WHIP_ANSWER_MAX is
 * fine). Returns the new length, or 0 if the answer is malformed/truncated.
 */
static size_t sdp_rewrite_answer(const char *in, size_t in_len, char *out, size_t out_cap)
{
    const char *pub = s_public_ip();
    char mids[SDP_MAX_SECTIONS][48];
    bool has_mid[SDP_MAX_SECTIONS] = { false };
    int nsec = 0;

    /* Pass 1: each media section's a=mid, wherever it sits in the section. */
    int cur = -1;
    size_t pos = 0;
    while (pos < in_len) {
        size_t eol = pos;
        while (eol < in_len && in[eol] != '\n') {
            eol++;
        }
        size_t ll = eol - pos;
        if (ll > 0 && in[pos + ll - 1] == '\r') {
            ll--;
        }
        if (ll >= 2 && strncmp(&in[pos], "m=", 2) == 0) {
            cur = (nsec < SDP_MAX_SECTIONS) ? nsec++ : -1;
        } else if (cur >= 0 && !has_mid[cur] && ll >= 6 &&
                   strncmp(&in[pos], "a=mid:", 6) == 0) {
            size_t n = ll;
            if (n >= sizeof(mids[cur])) {
                n = sizeof(mids[cur]) - 1;
            }
            memcpy(mids[cur], &in[pos], n);
            mids[cur][n] = '\0';
            has_mid[cur] = true;
        }
        pos = eol + 1;
    }

    /* Pass 2: rebuild with mid right after m= and rewritten candidates. */
    size_t o = 0;
    cur = -1;
    bool mid_done = true;
    int ports[SDP_MAX_CANDS];
    int nports = 0;
    pos = 0;
    while (pos < in_len) {
        size_t eol = pos;
        while (eol < in_len && in[eol] != '\n') {
            eol++;
        }
        size_t next = eol + 1;
        size_t ll = eol - pos;
        if (ll > 0 && in[pos + ll - 1] == '\r') {
            ll--;
        }
        const char *lp = &in[pos];

        if (ll >= 2 && strncmp(lp, "m=", 2) == 0) {
            cur++;
            if (o + ll + 2 > out_cap) {
                return 0;
            }
            memcpy(&out[o], lp, ll);
            o += ll;
            out[o++] = '\r';
            out[o++] = '\n';
            if (cur < nsec && has_mid[cur]) {
                size_t ml = strlen(mids[cur]);
                if (o + ml + 2 > out_cap) {
                    return 0;
                }
                memcpy(&out[o], mids[cur], ml);
                o += ml;
                out[o++] = '\r';
                out[o++] = '\n';
                mid_done = true;
            } else {
                mid_done = false;
            }
        } else if (ll >= 6 && strncmp(lp, "a=mid:", 6) == 0) {
            if (!mid_done) {
                if (o + ll + 2 > out_cap) {
                    return 0;
                }
                memcpy(&out[o], lp, ll);
                o += ll;
                out[o++] = '\r';
                out[o++] = '\n';
                mid_done = true;
            }
            /* else: already emitted right after the m= line */
        } else if (ll >= 12 && strncmp(lp, "a=candidate:", 12) == 0) {
            size_t tl = 0, al = 0, pl = 0;
            const char *tr = sdp_field(lp, 2, &tl);   /* transport */
            const char *ad = sdp_field(lp, 4, &al);   /* address   */
            const char *po = sdp_field(lp, 5, &pl);   /* port      */
            char pbuf[8];
            int port = 0;
            bool ipv4_udp = tr != NULL && ad != NULL && po != NULL &&
                            pl > 0 && pl < sizeof(pbuf) &&
                            tl == 3 && strncasecmp(tr, "udp", 3) == 0 &&
                            memchr(ad, ':', al) == NULL;
            bool host_cand = ipv4_udp && strstr(lp, " typ host") != NULL;
            bool dup = false;
            if (ipv4_udp) {
                memcpy(pbuf, po, pl);
                pbuf[pl] = '\0';
                port = atoi(pbuf);
                ipv4_udp = port > 0;
                for (int i = 0; i < nports; i++) {
                    if (ports[i] == port) {
                        dup = true;
                    }
                }
            }
            if (ipv4_udp && !dup && nports < SDP_MAX_CANDS) {
                ports[nports++] = port;
                if (host_cand) {
                    /* rewrite: keep the line, replace only the address field */
                    size_t addr_off = (size_t)(ad - lp);
                    size_t rest_len = ll - (size_t)(po - lp);
                    size_t need = addr_off + strlen(pub) + 1 + rest_len + 2;
                    if (o + need > out_cap) {
                        return 0;
                    }
                    memcpy(&out[o], lp, addr_off);
                    o += addr_off;
                    strcpy(&out[o], pub);
                    o += strlen(pub);
                    out[o++] = ' ';
                    memcpy(&out[o], po, rest_len);
                    o += rest_len;
                    out[o++] = '\r';
                    out[o++] = '\n';
                } else {
                    /* non-host IPv4 UDP candidate (srflx/relay): keep as-is */
                    if (o + ll + 2 > out_cap) {
                        return 0;
                    }
                    memcpy(&out[o], lp, ll);
                    o += ll;
                    out[o++] = '\r';
                    out[o++] = '\n';
                }
            }
            /* TCP / IPv6 / malformed / duplicate / over-budget: dropped */
        } else if (ll >= 9 && strncmp(lp, "c=IN IP4 ", 9) == 0) {
            size_t plen = strlen(pub);
            if (o + 9 + plen + 2 > out_cap) {
                return 0;
            }
            memcpy(&out[o], "c=IN IP4 ", 9);
            o += 9;
            strcpy(&out[o], pub);
            o += plen;
            out[o++] = '\r';
            out[o++] = '\n';
        } else {
            if (o + ll + 2 > out_cap) {
                return 0;
            }
            memcpy(&out[o], lp, ll);
            o += ll;
            out[o++] = '\r';
            out[o++] = '\n';
        }
        pos = next;
    }
    if (o == 0) {
        return 0;
    }
    out[o] = '\0';
    return o;
}

/* ---------- session orchestration ---------- */

static esp_err_t build_offer_url(const webrtc_session_t *s, char *url, size_t cap)
{
    int n = snprintf(url, cap, "%s/api/v1/webrtc/realtime?sessionId=%s",
                     s->base_url, s->session_id);
    return (n > 0 && (size_t)n < cap) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

esp_err_t webrtc_session_run(webrtc_session_t *session)
{
    if (session == NULL || session->base_url[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    session->state = WEBRTC_SESSION_NEGOTIATING;
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.session = session;
    s_diag_pts_prev = -1;      /* per-session fresh pts baseline */
    s_plc_last_used = false;   /* per-session fresh PLC source */

    esp_err_t err = media_codec_open(&session->codec);
    if (err != ESP_OK) {
        session->state = WEBRTC_SESSION_FAILED;
        return err;
    }

    s_ctx.sdp = heap_caps_malloc(SDP_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_ctx.chunk_buf = heap_caps_malloc(MESSAGE_MAX + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_ctx.events = xQueueCreate(16, sizeof(char *));
    if (s_ctx.sdp == NULL || s_ctx.chunk_buf == NULL || s_ctx.events == NULL) {
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    esp_peer_audio_stream_info_t audio = {
        .codec = ESP_PEER_AUDIO_CODEC_OPUS,
        .sample_rate = CODEC_PCM_RATE,
        .channel = 1,
    };
    esp_peer_cfg_t cfg = {
        .role = ESP_PEER_ROLE_CONTROLLING,
        .ice_trans_policy = ESP_PEER_ICE_TRANS_POLICY_ALL,
        .audio_info = audio,
        .audio_dir = ESP_PEER_MEDIA_DIR_SEND_RECV,
        .video_dir = ESP_PEER_MEDIA_DIR_NONE,
        .enable_data_channel = true,
        .manual_ch_create = true,   /* client adds only the placeholder channel */
        .ctx = &s_ctx,
        .on_state = on_state,
        .on_msg = on_msg,
        .on_audio_data = on_audio_data,
        .on_channel_open = on_channel_open,
        .on_data = on_data,
    };
    esp_peer_default_cfg_t default_cfg = {
        .agent_recv_timeout = 200,
        .data_ch_cfg = {
            .recv_cache_size = 2048,
            .send_cache_size = 2048,
        },
        .rtp_cfg = {
            .audio_recv_jitter = {
                .cache_size = 4096,
            },
            .send_pool_size = 4096,
            .send_queue_num = 32,
        },
    };
    cfg.extra_cfg = &default_cfg;
    cfg.extra_size = sizeof(default_cfg);

    int ret = esp_peer_open(&cfg, esp_peer_get_default_impl(), &s_ctx.peer);
    if (ret != 0) {
        ESP_LOGE(TAG, "esp_peer_open ret=%d", ret);
        err = ESP_FAIL;
        goto cleanup;
    }

    /* Pin to CPU1: Opus+mSBC+SRTP keep IDLE0 starved otherwise, which broke
     * the SCO link mid-call (task_wdt + BT xmit Q overflow -> phone hangup). */
    static StackType_t *s_peer_stack;
    static StaticTask_t s_peer_tcb;
    static TaskHandle_t s_peer_handle;
    /* The previous peer_loop (static TCB) must be fully reaped by idle before reuse. */
    for (int i = 0; i < 50 && s_peer_handle != NULL &&
                    eTaskGetState(s_peer_handle) != eDeleted; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    s_peer_handle = psram_task_create(peer_task, "peer_loop", 24576, &s_ctx, 5, 1,
                                      &s_peer_stack, &s_peer_tcb);
    if (s_peer_handle == NULL) {
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    ret = esp_peer_new_connection(s_ctx.peer);
    if (ret != 0) {
        ESP_LOGE(TAG, "esp_peer_new_connection ret=%d", ret);
        err = ESP_FAIL;
        goto cleanup;
    }

    /* Wait for the local SDP (candidates gathered) */
    int64_t deadline_us = esp_timer_get_time() + 15 * 1000 * 1000LL;
    while (!s_ctx.sdp_ready && esp_timer_get_time() < deadline_us) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!s_ctx.sdp_ready) {
        ESP_LOGE(TAG, "local SDP not generated in time");
        err = ESP_ERR_TIMEOUT;
        goto cleanup;
    }
    sdp_limit_opus_rate(s_ctx.sdp, SDP_MAX);
    {
        const char *l = strstr(s_ctx.sdp, "a=rtpmap:111");
        const char *f = strstr(s_ctx.sdp, "a=fmtp:111");
        ESP_LOGI(TAG, "offer opus: %.40s | %.140s", l ? l : "(no rtpmap)", f ? f : "(no fmtp)");
    }
    ESP_LOGI(TAG, "offer key lines:");
    sdp_log_key_lines(TAG, s_ctx.sdp, strlen(s_ctx.sdp));

    char url[256];
    err = build_offer_url(session, url, sizeof(url));
    if (err != ESP_OK) {
        goto cleanup;
    }

    char *answer = heap_caps_malloc(WHIP_ANSWER_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (answer == NULL) {
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    whip_result_t whip;
    err = whip_client_exchange(url, session->auth_token, s_ctx.sdp, answer, WHIP_ANSWER_MAX, &whip);
    if (err != ESP_OK || (whip.status != 200 && whip.status != 201)) {
        heap_caps_free(answer);
        err = (err == ESP_OK) ? ESP_FAIL : err;
        goto cleanup;
    }

    /* Rewrite the answer for this deployment (candidate rewrite + mid fix). */
    {
        char *answer2 = heap_caps_malloc(WHIP_ANSWER_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (answer2 == NULL) {
            ESP_LOGW(TAG, "no mem for answer rewrite, using raw answer");
        } else {
            size_t rewritten = sdp_rewrite_answer(answer, (size_t)whip.answer_len,
                                                  answer2, WHIP_ANSWER_MAX);
            if (rewritten > 0) {
                ESP_LOGI(TAG, "answer rewritten: %d -> %u bytes", whip.answer_len, (unsigned)rewritten);
                ESP_LOGI(TAG, "answer key lines:");
                sdp_log_key_lines(TAG, answer2, rewritten);
                heap_caps_free(answer);
                answer = answer2;
                whip.answer_len = (int)rewritten;
            } else {
                ESP_LOGW(TAG, "answer rewrite failed, using raw answer");
                heap_caps_free(answer2);
            }
        }
    }

    esp_peer_msg_t answer_msg = {
        .type = ESP_PEER_MSG_TYPE_SDP,
        .data = (uint8_t *)answer,
        .size = whip.answer_len,
    };
    ret = esp_peer_send_msg(s_ctx.peer, &answer_msg);
    heap_caps_free(answer);
    if (ret != 0) {
        ESP_LOGE(TAG, "set remote answer ret=%d", ret);
        err = ESP_FAIL;
        goto cleanup;
    }

    /* Wait for connection */
    deadline_us = esp_timer_get_time() + 20 * 1000 * 1000LL;
    while (!s_ctx.connected && !s_ctx.failed && esp_timer_get_time() < deadline_us) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!s_ctx.connected) {
        ESP_LOGE(TAG, "connect failed/timeout (failed=%d)", (int)s_ctx.failed);
        err = ESP_ERR_TIMEOUT;
        goto cleanup;
    }
    session->state = WEBRTC_SESSION_CONNECTED;
    ESP_LOGI(TAG, "CONNECTED, running %u s", (unsigned)session->run_seconds);
    sco_bridge_set_voice_ready(true);

    /*
     * Uplink pacing and control-event drain run from this task. The Opus
     * encode path needs the full session-task stack, and a separate 40KB
     * task does not fit the post-connect memory budget (31KB free after
     * esp_peer/DTLS/SCTP come up, which silently failed xTaskCreate before).
     */
    static int16_t up_pcm[CODEC_FRAME_SAMPLES];
    static uint8_t up_packet[CODEC_PACKET_MAX];
    int64_t next_us = esp_timer_get_time();
    int64_t end_us = next_us + (int64_t)session->run_seconds * 1000000LL;
    uint32_t up_fail = 0;
    int up_fail_code = 0;
    int64_t next_stat_us = next_us + 10 * 1000000LL;
    int64_t idle_since_us = 0;
    while (esp_timer_get_time() < end_us && !s_ctx.failed) {
        /* The channel exists only for a call: leave 1.5 s after it ends. */
        if (sco_bridge_call_wanted()) {
            idle_since_us = 0;
        } else if (idle_since_us == 0) {
            idle_since_us = esp_timer_get_time();
        } else if (esp_timer_get_time() - idle_since_us > 1500 * 1000LL) {
            ESP_LOGI(TAG, "call ended, closing voice channel");
            break;
        }
        next_us += CODEC_FRAME_MS * 1000;
        /* Uplink: SCO mic when available, else silence (keeps RTP flowing). */
        if (!sco_bridge_mic_read16k(up_pcm, CODEC_FRAME_SAMPLES)) {
            memset(up_pcm, 0, sizeof(up_pcm));
        }
        int n = media_codec_encode(&session->codec, up_pcm, up_packet, sizeof(up_packet));
        if (n > 0) {
            esp_peer_audio_frame_t frame = {
                .pts = session->uplink_pts,
                .data = up_packet,
                .size = n,
            };
            /*
             * libpeer maps pts straight onto the RTP timestamp and treats it
             * as MILLISECONDS (wire ts = pts * 48, 48 kHz opus clock). A
             * constant pts=0 made every packet carry ts=0, so the server-side
             * jitter buffer collapsed the whole uplink into silence. Feed
             * media time in ms: +20 per 20 ms frame.
             */
            session->uplink_pts += 20;
            int snd = esp_peer_send_audio(s_ctx.peer, &frame);
            if (snd == 0) {
                session->uplink_frames++;
            } else {
                up_fail++;
                if (up_fail_code == 0) {
                    up_fail_code = snd;
                    ESP_LOGW(TAG, "send_audio rejected, ret=%d", snd);
                }
            }
        }
        /* Drain control events, remember/log each message type */
        char *text = NULL;
        while (xQueueReceive(s_ctx.events, &text, 0) == pdTRUE) {
            cJSON *root = cJSON_Parse(text);
            if (root != NULL) {
                cJSON *type = cJSON_GetObjectItem(root, "type");
                if (cJSON_IsString(type)) {
                    snprintf(session->last_event_type, sizeof(session->last_event_type),
                             "%s", type->valuestring);
                    ESP_LOGI(TAG, "event: %s", session->last_event_type);
                }
                cJSON_Delete(root);
            }
            free(text);
        }
        int wait_ms = (int)((next_us - esp_timer_get_time()) / 1000);
        if (wait_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
        }
        if (esp_timer_get_time() >= next_stat_us) {
            next_stat_us += 10 * 1000000LL;
            sco_bridge_stats_t sco;
            sco_bridge_get_stats(&sco);
            /* TEMP-P2-DIAG: window deltas of downlink RX counters */
            static uint32_t p_rxd, p_fail, p_pad, p_trim, p_cb;
            static int32_t  p_bytes;
            static uint64_t p_level_sum;
            static uint32_t p_level_n, p_wg30, p_wg60, p_wg150, p_cbg10;
            static uint64_t p_cons;
            static uint32_t p_drops, p_dbytes;
            uint32_t w_rxd = s_diag_rxd - p_rxd;          p_rxd = s_diag_rxd;
            uint32_t w_fail = s_diag_dec_fail - p_fail;   p_fail = s_diag_dec_fail;
            int32_t w_bytes = s_diag_rxd_bytes - p_bytes;  p_bytes = (int32_t)s_diag_rxd_bytes;
            uint32_t w_pad = sco.pad_frames - p_pad;      p_pad = sco.pad_frames;
            uint32_t w_trim = sco.trims - p_trim;         p_trim = sco.trims;
            uint32_t w_cb = sco.sco_cb_calls - p_cb;      p_cb = sco.sco_cb_calls;
            ESP_LOGI(TAG, "audio 10s: up=%lu down=%lu mic=%lu/%lu spk=%lu sco_level=%d",
                     (unsigned long)session->uplink_frames,
                     (unsigned long)session->downlink_frames,
                     (unsigned long)sco.mic_frames, (unsigned long)sco.mic_empty,
                     (unsigned long)sco.spk_frames, sco_bridge_spk_level_ms());
            ESP_LOGI(TAG, "rx-diag 10s: rxd=%lu (%.1f/s) bytes=%ld dec_fail=%lu ring=%dms[%d..%d] pad=%lu trim=%lu cb=%lu",
                     (unsigned long)w_rxd, (float)w_rxd / 10.0f, (long)w_bytes,
                     (unsigned long)w_fail,
                     sco_bridge_spk_level_ms(),
                     s_diag_ring_min == (1 << 20) ? -1 : s_diag_ring_min,
                     s_diag_ring_max == -1 ? -1 : s_diag_ring_max,
                     (unsigned long)w_pad, (unsigned long)w_trim, (unsigned long)w_cb);
            ESP_LOGI(TAG, "rx-diag 10s: gap_events=%lu missing=%lu conceal=%lu pts_zero=%lu",
                     (unsigned long)(s_diag_gap_events), (unsigned long)(s_diag_gap_missing),
                     (unsigned long)(s_diag_conceal), (unsigned long)(s_diag_pts_zero));
            uint32_t w_tb = sco.trim_bytes;   /* monotonic since sco_open */
            int      w_tml = sco.trim_max_level;
            uint32_t w_lsum_delta = (uint32_t)(sco.level_sum_ms - p_level_sum);
            uint32_t w_ln_delta   = sco.level_n - p_level_n;
            p_level_sum = sco.level_sum_ms;
            p_level_n   = sco.level_n;
            uint32_t lvl_avg = w_ln_delta ? (w_lsum_delta / w_ln_delta) : 0;
            ESP_LOGI(TAG,
                     "sco-diag 10s: lvl_avg=%lums pad=%lu trim=%lu (%luB@%dms) "
                     "wrgap30/60/150=%lu/%lu/%lu cbg>10ms=%lu max=%lous",
                     (unsigned long)lvl_avg,
                     (unsigned long)w_pad, (unsigned long)w_trim,
                     (unsigned long)w_tb, (w_trim ? w_tml : 0),
                     (unsigned long)(sco.write_gap_30 - p_wg30),
                     (unsigned long)(sco.write_gap_60 - p_wg60),
                     (unsigned long)(sco.write_gap_150 - p_wg150),
                     (unsigned long)(sco.cb_gap_10ms - p_cbg10),
                     (unsigned long)sco.cb_gap_max_us);
            p_wg30 = sco.write_gap_30; p_wg60 = sco.write_gap_60;
            p_wg150 = sco.write_gap_150; p_cbg10 = sco.cb_gap_10ms;
            /* P1b/eSCO telemetry: consumed per cb (realtime = 240B/7.5ms cb
             * => 48B per ms of cb), cb length spread (retrans windows inject
             * oversized cbs), drip-drop activity. len = lifetime min/max. */
            uint32_t w_cons = (uint32_t)(sco.consumed_bytes - p_cons);
            uint32_t w_dr = sco.drop_events - p_drops;
            uint32_t w_db = sco.drop_bytes - p_dbytes;
            p_cons = sco.consumed_bytes;
            p_drops = sco.drop_events;
            p_dbytes = sco.drop_bytes;
            uint32_t cons_bpcbi = w_cb ? (uint32_t)(w_cons / w_cb) : 0;   /* B per cb */
            ESP_LOGI(TAG,
                     "sco-diag2 10s: cons=%luB (%luB/cb) len=%lu..%lu "
                     "drop=%lu (%luB)",
                     (unsigned long)w_cons,
                     (unsigned long)cons_bpcbi,
                     (unsigned long)sco.cb_len_min,
                     (unsigned long)sco.cb_len_max,
                     (unsigned long)w_dr, (unsigned long)w_db);
            static int s_diag_pts_print = 8;   /* print first pts values per boot */
            if (s_diag_pts_print > 0) {
                s_diag_pts_print--;
                ESP_LOGI(TAG, "rx-diag pts: last=%d prev=%d", (int)s_diag_pts_prev, (int)s_diag_pts_last2);
            }
            (void)0; /* TEMP-P2-DIAG */
            s_diag_ring_min = 1 << 20; s_diag_ring_max = -1;   /* TEMP-P2-DIAG window resets */
            s_diag_gap_events = 0; s_diag_gap_missing = 0; s_diag_conceal = 0; s_diag_pts_zero = 0;
        }
    }

    sco_bridge_set_voice_ready(false);
    sco_bridge_stats_t sco;
    sco_bridge_get_stats(&sco);
    ESP_LOGI(TAG, "summary: up=%lu down=%lu up_fail=%lu(%d) ctrl_rx=%lu ctrl_tx=%lu chunks=%lu last=%s",
             (unsigned long)session->uplink_frames,
             (unsigned long)session->downlink_frames,
             (unsigned long)up_fail, up_fail_code,
             (unsigned long)session->control_rx_events,
             (unsigned long)session->control_tx_events,
             (unsigned long)session->control_rx_chunks,
             session->last_event_type);
    ESP_LOGI(TAG, "sco: mic=%lu silence=%lu spk=%lu pad=%lu trim=%lu cb=%lu dec_fail=%lu",
             (unsigned long)sco.mic_frames, (unsigned long)sco.mic_empty,
             (unsigned long)sco.spk_frames, (unsigned long)sco.pad_frames,
             (unsigned long)sco.trims, (unsigned long)sco.sco_cb_calls,
             (unsigned long)session->codec.decode_failures);

    session->state = WEBRTC_SESSION_CLOSED;

    if (whip.location[0] != '\0') {
        char full[320];
        if (whip.location[0] == '/') {
            snprintf(full, sizeof(full), "%s%s", session->base_url, whip.location);
        } else {
            snprintf(full, sizeof(full), "%s", whip.location);
        }
        whip_client_delete(full, session->auth_token);
    }

cleanup:
    sco_bridge_set_voice_ready(false);
    if (s_ctx.peer != NULL) {
        esp_peer_close(s_ctx.peer);
        s_ctx.peer = NULL;
    }
    if (s_ctx.events != NULL) {
        char *text = NULL;
        while (xQueueReceive(s_ctx.events, &text, 0) == pdTRUE) {
            free(text);
        }
        vQueueDelete(s_ctx.events);
        s_ctx.events = NULL;
    }
    if (s_ctx.sdp != NULL) {
        heap_caps_free(s_ctx.sdp);
        s_ctx.sdp = NULL;
    }
    if (s_ctx.chunk_buf != NULL) {
        heap_caps_free(s_ctx.chunk_buf);
        s_ctx.chunk_buf = NULL;
    }
    if (err != ESP_OK) {
        session->state = WEBRTC_SESSION_FAILED;
    }
    media_codec_close(&session->codec);
    return err;
}
