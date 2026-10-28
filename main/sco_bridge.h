#ifndef SCO_BRIDGE_H
#define SCO_BRIDGE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SCO audio bridge for the HFP-client role (device = HF, phone = AG).
 *
 * Uplink:   phone mic -> SCO -> incoming data cb -> mic ring (native rate)
 *           -> sco_bridge_mic_read16k() pull, resampled to 16 kHz.
 * Downlink: 16 kHz PCM -> sco_bridge_spk_write16k() -> spk ring
 *           -> outgoing data cb (7.5 ms pump) -> SCO -> phone speaker.
 *
 * Rings live in PSRAM (same as production AUDIO_SCO_OUT_RING_SIZE=8192).
 * The pull model means no capture task and no backlog: the session paces
 * itself every 20 ms and reads whatever is there, else encodes silence.
 */

/* Register the HFP client event + data callbacks. Call once after
 * esp_hf_client_init(). Also connects to GW_BT_PEER_ADDR when configured. */
void sco_bridge_init(void);

/* Pull exactly `samples` 16 kHz mono mic samples. Returns false when no SCO
 * audio is available (caller encodes silence/DTX instead). */
bool sco_bridge_mic_read16k(int16_t *out, size_t samples);

/* Push 16 kHz mono speaker samples toward the SCO ring (resampled to the
 * negotiated SCO rate, trimmed at the high-water mark). */
void sco_bridge_spk_write16k(const int16_t *pcm, size_t samples);

bool sco_bridge_sco_active(void);

/** True while a call is ringing, being set up or active on the phone. */
bool sco_bridge_call_wanted(void);
/** Block until a call starts (UINT32_MAX = forever); true if one is in progress. */
bool sco_bridge_wait_call(uint32_t timeout_ms);
/** Server voice channel is up; auto-answer waits for this. */
void sco_bridge_set_voice_ready(bool ready);
int  sco_bridge_spk_level_ms(void);

typedef struct {
    uint32_t mic_frames;    /* 20 ms uplink frames with real mic audio */
    uint32_t mic_empty;     /* 20 ms uplink frames filled with silence  */
    uint32_t spk_frames;    /* downlink frames pushed to the SCO ring   */
    uint32_t pad_frames;    /* outgoing cb calls padded with silence    */
    uint32_t trims;         /* spk ring over-HWM trims                  */
    uint32_t sco_cb_calls;  /* outgoing data cb invocations             */
    /* P0 stutter-spectrum diagnostics (window deltas computed by caller) */
    uint32_t trim_bytes;    /* downlink bytes dropped by over-HWM trims */
    int      trim_max_level;/* spk ring level (ms) observed at a trim   */
    uint32_t write_gap_30;  /* spk writes arriving 30-60 ms apart       */
    uint32_t write_gap_60;  /* 60-150 ms apart                          */
    uint32_t write_gap_150; /* > 150 ms apart (delivery holes)          */
    uint64_t level_sum_ms;  /* sum of spk ring level ms sampled per cb  */
    uint32_t level_n;       /* cb samples for level_sum_ms              */
    uint32_t cb_gap_max_us; /* max SCO cb inter-arrival gap (us)        */
    uint32_t cb_gap_10ms;   /* SCO cb inter-arrival gaps > 10 ms        */
    /* cb size + flow telemetry: reveals eSCO retrans slacks (cb length /
     * rate jumps) and the consumed-vs-produced audio balance.             */
    uint32_t cb_len_min;    /* smallest len requested by the BT stack   */
    uint32_t cb_len_max;    /* largest len requested by the BT stack    */
    uint64_t consumed_bytes;/* PCM bytes actually read from the spk ring */
    uint32_t drop_events;   /* drip-drop skip events (P1b anti-burst)   */
    uint32_t drop_bytes;    /* drip-drop skipped PCM bytes              */
} sco_bridge_stats_t;

void sco_bridge_get_stats(sco_bridge_stats_t *out);

#ifdef __cplusplus
}
#endif
#endif /* SCO_BRIDGE_H */
