#ifndef WEBRTC_SESSION_H
#define WEBRTC_SESSION_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "media_codec.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WEBRTC_SESSION_IDLE = 0,
    WEBRTC_SESSION_NEGOTIATING,     /* offer posted, waiting for answer/ICE */
    WEBRTC_SESSION_CONNECTED,       /* ICE+DTLS connected */
    WEBRTC_SESSION_FAILED,
    WEBRTC_SESSION_CLOSED,
} webrtc_session_state_t;

typedef struct {
    /* Config: set before connect */
    char     base_url[192];    /* https://host:port */
    char     auth_token[96];
    char     session_id[32];   /* "main" */
    uint32_t run_seconds;      /* how long to keep the session (P2 smoke) */

    /* State */
    webrtc_session_state_t state;
    media_codec_t codec;
    uint32_t uplink_frames;       /* encoded 20ms frames sent */
    uint32_t uplink_pts;          /* media time in ms fed to libpeer pts */
    uint32_t downlink_frames;     /* decoded 20ms frames received */
    uint32_t control_rx_events;   /* JSON events on txt channel */
    uint32_t control_tx_events;
    uint32_t control_rx_chunks;   /* transport chunk frames consumed */
    char     last_event_type[48];
} webrtc_session_t;

/**
 * Bring up the session: codec, esp_peer, WHIP signaling, ICE/DTLS.
 * Blocks until connected, failed, or run_seconds elapsed (whichever first),
 * then closes and returns 0 on success.
 */
esp_err_t webrtc_session_run(webrtc_session_t *session);

/**
 * Override the gateway public IP used by the answer-SDP rewrite. Call before
 * webrtc_session_run(); empty string keeps the menuconfig default
 * (CONFIG_GW_RTC_PUBLIC_IP).
 */
void webrtc_session_set_public_ip(const char *ip);

#ifdef __cplusplus
}
#endif
#endif /* WEBRTC_SESSION_H */
