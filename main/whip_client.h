#ifndef WHIP_CLIENT_H
#define WHIP_CLIENT_H

#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char location[128];   /* Resource URL from the Location header (DELETE target) */
    int  status;          /* HTTP status of the POST */
    int  answer_len;      /* Length of the SDP answer written to answer_buf */
} whip_result_t;

/**
 * POST an SDP offer to the Qwen gateway WHIP-style endpoint.
 *
 * answer_buf must be at least WHIP_ANSWER_MAX bytes. On success the answer SDP
 * is written (NUL-terminated) and result->location holds the resource URL.
 */
esp_err_t whip_client_exchange(const char *url,       /* full https://…/api/v1/webrtc/realtime?sessionId=main */
                               const char *auth_token, /* Bearer token, may be NULL */
                               const char *offer,      /* NUL-terminated SDP offer */
                               char *answer_buf,       /* out: answer SDP */
                               size_t answer_cap,      /* capacity of answer_buf */
                               whip_result_t *result);

/** DELETE the resource previously created (Location header). */
esp_err_t whip_client_delete(const char *url, const char *auth_token);

#define WHIP_ANSWER_MAX 8192

#ifdef __cplusplus
}
#endif
#endif /* WHIP_CLIENT_H */
