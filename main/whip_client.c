/*
 * WHIP-style signaling for the Qwen gateway WebRTC endpoint.
 *
 * POST /api/v1/webrtc/realtime?sessionId=main  (Content-Type: application/sdp)
 *   -> 200 + application/sdp answer + Location: resource URL
 * DELETE <resource> -> 204
 */

#include "whip_client.h"

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"

static const char *TAG = "whip";

#define SCRATCH_CAP 2048

typedef struct {
    char *buf;          /* body accumulator (answer or scratch) */
    size_t len;
    size_t cap;
    bool truncated;
    char *location;     /* out: Location header value */
    size_t location_cap;
} recv_ctx_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    recv_ctx_t *ctx = (recv_ctx_t *)evt->user_data;
    if (ctx == NULL) {
        return ESP_OK;
    }
    if (evt->event_id == HTTP_EVENT_ON_DATA && ctx->buf) {
        size_t room = ctx->cap - ctx->len;
        size_t copy = evt->data_len < room ? (size_t)evt->data_len : room;
        memcpy(ctx->buf + ctx->len, evt->data, copy);
        ctx->len += copy;
        if (copy < (size_t)evt->data_len) {
            ctx->truncated = true;
        }
    } else if (evt->event_id == HTTP_EVENT_ON_HEADER &&
               ctx->location && ctx->location_cap > 0 &&
               strcasecmp(evt->header_key, "Location") == 0) {
        snprintf(ctx->location, ctx->location_cap, "%s", evt->header_value);
    }
    return ESP_OK;
}

static void apply_auth(esp_http_client_handle_t client, const char *token, char *hdr, size_t hdr_cap)
{
    if (token == NULL || token[0] == '\0') {
        return;
    }
    snprintf(hdr, hdr_cap, "Bearer %s", token);
    esp_http_client_set_header(client, "Authorization", hdr);
}

esp_err_t whip_client_exchange(const char *url, const char *auth_token, const char *offer,
                               char *answer_buf, size_t answer_cap, whip_result_t *result)
{
    if (url == NULL || offer == NULL || answer_buf == NULL || result == NULL || answer_cap == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(result, 0, sizeof(*result));

    char *hdr = heap_caps_malloc(192, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (hdr == NULL) {
        return ESP_ERR_NO_MEM;
    }
    recv_ctx_t ctx = {
        .buf = answer_buf, .len = 0, .cap = answer_cap - 1, .truncated = false,
        .location = result->location, .location_cap = sizeof(result->location),
    };
    answer_buf[0] = '\0';
    result->location[0] = '\0';

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 20000,
        .buffer_size = 2048,
        .buffer_size_tx = 8192,   /* offer SDP with candidates is ~5 KB */
        .event_handler = http_event_handler,
        .user_data = &ctx,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .skip_cert_common_name_check = false,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        heap_caps_free(hdr);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/sdp");
    apply_auth(client, auth_token, hdr, 192);
    esp_http_client_set_post_field(client, offer, (int)strlen(offer));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        result->status = esp_http_client_get_status_code(client);
        answer_buf[ctx.len] = '\0';
        result->answer_len = (int)ctx.len;
        if ((result->status == 200 || result->status == 201) && ctx.truncated) {
            ESP_LOGW(TAG, "answer truncated at %u bytes", (unsigned)ctx.cap);
            err = ESP_ERR_INVALID_SIZE;
        }
        if (result->status < 200 || result->status > 299) {
            ESP_LOGW(TAG, "POST status=%d body=%.200s", result->status, answer_buf);
        } else {
            ESP_LOGI(TAG, "POST ok status=%d answer=%d bytes location=%s",
                     result->status, result->answer_len, result->location);
        }
    } else {
        ESP_LOGW(TAG, "POST perform err=%d", (int)err);
    }
    esp_http_client_cleanup(client);
    heap_caps_free(hdr);
    return err;
}

esp_err_t whip_client_delete(const char *url, const char *auth_token)
{
    if (url == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    char *hdr = heap_caps_malloc(192, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    char *scratch = heap_caps_malloc(SCRATCH_CAP, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (hdr == NULL || scratch == NULL) {
        heap_caps_free(hdr);
        heap_caps_free(scratch);
        return ESP_ERR_NO_MEM;
    }
    recv_ctx_t ctx = {.buf = scratch, .len = 0, .cap = SCRATCH_CAP - 1, .truncated = false,
                      .location = NULL, .location_cap = 0};

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_DELETE,
        .timeout_ms = 10000,
        .buffer_size = 1024,
        .event_handler = http_event_handler,
        .user_data = &ctx,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        heap_caps_free(hdr);
        heap_caps_free(scratch);
        return ESP_FAIL;
    }
    apply_auth(client, auth_token, hdr, 192);
    esp_err_t err = esp_http_client_perform(client);
    int status = err == ESP_OK ? esp_http_client_get_status_code(client) : 0;
    esp_http_client_cleanup(client);
    heap_caps_free(hdr);
    heap_caps_free(scratch);
    if (err == ESP_OK && status != 204 && status != 200) {
        ESP_LOGW(TAG, "DELETE status=%d", status);
        return ESP_FAIL;
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "DELETE ok status=%d", status);
    }
    return err;
}
