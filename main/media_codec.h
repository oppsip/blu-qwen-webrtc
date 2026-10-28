#ifndef MEDIA_CODEC_H
#define MEDIA_CODEC_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CODEC_PCM_RATE      16000   /* Opus logical rate used with the gateway */
#define CODEC_FRAME_MS      20
#define CODEC_FRAME_SAMPLES (CODEC_PCM_RATE * CODEC_FRAME_MS / 1000)   /* 320 */
#define CODEC_PACKET_MAX    512     /* one encoded Opus packet, well above 24 kbps */

typedef struct {
    void *encoder;
    void *decoder;
    uint32_t frames_encoded;
    uint32_t frames_decoded;
    uint32_t decode_failures;
} media_codec_t;

/** Open Opus encoder+decoder pair at 16 kHz mono 20 ms (VoIP, complexity 0). */
esp_err_t media_codec_open(media_codec_t *codec);

void media_codec_close(media_codec_t *codec);

/** PCM in (CODEC_FRAME_SAMPLES samples) -> packet out; returns packet length or <0. */
int media_codec_encode(media_codec_t *codec, const int16_t *pcm, uint8_t *packet, size_t cap);

/** Opus packet in -> PCM out (CODEC_FRAME_SAMPLES samples); returns sample count or <0. */
int media_codec_decode(media_codec_t *codec, const uint8_t *packet, size_t len, int16_t *pcm, size_t samples_cap);

#ifdef __cplusplus
}
#endif
#endif /* MEDIA_CODEC_H */
