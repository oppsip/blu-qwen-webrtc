#include "media_codec.h"

#include <string.h>

#include "esp_log.h"
#include "esp_opus_enc.h"
#include "esp_opus_dec.h"

static const char *TAG = "codec";

esp_err_t media_codec_open(media_codec_t *codec)
{
    if (codec == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(codec, 0, sizeof(*codec));

    esp_opus_enc_config_t enc_cfg = ESP_OPUS_ENC_CONFIG_DEFAULT();
    enc_cfg.sample_rate = CODEC_PCM_RATE;
    enc_cfg.channel = ESP_AUDIO_MONO;
    enc_cfg.bitrate = 24000;
    enc_cfg.frame_duration = ESP_OPUS_ENC_FRAME_DURATION_20_MS;
    enc_cfg.application_mode = ESP_OPUS_ENC_APPLICATION_VOIP;
    enc_cfg.complexity = 0;
    /* FEC doubles the encoder cost (silk LPC/LBRR path) and kept IDLE0 on
     * CPU0 starved mid-call; disable until basic SCO bridging is stable. */
    enc_cfg.enable_fec = false;
    enc_cfg.enable_dtx = false;
    enc_cfg.enable_vbr = true;
    if (esp_opus_enc_open(&enc_cfg, sizeof(enc_cfg), &codec->encoder) != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "opus encoder open failed");
        return ESP_FAIL;
    }

    esp_opus_dec_cfg_t dec_cfg = {
        .sample_rate = CODEC_PCM_RATE,
        .channel = ESP_AUDIO_MONO,
        .frame_duration = ESP_OPUS_DEC_FRAME_DURATION_20_MS,
        .self_delimited = false,
    };
    if (esp_opus_dec_open(&dec_cfg, sizeof(dec_cfg), &codec->decoder) != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "opus decoder open failed");
        esp_opus_enc_close(codec->encoder);
        codec->encoder = NULL;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "opus 16k mono 20ms pair open (fec=0 vbr=1)");
    return ESP_OK;
}

void media_codec_close(media_codec_t *codec)
{
    if (codec == NULL) {
        return;
    }
    if (codec->encoder) {
        esp_opus_enc_close(codec->encoder);
        codec->encoder = NULL;
    }
    if (codec->decoder) {
        esp_opus_dec_close(codec->decoder);
        codec->decoder = NULL;
    }
}

int media_codec_encode(media_codec_t *codec, const int16_t *pcm, uint8_t *packet, size_t cap)
{
    if (codec == NULL || codec->encoder == NULL || pcm == NULL || packet == NULL || cap == 0) {
        return -1;
    }
    esp_audio_enc_in_frame_t in = {
        .buffer = (uint8_t *)pcm,
        .len = CODEC_FRAME_SAMPLES * sizeof(int16_t),
    };
    esp_audio_enc_out_frame_t out = {.buffer = packet, .len = (int)cap};
    if (esp_opus_enc_process(codec->encoder, &in, &out) != ESP_AUDIO_ERR_OK) {
        return -2;
    }
    codec->frames_encoded++;
    return (int)out.encoded_bytes;
}

int media_codec_decode(media_codec_t *codec, const uint8_t *packet, size_t len, int16_t *pcm, size_t samples_cap)
{
    if (codec == NULL || codec->decoder == NULL || packet == NULL || len == 0 ||
        pcm == NULL || samples_cap < CODEC_FRAME_SAMPLES) {
        return -1;
    }
    esp_audio_dec_in_raw_t raw = {.buffer = (uint8_t *)packet, .len = (int)len};
    esp_audio_dec_out_frame_t frame = {
        .buffer = (uint8_t *)pcm,
        .len = (int)(samples_cap * sizeof(int16_t)),
    };
    esp_audio_dec_info_t info;
    if (esp_opus_dec_decode(codec->decoder, &raw, &frame, &info) != ESP_AUDIO_ERR_OK) {
        codec->decode_failures++;
        return -2;
    }
    codec->frames_decoded++;
    return (int)(frame.decoded_size / sizeof(int16_t));
}
