#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Runtime system configuration, persisted in NVS over the Kconfig defaults.
 * Namespaces (own): "wifi", "rtc", "bt", "system"; factory reset keeps the
 * legacy "hfp_mgr" namespace untouched so an already-bonded phone keeps
 * working.
 */
typedef struct {
    /* WiFi (namespace "wifi") */
    char wifi_ssid[33];
    char wifi_password[65];

    /* WebRTC voice service (namespace "rtc") */
    char     rtc_base_url[96];
    char     rtc_token[64];
    uint32_t rtc_session_seconds;   /* one session cycle, seconds */
    char     rtc_public_ip[16];     /* gateway public IP for the answer rewrite */

    /* Classic BT / HFP (namespace "bt") */
    char bt_device_name[32];        /* ESP32 broadcast name          */
    char bt_peer_addr[18];          /* "aa:bb:cc:dd:ee:ff" or empty  */
    bool bt_auto_answer;
} system_config_t;

/**
 * Initialize the config manager: NVS already initialized by the caller.
 * Loads Kconfig defaults, then overlays anything present in NVS.
 */
esp_err_t config_manager_init(void);

/** Current configuration (in-memory singleton, read-only). */
const system_config_t *config_get(void);

/** Persist WiFi credentials. */
esp_err_t config_set_wifi(const char *ssid, const char *password);

/**
 * Persist voice-service settings. NULL/zero parameters keep the stored
 * value; session_seconds must be 30..3600.
 */
esp_err_t config_set_rtc(const char *base_url, const char *token,
                         int32_t session_seconds, const char *public_ip);

/**
 * Persist Bluetooth settings. NULL/zero parameters keep the stored value.
 * peer_addr may be "" to clear the explicit override (falls back to the
 * NVS-adopted address the phone bonded with).
 */
esp_err_t config_set_bluetooth(const char *device_name, const char *peer_addr,
                               int32_t auto_answer);

/**
 * Factory reset for the user configuration: erases the wifi/rtc/bt/system
 * namespaces, removes Classic BT bonds (when Bluedroid is up), and restores
 * the menuconfig defaults in memory. The legacy "hfp_mgr" namespace stays.
 * Call esp_restart() afterwards.
 */
esp_err_t config_reset_defaults(void);

/** true before any BLE-config session has ever been saved. */
bool config_is_first_boot(void);

/** Persist the "configured" flag (call after a config session applies). */
void config_mark_configured(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MANAGER_H */
