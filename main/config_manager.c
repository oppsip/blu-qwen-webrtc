/*
 * Runtime configuration: NVS-backed values over menuconfig defaults.
 * Structure follows au_gw/config_manager.c (trimmed to this firmware's
 * scope: WiFi, voice-service and Bluetooth settings).
 */
#include "config_manager.h"

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "nvs.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"

static const char *TAG = "cfg";

/* NVS namespaces */
#define NS_WIFI   "wifi"
#define NS_RTC    "rtc"
#define NS_BT     "bt"
#define NS_SYSTEM "system"

static system_config_t s_config;

/* ---- defaults from menuconfig (device works like today with empty NVS) ---- */

static void config_apply_defaults(void)
{
    memset(&s_config, 0, sizeof(s_config));
    strlcpy(s_config.wifi_ssid,     CONFIG_GW_WIFI_SSID,        sizeof(s_config.wifi_ssid));
    strlcpy(s_config.wifi_password, CONFIG_GW_WIFI_PASSWORD,    sizeof(s_config.wifi_password));
    strlcpy(s_config.rtc_base_url,  CONFIG_GW_RTC_BASE_URL,     sizeof(s_config.rtc_base_url));
    strlcpy(s_config.rtc_token,     CONFIG_GW_RTC_TOKEN,        sizeof(s_config.rtc_token));

    int32_t sec = (int32_t)CONFIG_GW_RTC_SESSION_SECONDS;
    if (sec < 30) sec = 30;
    if (sec > 3600) sec = 3600;
    s_config.rtc_session_seconds = (uint32_t)sec;

    strlcpy(s_config.rtc_public_ip, CONFIG_GW_RTC_PUBLIC_IP,    sizeof(s_config.rtc_public_ip));
    strlcpy(s_config.bt_device_name, CONFIG_GW_BT_DEVICE_NAME,  sizeof(s_config.bt_device_name));
    strlcpy(s_config.bt_peer_addr,  CONFIG_GW_BT_PEER_ADDR,     sizeof(s_config.bt_peer_addr));
    s_config.bt_auto_answer = CONFIG_GW_BT_AUTO_ANSWER;
}

static bool cfg_is_first_boot = true;

/* ---- string helpers ------------------------------------------------------ */

static esp_err_t nvs_write_str(const char *ns, const char *key, const char *val)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(h, key, val ? val : "");
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

static esp_err_t nvs_write_u8(const char *ns, const char *key, uint8_t val)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u8(h, key, val);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

static esp_err_t nvs_write_u32(const char *ns, const char *key, uint32_t val)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u32(h, key, val);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

/* ---- public API ---------------------------------------------------------- */

const system_config_t *config_get(void)
{
    return &s_config;
}

esp_err_t config_set_wifi(const char *ssid, const char *password)
{
    if (ssid == NULL || ssid[0] == '\0' || password == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS_WIFI, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(h, "ssid", ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(h, "password", password);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi save failed: %s", esp_err_to_name(err));
        return err;
    }
    strlcpy(s_config.wifi_ssid, ssid, sizeof(s_config.wifi_ssid));
    strlcpy(s_config.wifi_password, password, sizeof(s_config.wifi_password));
    return ESP_OK;
}

esp_err_t config_set_rtc(const char *base_url, const char *token,
                         int32_t session_seconds, const char *public_ip)
{
    esp_err_t err = ESP_OK;
    if (base_url != NULL) {
        err = nvs_write_str(NS_RTC, "base_url", base_url);
        if (err == ESP_OK) {
            strlcpy(s_config.rtc_base_url, base_url, sizeof(s_config.rtc_base_url));
        }
    }
    if (err == ESP_OK && token != NULL) {
        err = nvs_write_str(NS_RTC, "token", token);
        if (err == ESP_OK) {
            strlcpy(s_config.rtc_token, token, sizeof(s_config.rtc_token));
        }
    }
    if (err == ESP_OK && session_seconds > 0) {
        uint32_t sec = (uint32_t)session_seconds;
        if (sec < 30) sec = 30;
        if (sec > 3600) sec = 3600;
        err = nvs_write_u32(NS_RTC, "session_s", sec);
        if (err == ESP_OK) {
            s_config.rtc_session_seconds = sec;
        }
    }
    if (err == ESP_OK && public_ip != NULL) {
        err = nvs_write_str(NS_RTC, "public_ip", public_ip);
        if (err == ESP_OK) {
            strlcpy(s_config.rtc_public_ip, public_ip, sizeof(s_config.rtc_public_ip));
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rtc save failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t config_set_bluetooth(const char *device_name, const char *peer_addr,
                               int32_t auto_answer)
{
    esp_err_t err = ESP_OK;
    if (device_name != NULL && device_name[0] != '\0') {
        err = nvs_write_str(NS_BT, "name", device_name);
        if (err == ESP_OK) {
            strlcpy(s_config.bt_device_name, device_name, sizeof(s_config.bt_device_name));
        }
    }
    if (err == ESP_OK && peer_addr != NULL) {
        err = nvs_write_str(NS_BT, "peer", peer_addr);
        if (err == ESP_OK) {
            strlcpy(s_config.bt_peer_addr, peer_addr, sizeof(s_config.bt_peer_addr));
        }
    }
    if (err == ESP_OK && auto_answer >= 0) {
        uint8_t val = auto_answer ? 1 : 0;
        err = nvs_write_u8(NS_BT, "auto_answer", val);
        if (err == ESP_OK) {
            s_config.bt_auto_answer = val != 0;
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bt save failed: %s", esp_err_to_name(err));
    }
    return err;
}

/* Remove Classic BT bonds. Only usable while Bluedroid is enabled (the BLE
 * config mode enables it). */
#define CFG_BOND_LIST_MAX 16
static void clear_classic_bonds(void)
{
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        return;
    }
    int dev_num = esp_bt_gap_get_bond_device_num();
    if (dev_num <= 0) {
        return;
    }
    if (dev_num > CFG_BOND_LIST_MAX) {
        dev_num = CFG_BOND_LIST_MAX;
    }
    esp_bd_addr_t list[CFG_BOND_LIST_MAX];
    int fetched = dev_num;
    if (esp_bt_gap_get_bond_device_list(&fetched, list) != ESP_OK) {
        return;
    }
    for (int i = 0; i < fetched; i++) {
        if (esp_bt_gap_remove_bond_device(list[i]) == ESP_OK) {
            ESP_LOGI(TAG, "removed Classic bond " ESP_BD_ADDR_STR,
                     ESP_BD_ADDR_HEX(list[i]));
        }
    }
}

esp_err_t config_reset_defaults(void)
{
    const char *namespaces_to_erase[] = { NS_WIFI, NS_RTC, NS_BT, NS_SYSTEM };
    esp_err_t last_err = ESP_OK;
    for (size_t i = 0; i < sizeof(namespaces_to_erase) / sizeof(namespaces_to_erase[0]); i++) {
        nvs_handle_t h;
        if (nvs_open(namespaces_to_erase[i], NVS_READWRITE, &h) == ESP_OK) {
            esp_err_t err = nvs_erase_all(h);
            if (err == ESP_OK) {
                err = nvs_commit(h);
            }
            nvs_close(h);
            if (err != ESP_OK) {
                last_err = err;
            }
            ESP_LOGW(TAG, "factory reset: erased namespace %s", namespaces_to_erase[i]);
        }
    }
    clear_classic_bonds();
    config_apply_defaults();
    return last_err;
}

bool config_is_first_boot(void)
{
    return cfg_is_first_boot;
}

void config_mark_configured(void)
{
    if (nvs_write_u8(NS_SYSTEM, "initialized", 1) == ESP_OK) {
        cfg_is_first_boot = false;
    }
}

esp_err_t config_manager_init(void)
{
    config_apply_defaults();

    char str_buf[96];
    size_t slen;
    uint32_t u32v;
    uint8_t u8v;
    nvs_handle_t h;

    if (nvs_open(NS_WIFI, NVS_READONLY, &h) == ESP_OK) {
        slen = sizeof(str_buf);
        if (nvs_get_str(h, "ssid", str_buf, &slen) == ESP_OK && str_buf[0]) {
            strlcpy(s_config.wifi_ssid, str_buf, sizeof(s_config.wifi_ssid));
        }
        slen = sizeof(str_buf);
        if (nvs_get_str(h, "password", str_buf, &slen) == ESP_OK) {
            strlcpy(s_config.wifi_password, str_buf, sizeof(s_config.wifi_password));
        }
        nvs_close(h);
    }

    if (nvs_open(NS_RTC, NVS_READONLY, &h) == ESP_OK) {
        slen = sizeof(str_buf);
        if (nvs_get_str(h, "base_url", str_buf, &slen) == ESP_OK && str_buf[0]) {
            strlcpy(s_config.rtc_base_url, str_buf, sizeof(s_config.rtc_base_url));
        }
        slen = sizeof(str_buf);
        if (nvs_get_str(h, "token", str_buf, &slen) == ESP_OK) {
            strlcpy(s_config.rtc_token, str_buf, sizeof(s_config.rtc_token));
        }
        slen = sizeof(str_buf);
        if (nvs_get_str(h, "public_ip", str_buf, &slen) == ESP_OK && str_buf[0]) {
            strlcpy(s_config.rtc_public_ip, str_buf, sizeof(s_config.rtc_public_ip));
        }
        if (nvs_get_u32(h, "session_s", &u32v) == ESP_OK) {
            if (u32v < 30) u32v = 30;
            if (u32v > 3600) u32v = 3600;
            s_config.rtc_session_seconds = u32v;
        }
        nvs_close(h);
    }

    if (nvs_open(NS_BT, NVS_READONLY, &h) == ESP_OK) {
        slen = sizeof(str_buf);
        if (nvs_get_str(h, "name", str_buf, &slen) == ESP_OK && str_buf[0]) {
            strlcpy(s_config.bt_device_name, str_buf, sizeof(s_config.bt_device_name));
        }
        slen = sizeof(str_buf);
        if (nvs_get_str(h, "peer", str_buf, &slen) == ESP_OK &&
                slen < sizeof(s_config.bt_peer_addr)) {
            strlcpy(s_config.bt_peer_addr, str_buf, sizeof(s_config.bt_peer_addr));
        }
        if (nvs_get_u8(h, "auto_answer", &u8v) == ESP_OK) {
            s_config.bt_auto_answer = u8v != 0;
        }
        nvs_close(h);
    }

    /* First-boot flag: namespace missing / key missing => never configured. */
    cfg_is_first_boot = true;
    if (nvs_open(NS_SYSTEM, NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_u8(h, "initialized", &u8v) == ESP_OK) {
            cfg_is_first_boot = u8v == 0;
        }
        nvs_close(h);
    }

    ESP_LOGI(TAG, "config loaded: ssid=\"%s\" base_url=%s bt=\"%s\" peer=%s "
             "session=%lus ip=%s first_boot=%d",
             s_config.wifi_ssid, s_config.rtc_base_url, s_config.bt_device_name,
             s_config.bt_peer_addr[0] ? s_config.bt_peer_addr : "(auto)",
             (unsigned long)s_config.rtc_session_seconds, s_config.rtc_public_ip,
             cfg_is_first_boot);
    return ESP_OK;
}
