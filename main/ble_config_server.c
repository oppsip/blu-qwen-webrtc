/**
 * BLE Configuration GATT Server (adapted from au_gw/ble_config_server.c).
 *
 * Exposes a custom BLE service (UUID FFF0) with two characteristics:
 *   FFF1 - Write:  phone sends JSON commands (chunked, newline-terminated)
 *   FFF2 - Notify: ESP32 sends JSON responses
 *
 * Runs only during the startup configuration window (GPIO13 held low at
 * power-on); the firmware reboots after the phone saves the configuration.
 *
 * The WeChat mini program is schema-driven and reuses the au_gw protocol:
 *   {"cmd":"get_ui"}    -> {"ok":true,"fw":"x.y.z","ui":{...},"values":{...},
 *                          "contacts":{}}
 *   {"cmd":"set_config","values":{...}} -> {"ok":true} then reboot
 *
 * Low-level commands (kept for compatibility and factory tooling):
 *   {"cmd":"get_config"}
 *   {"cmd":"set_wifi","ssid":"x","pwd":"x"}
 *   {"cmd":"set_rtc","burl":"https://...","tok":"...","ssec":600,"pip":"1.2.3.4"}
 *   {"cmd":"set_bt","name":"x","peer":"aa:bb:cc:dd:ee:ff","bta":1}
 *   {"cmd":"factory_reset"}
 *   {"cmd":"reboot"}
 *
 * Value keys (values map; secrets arriving as "" or "********" keep the
 * stored value): ssid wpw burl tok ssec pip btn peer bta frs.
 */

#include "ble_config_server.h"
#include "config_manager.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

static const char *TAG = "ble_cfg";

#define SECRET_PLACEHOLDER "********"

static bool keep_secret(const char *s)
{
    return s == NULL || s[0] == '\0' || strcmp(s, SECRET_PLACEHOLDER) == 0;
}

/* ============================================================
 * BLE UUIDs and attribute indices
 * ============================================================ */

#define GATTS_SERVICE_UUID     0xFFF0
#define GATTS_CHAR_UUID_WRITE  0xFFF1
#define GATTS_CHAR_UUID_NOTIFY 0xFFF2
#define GATTS_APP_ID           0

#define BLE_CFG_MTU_MAX      200
#define BLE_CFG_RX_BUF_SIZE  4096  /* set_config carries the full values map */

enum {
    IDX_SVC,
    IDX_CHAR_WRITE_DECL,
    IDX_CHAR_WRITE_VAL,
    IDX_CHAR_NOTIFY_DECL,
    IDX_CHAR_NOTIFY_VAL,
    IDX_CHAR_NOTIFY_CFG,
    IDX_NB,
};

/* ============================================================
 * State
 * ============================================================ */

static uint16_t s_handle_table[IDX_NB];
static uint16_t s_conn_id        = 0xFFFF;
static esp_gatt_if_t s_gatts_if  = ESP_GATT_IF_NONE;
static bool s_notify_enabled     = false;
static bool s_started            = false;

static char s_rx_buf[BLE_CFG_RX_BUF_SIZE];
static int  s_rx_len             = 0;

#define BIT_REBOOT_REQUESTED  BIT0
static EventGroupHandle_t s_evt_group;

/* ============================================================
 * GATT attribute table
 * ============================================================ */

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t char_decl_uuid       = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t char_client_cfg_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t svc_uuid             = GATTS_SERVICE_UUID;
static const uint16_t char_uuid_write      = GATTS_CHAR_UUID_WRITE;
static const uint16_t char_uuid_notify     = GATTS_CHAR_UUID_NOTIFY;

static const uint8_t char_prop_write  = ESP_GATT_CHAR_PROP_BIT_WRITE |
                                        ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY |
                                        ESP_GATT_CHAR_PROP_BIT_READ;
static uint8_t cccd_value[2]          = {0x00, 0x00};

static const esp_gatts_attr_db_t s_gatt_db[IDX_NB] = {
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
         ESP_GATT_PERM_READ,
         sizeof(svc_uuid), sizeof(svc_uuid), (uint8_t *)&svc_uuid}
    },
    [IDX_CHAR_WRITE_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid,
         ESP_GATT_PERM_READ,
         sizeof(char_prop_write), sizeof(char_prop_write), (uint8_t *)&char_prop_write}
    },
    [IDX_CHAR_WRITE_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_uuid_write,
         ESP_GATT_PERM_WRITE,
         BLE_CFG_MTU_MAX, 0, NULL}
    },
    [IDX_CHAR_NOTIFY_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid,
         ESP_GATT_PERM_READ,
         sizeof(char_prop_notify), sizeof(char_prop_notify), (uint8_t *)&char_prop_notify}
    },
    [IDX_CHAR_NOTIFY_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_uuid_notify,
         ESP_GATT_PERM_READ,
         BLE_CFG_MTU_MAX, 0, NULL}
    },
    [IDX_CHAR_NOTIFY_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_client_cfg_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(cccd_value), sizeof(cccd_value), cccd_value}
    },
};

/* ============================================================
 * Advertising
 * ============================================================ */

static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min        = 0x20,   /* 20 ms */
    .adv_int_max        = 0x40,   /* 40 ms */
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ============================================================
 * Notify helpers
 * ============================================================ */

static void send_notify(const char *json)
{
    if (s_conn_id == 0xFFFF || !s_notify_enabled ||
            s_gatts_if == ESP_GATT_IF_NONE) {
        return;
    }
    static char s_notify_buf[BLE_CFG_MTU_MAX + 1];
    size_t max_chunk = BLE_CFG_MTU_MAX - 3;   /* ATT overhead */
    size_t len = strlen(json);
    size_t off = 0;

    while (off < len) {
        size_t chunk = len - off;
        if (chunk > max_chunk) {
            chunk = max_chunk;
        }
        memcpy(s_notify_buf, json + off, chunk);
        esp_ble_gatts_send_indicate(s_gatts_if, s_conn_id,
                                    s_handle_table[IDX_CHAR_NOTIFY_VAL],
                                    (uint16_t)chunk, (uint8_t *)s_notify_buf, false);
        off += chunk;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    s_notify_buf[0] = '\n';
    esp_ble_gatts_send_indicate(s_gatts_if, s_conn_id,
                                s_handle_table[IDX_CHAR_NOTIFY_VAL],
                                1, (uint8_t *)s_notify_buf, false);
}

/* send_notify() paces one chunk every 20 ms; the ~2 KB get_ui response would
 * block the BTC task for ~0.4 s, so oversized responses run in a short-lived
 * worker task instead. Takes ownership of @json (malloc'd). */
static void ble_tx_task(void *arg)
{
    char *json = (char *)arg;
    send_notify(json);
    free(json);
    vTaskDelete(NULL);
}

static void send_notify_owned(char *json)
{
    if (json == NULL) {
        send_notify("{\"ok\":false,\"err\":\"out of memory\"}");
        return;
    }
    if (xTaskCreate(ble_tx_task, "ble_tx", 3072, json, 5, NULL) != pdPASS) {
        send_notify(json);   /* best effort */
        free(json);
    }
}

/* ============================================================
 * Schema (the mini program renders what we return)
 * ============================================================ */

static cJSON *label_obj(const char *zh, const char *en)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "zh", zh);
    cJSON_AddStringToObject(o, "en", en);
    return o;
}

static cJSON *ui_field(const char *key, const char *type,
                       const char *zh, const char *en)
{
    cJSON *f = cJSON_CreateObject();
    cJSON_AddStringToObject(f, "key", key);
    cJSON_AddStringToObject(f, "type", type);
    cJSON_AddItemToObject(f, "label", label_obj(zh, en));
    return f;
}

static cJSON *build_ui(void)
{
    cJSON *ui = cJSON_CreateObject();
    cJSON *tabs = cJSON_AddArrayToObject(ui, "tabs");

    /* ---- WiFi ---- */
    {
        cJSON *tab = cJSON_CreateObject();
        cJSON_AddStringToObject(tab, "id", "wifi");
        cJSON_AddItemToObject(tab, "label", label_obj("WiFi 网络", "WiFi"));
        cJSON *fields = cJSON_AddArrayToObject(tab, "fields");

        cJSON *ssid = ui_field("ssid", "text", "SSID", "SSID");
        cJSON_AddNumberToObject(ssid, "maxlength", 32);
        cJSON_AddItemToArray(fields, ssid);

        cJSON *wpw = ui_field("wpw", "password", "WiFi 密码", "WiFi Password");
        cJSON_AddNumberToObject(wpw, "maxlength", 64);
        cJSON_AddStringToObject(wpw, "placeholder",
                                "留空或 ******** 保持原密码 / empty keeps current password");
        cJSON_AddItemToArray(fields, wpw);

        cJSON_AddItemToArray(tabs, tab);
    }

    /* ---- Voice service ---- */
    {
        cJSON *tab = cJSON_CreateObject();
        cJSON_AddStringToObject(tab, "id", "rtc");
        cJSON_AddItemToObject(tab, "label", label_obj("语音服务", "Voice Service"));
        cJSON *fields = cJSON_AddArrayToObject(tab, "fields");

        cJSON *burl = ui_field("burl", "text", "服务地址", "Base URL");
        cJSON_AddNumberToObject(burl, "maxlength", 95);
        cJSON_AddItemToArray(fields, burl);

        cJSON *tok = ui_field("tok", "password", "访问令牌", "Access Token");
        cJSON_AddNumberToObject(tok, "maxlength", 63);
        cJSON_AddStringToObject(tok, "placeholder",
                                "留空或 ******** 保持原令牌 / empty keeps current token");
        cJSON_AddItemToArray(fields, tok);

        cJSON *ssec = ui_field("ssec", "number", "会话周期（秒）", "Session Cycle (sec)");
        cJSON_AddNumberToObject(ssec, "min", 30);
        cJSON_AddNumberToObject(ssec, "max", 3600);
        cJSON_AddItemToArray(fields, ssec);

        cJSON *pip = ui_field("pip", "text", "网关公网 IP", "Gateway Public IP");
        cJSON_AddNumberToObject(pip, "maxlength", 15);
        cJSON_AddItemToArray(fields, pip);

        cJSON_AddItemToArray(tabs, tab);
    }

    /* ---- Bluetooth ---- */
    {
        cJSON *tab = cJSON_CreateObject();
        cJSON_AddStringToObject(tab, "id", "bt");
        cJSON_AddItemToObject(tab, "label", label_obj("蓝牙", "Bluetooth"));
        cJSON *fields = cJSON_AddArrayToObject(tab, "fields");

        cJSON *btn = ui_field("btn", "text", "设备名称", "Device Name");
        cJSON_AddNumberToObject(btn, "maxlength", 31);
        cJSON_AddItemToArray(fields, btn);

        cJSON *peer = ui_field("peer", "text", "手机地址（可选）", "Phone Address (optional)");
        cJSON_AddNumberToObject(peer, "maxlength", 17);
        cJSON_AddStringToObject(peer, "placeholder",
                                "aa:bb:cc:dd:ee:ff，留空=自动学习 / empty = auto");
        cJSON_AddItemToArray(fields, peer);

        cJSON *bta = ui_field("bta", "switch", "来电自动接听", "Auto Answer");
        cJSON_AddItemToArray(fields, bta);

        cJSON_AddItemToArray(tabs, tab);
    }

    /* ---- System ---- */
    {
        cJSON *tab = cJSON_CreateObject();
        cJSON_AddStringToObject(tab, "id", "sys");
        cJSON_AddItemToObject(tab, "label", label_obj("系统", "System"));
        cJSON *fields = cJSON_AddArrayToObject(tab, "fields");

        cJSON *frs = ui_field("frs", "action", "恢复出厂设置", "Factory Reset");
        cJSON_AddItemToObject(frs, "button",
                              label_obj("恢复出厂设置", "Factory Reset"));
        cJSON_AddItemToObject(frs, "confirm",
                              label_obj("将清除 WiFi、语音服务、蓝牙配置并重启，已配对手机地址学习记录保留。确定继续？",
                                        "This clears WiFi, voice-service, and Bluetooth configuration, then reboots. Continue?"));
        cJSON_AddItemToObject(frs, "hint",
                              label_obj("重启后使用固件 menuconfig 中的出厂初值。",
                                        "After reboot, menuconfig defaults apply."));
        cJSON_AddItemToArray(fields, frs);

        cJSON_AddItemToArray(tabs, tab);
    }

    return ui;
}

static const char *mask_or(const char *secret)
{
    return secret[0] ? SECRET_PLACEHOLDER : "";
}

static cJSON *build_values(void)
{
    const system_config_t *c = config_get();
    cJSON *v = cJSON_CreateObject();
    cJSON_AddStringToObject(v, "ssid", c->wifi_ssid);
    cJSON_AddStringToObject(v, "wpw", mask_or(c->wifi_password));
    cJSON_AddStringToObject(v, "burl", c->rtc_base_url);
    cJSON_AddStringToObject(v, "tok", mask_or(c->rtc_token));
    cJSON_AddNumberToObject(v, "ssec", (double)c->rtc_session_seconds);
    cJSON_AddStringToObject(v, "pip", c->rtc_public_ip);
    cJSON_AddStringToObject(v, "btn", c->bt_device_name);
    cJSON_AddStringToObject(v, "peer", c->bt_peer_addr);
    cJSON_AddBoolToObject(v, "bta", c->bt_auto_answer);
    return v;
}

/* ============================================================
 * values application (set_config path)
 * ============================================================ */

static const char *json_str(const cJSON *obj, const char *key)
{
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    return cJSON_IsString(it) ? it->valuestring : NULL;
}

static int json_int(const cJSON *obj, const char *key, int def)
{
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsNumber(it)) {
        return (int)it->valuedouble;
    }
    if (cJSON_IsString(it) && it->valuestring[0]) {
        return atoi(it->valuestring);
    }
    if (cJSON_IsBool(it)) {
        return cJSON_IsTrue(it) ? 1 : 0;
    }
    return def;
}

/* light IPv4 check for the gateway public IP (au_gw style) */
static bool valid_ipv4(const char *s)
{
    if (s == NULL || s[0] == '\0') {
        return false;
    }
    int net[4]; char tail;
    return sscanf(s, "%d.%d.%d.%d%c", &net[0], &net[1], &net[2], &net[3], &tail) == 4 &&
           net[0] >= 0 && net[0] <= 255 && net[1] >= 0 && net[1] <= 255 &&
           net[2] >= 0 && net[2] <= 255 && net[3] >= 0 && net[3] <= 255;
}

/* Normalize a BD address into lowercase "aa:bb:.." form.
 * "-" or "" means explicit clear (out=""). Returns false on garbage. */
static bool parse_peer_key(const char *in, char *out, size_t out_sz)
{
    out[0] = '\0';
    if (in == NULL) {
        return true;
    }
    if (in[0] == '\0' || strcmp(in, "-") == 0) {
        return true;   /* explicit clear, out stays "" */
    }
    unsigned a, b, c, d, e, f;
    if (sscanf(in, "%x:%x:%x:%x:%x:%x",
               &a, &b, &c, &d, &e, &f) == 6 &&
            a < 256 && b < 256 && c < 256 && d < 256 && e < 256 && f < 256 &&
            strlen(in) == strlen("aa:bb:cc:dd:ee:ff")) {
        snprintf(out, out_sz, "%02x:%02x:%02x:%02x:%02x:%02x",
                 a, b, c, d, e, f);
        return true;
    }
    return false;
}

static esp_err_t apply_values(const cJSON *values)
{
    if (!cJSON_IsObject(values)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (json_int(values, "frs", 0) != 0) {
        ESP_LOGW(TAG, "factory reset requested via BLE config");
        return config_reset_defaults();
    }

    const system_config_t *cur = config_get();
    esp_err_t err;

    const char *ssid = json_str(values, "ssid");
    const char *wpw = json_str(values, "wpw");
    if (ssid != NULL) {
        const char *pass = keep_secret(wpw) ? cur->wifi_password : wpw;
        err = config_set_wifi(ssid, pass ? pass : "");
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "apply wifi failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    /* Missing keys keep the stored value; NULL also keeps. */
    const char *burl = json_str(values, "burl");
    const char *tok = json_str(values, "tok");
    int32_t ssec = -1;
    const cJSON *ssec_item = cJSON_GetObjectItemCaseSensitive(values, "ssec");
    if (cJSON_IsNumber(ssec_item)) {
        ssec = (int32_t)ssec_item->valuedouble;
    }
    const char *pip = json_str(values, "pip");
    if (pip != NULL && pip[0] != '\0' && !valid_ipv4(pip)) {
        return ESP_ERR_INVALID_ARG;
    }
    err = config_set_rtc(burl, keep_secret(tok) ? NULL : tok, ssec,
                         (pip != NULL && pip[0] != '\0') ? pip : NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "apply rtc failed: %s", esp_err_to_name(err));
        return err;
    }

    const char *btn = json_str(values, "btn");
    int32_t bta = -1;
    const cJSON *bta_item = cJSON_GetObjectItemCaseSensitive(values, "bta");
    if (cJSON_IsBool(bta_item) || cJSON_IsNumber(bta_item)) {
        bta = json_int(values, "bta", cur->bt_auto_answer ? 1 : 0);
    }
    char peer_buf[19];
    bool peer_given = false;
    const char *peer = json_str(values, "peer");
    if (peer != NULL) {
        if (!parse_peer_key(peer, peer_buf, sizeof(peer_buf))) {
            return ESP_ERR_INVALID_ARG;
        }
        peer_given = true;
        peer = peer_buf;   /* "" clears the explicit override */
    }
    if (btn != NULL || peer_given || bta >= 0) {
        err = config_set_bluetooth(btn, peer_given ? peer : NULL, bta);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "apply bt failed: %s", esp_err_to_name(err));
        }
        return err;
    }
    return ESP_OK;
}

/* ============================================================
 * JSON command processing
 * ============================================================ */

/* Minimal key-value extraction from a flat JSON object (au_gw helper).
 * Returns true if @key was found; strings are copied unescaped. */
static bool json_get(const char *json, const char *key,
                     char *out_buf, size_t out_sz)
{
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (!p) return false;
    p += strlen(pat);
    while (*p == ' ' || *p == ':') p++;
    if (*p == '"') {
        p++;
        size_t i = 0;
        while (*p && *p != '"' && i < out_sz - 1) {
            out_buf[i++] = *p++;
        }
        out_buf[i] = '\0';
        return true;
    }
    size_t i = 0;
    while (*p && *p != ',' && *p != '}' && *p != '\n' && i < out_sz - 1) {
        out_buf[i++] = *p++;
    }
    out_buf[i] = '\0';
    return i > 0;
}

static void process_command(const char *json)
{
    char cmd[32] = {0};
    if (!json_get(json, "cmd", cmd, sizeof(cmd))) {
        send_notify("{\"ok\":false,\"err\":\"missing cmd\"}");
        return;
    }
    ESP_LOGI(TAG, "cmd: %s", cmd);

    if (strcmp(cmd, "get_config") == 0) {
        const system_config_t *c = config_get();
        char buf[512];
        snprintf(buf, sizeof(buf),
            "{\"ok\":true,\"cfg\":{"
            "\"fw\":\"%s\","
            "\"ssid\":\"%s\","
            "\"wpw\":\"%s\","
            "\"burl\":\"%s\","
            "\"tok\":\"%s\","
            "\"ssec\":%lu,"
            "\"pip\":\"%s\","
            "\"btn\":\"%s\","
            "\"peer\":\"%s\","
            "\"bta\":%d"
            "}}",
            esp_app_get_description()->version,
            c->wifi_ssid,
            c->wifi_password[0] ? SECRET_PLACEHOLDER : "",
            c->rtc_base_url,
            c->rtc_token[0] ? SECRET_PLACEHOLDER : "",
            (unsigned long)c->rtc_session_seconds,
            c->rtc_public_ip,
            c->bt_device_name,
            c->bt_peer_addr,
            c->bt_auto_answer ? 1 : 0);
        send_notify(buf);

    } else if (strcmp(cmd, "set_wifi") == 0) {
        char ssid[33] = {0}, pwd[65] = {0};
        json_get(json, "ssid", ssid, sizeof(ssid));
        json_get(json, "pwd",  pwd,  sizeof(pwd));
        if (ssid[0] == '\0') {
            send_notify("{\"ok\":false,\"err\":\"ssid required\"}");
            return;
        }
        const system_config_t *cur = config_get();
        const char *pass = keep_secret(pwd) ? cur->wifi_password : pwd;
        esp_err_t err = config_set_wifi(ssid, pass);
        send_notify(err == ESP_OK ? "{\"ok\":true}" :
                                    "{\"ok\":false,\"err\":\"nvs write failed\"}");

    } else if (strcmp(cmd, "set_rtc") == 0) {
        char burl[96] = {0}, tok[64] = {0}, ssec_s[12] = {0}, pip[16] = {0};
        json_get(json, "burl", burl,  sizeof(burl));
        json_get(json, "tok",  tok,   sizeof(tok));
        json_get(json, "ssec", ssec_s, sizeof(ssec_s));
        json_get(json, "pip",  pip,   sizeof(pip));
        if (pip[0] != '\0' && !valid_ipv4(pip)) {
            send_notify("{\"ok\":false,\"err\":\"invalid public ip\"}");
            return;
        }
        const char *token = keep_secret(tok) ? NULL : tok;
        int32_t ssec = ssec_s[0] ? (int32_t)atoi(ssec_s) : -1;
        esp_err_t err = config_set_rtc(burl[0] ? burl : NULL, token, ssec,
                                       pip[0] ? pip : NULL);
        send_notify(err == ESP_OK ? "{\"ok\":true}" :
                                    "{\"ok\":false,\"err\":\"nvs write failed\"}");

    } else if (strcmp(cmd, "set_bt") == 0) {
        char name[32] = {0}, peer_in[24] = {0}, bta_s[4] = {0}, peer[19];
        json_get(json, "name", name,  sizeof(name));
        json_get(json, "peer", peer_in, sizeof(peer_in));
        json_get(json, "bta",  bta_s, sizeof(bta_s));
        if (!parse_peer_key(peer_in, peer, sizeof(peer))) {
            send_notify("{\"ok\":false,\"err\":\"invalid peer address\"}");
            return;
        }
        int32_t bta = bta_s[0] ? (int32_t)atoi(bta_s) : -1;
        esp_err_t err = config_set_bluetooth(name[0] ? name : NULL,
                                            peer_in[0] ? peer : NULL, bta);
        send_notify(err == ESP_OK ? "{\"ok\":true}" :
                                    "{\"ok\":false,\"err\":\"nvs write failed\"}");

    } else if (strcmp(cmd, "get_ui") == 0) {
        cJSON *ui = build_ui();
        cJSON *vals = build_values();
        cJSON *root = (ui && vals) ? cJSON_CreateObject() : NULL;
        if (root == NULL) {
            cJSON_Delete(ui);
            cJSON_Delete(vals);
            send_notify("{\"ok\":false,\"err\":\"out of memory\"}");
            return;
        }
        cJSON_AddBoolToObject(root, "ok", true);
        cJSON_AddStringToObject(root, "fw", esp_app_get_description()->version);
        cJSON_AddItemToObject(root, "ui", ui);
        cJSON_AddItemToObject(root, "values", vals);
        cJSON_AddItemToObject(root, "contacts", cJSON_CreateObject());
        char *out = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);
        send_notify_owned(out);

    } else if (strcmp(cmd, "set_config") == 0) {
        /* One atomic submit; missing keys keep the stored value, an unchanged
         * secret arrives as ""; "frs":1 resets to defaults. Then reboot. */
        cJSON *root = cJSON_Parse(json);
        if (root == NULL) {
            send_notify("{\"ok\":false,\"err\":\"invalid json\"}");
            return;
        }
        const cJSON *values = cJSON_GetObjectItemCaseSensitive(root, "values");
        esp_err_t err = apply_values(values);
        cJSON_Delete(root);
        if (err == ESP_OK) {
            send_notify("{\"ok\":true}");
            vTaskDelay(pdMS_TO_TICKS(300));   /* let notify flush */
            xEventGroupSetBits(s_evt_group, BIT_REBOOT_REQUESTED);
        } else {
            char err_buf[64];
            snprintf(err_buf, sizeof(err_buf),
                     "{\"ok\":false,\"err\":\"%s\"}", esp_err_to_name(err));
            send_notify(err_buf);
        }

    } else if (strcmp(cmd, "factory_reset") == 0) {
        config_reset_defaults();
        send_notify("{\"ok\":true,\"reboot\":true}");
        vTaskDelay(pdMS_TO_TICKS(300));
        xEventGroupSetBits(s_evt_group, BIT_REBOOT_REQUESTED);

    } else if (strcmp(cmd, "reboot") == 0) {
        send_notify("{\"ok\":true}");
        vTaskDelay(pdMS_TO_TICKS(300));
        xEventGroupSetBits(s_evt_group, BIT_REBOOT_REQUESTED);

    } else {
        char err_buf[80];
        snprintf(err_buf, sizeof(err_buf), "{\"ok\":false,\"err\":\"unknown cmd: %s\"}", cmd);
        send_notify(err_buf);
    }
}

/* ============================================================
 * GATT server event handler
 * ============================================================ */

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        s_gatts_if = gatts_if;
        esp_ble_gatts_create_attr_tab(s_gatt_db, gatts_if, IDX_NB, 0);
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK ||
                param->add_attr_tab.num_handle != IDX_NB) {
            ESP_LOGE(TAG, "attr table create failed: status=%d handles=%d",
                     param->add_attr_tab.status, param->add_attr_tab.num_handle);
            break;
        }
        memcpy(s_handle_table, param->add_attr_tab.handles,
               sizeof(s_handle_table));
        esp_ble_gatts_start_service(s_handle_table[IDX_SVC]);
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "GATT service started, handle=%d", param->start.service_handle);
        if (s_started) {
            esp_ble_gap_start_advertising(&s_adv_params);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        s_conn_id = param->connect.conn_id;
        s_notify_enabled = false;
        s_rx_len = 0;
        ESP_LOGI(TAG, "BLE connected, conn_id=%d", s_conn_id);
        esp_ble_gatt_set_local_mtu(BLE_CFG_MTU_MAX);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "BLE disconnected, reason=0x%02x", param->disconnect.reason);
        s_conn_id = 0xFFFF;
        s_notify_enabled = false;
        s_rx_len = 0;
        if (s_started) {
            esp_ble_gap_start_advertising(&s_adv_params);
        }
        break;

    case ESP_GATTS_WRITE_EVT: {
        uint16_t handle = param->write.handle;
        if (handle == s_handle_table[IDX_CHAR_NOTIFY_CFG]) {
            if (param->write.len == 2) {
                s_notify_enabled = (param->write.value[0] == 0x01);
                ESP_LOGI(TAG, "Notify %s", s_notify_enabled ? "enabled" : "disabled");
            }
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
        } else if (handle == s_handle_table[IDX_CHAR_WRITE_VAL]) {
            const uint8_t *data = param->write.value;
            uint16_t       dlen = param->write.len;
            if (s_rx_len + dlen < (int)sizeof(s_rx_buf) - 1) {
                memcpy(s_rx_buf + s_rx_len, data, dlen);
                s_rx_len += dlen;
                s_rx_buf[s_rx_len] = '\0';
            } else {
                ESP_LOGW(TAG, "RX buffer overflow, discarding");
                s_rx_len = 0;
            }
            if (strchr(s_rx_buf, '\n') != NULL) {
                process_command(s_rx_buf);
                s_rx_len = 0;
            }
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }
        break;
    }

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "MTU negotiated: %d", param->mtu.mtu);
        break;

    default:
        break;
    }
}

/* ============================================================
 * GAP event handler
 * ============================================================ */

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed: %d", param->adv_start_cmpl.status);
        } else {
            ESP_LOGI(TAG, "BLE advertising started");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "BLE advertising stopped");
        break;
    default:
        break;
    }
}

/* ============================================================
 * Public API
 * ============================================================ */

esp_err_t ble_config_server_init(const char *device_name)
{
    s_evt_group = xEventGroupCreate();
    if (!s_evt_group) return ESP_ERR_NO_MEM;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* sdkconfig uses BTDM firmware; must enable with BTDM even though only
     * BLE GATT is used here. Classic BT remains idle during config mode. */
    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_register_callback(gatts_event_handler);

    /* Advertising name — the phone sees this when scanning */
    char adv_name[32];
    if (device_name && device_name[0]) {
        snprintf(adv_name, sizeof(adv_name), "%.20s-CFG", device_name);
    } else {
        strlcpy(adv_name, CONFIG_GW_BLE_ADV_NAME, sizeof(adv_name));
    }
    esp_ble_gap_set_device_name(adv_name);

    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp        = false,
        .include_name        = true,
        .include_txpower     = false,
        .min_interval        = 0x0006,
        .max_interval        = 0x0010,
        .appearance          = 0x00,
        .manufacturer_len    = 0,
        .p_manufacturer_data = NULL,
        .service_data_len    = 0,
        .p_service_data      = NULL,
        .service_uuid_len    = 0,
        .p_service_uuid      = NULL,
        .flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };
    esp_ble_gap_config_adv_data(&adv_data);

    ret = esp_ble_gatts_app_register(GATTS_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_ble_gatt_set_local_mtu(BLE_CFG_MTU_MAX);

    ESP_LOGI(TAG, "BLE config server initialized, name: %s", adv_name);
    return ESP_OK;
}

esp_err_t ble_config_server_start(void)
{
    s_started = true;
    /* Advertising starts automatically after the GATT service is up
     * (ESP_GATTS_START_EVT). If already registered, start now. */
    if (s_handle_table[IDX_SVC] != 0) {
        esp_ble_gap_start_advertising(&s_adv_params);
    }
    return ESP_OK;
}

void ble_config_server_stop(void)
{
    s_started = false;
    esp_ble_gap_stop_advertising();
    if (s_conn_id != 0xFFFF) {
        esp_ble_gatts_close(s_gatts_if, s_conn_id);
        s_conn_id = 0xFFFF;
    }
}

bool ble_config_server_is_connected(void)
{
    return s_conn_id != 0xFFFF;
}

bool ble_config_server_wait_for_reboot(uint32_t timeout_ms)
{
    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY
                                         : pdMS_TO_TICKS(timeout_ms);
    EventBits_t bits = xEventGroupWaitBits(s_evt_group,
                                           BIT_REBOOT_REQUESTED,
                                           pdFALSE, pdTRUE, ticks);
    return (bits & BIT_REBOOT_REQUESTED) != 0;
}
