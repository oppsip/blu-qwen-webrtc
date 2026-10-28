/*
 * blu-qwen-webrtc P3: runtime configuration + WiFi/ethernet + classic BT
 * (HFP client) + WebRTC session.
 *
 * Boot flow: NVS -> config_manager -> config button (GPIO13) -> BLE config
 * mode (au_gw-compatible, minimi program/F schema, reboots after saving)
 * or normal operation:
 *   network (Ethernet attempted first, WiFi fallback when configured)
 *     -> BT (HFP) -> WebRTC session cycles per incoming call.
 * SCO phone audio is bridged both ways via sco_bridge: mic feeds the Opus
 * uplink (silence when no call), decoded downlink feeds the SCO out ring.
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_coexist.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"

#include "config_manager.h"
#include "ble_config_server.h"
#include "sco_bridge.h"
#include "psram_task.h"
#include "webrtc_session.h"

#ifdef CONFIG_GW_NET_ETHERNET
#include "ethernet_manager.h"
#endif

static const char *TAG = "main";

static bool s_wifi_started;
static bool s_wifi_up;
static bool s_bt_up;

static void log_heap(const char *stage)
{
    ESP_LOGI(TAG, "[heap %s] internal free=%u min=%u | psram free=%u",
             stage,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
}

/* ============================ BLE config mode ============================ */

#define CFG_TRIGGER_GPIO   CONFIG_GW_CFG_TRIGGER_GPIO
#define CFG_BOOT_DEBOUNCE_MS 200

static void config_button_gpio_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CFG_TRIGGER_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Config button GPIO%d init failed: %s",
                 CFG_TRIGGER_GPIO, esp_err_to_name(err));
    }
}

/* au_gw style: the pin read low continuously for CFG_BOOT_DEBOUNCE_MS right
 * after power-on enters the BLE configuration mode. */
static bool check_config_trigger(void)
{
    if (gpio_get_level(CFG_TRIGGER_GPIO) != 0) {
        return false;   /* not pressed at power-on */
    }
    int elapsed = 0;
    while (elapsed < CFG_BOOT_DEBOUNCE_MS) {
        vTaskDelay(pdMS_TO_TICKS(20));
        elapsed += 20;
        if (gpio_get_level(CFG_TRIGGER_GPIO) != 0) {
            ESP_LOGI(TAG, "Config button bounce at boot, ignoring");
            return false;
        }
    }
    ESP_LOGI(TAG, "Config button pressed at boot — entering BLE config mode");
    return true;
}

static bool s_ble_config_attempted;   /* BLE controller already in use this boot */

static void run_ble_config_mode(const char *bt_name)
{
    s_ble_config_attempted = true;
    ESP_LOGI(TAG, "=== BLE Config Mode ===");
    ESP_LOGI(TAG, "Scan for \"%s%s\" (BLE FFF0 service) and send get_ui/set_config",
             bt_name[0] ? bt_name : CONFIG_GW_BLE_ADV_NAME,
             bt_name[0] ? "-CFG" : "");
    ESP_LOGI(TAG, "Waiting until the configuration is saved (no timeout)");

    esp_err_t ret = ble_config_server_init(bt_name);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE config server init failed: %s", esp_err_to_name(ret));
        return;   /* fall through to the normal boot */
    }
    ble_config_server_start();

    while (!ble_config_server_wait_for_reboot(0)) {
        ESP_LOGW(TAG, "BLE wait returned without reboot request, waiting again");
    }

    ble_config_server_stop();
    config_mark_configured();
    ESP_LOGI(TAG, "Config saved — rebooting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

/* ============================ WiFi ============================ */

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got IP " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_up = true;
    }
}

static void wifi_start(void)
{
    if (s_wifi_started) {
        return;
    }
    /* esp_netif_init/event loop may already exist in the Ethernet build. */
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        on_wifi_event, NULL, NULL));

    wifi_config_t wifi_cfg = {0};
    const system_config_t *cfg = config_get();
    strlcpy((char *)wifi_cfg.sta.ssid, cfg->wifi_ssid, sizeof(wifi_cfg.sta.ssid));
    strlcpy((char *)wifi_cfg.sta.password, cfg->wifi_password, sizeof(wifi_cfg.sta.password));
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    s_wifi_started = true;
    /*
     * Realtime audio + SCO: default modem sleep wakes RX every ~3 beacons
     * (~300 ms), queuing ~10 RTP frames as one burst. The SPK ring gets
     * [0..220ms] swings and the pump pads ~30% silence mid-call. Keep the
     * radio awake for a streaming device.
     */
    esp_wifi_set_ps(WIFI_PS_NONE);
    /* Coex arbitration experiments (2026-10-28, both reverted): WiFi-
     * favoring and BALANCE preferences corrupt the eSCO TX air path — the
     * phone plays noise once WebRTC traffic ramps. Default BT-favoring
     * arbitration is mandatory; the WiFi downlink loss (~32/50 packets/s,
     * downlink-only, uplink clean) is structural on ESP32 classic. */
    ESP_LOGI(TAG, "WiFi started, SSID \"%s\"", cfg->wifi_ssid);
}

/* ============================ classic BT ============================ */

static void bt_start(void)
{
    /* BLE is only used by the config session (separate boot mode that ends
     * in a reboot). In normal operation run the controller classic-only and
     * hand its BLE memory back to the heap; this is permanent until reboot. */
    esp_bt_mode_t mode = ESP_BT_MODE_CLASSIC_BT;
    if (s_ble_config_attempted) {
        mode = ESP_BT_MODE_BTDM;   /* config init failed midway: keep the old behaviour */
    } else {
        ESP_ERROR_CHECK(esp_bt_mem_release(ESP_BT_MODE_BLE));
        log_heap("ble mem released");
    }
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = mode;   /* enable() must use the mode init() was given */
    if (mode == ESP_BT_MODE_CLASSIC_BT) {
        bt_cfg.ble_max_conn = 0;
    }
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(mode));
    log_heap("bt ctrl");
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    log_heap("bluedroid");
    /* Registers HFP event + data callbacks (SCO bridge), then esp_hf_client_init. */
    sco_bridge_init();
    log_heap("hfp client");
    s_bt_up = true;
}

/* ============================ network ============================ */

static bool wifi_configured(void)
{
    const system_config_t *cfg = config_get();
    return cfg->wifi_ssid[0] != '\0' && cfg->wifi_password[0] != '\0';
}

static void network_init(void)
{
#ifdef CONFIG_GW_NET_ETHERNET
    static const eth_test_callbacks_t eth_cb = {
        .on_connected = NULL,      /* eth_test_is_connected() polled below */
        .on_disconnected = NULL,
    };
    ESP_ERROR_CHECK(eth_test_init(&eth_cb));
#else
    wifi_start();
#endif
}

static bool network_is_up(void)
{
    bool up = false;
#ifdef CONFIG_GW_NET_ETHERNET
    up = eth_test_is_connected();
#endif
    return up || s_wifi_up;
}

/* Network-ready signal: the monitor task sets it whenever any interface has
 * an IP; app_main waits on it (with timeout) before bringing up BT. */
static EventGroupHandle_t s_net_events;
#define NET_UP_BIT  BIT0

/* Ethernet-first watchdog: at boot (and at runtime) if the wired link has no
 * IP for GW_WIFI_FALLBACK_SEC and a WiFi SSID is configured, start WiFi.
 * Also publishes NET_UP_BIT and periodic WiFi RSSI (link-quality diagnostics). */
static void network_monitor_task(void *arg)
{
    (void)arg;
    bool wifi_fallback_started = false;
    int seconds_down = 0;
    int rssi_tick = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
#ifdef CONFIG_GW_NET_ETHERNET
        if (eth_test_is_connected()) {
            seconds_down = 0;
        } else {
            seconds_down++;
        }
        if (!wifi_fallback_started && seconds_down >= CONFIG_GW_WIFI_FALLBACK_SEC &&
                wifi_configured()) {
            ESP_LOGW(TAG, "Ethernet not ready for %d s, falling back to WiFi",
                     CONFIG_GW_WIFI_FALLBACK_SEC);
            wifi_start();
            wifi_fallback_started = true;
        }
#endif
        if (network_is_up()) {
            xEventGroupSetBits(s_net_events, NET_UP_BIT);
        }
        if (s_wifi_up) {
            if (++rssi_tick >= 10) {
                rssi_tick = 0;
                wifi_ap_record_t ap;
                if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                    ESP_LOGI(TAG, "wifi rssi=%d channel=%d", ap.rssi, ap.primary);
                }
            }
        } else {
            rssi_tick = 0;
        }
    }
}

/* ============================ session cycles ============================ */

static void session_cycle(void)
{
    static webrtc_session_t session;
    memset(&session, 0, sizeof(session));
    const system_config_t *cfg = config_get();
    snprintf(session.base_url, sizeof(session.base_url), "%s", cfg->rtc_base_url);
    snprintf(session.auth_token, sizeof(session.auth_token), "%s", cfg->rtc_token);
    snprintf(session.session_id, sizeof(session.session_id), "main");
    session.run_seconds = cfg->rtc_session_seconds;
    webrtc_session_set_public_ip(cfg->rtc_public_ip);

    esp_err_t err = webrtc_session_run(&session);
    ESP_LOGI(TAG, "session finished err=%d state=%d up=%lu down=%lu ctrl=%lu",
             (int)err, (int)session.state,
             (unsigned long)session.uplink_frames,
             (unsigned long)session.downlink_frames,
             (unsigned long)session.control_rx_events);
}

static void stats_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        log_heap("periodic");
    }
}

/* Opus and esp_peer need deep stacks; the main task (3.5 KB) cannot run them. */
static void session_task(void *arg)
{
    (void)arg;
    int cycle = 0;
    while (1) {
        /* Voice channel only while a call is ringing/active. */
        if (!sco_bridge_call_wanted()) {
            ESP_LOGI(TAG, "idle, waiting for a call");
            sco_bridge_wait_call(UINT32_MAX);
            if (!sco_bridge_call_wanted()) {
                continue;
            }
        }
        cycle++;
        ESP_LOGI(TAG, "=== session cycle %d (call) ===", cycle);
        log_heap("pre-session");
        session_cycle();
        log_heap("post-session");
        vTaskDelay(pdMS_TO_TICKS(sco_bridge_call_wanted() ? 2000 : 500));   /* retry fast if the call is still up */
    }
}

/* ============================ entry ============================ */

void app_main(void)
{
    /* The peer lib prints one BUF_MNGR line per packet when a backlog burst
     * lands (50+ lines = ~100 ms of blocking UART@115200), stalling the
     * audio path right when the downlink needs to be drained. */
    esp_log_level_set("BUF_MNGR", ESP_LOG_NONE);
    esp_log_level_set("AGENT", ESP_LOG_WARN);
    ESP_LOGI(TAG, "blu-qwen-webrtc P3 start");
    log_heap("boot");
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_ERROR_CHECK(config_manager_init());

    /* BLE config session (au_gw style, button at boot). Never returns when
     * a config is saved — the path above reboots the device. */
    config_button_gpio_init();
    if (check_config_trigger()) {
        const system_config_t *cfg = config_get();
        run_ble_config_mode(cfg->bt_device_name);
    }

    network_init();
    s_net_events = xEventGroupCreate();
    xTaskCreate(network_monitor_task, "netmon", 3072, NULL, 3, NULL);
    /* Wait for any interface to get an IP; after 30 s continue anyway so BT
     * (HFP) still comes up and WebRTC retries once the network returns. */
    EventBits_t net_bits = xEventGroupWaitBits(s_net_events, NET_UP_BIT,
                                               pdFALSE, pdTRUE,
                                               pdMS_TO_TICKS(30000));
    if (!(net_bits & NET_UP_BIT)) {
        ESP_LOGW(TAG, "No network after 30 s — starting BT anyway, WebRTC retries later");
    }
    xTaskCreate(network_monitor_task, "netmon", 3072, NULL, 3, NULL);

    bt_start();
    log_heap("bt ready");

    xTaskCreate(stats_task, "stats", 3072, NULL, 1, NULL);
    /* Pin to CPU1 so wifi/bluedroid/tcpip keep CPU0 (see peer_loop note). */
    {
        static StackType_t *s_session_stack;
        static StaticTask_t s_session_tcb;
        if (psram_task_create(session_task, "session", 40960, NULL, 5, 1,
                              &s_session_stack, &s_session_tcb) == NULL) {
            ESP_LOGE(TAG, "session task create failed");
        }
    }
}
