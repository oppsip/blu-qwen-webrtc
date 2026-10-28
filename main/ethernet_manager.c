/* Minimal LAN8720 RMII bring-up (DHCP only) for the wired-network test.
 * Ported from production repo/firmware/main/ethernet_manager.c. */
#include "ethernet_manager.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_eth.h"
#include "esp_eth_driver.h"
#include "esp_eth_mac.h"
#include "esp_eth_mac_esp.h"
#include "esp_eth_netif_glue.h"
#include "esp_eth_phy.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "eth";

#define ETH_PHY_ADDR        1
#define ETH_MDC_GPIO        23
#define ETH_MDIO_GPIO       18
#define ETH_PHY_RESET_GPIO  5
#define ETH_CLK_SELECT_GPIO 4
#define ETH_REF_CLK_GPIO    0

static bool s_link_up;
static bool s_has_ip;
static bool s_connected;
static const eth_test_callbacks_t *s_cb;
static esp_eth_handle_t s_eth_handle;
static esp_eth_netif_glue_handle_t s_eth_glue;

static void update_connected(void)
{
    bool now = s_link_up && s_has_ip;
    if (now == s_connected) {
        return;
    }
    s_connected = now;
    if (now) {
        if (s_cb && s_cb->on_connected) {
            s_cb->on_connected();
        }
    } else {
        ESP_LOGW(TAG, "Ethernet lost IP/link");
        if (s_cb && s_cb->on_disconnected) {
            s_cb->on_disconnected();
        }
    }
}

static void select_rmii_clock_input(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = BIT64(ETH_CLK_SELECT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(ETH_CLK_SELECT_GPIO, 1));
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void release_phy_reset_or_power(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = BIT64(ETH_PHY_RESET_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(ETH_PHY_RESET_GPIO, 1));
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void eth_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    esp_eth_handle_t h = *(esp_eth_handle_t *)data;
    switch (id) {
    case ETHERNET_EVENT_CONNECTED: {
        uint8_t mac[6] = {0};
        esp_eth_ioctl(h, ETH_CMD_G_MAC_ADDR, mac);
        ESP_LOGI(TAG, "Link up HW %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        s_link_up = true;
        update_connected();
        break;
    }
    case ETHERNET_EVENT_DISCONNECTED:
        s_link_up = false;
        update_connected();
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Started (LAN8720 phy_addr=%d)", ETH_PHY_ADDR);
        break;
    case ETHERNET_EVENT_STOP:
        s_link_up = false;
        update_connected();
        break;
    default:
        break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    if (id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        s_has_ip = true;
        ESP_LOGI(TAG, "got IP " IPSTR " mask " IPSTR " gw " IPSTR " via ETHERNET",
                 IP2STR(&ev->ip_info.ip), IP2STR(&ev->ip_info.netmask), IP2STR(&ev->ip_info.gw));
        update_connected();
    } else if (id == IP_EVENT_ETH_LOST_IP) {
        s_has_ip = false;
        update_connected();
    }
}

esp_err_t eth_test_init(const eth_test_callbacks_t *cb)
{
    s_cb = cb;

    select_rmii_clock_input();
    release_phy_reset_or_power();

    /* lwIP + default loop must exist before esp_netif_new(); the WiFi path
     * normally does this, but in wired mode nothing else does. */
    esp_err_t netif_err = esp_netif_init();
    if (netif_err != ESP_OK && netif_err != ESP_ERR_ESP_NETIF_INIT_FAILED) {
        return netif_err;
    }
    esp_err_t loop_err = esp_event_loop_create_default();
    if (loop_err != ESP_OK && loop_err != ESP_ERR_INVALID_STATE) {
        return loop_err;
    }

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_cfg.smi_gpio.mdc_num = ETH_MDC_GPIO;
    emac_cfg.smi_gpio.mdio_num = ETH_MDIO_GPIO;
    emac_cfg.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_cfg.clock_config.rmii.clock_gpio = ETH_REF_CLK_GPIO;
    phy_cfg.phy_addr = ETH_PHY_ADDR;
    phy_cfg.reset_gpio_num = ETH_PHY_RESET_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);
    if (mac == NULL) {
        ESP_LOGE(TAG, "create EMAC failed");
        return ESP_FAIL;
    }
    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_cfg);
    if (phy == NULL) {
        ESP_LOGE(TAG, "create LAN8720 PHY failed");
        mac->del(mac);
        return ESP_FAIL;
    }
    esp_eth_config_t cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_err_t err = esp_eth_driver_install(&cfg, &s_eth_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "install eth driver failed: %s", esp_err_to_name(err));
        mac->del(mac);
        phy->del(phy);
        return err;
    }

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *netif = esp_netif_new(&netif_cfg);
    if (netif == NULL) {
        ESP_LOGE(TAG, "create eth netif failed");
        return ESP_FAIL;
    }
    s_eth_glue = esp_eth_new_netif_glue(s_eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(netif, s_eth_glue));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_LOST_IP, ip_event_handler, NULL));

    return esp_eth_start(s_eth_handle);
}

bool eth_test_is_connected(void)
{
    return s_connected;
}
