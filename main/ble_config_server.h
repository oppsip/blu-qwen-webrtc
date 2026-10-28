#ifndef BLE_CONFIG_SERVER_H
#define BLE_CONFIG_SERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * Bring up controller + Bluedroid + the GATT config service (UUID FFF0) and
 * start advertising "<device_name>-CFG" (or GW_BLE_ADV_NAME when device_name
 * is empty). Classic BT stays idle while the config session runs.
 */
esp_err_t ble_config_server_init(const char *device_name);

esp_err_t ble_config_server_start(void);
void ble_config_server_stop(void);
bool ble_config_server_is_connected(void);

/** Wait for a set_config/factory_reset/reboot from the phone.
 *  timeout_ms == 0 waits forever. Returns true if a reboot was requested. */
bool ble_config_server_wait_for_reboot(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BLE_CONFIG_SERVER_H */
