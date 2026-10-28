/*
 * Minimal LAN8720 RMII Ethernet manager for the wired-network audio test.
 * GPIO map mirrors repo/firmware/main/ethernet_manager.c ( production ):
 *   PHY addr 1, MDC 23, MDIO 18, reset 5, ext 50 MHz RMII clock -> GPIO0,
 *   GPIO4 high selects the external clock input.
 */
#pragma once

#include <stdbool.h>
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void (*on_connected)(void);
    void (*on_disconnected)(void);
} eth_test_callbacks_t;

esp_err_t eth_test_init(const eth_test_callbacks_t *cb);
bool eth_test_is_connected(void);

#ifdef __cplusplus
}
#endif
