# blu-qwen-webrtc

ESP32 (WROVER-E) gateway: Bluetooth HFP ⇄ Qwen Audio Agent over WebRTC.
Plan: `/home/liyc/.opencode/plan/webrtc-migration.md`.

## Status

- **P0 feasibility — passed** (`main/main.c` is a probe, not product code).
- P1 server verification: not started.
- P2 firmware transport: not started.

## P0 results (ESP32-WROVER-E, ESP-IDF v6.1, esp_peer 1.5.7, esp_audio_codec 2.6.2)

| Stage | Internal DRAM free | Notes |
| --- | --- | --- |
| Boot (no BT, no WiFi) | 233 KB | |
| Opus encoder open (16 kHz mono, 20 ms) | −43 KB | encode 20 ms frame ≈ 6–9 ms (cold start) |
| WiFi STA connected | — | |
| BT controller (BTDM) | 123 KB | |
| Bluedroid + HFP client | 105 KB | |
| `esp_peer_open` + local SDP (947 B) | 92 KB | |
| Steady state (2 min) | **97 KB free, 80 KB min** | PSRAM ~3.6 MB free |

Exit criterion (≥ 40 KB internal margin with BT HFP + WiFi + esp_peer): **met**.

Not yet measured: Opus encoder and decoder open at the same time, ICE/DTLS
with a real peer, CPU load under WiFi + SCO traffic.

## Findings that shaped the configuration

1. `esp_peer` 1.5.7 needs `CONFIG_MBEDTLS_X509_CREATE_C=y` on IDF 6.x, otherwise
   `dtls_srtp_v6.c` does not build.
2. Opus encoder needs ~40 KB of task stack (esp_audio_codec docs). Running it on
   the main task (3.5 KB) corrupts the FreeRTOS critical-section nesting.
3. `esp_peer_open` must not run in the system event task (~2 KB stack).
4. Classic BT + WiFi + esp_peer does not fit in internal DRAM with defaults
   (`esp_peer_open` returned −2 with 816 B minimum). Fixed by:
   - `CONFIG_SPIRAM_USE_MALLOC=y`, `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096`
   - `CONFIG_BT_ALLOCATION_FROM_SPIRAM_FIRST=y`
   - disabling HFP AG, A2DP and AVRCP (production uses the HFP client role).
5. Image with BT + WiFi + esp_peer + Opus is ~2.1 MB; the default factory
   partitions are too small, hence `partitions.csv`.

## Build and flash

```sh
source ~/.espressif/v6.1/esp-idf/export.sh
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

WiFi credentials are not committed. `sdkconfig` is generated and git-ignored;
set `CONFIG_GW_WIFI_SSID` and `CONFIG_GW_WIFI_PASSWORD` there (or via menuconfig,
"P0 Network (test)").
