// ESP8684 (ESP32-C2 family) wired to the ESP32-P4 like Espressif's reference design: UART0 + EN + BOOT (GPIO9).
// The P4 can therefore flash it with esp-serial-flasher (same as the "slave OTA" of ESP-Hosted).
// Step 1 (c2_probe) is read-only: it resets the C2 into its ROM download mode, reads chip id / MAC / flash size.
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "esp_loader.h"
#include "esp32_port.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void cdc_log(const char *fmt, ...);

#define C2_UART      UART_NUM_1
#define PIN_C2_RX    GPIO_NUM_19   // net TX_ESP8684 (C2 TXD0 -> P4)
#define PIN_C2_TX    GPIO_NUM_20   // net RX_ESP8684 (P4 -> C2 RXD0)
#define PIN_C2_EN    GPIO_NUM_32   // EN_ESP8684 (C2 CHIP_EN)
#define PIN_C2_BOOT  GPIO_NUM_28   // BOOT_ESP8684 (C2 GPIO9)

static esp32_port_t s_port = {
    .port.ops = &esp32_uart_ops,
    .baud_rate = 115200,
    .uart_port = C2_UART,
    .uart_rx_pin = PIN_C2_RX,
    .uart_tx_pin = PIN_C2_TX,
    .reset_pin = PIN_C2_EN,
    .boot_pin = PIN_C2_BOOT,
};
static esp_loader_t s_loader;

static const char *chip_name(target_chip_t t)
{
    switch (t) {
    case ESP32C2_CHIP: return "ESP32-C2 / ESP8684";
    case ESP32C3_CHIP: return "ESP32-C3";
    case ESP32C6_CHIP: return "ESP32-C6";
    default: return "other";
    }
}

// Diagnostic: reset the C2 normally / into download mode and print what it says on its UART (ROM boot log).
static void c2_listen(int boot_low, int swap)
{
    gpio_config_t o = { .pin_bit_mask = (1ULL << PIN_C2_EN) | (1ULL << PIN_C2_BOOT), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&o);
    uart_config_t uc = { .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                         .stop_bits = UART_STOP_BITS_1, .source_clk = UART_SCLK_DEFAULT };
    uart_driver_install(C2_UART, 512, 0, 0, NULL, 0);
    uart_param_config(C2_UART, &uc);
    uart_set_pin(C2_UART, swap ? PIN_C2_RX : PIN_C2_TX, swap ? PIN_C2_TX : PIN_C2_RX, -1, -1);
    gpio_set_level(PIN_C2_BOOT, boot_low ? 0 : 1);
    gpio_set_level(PIN_C2_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_flush_input(C2_UART);
    gpio_set_level(PIN_C2_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t b[160];
    int n = uart_read_bytes(C2_UART, b, sizeof(b) - 1, pdMS_TO_TICKS(800));
    if (n < 0) n = 0;
    for (int i = 0; i < n; i++) if (b[i] < 32 || b[i] > 126) b[i] = '.';
    b[n] = 0;
    cdc_log("c2 listen (boot %s, %s): %d bytes: %s", boot_low ? "LOW" : "high", swap ? "swapped pins" : "normal pins", n, (char *)b);
    uart_driver_delete(C2_UART);
}

// Diagnostic: hand-made SLIP "sync" request, to see whether the C2 answers (tests the P4 -> C2 wire too).
static void c2_sync(int baud, int boot_low)
{
    gpio_config_t o = { .pin_bit_mask = (1ULL << PIN_C2_EN) | (1ULL << PIN_C2_BOOT), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&o);
    uart_config_t uc = { .baud_rate = baud, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                         .stop_bits = UART_STOP_BITS_1, .source_clk = UART_SCLK_DEFAULT };
    uart_driver_install(C2_UART, 512, 512, 0, NULL, 0);
    uart_param_config(C2_UART, &uc);
    uart_set_pin(C2_UART, PIN_C2_TX, PIN_C2_RX, -1, -1);
    gpio_set_level(PIN_C2_BOOT, boot_low ? 0 : 1);
    gpio_set_level(PIN_C2_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_C2_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    uart_flush_input(C2_UART);
    uint8_t req[8 + 36 + 2] = { 0xC0, 0x00, 0x08, 0x24, 0, 0, 0, 0, 0, 0x07, 0x07, 0x12, 0x20 };
    for (int i = 13; i < 45; i++) req[i] = 0x55;
    req[45] = 0xC0;
    int total = 0;
    uint8_t b[64];
    char hex[3 * 24 + 1] = "";
    for (int t = 0; t < 5; t++) {
        uart_write_bytes(C2_UART, req, 46);
        int n = uart_read_bytes(C2_UART, b, sizeof(b), pdMS_TO_TICKS(100));
        if (n > 0) {
            total += n;
            for (int i = 0; i < n && i < 24; i++) sprintf(hex + 3 * i, "%02X ", b[i]);
            break;
        }
    }
    cdc_log("c2 sync @%d boot %s: %d bytes %s", baud, boot_low ? "LOW" : "high", total, hex);
    uart_driver_delete(C2_UART);
}

void c2_probe(void)
{
    c2_listen(0, 0);
    c2_sync(115200, 1);
    c2_sync(115200, 0);
    cdc_log("c2: connecting to the ESP8684 (UART%d, RX=%d TX=%d EN=%d BOOT=%d)", C2_UART, PIN_C2_RX, PIN_C2_TX, PIN_C2_EN, PIN_C2_BOOT);
    esp_loader_error_t e = esp_loader_init_serial(&s_loader, &s_port.port);
    if (e != ESP_LOADER_SUCCESS) { cdc_log("c2: init failed (%d)", e); return; }
    esp_loader_connect_args_t args = ESP_LOADER_CONNECT_DEFAULT();
    e = esp_loader_connect(&s_loader, &args);
    if (e != ESP_LOADER_SUCCESS) {
        cdc_log("c2: no answer from the ROM bootloader (error %d)", e);
        esp_loader_deinit(&s_loader);
        return;
    }
    target_chip_t t = esp_loader_get_target(&s_loader);
    uint16_t rev = 0;
    uint8_t mac[6] = {0};
    uint32_t fsz = 0;
    esp_loader_get_chip_revision(&s_loader, &rev);
    esp_loader_read_mac(&s_loader, mac);
    esp_loader_flash_detect_size(&s_loader, &fsz);
    cdc_log("c2: CONNECTED: %s rev %d, MAC %02X:%02X:%02X:%02X:%02X:%02X, flash %u KB", chip_name(t), rev,
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], (unsigned)(fsz / 1024));
    esp_loader_deinit(&s_loader);       // releases the UART; the C2 stays in download mode until it is reset
}
