// ESP8684 (ESP32-C2 family) wired to the ESP32-P4 like Espressif's reference design: UART0 + EN + BOOT (GPIO9).
// The P4 can therefore flash it with esp-serial-flasher (same as the "slave OTA" of ESP-Hosted).
// Step 1 (c2_probe) is read-only: it resets the C2 into its ROM download mode, reads chip id / MAC / flash size.
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "driver/ledc.h"
#include "esp_loader.h"
#include "esp32_port.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

void cdc_log(const char *fmt, ...);
void encoder_zero(void);
extern volatile int g_unit_inch;

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

#define PIN_BUZZ     GPIO_NUM_43
#define PIN_PWR_OFF  GPIO_NUM_17   // PWR_OFF: held HIGH to keep the board powered on battery; driven LOW the board cuts its own supply
#define PWR_HOLD     1
#define C2_ENABLE    1             // 0 = keep the ESP8684 in reset (no Wi-Fi / BLE radio): test whether the radio bursts disturb the display
#define POWER_PINS   1             // 0 = firmware never touches PWR_OFF (GPIO17) / PWR_ON (GPIO18): battery test, the board does nothing with them

// Passive buzzer (GPIO43 -> 1k -> 2N7002 gate, buzzer between 3.3 V and the drain): it only sounds when driven with a
// square wave, so notes are LEDC PWM at the note frequency, 50 % duty.
typedef struct { uint16_t hz; uint16_t ms; } note_t;   // hz 0 = rest

static SemaphoreHandle_t s_buzz_mtx;   // one tune at a time (melody from the link task, switch sounds from the inputs task)

static void buzzer_init(void)
{
    static int done;
    if (done) return;   // called both from power_boot() (early) and c2_run()
    done = 1;
    s_buzz_mtx = xSemaphoreCreateMutex();
    ledc_timer_config_t t = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT, .timer_num = LEDC_TIMER_0,
                              .freq_hz = 2000, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    ledc_channel_config_t c = { .gpio_num = PIN_BUZZ, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
                                .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 };
    ledc_channel_config(&c);
}

static void buzzer_tone(unsigned hz, unsigned ms)
{
    if (hz) {
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, hz);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    vTaskDelay(pdMS_TO_TICKS(ms));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(30));   // short gap so repeated notes are distinct
}

static void buzzer_play(const note_t *m, int n)
{
    xSemaphoreTake(s_buzz_mtx, portMAX_DELAY);
    for (int i = 0; i < n; i++) buzzer_tone(m[i].hz, m[i].ms);
    xSemaphoreGive(s_buzz_mtx);
}

enum { C5 = 523, D5 = 587, E5 = 659, F5 = 698, G5 = 784 };

// Ode to Joy (first phrase), played on each press of the web page button.
static const note_t s_bye[] = { {G5,120},{E5,120},{C5,240} };   // BLE disconnect: short falling arpeggio
// Themed tunes: power-on rises, power-off falls.
static const note_t s_snd_on[]     = { {C5,90},{E5,90},{G5,90},{1047,260} };
static const note_t s_snd_off[]    = { {1047,90},{G5,90},{E5,90},{C5,320} };
static const note_t s_melody[] = {
    {E5,250},{E5,250},{F5,250},{G5,250},{G5,250},{F5,250},{E5,250},{D5,250},
    {C5,250},{C5,250},{D5,250},{E5,250},{E5,375},{D5,125},{D5,500},
};

// Lines sent by the C2 firmware on its UART0 (115200): "$BUZZ" = a press on the web page button. Anything else (ROM boot log) is ignored.
static void c2_link_task(void *arg)
{
    char line[32];
    int n = 0;
    uart_flush_input(C2_UART);
    for (;;) {
        uint8_t ch;
        if (uart_read_bytes(C2_UART, &ch, 1, pdMS_TO_TICKS(100)) != 1) continue;
        if (ch == '\n') {
            line[n] = 0;
            for (int i = 0; i < n; i++) if (line[i] < 32 || line[i] > 126) line[i] = '.';
            if (n) cdc_log("c2 rx: %s", line);
            if (strstr(line, "$OFF")) {
                cdc_log("c2: power off requested (PWR_OFF GPIO%d low)", PIN_PWR_OFF);
                buzzer_play(s_snd_off, sizeof(s_snd_off) / sizeof(s_snd_off[0]));
                if (POWER_PINS) {
                    gpio_set_level(PIN_PWR_OFF, !PWR_HOLD);     // release the latch
                    vTaskDelay(pdMS_TO_TICKS(2000));            // on battery the board is dead by now; on USB it stays alive
                    gpio_set_level(PIN_PWR_OFF, PWR_HOLD);      // still alive (USB): hold the latch again
                }
                uart_flush_input(C2_UART);
            } else if (strstr(line, "$DISC")) {
                cdc_log("c2: BLE disconnected");
                buzzer_play(s_bye, sizeof(s_bye) / sizeof(s_bye[0]));
                uart_flush_input(C2_UART);
            } else if (strstr(line, "$CONN") || strstr(line, "$BUZZ")) {   // BLE connected, or web button
                cdc_log("c2: %s", strstr(line, "$CONN") ? "BLE connected" : "buzzer");
                buzzer_play(s_melody, sizeof(s_melody) / sizeof(s_melody[0]));
                uart_flush_input(C2_UART);   // presses made while the melody plays are dropped
            }
            n = 0;
        } else if (n < (int)sizeof(line) - 1) {
            line[n++] = (char)ch;
        }
    }
}

#define PIN_MAIN_SW  GPIO_NUM_0    // main switch: no external pull-up -> internal pull-up, pressed = 0
#define PIN_PWR_ON   GPIO_NUM_18   // "power on" switch: read only, never driven; idles low (external pull-down), goes high when pressed

// Main switch: a sound on each press (not on release). Power switch: no sound on press; held 3 s = power off (see below).
static const note_t s_snd_unit[] = { {1200,60} };                        // short press of the power switch: mm <-> inch
static const note_t s_snd_main[] = { {1000,70},{1000,70} };           // main switch: two short beeps

// Main switch (GPIO0): idle high through the internal pull-up, pressed = low. Power-on switch (GPIO18): idle low, pressed = high.
// Both are debounced over two 20 ms samples; the first stable reading after boot is not treated as a press.
static void c2_inputs_task(void *arg)
{
    gpio_config_t in_m = { .pin_bit_mask = 1ULL << PIN_MAIN_SW, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config_t in_p = { .pin_bit_mask = 1ULL << PIN_PWR_ON, .mode = GPIO_MODE_INPUT };   // no internal pull: the board already has its own pull-down
    gpio_config(&in_m);
    if (POWER_PINS) gpio_config(&in_p);
    int prev_m = -1, prev_p = -1, raw_m = -1, raw_p = -1, armed = 0, held_ms = 0, press_ok = 0;
    for (;;) {
        int m = gpio_get_level(PIN_MAIN_SW) == 0;   // 1 = pressed
        int p = POWER_PINS ? gpio_get_level(PIN_PWR_ON) == 1 : 0;    // 1 = pressed
        if (m == raw_m && m != prev_m) {
            if (prev_m >= 0) cdc_log("inputs: main switch (GPIO0) %s", m ? "pressed" : "released");
            if (prev_m >= 0 && m) encoder_zero();   // main switch press: zero the measurement
            if (prev_m >= 0 && m) buzzer_play(s_snd_main, sizeof(s_snd_main) / sizeof(s_snd_main[0]));
            prev_m = m;
        }
        if (p == raw_p && p != prev_p) {
            if (prev_p >= 0) cdc_log("inputs: power-on switch (GPIO18) %s", p ? "pressed" : "released");
            if (p) {
                press_ok = armed;       // the press that switched the board on means nothing
            } else {
                if (POWER_PINS && press_ok && held_ms < 3000) {        // short press: millimetres <-> inches
                    g_unit_inch = !g_unit_inch;
                    cdc_log("units: %s", g_unit_inch ? "inch" : "mm");
                    buzzer_play(s_snd_unit, sizeof(s_snd_unit) / sizeof(s_snd_unit[0]));
                }
                armed = 1;              // the press that switched the board on must be released first
            }
            prev_p = p;
            held_ms = 0;
        }
        if (POWER_PINS && armed && p && raw_p) {
            held_ms += 20;
            if (held_ms >= 3000) {      // held 3 s: switch the board off
                cdc_log("power: power button held 3 s -> switching off (GPIO%d low)", PIN_PWR_OFF);
                buzzer_play(s_snd_off, sizeof(s_snd_off) / sizeof(s_snd_off[0]));
                gpio_set_level(PIN_PWR_OFF, !PWR_HOLD);
                vTaskDelay(pdMS_TO_TICKS(2000));            // on battery the board is dead by now; on USB it stays alive
                gpio_set_level(PIN_PWR_OFF, PWR_HOLD);      // still alive (USB): hold the latch again
                armed = 0;              // needs a release before the next power-off
                press_ok = 0;           // and this press must not toggle the units when released
                held_ms = 0;
            }
        }
        raw_m = m; raw_p = p;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Power-on sequence, called first thing in app_main. On battery the board is powered by the "power on" button (GPIO18, high when pressed)
// only while it is held: we take the latch (PWR_OFF GPIO17 high) immediately so the board stays on after the release, and play the
// power-on tune. On USB it does the same. Power-off is handled by c2_inputs_task (power button held for 3 s).
void power_boot(void)
{
    if (!POWER_PINS) return;
    buzzer_init();
    gpio_set_level(PIN_PWR_OFF, PWR_HOLD);
    gpio_config_t po = { .pin_bit_mask = 1ULL << PIN_PWR_OFF, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&po);
    gpio_config_t in = { .pin_bit_mask = 1ULL << PIN_PWR_ON, .mode = GPIO_MODE_INPUT };   // the board has its own pull-down
    gpio_config(&in);
    cdc_log("power: latch taken (GPIO%d high)", PIN_PWR_OFF);
    buzzer_play(s_snd_on, sizeof(s_snd_on) / sizeof(s_snd_on[0]));
}

// Normal boot of the C2 from its flash: BOOT (GPIO9) held high, then a reset pulse on EN. Without this the pins float
// after the P4 restarts and the C2 may stay in download mode / reset. Then listens to the C2 (buzzer requests from its web page).
void c2_run(void)
{
    gpio_config_t o = { .pin_bit_mask = (1ULL << PIN_C2_EN) | (1ULL << PIN_C2_BOOT), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&o);
    buzzer_init();
    uart_config_t uc = { .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                         .stop_bits = UART_STOP_BITS_1, .source_clk = UART_SCLK_DEFAULT };
    uart_driver_install(C2_UART, 512, 0, 0, NULL, 0);
    uart_param_config(C2_UART, &uc);
    uart_set_pin(C2_UART, PIN_C2_TX, PIN_C2_RX, -1, -1);
    gpio_set_level(PIN_C2_BOOT, 1);
    gpio_set_level(PIN_C2_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    if (C2_ENABLE) {
        gpio_set_level(PIN_C2_EN, 1);
        xTaskCreate(c2_link_task, "c2_link", 3072, NULL, 3, NULL);
    }   // else: EN stays low, the C2 is held in reset
    xTaskCreate(c2_inputs_task, "c2_inputs", 3072, NULL, 3, NULL);
}

// Blinky for the C2 (c2_blink/, LED on GPIO18): merged image, flashed at 0x0 (bootloader + partition table + app).
extern const uint8_t c2_blink_start[] asm("_binary_c2_blink_bin_start");
extern const uint8_t c2_blink_end[] asm("_binary_c2_blink_bin_end");

extern volatile int g_lvgl_pause;

static void c2_flash_blink_inner(void);

static void c2_flash_blink(void)
{
    g_lvgl_pause = 1;
    c2_flash_blink_inner();
    g_lvgl_pause = 0;
}

static void c2_flash_blink_inner(void)
{
    uint32_t size = c2_blink_end - c2_blink_start;
    esp_loader_flash_cfg_t cfg = { .offset = 0, .image_size = (size + 3) & ~3u, .block_size = 1024 };
    if (esp_loader_change_transmission_rate(&s_loader, 460800) != ESP_LOADER_SUCCESS) cdc_log("c2: staying at 115200");
    cdc_log("c2: flashing %u bytes at 0x0 ...", (unsigned)size);
    esp_loader_error_t e = esp_loader_flash_start(&s_loader, &cfg);
    if (e != ESP_LOADER_SUCCESS) { cdc_log("c2: flash_start failed (%d)", e); return; }
    for (uint32_t off = 0; off < size; off += cfg.block_size) {
        uint32_t n = size - off < cfg.block_size ? size - off : cfg.block_size;
        e = esp_loader_flash_write(&s_loader, &cfg, c2_blink_start + off, n);
        if (e != ESP_LOADER_SUCCESS) { cdc_log("c2: flash_write failed at 0x%x (%d)", (unsigned)off, e); return; }
    }
    e = esp_loader_flash_finish(&s_loader, &cfg);   // MD5 check
    if (e != ESP_LOADER_SUCCESS) { cdc_log("c2: flash_finish / MD5 check failed (%d)", e); return; }
    cdc_log("c2: flashed and verified, resetting the C2 into the blinky");
    esp_loader_reset_target(&s_loader);
}

void c2_probe(int flash)
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
    if (flash) c2_flash_blink();
    esp_loader_deinit(&s_loader);       // releases the UART; without a flash the C2 stays in download mode until it is reset
}
