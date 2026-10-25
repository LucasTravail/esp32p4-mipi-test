// ESP32-P4 / CH13613 466x466 MIPI-DSI test: full-screen red / green / blue in sequence,
// written into the panel's own GRAM with DCS RAMW/RAMWC (DSI command mode).
// The DPI video path of the P4 stalls after one frame with this panel, so it is not used.
// + TinyUSB CDC ACM with "1200 baud => ROM download mode" auto-reset, and logs over CDC.
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "soc/soc.h"
#include "soc/lp_system_reg.h"
#include "soc/mipi_dsi_host_struct.h"

static const char *TAG = "p4_mipi";

#define PIN_LCD_ENABLE GPIO_NUM_15   // VCI_EN, active high
#define PIN_LCD_RESET  GPIO_NUM_14   // active low
#define PIN_LED        GPIO_NUM_21

#define LCD_H 466
#define LCD_V 466

static TaskHandle_t s_boot_task;
static esp_lcd_panel_io_handle_t s_io;

typedef struct { uint8_t cmd; uint8_t len; uint8_t p[4]; uint16_t delay_ms; } lcd_cmd_t;
// Same sequence as the panel vendor's init file (CH13613 EDO 1.504", 1 lane, 466x466).
static const lcd_cmd_t k_init[] = {
    {0x2A, 4, {0x00, 0x06, 0x01, 0xD7}, 0},   // CASET 6..471
    {0x2B, 4, {0x00, 0x00, 0x01, 0xD1}, 0},   // RASET 0..465
    {0x35, 1, {0x00}, 0},
    {0x36, 1, {0x00}, 0},
    {0x64, 1, {0x00}, 0},
    {0x68, 1, {0x01}, 0},
    {0x67, 1, {0x01}, 0},
    {0x3A, 1, {0x07}, 0},                     // 24 bit/pixel
    {0x11, 0, {0}, 120},
    {0x13, 0, {0}, 0},
    {0x29, 0, {0}, 120},
};

// ---- USB: 1200 bps => bootloader --------------------------------------------
static void line_coding_cb(int itf, cdcacm_event_t *event)
{
    // USB context: only signal a task, never restart here.
    if (event->line_coding_changed_data.p_line_coding->bit_rate == 1200 && s_boot_task) {
        xTaskNotifyGive(s_boot_task);
    }
}

static void boot_request_task(void *arg)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_LOGW(TAG, "1200 bps CDC request: rebooting into ROM download mode");
    vTaskDelay(pdMS_TO_TICKS(100));
    REG_SET_BIT(LP_SYSTEM_REG_SYS_CTRL_REG, LP_SYSTEM_REG_FORCE_DOWNLOAD_BOOT);
    esp_restart();
}

static esp_err_t usb_start(void)
{
    xTaskCreate(boot_request_task, "boot_req", 3072, NULL, 5, &s_boot_task);

    const tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    esp_err_t err = tinyusb_driver_install(&tusb_cfg);
    if (err != ESP_OK) return err;

    const tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = line_coding_cb,
    };
    return tinyusb_cdcacm_init(&acm_cfg);
}

// ---- Logging over USB CDC (UART0 not always reachable) ----------------------
static void cdc_log(const char *fmt, ...)
{
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf) - 2, fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n > (int)sizeof(buf) - 3) n = sizeof(buf) - 3;
    buf[n++] = '\r';
    buf[n++] = '\n';
    ESP_LOGI(TAG, "%.*s", n - 2, buf);
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (const uint8_t *)buf, n);
    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
}

// ---- LCD ----------------------------------------------------------------------
static esp_err_t lcd_start(void)
{
    gpio_set_level(PIN_LCD_RESET, 0);
    gpio_set_level(PIN_LCD_ENABLE, 0);
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LCD_RESET) | (1ULL << PIN_LCD_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&io));
    gpio_set_level(PIN_LCD_RESET, 0);
    gpio_set_level(PIN_LCD_ENABLE, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_lcd_dsi_bus_handle_t bus = NULL;
    esp_lcd_dsi_bus_config_t bus_cfg = {
        .bus_id = 0,
        .num_data_lanes = 1,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = 250,
    };
    esp_err_t err = esp_lcd_new_dsi_bus(&bus_cfg, &bus);
    if (err != ESP_OK) return err;

    esp_lcd_dbi_io_config_t dbi_cfg = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    err = esp_lcd_new_panel_io_dbi(bus, &dbi_cfg, &s_io);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_LCD_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    for (size_t i = 0; i < sizeof(k_init) / sizeof(k_init[0]); i++) {
        const lcd_cmd_t *c = &k_init[i];
        err = esp_lcd_panel_io_tx_param(s_io, c->cmd, c->len ? c->p : NULL, c->len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "LCD cmd 0x%02X failed: %s", c->cmd, esp_err_to_name(err));
            return err;
        }
        if (c->delay_ms) vTaskDelay(pdMS_TO_TICKS(c->delay_ms));
    }
    return ESP_OK;
}

// Fill the whole panel with one colour: RAMW (0x2C) with the first chunk, then RAMWC (0x3C).
// The IDF DCS-long-write helper pushes the whole payload into the host's payload FIFO *before*
// the packet header, so a payload bigger than that FIFO deadlocks: keep packets small
// (40 px = 120 bytes + command byte = 31 words).
#define CHUNK_PX 40
static esp_err_t fill_color(uint8_t r, uint8_t g, uint8_t b)
{
    static uint8_t buf[CHUNK_PX * 3];
    for (int x = 0; x < CHUNK_PX; x++) {
        buf[3 * x + 0] = r;
        buf[3 * x + 1] = g;
        buf[3 * x + 2] = b;
    }
    esp_err_t err = esp_lcd_panel_io_tx_param(s_io, 0x2A, k_init[0].p, 4);   // reset write window
    if (err == ESP_OK) err = esp_lcd_panel_io_tx_param(s_io, 0x2B, k_init[1].p, 4);
    int remaining = LCD_H * LCD_V;
    bool first = true;
    while (remaining > 0 && err == ESP_OK) {
        int px = remaining < CHUNK_PX ? remaining : CHUNK_PX;
        err = esp_lcd_panel_io_tx_param(s_io, first ? 0x2C : 0x3C, buf, px * 3);
        first = false;
        remaining -= px;
    }
    return err;
}

void app_main(void)
{
    gpio_config_t led = { .pin_bit_mask = 1ULL << PIN_LED, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&led);
    gpio_set_level(PIN_LED, 1);

    // USB first so a bootloader request works even if the LCD fails.
    esp_err_t err = usb_start();
    if (err != ESP_OK) ESP_LOGE(TAG, "USB init failed: %s", esp_err_to_name(err));

    err = lcd_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LCD init failed: %s (no retry, USB stays active)", esp_err_to_name(err));
        return;
    }
    cdc_log("LCD init done, drawing colours with DCS RAM writes");
    vTaskDelay(pdMS_TO_TICKS(300));

    // The datasheet says RAMW/RAMWC must be sent in MIPI HS format: DCS long writes in HS (0 = HS, 1 = LP).
    MIPI_DSI_HOST.cmd_mode_cfg.dcs_lw_tx = 0;
    // The power IC for ELVDD/ELVSS is driven by the CH13613 over SWIRE; force it on (reg 0x6C = 0xA5)
    // and set full brightness, in case it is not enabled automatically without a video stream.
    {
        const uint8_t sw = 0xA5, dbv = 0xFF;
        esp_lcd_panel_io_tx_param(s_io, 0x6C, &sw, 1);
        esp_lcd_panel_io_tx_param(s_io, 0x51, &dbv, 1);
        uint8_t v6c = 0xEE, v51 = 0xEE;
        esp_lcd_panel_io_rx_param(s_io, 0x52, &v51, 1);
        cdc_log("SWIRE manual=A5 sent, brightness readback=%02X", v51);
        (void)v6c;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    // (ALLPON/ALLPOFF test removed: ALLPOFF is not left by NORON on this chip, ImgMode stayed 0x08)
    uint8_t pm = 0xEE, im = 0xEE;
    esp_lcd_panel_io_rx_param(s_io, 0x0A, &pm, 1);
    esp_lcd_panel_io_rx_param(s_io, 0x0D, &im, 1);
    cdc_log("after NORON: PowerMode=%02X ImgMode=%02X", pm, im);

    static const struct { const char *name; uint8_t r, g, b; } colors[] = {
        {"red", 255, 0, 0}, {"green", 0, 255, 0}, {"blue", 0, 0, 255},
        {"white", 255, 255, 255}, {"black", 0, 0, 0},
    };
    for (size_t i = 0;; i = (i + 1) % (sizeof(colors) / sizeof(colors[0]))) {
        TickType_t t0 = xTaskGetTickCount();
        esp_err_t e = fill_color(colors[i].r, colors[i].g, colors[i].b);
        // read back the first 2 pixels of the GRAM to check that the write really landed
        uint8_t rd[7] = {0};
        esp_lcd_panel_io_tx_param(s_io, 0x2A, k_init[0].p, 4);
        esp_lcd_panel_io_tx_param(s_io, 0x2B, k_init[1].p, 4);
        esp_err_t er = esp_lcd_panel_io_rx_param(s_io, 0x2E, rd, sizeof(rd));
        cdc_log("GRAM read (%s): %02X %02X %02X %02X %02X %02X %02X", er == ESP_OK ? "ok" : "ERR",
                rd[0], rd[1], rd[2], rd[3], rd[4], rd[5], rd[6]);
        cdc_log("colour: %s  (write %s, %lld ms)", colors[i].name, e == ESP_OK ? "ok" : esp_err_to_name(e),
                (long long)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS));
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}
