// ESP32-P4 / CH13613 466x466 MIPI-DSI test: full-screen red / green / blue in sequence,
// written into the panel's own GRAM with DCS RAMW/RAMWC (DSI command mode).
// The DPI video path of the P4 stalls after one frame with this panel, so it is not used.
// + TinyUSB CDC ACM with "1200 baud => ROM download mode" auto-reset, and logs over CDC.
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
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

// RAMW (0x2C) / RAMWC (0x3C) are sent in HS and in small packets: the IDF DCS long-write helper pushes
// the whole payload into the host's payload FIFO *before* the packet header, so a payload bigger than
// that FIFO deadlocks (40 px = 120 bytes + command byte = 31 words is safe).
#define CHUNK_PX 40
#define COL_OFFSET 6   // the panel's 466 columns start at GRAM column 6 (CASET 6..471)

static esp_err_t set_window(int x0, int y0, int x1, int y1)
{
    const uint8_t ca[4] = { (x0 + COL_OFFSET) >> 8, (x0 + COL_OFFSET) & 0xFF, (x1 + COL_OFFSET) >> 8, (x1 + COL_OFFSET) & 0xFF };
    const uint8_t ra[4] = { y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF };
    esp_err_t err = esp_lcd_panel_io_tx_param(s_io, 0x2A, ca, 4);
    if (err == ESP_OK) err = esp_lcd_panel_io_tx_param(s_io, 0x2B, ra, 4);
    return err;
}

typedef void (*pixel_fn)(int x, int y, uint8_t *rgb, void *ctx);

// Write the window x0..x1 / y0..y1 (inclusive) pixel by pixel through fn().
static esp_err_t draw_window(int x0, int y0, int x1, int y1, pixel_fn fn, void *ctx)
{
    esp_err_t err = set_window(x0, y0, x1, y1);
    static uint8_t buf[CHUNK_PX * 3];
    int n = 0;
    bool first = true;
    for (int y = y0; y <= y1 && err == ESP_OK; y++) {
        for (int x = x0; x <= x1 && err == ESP_OK; x++) {
            fn(x, y, &buf[3 * n], ctx);
            if (++n == CHUNK_PX) {
                err = esp_lcd_panel_io_tx_param(s_io, first ? 0x2C : 0x3C, buf, n * 3);
                first = false;
                n = 0;
            }
        }
    }
    if (err == ESP_OK && n) err = esp_lcd_panel_io_tx_param(s_io, first ? 0x2C : 0x3C, buf, n * 3);
    return err;
}

static void px_black(int x, int y, uint8_t *rgb, void *ctx) { rgb[0] = rgb[1] = rgb[2] = 0; }

// ---- Bouncing ball -------------------------------------------------------------
#define BALL_R   22
#define ERASE_MARGIN 4   // the redrawn box is bigger than old+new ball, so nothing of the old image stays
#define SCREEN_R 233   // the panel is round: 466 / 2

typedef struct { int cx, cy; } ball_t;

static void px_ball(int x, int y, uint8_t *rgb, void *ctx)
{
    const ball_t *b = ctx;
    int dx = x - b->cx, dy = y - b->cy;
    bool inside = dx * dx + dy * dy <= BALL_R * BALL_R;
    rgb[0] = inside ? 255 : 0;   // solid red ball on a black background
    rgb[1] = 0;
    rgb[2] = 0;
}

static esp_err_t draw_ball(const ball_t *old, const ball_t *cur)
{
    int x0 = (old->cx < cur->cx ? old->cx : cur->cx) - BALL_R - ERASE_MARGIN;
    int x1 = (old->cx > cur->cx ? old->cx : cur->cx) + BALL_R + ERASE_MARGIN;
    int y0 = (old->cy < cur->cy ? old->cy : cur->cy) - BALL_R - ERASE_MARGIN;
    int y1 = (old->cy > cur->cy ? old->cy : cur->cy) + BALL_R + ERASE_MARGIN;
    // clip, and keep an even start column and an even width (partial-update restriction of the chip)
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > LCD_H - 1) x1 = LCD_H - 1;
    if (y1 > LCD_V - 1) y1 = LCD_V - 1;
    // datasheet (CASET/RASET): for partial updates the start and the size must be even, in X AND in Y
    x0 &= ~1;
    x1 |= 1;
    y0 &= ~1;
    y1 |= 1;
    return draw_window(x0, y0, x1, y1, px_ball, (void *)cur);
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
    vTaskDelay(pdMS_TO_TICKS(300));

    // RAMW/RAMWC must be sent in MIPI HS format (datasheet): DCS long writes in HS (0 = HS, 1 = LP).
    MIPI_DSI_HOST.cmd_mode_cfg.dcs_lw_tx = 0;
    // The power IC for ELVDD/ELVSS is driven by the CH13613 over SWIRE: force it on (0x6C = 0xA5),
    // full brightness (0x51 = 0xFF). Without this the panel stays black.
    {
        const uint8_t sw = 0xA5, dbv = 0xFF;
        esp_lcd_panel_io_tx_param(s_io, 0x6C, &sw, 1);
        esp_lcd_panel_io_tx_param(s_io, 0x51, &dbv, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    draw_window(0, 0, LCD_H - 1, LCD_V - 1, px_black, NULL);   // clear the screen
    cdc_log("LCD ready, bouncing ball");

    ball_t cur = { LCD_H / 2, LCD_V / 2 }, old = cur;
    float px = cur.cx, py = cur.cy, vx = 3.1f, vy = 2.3f;   // pixels per frame
    const float lim = SCREEN_R - BALL_R - 2;
    TickType_t t_log = xTaskGetTickCount();
    int frames = 0;
    for (;;) {
        old = cur;
        px += vx;
        py += vy;
        // bounce on the round screen edge: reflect the velocity about the normal
        float dx = px - LCD_H / 2, dy = py - LCD_V / 2;
        float d2 = dx * dx + dy * dy;
        if (d2 > lim * lim) {
            float d = sqrtf(d2);
            float nx = dx / d, ny = dy / d;
            float dot = vx * nx + vy * ny;
            if (dot > 0) {          // moving outwards: v' = v - 2 (v.n) n
                vx -= 2 * dot * nx;
                vy -= 2 * dot * ny;
                // small angle jitter so the path never repeats exactly
                float c = 0.996f, sn = 0.09f * ((frames & 1) ? 1 : -1);
                float tx = vx * c - vy * sn;
                vy = vx * sn + vy * c;
                vx = tx;
            }
            px = LCD_H / 2 + nx * (lim - 1);
            py = LCD_V / 2 + ny * (lim - 1);
        }
        cur.cx = (int)(px + 0.5f);
        cur.cy = (int)(py + 0.5f);
        esp_err_t e = draw_ball(&old, &cur);
        if (e != ESP_OK) { cdc_log("draw error: %s", esp_err_to_name(e)); vTaskDelay(pdMS_TO_TICKS(500)); }
        frames++;
        if (xTaskGetTickCount() - t_log >= pdMS_TO_TICKS(5000)) {
            cdc_log("ball at (%d,%d) v=(%.2f,%.2f), %d frames in 5 s", cur.cx, cur.cy, vx, vy, frames);
            frames = 0;
            t_log = xTaskGetTickCount();
        }
        vTaskDelay(1);
    }
}
