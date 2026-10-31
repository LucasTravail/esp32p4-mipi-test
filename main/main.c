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
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "lvgl.h"
#if __has_include("esp32p4_ui.h")
#include "esp32p4_ui.h"
#define UI_START_SCREEN_CREATE screen1_create   // screens/screen1.xml of the LVGL Pro project
#endif

static const char *TAG = "p4_mipi";

#define PIN_LCD_ENABLE GPIO_NUM_15   // VCI_EN, active high
#define PIN_LCD_RESET  GPIO_NUM_14   // active low
#define PIN_LED        GPIO_NUM_21

#define PIN_TP_SDA     GPIO_NUM_7    // I2C_SDA (external 10k pull-up)
#define PIN_TP_SCL     GPIO_NUM_5    // I2C_SCL (external 10k pull-up)
#define PIN_TP_RST     GPIO_NUM_3    // TP_RST, active low
#define PIN_TP_INT     GPIO_NUM_12   // TP_INT, active low
#define TP_ADDR        0x5A          // CST9217: 8-bit address 0xB4/0xB5

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
    {0x3A, 1, {0x05}, 0},                     // 16 bit/pixel (RGB565): half the data to send
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
void cdc_log(const char *fmt, ...)
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

// ---- Touch: Hynitron CST9217 over I2C -----------------------------------------------
static i2c_master_dev_handle_t s_tp;
static bool s_tp_ok;

// The CST9217 uses 16-bit register addresses (big endian).
static esp_err_t tp_read(uint16_t reg, uint8_t *data, size_t len)
{
    const uint8_t a[2] = { reg >> 8, reg & 0xFF };
    return i2c_master_transmit_receive(s_tp, a, 2, data, len, 50);
}

static esp_err_t tp_write(uint16_t reg, uint8_t val)
{
    const uint8_t a[3] = { reg >> 8, reg & 0xFF, val };
    return i2c_master_transmit(s_tp, a, 3, 50);
}

static void touch_init(void)
{
    // hardware reset
    gpio_config_t rst = { .pin_bit_mask = 1ULL << PIN_TP_RST, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&rst);
    gpio_config_t irq = { .pin_bit_mask = 1ULL << PIN_TP_INT, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&irq);
    gpio_set_level(PIN_TP_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    {
        gpio_config_t in0 = { .pin_bit_mask = (1ULL << PIN_TP_SDA) | (1ULL << PIN_TP_SCL), .mode = GPIO_MODE_INPUT };
        gpio_config(&in0);
        cdc_log("touch: levels while TP_RST is LOW: SDA=%d SCL=%d INT=%d", gpio_get_level(PIN_TP_SDA), gpio_get_level(PIN_TP_SCL), gpio_get_level(PIN_TP_INT));
    }
    gpio_set_level(PIN_TP_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(150));   // chip re-initialisation time (datasheet: 100 ms)

    // idle levels of the lines (external pull-ups should give 1/1/1)
    gpio_config_t in = { .pin_bit_mask = (1ULL << PIN_TP_SDA) | (1ULL << PIN_TP_SCL), .mode = GPIO_MODE_INPUT };
    gpio_config(&in);
    cdc_log("touch: idle levels SDA=%d SCL=%d INT=%d", gpio_get_level(PIN_TP_SDA), gpio_get_level(PIN_TP_SCL), gpio_get_level(PIN_TP_INT));

    // SDA stuck low? Try the standard I2C bus recovery: up to 9 clock pulses on SCL, then a STOP.
    if (gpio_get_level(PIN_TP_SDA) == 0) {
        gpio_config_t od = { .pin_bit_mask = (1ULL << PIN_TP_SDA) | (1ULL << PIN_TP_SCL), .mode = GPIO_MODE_INPUT_OUTPUT_OD, .pull_up_en = GPIO_PULLUP_ENABLE };
        gpio_config(&od);
        gpio_set_level(PIN_TP_SDA, 1);
        for (int i = 0; i < 9 && gpio_get_level(PIN_TP_SDA) == 0; i++) {
            gpio_set_level(PIN_TP_SCL, 0); esp_rom_delay_us(10);
            gpio_set_level(PIN_TP_SCL, 1); esp_rom_delay_us(10);
        }
        gpio_set_level(PIN_TP_SDA, 0); esp_rom_delay_us(10);   // STOP = SDA rises while SCL is high
        gpio_set_level(PIN_TP_SCL, 1); esp_rom_delay_us(10);
        gpio_set_level(PIN_TP_SDA, 1); esp_rom_delay_us(10);
        cdc_log("touch: after bus recovery SDA=%d SCL=%d", gpio_get_level(PIN_TP_SDA), gpio_get_level(PIN_TP_SCL));
        gpio_reset_pin(PIN_TP_SDA);
        gpio_reset_pin(PIN_TP_SCL);
    }

    i2c_master_bus_handle_t bus = NULL;
    char found[120];
    int addr_found = -1;
    bool quick = true;   // first try only the expected address, scan the whole bus only if it fails
    for (int attempt = 0; attempt < 3 && addr_found < 0; attempt++) {
        const gpio_num_t sda = attempt == 1 ? PIN_TP_SCL : PIN_TP_SDA, scl = attempt == 1 ? PIN_TP_SDA : PIN_TP_SCL;
        const i2c_master_bus_config_t bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = sda,
            .scl_io_num = scl,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) { cdc_log("touch: I2C bus init failed"); return; }
        int n = snprintf(found, sizeof(found), "touch: scan (SDA=%d SCL=%d):", (int)sda, (int)scl);
        for (int a = quick ? TP_ADDR : 0x08; a < (quick ? TP_ADDR + 1 : 0x78); a++) {
            if (i2c_master_probe(bus, a, 20) == ESP_OK) { n += snprintf(found + n, sizeof(found) - n, " 0x%02X", a); if (addr_found < 0) addr_found = a; }
        }
        cdc_log("%s", found);
        if (addr_found < 0) { i2c_del_master_bus(bus); bus = NULL; quick = false; }
    }
    if (addr_found < 0) { cdc_log("touch: nothing answers on the I2C bus (touch disabled)"); return; }

    const i2c_device_config_t dev_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addr_found, .scl_speed_hz = 400000 };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &s_tp) != ESP_OK) { cdc_log("touch: add device failed"); return; }
    uint8_t id[4] = {0};
    esp_err_t e = tp_read(0xD204, id, sizeof(id));
    cdc_log("touch: chip id read (0xD204) %s: %02X %02X %02X %02X", e == ESP_OK ? "ok" : esp_err_to_name(e), id[0], id[1], id[2], id[3]);
    s_tp_ok = (e == ESP_OK);
}

static volatile int s_tx, s_ty;
static volatile int s_tp_polls, s_tp_valid, s_tp_err;
static volatile uint8_t s_tp_raw[7];
static volatile bool s_tpress;

// Returns true when a valid frame was read. Frame layout (CST92xx family):
//   buf[0..4] first point: [id<<4 | status][x hi][y hi][x lo<<4 | y lo][pressure]
//   buf[5] = point count (& 0x7F), buf[6] = 0xAB when the frame is valid; ack by writing 0xAB to 0xD000.
static bool touch_poll(void)
{
    uint8_t b[16] = {0};
    // INT is only a short pulse (missed by polling), and the chip returns 0xFF when read too often: poll every 30 ms.
    static TickType_t last;
    TickType_t now = xTaskGetTickCount();
    if (now - last < pdMS_TO_TICKS(30)) return false;
    last = now;
    s_tp_polls++;
    if (!s_tp_ok || tp_read(0xD000, b, sizeof(b)) != ESP_OK) { s_tp_err++; return false; }
    for (int i = 0; i < 7; i++) s_tp_raw[i] = b[i];
    bool valid = (b[6] == 0xAB) && (b[0] != 0xAB);
    if (valid) {
        s_tp_valid++;
        int count = b[5] & 0x7F;
        bool down = count > 0 && (b[0] & 0x0F) == 0x06;
        if (down) {
            s_tx = (b[1] << 4) | (b[3] >> 4);
            s_ty = (b[2] << 4) | (b[3] & 0x0F);
        }
        if (down != s_tpress) cdc_log("touch %s x=%d y=%d  raw=%02X %02X %02X %02X %02X %02X %02X", down ? "DOWN" : "UP  ", s_tx, s_ty,
                                      b[0], b[1], b[2], b[3], b[4], b[5], b[6]);
        s_tpress = down;
    }
    if (b[6] == 0xAB || valid) tp_write(0xD000, 0xAB);   // acknowledge
    return valid;
}

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    touch_poll();
    data->point.x = s_tx;
    data->point.y = s_ty;
    data->state = s_tpress ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

// ESP8684 link test / programming (c2.c). GPIO8 of the ESP8684 now has a pull-up (download mode needs GPIO8=1, GPIO9=0).
// C2_PROBE 1 runs it once at boot; C2_FLASH 1 also flashes c2_blink.bin (LED on GPIO18) and runs it.
// Done once (blinky flashed and verified): left off so the C2 is not reflashed at every boot.
#define C2_PROBE 0
#define C2_FLASH 1
void c2_probe(int flash);
void c2_run(void);
void power_boot(void);
void encoder_init(void);
int encoder_mm100(void);   // AS5311 position in 1/100 mm since the last zero (encoder.c)

// ---- Fast pixel path ----------------------------------------------------------------
// The IDF DBI driver enables command ACK, so after EVERY packet the host turns the bus around (BTA) and waits for
// the panel to acknowledge: that, not the lane rate, was limiting us to ~7 MB/s. Pixels are written without ACK,
// packets back to back (the payload must be complete in the host FIFO before the header is written).
// Pixels are LVGL RGB565 (little endian in memory); the panel wants big endian: swapped on the fly.
#define FAST_PKT_PX 126    // pixels per DSI packet; the whole payload must fit the host FIFO (header goes last)

static inline uint32_t swap16x2(uint32_t w) { return ((w & 0x00FF00FFu) << 8) | ((w >> 8) & 0x00FF00FFu); }

static volatile int g_dsi_to;
static inline void dsi_put(uint32_t w)
{
    uint32_t guard = 0;
    while (MIPI_DSI_HOST.cmd_pkt_status.gen_pld_w_full) {
        if (++guard > 200000) { g_dsi_to++; return; }
    }
    MIPI_DSI_HOST.gen_pld_data.val = w;
}

// One DCS long write: cmd (0x2C first packet / 0x3C continuation) + npx RGB565 pixels (npx even, except the last).
static void dsi_pixels_packet(uint8_t cmd, const uint16_t *src, int npx)
{
    uint32_t bytes = (uint32_t)npx * 2, wc = bytes + 1;
    uint32_t prev = (uint32_t)cmd << 24;        // the command byte travels in front of the pixel bytes
    int pairs = npx >> 1;
    if ((uintptr_t)src & 2) {                   // 2-byte aligned source: build words from halfwords
        for (int i = 0; i < pairs && !g_dsi_to; i++) {
            uint32_t w = swap16x2((uint32_t)src[2 * i] | ((uint32_t)src[2 * i + 1] << 16));
            dsi_put((prev >> 24) | (w << 8));
            prev = w;
        }
    } else {
        const uint32_t *s32 = (const uint32_t *)src;
        for (int i = 0; i < pairs && !g_dsi_to; i++) {
            uint32_t w = swap16x2(s32[i]);
            dsi_put((prev >> 24) | (w << 8));
            prev = w;
        }
    }
    if (npx & 1) {                              // odd pixel count (end of an area)
        uint32_t w = swap16x2(src[npx - 1]);
        dsi_put((prev >> 24) | (w << 8));       // cmd-shifted: 3 valid bytes (prev top byte + 2 pixel bytes)
    } else {
        dsi_put(prev >> 24);                    // the last pixel byte, alone in its word
    }
    uint32_t guard = 0;
    while (MIPI_DSI_HOST.cmd_pkt_status.gen_cmd_full) { if (++guard > 200000) { g_dsi_to++; break; } }
    MIPI_DSI_HOST.gen_hdr.val = 0x39u | ((wc & 0xFF) << 8) | ((wc >> 8) << 16);   // DCS long write, VC 0
}

// Send pixels to the already-selected window (CASET/RASET): dsi_begin() then any number of dsi_pixels() calls
// (the first packet is RAMW 0x2C, all the others RAMWC 0x3C).
static uint8_t s_dsi_cmd = 0x2C;
static void dsi_begin(void) { s_dsi_cmd = 0x2C; }
static void dsi_pixels(const uint16_t *src, int npx)
{
    while (npx > 0) {
        int n = npx < FAST_PKT_PX ? npx : FAST_PKT_PX;
        dsi_pixels_packet(s_dsi_cmd, src, n);
        s_dsi_cmd = 0x3C;
        src += n;
        npx -= n;
    }
}

// ---- LVGL ----------------------------------------------------------------------
// Partial rendering: LVGL draws into two small internal-RAM buffers and flush_cb sends each dirty
// rectangle into the panel's GRAM (DCS RAMW/RAMWC in HS). The panel needs even-aligned windows.
#define DRAW_LINES 60
#define LVGL_BUF_BYTES (LCD_H * DRAW_LINES * 2)

static void lvgl_round_cb(lv_event_t *e)
{
    lv_area_t *a = lv_event_get_param(e);   // LV_EVENT_INVALIDATE_AREA passes the area
    a->x1 &= ~1;           // even first column / row, odd last one => even width / height
    a->y1 &= ~1;
    a->x2 |= 1;
    a->y2 |= 1;
    if (a->x2 > LCD_H - 1) a->x2 = LCD_H - 1;
    if (a->y2 > LCD_V - 1) a->y2 = LCD_V - 1;
}

static volatile int64_t g_fl_us, g_lv_us;
static volatile int g_fl_n, g_fl_px;

static uint16_t *s_shadow;          // copy of what LVGL last rendered (the current screen), RGB565, PSRAM
static bool s_capture;              // true: flush only fills s_shadow, nothing is sent to the panel

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    int64_t t0 = esp_timer_get_time();
    int w = area->x2 - area->x1 + 1, h = area->y2 - area->y1 + 1, npx = w * h;
    if (s_shadow) {
        for (int y = 0; y < h; y++)
            memcpy(s_shadow + (area->y1 + y) * LCD_H + area->x1, px_map + y * w * 2, w * 2);
    }
    if (!s_capture) {
        esp_err_t err = set_window(area->x1, area->y1, area->x2, area->y2);
        if (err == ESP_OK) { dsi_begin(); dsi_pixels((const uint16_t *)px_map, npx); }
        else ESP_LOGE(TAG, "flush failed: %s", esp_err_to_name(err));
    }
    g_fl_us += esp_timer_get_time() - t0;
    if (g_dsi_to) {
        static int rep;
        if (rep++ < 5) cdc_log("DSI timeout x%d status=0x%08x", g_dsi_to, (unsigned)MIPI_DSI_HOST.cmd_pkt_status.val);
        g_dsi_to = 0;
    }
    g_fl_n++;
    g_fl_px += npx;
    lv_display_flush_ready(disp);
}

static uint32_t lvgl_tick_cb(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void nav_poll(void);

volatile int g_lvgl_pause;   // set by c2.c while it flashes the ESP8684 (the UI stays frozen, the P4 crashed when both ran)

static void lvgl_task(void *arg)
{
    for (;;) {
        if (g_lvgl_pause) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
        nav_poll();
        int64_t t0 = esp_timer_get_time();
        uint32_t wait_ms = lv_timer_handler();
        g_lv_us += esp_timer_get_time() - t0;
        static int64_t last;
        if (t0 - last > 2000000) {
            cdc_log("perf: handler %d ms, flush %d ms (%d flushes, %d px) per 2 s", (int)(g_lv_us / 1000),
                    (int)(g_fl_us / 1000), g_fl_n, g_fl_px);
            g_lv_us = g_fl_us = 0; g_fl_n = g_fl_px = 0; last = t0;
        }
        vTaskDelay(pdMS_TO_TICKS(wait_ms < 10 ? 10 : (wait_ms > 100 ? 100 : wait_ms)));
    }
}

// ---- Demo UI: arc gauge + percentage + a red ball orbiting the edge -----------------
static lv_obj_t *s_arc, *s_label, *s_ball;

static void arc_changed_cb(lv_event_t *e)
{
    lv_label_set_text_fmt(s_label, "%d%%", (int)lv_arc_get_value(s_arc));
}

static void anim_orbit_cb(void *var, int32_t deg)
{
    const float r = 205.0f, rad = deg * 3.14159265f / 180.0f;
    lv_obj_set_pos(s_ball, (int)(LCD_H / 2 + r * cosf(rad)) - 12, (int)(LCD_V / 2 + r * sinf(rad)) - 12);
}

__attribute__((unused)) static void ui_create(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    s_arc = lv_arc_create(scr);
    lv_obj_set_size(s_arc, 340, 340);
    lv_obj_center(s_arc);
    lv_arc_set_rotation(s_arc, 135);
    lv_arc_set_bg_angles(s_arc, 0, 270);
    lv_arc_set_range(s_arc, 0, 100);
    lv_obj_set_style_bg_color(s_arc, lv_color_white(), LV_PART_KNOB);   // draggable with the finger
    lv_obj_add_event_cb(s_arc, arc_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_arc_set_value(s_arc, 50);
    lv_obj_set_style_arc_width(s_arc, 24, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_arc, 24, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_arc, lv_color_make(40, 40, 40), LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_arc, lv_color_make(0, 200, 255), LV_PART_INDICATOR);

    s_label = lv_label_create(scr);
    lv_obj_set_style_text_font(s_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_label, lv_color_white(), 0);
    lv_obj_center(s_label);
    lv_label_set_text(s_label, "50%");

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "ESP32-P4  LVGL");
    lv_obj_set_style_text_color(title, lv_color_make(160, 160, 160), 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 50);

    s_ball = lv_obj_create(scr);
    lv_obj_set_size(s_ball, 24, 24);
    lv_obj_set_style_radius(s_ball, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_ball, lv_color_make(255, 0, 0), 0);
    lv_obj_set_style_border_width(s_ball, 0, 0);
    lv_obj_remove_flag(s_ball, LV_OBJ_FLAG_SCROLLABLE);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_exec_cb(&a, anim_orbit_cb);
    lv_anim_set_values(&a, 0, 360);
    lv_anim_set_duration(&a, 3000);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);
}

// ---- Test pattern UI ("mire"): tap the screen to go to the next pattern ----------------
#define UI_MIRE 1   // 1 = test pattern, 0 = the arc/ball demo (ui_create)
enum { PAT_BARS, PAT_GRID, PAT_GRAY, PAT_RGB_GRAD, PAT_RED, PAT_GREEN, PAT_BLUE, PAT_WHITE, PAT_COUNT };
static int s_pat;
static lv_obj_t *s_mire;

static lv_obj_t *rect(lv_obj_t *parent, int x, int y, int w, int h, lv_color_t c)
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, w, h);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(o, c, 0);
    return o;
}

static void gradient(lv_obj_t *parent, int y, int h, lv_color_t c0, lv_color_t c1)
{
    lv_obj_t *o = rect(parent, 0, y, LCD_H, h, c0);
    lv_obj_set_style_bg_grad_color(o, c1, 0);
    lv_obj_set_style_bg_grad_dir(o, LV_GRAD_DIR_HOR, 0);
}

static void mire_build(void)
{
    lv_obj_clean(s_mire);
    lv_obj_set_style_bg_color(s_mire, lv_color_black(), 0);
    switch (s_pat) {
    case PAT_BARS: {   // EBU-style bars: white yellow cyan green magenta red blue black
        static const uint32_t cols[8] = { 0xFFFFFF, 0xFFFF00, 0x00FFFF, 0x00FF00, 0xFF00FF, 0xFF0000, 0x0000FF, 0x000000 };
        for (int i = 0; i < 8; i++) {
            int x0 = i * LCD_H / 8, x1 = (i + 1) * LCD_H / 8;
            rect(s_mire, x0, 0, x1 - x0, LCD_V, lv_color_hex(cols[i]));
        }
        break;
    }
    case PAT_GRID: {   // 50 px grid, border circle, centre cross
        for (int v = 0; v <= LCD_H; v += 50) {
            rect(s_mire, v, 0, 1, LCD_V, lv_color_make(80, 80, 80));
            rect(s_mire, 0, v, LCD_H, 1, lv_color_make(80, 80, 80));
        }
        rect(s_mire, LCD_H / 2 - 1, 0, 2, LCD_V, lv_color_white());
        rect(s_mire, 0, LCD_V / 2 - 1, LCD_H, 2, lv_color_white());
        lv_obj_t *c = lv_obj_create(s_mire);
        lv_obj_remove_style_all(c);
        lv_obj_set_size(c, LCD_H - 2, LCD_V - 2);
        lv_obj_set_pos(c, 1, 1);
        lv_obj_set_style_radius(c, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(c, 3, 0);
        lv_obj_set_style_border_color(c, lv_color_make(255, 200, 0), 0);
        lv_obj_set_style_bg_opa(c, LV_OPA_TRANSP, 0);
        break;
    }
    case PAT_GRAY:     // black -> white gradient
        gradient(s_mire, 0, LCD_V, lv_color_black(), lv_color_white());
        break;
    case PAT_RGB_GRAD: // one row per primary + white
        gradient(s_mire, 0, LCD_V / 4, lv_color_black(), lv_color_make(255, 0, 0));
        gradient(s_mire, LCD_V / 4, LCD_V / 4, lv_color_black(), lv_color_make(0, 255, 0));
        gradient(s_mire, LCD_V / 2, LCD_V / 4, lv_color_black(), lv_color_make(0, 0, 255));
        gradient(s_mire, 3 * LCD_V / 4, LCD_V / 4, lv_color_black(), lv_color_white());
        break;
    case PAT_RED:   rect(s_mire, 0, 0, LCD_H, LCD_V, lv_color_make(255, 0, 0)); break;
    case PAT_GREEN: rect(s_mire, 0, 0, LCD_H, LCD_V, lv_color_make(0, 255, 0)); break;
    case PAT_BLUE:  rect(s_mire, 0, 0, LCD_H, LCD_V, lv_color_make(0, 0, 255)); break;
    case PAT_WHITE: rect(s_mire, 0, 0, LCD_H, LCD_V, lv_color_white()); break;
    }
    // pattern number, in a black pill near the top so it is readable on every pattern
    lv_obj_t *pill = lv_obj_create(s_mire);
    lv_obj_remove_style_all(pill);
    lv_obj_set_size(pill, 120, 36);
    lv_obj_align(pill, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_radius(pill, 18, 0);
    lv_obj_set_style_bg_opa(pill, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(pill, lv_color_black(), 0);
    lv_obj_t *l = lv_label_create(pill);
    lv_label_set_text_fmt(l, "%d / %d", s_pat + 1, PAT_COUNT);
    lv_obj_set_style_text_color(l, lv_color_white(), 0);
    lv_obj_center(l);
}

static void mire_click_cb(lv_event_t *e)
{
    s_pat = (s_pat + 1) % PAT_COUNT;
    mire_build();
}

static void mire_create(void)
{
    s_mire = lv_screen_active();
    lv_obj_remove_flag(s_mire, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_mire, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_mire, mire_click_cb, LV_EVENT_CLICKED, NULL);
    mire_build();
}

// ---- Hardware test WITHOUT LVGL: touch arc gauge + orbiting ball --------------------
// Everything is drawn analytically straight into the panel's GRAM (partial windows), the touch
// controller is polled directly. If this works, display + touch hardware are fine.
#define USE_LVGL 1   // 0 = no-LVGL hardware test, 1 = LVGL UI (mire / editor UI)

#define ARC_R_IN   146
#define ARC_R_OUT  170
#define ARC_START  135.0f          // degrees, 0 = right, clockwise (y points down)
#define ARC_SPAN   270.0f
#define ORBIT_R    205
#define BALL_RAD   12
#define CX (LCD_H / 2)
#define CY (LCD_V / 2)

typedef struct { int value; int ball_x, ball_y; float knob_x, knob_y; } hw_state_t;

static void hw_update_knob(hw_state_t *st)
{
    float ka = (ARC_START + ARC_SPAN * st->value / 100.0f) * 0.0174533f;
    float mid = (ARC_R_IN + ARC_R_OUT) * 0.5f;
    st->knob_x = CX + mid * cosf(ka);
    st->knob_y = CY + mid * sinf(ka);
}

// 7-segment digit, 36 x 64 px, stroke 8: returns true if (x, y) (relative to the digit) is lit
static bool seg_on(int d, int x, int y)
{
    static const uint8_t m[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };   // gfedcba
    const int w = 36, h = 64, t = 8;
    uint8_t g = m[d];
    if (x < 0 || y < 0 || x >= w || y >= h) return false;
    if ((g & 0x01) && y < t) return true;                               // a  top
    if ((g & 0x08) && y >= h - t) return true;                          // d  bottom
    if ((g & 0x40) && y >= h / 2 - t / 2 && y < h / 2 + t / 2) return true;   // g  middle
    if ((g & 0x20) && x < t && y < h / 2) return true;                  // f  upper left
    if ((g & 0x02) && x >= w - t && y < h / 2) return true;            // b  upper right
    if ((g & 0x10) && x < t && y >= h / 2) return true;                // e  lower left
    if ((g & 0x04) && x >= w - t && y >= h / 2) return true;           // c  lower right
    return false;
}

static void colour_at(const hw_state_t *st, int x, int y, uint8_t *rgb)
{
    int r = 0, g = 0, b = 0;
    int dx = x - CX, dy = y - CY;
    int d2 = dx * dx + dy * dy;

    // arc ring
    if (d2 >= ARC_R_IN * ARC_R_IN && d2 <= ARC_R_OUT * ARC_R_OUT) {
        float a = atan2f((float)dy, (float)dx) * 57.29578f - ARC_START;
        while (a < 0) a += 360.0f;
        if (a <= ARC_SPAN) {
            if (a <= ARC_SPAN * st->value / 100.0f) { r = 0; g = 200; b = 255; }
            else { r = g = b = 45; }
        }
    }
    // knob at the end of the filled part (position precomputed in hw_update_knob)
    {
        float ex = x - st->knob_x, ey = y - st->knob_y;
        if (ex * ex + ey * ey <= 14.0f * 14.0f) r = g = b = 255;
    }
    // value as 1..3 digits in the centre
    if (dx > -80 && dx < 80 && dy > -34 && dy < 34) {
        int v = st->value, digits = v >= 100 ? 3 : (v >= 10 ? 2 : 1);
        int total = digits * 36 + (digits - 1) * 10, x0 = CX - total / 2, y0 = CY - 32;
        for (int i = 0; i < digits; i++) {
            int div = i == digits - 1 ? 1 : (digits - i == 3 ? 100 : 10);
            int dig = (v / div) % 10;
            if (seg_on(dig, x - (x0 + i * 46), y - y0)) r = g = b = 255;
        }
    }
    // orbiting ball
    {
        int bx = x - st->ball_x, by = y - st->ball_y;
        if (bx * bx + by * by <= BALL_RAD * BALL_RAD) { r = 255; g = b = 0; }
    }
    rgb[0] = r; rgb[1] = g; rgb[2] = b;
}

static volatile int g_pk, g_x, g_y, g_stage;
static void monitor_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        cdc_log("mon: stage=%d pk=%d x=%d y=%d", g_stage, g_pk, g_x, g_y);
    }
}

// Redraw the rectangle x0..x1 / y0..y1 (clipped, aligned on even coordinates as the chip requires).
// The pixels are computed into a RAM buffer FIRST and then streamed to the panel back to back:
// gaps between DSI packets (computing colours between packets) made the host stall after a few packets.
static esp_err_t redraw(const hw_state_t *st, int x0, int y0, int x1, int y1)
{
    static uint8_t *fbuf;
    if (!fbuf) fbuf = heap_caps_malloc(LCD_H * LCD_V * 3, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!fbuf) return ESP_ERR_NO_MEM;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > LCD_H - 1) x1 = LCD_H - 1;
    if (y1 > LCD_V - 1) y1 = LCD_V - 1;
    x0 &= ~1; y0 &= ~1; x1 |= 1; y1 |= 1;

    uint8_t *p = fbuf;
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++, p += 3) colour_at(st, x, y, p);
    }
    int total = (x1 - x0 + 1) * (y1 - y0 + 1);

    esp_err_t err = set_window(x0, y0, x1, y1);
    const uint8_t *q = fbuf;
    bool first = true;
    while (total > 0 && err == ESP_OK) {
        int n = total < CHUNK_PX ? total : CHUNK_PX;
        err = esp_lcd_panel_io_tx_param(s_io, first ? 0x2C : 0x3C, q, n * 3);
        first = false;
        q += n * 3;
        total -= n;
    }
    return err;
}

static void hw_test_run(void)
{
    hw_state_t st = { .value = 50, .ball_x = CX + ORBIT_R, .ball_y = CY };
    hw_update_knob(&st);
    TickType_t t0 = xTaskGetTickCount();
    for (int y = 0; y < LCD_V; y += 58) {   // strips, timed, so a slow / stuck part shows up in the log
        TickType_t ts = xTaskGetTickCount();
        esp_err_t er = redraw(&st, 0, y, LCD_H - 1, y + 57);
        cdc_log("strip y=%d: %s, %d ms", y, er == ESP_OK ? "ok" : esp_err_to_name(er), (int)((xTaskGetTickCount() - ts) * portTICK_PERIOD_MS));
    }
    cdc_log("first full redraw: %d ms", (int)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS));
    cdc_log("hardware test (no LVGL): touch the arc to set the gauge, the red ball orbits");

    int deg = 0;
    TickType_t t_log = xTaskGetTickCount();
    int frames = 0;
    for (;;) {
        // --- touch: the ring sets the value from the touch angle
        touch_poll();
        int new_val = st.value;
        if (s_tpress) {
            int dx = s_tx - CX, dy = s_ty - CY;
            if (dx * dx + dy * dy > 60 * 60) {
                float a = atan2f((float)dy, (float)dx) * 57.29578f - ARC_START;
                while (a < 0) a += 360.0f;
                if (a > ARC_SPAN) a = (a - ARC_SPAN < 360.0f - a) ? ARC_SPAN : 0.0f;   // dead zone: snap
                new_val = (int)(a * 100.0f / ARC_SPAN + 0.5f);
            }
        }
        // --- ball
        deg = (deg + 4) % 360;
        int nbx = CX + (int)(ORBIT_R * cosf(deg * 0.0174533f));
        int nby = CY + (int)(ORBIT_R * sinf(deg * 0.0174533f));

        esp_err_t e = ESP_OK;
        int ox = st.ball_x, oy = st.ball_y;
        st.ball_x = nbx; st.ball_y = nby;
        int m = BALL_RAD + 4;
        int bx0 = (ox < nbx ? ox : nbx) - m, bx1 = (ox > nbx ? ox : nbx) + m;
        int by0 = (oy < nby ? oy : nby) - m, by1 = (oy > nby ? oy : nby) + m;
        e = redraw(&st, bx0, by0, bx1, by1);
        if (e == ESP_OK && new_val != st.value) {
            st.value = new_val;
            hw_update_knob(&st);
            e = redraw(&st, CX - ARC_R_OUT - 16, CY - ARC_R_OUT - 16, CX + ARC_R_OUT + 16, CY + ARC_R_OUT + 16);
        }
        if (e != ESP_OK) { cdc_log("draw error: %s", esp_err_to_name(e)); vTaskDelay(pdMS_TO_TICKS(500)); }
        frames++;
        if (xTaskGetTickCount() - t_log >= pdMS_TO_TICKS(5000)) {
            cdc_log("value=%d touch=%d (%d,%d) %d frames | tp_ok=%d polls=%d valid=%d err=%d raw=%02X %02X %02X %02X %02X %02X %02X INT=%d", st.value, (int)s_tpress, s_tx, s_ty, frames, (int)s_tp_ok, s_tp_polls, s_tp_valid, s_tp_err, s_tp_raw[0], s_tp_raw[1], s_tp_raw[2], s_tp_raw[3], s_tp_raw[4], s_tp_raw[5], s_tp_raw[6], gpio_get_level(PIN_TP_INT));
            frames = 0;
            t_log = xTaskGetTickCount();
        }
        vTaskDelay(1);
    }
}


// ---- MJPEG clip (main/clip.bin, made by tools/make_clip.py), played on a button ----
extern const uint8_t clip_start[] asm("_binary_clip_bin_start");
extern const uint8_t clip_end[] asm("_binary_clip_bin_end");
static lv_image_dsc_t s_frame_dsc;
static lv_obj_t *s_video_img;
static lv_timer_t *s_video_timer;
static const uint8_t *s_frame_ptr;

static void video_stop(void)
{
    lv_timer_pause(s_video_timer);
    lv_obj_add_flag(s_video_img, LV_OBJ_FLAG_HIDDEN);
}

static void video_tick(lv_timer_t *t)
{
    (void)t;
    if (s_frame_ptr >= clip_end) {      // clip finished: back to the menu, no loop
        video_stop();
        return;
    }
    uint32_t sz;
    memcpy(&sz, s_frame_ptr, 4);
    s_frame_dsc.data = s_frame_ptr + 4;
    s_frame_dsc.data_size = sz;
    s_frame_ptr += 4 + sz;
    lv_image_cache_drop(&s_frame_dsc);
    static int n;
    if (n++ < 3) {
        lv_image_header_t h;
        lv_result_t r = lv_image_decoder_get_info(&s_frame_dsc, &h);
        cdc_log("video frame size=%u info=%d %dx%d", (unsigned)sz, (int)r, (int)h.w, (int)h.h);
    }
    lv_image_set_src(s_video_img, &s_frame_dsc);
}

static void video_btn_cb(lv_event_t *e)
{
    (void)e;
    s_frame_ptr = clip_start;
    lv_obj_remove_flag(s_video_img, LV_OBJ_FLAG_HIDDEN);
    lv_timer_reset(s_video_timer);
    lv_timer_resume(s_video_timer);
}

static void video_img_cb(lv_event_t *e)     // tap during playback = stop
{
    (void)e;
    video_stop();
}

static void video_button_create(lv_obj_t *scr)
{
    s_frame_ptr = clip_start;
    s_frame_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    s_frame_dsc.header.cf = LV_COLOR_FORMAT_RAW;
    s_frame_dsc.header.w = 466;
    s_frame_dsc.header.h = 466;
    s_video_img = lv_image_create(scr);
    lv_obj_align(s_video_img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(s_video_img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_video_img, video_img_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(s_video_img, LV_OBJ_FLAG_HIDDEN);
    s_video_timer = lv_timer_create(video_tick, 125, NULL);
    lv_timer_pause(s_video_timer);

    // reuse the "Touch me" button of the exported screen
    lv_obj_t *b = NULL;
    for (uint32_t i = 0; i < lv_obj_get_child_count(scr); i++) {
        lv_obj_t *c = lv_obj_get_child(scr, i);
        if (lv_obj_check_type(c, &lv_button_class)) { b = c; break; }
    }
    cdc_log("video: button %s", b ? "found" : "NOT found");
    if (!b) return;
    lv_obj_add_event_cb(b, video_btn_cb, LV_EVENT_CLICKED, NULL);
}

// ---- Digital caliper gauge: ONE round gauge, 0 at the top, +-150 mm = one full turn ----
// Positive fills clockwise (blue), negative counter-clockwise (red). The colour depends on the POSITION on the
// ring (light near 0, darker and darker with the distance from the top). The ring is drawn by a single custom
// draw callback (CAL_SEGS fixed-colour slices + two round caps) and only the small area around the moving tip is
// invalidated, so very little has to be re-rendered and sent over DSI.
#define UI_CALIPER      1
#define CALIPER_MAX_MM  150     // one full turn = 150 mm
#define CAL_SEGS        72
#define CAL_SEG_DEG     (360 / CAL_SEGS)
#define CAL_SIZE        450
#define CAL_WIDTH       30
#define CAL_POS_LIGHT   0x2EB4FF
#define CAL_POS_DARK    0x001C6B
#define CAL_NEG_LIGHT   0xFF3B30
#define CAL_NEG_DARK    0x5A0000

extern const uint8_t value_font_start[] asm("_binary_value_font_ttf_start");
extern const uint8_t value_font_end[] asm("_binary_value_font_ttf_end");
static lv_obj_t *s_cal_gauge, *s_cal_val, *s_cal_unit;
volatile int g_unit_inch;   // 0 = mm, 1 = inch: toggled by a short press of the power-on switch (c2.c)
static int s_cal_inch = -1;
static int s_cal_neg, s_cal_fill, s_cal_v = 1 << 30;      // direction, filled degrees from the top

static lv_color_t caliper_col(int neg, int dist_deg)        // colour at a distance from the top, in degrees
{
    return lv_color_mix(lv_color_hex(neg ? CAL_NEG_LIGHT : CAL_POS_LIGHT),     // dark at 0 mm, light at +-150 mm
                        lv_color_hex(neg ? CAL_NEG_DARK : CAL_POS_DARK),
                        (uint8_t)((90 + dist_deg * 3 / 4) * 255 / 360));   // starts at the former 1/4 colour (not too dark)
}

static void caliper_pt(const lv_area_t *c, int deg, int *x, int *y)   // point on the ring centre line
{
    float r = (CAL_SIZE - CAL_WIDTH) / 2.0f, t = (float)deg * (float)M_PI / 180.0f;
    *x = (c->x1 + c->x2) / 2 + (int)lroundf(r * sinf(t));
    *y = (c->y1 + c->y2) / 2 - (int)lroundf(r * cosf(t));
}

static void caliper_disc(lv_layer_t *layer, const lv_area_t *c, int deg, lv_color_t col)
{
    int x, y;
    caliper_pt(c, deg, &x, &y);
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = col;
    d.radius = LV_RADIUS_CIRCLE;
    lv_area_t a = {x - CAL_WIDTH / 2, y - CAL_WIDTH / 2, x + CAL_WIDTH / 2 - 1, y + CAL_WIDTH / 2 - 1};
    lv_draw_rect(layer, &d, &a);
}

static void caliper_draw_cb(lv_event_t *e)
{
    lv_layer_t *layer = lv_event_get_layer(e);
    lv_obj_t *obj = lv_event_get_target_obj(e);
    lv_area_t c;
    lv_obj_get_coords(obj, &c);
    const lv_area_t *clip = &layer->_clip_area;
    int cx = (c.x1 + c.x2) / 2, cy = (c.y1 + c.y2) / 2, ro = CAL_SIZE / 2, ri = ro - CAL_WIDTH;

    // nothing of the ring in this clip (e.g. only the central label is being redrawn)?
    int fx = LV_MAX(LV_ABS(clip->x1 - cx), LV_ABS(clip->x2 - cx)), fy = LV_MAX(LV_ABS(clip->y1 - cy), LV_ABS(clip->y2 - cy));
    if (fx * fx + fy * fy < ri * ri) return;

    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.center.x = cx;
    d.center.y = cy;
    d.radius = ro;
    d.width = CAL_WIDTH;
    d.color = lv_color_hex(0x262626);                 // grey track
    d.start_angle = 0;
    d.end_angle = 359;
    lv_draw_arc(layer, &d);

    int fs = s_cal_neg ? 360 - s_cal_fill : 0, fe = s_cal_neg ? 360 : s_cal_fill;
    for (int k = 0; k < CAL_SEGS && s_cal_fill > 0; k++) {
        int lo = k * CAL_SEG_DEG, hi = lo + CAL_SEG_DEG;
        int s = fs > lo ? fs : lo, en = fe < hi ? fe : hi;
        if (en <= s) continue;
        int mid = lo + CAL_SEG_DEG / 2;
        d.color = caliper_col(s_cal_neg, s_cal_neg ? 360 - mid : mid);
        d.start_angle = (s + 270) % 360;                         // LVGL: 0 = 3 o'clock, clockwise
        d.end_angle = (en + 270 + (en < 360 ? 1 : 0)) % 360;     // 1 degree overlap hides the seam
        lv_draw_arc(layer, &d);
    }
    if (s_cal_fill > 0) {
        caliper_disc(layer, &c, 0, caliper_col(s_cal_neg, 0));
        caliper_disc(layer, &c, s_cal_neg ? 360 - s_cal_fill : s_cal_fill, caliper_col(s_cal_neg, s_cal_fill));
    }
}

// invalidate the box around the ring between two angles (clockwise from the top)
static void caliper_inv(int d1, int d2)
{
    lv_area_t c, a;
    lv_obj_get_coords(s_cal_gauge, &c);
    int x, y, step = 5;
    a.x1 = a.y1 = 10000;
    a.x2 = a.y2 = -10000;
    for (int d = d1; d <= d2 + step - 1; d += step) {
        caliper_pt(&c, d < d2 ? d : d2, &x, &y);
        a.x1 = LV_MIN(a.x1, x); a.y1 = LV_MIN(a.y1, y);
        a.x2 = LV_MAX(a.x2, x); a.y2 = LV_MAX(a.y2, y);
        if (d >= d2) break;
    }
    lv_area_t r = {a.x1 - CAL_WIDTH - 2, a.y1 - CAL_WIDTH - 2, a.x2 + CAL_WIDTH + 2, a.y2 + CAL_WIDTH + 2};
    lv_obj_invalidate_area(s_cal_gauge, &r);
}

static void caliper_set_mm100(int v)       // v in 1/100 mm, -15000..15000
{
    const int lim = CALIPER_MAX_MM * 100;
    if (v < -lim) v = -lim;
    if (v > lim) v = lim;
    int neg = v < 0;
    int a = neg ? -v : v;
    int fill = (int)((float)a * 360.0f / (float)lim + 0.5f);      // degrees filled from the top

    if (neg != s_cal_neg || fill != s_cal_fill) {
        int on = s_cal_neg, of = s_cal_fill;
        s_cal_neg = neg;
        s_cal_fill = fill;
        if (on != neg || of == 0 || fill == 0) {                  // direction / start cap changed: whole filled span
            caliper_inv(0, of > fill ? of : fill);
            if (on != neg) caliper_inv(0, fill);
        } else {                                                  // only the part between the old and new tip
            int lo = of < fill ? of : fill, hi = of < fill ? fill : of;
            caliper_inv(neg ? 360 - hi : lo, neg ? 360 - lo : hi);
        }
    }
    int inch = g_unit_inch;
    if (v != s_cal_v || inch != s_cal_inch) {
        s_cal_v = v;
        if (inch != s_cal_inch) lv_label_set_text(s_cal_unit, inch ? "in" : "mm");
        s_cal_inch = inch;
        if (inch) {                                               // 1/100 mm -> 1/1000 inch (25.4 mm per inch)
            int mi = (a * 100 + 127) / 254;
            lv_label_set_text_fmt(s_cal_val, "%s%d.%03d", neg ? "-" : "", mi / 1000, mi % 1000);
        } else {
            lv_label_set_text_fmt(s_cal_val, "%s%d.%02d", neg ? "-" : "", a / 100, a % 100);
        }
    }
}

#define CALIPER_DEMO 0   // 1: fake measurement sweeping 0 -> +150 -> 0 -> -150 -> 0 mm, 24 s per cycle; 0: real AS5311 reading
static void caliper_demo_cb(lv_timer_t *t)
{
    (void)t;
#if CALIPER_DEMO
    float ph = (float)lv_tick_get() / 24000.0f * 2.0f * (float)M_PI;
    caliper_set_mm100((int)(sinf(ph) * CALIPER_MAX_MM * 100));
#else
    caliper_set_mm100(encoder_mm100());
#endif
}

// ---- Battery: 1S LiPo on GPIO23 (BAT_ADC) through a 200k/200k divider; icon with 5 invisible 20 % steps ----
#define PIN_BAT_ADC      GPIO_NUM_23
#define BAT_DIV          2.0f       // two equal resistors: the pin sees Vbat / 2
#define BAT_CAL          1.0f       // calibration factor: (multimeter volts at +BATT) / (volts printed in the "battery:" log)
#define BAT_LOW_PCT      20         // icon turns red below this
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_adc_cali;
static adc_channel_t s_bat_ch;
static lv_obj_t *s_bat_body, *s_bat_nub, *s_bat_fill;
static float s_bat_v;               // filtered battery voltage
static int s_bat_level = -1, s_bat_red = -1;
#define BAT_W 44                    // icon geometry (px)
#define BAT_H 22
#define BAT_PAD 3

static int bat_percent(float v)     // typical 1S LiPo discharge curve
{
    static const float tv[] = { 3.27f, 3.61f, 3.69f, 3.71f, 3.73f, 3.75f, 3.77f, 3.79f, 3.80f, 3.82f, 3.84f,
                                3.85f, 3.87f, 3.91f, 3.95f, 3.98f, 4.02f, 4.08f, 4.11f, 4.15f, 4.20f };
    static const int tp[] = { 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };
    const int n = sizeof(tv) / sizeof(tv[0]);
    if (v <= tv[0]) return 0;
    if (v >= tv[n - 1]) return 100;
    for (int i = 1; i < n; i++)
        if (v <= tv[i]) return tp[i - 1] + (int)((v - tv[i - 1]) / (tv[i] - tv[i - 1]) * (tp[i] - tp[i - 1]));
    return 100;
}

static void bat_adc_init(void)
{
    adc_unit_t unit;
    if (adc_oneshot_io_to_channel(PIN_BAT_ADC, &unit, &s_bat_ch) != ESP_OK) { cdc_log("battery: GPIO23 is not an ADC pin"); return; }
    adc_oneshot_unit_init_cfg_t uc = { .unit_id = unit };
    if (adc_oneshot_new_unit(&uc, &s_adc) != ESP_OK) { cdc_log("battery: ADC init failed"); s_adc = NULL; return; }
    adc_oneshot_chan_cfg_t cc = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    adc_oneshot_config_channel(s_adc, s_bat_ch, &cc);
    adc_cali_curve_fitting_config_t cf = { .unit_id = unit, .chan = s_bat_ch, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    if (adc_cali_create_scheme_curve_fitting(&cf, &s_adc_cali) != ESP_OK) { s_adc_cali = NULL; cdc_log("battery: no ADC calibration, using raw scale"); }
}

static float bat_read_volts(void)    // average of 32 samples
{
    if (!s_adc) return 0;
    long sum = 0;
    int n = 0;
    for (int i = 0; i < 32; i++) {
        int raw;
        if (adc_oneshot_read(s_adc, s_bat_ch, &raw) == ESP_OK) { sum += raw; n++; }
    }
    if (!n) return 0;
    int raw = (int)(sum / n), mv;
    if (!s_adc_cali || adc_cali_raw_to_voltage(s_adc_cali, raw, &mv) != ESP_OK) mv = raw * 3300 / 4095;
    return (float)mv * 0.001f * BAT_DIV * BAT_CAL;
}

static lv_obj_t *s_bat_bolt;
static int s_bat_chg = -1;

// Show a battery state: pct 0..100 in 5 invisible steps of 20 %, red at 20 % or less, green with a bolt while charging.
static void bat_apply(int pct, int charging)
{
    int level = charging ? 5 : (pct + 10) / 20;
    int red = !charging && pct <= BAT_LOW_PCT;
    if (level == s_bat_level && red == s_bat_red && charging == s_bat_chg) return;
    s_bat_level = level;
    s_bat_red = red;
    s_bat_chg = charging;
    lv_color_t col = charging ? lv_color_hex(0x2E9E58) : red ? lv_color_hex(0xC04444) : lv_color_hex(0x707070);   // muted: stays in the background
    lv_obj_set_style_border_color(s_bat_body, col, 0);
    lv_obj_set_style_bg_color(s_bat_nub, col, 0);
    lv_obj_set_style_bg_color(s_bat_fill, col, 0);
    int inner = BAT_W - 2 * BAT_PAD - 4;                    // fill area inside the 2 px border
    lv_obj_set_width(s_bat_fill, level * inner / 5);
    lv_obj_set_style_bg_opa(s_bat_fill, level ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    if (charging) lv_obj_remove_flag(s_bat_bolt, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(s_bat_bolt, LV_OBJ_FLAG_HIDDEN);
}

#define BAT_SIM 0       // 1: slowly cycle charging, 100, 80, 60, 40, 20 % (demo); 0: show the real battery

static void bat_sim_cb(lv_timer_t *t)
{
    (void)t;
    static const int pct[] = { -1, 100, 80, 60, 40, 20 };    // -1 = charging
    static int i;
    bat_apply(pct[i] < 0 ? 0 : pct[i], pct[i] < 0);
    i = (i + 1) % (int)(sizeof(pct) / sizeof(pct[0]));
}

static void bat_timer_cb(lv_timer_t *t)
{
    (void)t;
    float v = bat_read_volts();
    if (v < 1.0f) return;                                   // nothing connected / ADC failed
    s_bat_v = s_bat_v < 1.0f ? v : s_bat_v * 0.8f + v * 0.2f;
    int pct = bat_percent(s_bat_v);
    static int last_log;
    if (++last_log >= 5) { last_log = 0; cdc_log("battery: %.2f V -> %d %%", s_bat_v, pct); }
#if !BAT_SIM
    bat_apply(pct, 0);
#endif
}

static void battery_create(lv_obj_t *scr)
{
    s_bat_body = lv_obj_create(scr);
    lv_obj_set_size(s_bat_body, BAT_W, BAT_H);
    lv_obj_align(s_bat_body, LV_ALIGN_CENTER, -2, 118);
    lv_obj_set_style_bg_opa(s_bat_body, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_bat_body, 2, 0);
    lv_obj_set_style_radius(s_bat_body, 5, 0);
    lv_obj_set_style_pad_all(s_bat_body, 0, 0);
    lv_obj_remove_flag(s_bat_body, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    s_bat_nub = lv_obj_create(scr);
    lv_obj_set_size(s_bat_nub, 4, 10);
    lv_obj_align_to(s_bat_nub, s_bat_body, LV_ALIGN_OUT_RIGHT_MID, 1, 0);
    lv_obj_set_style_border_width(s_bat_nub, 0, 0);
    lv_obj_set_style_radius(s_bat_nub, 2, 0);
    lv_obj_remove_flag(s_bat_nub, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    s_bat_fill = lv_obj_create(s_bat_body);
    lv_obj_set_height(s_bat_fill, BAT_H - 2 * BAT_PAD - 4);
    lv_obj_set_width(s_bat_fill, 0);
    lv_obj_align(s_bat_fill, LV_ALIGN_LEFT_MID, BAT_PAD - 1, 0);
    lv_obj_set_style_border_width(s_bat_fill, 0, 0);
    lv_obj_set_style_radius(s_bat_fill, 2, 0);
    lv_obj_set_style_pad_all(s_bat_fill, 0, 0);
    lv_obj_remove_flag(s_bat_fill, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    s_bat_bolt = lv_label_create(s_bat_body);
    lv_label_set_text(s_bat_bolt, LV_SYMBOL_CHARGE);
    lv_obj_set_style_text_font(s_bat_bolt, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_bat_bolt, lv_color_black(), 0);
    lv_obj_center(s_bat_bolt);
    lv_obj_add_flag(s_bat_bolt, LV_OBJ_FLAG_HIDDEN);
    bat_adc_init();
    bat_timer_cb(NULL);
    lv_timer_create(bat_timer_cb, 1000, NULL);
#if BAT_SIM
    bat_sim_cb(NULL);
    lv_timer_create(bat_sim_cb, 2500, NULL);
#endif
}

static lv_obj_t *caliper_create(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    s_cal_gauge = lv_obj_create(scr);
    lv_obj_set_size(s_cal_gauge, CAL_SIZE, CAL_SIZE);
    lv_obj_center(s_cal_gauge);
    lv_obj_set_style_bg_opa(s_cal_gauge, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_cal_gauge, 0, 0);
    lv_obj_set_style_pad_all(s_cal_gauge, 0, 0);
    lv_obj_remove_flag(s_cal_gauge, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);   // display only
    lv_obj_add_event_cb(s_cal_gauge, caliper_draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    s_cal_val = lv_label_create(scr);
    lv_font_t *big = lv_tiny_ttf_create_data(value_font_start, value_font_end - value_font_start, 68);
    lv_obj_set_style_text_font(s_cal_val, big ? big : &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_cal_val, lv_color_white(), 0);
    lv_obj_align(s_cal_val, LV_ALIGN_CENTER, 0, 0);

    s_cal_unit = lv_label_create(scr);
    lv_label_set_text(s_cal_unit, "mm");
    lv_obj_set_style_text_font(s_cal_unit, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_cal_unit, lv_color_hex(0x7A7A7A), 0);
    lv_obj_align(s_cal_unit, LV_ALIGN_CENTER, 0, 75);

    battery_create(scr);

    caliper_set_mm100(0);
    lv_timer_create(caliper_demo_cb, 33, NULL);
    return scr;
}

// ---- 4-way swipe navigation: the finger drags the next screen in; pre-rendered images, no LVGL redraw ----
// The home screen (gauge) is rendered by LVGL as usual (and mirrored into s_shadow by the flush callback). The four
// menus are rendered ONCE into PSRAM images. While the finger drags, and during the spring that finishes the move,
// each frame is composed straight from those images into the panel's GRAM: no rendering, only PSRAM reads and DSI.
// The incoming screen slides over the outgoing one, which moves more slowly (parallax).
enum { D_LEFT, D_RIGHT, D_UP, D_DOWN };       // swipe direction; menu d is reached by swiping d
#define NAV_PARALLAX  0.30f
#define NAV_START_PX  14
#define IMG_BYTES     (LCD_H * LCD_V * 2)
#define PIN_TE        GPIO_NUM_13

static uint16_t *s_menu_img[4], *s_home_img;
static lv_obj_t *s_home_scr, *s_menu_scr[4];
static int s_cur = -1;                         // -1 = home, 0..3 = menu d
static bool s_te_ok;
static uint16_t s_row[LCD_H] __attribute__((aligned(4)));
static int g_nav_frames, g_nav_us;

static lv_obj_t *nav_menu_create(const char *title, const char *arrow, uint32_t col)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(col), 0);
    lv_obj_set_style_bg_grad_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_grad_dir(scr, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *a = lv_label_create(scr);
    lv_label_set_text(a, arrow);
    lv_obj_set_style_text_font(a, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(a, lv_color_white(), 0);
    lv_obj_align(a, LV_ALIGN_CENTER, 0, -30);
    lv_obj_t *t = lv_label_create(scr);
    lv_label_set_text(t, title);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(t, lv_color_white(), 0);
    lv_obj_align(t, LV_ALIGN_CENTER, 0, 35);
    return scr;
}

static void te_probe(void)
{
    gpio_config_t te = { .pin_bit_mask = 1ULL << PIN_TE, .mode = GPIO_MODE_INPUT };
    gpio_config(&te);
    int64_t t0 = esp_timer_get_time(), first = 0, last = 0;
    int edges = 0, prev = gpio_get_level(PIN_TE);
    while (esp_timer_get_time() - t0 < 200000) {
        int l = gpio_get_level(PIN_TE);
        if (l && !prev) {
            int64_t t = esp_timer_get_time();
            if (!edges) first = t;
            last = t;
            edges++;
        }
        prev = l;
    }
    s_te_ok = edges >= 3;
    cdc_log("TE: %d rising edges in 200 ms%s", edges, s_te_ok ? "" : " (no tearing sync)");
    if (s_te_ok) cdc_log("TE period %d us", (int)((last - first) / (edges - 1)));
}

static void te_wait(void)                      // start the frame right after the panel's vertical sync
{
    if (!s_te_ok) return;
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(PIN_TE)) if (esp_timer_get_time() - t0 > 30000) return;
    while (!gpio_get_level(PIN_TE)) if (esp_timer_get_time() - t0 > 30000) return;
}

// One frame: the incoming image B has travelled o pixels over the outgoing image A in direction dir.
static void nav_frame(const uint16_t *A, const uint16_t *B, int dir, int o)
{
    if (o < 0) o = 0;
    if (o > LCD_H) o = LCD_H;
    int ao = (int)((float)o * NAV_PARALLAX);
    te_wait();
    int64_t t0 = esp_timer_get_time();
    set_window(0, 0, LCD_H - 1, LCD_V - 1);
    dsi_begin();
    for (int y = 0; y < LCD_V; y++) {
        const uint16_t *src;
        const uint16_t *a = A + y * LCD_H, *b = B + y * LCD_H;
        switch (dir) {
        case D_LEFT:
            memcpy(s_row, a + ao, (LCD_H - o) * 2);
            memcpy(s_row + (LCD_H - o), b, o * 2);
            src = s_row;
            break;
        case D_RIGHT:
            memcpy(s_row, b + (LCD_H - o), o * 2);
            memcpy(s_row + o, a + (o - ao), (LCD_H - o) * 2);
            src = s_row;
            break;
        case D_UP:
            src = (y < LCD_V - o) ? A + (y + ao) * LCD_H : B + (y - (LCD_V - o)) * LCD_H;
            break;
        default: // D_DOWN
            src = (y < o) ? B + (LCD_V - o + y) * LCD_H : A + (y - ao) * LCD_H;
            break;
        }
        dsi_pixels(src, LCD_H);
    }
    g_nav_us += (int)(esp_timer_get_time() - t0);
    g_nav_frames++;
}

static int nav_progress(int dir, int x0, int y0)     // finger travel along the swipe direction
{
    int p = dir == D_LEFT ? x0 - s_tx : dir == D_RIGHT ? s_tx - x0 : dir == D_UP ? y0 - s_ty : s_ty - y0;
    return p < 0 ? 0 : (p > LCD_H ? LCD_H : p);
}

// Drag + release spring. Returns true if the move completed (the other screen is now current).
static bool nav_run(int dir, int x0, int y0)
{
    const uint16_t *A = s_cur < 0 ? s_shadow : s_menu_img[s_cur];
    const uint16_t *B = s_cur < 0 ? s_menu_img[dir] : s_home_img;
    float o = 0, v = 0;
    int64_t tp = esp_timer_get_time();
    // 1. the finger drags (the displayed offset chases the finger a little behind it: smooth)
    while (s_tpress) {
        touch_poll();
        int64_t tn = esp_timer_get_time();
        float dt = (float)(tn - tp) * 1e-6f;
        tp = tn;
        float target = (float)nav_progress(dir, x0, y0), prev = o;
        o += (target - o) * (1.0f - expf(-dt / 0.035f));
        if (dt > 0) v = 0.7f * v + 0.3f * (o - prev) / dt;
        nav_frame(A, B, dir, (int)(o + 0.5f));
        taskYIELD();
    }
    // 2. released: critically damped spring to the end (or back), keeping the speed of the finger
    bool done = o > 0.40f * LCD_H || v > 500.0f;
    if (v < -500.0f) done = false;
    float T = done ? (float)LCD_H : 0.0f;
    const float w = 15.0f;
    tp = esp_timer_get_time();
    for (;;) {
        int64_t tn = esp_timer_get_time();
        float dt = (float)(tn - tp) * 1e-6f;
        if (dt > 0.04f) dt = 0.04f;
        tp = tn;
        float acc = -w * w * (o - T) - 2.0f * w * v;
        v += acc * dt;
        o += v * dt;
        if (o > LCD_H) { o = LCD_H; if (v > 0) v = 0; }
        if (o < 0) { o = 0; if (v < 0) v = 0; }
        bool settled = fabsf(o - T) < 0.6f && fabsf(v) < 10.0f;
        nav_frame(A, B, dir, settled ? (int)T : (int)(o + 0.5f));
        if (settled) break;
        taskYIELD();
    }
    return done;
}

// Called from the LVGL task: starts a drag when the finger moved enough in an allowed direction.
static void nav_poll(void)
{
    static bool was;
    static int x0, y0;
    touch_poll();
    if (s_tpress && !was) { x0 = s_tx; y0 = s_ty; }
    was = s_tpress;
    if (!s_tpress || !s_shadow) return;
    int dx = s_tx - x0, dy = s_ty - y0, dir;
    if (LV_ABS(dx) >= LV_ABS(dy) && LV_ABS(dx) > NAV_START_PX) dir = dx < 0 ? D_LEFT : D_RIGHT;
    else if (LV_ABS(dy) > LV_ABS(dx) && LV_ABS(dy) > NAV_START_PX) dir = dy < 0 ? D_UP : D_DOWN;
    else return;
    if (s_cur >= 0 && dir != (s_cur ^ 1)) return;           // in a menu: only the way back
    int from = s_cur;
    if (nav_run(dir, x0, y0)) {
        if (from < 0) {
            memcpy(s_home_img, s_shadow, IMG_BYTES);
            s_cur = dir;
            lv_screen_load(s_menu_scr[dir]);
        } else {
            s_cur = -1;
            lv_screen_load(s_home_scr);
        }
        lv_obj_invalidate(lv_screen_active());               // LVGL redraws the same picture: nothing visible changes
    }
    was = s_tpress;
    cdc_log("nav: %d frames, avg %d us/frame", g_nav_frames, g_nav_frames ? g_nav_us / g_nav_frames : 0);
    g_nav_frames = g_nav_us = 0;
}

static void nav_init(lv_obj_t *home)
{
    s_shadow = heap_caps_aligned_alloc(64, IMG_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_home_img = heap_caps_aligned_alloc(64, IMG_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    for (int i = 0; i < 4; i++) s_menu_img[i] = heap_caps_aligned_alloc(64, IMG_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_shadow || !s_home_img || !s_menu_img[0] || !s_menu_img[1] || !s_menu_img[2] || !s_menu_img[3]) {
        cdc_log("nav: no PSRAM for the screen images, swipe disabled");
        s_shadow = NULL;
        lv_screen_load(home);
        return;
    }
    s_home_scr = home;
    s_menu_scr[D_LEFT] = nav_menu_create("MENU DROITE", LV_SYMBOL_RIGHT, 0x1E6FD9);   // reached by swiping left
    s_menu_scr[D_RIGHT] = nav_menu_create("MENU GAUCHE", LV_SYMBOL_LEFT, 0x8A2BE2);
    s_menu_scr[D_UP] = nav_menu_create("MENU BAS", LV_SYMBOL_DOWN, 0x1FA855);
    s_menu_scr[D_DOWN] = nav_menu_create("MENU HAUT", LV_SYMBOL_UP, 0xE0801A);
    s_capture = true;                                                   // render each menu once, into its image
    for (int i = 0; i < 4; i++) {
        lv_screen_load(s_menu_scr[i]);
        lv_obj_invalidate(s_menu_scr[i]);
        lv_refr_now(NULL);
        memcpy(s_menu_img[i], s_shadow, IMG_BYTES);
    }
    s_capture = false;
    lv_screen_load(home);
    cdc_log("nav: screens ready, PSRAM free %u KB", (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
}

// Heartbeat: the activity LED toggles at 2 Hz as long as FreeRTOS is alive (independent of the display).
static void heartbeat_task(void *arg)
{
    for (int on = 1;; on ^= 1) {
        gpio_set_level(PIN_LED, on);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// ESP8684 handling in its own task, started before the LCD so a display stall cannot block it.
static void c2_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    // 1 power-on, 3 software, 4 panic, 5 interrupt WDT, 6 task WDT, 7 other WDT, 9 brownout
    cdc_log("reset reason: %d", (int)esp_reset_reason());
#if C2_PROBE
    c2_probe(C2_FLASH);   // test of the ESP8684 link (see c2.c); C2_FLASH 1 also flashes the c2_blink image
#else
    c2_run();             // release the ESP8684 so it boots its own firmware from flash
#endif
    vTaskDelete(NULL);
}

void app_main(void)
{
    gpio_config_t led = { .pin_bit_mask = 1ULL << PIN_LED, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&led);
    gpio_set_level(PIN_LED, 1);
    xTaskCreate(heartbeat_task, "heartbeat", 2048, NULL, 10, NULL);   // above the LVGL task so it keeps blinking if the display stalls
    power_boot();   // takes the power latch (PWR_OFF high) at once and plays the power-on tune (see c2.c)
    encoder_init();   // AS5311 on GPIO33/37 (PCNT); needs the UART console off, see sdkconfig.defaults

    // USB first so a bootloader request works even if the LCD fails.
    esp_err_t err = usb_start();
    if (err != ESP_OK) ESP_LOGE(TAG, "USB init failed: %s", esp_err_to_name(err));
    xTaskCreate(c2_task, "c2", 8192, NULL, 2, NULL);

    err = lcd_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LCD init failed: %s (no retry, USB stays active)", esp_err_to_name(err));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(300));

    // RAMW/RAMWC must be sent in MIPI HS format (datasheet): DCS long writes in HS (0 = HS, 1 = LP).
    MIPI_DSI_HOST.cmd_mode_cfg.dcs_lw_tx = 0;
    MIPI_DSI_HOST.cmd_mode_cfg.ack_rqst_en = 0;   // no bus turn-around after each packet (see dsi_pixels)
    // The power IC for ELVDD/ELVSS is driven by the CH13613 over SWIRE: force it on (0x6C = 0xA5),
    // full brightness (0x51 = 0xFF). Without this the panel stays black.
    {
        const uint8_t sw = 0xA5, dbv = 0xFF;
        esp_lcd_panel_io_tx_param(s_io, 0x6C, &sw, 1);
        esp_lcd_panel_io_tx_param(s_io, 0x51, &dbv, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // The touch controller is only powered/usable once the panel power is on, so it is brought up after
    // the LCD. Keep this short (quick probe of 0x5A only): a multi-second bring-up between the LCD init
    // and the first drawing made the DSI host stall.
    vTaskDelay(pdMS_TO_TICKS(1000));   // let the panel-side supplies settle before resetting the touch chip
    touch_init();

    te_probe();
#if !USE_LVGL
    hw_test_run();   // never returns
    return;
#endif
    lv_init();
    lv_tick_set_cb(lvgl_tick_cb);
    lv_display_t *disp = lv_display_create(LCD_H, LCD_V);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    void *b1 = heap_caps_malloc(LVGL_BUF_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    void *b2 = heap_caps_malloc(LVGL_BUF_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!b1 || !b2) { ESP_LOGE(TAG, "no memory for the LVGL draw buffers"); return; }
    lv_display_set_buffers(disp, b1, b2, LVGL_BUF_BYTES, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp, lvgl_flush_cb);
    lv_display_add_event_cb(disp, lvgl_round_cb, LV_EVENT_INVALIDATE_AREA, NULL);

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read_cb);

#if __has_include("esp32p4_ui.h")
    // UI exported by the LVGL Pro Editor from ../ui/esp32p4_ui (start screen: screens/screen1.xml)
    esp32p4_ui_init("");
    lv_screen_load(UI_START_SCREEN_CREATE());
    video_button_create(lv_screen_active());
#if UI_CALIPER
    nav_init(caliper_create());
#endif
#else
#if UI_MIRE
    mire_create();   // built-in test pattern, tap to change
#else
    ui_create();     // built-in arc / ball demo
#endif
#endif
    xTaskCreate(lvgl_task, "lvgl", 8192, NULL, 4, NULL);
    cdc_log("LVGL %d.%d.%d running", LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH);
}
