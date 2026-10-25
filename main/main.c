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
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "lvgl.h"
#if __has_include("esp32p4_ui.h")
#include "esp32p4_ui.h"
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

// ---- LVGL ----------------------------------------------------------------------
// Partial rendering: LVGL draws into two small internal-RAM buffers and flush_cb sends each dirty
// rectangle into the panel's GRAM (DCS RAMW/RAMWC in HS). The panel needs even-aligned windows.
#define DRAW_LINES 40
#define LVGL_BUF_BYTES (LCD_H * DRAW_LINES * 3)

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

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    static uint8_t buf[CHUNK_PX * 3];
    esp_err_t err = set_window(area->x1, area->y1, area->x2, area->y2);
    int total = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    bool first = true;
    while (total > 0 && err == ESP_OK) {
        int n = total < CHUNK_PX ? total : CHUNK_PX;
        for (int i = 0; i < n; i++) {   // LVGL RGB888 is B,G,R in memory; the panel wants R,G,B
            buf[3 * i + 0] = px_map[3 * i + 2];
            buf[3 * i + 1] = px_map[3 * i + 1];
            buf[3 * i + 2] = px_map[3 * i + 0];
        }
        err = esp_lcd_panel_io_tx_param(s_io, first ? 0x2C : 0x3C, buf, n * 3);
        first = false;
        px_map += n * 3;
        total -= n;
    }
    if (err != ESP_OK) ESP_LOGE(TAG, "flush failed: %s", esp_err_to_name(err));
    lv_display_flush_ready(disp);
}

static uint32_t lvgl_tick_cb(void) { return xTaskGetTickCount() * portTICK_PERIOD_MS; }

static void lvgl_task(void *arg)
{
    for (;;) {
        uint32_t wait_ms = lv_timer_handler();
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
#define USE_LVGL 0   // 0 = no-LVGL hardware test, 1 = LVGL UI (mire / editor UI)

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

    // The touch controller is only powered/usable once the panel power is on, so it is brought up after
    // the LCD. Keep this short (quick probe of 0x5A only): a multi-second bring-up between the LCD init
    // and the first drawing made the DSI host stall.
    vTaskDelay(pdMS_TO_TICKS(1000));   // let the panel-side supplies settle before resetting the touch chip
    touch_init();

#if !USE_LVGL
    hw_test_run();   // never returns
    return;
#endif
    lv_init();
    lv_tick_set_cb(lvgl_tick_cb);
    lv_display_t *disp = lv_display_create(LCD_H, LCD_V);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB888);
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
    // UI exported by the LVGL Pro Editor from ../ui (project name esp32p4_ui, start screen main_screen)
    esp32p4_ui_init("");
    lv_screen_load(main_screen_create());
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
