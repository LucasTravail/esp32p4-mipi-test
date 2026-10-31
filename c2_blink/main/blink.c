// ESP8684 firmware: open Wi-Fi access point (no password) "Caliper-C2" + web page at http://192.168.4.1/ + activity LED (GPIO18).
// Each press on the web button sends the line "$BUZZ" on UART0 (GPIO20 -> P4 GPIO19); the P4 then sounds its buzzer (main/c2.c).
// Bluetooth LE: advertises as "Caliper-C2" (connectable, no services of our own); a BLE connect sends "$CONN" and a disconnect "$DISC"
// on the same UART (the P4 plays a tune on each). Any generic BLE scanner app (nRF Connect, LightBlue) can connect.
// LED: 2 Hz blink = access point up, steady while at least one station is associated. Console/logs are off: UART0 is the P4 link.
// Flashed from the ESP32-P4 (see ../main/c2.c); built with c2_blink/build.ps1 -> main/c2_blink.bin (merged image at 0x0).
#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define PIN_LED   GPIO_NUM_18
#define AP_SSID   "Caliper-C2"
#define AP_CHAN   6

static volatile int s_clients;
static volatile int s_press;

static const char PAGE[] =
    "<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Caliper</title><style>body{font-family:sans-serif;text-align:center;background:#111;color:#eee;margin-top:15vh}"
    "button{font-size:2em;padding:.8em 1.6em;border:0;border-radius:.6em;background:#2a8;color:#fff;touch-action:manipulation}"
    "button:active{background:#186}.l{display:inline-block;width:1.3em;height:1.3em;border-radius:50%;background:#444;vertical-align:middle;margin:0 .5em}"
    ".on{background:#3d3;box-shadow:0 0 .6em #3d3}</style></head><body><h1>Caliper</h1>"
    "<button onclick=\"fetch('/buzz',{method:'POST'}).then(()=>{n.textContent=++c})\">Buzzer</button>"
    "<p>Appuis : <span id=n>0</span></p>"
    "<p><button style='background:#b33;font-size:1.3em' onclick=\"if(confirm('Eteindre la carte ?'))fetch('/off',{method:'POST'})\">Eteindre</button></p>"
    "<h3>Entrees</h3>"
    "<p>Main switch (GPIO0)<span id=m class=l></span><small id=mv>?</small></p>"
    "<p>Power on (GPIO18)<span id=p class=l></span><small id=pv>?</small></p>"
    "<script>var c=0,n=document.getElementById('n');"
    "function lamp(i,v){document.getElementById(i).className='l'+(v==0?' on':'');"
    "document.getElementById(i+'v').textContent=v<0?'?':(v==0?'appuye (niveau 0)':'relache (niveau 1)')}"
    "setInterval(function(){fetch('/state').then(function(r){return r.json()}).then(function(s){lamp('m',s.m);lamp('p',s.p)})"
    ".catch(function(){lamp('m',-1);lamp('p',-1)})},250)</script></body></html>";

static volatile int s_in_main = -1, s_in_pwr = -1;   // levels of the P4 inputs GPIO0 / GPIO18 as reported by the P4 (-1 = unknown)

static esp_err_t state_get(httpd_req_t *r)
{
    char b[48];
    int n = snprintf(b, sizeof(b), "{\"m\":%d,\"p\":%d}", s_in_main, s_in_pwr);
    httpd_resp_set_type(r, "application/json");
    return httpd_resp_send(r, b, n);
}

// Lines from the P4: "$IN m=<level> p=<level>" (levels of GPIO0 and GPIO18, 0 = pressed).
static void p4_rx_task(void *arg)
{
    char line[40];
    int n = 0;
    for (;;) {
        uint8_t ch;
        if (uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(200)) != 1) continue;
        if (ch == '\n') {
            line[n] = 0;
            int m, p;
            if (sscanf(line, "$IN m=%d p=%d", &m, &p) == 2) { s_in_main = m; s_in_pwr = p; }
            n = 0;
        } else if (n < (int)sizeof(line) - 1) {
            line[n++] = (char)ch;
        }
    }
}

static esp_err_t page_get(httpd_req_t *r)
{
    httpd_resp_set_type(r, "text/html");
    return httpd_resp_send(r, PAGE, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t buzz_post(httpd_req_t *r)
{
    uart_write_bytes(UART_NUM_0, "$BUZZ\n", 6);
    s_press = 4;                                // LED feedback: off for 4 x 50 ms
    return httpd_resp_send(r, "ok", 2);
}

static esp_err_t off_post(httpd_req_t *r)
{
    uart_write_bytes(UART_NUM_0, "$OFF\n", 5);   // the P4 pulses its PWR_OFF pin
    return httpd_resp_send(r, "ok", 2);
}

static esp_err_t no_content(httpd_req_t *r)     // favicon etc.
{
    return httpd_resp_send(r, NULL, 0);
}

static void web_start(void)
{
    uart_config_t uc = { .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                         .stop_bits = UART_STOP_BITS_1, .source_clk = UART_SCLK_DEFAULT };
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uc);
    uart_set_pin(UART_NUM_0, GPIO_NUM_20, GPIO_NUM_19, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);   // TXD0 -> P4 GPIO19, RXD0 <- P4 GPIO20
    httpd_handle_t h = NULL;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    if (httpd_start(&h, &cfg) != ESP_OK) return;
    httpd_uri_t u_page = { .uri = "/", .method = HTTP_GET, .handler = page_get };
    httpd_uri_t u_buzz = { .uri = "/buzz", .method = HTTP_POST, .handler = buzz_post };
    httpd_uri_t u_off = { .uri = "/off", .method = HTTP_POST, .handler = off_post };
    httpd_register_uri_handler(h, &u_off);
    httpd_uri_t u_state = { .uri = "/state", .method = HTTP_GET, .handler = state_get };
    httpd_register_uri_handler(h, &u_state);
    xTaskCreate(p4_rx_task, "p4_rx", 3072, NULL, 4, NULL);
    httpd_uri_t u_ico = { .uri = "/favicon.ico", .method = HTTP_GET, .handler = no_content };
    httpd_register_uri_handler(h, &u_page);
    httpd_register_uri_handler(h, &u_buzz);
    httpd_register_uri_handler(h, &u_ico);
}

static void ble_advertise(void);
static int s_ble_init = -99, s_ble_sync, s_ble_adv = -99, s_ble_addr = -99;
static uint8_t s_own_addr_type;

static int ble_gap_cb(struct ble_gap_event *ev, void *arg)
{
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0) uart_write_bytes(UART_NUM_0, "$CONN\n", 6);
        else ble_advertise();
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        uart_write_bytes(UART_NUM_0, "$DISC\n", 6);
        ble_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_advertise();
        break;
    }
    return 0;
}

static void ble_advertise(void)
{
    const char *name = ble_svc_gap_device_name();
    struct ble_hs_adv_fields f = { 0 };
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    f.name = (const uint8_t *)name;
    f.name_len = strlen(name);
    f.name_is_complete = 1;
    ble_gap_adv_set_fields(&f);
    struct ble_gap_adv_params p = { 0 };
    p.conn_mode = BLE_GAP_CONN_MODE_UND;
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    s_ble_adv = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &p, ble_gap_cb, NULL);
}

static void ble_on_sync(void)
{
    s_ble_sync = 1;
    s_ble_addr = ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    ble_advertise();
}

static void ble_host_task(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_start(void)
{
    s_ble_init = nimble_port_init();
    if (s_ble_init != ESP_OK) return;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("Caliper-C2");
    nimble_port_freertos_init(ble_host_task);
}

static void wifi_event_cb(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == WIFI_EVENT_AP_STACONNECTED) s_clients++;
    else if (id == WIFI_EVENT_AP_STADISCONNECTED && s_clients > 0) s_clients--;
}

static void wifi_ap_start(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();     // DHCP server on 192.168.4.1
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_cb, NULL);

    wifi_config_t ap = { 0 };
    strcpy((char *)ap.ap.ssid, AP_SSID);
    ap.ap.ssid_len = strlen(AP_SSID);
    ap.ap.channel = AP_CHAN;
    ap.ap.authmode = WIFI_AUTH_OPEN;        // no password for now
    ap.ap.max_connection = 4;
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap);
    esp_wifi_start();
}

void app_main(void)
{
    gpio_config_t o = { .pin_bit_mask = 1ULL << PIN_LED, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&o);
    gpio_set_level(PIN_LED, 1);
    wifi_ap_start();
    web_start();
    ble_start();
    for (int tick = 0;; tick++) {               // 50 ms ticks
        if (tick % 60 == 0) {                   // every 3 s: BLE status for the P4 log (the C2 has no console)
            char st[80];
            int n = snprintf(st, sizeof(st), "$BLE init=%d sync=%d addr=%d adv=%d type=%d heap=%u\n", s_ble_init, s_ble_sync,
                             s_ble_addr, s_ble_adv, s_own_addr_type, (unsigned)esp_get_free_heap_size());
            uart_write_bytes(UART_NUM_0, st, n);
        }
        int led = s_clients ? 1 : (tick / 5) & 1;
        if (s_press > 0) { s_press--; led = 0; }    // a button press on the web page blanks the LED briefly
        gpio_set_level(PIN_LED, led);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
