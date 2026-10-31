// AS5311 magnetic linear encoder, incremental (A/B quadrature) output, decoded by the PCNT peripheral (x4, hardware, no CPU load).
// Pole pair length 2 mm, 1024 A/B periods per pole pair -> 4096 edges per 2 mm = 2048 counts per mm (0.488 um per count).
// The Index pulse (GPIO38) is not used. GPIO37/38 are UART0's default pins: the console is disabled in sdkconfig.defaults.
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void cdc_log(const char *fmt, ...);

#define PIN_ENC_A          GPIO_NUM_33
#define PIN_ENC_B          GPIO_NUM_37
#define ENC_COUNTS_PER_MM  2048     // 4096 edges per 2 mm pole pair
#define ENC_ENABLE         1        // 0 = firmware never touches the sensor pins (A/B test of the display glitches)
#define ENC_DIR            1        // set to -1 if the displayed value decreases when the ruler moves the "positive" way

static pcnt_unit_handle_t s_unit;
static volatile int s_zero;

void encoder_init(void)
{
    if (!ENC_ENABLE) return;
    gpio_config_t in = { .pin_bit_mask = (1ULL << PIN_ENC_A) | (1ULL << PIN_ENC_B), .mode = GPIO_MODE_INPUT };   // also frees GPIO37 from UART0
    gpio_config(&in);

    pcnt_unit_config_t uc = { .low_limit = -10000, .high_limit = 10000, .flags.accum_count = 1 };   // accumulates beyond 16 bits
    if (pcnt_new_unit(&uc, &s_unit) != ESP_OK) { cdc_log("encoder: pcnt unit failed"); return; }
    pcnt_glitch_filter_config_t fc = { .max_glitch_ns = 100 };   // edges are >= ~490 ns apart at 1 m/s
    pcnt_unit_set_glitch_filter(s_unit, &fc);

    pcnt_chan_config_t ca = { .edge_gpio_num = PIN_ENC_A, .level_gpio_num = PIN_ENC_B };
    pcnt_chan_config_t cb = { .edge_gpio_num = PIN_ENC_B, .level_gpio_num = PIN_ENC_A };
    pcnt_channel_handle_t cha, chb;
    pcnt_new_channel(s_unit, &ca, &cha);
    pcnt_new_channel(s_unit, &cb, &chb);
    pcnt_channel_set_edge_action(cha, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(cha, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(chb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chb, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_unit_add_watch_point(s_unit, uc.low_limit);    // required by accum_count
    pcnt_unit_add_watch_point(s_unit, uc.high_limit);
    pcnt_unit_enable(s_unit);
    pcnt_unit_clear_count(s_unit);
    pcnt_unit_start(s_unit);
    cdc_log("encoder: AS5311 on A=GPIO%d B=GPIO%d, %d counts/mm", PIN_ENC_A, PIN_ENC_B, ENC_COUNTS_PER_MM);
}

static int encoder_counts(void)
{
    int c = 0;
    if (s_unit) pcnt_unit_get_count(s_unit, &c);
    return c;
}

// Position since the last zero, in 1/100 mm.
int encoder_mm100(void)
{
    int64_t d = (int64_t)(encoder_counts() - s_zero) * ENC_DIR * 100;
    return (int)(d >= 0 ? (d + ENC_COUNTS_PER_MM / 2) / ENC_COUNTS_PER_MM : -((-d + ENC_COUNTS_PER_MM / 2) / ENC_COUNTS_PER_MM));
}

void encoder_zero(void)
{
    s_zero = encoder_counts();
    cdc_log("encoder: zeroed at count %d", s_zero);
}
