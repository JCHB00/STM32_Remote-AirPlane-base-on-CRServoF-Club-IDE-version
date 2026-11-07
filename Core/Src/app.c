#include "app.h"
#include "servo.h"
#include "crsf.h"
#include "main.h"

#define NUM_OUTPUTS 8
#define THROTTLE_OUTPUT_INDEX 2     // Index 2 corresponds to your motor output (Channel 3)
#define THROTTLE_MIN_US       988     // Zero throttle / arming pulse

enum { fsaNoPulses = 0, fsaHold = 1 };                  /* 与原库枚举值一致 */
static const int OUTPUT_MAP[NUM_OUTPUTS] = { 1, 2, 3, 4, 6, 7, 8, 12 };  /* 负数=反相 */
static const int OUTPUT_FAILSAFE[NUM_OUTPUTS] = {
    1500, 1500, 988, 1500, fsaHold, fsaHold, fsaHold, fsaNoPulses };
// Track the current smoothly-ramped throttle value
static int s_current_throttle_us = THROTTLE_MIN_US;
static volatile bool s_volt_failsafe = false;

/* ===================== LED manager ===================== */
static LedMode  s_led_mode = LED_MODE_OFF;
static bool     s_led_pin_on = false;
static uint32_t s_led_next_toggle_ms = 0;
static bool     s_led_pulse_active = false;
static uint32_t s_led_pulse_end_ms = 0;

void led_set(bool on) {
    s_led_pin_on = on;
    /* Active HIGH per PCB schematic */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void app_led_set_mode(LedMode mode) {
    if (mode == s_led_mode) return;
    s_led_mode = mode;
    s_led_next_toggle_ms = HAL_GetTick();
}

void app_led_pulse(uint32_t ms) {
    s_led_pulse_active = true;
    s_led_pulse_end_ms = HAL_GetTick() + ms;
    led_set(true);
}

void app_led_tick(void) {
    uint32_t now = HAL_GetTick();
    if (s_led_pulse_active) {
        if ((int32_t)(now - s_led_pulse_end_ms) >= 0) {
            s_led_pulse_active = false;
            s_led_next_toggle_ms = now;
        } else return;
    }
    switch (s_led_mode) {
    case LED_MODE_OFF:        led_set(false); break;
    case LED_MODE_ON:         led_set(true);  break;
    case LED_MODE_BLINK_SLOW:
        if ((int32_t)(now - s_led_next_toggle_ms) >= 0) {
            led_set(!s_led_pin_on);
            s_led_next_toggle_ms = now + 200; /* 2.5 Hz */
        }
        break;
    case LED_MODE_BLINK_FAST:
        if ((int32_t)(now - s_led_next_toggle_ms) >= 0) {
            led_set(!s_led_pin_on);
            s_led_next_toggle_ms = now + 100; /* 5 Hz */
        }
        break;
    }
}

/* ============== Voltage failsafe latch ============== */
void app_set_volt_failsafe(bool on) {
    if (on && !s_volt_failsafe) s_current_throttle_us = THROTTLE_MIN_US;
    s_volt_failsafe = on;
}
bool app_volt_failsafe_active(void) { return s_volt_failsafe; }
void app_reset_throttle_ramp(void) { s_current_throttle_us = THROTTLE_MIN_US; }

/* ============== Throttle soft-start ============== */
static int apply_throttle_ramp(int target_us) {
    // Maximum microsecond increase per packet update (adjust to control ramp speed)
    // ExpressLRS updates packets frequently (~150Hz / every ~7ms). 
    // A step of 2 to 5 µs per update provides a smooth 0.5s to 1s ramp-up time.
    const int max_ramp_step = 3; 

    if (target_us > s_current_throttle_us) {
        // Ramping up smoothly
        s_current_throttle_us += max_ramp_step;
        if (s_current_throttle_us > target_us) {
            s_current_throttle_us = target_us;
        }
    } 
    else {
        // INSTANT response when reducing throttle or cutting motor for safety
        s_current_throttle_us = target_us;
    }

    return s_current_throttle_us;
}

void packet_channels(void) {
	if (s_volt_failsafe) return; /* Failsafe owns the servos */
    for (int out = 0; out < NUM_OUTPUTS; out++) {
        int ch = OUTPUT_MAP[out], us;
        if (ch > 0) us = (int)crsf_get_channel(ch);
        else        us = 3000 - (int)crsf_get_channel(-ch);   /* 反相:1500 镜像 */

        // Apply soft-start specifically to the throttle output channel
        if (out == THROTTLE_OUTPUT_INDEX) {
            us = apply_throttle_ramp(us);
        }

        servo_set_us(out, us);
    }
}

void failsafe_apply(void) {
    for (int out = 0; out < NUM_OUTPUTS; out++) {
        if (OUTPUT_FAILSAFE[out] == fsaNoPulses)      servo_set_us(out, 0);
        else if (OUTPUT_FAILSAFE[out] != fsaHold)     servo_set_us(out, OUTPUT_FAILSAFE[out]);
        /* fsaHold:什么都不做 */
    }
    s_current_throttle_us = THROTTLE_MIN_US;   /* re-arm ramp for next arm */
}
