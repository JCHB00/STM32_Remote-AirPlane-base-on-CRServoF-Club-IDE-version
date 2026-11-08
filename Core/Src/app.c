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

/* ===================== Status & Warning Variables ===================== */
static VoltState s_state_6V = VOLT_STATE_NORMAL;
static VoltState s_state_BEC = VOLT_STATE_NORMAL;

/* External hardware functions defined in main.c */
extern void BLUE_LED_ON(void);
extern void BLUE_LED_OFF(void);
extern void GREEN_LED_ON(void);
extern void GREEN_LED_OFF(void);
extern void RED_LED_ON(void);
extern void RED_LED_OFF(void);
extern void BUZZER_ON(void);
extern void BUZZER_OFF(void);

void led_set(bool on) {
    /* Active HIGH per PCB schematic (LED to GND) */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* Called from main.c every 100ms to update the current voltage states */
void app_set_voltage_state(VoltState state_6V, VoltState state_BEC) {
    s_state_6V = state_6V;
    s_state_BEC = state_BEC;
}

/* Called every loop iteration to update LEDs and Buzzer */
void app_status_tick(void) {
    bool link_up = crsf_is_link_up();
    
    bool is_crit = (s_state_6V == VOLT_STATE_CRITICAL) || 
                   (s_state_BEC == VOLT_STATE_CRITICAL) || 
                   (s_state_6V == VOLT_STATE_VDDA_FAIL);
    bool is_warn = (s_state_6V == VOLT_STATE_WARN) || 
                   (s_state_BEC == VOLT_STATE_WARN);

    /* 1. Green LED: ON only if link is up AND voltage is perfectly normal */
    if (link_up && !is_warn && !is_crit) {
        GREEN_LED_ON();
    } else {
        GREEN_LED_OFF();
    }

    /* 2. Blue LED: ON if connection failure (link down) */
    if (!link_up) {
        BLUE_LED_ON();
    } else {
        BLUE_LED_OFF();
    }

    /* 3. Red LED & On-Board PC13: ON if voltage is CRITICAL */
    if (is_crit) {
        RED_LED_ON();
        
        /* Determine which battery failed for the PC13 indicator */
        if (s_state_BEC == VOLT_STATE_CRITICAL || s_state_6V == VOLT_STATE_VDDA_FAIL) {
            led_set(true);  /* PC13 ON: LiPo/BEC fail (takes priority) */
        } else if (s_state_6V == VOLT_STATE_CRITICAL) {
            led_set(false); /* PC13 OFF: 6V Battery Box fail */
        }
    } else {
        RED_LED_OFF();
        led_set(false);
    }

    /* 4. Buzzer: Solid ON if Red (Critical) OR Blue (Link Fail) is active */
    if (is_crit || !link_up) {
        BUZZER_ON();
    } else {
        BUZZER_OFF();
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
