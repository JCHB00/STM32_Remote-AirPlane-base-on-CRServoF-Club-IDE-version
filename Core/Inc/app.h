/*
 * app.h
 *
 *  Created on: Oct 3, 2026
 *      Author: 17049
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include <stdbool.h>
#include <stdint.h>
void packet_channels(void);   /* 每帧通道数据 → 8 路输出 */
void failsafe_apply(void);    /* 失控保护动作 */

/* ---- LED manager (single owner of the LED pin) ---- */
typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_ON,
    LED_MODE_BLINK_SLOW,   /* ~2.5 Hz - warning           */
    LED_MODE_BLINK_FAST,   /* ~5 Hz   - critical / vdda   */
} LedMode;

void led_set(bool on);
void app_led_set_mode(LedMode mode);
void app_led_pulse(uint32_t ms);
void app_led_tick(void);

/* ---- Voltage failsafe latch ---- */
void app_set_volt_failsafe(bool on);
bool app_volt_failsafe_active(void);

/* ---- Throttle soft-start reset ---- */
void app_reset_throttle_ramp(void);

#endif /* INC_APP_H_ */
