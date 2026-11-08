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

/* ---- Voltage States ---- */
typedef enum {
    VOLT_STATE_NORMAL = 0,
    VOLT_STATE_WARN,
    VOLT_STATE_CRITICAL,
    VOLT_STATE_VDDA_FAIL
} VoltState;

/* ---- LED & Buzzer Manager ---- */
void led_set(bool on); // Controls On-Board PC13
void app_set_voltage_state(VoltState state_6V, VoltState state_BEC);
void app_status_tick(void);

/* ---- Voltage failsafe latch ---- */
void app_set_volt_failsafe(bool on);
bool app_volt_failsafe_active(void);

/* ---- Throttle soft-start reset ---- */
void app_reset_throttle_ramp(void);

#endif /* INC_APP_H_ */
