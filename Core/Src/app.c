#include "app.h"
#include "servo.h"
#include "crsf.h"
#include "main.h"

#define NUM_OUTPUTS 8
enum { fsaNoPulses = 0, fsaHold = 1 };                  /* 与原库枚举值一致 */
static const int OUTPUT_MAP[NUM_OUTPUTS] = { 1, 2, 3, 4, 6, 7, 8, 12 };  /* 负数=反相 */
static const int OUTPUT_FAILSAFE[NUM_OUTPUTS] = {
    1500, 1500, 988, 1500, fsaHold, fsaHold, fsaHold, fsaNoPulses };

void packet_channels(void) {
    for (int out = 0; out < NUM_OUTPUTS; out++) {
        int ch = OUTPUT_MAP[out], us;
        if (ch > 0) us = (int)crsf_get_channel(ch);
        else        us = 3000 - (int)crsf_get_channel(-ch);   /* 反相:1500 镜像 */
        servo_set_us(out, us);
    }
}

void failsafe_apply(void) {
    for (int out = 0; out < NUM_OUTPUTS; out++) {
        if (OUTPUT_FAILSAFE[out] == fsaNoPulses)      servo_set_us(out, 0);
        else if (OUTPUT_FAILSAFE[out] != fsaHold)     servo_set_us(out, OUTPUT_FAILSAFE[out]);
        /* fsaHold:什么都不做 */
    }
}

void led_set(bool on)
{
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

