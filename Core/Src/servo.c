#include "servo.h"
#include "main.h"
#define NUM_OUTPUTS 8


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
typedef struct { TIM_HandleTypeDef *htim; uint32_t ch; GPIO_TypeDef *port; uint16_t pin; } ServoPort;

static const ServoPort P[NUM_OUTPUTS] = {
    { &htim2, TIM_CHANNEL_1, GPIOA, GPIO_PIN_15 },
    { &htim2, TIM_CHANNEL_2, GPIOB, GPIO_PIN_3  },
    { &htim2, TIM_CHANNEL_3, GPIOB, GPIO_PIN_10 },
    { &htim2, TIM_CHANNEL_4, GPIOB, GPIO_PIN_11 },
    { &htim3, TIM_CHANNEL_1, GPIOA, GPIO_PIN_6  },
    { &htim3, TIM_CHANNEL_2, GPIOA, GPIO_PIN_7  },
    { &htim3, TIM_CHANNEL_3, GPIOB, GPIO_PIN_0  },
    { &htim3, TIM_CHANNEL_4, GPIOB, GPIO_PIN_1  },
};
static int g_lastUs[NUM_OUTPUTS];   /* 0 = 已停止(无脉冲) */

static void pin_mode(int i, int enable) {
    GPIO_InitTypeDef gi = {0};
    gi.Pin = P[i].pin;
    if (enable) { gi.Mode = GPIO_MODE_AF_PP; gi.Speed = GPIO_SPEED_FREQ_HIGH; }
    else        { gi.Mode = GPIO_MODE_INPUT; gi.Pull = GPIO_PULLDOWN; }
    HAL_GPIO_Init(P[i].port, &gi);
}

void servo_init(void) {            /* 上电先全部不输出 */
    for (int i = 0; i < NUM_OUTPUTS; i++) { pin_mode(i, 0); g_lastUs[i] = 0; }
}

void servo_set_us(int i, int us) {
    if (us > 0) {
        if (g_lastUs[i] == 0) {    /* 从"停止"恢复 → 先接回定时器 */
            pin_mode(i, 1);
            HAL_TIM_PWM_Start(P[i].htim, P[i].ch);
        }
        __HAL_TIM_SET_COMPARE(P[i].htim, P[i].ch, (uint32_t)us);   /* CCR = 脉宽 µs */
    } else {                       /* fsaNoPulses:停脉冲 + 引脚下拉 */
        HAL_TIM_PWM_Stop(P[i].htim, P[i].ch);
        pin_mode(i, 0);
    }
    g_lastUs[i] = us;
}
