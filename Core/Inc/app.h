/*
 * app.h
 *
 *  Created on: Oct 3, 2026
 *      Author: 17049
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include <stdbool.h>
void packet_channels(void);   /* 每帧通道数据 → 8 路输出 */
void failsafe_apply(void);    /* 失控保护动作 */
void led_set(bool on);

#endif /* INC_APP_H_ */
