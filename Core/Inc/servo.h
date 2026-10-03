/*
 * servo.h
 *
 *  Created on: Oct 3, 2026
 *      Author: 17049
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include <stdint.h>
void servo_init(void);               /* 上电:全部输出脚 = 输入下拉,不输出 */
void servo_set_us(int out, int us);  /* us>0:输出脉宽(µs);us<=0:停脉冲 */

#endif /* INC_SERVO_H_ */
