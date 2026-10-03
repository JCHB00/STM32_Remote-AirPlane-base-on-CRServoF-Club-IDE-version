/*
 * crsf.h
 *
 *  Created on: Oct 3, 2026
 *      Author: 17049
 */

#ifndef INC_CRSF_H_
#define INC_CRSF_H_


#include <stdint.h>
#include <stdbool.h>
void    crsf_init(void);                 /* 状态清零 */
void    crsf_rx_byte(uint8_t b);         /* 主循环逐字节喂入 */
void    crsf_check_link(void);           /* 主循环轮询,300ms 超时→失控保护 */
int32_t crsf_get_channel(uint8_t ch1);   /* 1 基,返回 µs */
bool    crsf_is_link_up(void);           /* 调试用 */


#endif /* INC_CRSF_H_ */
