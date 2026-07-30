/**
* @file LED.h
 * @brief DM-MC-Board02 onboard WS2812B RGB LED driver by GPIO bit-bang.
 */

#ifndef DM_LED_H
#define DM_LED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void LED_init(void);

void LED_SET_RGB(uint8_t red, uint8_t green, uint8_t blue);

void LED_RED_SET(void); // 数据超出范围
void LED_RED_RESET(void);
void LED_RED_TOGGLE(void);
void LED_RDE_TOGGLE(void);

void LED_GREEN_SET(void); //
void LED_GREEN_RESET(void);
void LED_GREEN_TOGGLE(void);

void LED_BLUE_SET(void); //
void LED_BLUE_RESET(void);
void LED_BLUE_TOGGLE(void);

void LED_SKY_SET(void); // 正常上电
void LED_SKY_RESET(void);
void LED_SKY_TOGGLE(void);

void LED_YELLOW_SET(void); // 串口句柄为NULL
void LED_PURPLE_SET(void); // 正常工作
void LED_PINK_SET(void); // 电机句柄为NULL


void LED_off(void);

#ifdef __cplusplus
}
#endif

#endif /* DM_LED_H */