/*
 * display_driver.h
 *
 *  Created on: Sep 13, 2020
 *      Author: Kevin Gao
 *      Co Auth: Wilfred MK
 */

#ifndef INC_DISPLAY_DRIVER_H_
#define INC_DISPLAY_DRIVER_H_

#include "u8g2.h"
#include "u8x8.h"

int32_t CH1_DC;
int LCDStatus, demoMode, button, button2, guiConnected, alarmTglScreen, buttonPressed, buttonPast, alarmFlickerEnable, cradleTimerEnable, cradleEnable, cradleTimer;
char clockBuffer[12], weatherIcon[2];



extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;

void cradle_brightness_override(void);

uint8_t byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

uint8_t gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

void lcd_init(u8g2_t* u8g2);

void brighness_control(void);

void update_LCD(u8g2_t* u8g2);

void draw_battIcon(u8g2_t* u8g2);

#endif /* INC_DISPLAY_DRIVER_H_ */
