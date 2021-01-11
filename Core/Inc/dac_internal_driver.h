/*
 * dac_internal_driver.h
 *
 *  Created on: Sep 18, 2020
 *      Author: Wilfred MK
 */

#ifndef INC_DAC_INTERNAL_DRIVER_H_
#define INC_DAC_INTERNAL_DRIVER_H_

#include "main.h"
#include "math.h"

extern SPI_HandleTypeDef hspi2;

float dacScale;
float volLvl;
int volNorm, anglePast;
char volBuffer[4];
char  guiBuffVol[4];
uint8_t potWrite[2];

int time4;

uint32_t dacOut, sineDigitalArr[100];

void play_alarm(DAC_HandleTypeDef hdac1);

void dac_init(DAC_HandleTypeDef hdac1, TIM_HandleTypeDef htim6);

void set_dac_dc(DAC_HandleTypeDef hdac1);

void update_vol(void);

void get_sine_val(void);

#endif /* INC_DAC_INTERNAL_DRIVER_H_ */
