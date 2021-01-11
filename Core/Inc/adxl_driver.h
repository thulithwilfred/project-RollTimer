/*
 * adxl_driver.h
 *
 *  Created on: Sep 13, 2020
 *      Author: Wilfred MK
 */
#ifndef INC_ADXL_DRIVER_H_
#define INC_ADXL_DRIVER_H_

#include <math.h>

uint8_t data_rec[6];
int16_t x,y, z;

short int MODE;
float angle, phi, gYZAngle, gXZAngle;
float gX, gY, gZ, gAvg, gAvgPast;


int tap, dbleTap, doubleTapDetect, singleTapDetect, overrideFlag, adxl_enable;

char buffX[12];
char buffY[12];
char buffZ[12];
char buffMode[12];

char guiBuffX[12];
char guiBuffY[12];
char guiBuffZ[12];
char guiMODE[3];
char guiBuffMode[4];

#define adxl_i2c_address 0x53<<1

void adxl343_write_i2c(uint8_t reg, uint8_t value, I2C_HandleTypeDef hi2c1);

void adxl343_read_tap(I2C_HandleTypeDef hi2c1);

void adxl343_read_i2c(uint8_t reg, uint8_t numberofbytes, I2C_HandleTypeDef hi2c1);

void adxl343_i2c_init(I2C_HandleTypeDef hi2c1);

void adxl343_read_XYZ(I2C_HandleTypeDef hi2c1);

void set_mode(void);

#endif /* INC_ADXL_DRIVER_H_ */
