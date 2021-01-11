/*
 * light_sensor.h
 *
 *  Created on: Oct 8, 2020
 *      Author: Wilfred Mallawa K
 */

#ifndef INC_LIGHT_SENSOR_H_
#define INC_LIGHT_SENSOR_H_

#include "main.h"
int lightVal;
float brightLvl;

void read_update_lightsensor(uint32_t adcLightVal);

#endif /* INC_LIGHT_SENSOR_H_ */
