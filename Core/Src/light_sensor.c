/*
 * light_sensor.c
 *
 *  Created on: Oct 8, 2020
 *      Author: Wilfred Mallawa K
 */

#include "main.h"
#include "light_sensor.h"

/* Sets brightness levels based on the ADC values from the light sensor */
void read_update_lightsensor(uint32_t adcLightVal) {

	lightVal = adcLightVal;
/* BACKLIGHT OFF
	if (lightVal  < 25) {
		brightLvl = 0;
	} */

	if (lightVal  <= 20) {
		brightLvl = 0;
	}

	if (lightVal  > 20 && lightVal  < 90) {
		brightLvl = 1;
	}

	if (lightVal  > 90 && lightVal  < 140) {
		brightLvl = 2;
	}

	if (lightVal  > 140 && lightVal  < 170) {
		brightLvl = 3;
	}

	if (lightVal  > 170 && lightVal  < 3000) {
		brightLvl = 4;
	}

	if (lightVal  > 3000) {
		brightLvl = 5;
	}

}
