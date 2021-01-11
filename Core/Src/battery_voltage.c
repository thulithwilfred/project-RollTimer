/*
 * battery_voltage.c
 *
 *  Created on: Oct 8, 2020
 *      Author: Wilfred Mallawa K
 */

#include "battery_voltage.h"
#include "math.h"

/* Reads and updates the battery Voltage */
void read_update_battery(uint32_t adcRead) {

	voltage = ((adcRead * 3.3) / 4096 ) * 1.303;

	voltFrac = modf(voltage, &voltInt);

	snprintf(batteryBuff, 6,"%d.%02dV", (int)voltInt, (int)(voltFrac * 100));
}

