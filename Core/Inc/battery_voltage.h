/*
 * battery_voltage.h
 *
 *  Created on: Oct 8, 2020
 *      Author: Wilfred Mallawa K
 */

#ifndef INC_BATTERY_VOLTAGE_H_
#define INC_BATTERY_VOLTAGE_H_

#include "main.h"

/* Variables */
char batteryBuff[12];
double voltage, voltFrac, voltInt;

void read_update_battery(uint32_t adcRead);


#endif /* INC_BATTERY_VOLTAGE_H_ */
