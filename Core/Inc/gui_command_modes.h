/*
 * gui_commad_modes.h
 *
 *  Created on: Sep 21, 2020
 *      Author: Wilfred MK
 */

#ifndef INC_GUI_COMMAND_MODES_H_
#define INC_GUI_COMMAND_MODES_H_

/* Private Variables */
/*
 * GUI_MODE USAGE:
 * 					0x00 - NO MODE SET BY GUI
 * 					0x01 -
 * 					0x02 -
 * 					0x03 -
 *
 */
unsigned int GUI_MODE;

uint8_t imageBmp[1024];
uint8_t guiTime[7];
uint8_t guiAlarmTime[6];
char guiTempAlarmHrs[3], guiTempAlarmMns[3];
char realTimeBuffer[9];

int guiTimeSet;

char guiSec[2], guiMin[2], guiHr[2];
int gS, gM, gH, guiAckPast, guiAckTime, imageInterrupt, saveAlarmToFlash ;

void receive_alarms(void);

void receive_time(void);

void set_gui_mode_flag(char* commandByte, UART_HandleTypeDef *huart);

void update_gui(UART_HandleTypeDef* huart);


#endif /* INC_GUI_COMMAD_MODES_H_ */
