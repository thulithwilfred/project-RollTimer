/*
 * gui_command_modes.c
 *
 *  Created on: Sep 21, 2020
 *      Author: Wilfred MK
 */

#ifndef SRC_GUI_COMMAND_MODES_C_
#define SRC_GUI_COMMAND_MODES_C_

#include "main.h"
#include "display_driver.h"
#include "adxl_driver.h"
#include "HTS221_Driver.h"
#include "u8g2.h"
#include "string.h"
#include "gui_command_modes.h"
#include  "dac_internal_driver.h"
#include "clock_stopwatch.h"
#include "rw_flash_driver.h"

/* Process Received Alarms From GUI */
void receive_alarms(void) {
	if (GUI_MODE == 0x05) {
		/*Alarm 1 MODE*/
		guiTempAlarmHrs[0] = guiAlarmTime[2];
		guiTempAlarmHrs[1] = guiAlarmTime[3];
		guiTempAlarmMns[0] = guiAlarmTime[4];
		guiTempAlarmMns[1] = guiAlarmTime[5];


		if (guiAlarmTime[0] == '1') {
			/* Alarm 1 Enabled */
				if (guiAlarmTime[1] == 'A') {
					alarm1Hrs = atoi(guiTempAlarmHrs);
					alarm1Mns = atoi(guiTempAlarmMns);
					strcpy(alarm1Mode, "AM");
				}

				if (guiAlarmTime[1] == 'P') {
					alarm1Hrs = atoi(guiTempAlarmHrs);
					alarm1Mns = atoi(guiTempAlarmMns);
					strcpy(alarm1Mode, "PM");
				}

				snprintf(alarm1Buffer, 10, "%02d:%02d%s", alarm1Hrs, alarm1Mns, alarm1Mode);

		} else {
			/* Alarm 1 Disabled */
			alarm1Hrs = 0xFF;
			alarm1Mns = 0xFF;
			snprintf(alarm1Buffer, 12, "Not Set");
		}
	}

	if (GUI_MODE == 0x06) {
		/*Alarm 2 MODE*/
		guiTempAlarmHrs[0] = guiAlarmTime[2];
		guiTempAlarmHrs[1] = guiAlarmTime[3];
		guiTempAlarmMns[0] = guiAlarmTime[4];
		guiTempAlarmMns[1] = guiAlarmTime[5];


		if (guiAlarmTime[0] == '1') {
			/* Alarm 2 Enabled */
				if (guiAlarmTime[1] == 'A') {
					alarm2Hrs = atoi(guiTempAlarmHrs);
					alarm2Mns = atoi(guiTempAlarmMns);
					strcpy(alarm2Mode, "AM");
				}

				if (guiAlarmTime[1] == 'P') {
					alarm2Hrs = atoi(guiTempAlarmHrs);
					alarm2Mns = atoi(guiTempAlarmMns);
					strcpy(alarm2Mode, "PM");
				}

				snprintf(alarm2Buffer, 10, "%02d:%02d%s", alarm2Hrs, alarm2Mns, alarm2Mode);

		} else {
			/* Alarm 2 Disabled */
			alarm2Hrs = 0xFF;
			alarm2Mns = 0xFF;
			snprintf(alarm2Buffer, 12, "Not Set");
		}
	}

	if (GUI_MODE == 0x07) {
		/*Alarm 3 MODE*/
		guiTempAlarmHrs[0] = guiAlarmTime[2];
		guiTempAlarmHrs[1] = guiAlarmTime[3];
		guiTempAlarmMns[0] = guiAlarmTime[4];
		guiTempAlarmMns[1] = guiAlarmTime[5];


		if (guiAlarmTime[0] == '1') {
			/* Alarm 3 Enabled */
				if (guiAlarmTime[1] == 'A') {
					alarm3Hrs = atoi(guiTempAlarmHrs);
					alarm3Mns = atoi(guiTempAlarmMns);
					strcpy(alarm3Mode, "AM");
				}

				if (guiAlarmTime[1] == 'P') {
					alarm3Hrs = atoi(guiTempAlarmHrs);
					alarm3Mns = atoi(guiTempAlarmMns);
					strcpy(alarm3Mode, "PM");
				}

				snprintf(alarm3Buffer, 10, "%02d:%02d%s", alarm3Hrs, alarm3Mns, alarm3Mode);

		} else {
			/* Alarm 3 Disabled */
			alarm3Hrs = 0xFF;
			alarm3Mns = 0xFF;
			snprintf(alarm3Buffer, 12, "Not Set");
		}
	}

	if (GUI_MODE == 0x08) {
		/*Alarm 4 MODE*/
		guiTempAlarmHrs[0] = guiAlarmTime[2];
		guiTempAlarmHrs[1] = guiAlarmTime[3];
		guiTempAlarmMns[0] = guiAlarmTime[4];
		guiTempAlarmMns[1] = guiAlarmTime[5];


		if (guiAlarmTime[0] == '1') {
			/* Alarm 4 Enabled */
				if (guiAlarmTime[1] == 'A') {
					alarm4Hrs = atoi(guiTempAlarmHrs);
					alarm4Mns = atoi(guiTempAlarmMns);
					strcpy(alarm4Mode, "AM");
				}

				if (guiAlarmTime[1] == 'P') {
					alarm4Hrs = atoi(guiTempAlarmHrs);
					alarm4Mns = atoi(guiTempAlarmMns);
					strcpy(alarm4Mode, "PM");
				}

				snprintf(alarm4Buffer, 10, "%02d:%02d%s", alarm4Hrs, alarm4Mns, alarm4Mode);

		} else {
			/* Alarm 4 Disabled */
			alarm4Hrs = 0xFF;
			alarm4Mns = 0xFF;
			snprintf(alarm4Buffer, 12, "Not Set");
		}
	}
}

/* Process Received Time From GUI */
void receive_time(void) {

	if (GUI_MODE == 0x04) {
		/* Copy Gui Data To Local Array */
		guiHr[0] = guiTime[0];
		guiHr[1] = guiTime[1];

		guiMin[0] = guiTime[2];
		guiMin[1] = guiTime[3];

		guiSec[0] = guiTime[4];
		guiSec[1] = guiTime[5];

		if (guiTime[6] == 'A') {
			strcpy(AmPmBuffer, "AM");
		}

		if (guiTime[6] == 'P') {
			strcpy(AmPmBuffer, "PM");
		}

		gH = atoi(guiHr);
		gM = atoi(guiMin);
		gS = atoi(guiSec);

	}


	guiTimeSet = 1;

}

/* Sets a mode that expects a specific set of data from the GUI next */
void set_gui_mode_flag(char* commandByte, UART_HandleTypeDef *huart) {

	if (GUI_MODE) {

		receive_time();

		receive_alarms();

		GUI_MODE = 0x00;
		HAL_UART_Receive_IT(huart, (uint8_t* )commandByte, 2);
	}

	if (!strcmp(commandByte, "SF")) {
		GUI_MODE = 0x01;
		HAL_UART_Receive_IT(huart, (uint8_t* )commandByte, 8);

	}

	if (!strcmp(commandByte, "RF")) {
		GUI_MODE = 0x00;
		//HAL_UART_Receive_IT(huart, (uint8_t* )commandByte, 2);
	}

	if (!strcmp(commandByte, "IM")) {
		/* Next Data Set To Receive is to be DMA bitmap of Image */
		GUI_MODE = 0;
		flashSafeXBM = 0;
		HAL_UART_Receive_IT(huart, (uint8_t* )imageBmp, 1024);
	}
	if (!strcmp(commandByte, "TM")) {
		/* Next Data Set To Receive is time HR:MN:SC */
		GUI_MODE = 0x04;

		HAL_UART_Receive_IT(huart, (uint8_t* )guiTime, 7);
	}

	if(!strcmp(commandByte, "I#")) {
		/* Received Image, Save to flash CHECK */
		saveXBMToFlash = 1;
		check_flash_save_load();
		HAL_UART_Receive_IT(huart, (uint8_t* )commandByte, 2);
	}

	if(!strcmp(commandByte, "A#")) {
		/* All Alarm Data Received, Save to Flash */
		saveAlarmToFlash = 1;
		check_flash_save_load();
		HAL_UART_Receive_IT(huart, (uint8_t* )commandByte, 2);
	}

	if (!strcmp(commandByte, "HI")) {
		/* GUI ACK 1s */;
		GUI_MODE = 0;
		guiAckTime = HAL_GetTick();
		HAL_UART_Receive_IT(huart, (uint8_t* )commandByte, 2);

	}

	if (!strcmp(commandByte, "A1")) {
		GUI_MODE = 0x05;
		HAL_UART_Receive_IT(huart, (uint8_t* )guiAlarmTime, 6);
	}

	if (!strcmp(commandByte, "A2")) {
		GUI_MODE = 0x06;
		HAL_UART_Receive_IT(huart, (uint8_t* )guiAlarmTime, 6);
	}

	if (!strcmp(commandByte, "A3")) {
		GUI_MODE = 0x07;
		HAL_UART_Receive_IT(huart, (uint8_t* )guiAlarmTime, 6);
	}

	if (!strcmp(commandByte, "A4")) {
		GUI_MODE = 0x08;
		HAL_UART_Receive_IT(huart, (uint8_t* )guiAlarmTime, 6);
	}
	/* Additional Modes to be added here
	 *
	 *NOTES: FOR THE BITMAP USE DMA INTERRUPT AND SAVE STRAIGH TO BUFFER.
	 *
	 *  */
}

/* Will Be called every 1 second  to send data USART1 TX *
 *
 * The Following Data Buffers are prefixed with "$" which is the GUI data seperator character
 */
void update_gui(UART_HandleTypeDef* huart) {

	/* Acclro Data  4g Normalized Values*/
	HAL_UART_Transmit(huart, (uint8_t*) guiBuffX, strlen(guiBuffX), 10);
	HAL_UART_Transmit(huart, (uint8_t*) guiBuffY, strlen(guiBuffY), 10);
	HAL_UART_Transmit(huart, (uint8_t*) guiBuffZ, strlen(guiBuffZ), 10);

	/* Temp and Hum */
	HAL_UART_Transmit(huart, (uint8_t*) guiTemp, strlen(guiTemp), 10);
	HAL_UART_Transmit(huart, (uint8_t*) guiHum, strlen(guiHum), 10);

	/* Current FSM Mode & Volume Level */
	HAL_UART_Transmit(huart, (uint8_t*) guiBuffMode, strlen(guiBuffMode), 10);
	HAL_UART_Transmit(huart, (uint8_t*) guiBuffVol, strlen(guiBuffVol), 10);

}


#endif /* SRC_GUI_COMMAND_MODES_C_ */
