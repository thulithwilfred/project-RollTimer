/*
 * rw_flash_driver.c
 *
 *  Created on: Oct 16, 2020
 *      Author: Wilfred Mallawa K
 */

#include "main.h"
#include "rw_flash_driver.h"
#include "adxl_driver.h"
#include "display_driver.h"
#include  "HTS221_Driver.h"
#include  "dac_internal_driver.h"
#include  "dac_internal_driver.h"
#include "gui_command_modes.h"
#include "clock_stopwatch.h"
#include "battery_voltage.h"
#include "light_sensor.h"

/* Checks for items saved in flash, and will load allowed items from flash to RAM */
void check_flash_save_load(void) {

	if (saveXBMToFlash) {

		saveXBMToFlash = 0;
		convert_to_64bit_arr(imageBmp, sizeof(imageBmp), XBM_FLASH);
		load_flashXBM();

		/* SET FLASH FLAG INDICATES XBM WAS WRITTEN */
		flashSafeXBM = 1;
	}

	if (saveAlarmToFlash) {

		saveAlarmToFlash = 0;

		int mode1 = 0;
		int mode2 = 0;
		int mode3 = 0;
		int mode4 = 0;

		if (!strcmp(alarm1Mode, "AM")) {
			mode1 = 0x00;
		} else {
			mode1 = 0x99;
		}

		if (!strcmp(alarm2Mode, "AM")) {
			mode2 = 0x00;
		} else {
			mode2 = 0x99;
		}

		if (!strcmp(alarm3Mode, "AM")) {
			mode3 = 0x00;
		} else {
			mode3 = 0x99;
		}

		if (!strcmp(alarm4Mode, "AM")) {
			mode4 = 0x00;
		} else {
			mode4 = 0x99;
		}

		alarms[0] = alarm1Hrs << 16 | alarm1Mns << 8 | mode1;
		alarms[1] = alarm2Hrs << 16 | alarm2Mns << 8 | mode2;
		alarms[2] = alarm3Hrs << 16 | alarm3Mns << 8 | mode3;
		alarms[3] = alarm4Hrs << 16 | alarm4Mns << 8 | mode4;

		write_to_flash(0x08040000 , alarms, 0, 2, sizeof(alarms));
		load_alarms();
		set_alarms_from_flash_read();
	}
}
/* Sets alarms to internal values, once read from flash */
void set_alarms_from_flash_read(void) {

		alarm1Hrs = flashReadAlarms[0] >> 16 & 0xFF;
		alarm1Mns = flashReadAlarms[0] >> 8 & 0xFF ;

		if ((uint8_t) flashReadAlarms[0] == 0x00) {
			strcpy(alarm1Mode, "AM");
		} else {
			strcpy(alarm1Mode, "PM");
		}

		alarm2Hrs = flashReadAlarms[1] >> 16 & 0xFF;
		alarm2Mns = flashReadAlarms[1] >> 8 & 0xFF ;

		if ((uint8_t) flashReadAlarms[1] == 0x00) {
			strcpy(alarm2Mode, "AM");
		} else {
			strcpy(alarm2Mode, "PM");
		}

		alarm3Hrs = flashReadAlarms[2] >> 16 & 0xFF;
		alarm3Mns = flashReadAlarms[2] >> 8 & 0xFF ;

		if ((uint8_t) flashReadAlarms[2] == 0x00) {
			strcpy(alarm3Mode, "AM");
		} else {
			strcpy(alarm3Mode, "PM");
		}

		alarm4Hrs = flashReadAlarms[3] >> 16 & 0xFF;
		alarm4Mns = flashReadAlarms[3] >> 8 & 0xFF ;

		if ((uint8_t) flashReadAlarms[3] == 0x00) {
			strcpy(alarm4Mode, "AM");
		} else {
			strcpy(alarm4Mode, "PM");
		}

		snprintf(alarm1Buffer, 10, "%02d:%02d%s", alarm1Hrs, alarm1Mns, alarm1Mode);
		snprintf(alarm2Buffer, 10, "%02d:%02d%s", alarm2Hrs, alarm2Mns, alarm2Mode);
		snprintf(alarm3Buffer, 10, "%02d:%02d%s", alarm3Hrs, alarm3Mns, alarm3Mode);
		snprintf(alarm4Buffer, 10, "%02d:%02d%s", alarm4Hrs, alarm4Mns, alarm4Mode);


		if (alarm1Hrs == 0xFF) {
			snprintf(alarm1Buffer, 12, "Not Set");
		}
		if (alarm2Hrs == 0xFF) {
			snprintf(alarm2Buffer, 12, "Not Set");
		}
		if (alarm3Hrs == 0xFF) {
			snprintf(alarm3Buffer, 12, "Not Set");
		}
		if (alarm4Hrs == 0xFF) {
			snprintf(alarm4Buffer, 12, "Not Set");
		}

}

/* Loads alarms from predefined saved flash addres */
void load_alarms(void) {

	/* Starting Address 0x08040000*/
	int addr = 0;


	for (int i = 0; i < 4; ++i) {
		doubleWord_Alarms = *(__IO uint64_t*)(ALARM_START_ADR + addr);
		flashReadAlarms[i] = doubleWord_Alarms;
		addr += 8;
	}
}

/* Convers a UINT_8 array of arbitrary length into a UINT64_T array for double word flashing */
void convert_to_64bit_arr(uint8_t* buffer, int bufferSize, int bufferType) {

	int j = 0;

	if (bufferType == XBM_FLASH) {
		for (int i = 0; i < bufferSize ; i += 8) {
			flashBufferXBM[j] = (uint64_t)buffer[i]<<56| (uint64_t)buffer[i + 1]<<48| (uint64_t)buffer[i + 2]<<40| (uint64_t)buffer[i + 3]<<32| (uint64_t)buffer[i + 4]<<24| (uint64_t)buffer[i + 5]<<16| (uint64_t)buffer[i + 6]<<8| (uint64_t)buffer[i + 7];
			j++;
		}

		 if (write_to_flash(0x0803F800 , flashBufferXBM, 127, 1, sizeof(flashBufferXBM))) {
			 flashSafeXBM = 1;
		 }
	}

	if (bufferType == WAV_FLASH) {


	}

	if (bufferType == ALARM_FLASH) {
	}
}


/*This function will calculate the amount of pages required, erase them from the starting address and then write
 * the passed in data to flash (data width 64 bits double-word)
 *
 * Param1: Starting Address of where to write
 * Param2: uint64_t data array will hold the data to be written
 * Param3: Page num within bank, (0 - 127)
 * Param4: Flash bank selection (Bank1: 1, Bank2: 2)
 * Param5: sizeof(dataArr), used to calculate the pages required.
 *
 */
uint32_t write_to_flash(uint32_t startAddr, uint64_t* dataArr, int pageNum,int flashBank, int dataSize) {

	static FLASH_EraseInitTypeDef pageEraseInit = {0};
	int writeCount = 0;
	uint32_t flashErr;

	/* 2KB Page Size*/
	/* Calculating Pages Required: Each Index of uint64_t hold 8bytes, each page can hold 2048Bytes */
	pagesRequired = (int)(dataSize / 2048);

	if (pagesRequired == 0) {
		pagesRequired++;
	}

	test = dataSize;
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
	                           FLASH_FLAG_PGAERR |  FLASH_FLAG_PGSERR);

	/* Erase Write Area */
	pageEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	pageEraseInit.Page = pageNum;
	pageEraseInit.NbPages = pagesRequired;

	/* BANK SELECTION */
	if (flashBank == 1) {
		pageEraseInit.Banks = FLASH_BANK_1;
	}
	if (flashBank == 2) {
		pageEraseInit.Banks = FLASH_BANK_2; /* BANK SELECTION */
	}



	/* Enable Flash Control */
	HAL_FLASH_Unlock();


	if (HAL_FLASHEx_Erase(&pageEraseInit, &flashErr) != HAL_OK) {
		/* Erase Error */
		return HAL_FLASH_GetError();
	}

	while(writeCount < dataSize/8) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, startAddr, dataArr[writeCount]) == HAL_OK) {
			startAddr += 8;
			writeCount++;
		} else {
			return HAL_FLASH_GetError();
		}
	}
	HAL_FLASH_Lock();

	//flashRead = *(__IO uint64_t*)0x08040080;
	return 1;
}

/* Loads an XBM from Flash */
void load_flashXBM(void) {
	/* 0x0803F800 Starting address for the XBM IMAGE */
	int j = 0;
	int addr = 0;
	for (int i = 0; i < 128; i++) {

		doubleWord_XBM = *(__IO uint64_t*)(XBM_START_ADR + addr);

		flashReadXBM[j + 7] = doubleWord_XBM;
		flashReadXBM[j + 6] = doubleWord_XBM >> 8;
		flashReadXBM[j + 5] = doubleWord_XBM >> 16;
		flashReadXBM[j + 4] = doubleWord_XBM >> 24;
		flashReadXBM[j + 3] = doubleWord_XBM >> 32;
		flashReadXBM[j + 2] = doubleWord_XBM >> 40;
		flashReadXBM[j + 1] = doubleWord_XBM >> 48;
		flashReadXBM[j + 0] = doubleWord_XBM >> 56;

		j += 8;
		addr += 8;
	}
}

/* Saves an XBM to flash */
void XBM_saved_to_flash (void) {

	load_flashXBM();

	flashSafeXBM = 0;

	for (int i = 0; i < 1024; ++i) {
		if (!flashReadXBM[i]) {
			flashSafeXBM = 1;
			break;
		}
	}
}

