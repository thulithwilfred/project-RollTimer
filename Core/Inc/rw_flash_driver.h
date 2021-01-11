/*
 * rw_flash_driver.h
 *
 *  Created on: Oct 16, 2020
 *      Author: Wilfred Mallawa K
 */

#ifndef INC_RW_FLASH_DRIVER_H_
#define INC_RW_FLASH_DRIVER_H_
#define XBM_FLASH 1
#define WAV_FLASH 2
#define ALARM_FLASH 3
#define XBM_START_ADR 0x0803F800
#define ALARM_START_ADR 0x08040000

int wordLength;
uint64_t flashBuffer[128];


int pagesRequired, test;
uint64_t flashRead;

uint64_t flashBufferXBM[128];
uint64_t flashBufferWAV[256];
uint64_t flashBufferAlarm[1];

uint8_t flashReadXBM[1024];
uint64_t doubleWord_XBM, doubleWord_Alarms;

uint64_t alarms[4];
uint64_t flashReadAlarms[4];

int temp;

/* Flash Save/Load */
int saveXBMToFlash, flashSafeXBM;

uint32_t write_to_flash(uint32_t startAddr, uint64_t* dataArr, int pageNum, int  flashBank, int dataSize);

void check_flash_save_load(void);

void convert_to_64bit_arr(uint8_t* buffer, int bufferSize, int bufferType);

void load_flashXBM(void);

void load_alarms(void);

void set_alarms_from_flash_read(void);

void is_image_saved_to_flash(void);

void XBM_saved_to_flash (void);

#endif /* INC_RW_FLASH_DRIVER_H_ */
