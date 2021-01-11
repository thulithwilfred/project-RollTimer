/*
 * adxl_driver.c
 *
 *  Created on: Sep 13, 2020
 *      Author: Wilfred MK
 */
#include "main.h"
#include "adxl_driver.h"


/* Detects taps fom the accelorometer and sets internal parameters */
void adxl343_read_tap(I2C_HandleTypeDef hi2c1) {
	data_rec[0] = 0;
	adxl343_read_i2c(0x30, 1, hi2c1);

	tap = data_rec[0] & 64;
	dbleTap = data_rec[0] & 32;

	if(dbleTap && tap) {
		doubleTapDetect = 1;
	} else {
		doubleTapDetect = 0;
	}

	if (tap && dbleTap == 0) {
		singleTapDetect = 1;
	} else {
		singleTapDetect = 0;
	}
}


/* Read X, Y, Z Values from ADXl343 and Save to internal buffer,
 * Calculate X-Y Angle
 * Set Mode based on read values.
 */
void adxl343_read_XYZ(I2C_HandleTypeDef hi2c1) {

      /*Register 0x32 to Register 0x37 are eight bits
	each and hold the output data for each axis */

	  adxl343_read_i2c(0x32, 6, hi2c1);

	  /*Register 0x32 and
	  Register 0x33 hold the output data for the x-axis */
	  x = (data_rec[1] << 8) | data_rec[0];

	  /*  Register 0x34 and
	  Register 0x35 hold the output data for the y-axis */
	  y = (data_rec[3] << 8) | data_rec[2];

	  /*  Register 0x36
	 and Register 0x37 hold the output data for the z-axis.*/
	  z = (data_rec[5] << 8) | data_rec[4];



	  gX = 0.0078 * x;  //* 7.8/1000 Scale Factor At 4g measurement
	  gY = 0.0078 * y;
	  gZ = 0.0078 * z;

	  //gAvg = (gX + gY + gZ) / 3;
	  gAvg = sqrt(pow(gX,2) + pow(gY,2) + pow(gZ,2)) ;

	  /* Calculate X-Y Relative Angle
	   -1 Set Reference Direction X RIGHT Y UP */
	  angle = 1 * (atan2(gX, gY) * 180) / M_PI;

	  gYZAngle = -1 * (((atan2(gY, gZ) * 180) / M_PI));
	  gXZAngle = -1 * (((atan2(gX, gZ) * 180) / M_PI));

	  snprintf(buffX, 12,"X:%d", x); //Convert to string for printing to LCD
	  snprintf(buffY, 12, "Y:%d", y);
	  snprintf(buffZ,12, "Z:%d", z);

	  snprintf(guiBuffX, 12,"$X%d", (int)(gX * 100)); //Convert to string for printing to GUI
	  snprintf(guiBuffY, 12, "$Y%d", (int)(gY * 100));
	  snprintf(guiBuffZ,12, "$Z%d", (int)(gZ * 100));

	  /* Determine the Operation Mode for the current cycle (250ms Intervals),
	  based on previously calculated angle and Z value. */
	  set_mode();
}

/*
 * Sets operating mode based on positional XY angle and Z vector
 */
void set_mode(void) {

	if (angle >= -50 && angle <= 35 && (MODE != 5)) {
		//MODE 1 STOPWATCG
		MODE = 1;
		guiBuffMode[0] = '$';
		guiBuffMode[1] = 'M';
		guiBuffMode[2] = '1';

	}

	if (angle >= 35 && angle < 130 && (MODE != 5)) {
		//MODE TEMP HUM
		MODE = 2;
		guiBuffMode[0] = '$';
		guiBuffMode[1] = 'M';
		guiBuffMode[2] = '2';
	}

	if (((angle >= 130 && angle <= 180) || (angle >= -180 && angle <= -130)) && (MODE != 5)) {
		//MODE 3 Voltage
		MODE = 3;
		guiBuffMode[0] = '$';
		guiBuffMode[1] = 'M';
		guiBuffMode[2] = '3';
	}

	if (angle >= -130 && angle <= -50  && (MODE != 5)) {
		//MODE Clock With Alarm
		MODE = 4;
		guiBuffMode[0] = '$';
		guiBuffMode[1] = 'M';
		guiBuffMode[2] = '4';
	}

	if (MODE == 5) {
		guiBuffMode[0] = '$';
		guiBuffMode[1] = 'M';
		guiBuffMode[2] = '5';
	}
}

/*
 * Write to ADXL343 registers to set operational registers on I2C
 */
void adxl343_write_i2c(uint8_t reg, uint8_t value, I2C_HandleTypeDef hi2c1) {

	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_I2C_Master_Transmit (&hi2c1, adxl_i2c_address, data, 2, 10);

}

/*
 * Wrapper read function, to read specific register values on the ADXL343 - I2C
 */
void adxl343_read_i2c(uint8_t reg, uint8_t byteCount, I2C_HandleTypeDef hi2c1) {

	HAL_I2C_Mem_Read(&hi2c1, adxl_i2c_address, reg, 1, data_rec, byteCount, 100);

}

/*
 * Clear and Set init operation registers on the ADXL343 - I2C
 */
void adxl343_i2c_init(I2C_HandleTypeDef hi2c1) {

	//This Register is read only and holds 0xE5 for testing.
	adxl343_read_i2c(0x00, 1, hi2c1);

	//Reset POWER_CTL Register
	adxl343_write_i2c(0x2d, 0, hi2c1);

	/* Set POWER_CTL Register (Measure Bit SET)
		measuring mode */
	adxl343_write_i2c(0x2d, 0x08, hi2c1); //measure bit-1

	/* Tap Detection Config */
	adxl343_write_i2c(0x2A, 0x01, hi2c1); //Enable Tap on Z Axis

	adxl343_write_i2c(0x1D, 40, hi2c1);  //Tap Threshold 2.5g (2.5/62.5mg)

	adxl343_write_i2c(0x21, 32, hi2c1);  //Tap Duration 0.2Secs, 625uS register compare

	adxl343_write_i2c(0x22, 80, hi2c1);  //Double Tap Duration 0.1Secs, 1.25ms register compare

	adxl343_write_i2c(0x23, 240, hi2c1); //Double Tap Delay Window 0.3s , 1.25ms register compare

	adxl343_write_i2c(0x2F, 0x0, hi2c1); //Enable all Interrupts to Pin 1

	adxl343_write_i2c(0x2E, 0x60, hi2c1); //Enable Interrupt registers for single and double tap.

	/* Sets the DATA_FORMAT Register,
	 	 Formats the XYZ Data with 4g Range, and left justified MSB*/
	//adxl343_write_i2c(0x31, 0x05, hi2c1);
	adxl343_write_i2c(0x31, 0x01, hi2c1);

}










