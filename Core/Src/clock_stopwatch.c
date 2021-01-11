/*
 * clock_stopwatch.c
 *
 *  Created on: Sep 24, 2020
 *      Author: Wilfred MK
 */

#include "main.h"
#include "clock_stopwatch.h"
#include "adxl_driver.h"
#include "HTS221_Driver.h"
#include "gui_command_modes.h"
#include "clock_stopwatch.h"
#include  "dac_internal_driver.h"

/* Clears all alarms */
void clear_alarms(void) {

	alarm1Hrs = 0xFF;
	alarm2Hrs= 0xFF;
	alarm3Hrs = 0xFF;
	alarm4Hrs = 0xFF;

	alarm1Mns =  0xFF;
	alarm2Mns = 0xFF;
	alarm3Mns = 0xFF;
	alarm4Mns = 0xFF;

}

/* detects if an alarm has occured, and triggers the alarm tone */
void is_alarm(DAC_HandleTypeDef hdac1) {

	if (alarm1Hrs != 0xFF && alarm1Mns != 0xFF) {
		/* Alarm 1 Set */
		if (!strcmp(AmPmBuffer, alarm1Mode)) {

			if (alarm1Hrs == gH && alarm1Mns == gM && alarm1Set) {

				//HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineDigitalArr, 100, DAC_ALIGN_12B_R);
				play_alarm(hdac1);
				alarmRinging = 1;

			}
		}
	}

	/* Add Alarms 2 - 4 */
	if (alarm2Hrs != 0xFF && alarm2Mns != 0xFF) {
		/* Alarm 1 Set */
		if (!strcmp(AmPmBuffer, alarm2Mode) && alarm2Set) {

			if (alarm2Hrs == gH && alarm2Mns == gM) {

				//HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineDigitalArr, 100, DAC_ALIGN_12B_R);
				play_alarm(hdac1);
				alarmRinging = 1;
			}
		}
	}

	if (alarm3Hrs != 0xFF && alarm3Mns != 0xFF) {
		/* Alarm 1 Set */
		if (!strcmp(AmPmBuffer, alarm3Mode) && alarm3Set) {

			if (alarm3Hrs == gH && alarm3Mns == gM) {

				//HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineDigitalArr, 100, DAC_ALIGN_12B_R);
				play_alarm(hdac1);
				alarmRinging = 1;

			}
		}
	}

	if (alarm4Hrs != 0xFF && alarm4Mns != 0xFF) {
		/* Alarm 1 Set */
		if (!strcmp(AmPmBuffer, alarm4Mode) && alarm4Set) {

			if (alarm4Hrs == gH && alarm4Mns == gM) {

					//HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sineDigitalArr, 100, DAC_ALIGN_12B_R);
					play_alarm(hdac1);
					alarmRinging = 1;

			}
		}
	}

	/* Stop alarm with a tap, only if in mode 4*/
	if (MODE == 4 && singleTapDetect) {
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		alarmRinging = 0;
	}
}

/* Updates the internal clock at a normal second rate */
void update_clock(void) {

	gMs++;

	if (gMs > 1000) {
		gMs = 0;
		gS++;
	}

	if (gS >= 60) {
		gS = 0;
		gM++;
	}

	if (gM >= 60) {
		gM = 0;
		gH++;
	}

	if (gH == 12 && gM == 0 && gS == 0) {
		if (!strcmp(AmPmBuffer,"AM")) {
			strcpy(AmPmBuffer, "PM");
		} else {
			strcpy(AmPmBuffer, "AM");
		}
	}

	if (gH > 12) {
		gH = 1;
	}

	snprintf(realTimeBuffer, 12,"%02d:%02d:%02d", gH, gM, gS);
	//fix_clock_buffer();
}

/* Updates the clock at 60x Speeds */
void update_clock2(void) {

	gMs++;

	if (gMs > 16) {
		gS++;
		gMs = 0;
	}

	if (gS >= 60) {
		gS = 0;
		gM++;
	}

	if (gM >= 60) {
		gM = 0;
		gH++;
	}

	if (gH == 12 && gM == 0 && gS == 0) {
		if (!strcmp(AmPmBuffer,"AM")) {
			strcpy(AmPmBuffer, "PM");
		} else {
			strcpy(AmPmBuffer, "AM");
		}
	}

	if (gH > 12) {
		gH = 1;
	}

	snprintf(realTimeBuffer, 12,"%02d:%02d:%02d", gH, gM, gS);
	//fix_clock_buffer();

}

/* Clears the stopwatch */
void clear_stopwatch(void) {
	swMs = 0;
	swS = 0;
	swM = 0;
	swH = 0;
}

/* Increments the stopwatch at a normal per second rate */
void update_stopWatch(void) {

		swMs++;

		if (swMs >= 1000) {
			swMs = 0;
			swS++;
		}

		if (swS >= 60) {
			swS = 0;
			swM++;
		}

		if (swM >= 60) {
			swM = 0;
			swH++;
		}

		if (swH >= 24) {
			swH = 0;
		}


		snprintf(stopWatchBuffer, 13,"%02d:%02d:%02d:%03d", swH, swM, swS, swMs);


}

/* Increment the stopwatch at a 60x Speed */
void update_stopWatch2(void) {

		swMs++;

		if (swMs > 16) {
			swMs = 0;
			swS++;
		}

		if (swS >= 60) {
			swS = 0;
			swM++;
		}

		if (swM >= 60) {
			swM = 0;
			swH++;
		}

		if (swH >= 24) {
			swH = 0;
		}

		snprintf(stopWatchBuffer, 13,"%02d:%02d:%02d:%03d", swH, swM, swS, swMs);

}


