/*
 * clock_stopwatch.h
 *
 *  Created on: Sep 24, 2020
 *      Author: Wilfred MK
 */

#ifndef INC_CLOCK_STOPWATCH_H_
#define INC_CLOCK_STOPWATCH_H_

int swH, swM, swS, swMs;
int gMs;
char AmPmBuffer[3];
char stopWatchBuffer[14];

int alarm1Hrs, alarm2Hrs, alarm3Hrs, alarm4Hrs, alarmSet, startStopWatch, hrsFlag, mnsFlag, scsFlag, defaultClk, doubleTapClear;
int alarm1Mns, alarm2Mns, alarm3Mns, alarm4Mns;
char alarm1Mode[4], alarm2Mode[4], alarm3Mode[4], alarm4Mode[4], alarm1Buffer[14], alarm2Buffer[14], alarm3Buffer[14], alarm4Buffer[14];
int alarm1Set, alarm2Set, alarm3Set, alarm4Set, alarmRinging;

void is_alarm(DAC_HandleTypeDef hdac1);

void clear_alarms(void);

void update_clock(void);

void update_stopWatch(void);

void update_stopWatch2(void);

void update_clock2(void);

void clear_stopwatch(void);

void fix_clock_buffer(void);

#endif /* INC_CLOCK_STOPWATCH_H_ */
