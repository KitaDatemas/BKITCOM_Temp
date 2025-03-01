/*
 * softwareTimer.c
 *
 *  Created on: Feb 14, 2025
 *      Author: ASUS
 */
#include "softwareTimer.h"

TIM_HandleTypeDef * tim;
uint32_t timerCount [NB_OF_TIMERS], timerFlag [NB_OF_TIMERS];

void timerInit(TIM_HandleTypeDef * htim) {
	tim = htim;
	HAL_TIM_Base_Start_IT(tim);
}

void setTimer(uint32_t timer, uint32_t duration) {
	if (timer < 0 || timer >= NB_OF_TIMERS) 		return;
	timerTick [timer] = 0;
	timerCount [timer] = duration / NB_OF_TIMERS;
	timerFlag [timer] = 0;
}

void timerRun(void) {
	for (int timerIdx = 0; timerIdx < NB_OF_TIMERS; timerIdx++) {
		if (timerCount[timerIdx] > 0) {
			if (--timerCount[timerIdx] == 0) {
				timerFlag[timerIdx] = 1;
			}
		}

	}
}

uint8_t checkTimerFlag (uint32_t timer) {
	if (timer < 0 || timer >= NB_OF_TIMERS)			return 0;
	return timerFlag [timer];
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
	timerRun();
}
