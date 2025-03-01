/*
 * softwareTimer.h
 *
 *  Created on: Feb 14, 2025
 *      Author: ASUS
 */

#ifndef INC_SOFTWARETIMER_H_
#define INC_SOFTWARETIMER_H_
#include "main.h"

#define TIMER_CYCLE 10 /*Timer cycle 10ms*/
#define NB_OF_TIMERS 10

extern TIM_HandleTypeDef * tim;
extern uint32_t timerTick [NB_OF_TIMERS], timerCount [NB_OF_TIMERS], timerFlag [NB_OF_TIMERS];

/*
 * @brief	This is for passing Tim handler variable
 * @param	Tim handler variable
 * @retval	None
 * */
void timerInit(TIM_HandleTypeDef * htim);

/*
 * @brief	This function is used for set software timer
 * @param	timer: Provide software timer to know which timer to be set
 * @param	duration: duration of time
 * @retval	None
 * */
void setTimer(uint32_t timer, uint32_t duration);

/*
 * @brief	Timer run function to be put in timer call back function
 * @retval	None
 * */
void timerRun(void);

/*
 * @brief	Function to check if timer ran out
 * @param	timer: Provide which timer to be checked
 * @retval	TimerFlag value corresponding to the given timer
 * */
uint8_t checkTimerFlag (uint32_t timer);

#endif /* INC_SOFTWARETIMER_H_ */
