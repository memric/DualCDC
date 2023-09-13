#ifndef US_TIMER_H_
#define US_TIMER_H_

#include "main.h"

uint32_t us_timer_init(void);
void us_timer_start(TIM_HandleTypeDef *htim, uint32_t us);
void us_timer_callback(TIM_HandleTypeDef *htim);

#endif /*US_TIMER_H_*/
