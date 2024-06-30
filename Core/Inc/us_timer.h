#ifndef US_TIMER_H_
#define US_TIMER_H_

#include "main.h"

uint32_t usTimer_Init(void);
void usTimer_Start(TIM_HandleTypeDef *htim, uint32_t us);
void usTimer_Callback(TIM_HandleTypeDef *htim);

#endif /*US_TIMER_H_*/
