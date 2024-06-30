#include "us_timer.h"

static uint8_t us_timer_elapsed = 0;

void usTimer_Start(TIM_HandleTypeDef *htim, uint32_t us)
{
	if (us < 1) return;

	HAL_TIM_Base_Stop_IT(htim);
	__HAL_TIM_SET_COUNTER(htim, 0);
	__HAL_TIM_SET_AUTORELOAD(htim, us);
	HAL_TIM_Base_Start_IT(htim);

	while (us_timer_elapsed == 0)
	{
		asm("nop");
	}
	us_timer_elapsed = 0;
}

/**
 * @brief Call when timer elapsed
 * @param htim
 */
void usTimer_Callback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_Base_Stop_IT(htim);

	us_timer_elapsed = 1;
}
