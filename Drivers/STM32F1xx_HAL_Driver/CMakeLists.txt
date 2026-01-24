cmake_minimum_required(VERSION 3.20)

project(target-HAL LANGUAGES C CXX)
 
set(HAL_SOURCES 
	Src/stm32f1xx_hal_cortex.c
	Src/stm32f1xx_hal_dma.c
	Src/stm32f1xx_hal_exti.c
	Src/stm32f1xx_hal_flash_ex.c
	Src/stm32f1xx_hal_flash.c
	Src/stm32f1xx_hal_gpio_ex.c
	Src/stm32f1xx_hal_gpio.c
	Src/stm32f1xx_hal_iwdg.c
	Src/stm32f1xx_hal_pcd_ex.c
	Src/stm32f1xx_hal_pcd.c
	Src/stm32f1xx_hal_pwr.c
	Src/stm32f1xx_hal_rcc_ex.c
	Src/stm32f1xx_hal_rcc.c
	Src/stm32f1xx_hal_tim_ex.c
	Src/stm32f1xx_hal_tim.c
	Src/stm32f1xx_hal_uart.c
	Src/stm32f1xx_hal.c
	Src/stm32f1xx_ll_usb.c
)

set(HAL_INCLUDES
    Inc
)
 
add_library(HAL STATIC ${HAL_SOURCES})
target_include_directories(HAL PUBLIC ${HAL_INCLUDES})