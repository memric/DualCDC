# STM32 Dual USB CDC

 Dual virtual COM port example project for STM32F103C8T6. 
 
 Contains STM32CubeIDE and STM32CubeMX projects. Blue Pill devboard is used for testing. First virtual COM port emulates on UART1, second on UART2. UART1 may be used for RS485 communication with DE signal assertion. UART3 is used for debug trace.

 Inspired by [GaryLee/stm32f103_dual_vcp](https://github.com/GaryLee/stm32f103_dual_vcp.git)

 ## Pinout
 | Pin  | Pin Nb. | Function  |
|------|---------|-----------|
| PA10 | 31      | USART1_RX |
| PA9  | 30      | USART1_TX |
| PA3  | 13      | USART2_RX |
| PA2  | 12      | USART2_TX |
| PB11 | 22      | USART3_RX |
| PB10 | 21      | USART3_TX |
| PA8  | 29      | RS485_DE  |
| PC13 | 2       | LED       |

