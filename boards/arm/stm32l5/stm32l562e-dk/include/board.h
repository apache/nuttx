/****************************************************************************
 * boards/arm/stm32l5/stm32l562e-dk/include/board.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32L5_STM32L562E_DK_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32L5_STM32L562E_DK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Currently the STM32L562E-DK board support is restricted to running NuttX
 * in the Non-Secure domain together with TrustedFirmware-M (TFM).  In this
 * setup the clock configuration is done by TFM, not by NuttX.  The
 * respective code is in STM32CubeL5/Projects/STM32L562E-DK/Applications/TFM/
 * TFM_SBSFU_Boot/Src/boot_hal.c and configures the clocks as follows:
 *
 *   System Clock source : PLL (MSI)
 *   SYSCLK(Hz)          : 110000000
 *   HCLK(Hz)            : 110000000
 *   AHB Prescaler       : 1
 *   APB1 Prescaler      : 1
 *   APB2 Prescaler      : 1
 *   MSI Frequency(Hz)   : 4000000
 *   PLLM                : 1
 *   PLLN                : 55
 *   PLLP                : 2
 *   PLLQ                : 2
 *   PLLR                : 2
 *   Flash Latency(WS)   : 5
 *   Voltage range       : 0
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * MSI - 4 MHz, autotrimmed via LSE
 * HSE - not installed
 * LSE - 32.768 kHz installed
 */

#define STM32L5_HSI_FREQUENCY     16000000ul
#define STM32L5_LSI_FREQUENCY     32000
#define STM32L5_MSI_FREQUENCY     4000000ul
#define STM32L5_LSE_FREQUENCY     32768

#define STM32L5_SYSCLK_FREQUENCY  110000000ul
#define STM32L5_HCLK_FREQUENCY    STM32L5_SYSCLK_FREQUENCY
#define STM32L5_PCLK1_FREQUENCY   STM32L5_HCLK_FREQUENCY
#define STM32L5_PCLK2_FREQUENCY   (STM32L5_HCLK_FREQUENCY / 1)

/* The timer clock frequencies are automatically defined by hardware.  If the
 * APB prescaler equals 1, the timer clock frequencies are set to the same
 * frequency as that of the APB domain. Otherwise they are set to twice.
 * Note: TIM1,15,16 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32L5_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    STM32L5_HCLK_FREQUENCY
#define BOARD_TIM3_FREQUENCY    STM32L5_HCLK_FREQUENCY
#define BOARD_TIM4_FREQUENCY    STM32L5_HCLK_FREQUENCY
#define BOARD_TIM5_FREQUENCY    STM32L5_HCLK_FREQUENCY
#define BOARD_TIM6_FREQUENCY    STM32L5_HCLK_FREQUENCY
#define BOARD_TIM7_FREQUENCY    STM32L5_HCLK_FREQUENCY
#define BOARD_TIM15_FREQUENCY   STM32L5_HCLK_FREQUENCY
#define BOARD_TIM16_FREQUENCY   STM32L5_HCLK_FREQUENCY
#define BOARD_LPTIM1_FREQUENCY  STM32L5_HCLK_FREQUENCY
#define BOARD_LPTIM2_FREQUENCY  STM32L5_HCLK_FREQUENCY

/* DMA Channel/Stream Selections ********************************************/

/* Alternate function pin selections ****************************************/

/* USART3: Connected to Arduino connector D0/D1 (or to STLink VCP if solder
 * bridges SB123 to SB130 are re-worked accordingly).
 */

#define GPIO_USART1_RX   GPIO_USART1_RX_1    /* PA10 */
#define GPIO_USART1_TX   GPIO_USART1_TX_1    /* PA9  */

/* LED definitions **********************************************************/

/* The STM32L562E-DK board has numerous LEDs but only two, LD9 a Red LED,
 * and LD10 a Green LED, that can be controlled by software.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs
 * in any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2

#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_RED     BOARD_LED2

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_autoleds.c. The LEDs are used to encode OS-
 * related events as follows:
 *
 *
 *   SYMBOL                     Meaning                    LED state
 *                                                        Red   Green
 *   ----------------------  --------------------------  ------ -----
 */
#define LED_STARTED        0 /* NuttX has been started   OFF    OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  ON     OFF  */
#define LED_IRQSENABLED    2 /* Interrupts enabled       ON     ON   */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    ON   */
#define LED_INIRQ          4 /* In an interrupt          GLOW   N/C  */
#define LED_SIGNAL         5 /* In a signal handler      GLOW   N/C  */
#define LED_ASSERTION      6 /* An assertion failed      GLOW   N/C  */
#define LED_PANIC          7 /* The system has crashed   Blink  OFF  */
#define LED_IDLE           8 /* MCU is is sleep mode     N/C    ON   */

/* Thus if the Green LED is statically on, NuttX has successfully booted and
 * is, apparently, idleing.  If the Red LED is flashing at approximately 2Hz,
 * then a fatal error has been detected and the system has halted.
 */

/* Button definitions *******************************************************/

/* The Nucleo-L552ZE supports one button:  Pushbutton B1, labeled "User", is
 * connected to GPIO PC13.
 * A high value will be sensed when the button is pressed.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l5_board_initialize
 *
 * Description:
 *   All STM32L5 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices
 *   have been initialized.
 *
 ****************************************************************************/

void stm32l5_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32L5_STM32L562E_DK_INCLUDE_BOARD_H */
