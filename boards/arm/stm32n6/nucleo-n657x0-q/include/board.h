/****************************************************************************
 * boards/arm/stm32n6/nucleo-n657x0-q/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ARM_STM32N6_NUCLEO_N657X0_Q_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32N6_NUCLEO_N657X0_Q_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Clock tree (single PLL1 fed from the internal HSI):
 *
 *   HSI 64 MHz / M=4 * N=50 = 800 MHz VCO
 *     IC1  /4 = 200 MHz  -> CPU clock (CPUSW)
 *     IC2  /8 = 100 MHz  \
 *     IC6 /12 = 66.7 MHz  > SYSCLK components (SYSSW IC2_IC6_IC11)
 *     IC11 /8 = 100 MHz  /
 *   HPRE /2  = 50 MHz   -> HCLK
 *   PPRE1 /1 = 50 MHz   -> PCLK1
 *   PPRE2 /1 = 50 MHz   -> PCLK2
 *
 * Works with the default VOS SCALE1, no SMPS overdrive required.  Higher
 * CPU frequencies (600/800 MHz) are deferred to a follow-up.
 */

#define STM32_HSI_FREQUENCY     64000000ul

#define STM32_PLL1_M            4
#define STM32_PLL1_N            50
#define STM32_PLL1_IC1_DIV      4

#define STM32_CPUCLK_FREQUENCY  200000000ul
#define STM32_SYSCLK_FREQUENCY  (STM32_CPUCLK_FREQUENCY / 2)
#define STM32_HCLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 4)
#define STM32_PCLK1_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY

/* Timer input clock = SYSCLK (TIMPRE=0 default) */

#define STM32_APB1_TIM_FREQUENCY STM32_SYSCLK_FREQUENCY
#define STM32_APB2_TIM_FREQUENCY STM32_SYSCLK_FREQUENCY

/* I/O voltage domains ******************************************************/

/* GPIO port E (USART1 TX/RX on PE5/PE6) is on the VddIO2 and VddIO3
 * domains; both are wired to the board's 1.8 V rail.  This mask is
 * applied to PWR_SVMCR3 early in boot to mark the supplies valid and
 * select their 1.8 V range.
 */

#define BOARD_PWR_VDDIO  (PWR_SVMCR3_VDDIO2SV    | PWR_SVMCR3_VDDIO3SV | \
                          PWR_SVMCR3_VDDIO2VRSEL | PWR_SVMCR3_VDDIO3VRSEL)

/* LED definitions **********************************************************/

/* The Nucleo-N657X0-Q has three user LEDs (UM3417 silkscreen):
 *
 *   LD5  PG10  Red
 *   LD6  PG0   Green
 *   LD7  PG8   Blue
 *
 * They are not used by the board port unless CONFIG_ARCH_LEDS is defined.
 * In that case the usage by the board port is defined in include/board.h
 * and src/stm32_autoleds.c.  The LEDs are used to encode OS-related events
 * as follows.
 *
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       3

#define BOARD_LED_RED     BOARD_LED1
#define BOARD_LED_GREEN   BOARD_LED2
#define BOARD_LED_BLUE    BOARD_LED3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDS is defined, the LEDs are used to encode OS-related
 * events as follows:
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        Red   Green Blue
 *   ----------------------  --------------------------  ------ ------ ----
 */

#define LED_STARTED        0 /* NuttX has been started   OFF    OFF   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  OFF    OFF   ON   */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON    OFF  */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    ON    ON   */
#define LED_INIRQ          4 /* In an interrupt          N/C    N/C   GLOW */
#define LED_SIGNAL         5 /* In a signal handler      N/C    GLOW  N/C  */
#define LED_ASSERTION      6 /* An assertion failed      GLOW   N/C   GLOW */
#define LED_PANIC          7 /* The system has crashed   Blink  OFF   N/C  */
#define LED_IDLE           8 /* MCU is in sleep mode     ON     OFF   OFF  */

/* Alternate function pin selections ****************************************/

/* USART1 GPIOs *************************************************************/

/* USART1 (Nucleo Virtual Console): PE5=TX (AF7), PE6=RX (AF7)
 * Connected to the on-board ST-Link to provide a Virtual COM Port.
 */

#define GPIO_USART1_TX   GPIO_USART1_TX_1
#define GPIO_USART1_RX   GPIO_USART1_RX_1

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
 * Name: stm32_board_initialize
 *
 * Description:
 *   All STM32N6 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices
 *   have been initialized.
 *
 ****************************************************************************/

void stm32_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32N6_NUCLEO_N657X0_Q_INCLUDE_BOARD_H */
