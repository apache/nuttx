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
