/****************************************************************************
 * boards/arm/stm32wb/flipperzero/include/board.h
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

#ifndef __BOARDS_ARM_STM32WB_FLIPPERZERO_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32WB_FLIPPERZERO_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* Clocking *****************************************************************/

#include "flipperzero-clocking.h"

/* Do not include STM32WB header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USART */

#define GPIO_USART1_TX GPIO_USART1_TX_2    /* PB6 */
#define GPIO_USART1_RX GPIO_USART1_RX_2    /* PB7 */

/* LEDs */

/* LED index values for use with board_userled() */

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/stm32_autoleds.c.
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        1
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Buttons */

#define BUTTON_SW1        0  /* PC4, needs SB47 close */
#define BUTTON_SW2        1  /* PD0 */
#define BUTTON_SW3        2  /* PD1 */
#define NUM_BUTTONS       3

#define BUTTON_SW1_BIT    (1 << BUTTON_SW1)
#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)
#define BUTTON_SW3_BIT    (1 << BUTTON_SW3)

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
 * Name: stm32wb_board_initialize
 *
 * Description:
 *   All STM32WB architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32wb_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32WB_FLIPPERZERO_INCLUDE_BOARD_H */
