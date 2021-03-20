/****************************************************************************
 * boards/arm/stm32/viewtool-stm32f107/include/board.h
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

#ifndef __BOARDS_ARM_STM32_VIEWTOOL_STM32F107_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_VIEWTOOL_STM32F107_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "stm32_rcc.h"
#include "stm32_sdio.h"
#include "stm32.h"

/* Clocking *****************************************************************/

#if defined(CONFIG_ARCH_CHIP_STM32F107VC)
# include <arch/board/board-stm32f107vct6.h>
#elif defined(CONFIG_ARCH_CHIP_STM32F103VC)
# include <arch/board/board-stm32f103vct6.h>
#else
#  error Unrecognized STM32 chip
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* There are four LEDs on the ViewTool STM32F103/F107 board that can be
 * controlled by software:  LED1 through LED4.  All pulled high and can be
 * illuminated by driving the output to low
 *
 *   LED1 PA6
 *   LED2 PA7
 *   LED3 PB12
 *   LED4 PB13
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_LED4        3
#define BOARD_NLEDS       4

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)
#define BOARD_LED4_BIT    (1 << BOARD_LED4)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/stm32_leds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *      SYMBOL            Val    Meaning                     LED state
 *                                                       LED1 LED2 LED3 LED4
 *      ----------------- ---   -----------------------  ---- ---- ---- ----
 */
#define LED_STARTED       0  /* NuttX has been started   ON   OFF  OFF  OFF   */
#define LED_HEAPALLOCATE  1  /* Heap has been allocated  OFF  ON   OFF  OFF   */
#define LED_IRQSENABLED   2  /* Interrupts enabled       ON   ON   OFF  OFF   */
#define LED_STACKCREATED  3  /* Idle stack created       OFF  OFF  ON   OFF   */
#define LED_INIRQ         4  /* In an interrupt          N/C  N/C  N/C  GLOW  */
#define LED_SIGNAL        4  /* In a signal handler      N/C  N/C  N/C  GLOW  */
#define LED_ASSERTION     4  /* An assertion failed      N/C  N/C  N/C  GLOW  */
#define LED_PANIC         4  /* The system has crashed   N/C  N/C  N/C  FLASH */
#undef  LED_IDLE             /* MCU is is sleep mode         Not used         */

/* After booting, LED1-3 are not longer used by the system and can be used
 * for other purposes by the application (Of course, all LEDs are available
 * to the application if CONFIG_ARCH_LEDS is not defined.
 */

/* Buttons ******************************************************************/

/* All pulled high and will be sensed low when depressed.
 *
 *   SW2 PC11  Needs J42 closed
 *   SW3 PC12  Needs J43 closed
 *   SW4 PA0   Needs J44 closed
 */

#define BUTTON_SW2        0
#define BUTTON_SW3        1
#define BUTTON_SW4        2
#define NUM_BUTTONS       3

#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)
#define BUTTON_SW3_BIT    (1 << BUTTON_SW3)
#define BUTTON_SW4_BIT    (1 << BUTTON_SW4)

#endif /* __BOARDS_ARM_STM32_VIEWTOOL_STM32F107_INCLUDE_BOARD_H */
