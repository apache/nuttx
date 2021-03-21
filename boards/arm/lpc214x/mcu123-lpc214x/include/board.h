/****************************************************************************
 * boards/arm/lpc214x/mcu123-lpc214x/include/board.h
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

#ifndef __BOARDS_ARM_LPC214X_MCU123_LPC214X_INCLUDE_BOARD_H
#define __BOARDS_ARM_LPC214X_MCU123_LPC214X_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Oscillator frequency */

#define LPC214X_FOSC      12000000

/* PLL0 settings CCLK = PLL_M * FOSC PCLK = CCLK/APBDIV */

#define LPC214X_PLL_M     5
#define LPC214X_PLL_P     2
#define LPC214X_APB_DIV   1

/* USB Pll settings -- 48 MHz needed.  FUSB = PLL_M FOSC */

#define LPC214X_USBPLL_M  4
#define LPC214X_USBPLL_P  2

/* LED definitions **********************************************************/

#define LED_STARTED       0
#define LED_HEAPALLOCATE  1
#define LED_IRQSENABLED   2
#define LED_STACKCREATED  3
#define LED_INIRQ         4
#define LED_SIGNAL        5
#define LED_ASSERTION     6
#define LED_PANIC         7

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __BOARDS_ARM_LPC214X_MCU123_LPC214X_INCLUDE_BOARD_H */
