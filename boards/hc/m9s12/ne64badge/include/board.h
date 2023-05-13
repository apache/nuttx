/****************************************************************************
 * boards/hc/m9s12/ne64badge/include/board.h
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

#ifndef __BOARDS_HC_M9S12_NE64BADGE_INCLUDE_BOARD_H
#define __BOARDS_HC_M9S12_NE64BADGE_INCLUDE_BOARD_H

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

/* Frequency of the crystal oscillator */

#define HCS12_OSCCLK      16000000 /* 16MHz */

/* PLL Settings
 *
 * SYNR register controls the multiplication factor of the PLL.
 * If the PLL is on, the count in the loop divider (SYNR) register
 * effectively multiplies up the PLL clock (PLLCLK) from the reference
 * frequency by 2 x (SYNR+1). PLLCLK will not be below
 * the minimum VCO frequency (fSCM).
 *
 * The REFDV register provides a finer granularity for the PLL multiplier
 * steps.
 * The count in the reference divider divides OSCCLK frequency by REFDV + 1.
 *
 *   PLLCLK = 2 * OSCCLK * (SYNR + 1) / (REFDV + 1)
 *
 * If (PLLSEL = 1), Bus Clock = PLLCLK / 2
 */

#define HCS12_SYNR_VALUE  0x15
#define HCS12_REFDV_VALUE 0x15
#define HCS12_PLLCLK      (2*HCS12_OSCCLK*(HCS12_SYNR+1)/(HCS12_REFDV+1))
#define HCS12_BUSCLK      (HSC12_PLLCLK/2)

/* LED definitions **********************************************************/

/* The NE64 Badge board has 2 red LEDs that we will encode as: */

                             /* LED1 LED2 */
#define LED_STARTED       0  /*  OFF  OFF = Still initializing */
#define LED_HEAPALLOCATE  0  /*  OFF  OFF = Still initializing */
#define LED_IRQSENABLED   0  /*  OFF  OFF = Still initializing */
#define LED_STACKCREATED  1  /*  ON   OFF = Initialization complete */
#define LED_INIRQ         2  /*  NC   ON  = In an interrupt handler */
#define LED_SIGNAL        2  /*  NC   ON  = In a signal handler */
#define LED_ASSERTION     2  /*  NC   ON  = In an assertion */
#define LED_PANIC         2  /*  NC   ON  = Oops! We crashed. (flashing) */

/* Button definitions *******************************************************/

/* The NE64 badge has 2 switches,
 * one labeled SW1 and the other SW2 on the board (but
 * referred to as button1 and button2 in some documentation)
 */

#define BUTTON1           1 /* Bit 0: SW1 button is depressed */
#define BUTTON2           2 /* Bit 1: SW2 button is depressed */

#endif /* __BOARDS_HC_M9S12_NE64BADGE_INCLUDE_BOARD_H */
