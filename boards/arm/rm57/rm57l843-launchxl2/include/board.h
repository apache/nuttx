/****************************************************************************
 * boards/arm/rm57/rm57l843-launchxl2/include/board.h
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

#ifndef __BOARDS_ARM_RM57_RM57L843_LAUNCHXL2_INCLUDE_BOARD_H
#define __BOARDS_ARM_RM57_RM57L843_LAUNCHXL2_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Values below are taken directly from TI's HALCoGen HL_system.c
 * (setupPLL()/mapClocks()) and HL_system.h (OSC_FREQ/PLL1_FREQ/etc
 * comments), i.e. the PLL configuration this HALCoGen project was
 * actually generated with - not recomputed from scratch. Verify against
 * the LAUNCHXL2-RM57L schematic if the crystal is ever changed.
 */

/* 16 MHz crystal (HL_system.h OSC_FREQ) */
#define BOARD_FCLKIN_FREQUENCY 16000000

/* PLLCTL1/PLLCTL2 encode:
 *   NR (REFCLKDIV+1)     = 8
 *   PLLMUL (raw field)   = 0x9500
 *   OD (ODPLL+1)         = 1
 *   R  (PLLDIV+1, final) = 1
 *
 * HALCoGen computes the resulting PLL1/GCLK frequency as 300 MHz
 * (HL_system.h PLL1_FREQ/GCLK_FREQ) - taken as-is rather than
 * re-derived from the PLLMUL fixed-point encoding.
 */

#define BOARD_PLL_NR             8
#define BOARD_PLL_PLLMUL         0x9500
#define BOARD_PLL_OD             1
#define BOARD_PLL_R              1
#define BOARD_PLL_FREQUENCY      300000000  /* PLL1_FREQ / GCLK_FREQ */

/* HCLKCNTL = 1 -> HCLK = GCLK / (1 + 1) = 150 MHz (HL_system.h HCLK_FREQ) */

#define BOARD_HCLK_FREQUENCY     150000000

/* VCLK1 is the input clock to the SCI baud rate generator
 * (HL_system.h VCLK1_FREQ)
 */

#define BOARD_VCLK_FREQUENCY     75000000

/* Flash read wait-states (HL_system.c setupFlash(): FRDCNTL RWAIT field) */

#define BOARD_FLASH_RWAIT        3

/* RTI1 clock, used for the NuttX system tick (HL_system.h RTI_FREQ) */

#define BOARD_RTICLK_FREQUENCY   75000000

/* PIN Multiplexor Initializer **********************************************/

/* Pin-mux initialization is not yet implemented for this board;
 * BOARD_PINMUX_INITIALIZER is not currently defined.
 */

/* LED definitions **********************************************************/

/* The LAUNCHXL2-RM57L has two user LEDs, labeled B6 and B7 on the board
 * silkscreen (driven by GIOB[6] and GIOB[7] respectively; see
 * src/rm57l843-launchxl2.h for the GIO pin definitions).
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_B6         0
#define BOARD_LED_B7         1
#define BOARD_NLEDS          2

/* LED bits for use with board_userled_all() */

#define BOARD_LED_B6_BIT     (1 << BOARD_LED_B6)
#define BOARD_LED_B7_BIT     (1 << BOARD_LED_B7)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * src/rm57_autoleds.c. Both LEDs are driven together to encode
 * OS-related events as follows:
 */

#define LED_STARTED         0 /* NuttX has been started */
#define LED_HEAPALLOCATE    0 /* Heap has been allocated */
#define LED_IRQSENABLED     0 /* Interrupts enabled */
#define LED_STACKCREATED    1 /* Idle stack created */
#define LED_INIRQ           2 /* In an interrupt */
#define LED_SIGNAL          2 /* In a signal handler */
#define LED_ASSERTION       2 /* An assertion failed */
#define LED_PANIC           3 /* The system has crashed */
#undef  LED_IDLE              /* MCU is in sleep mode: Not used */

#endif /* __BOARDS_ARM_RM57_RM57L843_LAUNCHXL2_INCLUDE_BOARD_H */
