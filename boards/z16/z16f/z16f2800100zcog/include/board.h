/****************************************************************************
 * boards/z16/z16f/z16f2800100zcog/include/board.h
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

#ifndef __BOARDS_Z16_Z16F2811_Z16F2800100ZCOG_INCLUDE_BOARD_H
#define __BOARDS_Z16_Z16F2811_Z16F2800100ZCOG_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Z16F2800100ZCOG board has a 20MHz crystal.  The ZNEO clocking will be
 * configured to use this crystal frequency directly as the clock source
 */

#define BOARD_XTAL_FREQUENCY   20000000 /* 20MHz */
#define BOARD_CLKSRC           1        /* Clock source = external crystal */
#define BOARD_SYSTEM_FREQUENCY BOARD_XTAL_FREQUENCY

/* Flash option bits
 *
 * "Each time the option bits are programmed or erased, the device must be
 *  Reset for the change to take place. During any reset operation .., the
 *  option bits are automatically read from the Program memory and written
 *  to Option Configuration registers. ... Option Bit Control Register are
 *  loaded before the device exits Reset and the ZNEO CPU begins code
 *  execution. The Option Configuration registers are not part of the
 *  Register file and are not accessible for read or write access."
 */

#ifndef __ASSEMBLY__
#  define BOARD_FLOPTION0 (Z16F_FLOPTION0_MAXPWR | Z16F_FLOPTION0_WDTRES | \
                           Z16F_FLOPTION0_WDTA0 | Z16F_FLOPTION0_VBOA0 | \
                           Z16F_FLOPTION0_DBGUART | Z16F_FLOPTION0_FWP | \
                           Z16F_FLOPTION0_RP)

#  define BOARD_FLOPTION1 (Z16F_FLOPTION1_RESVD | Z16F_FLOPTION1_MCEN | \
                           Z16F_FLOPTION1_OFFH | Z16F_FLOPTION1_OFFL)

#  define BOARD_FLOPTION2 Z16F_FLOPTION2_RESVD

#  define BOARD_FLOPTION3 (Z16F_FLOPTION3_RESVD | Z16F_FLOPTION3_NORMAL | \
                           Z16F_FLOPTION3_ROMLESS)

/* The same settings, pre-digested for assembly language */

#else
#  define BOARD_FLOPTION0 %ff
#  define BOARD_FLOPTION1 %ff
#  define BOARD_FLOPTION2 %ff
#  define BOARD_FLOPTION3 %ff
#endif

/* LED pattern definitions
 *
 * The z16f2800100zcog board has four LEDs:
 *
 * - Green LED D1 which illuminates in the presence of Vcc
 * - Red LED D2 connected to chip port PA0_T0IN
 * - Yellow LED D3 connected to chip port PA1_T0OUT
 * - Green LED D4 connected to chip port PA2_DE0
 */

#define LED_STARTED            0
#define LED_HEAPALLOCATE       1
#define LED_IRQSENABLED        2
#define LED_STACKCREATED       3
#define LED_IDLE               4
#define LED_INIRQ              5
#define LED_SIGNAL             6
#define LED_ASSERTION          7
#define LED_PANIC              8

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __BOARDS_Z16_Z16F2811_Z16F2800100ZCOG_INCLUDE_BOARD_H */
