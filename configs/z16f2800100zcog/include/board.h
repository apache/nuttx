/****************************************************************************
 * configs/z16f2800100zcog/board.h
 *
 *   Copyright (C) 2008, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __CONFIGS_Z16F2800100ZCOG_INCLUDE_BOARD_H
#define __CONFIGS_Z16F2800100ZCOG_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Definitions
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
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif  /* __CONFIGS_Z16F2800100ZCOG_INCLUDE_BOARD_H */
