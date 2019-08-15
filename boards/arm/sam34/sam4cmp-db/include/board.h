/****************************************************************************
 * boards/arm/sam34/sam4cmp-db/include/board.h
 *
 *   Copyright (C) 2016 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
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

#ifndef __BOARDS_ARM_SAM34_SAM4CMP_DB_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAM34_SAM4CMP_DB_INCLUDE_BOARD_H

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

/* After power-on reset, the sam3u device is running on a 4MHz internal RC.
 * These definitions will configure clocking with MCK = 48MHz, PLLA = 96,
 * and CPU=120MHz.
 */

/* Main oscillator register settings */

#define BOARD_CKGR_MOR_MOSCXTST    (63 << PMC_CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */

/* PLLA configuration:
 *
 * Source: 12MHz crystall at 12MHz
 * PLLdiv: 10
 * PLLmul: 1 (bypassed)
 * Fpll:   (12MHz * 10) / 1 = 120MHz
 */

#define BOARD_MAINOSC_FREQUENCY    (8192000)
#define BOARD_CKGR_PLLAR_MUL       (9 << PMC_CKGR_PLLAR_MUL_SHIFT)
#define BOARD_CKGR_PLLAR_DIV       PMC_CKGR_PLLAR_DIV_BYPASS
#define BOARD_CKGR_PLLAR_COUNT     (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#define BOARD_PLLA_FREQUENCY       (10*BOARD_MAINOSC_FREQUENCY)

/* PLLB configuration
 *
 * Source: MAIN clock (i.e. 8.192MHz)
 * PLLdiv: 4
 * PLLmul: 45
 * Fpll: (8.192MHz * (44+1) / 4 = 92.120 MHz
 */

#define BOARD_CKGR_PLLBR_SRCB      (0 << PMC_CKGR_PLLBR_SRCB_SHIFT)
#define BOARD_CKGR_PLLBR_DIV       (4 << PMC_CKGR_PLLBR_DIV_SHIFT)
#define BOARD_CKGR_PLLBR_MUL       (44 << PMC_CKGR_PLLBR_MUL_SHIFT)
#define BOARD_CKGR_PLLBR_COUNT     (63 << PMC_CKGR_PLLBR_COUNT_SHIFT)
#define BOARD_PLLB_FREQUENCY       (92160000)

/* PMC master clock register settings */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLB
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV1
#define BOARD_MCK_FREQUENCY        (BOARD_PLLB_FREQUENCY/1)
#define BOARD_CPU_FREQUENCY        (BOARD_PLLB_FREQUENCY/1)

/* USB UTMI PLL start-up time */

#define BOARD_CKGR_UCKR_UPLLCOUNT  (3 << PMC_CKGR_UCKR_UPLLCOUNT_SHIFT)

/* FLASH wait states:
 *
 * DC Characteristics
 *
 * Parameter              Min   Typ  Max
 * ---------------------- ----- ----- ----
 * Vddcore DC Supply Core 1.08V 1.2V 1.32V
 * Vvddio  DC Supply I/Os 1.62V 3.3V 3.6V
 *
 *                     Wait   Maximum
 * Vddcore   Vvddio   States Frequency (MHz)
 * ------- ---------- ------ ---------------
 * 1.08V   1.62-3.6V    0        16
 * "   "   "  "-"  "    1        33
 * "   "   "  "-"  "    2        50
 * "   "   "  "-"  "    3        67
 * "   "   "  "-"  "    4        84
 * "   "   "  "-"  "    5       100
 * 1.08V   2.7-3.6V     0        20
 * "   "   " "-"  "     1        40
 * "   "   " "-"  "     2        60
 * "   "   " "-"  "     3        80
 * "   "   " "-"  "     4       100
 * 1.2V    1.62-3.6V    0        17
 * "  "    " "-"  "     1        34
 * "  "    " "-"  "     2        52
 * "  "    " "-"  "     3        69
 * "  "    " "-"  "     4        87
 * "  "    " "-"  "     5       104
 * "  "    " "-"  "     6       121
 * 1.2V    2.7-3.6V     0        21
 * "  "    " "-"  "     1        42
 * "  "    " "-"  "     2        63
 * "  "    " "-"  "     3        84
 * "  "    " "-"  "     4       105
 * "  "    " "-"  "     5       123 << SELECTION
 */

#define BOARD_FWS                  5

#endif /* __BOARDS_ARM_SAM34_SAM4CMP_DB_INCLUDE_BOARD_H */
