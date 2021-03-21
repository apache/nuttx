/****************************************************************************
 * boards/arm/sam34/sam4cmp-db/include/board.h
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
