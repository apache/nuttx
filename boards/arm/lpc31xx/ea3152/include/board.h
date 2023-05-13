/****************************************************************************
 * boards/arm/lpc31xx/ea3152/include/board.h
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

#ifndef __BOARDS_ARM_LPC31XX_EA3152_INCLUDE_BOARD_H
#define __BOARDS_ARM_LPC31XX_EA3152_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include "lpc31_cgudrvr.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Frequency of the FFAST input */

#define BOARD_FREQIN_FFAST     (12000000) /* ffast (12 MHz crystal) */

/* HPLL0 configuration */

#define BOARD_HPLL0_FINSEL CGU_HPFINSEL_FFAST /* Frequency input selection */
#define BOARD_HPLL0_NDEC   131                /* PLL N-divider value */
#define BOARD_HPLL0_MDEC   29784              /* PLL M-divider value */
#define BOARD_HPLL0_PDEC   7                  /* PLL P-divider value */
#define BOARD_HPLL0_SELR   0                  /* SELR bandwidth selection */
#define BOARD_HPLL0_SELI   8                  /* SELI bandwidth selection */
#define BOARD_HPLL0_SELP   31                 /* SELP bandwidth selection */
#define BOARD_HPLL0_MODE   0                  /* PLL mode */
#define BOARD_HPLL0_FREQ   406425600          /* Frequency of the PLL in MHz */

/* HPLL1 configuration */

#define BOARD_HPLL1_FINSEL CGU_HPFINSEL_FFAST /* Frequency input selection */
#define BOARD_HPLL1_NDEC   770                /* PLL N-divider value */
#define BOARD_HPLL1_MDEC   8191               /* PLL M-divider value */
#define BOARD_HPLL1_PDEC   98                 /* PLL P-divider value */
#define BOARD_HPLL1_SELR   0                  /* SELR bandwidth selection */
#define BOARD_HPLL1_SELI   16                 /* SELI bandwidth selection */
#define BOARD_HPLL1_SELP   8                  /* SELP bandwidth selection */
#define BOARD_HPLL1_MODE   0                  /* PLL mode */
#define BOARD_HPLL1_FREQ  180000000           /* Frequency of the PLL in MHz */

/* The following 3 bitsets determine which clocks will be enabled at
 * initialization time.
 */

#define BOARD_CLKS_0_31 \
 (_RBIT(CLKID_APB0CLK,0)|_RBIT(CLKID_APB1CLK,0)|_RBIT(CLKID_APB2CLK,0)|\
  _RBIT(CLKID_APB3CLK,0)|_RBIT(CLKID_APB4CLK,0)|_RBIT(CLKID_AHB2INTCCLK,0)|\
  _RBIT(CLKID_AHB0CLK,0)|_RBIT(CLKID_ARM926CORECLK,0)|_RBIT(CLKID_ARM926BUSIFCLK,0)|\
  _RBIT(CLKID_ARM926RETIMECLK,0)|_RBIT(CLKID_ISRAM0CLK,0)|_RBIT(CLKID_ISRAM1CLK,0)|\
  _RBIT(CLKID_ISROMCLK,0)|_RBIT(CLKID_INTCCLK,0)|_RBIT(CLKID_AHB2APB0PCLK,0)|\
  _RBIT(CLKID_EVENTROUTERPCLK,0)|_RBIT(CLKID_CLOCKOUT,0))

#define BOARD_CLKS_32_63 \
 (_RBIT(CLKID_IOCONFPCLK,32)|_RBIT(CLKID_CGUPCLK,32)|_RBIT(CLKID_SYSCREGPCLK,32)|\
  _RBIT(CLKID_OTPPCLK,32)|_RBIT(CLKID_AHB2APB1PCLK,32)|_RBIT(CLKID_AHB2APB2PCLK,32)|\
  _RBIT(CLKID_AHB2APB3PCLK,32)|_RBIT(CLKID_EDGEDETPCLK,32))

#define BOARD_CLKS_64_92 \
 (0)

/* LED definitions **********************************************************/

#define LED_STARTED       0
#define LED_HEAPALLOCATE  1
#define LED_IRQSENABLED   2
#define LED_STACKCREATED  3
#define LED_INIRQ         4
#define LED_SIGNAL        5
#define LED_ASSERTION     6
#define LED_PANIC         7

/* Button definitions *******************************************************/

#endif /* __BOARDS_ARM_LPC31XX_EA3152_INCLUDE__BOARD_H */
