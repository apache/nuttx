/****************************************************************************
 * boards/arm/lpc31xx/ea3152/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_LPC31XX_EA3152_INCLUDE_BOARD_H
#define __BOARDS_ARM_LPC31XX_EA3152_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
# include "lpc31_cgudrvr.h"
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
