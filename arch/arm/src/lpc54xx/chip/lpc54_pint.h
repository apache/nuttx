/****************************************************************************************************
 * arch/arm/src/lpc54xx/chip/lpc54_pint.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_PINT_H
#define __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_PINT_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/lpc54_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register offsets *********************************************************************************/

#define LPC54_PINT_ISEL_OFFSET    0x0000 /* Pin interrupt mode */
#define LPC54_PINT_IENR_OFFSET    0x0004 /* Pin interrupt level or rising edge interrupt enable */
#define LPC54_PINT_SIENR_OFFSET   0x0008 /* Pin interrupt level or rising edge interrupt enable set */
#define LPC54_PINT_CIENR_OFFSET   0x000c /* Pin interrupt level or rising edge interrupt enable clear */
#define LPC54_PINT_IENF_OFFSET    0x0010 /* Pin interrupt active level or falling edge interrupt enable */
#define LPC54_PINT_SIENF_OFFSET   0x0014 /* Pin interrupt active level or falling edge interrupt set */
#define LPC54_PINT_CIENF_OFFSET   0x0018 /* Pin interrupt active level or falling edge interrupt clear */
#define LPC54_PINT_RISE_OFFSET    0x001c /* Pin interrupt rising edge */
#define LPC54_PINT_FALL_OFFSET    0x0020 /* Pin interrupt falling edge */
#define LPC54_PINT_IST_OFFSET     0x0024 /* Pin interrupt status */
#define LPC54_PINT_PMCTRL_OFFSET  0x0028 /* Pattern match interrupt control */
#define LPC54_PINT_PMSRC_OFFSET   0x002c /* Pattern match interrupt bit-slice source */
#define LPC54_PINT_PMCFG_OFFSET   0x0030 /* Pattern match interrupt bit slice configuration */

/* Register addresses *******************************************************************************/

#define LPC54_PINT_ISEL           (LPC54_PINT_BASE + LPC54_PINT_ISEL_OFFSET)
#define LPC54_PINT_IENR           (LPC54_PINT_BASE + LPC54_PINT_IENR_OFFSET)
#define LPC54_PINT_SIENR          (LPC54_PINT_BASE + LPC54_PINT_SIENR_OFFSET)
#define LPC54_PINT_CIENR          (LPC54_PINT_BASE + LPC54_PINT_CIENR_OFFSET)
#define LPC54_PINT_IENF           (LPC54_PINT_BASE + LPC54_PINT_IENF_OFFSET)
#define LPC54_PINT_SIENF          (LPC54_PINT_BASE + LPC54_PINT_SIENF_OFFSET)
#define LPC54_PINT_CIENF          (LPC54_PINT_BASE + LPC54_PINT_CIENF_OFFSET)
#define LPC54_PINT_RISE           (LPC54_PINT_BASE + LPC54_PINT_RISE_OFFSET)
#define LPC54_PINT_FALL           (LPC54_PINT_BASE + LPC54_PINT_FALL_OFFSET)
#define LPC54_PINT_IST            (LPC54_PINT_BASE + LPC54_PINT_IST_OFFSET)
#define LPC54_PINT_PMCTRL         (LPC54_PINT_BASE + LPC54_PINT_PMCTRL_OFFSET)
#define LPC54_PINT_PMSRC          (LPC54_PINT_BASE + LPC54_PINT_PMSRC_OFFSET)
#define LPC54_PINT_PMCFG          (LPC54_PINT_BASE + LPC54_PINT_PMCFG_OFFSET)

/* Register bit definitions *************************************************************************/

/* Pin interrupt mode */

#define PINT_PMODE(n)             (1 << (n)) /* Pin n level(1) or edge(0) sensitive.  n=0..7 */

/* Pin interrupt level or rising edge interrupt enable, set, and clear registers */

#define PINT_ENRL(n)              (1 << (n)) /* Pin n enable(1) or disable(0) rising/level.  n=0..7 */

/* Pin interrupt active level or falling edge interrupt enable, set, and clear registers */

#define PINT_ENAF(n)              (1 << (n)) /* Pin n enable(1) or disable(0) falling/active.  n=0..7 */

/* Pin interrupt rising edge */

#define PINT_RDET(n)              (1 << (n)) /* R:Rising edge detected, W:Clear.  n=0..7 */

/* Pin interrupt falling edge */

#define PINT_FDET(n)              (1 << (n)) /* R:Falling edge detected, W:Clear.  n=0..7 */

/* Pin interrupt status */

#define PINT_PSTAT(n)             (1 << (n)) /* R:Interrupt pending, W:Clear edge or toggle level.  n=0..7 */

/* Pattern match interrupt control */

#define PINT_PMCTRL_SELPMATCH     (1 << 0)   /* Bit 0:  Rin interrupts interrupt or pattern match function */
#define PINT_PMCTRL_ENARXEV       (1 << 1)   /* Bit 1:  Enables RXEV output to CPU */
#define PINT_PMCTRL_PMAT_SHIFT    (24)       /* Bits 24-31: Current state of pattern matches */

/* Pattern match interrupt bit-slice source */

/* PINTSELn=1 indicates that PINSETn is the source to bit slice m. */

#define PINT_PMSRC_PINTSEL0       0
#define PINT_PMSRC_PINTSEL1       1
#define PINT_PMSRC_PINTSEL2       2
#define PINT_PMSRC_PINTSEL3       3
#define PINT_PMSRC_PINTSEL4       4
#define PINT_PMSRC_PINTSEL5       5
#define PINT_PMSRC_PINTSEL6       6
#define PINT_PMSRC_PINTSEL7       7

#define PINT_PMSRC_SRC0_SHIFT     (8)        /* Bits 8-10: Selects PINSELn as input source for bit slice 0 */
#define PINT_PMSRC_SRC0_MASK      (7 << PINT_PMSRC_SRC0_SHIFT)
#  define PINT_PMSRC_SRC0(n)      ((uint32_t)(n) << PINT_PMSRC_SRC0_SHIFT)
#define PINT_PMSRC_SRC1_SHIFT     (11)       /* Bits 11-13: Selects PINSELn as input source for bit slice 0 */
#define PINT_PMSRC_SRC1_MASK      (7 << PINT_PMSRC_SRC1_SHIFT)
#  define PINT_PMSRC_SRC1(n)      ((uint32_t)(n) << PINT_PMSRC_SRC1_SHIFT)
#define PINT_PMSRC_SRC2_SHIFT     (14)       /* Bits 14-16: Selects PINSELn as input source for bit slice 0 */
#define PINT_PMSRC_SRC2_MASK      (7 << PINT_PMSRC_SRC2_SHIFT)
#  define PINT_PMSRC_SRC2(n)      ((uint32_t)(n) << PINT_PMSRC_SRC2_SHIFT)
#define PINT_PMSRC_SRC3_SHIFT     (17)       /* Bits 17-19: Selects PINSELn as input source for bit slice 0 */
#define PINT_PMSRC_SRC3_MASK      (7 << PINT_PMSRC_SRC3_SHIFT)
#  define PINT_PMSRC_SRC3(n)      ((uint32_t)(n) << PINT_PMSRC_SRC3_SHIFT)
#define PINT_PMSRC_SRC4_SHIFT     (20)       /* Bits 20-22: Selects PINSELn as input source for bit slice 0 */
#define PINT_PMSRC_SRC4_MASK      (7 << PINT_PMSRC_SRC4_SHIFT)
#  define PINT_PMSRC_SRC4(n)      ((uint32_t)(n) << PINT_PMSRC_SRC4_SHIFT)
#define PINT_PMSRC_SRC5_SHIFT     (23)       /* Bits 23-25: Selects PINSELn as input source for bit slice 0 */
#define PINT_PMSRC_SRC5_MASK      (7 << PINT_PMSRC_SRC5_SHIFT)
#  define PINT_PMSRC_SRC5(n)      ((uint32_t)(n) << PINT_PMSRC_SRC5_SHIFT)
#define PINT_PMSRC_SRC6_SHIFT     (26)       /* Bits 26-28: Selects PINSELn as input source for bit slice 0 */
#define PINT_PMSRC_SRC6_MASK      (7 << PINT_PMSRC_SRC6_SHIFT)
#  define PINT_PMSRC_SRC6(n)      ((uint32_t)(n) << PINT_PMSRC_SRC6_SHIFT)
#define PINT_PMSRC_SRC7_SHIFT     (29)       /* Bits 29-31: Selects PINSELn as input source for bit slice 0 */
#define PINT_PMSRC_SRC7_MASK      (7 << PINT_PMSRC_SRC7_SHIFT)
#  define PINT_PMSRC_SRC7(n)      ((uint32_t)(n) << PINT_PMSRC_SRC7_SHIFT)

/* Pattern match interrupt bit slice configuration */

/* PINT_PMCFG_ENDPTSn:  Determines whether slice n is an endpoint of a product term (minterm). Pin
 * interrupt n in the NVIC is raised if the minterm evaluates as true.
 */

#define PINT_PMCFG_ENDPTS0        (1 << 0)   /* Bit 0: Slice n is an endpoint */
#define PINT_PMCFG_ENDPTS1        (1 << 1)   /* Bit 1: Slice n is an endpoint */
#define PINT_PMCFG_ENDPTS2        (1 << 2)   /* Bit 2: Slice n is an endpoint */
#define PINT_PMCFG_ENDPTS3        (1 << 3)   /* Bit 3: Slice n is an endpoint */
#define PINT_PMCFG_ENDPTS4        (1 << 4)   /* Bit 4: Slice n is an endpoint */
#define PINT_PMCFG_ENDPTS5        (1 << 5)   /* Bit 5: Slice n is an endpoint */
#define PINT_PMCFG_ENDPTS6        (1 << 6)   /* Bit 6: Slice n is an endpoint */

#define PINT_PMCFG_HIGH           0          /* Constant HIGH */
#define PINT_PMCFG_RISING         1          /* Sticky rising edge */
#define PINT_PMCFG_FALLING        2          /* Sticky falling edge */
#define PINT_PMCFG_BOTH           3          /* Sticky rising or falling edge */
#define PINT_PMCFG_HIGH_LEVEL     4          /* High level */
#define PINT_PMCFG_LOW_LEVEL      5          /* Low level */
#define PINT_PMCFG_ZERO           6          /* Constant 0 */
#define PINT_PMCFG_EVENT          7          /* Event */

#define PINT_PMCFG_CFG0_SHIFT     (8)        /* Bits 8-10:  Match condition for bit slice 0 */
#define PINT_PMCFG_CFG0_MASK      (7 << PINT_PMCFG_CFG0_SHIFT)
#  define PINT_PMCFG_CFG0(n)      ((uint32_t)(n) << PINT_PMCFG_CFG0_SHIFT)
#define PINT_PMCFG_CFG1_SHIFT     (8)        /* Bits 11-13: Match condition for bit slice 1 */
#define PINT_PMCFG_CFG1_MASK      (7 << PINT_PMCFG_CFG1_SHIFT)
#  define PINT_PMCFG_CFG1(n)      ((uint32_t)(n) << PINT_PMCFG_CFG1_SHIFT)
#define PINT_PMCFG_CFG2_SHIFT     (8)        /* Bits 14-16: Match condition for bit slice 2 */
#define PINT_PMCFG_CFG2_MASK      (7 << PINT_PMCFG_CFG2_SHIFT)
#  define PINT_PMCFG_CFG2(n)      ((uint32_t)(n) << PINT_PMCFG_CFG2_SHIFT)
#define PINT_PMCFG_CFG3_SHIFT     (8)        /* Bits 17-19: Match condition for bit slice 3 */
#define PINT_PMCFG_CFG3_MASK      (7 << PINT_PMCFG_CFG3_SHIFT)
#  define PINT_PMCFG_CFG3(n)      ((uint32_t)(n) << PINT_PMCFG_CFG3_SHIFT)
#define PINT_PMCFG_CFG4_SHIFT     (8)        /* Bits 20-22: Match condition for bit slice 4 */
#define PINT_PMCFG_CFG4_MASK      (7 << PINT_PMCFG_CFG4_SHIFT)
#  define PINT_PMCFG_CFG4(n)      ((uint32_t)(n) << PINT_PMCFG_CFG4_SHIFT)
#define PINT_PMCFG_CFG5_SHIFT     (8)        /* Bits 23-25: Match condition for bit slice 5 */
#define PINT_PMCFG_CFG5_MASK      (7 << PINT_PMCFG_CFG5_SHIFT)
#  define PINT_PMCFG_CFG5(n)      ((uint32_t)(n) << PINT_PMCFG_CFG5_SHIFT)
#define PINT_PMCFG_CFG6_SHIFT     (8)        /* Bits 26-28: Match condition for bit slice 6 */
#define PINT_PMCFG_CFG6_MASK      (7 << PINT_PMCFG_CFG6_SHIFT)
#  define PINT_PMCFG_CFG6(n)      ((uint32_t)(n) << PINT_PMCFG_CFG6_SHIFT)
#define PINT_PMCFG_CFG7_SHIFT     (8)        /* Bits 29-31: Match condition for bit slice 7 */
#define PINT_PMCFG_CFG7_MASK      (7 << PINT_PMCFG_CFG7_SHIFT)
#  define PINT_PMCFG_CFG7(n)      ((uint32_t)(n) << PINT_PMCFG_CFG7_SHIFT)

#endif /* __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_PINT_H */
