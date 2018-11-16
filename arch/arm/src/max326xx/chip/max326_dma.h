/************************************************************************************
 * arch/arm/src/max326xx/chip/max326_dma.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_MAX326XX_CHIP_MAX326_DMA_H
#define __ARCH_ARM_SRC_MAX326XX_CHIP_MAX326_DMA_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/max326_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define MAX326_DMA_CHAN0            0 /* DMA Channel 0 */
#define MAX326_DMA_CHAN1            1 /* DMA Channel 1 */
#define MAX326_DMA_CHAN2            2 /* DMA Channel 2 */
#define MAX326_DMA_CHAN3            3 /* DMA Channel 3 */
#define MAX326_DMA_NCHAN            4 /* Four DMA Channels */

/* DMA Control Registers */

#define MAX326_DMA_INTEN_OFFSET     0x0000 /* DMA Control register */
#define MAX326_DMA_INTFL_OFFSET     0x0004 /* DMA Interrupt Status register */

/* DMA Channel Registers */

#define MAX326_DMACH_OFFSET(n)      (0x0100 + )(n) << 5))
#define MAX326_DMACH0_OFFSET        0x0100
#define MAX326_DMACH1_OFFSET        0x0120
#define MAX326_DMACH2_OFFSET        0x0140
#define MAX326_DMACH3_OFFSET        0x0160

#define MAX326_DMACH_CFG_OFFSET     0x0000  /* DMA Channel Configuration Register */
#define MAX326_DMACH_STAT_OFFSET    0x0004  /* DMA Channel Status Register */
#define MAX326_DMACH_SRC_OFFSET     0x0008  /* DMA Channel Source Register */
#define MAX326_DMACH_DST_OFFSET     0x000c  /* DMA Channel Destination Register */
#define MAX326_DMACH_CNT_OFFSET     0x0010  /* DMA Channel Count Register */
#define MAX326_DMACH_SRCRLD_OFFSET  0x0014  /* DMA Channel Source Reload Register */
#define MAX326_DMACH_DSTRLD_OFFSET  0x0018  /* DMA Channel Destination Reload Register */
#define MAX326_DMACH_CNTRLD_OFFSET  0x001c  /* DMA Channel Count Reload Register */

/* Register Addresses ***************************************************************/

/* DMA Control Registers */

#define MAX326_DMA_INTEN            (MAX326_DMA_BASE + MAX326_DMA_INTEN_OFFSET)
#define MAX326_DMA_INTFL            (MAX326_DMA_BASE + MAX326_DMA_INTFL_OFFSET)

/* DMA Channel Registers */

#define MAX326_DMACH_BASE(n)        (MAX326_DMA_BASE + MAX326_DMACH_OFFSET(n))
#define MAX326_DMACH0_BASE          (MAX326_DMA_BASE + MAX326_DMACH0_OFFSET)
#define MAX326_DMACH1_BASE          (MAX326_DMA_BASE + MAX326_DMACH1_OFFSET)
#define MAX326_DMACH2_BASE          (MAX326_DMA_BASE + MAX326_DMACH2_OFFSET)
#define MAX326_DMACH3_BASE          (MAX326_DMA_BASE + MAX326_DMACH3_OFFSET)

#define MAX326_DMACH_CFG(n)         (MAX326_DMACH_BASE(n) + MAX326_DMACH_CFG_OFFSET)
#define MAX326_DMACH_STAT(n)        (MAX326_DMACH_BASE(n) + MAX326_DMACH_STAT_OFFSET)
#define MAX326_DMACH_SRC(n)         (MAX326_DMACH_BASE(n) + MAX326_DMACH_SRC_OFFSET)
#define MAX326_DMACH_DST(n)         (MAX326_DMACH_BASE(n) + MAX326_DMACH_DST_OFFSET)
#define MAX326_DMACH_CNT(n)         (MAX326_DMACH_BASE(n) + MAX326_DMACH_CNT_OFFSET)
#define MAX326_DMACH_SRCRLD(n)      (MAX326_DMACH_BASE(n) + MAX326_DMACH_SRCRLD_OFFSET)
#define MAX326_DMACH_DSTRLD(n)      (MAX326_DMACH_BASE(n) + MAX326_DMACH_DSTRLD_OFFSET)
#define MAX326_DMACH_CNTRLD(n)      (MAX326_DMACH_BASE(n) + MAX326_DMACH_CNTRLD_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* DMA Control register */

#define DMA_INTEN(n)                (1 << (n)) /* Channel n interrupt enable */

/* DMA Interrupt Status register */

#define DMA_INTFL(n)                (1 << (n)) /* Channel n interrupt pending */

/* DMA Channel Registers */

/* DMA Channel Configuration Register */

#define DMACH_CFG_CHEN              (1 << 0)  /* Bit nn:  Channel Enable */
#define DMACH_CFG_RLDEN             (1 << 1)  /* Bit nn:  Reload Enable */
#define DMACH_CFG_PRI_SHIFT         (2)       /* Bits 2-3: DMA priority */
#define DMACH_CFG_PRI_MASK          (xx << DMACH_CFG_PRI_SHIFT)
#define DMACH_CFG_REQSEL_SHIFT      (4)       /* Bits 4-9: Request Select */
#define DMACH_CFG_REQSEL_MASK       (0x3f << DMACH_CFG_REQSEL_SHIFT)
#  define DMACH_CFG_REQSEL_LO       (3 << DMACH_CFG_REQSEL_SHIFT)
#  define DMACH_CFG_REQSEL_MEDLO    (2 << DMACH_CFG_REQSEL_SHIFT)
#  define DMACH_CFG_REQSEL_MEDHI    (1 << DMACH_CFG_REQSEL_SHIFT)
#  define DMACH_CFG_REQSEL_HI       (0 << DMACH_CFG_REQSEL_SHIFT)
#define DMACH_CFG_REQWAIT           (1 << 10) /* Bit 10: Request Wait Enable */
#define DMACH_CFG_TOSEL_SHIFT       (11)      /* Bits 11-13: Time-Out Select */
#define DMACH_CFG_TOSEL_MASK        (7 << DMACH_CFG_TOSEL_SHIFT)
#  define DMACH_CFG_TOSEL_ 3to4     (0 << DMACH_CFG_TOSEL_SHIFT) /* 3-4 */
#  define DMACH_CFG_TOSEL_ 7to8     (1 << DMACH_CFG_TOSEL_SHIFT) /* 7-8 */
#  define DMACH_CFG_TOSEL_ 15to16   (2 << DMACH_CFG_TOSEL_SHIFT) /* 15-16 */
#  define DMACH_CFG_TOSEL_ 31to82   (3 << DMACH_CFG_TOSEL_SHIFT) /* 31-32 */
#  define DMACH_CFG_TOSEL_ 63to64   (4 << DMACH_CFG_TOSEL_SHIFT) /* 63-64 */
#  define DMACH_CFG_TOSEL_ 127to128 (5 << DMACH_CFG_TOSEL_SHIFT) /* 127-128 */
#  define DMACH_CFG_TOSEL_ 255to256 (6 << DMACH_CFG_TOSEL_SHIFT) /* 255-256 */
#  define DMACH_CFG_TOSEL_ 511to512 (7 << DMACH_CFG_TOSEL_SHIFT) /* 511-512 */
#define DMACH_CFG_PSSEL_SHIFT       (14)      /* Bits 14-15: Pre-Scale Select */
#define DMACH_CFG_PSSEL_MASK        (3 << DMACH_CFG_PSSEL_SHIFT)
#  define DMACH_CFG_PSSEL_DISABLE   (0 << DMACH_CFG_PSSEL_SHIFT) /* Disable timer */
#  define DMACH_CFG_PSSEL_DIV256    (1 << DMACH_CFG_PSSEL_SHIFT) /* fhclk / 256 */
#  define DMACH_CFG_PSSEL_DIV64K    (2 << DMACH_CFG_PSSEL_SHIFT) /* fhclk / 64K */
#  define DMACH_CFG_PSSEL_DIV16M    (3 << DMACH_CFG_PSSEL_SHIFT) /* fhclk / 16M */
#define DMACH_CFG_SRCWD_SHIFT       (16)      /* Bits 16-17: Source Width */
#  define DMACH_CFG_SRCWD_1BYTE     (0 << DMACH_CFG_SRCWD_SHIFT)
#  define DMACH_CFG_SRCWD_2BYTES    (1 << DMACH_CFG_SRCWD_SHIFT)
#  define DMACH_CFG_SRCWD_4BYTES    (2 << DMACH_CFG_SRCWD_SHIFT)
#define DMACH_CFG_SRINC             (1 << 18) /* Bit 18: Source Increment
                                               *         Enable */
#define DMACH_CFG_DSTWD_SHIFT       (20)      /* Bits 20-21: Destination Width */
#define DMACH_CFG_DSTWD_MASK        (3 << DMACH_CFG_DSTWD_SHIFT)
#  define DMACH_CFG_DSTWD_1BYTE     (0 << DMACH_CFG_DSTWD_SHIFT)
#  define DMACH_CFG_DSTWD_2BYTES    (1 << DMACH_CFG_DSTWD_SHIFT)
#  define DMACH_CFG_DSTWD_4BYTES    (2 << DMACH_CFG_DSTWD_SHIFT)
#define DMACH_CFG_DISTINC           (1 << 22) /* Bit 22: Destination Increment
                                               *         Enable */
#define DMACH_CFG_BRST_SHIFT        (24)      /* Bits 24-28: Burst Size */
#define DMACH_CFG_BRST_MASK         (31 << DMACH_CFG_BRST_SHIFT)
#  define DMACH_CFG_BRST(n)         ((uin32_t)((n) - 1) << DMACH_CFG_BRST_SHIFT) /* n=1..32 */
#define DMACH_CFG_CHDIEN            (1 << 30) /* Bit 30: Channel Disable
                                               *         Interrupt Enable */
#define DMACH_CFG_CTZIEN            (1 << 31) /* Bit 31: CTZ Interrupt Enable */

/* DMA Channel Status Register */

#define DMACH_STAT_CHST             (1 << 0)  /* Bit 0:  Channel Status */
#define DMACH_STAT_IPEND            (1 << 1)  /* Bit 1:  Channel Interrupt */
#define DMACH_STAT_CTZST            (1 << 2)  /* Bit 2:  CTZ Status */
#define DMACH_STAT_RLDST            (1 << 3)  /* Bit 3:  Reload Status */
#define DMACH_STAT_BUSERR           (1 << 4)  /* Bit 4:  Bus Error */
#define DMACH_STAT_TOST             (1 << 6)  /* Bit 6:  Time-Out Status */

/* DMA Channel Source Register (32-bit Source Device Address) */
/* DMA Channel Destination Register (32-bit Destination Device Address) */

/* DMA Channel Count Register */

#define DMACH_CNT_MASK              (0x00ffffff) /* Bits 0-23: DMA counter */

/* DMA Channel Source Reload Register */

#define DMACH_SRCRLD_MASK           (0x7fffffff) /* Bits 0-30: Source Address
                                                  *            Reload Value */

/* DMA Channel Destination Reload Register */

#define DMACH_DSTRLD_MASK           (0x7fffffff) /* Bits 0-30: Destination
                                                  *            Address Reload
                                                  *            Value */

/* DMA Channel Count Reload Register */

#define DMACH_CNTRLD_MASK           (0x00ffffff) /* Bits 0-23: Count Reload
                                                  *            Value */
#define DMACH_CNTRLD_RLDEN          (1 << 31)    /* Bit 31: Reload Enable */

#endif /* __ARCH_ARM_SRC_MAX326XX_CHIP_MAX326_DMA_H */
