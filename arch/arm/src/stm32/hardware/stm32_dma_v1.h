/************************************************************************************
 * arch/arm/src/stm32/hardware/stm32_dma_v1.h
 *
 *   Copyright (C) 2009, 2011-2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DMA_V1_DMA_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DMA_V1_DMA_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* This is implementation for STM32 DMA IP version 1 - F0, F1, F3, G4, L0, L1, L4 */

#define HAVE_IP_DMA_V1 1
#undef  HAVE_IP_DMA_V2

/* F0, L0, L4 have additional CSELR register */

#undef DMA_HAVE_CSELR

/* 2 DMA controllers */

#define DMA1                       (0)
#define DMA2                       (1)

/* These definitions apply to multiple STM32 families.
 *
 * The STM32 F1 and F3 families have 12 channels total:
 * 7 DMA1 channels(1-7) and 5 DMA2 channels (1-5).
 *
 * The STM32 G4 family has 16 channels total:
 * 8 DMA1 channels(1-8) and 8 DMA2 channels (1-8).
 */

#define DMA_CHAN1                  (0)
#define DMA_CHAN2                  (1)
#define DMA_CHAN3                  (2)
#define DMA_CHAN4                  (3)
#define DMA_CHAN5                  (4)
#define DMA_CHAN6                  (5)
#define DMA_CHAN7                  (6)
#define DMA_CHAN8                  (7)

/* Register Offsets *****************************************************************/

#define STM32_DMA_ISR_OFFSET       0x0000 /* DMA interrupt status register */
#define STM32_DMA_IFCR_OFFSET      0x0004 /* DMA interrupt flag clear register */

#define STM32_DMACHAN_OFFSET(n)    (0x0014*(n))
#define STM32_DMACHAN1_OFFSET      0x0000
#define STM32_DMACHAN2_OFFSET      0x0014
#define STM32_DMACHAN3_OFFSET      0x0028
#define STM32_DMACHAN4_OFFSET      0x003c
#define STM32_DMACHAN5_OFFSET      0x0050
#define STM32_DMACHAN6_OFFSET      0x0064
#define STM32_DMACHAN7_OFFSET      0x0078
#define STM32_DMACHAN8_OFFSET      0x008c

#define STM32_DMACHAN_CCR_OFFSET   0x0008 /* DMA channel configuration register */
#define STM32_DMACHAN_CNDTR_OFFSET 0x000c /* DMA channel number of data register */
#define STM32_DMACHAN_CPAR_OFFSET  0x0010 /* DMA channel peripheral address register */
#define STM32_DMACHAN_CMAR_OFFSET  0x0014 /* DMA channel 1 memory address register */

#define STM32_DMA_CCR_OFFSET(n)   (STM32_DMACHAN_CCR_OFFSET+STM32_DMACHAN_OFFSET(n))
#define STM32_DMA_CNDTR_OFFSET(n) (STM32_DMACHAN_CNDTR_OFFSET+STM32_DMACHAN_OFFSET(n))
#define STM32_DMA_CPAR_OFFSET(n)  (STM32_DMACHAN_CPAR_OFFSET+STM32_DMACHAN_OFFSET(n))
#define STM32_DMA_CMAR_OFFSET(n)  (STM32_DMACHAN_CMAR_OFFSET+STM32_DMACHAN_OFFSET(n))

#define STM32_DMA_CCR1_OFFSET     0x0008 /* DMA channel 1 configuration register */
#define STM32_DMA_CCR2_OFFSET     0x001c /* DMA channel 2 configuration register */
#define STM32_DMA_CCR3_OFFSET     0x0030 /* DMA channel 3 configuration register */
#define STM32_DMA_CCR4_OFFSET     0x0044 /* DMA channel 4 configuration register */
#define STM32_DMA_CCR5_OFFSET     0x0058 /* DMA channel 5 configuration register */
#define STM32_DMA_CCR6_OFFSET     0x006c /* DMA channel 6 configuration register */
#define STM32_DMA_CCR7_OFFSET     0x0080 /* DMA channel 7 configuration register */
#define STM32_DMA_CCR8_OFFSET     0x0094 /* DMA channel 8 configuration register */

#define STM32_DMA_CNDTR1_OFFSET   0x000c /* DMA channel 1 number of data register */
#define STM32_DMA_CNDTR2_OFFSET   0x0020 /* DMA channel 2 number of data register */
#define STM32_DMA_CNDTR3_OFFSET   0x0034 /* DMA channel 3 number of data register */
#define STM32_DMA_CNDTR4_OFFSET   0x0048 /* DMA channel 4 number of data register */
#define STM32_DMA_CNDTR5_OFFSET   0x005c /* DMA channel 5 number of data register */
#define STM32_DMA_CNDTR6_OFFSET   0x0070 /* DMA channel 6 number of data register */
#define STM32_DMA_CNDTR7_OFFSET   0x0084 /* DMA channel 7 number of data register */
#define STM32_DMA_CNDTR8_OFFSET   0x0098 /* DMA channel 8 number of data register */

#define STM32_DMA_CPAR1_OFFSET    0x0010 /* DMA channel 1 peripheral address register */
#define STM32_DMA_CPAR2_OFFSET    0x0024 /* DMA channel 2 peripheral address register */
#define STM32_DMA_CPAR3_OFFSET    0x0038 /* DMA channel 3 peripheral address register */
#define STM32_DMA_CPAR4_OFFSET    0x004c /* DMA channel 4 peripheral address register */
#define STM32_DMA_CPAR5_OFFSET    0x0060 /* DMA channel 5 peripheral address register */
#define STM32_DMA_CPAR6_OFFSET    0x0074 /* DMA channel 6 peripheral address register */
#define STM32_DMA_CPAR7_OFFSET    0x0088 /* DMA channel 7 peripheral address register */
#define STM32_DMA_CPAR8_OFFSET    0x009c /* DMA channel 8 peripheral address register */

#define STM32_DMA_CMAR1_OFFSET    0x0014 /* DMA channel 1 memory address register */
#define STM32_DMA_CMAR2_OFFSET    0x0028 /* DMA channel 2 memory address register */
#define STM32_DMA_CMAR3_OFFSET    0x003c /* DMA channel 3 memory address register */
#define STM32_DMA_CMAR4_OFFSET    0x0050 /* DMA channel 4 memory address register */
#define STM32_DMA_CMAR5_OFFSET    0x0064 /* DMA channel 5 memory address register */
#define STM32_DMA_CMAR6_OFFSET    0x0078 /* DMA channel 6 memory address register */
#define STM32_DMA_CMAR7_OFFSET    0x008c /* DMA channel 7 memory address register */
#define STM32_DMA_CMAR8_OFFSET    0x00a0 /* DMA channel 8 memory address register */

#ifdef DMA_HAVE_CSELR
#  define STM32_DMA_CSELR_OFFSET  0x00a8 /* DMA channel selection register */
#endif

/* Register Addresses ***************************************************************/

#define STM32_DMA1_ISRC           (STM32_DMA1_BASE+STM32_DMA_ISR_OFFSET)
#define STM32_DMA1_IFCR           (STM32_DMA1_BASE+STM32_DMA_IFCR_OFFSET)

#define STM32_DMA1_CCR(n)         (STM32_DMA1_BASE+STM32_DMA_CCR_OFFSET(n))
#define STM32_DMA1_CCR1           (STM32_DMA1_BASE+STM32_DMA_CCR1_OFFSET)
#define STM32_DMA1_CCR2           (STM32_DMA1_BASE+STM32_DMA_CCR2_OFFSET)
#define STM32_DMA1_CCR3           (STM32_DMA1_BASE+STM32_DMA_CCR3_OFFSET)
#define STM32_DMA1_CCR4           (STM32_DMA1_BASE+STM32_DMA_CCR4_OFFSET)
#define STM32_DMA1_CCR5           (STM32_DMA1_BASE+STM32_DMA_CCR5_OFFSET)
#define STM32_DMA1_CCR6           (STM32_DMA1_BASE+STM32_DMA_CCR6_OFFSET)
#define STM32_DMA1_CCR7           (STM32_DMA1_BASE+STM32_DMA_CCR7_OFFSET)
#if defined(CONFIG_STM32_HAVE_DMA1_CHAN8)
#  define STM32_DMA1_CCR8         (STM32_DMA1_BASE+STM32_DMA_CCR8_OFFSET)
#endif

#define STM32_DMA1_CNDTR(n)       (STM32_DMA1_BASE+STM32_DMA_CNDTR_OFFSET(n))
#define STM32_DMA1_CNDTR1         (STM32_DMA1_BASE+STM32_DMA_CNDTR1_OFFSET)
#define STM32_DMA1_CNDTR2         (STM32_DMA1_BASE+STM32_DMA_CNDTR2_OFFSET)
#define STM32_DMA1_CNDTR3         (STM32_DMA1_BASE+STM32_DMA_CNDTR3_OFFSET)
#define STM32_DMA1_CNDTR4         (STM32_DMA1_BASE+STM32_DMA_CNDTR4_OFFSET)
#define STM32_DMA1_CNDTR5         (STM32_DMA1_BASE+STM32_DMA_CNDTR5_OFFSET)
#define STM32_DMA1_CNDTR6         (STM32_DMA1_BASE+STM32_DMA_CNDTR6_OFFSET)
#define STM32_DMA1_CNDTR7         (STM32_DMA1_BASE+STM32_DMA_CNDTR7_OFFSET)
#if defined(CONFIG_STM32_HAVE_DMA1_CHAN8)
#  define STM32_DMA1_CNDTR8       (STM32_DMA1_BASE+STM32_DMA_CNDTR8_OFFSET)
#endif

#define STM32_DMA1_CPAR(n)        (STM32_DMA1_BASE+STM32_DMA_CPAR_OFFSET(n))
#define STM32_DMA1_CPAR1          (STM32_DMA1_BASE+STM32_DMA_CPAR1_OFFSET)
#define STM32_DMA1_CPAR2          (STM32_DMA1_BASE+STM32_DMA_CPAR2_OFFSET)
#define STM32_DMA1_CPAR3          (STM32_DMA1_BASE+STM32_DMA_CPAR3_OFFSET)
#define STM32_DMA1_CPAR4          (STM32_DMA1_BASE+STM32_DMA_CPAR4_OFFSET)
#define STM32_DMA1_CPAR5          (STM32_DMA1_BASE+STM32_DMA_CPAR5_OFFSET)
#define STM32_DMA1_CPAR6          (STM32_DMA1_BASE+STM32_DMA_CPAR6_OFFSET)
#define STM32_DMA1_CPAR7          (STM32_DMA1_BASE+STM32_DMA_CPAR7_OFFSET)
#if defined(CONFIG_STM32_HAVE_DMA1_CHAN8)
#  define STM32_DMA1_CPAR8        (STM32_DMA1_BASE+STM32_DMA_CPAR8_OFFSET)
#endif

#define STM32_DMA1_CMAR(n)        (STM32_DMA1_BASE+STM32_DMA_CMAR_OFFSET(n))
#define STM32_DMA1_CMAR1          (STM32_DMA1_BASE+STM32_DMA_CMAR1_OFFSET)
#define STM32_DMA1_CMAR2          (STM32_DMA1_BASE+STM32_DMA_CMAR2_OFFSET)
#define STM32_DMA1_CMAR3          (STM32_DMA1_BASE+STM32_DMA_CMAR3_OFFSET)
#define STM32_DMA1_CMAR4          (STM32_DMA1_BASE+STM32_DMA_CMAR4_OFFSET)
#define STM32_DMA1_CMAR5          (STM32_DMA1_BASE+STM32_DMA_CMAR5_OFFSET)
#define STM32_DMA1_CMAR6          (STM32_DMA1_BASE+STM32_DMA_CMAR6_OFFSET)
#define STM32_DMA1_CMAR7          (STM32_DMA1_BASE+STM32_DMA_CMAR7_OFFSET)
#if defined(CONFIG_STM32_HAVE_DMA1_CHAN8)
#  define STM32_DMA1_CMAR8        (STM32_DMA1_BASE+STM32_DMA_CMAR8_OFFSET)
#endif

#define STM32_DMA2_ISRC           (STM32_DMA2_BASE+STM32_DMA_ISR_OFFSET)
#define STM32_DMA2_IFCR           (STM32_DMA2_BASE+STM32_DMA_IFCR_OFFSET)

#define STM32_DMA2_CCR(n)         (STM32_DMA2_BASE+STM32_DMA_CCR_OFFSET(n))
#define STM32_DMA2_CCR1           (STM32_DMA2_BASE+STM32_DMA_CCR1_OFFSET)
#define STM32_DMA2_CCR2           (STM32_DMA2_BASE+STM32_DMA_CCR2_OFFSET)
#define STM32_DMA2_CCR3           (STM32_DMA2_BASE+STM32_DMA_CCR3_OFFSET)
#define STM32_DMA2_CCR4           (STM32_DMA2_BASE+STM32_DMA_CCR4_OFFSET)
#define STM32_DMA2_CCR5           (STM32_DMA2_BASE+STM32_DMA_CCR5_OFFSET)
#if defined(CONFIG_STM32_HAVE_DMA2_CHAN678)
#  define STM32_DMA2_CCR6         (STM32_DMA2_BASE+STM32_DMA_CCR6_OFFSET)
#  define STM32_DMA2_CCR7         (STM32_DMA2_BASE+STM32_DMA_CCR7_OFFSET)
#  define STM32_DMA2_CCR8         (STM32_DMA2_BASE+STM32_DMA_CCR8_OFFSET)
#endif

#define STM32_DMA2_CNDTR(n)       (STM32_DMA2_BASE+STM32_DMA_CNDTR_OFFSET(n))
#define STM32_DMA2_CNDTR1         (STM32_DMA2_BASE+STM32_DMA_CNDTR1_OFFSET)
#define STM32_DMA2_CNDTR2         (STM32_DMA2_BASE+STM32_DMA_CNDTR2_OFFSET)
#define STM32_DMA2_CNDTR3         (STM32_DMA2_BASE+STM32_DMA_CNDTR3_OFFSET)
#define STM32_DMA2_CNDTR4         (STM32_DMA2_BASE+STM32_DMA_CNDTR4_OFFSET)
#define STM32_DMA2_CNDTR5         (STM32_DMA2_BASE+STM32_DMA_CNDTR5_OFFSET)
#if defined(CONFIG_STM32_HAVE_DMA2_CHAN678)
#  define STM32_DMA2_CNDTR6       (STM32_DMA2_BASE+STM32_DMA_CNDTR6_OFFSET)
#  define STM32_DMA2_CNDTR7       (STM32_DMA2_BASE+STM32_DMA_CNDTR7_OFFSET)
#  define STM32_DMA2_CNDTR8       (STM32_DMA2_BASE+STM32_DMA_CNDTR8_OFFSET)
#endif

#define STM32_DMA2_CPAR(n)        (STM32_DMA2_BASE+STM32_DMA_CPAR_OFFSET(n))
#define STM32_DMA2_CPAR1          (STM32_DMA2_BASE+STM32_DMA_CPAR1_OFFSET)
#define STM32_DMA2_CPAR2          (STM32_DMA2_BASE+STM32_DMA_CPAR2_OFFSET)
#define STM32_DMA2_CPAR3          (STM32_DMA2_BASE+STM32_DMA_CPAR3_OFFSET)
#define STM32_DMA2_CPAR4          (STM32_DMA2_BASE+STM32_DMA_CPAR4_OFFSET)
#define STM32_DMA2_CPAR5          (STM32_DMA2_BASE+STM32_DMA_CPAR5_OFFSET)
#if defined(CONFIG_STM32_HAVE_DMA2_CHAN678)
#  define STM32_DMA2_CPAR6        (STM32_DMA2_BASE+STM32_DMA_CPAR6_OFFSET)
#  define STM32_DMA2_CPAR7        (STM32_DMA2_BASE+STM32_DMA_CPAR7_OFFSET)
#  define STM32_DMA2_CPAR8        (STM32_DMA2_BASE+STM32_DMA_CPAR8_OFFSET)
#endif

#define STM32_DMA2_CMAR(n)        (STM32_DMA2_BASE+STM32_DMA_CMAR_OFFSET(n))
#define STM32_DMA2_CMAR1          (STM32_DMA2_BASE+STM32_DMA_CMAR1_OFFSET)
#define STM32_DMA2_CMAR2          (STM32_DMA2_BASE+STM32_DMA_CMAR2_OFFSET)
#define STM32_DMA2_CMAR3          (STM32_DMA2_BASE+STM32_DMA_CMAR3_OFFSET)
#define STM32_DMA2_CMAR4          (STM32_DMA2_BASE+STM32_DMA_CMAR4_OFFSET)
#define STM32_DMA2_CMAR5          (STM32_DMA2_BASE+STM32_DMA_CMAR5_OFFSET)
#if defined(CONFIG_STM32_HAVE_DMA2_CHAN678)
#  define STM32_DMA2_CMAR6        (STM32_DMA2_BASE+STM32_DMA_CMAR6_OFFSET)
#  define STM32_DMA2_CMAR7        (STM32_DMA2_BASE+STM32_DMA_CMAR7_OFFSET)
#  define STM32_DMA2_CMAR8        (STM32_DMA2_BASE+STM32_DMA_CMAR8_OFFSET)
#endif

/* Register Bitfield Definitions ****************************************************/

#define DMA_CHAN_SHIFT(n)         ((n) << 2)
#define DMA_CHAN_MASK             0x0f
#define DMA_CHAN_GIF_BIT          (1 << 0)  /* Bit 0: Channel Global interrupt flag */
#define DMA_CHAN_TCIF_BIT         (1 << 1)  /* Bit 1: Channel Transfer Complete flag */
#define DMA_CHAN_HTIF_BIT         (1 << 2)  /* Bit 2: Channel Half Transfer flag */
#define DMA_CHAN_TEIF_BIT         (1 << 3)  /* Bit 3: Channel Transfer Error flag */

/* DMA interrupt status register */

#define DMA_ISR_CHAN_SHIFT(n)     DMA_CHAN_SHIFT(n)
#define DMA_ISR_CHAN_MASK(n)      (DMA_CHAN_MASK <<  DMA_ISR_CHAN_SHIFT(n))
#define DMA_ISR_CHAN1_SHIFT       (0)       /* Bits 3-0:  DMA Channel 1 interrupt status */
#define DMA_ISR_CHAN1_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN1_SHIFT)
#define DMA_ISR_CHAN2_SHIFT       (4)       /* Bits 7-4:  DMA Channel 2 interrupt status */
#define DMA_ISR_CHAN2_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN2_SHIFT)
#define DMA_ISR_CHAN3_SHIFT       (8)       /* Bits 11-8:  DMA Channel 3 interrupt status */
#define DMA_ISR_CHAN3_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN3_SHIFT)
#define DMA_ISR_CHAN4_SHIFT       (12)      /* Bits 15-12:  DMA Channel 4 interrupt status */
#define DMA_ISR_CHAN4_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN4_SHIFT)
#define DMA_ISR_CHAN5_SHIFT       (16)      /* Bits 19-16:  DMA Channel 5 interrupt status */
#define DMA_ISR_CHAN5_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN5_SHIFT)
#define DMA_ISR_CHAN6_SHIFT       (20)      /* Bits 23-20:  DMA Channel 6 interrupt status */
#define DMA_ISR_CHAN6_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN6_SHIFT)
#define DMA_ISR_CHAN7_SHIFT       (24)      /* Bits 27-24:  DMA Channel 7 interrupt status */
#define DMA_ISR_CHAN7_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN7_SHIFT)
#define DMA_ISR_CHAN8_SHIFT       (28)      /* Bits 31-28:  DMA Channel 8 interrupt status */
#define DMA_ISR_CHAN8_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN8_SHIFT)

#define DMA_ISR_GIF(n)            (DMA_CHAN_GIF_BIT << DMA_ISR_CHAN_SHIFT(n))
#define DMA_ISR_TCIF(n)           (DMA_CHAN_TCIF_BIT << DMA_ISR_CHAN_SHIFT(n))
#define DMA_ISR_HTIF(n)           (DMA_CHAN_HTIF_BIT << DMA_ISR_CHAN_SHIFT(n))
#define DMA_ISR_TEIF(n)           (DMA_CHAN_TEIF_BIT << DMA_ISR_CHAN_SHIFT(n))

/* DMA interrupt flag clear register */

#define DMA_IFCR_CHAN_SHIFT(n)    DMA_CHAN_SHIFT(n)
#define DMA_IFCR_CHAN_MASK(n)     (DMA_CHAN_MASK <<  DMA_IFCR_CHAN_SHIFT(n))
#define DMA_IFCR_CHAN1_SHIFT      (0)       /* Bits 3-0:  DMA Channel 1 interrupt flag clear */
#define DMA_IFCR_CHAN1_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN1_SHIFT)
#define DMA_IFCR_CHAN2_SHIFT      (4)       /* Bits 7-4:  DMA Channel 2 interrupt flag clear */
#define DMA_IFCR_CHAN2_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN2_SHIFT)
#define DMA_IFCR_CHAN3_SHIFT      (8)       /* Bits 11-8:  DMA Channel 3 interrupt flag clear */
#define DMA_IFCR_CHAN3_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN3_SHIFT)
#define DMA_IFCR_CHAN4_SHIFT      (12)      /* Bits 15-12:  DMA Channel 4 interrupt flag clear */
#define DMA_IFCR_CHAN4_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN4_SHIFT)
#define DMA_IFCR_CHAN5_SHIFT      (16)      /* Bits 19-16:  DMA Channel 5 interrupt flag clear */
#define DMA_IFCR_CHAN5_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN5_SHIFT)
#define DMA_IFCR_CHAN6_SHIFT      (20)      /* Bits 23-20:  DMA Channel 6 interrupt flag clear */
#define DMA_IFCR_CHAN6_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN6_SHIFT)
#define DMA_IFCR_CHAN7_SHIFT      (24)      /* Bits 27-24:  DMA Channel 7 interrupt flag clear */
#define DMA_IFCR_CHAN7_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN7_SHIFT)
#define DMA_IFCR_CHAN8_SHIFT      (28)      /* Bits 31-28:  DMA Channel 8 interrupt flag clear */
#define DMA_IFCR_CHAN8_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN8_SHIFT)

#if defined(CONFIG_STM32_HAVE_DMA1_CHAN8) || defined(CONFIG_STM32_HAVE_DMA2_CHAN678)
#  define DMA_IFCR_ALLCHANNELS    (0xffffffff)
#else
#  define DMA_IFCR_ALLCHANNELS    (0x0fffffff)
#endif

#define DMA_IFCR_CGIF(n)          (DMA_CHAN_GIF_BIT << DMA_IFCR_CHAN_SHIFT(n))
#define DMA_IFCR_CTCIF(n)         (DMA_CHAN_TCIF_BIT << DMA_IFCR_CHAN_SHIFT(n))
#define DMA_IFCR_CHTIF(n)         (DMA_CHAN_HTIF_BIT << DMA_IFCR_CHAN_SHIFT(n))
#define DMA_IFCR_CTEIF(n)         (DMA_CHAN_TEIF_BIT << DMA_IFCR_CHAN_SHIFT(n))

/* DMA channel configuration register */

#define DMA_CCR_EN                (1 << 0)                   /* Bit 0: Channel enable */
#define DMA_CCR_TCIE              (1 << 1)                   /* Bit 1: Transfer complete interrupt enable */
#define DMA_CCR_HTIE              (1 << 2)                   /* Bit 2: Half Transfer interrupt enable */
#define DMA_CCR_TEIE              (1 << 3)                   /* Bit 3: Transfer error interrupt enable */
#define DMA_CCR_DIR               (1 << 4)                   /* Bit 4: Data transfer direction */
#define DMA_CCR_CIRC              (1 << 5)                   /* Bit 5: Circular mode */
#define DMA_CCR_PINC              (1 << 6)                   /* Bit 6: Peripheral increment mode */
#define DMA_CCR_MINC              (1 << 7)                   /* Bit 7: Memory increment mode */
#define DMA_CCR_PSIZE_SHIFT       (8)                        /* Bits 8-9: Peripheral size */
#define DMA_CCR_PSIZE_MASK        (3 << DMA_CCR_PSIZE_SHIFT)
#  define DMA_CCR_PSIZE_8BITS     (0 << DMA_CCR_PSIZE_SHIFT) /* 00: 8-bits */
#  define DMA_CCR_PSIZE_16BITS    (1 << DMA_CCR_PSIZE_SHIFT) /* 01: 16-bits */
#  define DMA_CCR_PSIZE_32BITS    (2 << DMA_CCR_PSIZE_SHIFT) /* 10: 32-bits */
#define DMA_CCR_MSIZE_SHIFT       (10)                       /* Bits 10-11: Memory size */
#define DMA_CCR_MSIZE_MASK        (3 << DMA_CCR_MSIZE_SHIFT)
#  define DMA_CCR_MSIZE_8BITS     (0 << DMA_CCR_MSIZE_SHIFT) /* 00: 8-bits */
#  define DMA_CCR_MSIZE_16BITS    (1 << DMA_CCR_MSIZE_SHIFT) /* 01: 16-bits */
#  define DMA_CCR_MSIZE_32BITS    (2 << DMA_CCR_MSIZE_SHIFT) /* 10: 32-bits */
#define DMA_CCR_PL_SHIFT          (12)                       /* Bits 12-13: Channel Priority level */
#define DMA_CCR_PL_MASK           (3 << DMA_CCR_PL_SHIFT)
#  define DMA_CCR_PRILO           (0 << DMA_CCR_PL_SHIFT)    /* 00: Low */
#  define DMA_CCR_PRIMED          (1 << DMA_CCR_PL_SHIFT)    /* 01: Medium */
#  define DMA_CCR_PRIHI           (2 << DMA_CCR_PL_SHIFT)    /* 10: High */
#  define DMA_CCR_PRIVERYHI       (3 << DMA_CCR_PL_SHIFT)    /* 11: Very high */
#define DMA_CCR_MEM2MEM           (1 << 14)                  /* Bit 14: Memory to memory mode */

#define DMA_CCR_ALLINTS           (DMA_CCR_TEIE|DMA_CCR_HTIE|DMA_CCR_TCIE)

/* DMA channel number of data register */

#define DMA_CNDTR_NDT_SHIFT       (0)       /* Bits 15-0: Number of data to Transfer */
#define DMA_CNDTR_NDT_MASK        (0xffff << DMA_CNDTR_NDT_SHIFT)

/* DMA Channel mapping.  Each DMA channel has a mapping to several possible
 * sources/sinks of data.  The requests from peripherals assigned to a channel
 * are simply OR'ed together before entering the DMA block.  This means that only
 * one request on a given channel can be enabled at once.
 *
 * Alternative DMA channel selections are provided with a numeric suffix like _1,
 * _2, etc.  Drivers, however, will use the pin selection without the numeric suffix.
 * Additional definitions are required in the board.h file.
 */

#define STM32_DMA1_CHAN1          (0)
#define STM32_DMA1_CHAN2          (1)
#define STM32_DMA1_CHAN3          (2)
#define STM32_DMA1_CHAN4          (3)
#define STM32_DMA1_CHAN5          (4)
#define STM32_DMA1_CHAN6          (5)
#define STM32_DMA1_CHAN7          (6)
#if defined(CONFIG_STM32_HAVE_DMA1_CHAN8)
#  define STM32_DMA1_CHAN8        (7)
#  define STM32_DMA2_CHAN1        (8)
#  define STM32_DMA2_CHAN2        (9)
#  define STM32_DMA2_CHAN3        (10)
#  define STM32_DMA2_CHAN4        (11)
#  define STM32_DMA2_CHAN5        (12)
#  if defined(CONFIG_STM32_HAVE_DMA2_CHAN678)
#    define STM32_DMA2_CHAN6      (13)
#    define STM32_DMA2_CHAN7      (14)
#    define STM32_DMA2_CHAN8      (15)
#  endif
#else
#  define STM32_DMA2_CHAN1        (7)
#  define STM32_DMA2_CHAN2        (8)
#  define STM32_DMA2_CHAN3        (9)
#  define STM32_DMA2_CHAN4        (10)
#  define STM32_DMA2_CHAN5        (11)
#  if defined(CONFIG_STM32_HAVE_DMA2_CHAN678)
#    define STM32_DMA2_CHAN6      (12)
#    define STM32_DMA2_CHAN7      (13)
#    define STM32_DMA2_CHAN8      (14)
#  endif
#endif

#ifdef DMA_HAVE_CSELR
#  define DMACHAN_SETTING(chan, sel)     ((((sel) & 0xff) << 8) | ((chan) & 0xff))
#  define DMACHAN_SETTING_CHANNEL_MASK   0x00ff
#  define DMACHAN_SETTING_CHANNEL_SHIFT  (0)
#  define DMACHAN_SETTING_FUNCTION_MASK  0xff00
#  define DMACHAN_SETTING_FUNCTION_SHIFT (8)
#endif

#if defined(CONFIG_STM32_STM32L15XX)

#  define DMACHAN_ADC1            STM32_DMA1_CHAN1
#  define DMACHAN_TIM2_CH3        STM32_DMA1_CHAN1
#  define DMACHAN_TIM4_CH1        STM32_DMA1_CHAN1

#  define DMACHAN_SPI1_RX         STM32_DMA1_CHAN2
#  define DMACHAN_USART3_TX       STM32_DMA1_CHAN2
#  define DMACHAN_TIM2_UP         STM32_DMA1_CHAN2
#  define DMACHAN_TIM3_CH3        STM32_DMA1_CHAN2
#  define DMACHAN_TIM6_UP         STM32_DMA1_CHAN2
#  define DMACHAN_DAC1_CH1        STM32_DMA1_CHAN2

#  define DMACHAN_SPI1_TX         STM32_DMA1_CHAN3
#  define DMACHAN_USART3_RX       STM32_DMA1_CHAN3
#  define DMACHAN_TIM3_CH4        STM32_DMA1_CHAN3
#  define DMACHAN_TIM3_UP         STM32_DMA1_CHAN3
#  define DMACHAN_TIM7_UP         STM32_DMA1_CHAN3
#  define DMACHAN_DAC1_CH2        STM32_DMA1_CHAN3

#  define DMACHAN_SPI2_RX         STM32_DMA1_CHAN4
#  define DMACHAN_USART1_TX       STM32_DMA1_CHAN4
#  define DMACHAN_I2C2_TX         STM32_DMA1_CHAN4
#  define DMACHAN_TIM4_CH2        STM32_DMA1_CHAN4

#  define DMACHAN_SPI2_TX         STM32_DMA1_CHAN5
#  define DMACHAN_USART1_RX       STM32_DMA1_CHAN5
#  define DMACHAN_I2C2_RX         STM32_DMA1_CHAN5
#  define DMACHAN_TIM2_CH1        STM32_DMA1_CHAN5
#  define DMACHAN_TIM4_CH3        STM32_DMA1_CHAN5

#  define DMACHAN_USART2_RX       STM32_DMA1_CHAN6
#  define DMACHAN_I2C1_TX         STM32_DMA1_CHAN6
#  define DMACHAN_TIM3_CH1        STM32_DMA1_CHAN6
#  define DMACHAN_TIM3_TRIG       STM32_DMA1_CHAN6

#  define DMACHAN_USART2_TX       STM32_DMA1_CHAN7
#  define DMACHAN_I2C1_RX         STM32_DMA1_CHAN7
#  define DMACHAN_TIM2_CH2        STM32_DMA1_CHAN7
#  define DMACHAN_TIM2_CH4        STM32_DMA1_CHAN7
#  define DMACHAN_TIM4_UP         STM32_DMA1_CHAN7

#  define DMACHAN_SPI3_RX         STM32_DMA2_CHAN1
#  define DMACHAN_UART5_TX        STM32_DMA2_CHAN1
#  define DMACHAN_TIM5_CH4        STM32_DMA2_CHAN1
#  define DMACHAN_TIM5_TRIG       STM32_DMA2_CHAN1
#  define DMACHAN_TIM5_COM        STM32_DMA2_CHAN1

#  define DMACHAN_SPI3_TX         STM32_DMA2_CHAN2
#  define DMACHAN_UART5_RX        STM32_DMA2_CHAN2
#  define DMACHAN_TIM5_CH3        STM32_DMA2_CHAN2
#  define DMACHAN_TIM5_UP         STM32_DMA2_CHAN2

#  define DMACHAN_UART4_RX        STM32_DMA2_CHAN3
#  define DMACHAN_AES_OUT         STM32_DMA2_CHAN3

#  define DMACHAN_TIM5_CH2        STM32_DMA2_CHAN4
#  define DMACHAN_SDIO            STM32_DMA2_CHAN4

#  define DMACHAN_UART4_TX        STM32_DMA2_CHAN5
#  define DMACHAN_TIM5_CH1        STM32_DMA2_CHAN5
#  define DMACHAN_AES_IN          STM32_DMA2_CHAN5

#elif defined(CONFIG_STM32_STM32F10XX)

#  define DMACHAN_ADC1            STM32_DMA1_CHAN1
#  define DMACHAN_TIM2_CH3        STM32_DMA1_CHAN1
#  define DMACHAN_TIM4_CH1        STM32_DMA1_CHAN1

#  define DMACHAN_SPI1_RX         STM32_DMA1_CHAN2
#  define DMACHAN_USART3_TX       STM32_DMA1_CHAN2
#  define DMACHAN_TIM1_CH1        STM32_DMA1_CHAN2
#  define DMACHAN_TIM2_UP         STM32_DMA1_CHAN2
#  define DMACHAN_TIM3_CH3        STM32_DMA1_CHAN2

#  define DMACHAN_SPI1_TX         STM32_DMA1_CHAN3
#  define DMACHAN_USART3_RX       STM32_DMA1_CHAN3
#  define DMACHAN_TIM1_CH2        STM32_DMA1_CHAN3
#  define DMACHAN_TIM3_CH4        STM32_DMA1_CHAN3
#  define DMACHAN_TIM3_UP         STM32_DMA1_CHAN3

#  define DMACHAN_SPI2_RX         STM32_DMA1_CHAN4
#  define DMACHAN_I2S2_RX         STM32_DMA1_CHAN4
#  define DMACHAN_USART1_TX       STM32_DMA1_CHAN4
#  define DMACHAN_I2C2_TX         STM32_DMA1_CHAN4
#  define DMACHAN_TIM1_CH4        STM32_DMA1_CHAN4
#  define DMACHAN_TIM1_TRIG       STM32_DMA1_CHAN4
#  define DMACHAN_TIM1_COM        STM32_DMA1_CHAN4
#  define DMACHAN_TIM4_CH2        STM32_DMA1_CHAN4

#  define DMACHAN_SPI2_TX         STM32_DMA1_CHAN5
#  define DMACHAN_I2S2_TX         STM32_DMA1_CHAN5
#  define DMACHAN_USART1_RX       STM32_DMA1_CHAN5
#  define DMACHAN_I2C2_RX         STM32_DMA1_CHAN5
#  define DMACHAN_TIM1_UP         STM32_DMA1_CHAN5
#  define DMACHAN_TIM2_CH1        STM32_DMA1_CHAN5
#  define DMACHAN_TIM4_CH3        STM32_DMA1_CHAN5

#  define DMACHAN_USART2_RX       STM32_DMA1_CHAN6
#  define DMACHAN_I2C1_TX         STM32_DMA1_CHAN6
#  define DMACHAN_TIM1_CH3        STM32_DMA1_CHAN6
#  define DMACHAN_TIM3_CH1        STM32_DMA1_CHAN6
#  define DMACHAN_TIM3_TRIG       STM32_DMA1_CHAN6

#  define DMACHAN_USART2_TX       STM32_DMA1_CHAN7
#  define DMACHAN_I2C1_RX         STM32_DMA1_CHAN7
#  define DMACHAN_TIM2_CH2        STM32_DMA1_CHAN7
#  define DMACHAN_TIM2_CH4        STM32_DMA1_CHAN7
#  define DMACHAN_TIM4_UP         STM32_DMA1_CHAN7

#  define DMACHAN_SPI3_RX         STM32_DMA2_CHAN1
#  define DMACHAN_I2S3_RX         STM32_DMA2_CHAN1
#  define DMACHAN_TIM5_CH4        STM32_DMA2_CHAN1
#  define DMACHAN_TIM5_TRIG       STM32_DMA2_CHAN1
#  define DMACHAN_TIM8_CH3        STM32_DMA2_CHAN1
#  define DMACHAN_TIM8_UP         STM32_DMA2_CHAN1

#  define DMACHAN_SPI3_TX         STM32_DMA2_CHAN2
#  define DMACHAN_I2S3_TX         STM32_DMA2_CHAN2
#  define DMACHAN_TIM5_CH3        STM32_DMA2_CHAN2
#  define DMACHAN_TIM5_UP         STM32_DMA2_CHAN2
#  define DMACHAN_TIM8_TRIG       STM32_DMA2_CHAN2
#  define DMACHAN_TIM8_COM        STM32_DMA2_CHAN2

#  define DMACHAN_UART4_RX        STM32_DMA2_CHAN3
#  define DMACHAN_TIM6_UP         STM32_DMA2_CHAN3
#  define DMACHAN_DAC1_CH1        STM32_DMA2_CHAN3
#  define DMACHAN_TIM8_CH1        STM32_DMA2_CHAN3

#  define DMACHAN_SDIO            STM32_DMA2_CHAN4
#  define DMACHAN_TIM5_CH2        STM32_DMA2_CHAN4
#  define DMACHAN_TIM7_UP         STM32_DMA2_CHAN4
#  define DMACHAN_DAC1_CH2        STM32_DMA2_CHAN4

#  define DMACHAN_ADC3            STM32_DMA2_CHAN5
#  define DMACHAN_UART4_TX        STM32_DMA2_CHAN5
#  define DMACHAN_TIM5_CH1        STM32_DMA2_CHAN5
#  define DMACHAN_TIM8_CH2        STM32_DMA2_CHAN5

#elif defined(CONFIG_STM32_STM32F30XX)

#  define DMACHAN_ADC1            STM32_DMA1_CHAN1
#  define DMACHAN_TIM2_CH3        STM32_DMA1_CHAN1
#  define DMACHAN_TIM4_CH1        STM32_DMA1_CHAN1
#  define DMACHAN_TIM17_CH1       STM32_DMA1_CHAN1
#  define DMACHAN_TIM17_UP        STM32_DMA1_CHAN1

#  define DMACHAN_SPI1_RX         STM32_DMA1_CHAN2
#  define DMACHAN_USART3_TX       STM32_DMA1_CHAN2
#  define DMACHAN_TIM1_CH1        STM32_DMA1_CHAN2
#  define DMACHAN_TIM2_UP         STM32_DMA1_CHAN2
#  define DMACHAN_TIM3_CH3        STM32_DMA1_CHAN2

#  define DMACHAN_SPI1_TX         STM32_DMA1_CHAN3
#  define DMACHAN_USART3_RX       STM32_DMA1_CHAN3
#  define DMACHAN_TIM1_CH2_1      STM32_DMA1_CHAN3
#  define DMACHAN_TIM3_CH4        STM32_DMA1_CHAN3
#  define DMACHAN_TIM3_UP_2       STM32_DMA1_CHAN3
#  define DMACHAN_DAC1_CH1_1      STM32_DMA1_CHAN3

#  define DMACHAN_SPI2_RX         STM32_DMA1_CHAN4
#  define DMACHAN_I2S2_RX         STM32_DMA1_CHAN4
#  define DMACHAN_USART1_TX       STM32_DMA1_CHAN4
#  define DMACHAN_I2C2_TX         STM32_DMA1_CHAN4
#  define DMACHAN_TIM1_CH4        STM32_DMA1_CHAN4
#  define DMACHAN_TIM1_TRIG       STM32_DMA1_CHAN4
#  define DMACHAN_TIM1_COM        STM32_DMA1_CHAN4
#  define DMACHAN_TIM4_CH2        STM32_DMA1_CHAN4
#  define DMACHAN_TIM7_UP_1       STM32_DMA1_CHAN4
#  define DMACHAN_DAC1_CH2_1      STM32_DMA1_CHAN4 /* NOTE: a typo in the ref manual */

#  define DMACHAN_SPI2_TX         STM32_DMA1_CHAN5
#  define DMACHAN_I2S2_TX         STM32_DMA1_CHAN5
#  define DMACHAN_USART1_RX       STM32_DMA1_CHAN5
#  define DMACHAN_I2C2_RX         STM32_DMA1_CHAN5
#  define DMACHAN_TIM1_UP         STM32_DMA1_CHAN5
#  define DMACHAN_TIM2_CH1        STM32_DMA1_CHAN5
#  define DMACHAN_TIM4_CH3        STM32_DMA1_CHAN5
#  define DMACHAN_TIM15_CH1       STM32_DMA1_CHAN5
#  define DMACHAN_TIM15_UP        STM32_DMA1_CHAN5
#  define DMACHAN_TIM15_TRIG      STM32_DMA1_CHAN5
#  define DMACHAN_TIM15_COM       STM32_DMA1_CHAN5
#  define DMACHAN_DAC2_CH1        STM32_DMA1_CHAN5

#  define DMACHAN_USART2_RX       STM32_DMA1_CHAN6
#  define DMACHAN_I2C1_TX         STM32_DMA1_CHAN6
#  define DMACHAN_TIM1_CH3        STM32_DMA1_CHAN6
#  define DMACHAN_TIM3_CH1        STM32_DMA1_CHAN6
#  define DMACHAN_TIM3_TRIG       STM32_DMA1_CHAN6
#  define DMACHAN_TIM16_CH1       STM32_DMA1_CHAN6
#  define DMACHAN_TIM16_UP        STM32_DMA1_CHAN6

#  define DMACHAN_USART2_TX       STM32_DMA1_CHAN7
#  define DMACHAN_I2C1_RX         STM32_DMA1_CHAN7
#  define DMACHAN_TIM2_CH2        STM32_DMA1_CHAN7
#  define DMACHAN_TIM2_CH4        STM32_DMA1_CHAN7
#  define DMACHAN_TIM4_UP         STM32_DMA1_CHAN7
#  define DMACHAN_TIM17_CH1_2     STM32_DMA1_CHAN7
#  define DMACHAN_TIM17_UP_2      STM32_DMA1_CHAN7

#  define DMACHAN_ADC2_1          STM32_DMA2_CHAN1
#  define DMACHAN_SPI3_RX         STM32_DMA2_CHAN1
#  define DMACHAN_I2S3_RX         STM32_DMA2_CHAN1
#  define DMACHAN_TIM8_CH3        STM32_DMA2_CHAN1
#  define DMACHAN_TIM8_UP         STM32_DMA2_CHAN1

#  define DMACHAN_ADC4_1          STM32_DMA2_CHAN2
#  define DMACHAN_SPI3_TX         STM32_DMA2_CHAN2
#  define DMACHAN_I2S3_TX         STM32_DMA2_CHAN2
#  define DMACHAN_TIM8_CH4        STM32_DMA2_CHAN2
#  define DMACHAN_TIM8_TRIG       STM32_DMA2_CHAN2
#  define DMACHAN_TIM8_COM        STM32_DMA2_CHAN2

#  define DMACHAN_ADC2_2          STM32_DMA2_CHAN3
#  define DMACHAN_UART4_RX        STM32_DMA2_CHAN3
#  define DMACHAN_TIM6_UP         STM32_DMA2_CHAN3
#  define DMACHAN_DAC1_CH1_2      STM32_DMA2_CHAN3
#  define DMACHAN_TIM8_CH1        STM32_DMA2_CHAN3

#  define DMACHAN_ADC4_2          STM32_DMA2_CHAN4
#  define DMACHAN_TIM7_UP_2       STM32_DMA2_CHAN4
#  define DMACHAN_DAC1_CH2_2      STM32_DMA2_CHAN4

#  define DMACHAN_ADC3            STM32_DMA2_CHAN5
#  define DMACHAN_UART4_TX        STM32_DMA2_CHAN5
#  define DMACHAN_TIM8_CH2        STM32_DMA2_CHAN5

#elif defined(CONFIG_STM32_STM32F33XX)

#  define DMACHAN_ADC1            STM32_DMA1_CHAN1
#  define DMACHAN_TIM2_CH3        STM32_DMA1_CHAN1
#  define DMACHAN_TIM17_CH1_1     STM32_DMA1_CHAN1
#  define DMACHAN_TIM17_UP_1      STM32_DMA1_CHAN1

#  define DMACHAN_ADC2_1          STM32_DMA1_CHAN2
#  define DMACHAN_SPI1_RX_1       STM32_DMA1_CHAN2
#  define DMACHAN_USART3_TX       STM32_DMA1_CHAN2
#  define DMACHAN_I2C1_TX_3       STM32_DMA1_CHAN4
#  define DMACHAN_TIM1_CH1        STM32_DMA1_CHAN2
#  define DMACHAN_TIM2_UP         STM32_DMA1_CHAN2
#  define DMACHAN_TIM3_CH3        STM32_DMA1_CHAN2
#  define DMACHAN_HRTIM1_M        STM32_DMA1_CHAN2

#  define DMACHAN_SPI1_TX_1       STM32_DMA1_CHAN3
#  define DMACHAN_USART3_RX       STM32_DMA1_CHAN3
#  define DMACHAN_I2C1_RX_2       STM32_DMA1_CHAN3
#  define DMACHAN_TIM3_CH4        STM32_DMA1_CHAN3
#  define DMACHAN_TIM3_UP         STM32_DMA1_CHAN3
#  define DMACHAN_TIM6_UP         STM32_DMA1_CHAN3
#  define DMACHAN_DAC1_CH1        STM32_DMA1_CHAN3
#  define DMACHAN_TIM16_CH1_1     STM32_DMA1_CHAN3
#  define DMACHAN_TIM16_UP_1      STM32_DMA1_CHAN3
#  define DMACHAN_HRTIM1_A        STM32_DMA1_CHAN3

#  define DMACHAN_ADC2_2          STM32_DMA1_CHAN4
#  define DMACHAN_SPI1_RX_2       STM32_DMA1_CHAN4
#  define DMACHAN_USART1_TX       STM32_DMA1_CHAN4
#  define DMACHAN_I2C1_TX_3       STM32_DMA1_CHAN4
#  define DMACHAN_TIM1_CH4        STM32_DMA1_CHAN4
#  define DMACHAN_TIM1_TRIG       STM32_DMA1_CHAN4
#  define DMACHAN_TIM1_COM        STM32_DMA1_CHAN4
#  define DMACHAN_TIM7_UP         STM32_DMA1_CHAN4
#  define DMACHAN_DAC1_CH2        STM32_DMA1_CHAN4
#  define DMACHAN_HRTIM1_B        STM32_DMA1_CHAN4

#  define DMACHAN_SPI1_TX_2       STM32_DMA1_CHAN5
#  define DMACHAN_USART1_RX       STM32_DMA1_CHAN5
#  define DMACHAN_I2C1_RX_3       STM32_DMA1_CHAN5
#  define DMACHAN_TIM1_UP         STM32_DMA1_CHAN5
#  define DMACHAN_TIM2_CH1        STM32_DMA1_CHAN5
#  define DMACHAN_DAC2_CH1        STM32_DMA1_CHAN5
#  define DMACHAN_TIM15_CH1       STM32_DMA1_CHAN5
#  define DMACHAN_TIM15_UP        STM32_DMA1_CHAN5
#  define DMACHAN_TIM15_TRIG      STM32_DMA1_CHAN5
#  define DMACHAN_TIM15_COM       STM32_DMA1_CHAN5
#  define DMACHAN_HRTIM1_C        STM32_DMA1_CHAN5

#  define DMACHAN_SPI1_RX_3       STM32_DMA1_CHAN6
#  define DMACHAN_USART2_RX       STM32_DMA1_CHAN6
#  define DMACHAN_I2C1_TX_1       STM32_DMA1_CHAN6
#  define DMACHAN_TIM1_CH3        STM32_DMA1_CHAN6
#  define DMACHAN_TIM3_CH1        STM32_DMA1_CHAN6
#  define DMACHAN_TIM3_TRIG       STM32_DMA1_CHAN6
#  define DMACHAN_TIM16_CH1_2     STM32_DMA1_CHAN6
#  define DMACHAN_TIM16_UP_2      STM32_DMA1_CHAN6
#  define DMACHAN_HRTIM1_D        STM32_DMA1_CHAN6

#  define DMACHAN_SPI1_TX_3       STM32_DMA1_CHAN7
#  define DMACHAN_USART2_TX       STM32_DMA1_CHAN7
#  define DMACHAN_I2C1_RX_1       STM32_DMA1_CHAN7
#  define DMACHAN_TIM2_CH2        STM32_DMA1_CHAN7
#  define DMACHAN_TIM2_CH4        STM32_DMA1_CHAN7
#  define DMACHAN_TIM17_CH1_2     STM32_DMA1_CHAN7
#  define DMACHAN_TIM17_UP_2      STM32_DMA1_CHAN7
#  define DMACHAN_HRTIM1_E        STM32_DMA1_CHAN7

#elif defined(CONFIG_STM32_STM32F37XX)

#  define DMACHAN_ADC1            STM32_DMA1_CHAN1
#  define DMACHAN_TIM2_CH3        STM32_DMA1_CHAN1
#  define DMACHAN_TIM4_CH1        STM32_DMA1_CHAN1
#  define DMACHAN_TIM17_CH1       STM32_DMA1_CHAN1
#  define DMACHAN_TIM17_UP        STM32_DMA1_CHAN1

#  define DMACHAN_SPI1_RX         STM32_DMA1_CHAN2
#  define DMACHAN_USART3_TX       STM32_DMA1_CHAN2
#  define DMACHAN_TIM2_UP         STM32_DMA1_CHAN2
#  define DMACHAN_TIM3_CH3        STM32_DMA1_CHAN2
#  define DMACHAN_TIM19_CH1       STM32_DMA1_CHAN2

#  define DMACHAN_SPI1_TX         STM32_DMA1_CHAN3
#  define DMACHAN_USART3_RX       STM32_DMA1_CHAN3
#  define DMACHAN_TIM3_CH4        STM32_DMA1_CHAN3
#  define DMACHAN_TIM3_UP         STM32_DMA1_CHAN3
#  define DMACHAN_TIM6_UP         STM32_DMA1_CHAN3
#  define DMACHAN_DAC1_CH1        STM32_DMA1_CHAN3
#  define DMACHAN_TIM16_CH1       STM32_DMA1_CHAN3
#  define DMACHAN_TIM16_UP        STM32_DMA1_CHAN3
#  define DMACHAN_TIM19_CH2       STM32_DMA1_CHAN3

#  define DMACHAN_SPI2_RX         STM32_DMA1_CHAN4
#  define DMACHAN_USART1_TX       STM32_DMA1_CHAN4
#  define DMACHAN_I2C2_TX         STM32_DMA1_CHAN4
#  define DMACHAN_TIM4_CH2        STM32_DMA1_CHAN4
#  define DMACHAN_TIM7_UP         STM32_DMA1_CHAN4
#  define DMACHAN_DAC1_CH2        STM32_DMA1_CHAN4
#  define DMACHAN_TIM19_UP        STM32_DMA1_CHAN4

#  define DMACHAN_SPI2_TX         STM32_DMA1_CHAN5
#  define DMACHAN_USART1_RX       STM32_DMA1_CHAN5
#  define DMACHAN_I2C2_RX         STM32_DMA1_CHAN5
#  define DMACHAN_TIM2_CH1        STM32_DMA1_CHAN5
#  define DMACHAN_TIM4_CH3        STM32_DMA1_CHAN5
#  define DMACHAN_TIM18_UP        STM32_DMA1_CHAN5
#  define DMACHAN_DAC2_CH1        STM32_DMA1_CHAN5
#  define DMACHAN_TIM15_CH1       STM32_DMA1_CHAN5
#  define DMACHAN_TIM15_UP        STM32_DMA1_CHAN5
#  define DMACHAN_TIM15_TRIG      STM32_DMA1_CHAN5
#  define DMACHAN_TIM15_COM       STM32_DMA1_CHAN5

#  define DMACHAN_USART2_RX       STM32_DMA1_CHAN6
#  define DMACHAN_I2C1_TX         STM32_DMA1_CHAN6
#  define DMACHAN_TIM3_CH1        STM32_DMA1_CHAN6
#  define DMACHAN_TIM3_TRIG       STM32_DMA1_CHAN6
#  define DMACHAN_TIM16_CH1_2     STM32_DMA1_CHAN6
#  define DMACHAN_TIM16_UP_2      STM32_DMA1_CHAN6

#  define DMACHAN_USART2_TX       STM32_DMA1_CHAN7
#  define DMACHAN_I2C1_RX         STM32_DMA1_CHAN6
#  define DMACHAN_TIM2_CH2        STM32_DMA1_CHAN6
#  define DMACHAN_TIM2_CH4        STM32_DMA1_CHAN6
#  define DMACHAN_TIM4_UP         STM32_DMA1_CHAN6
#  define DMACHAN_TIM17_CH1_2     STM32_DMA1_CHAN6
#  define DMACHAN_TIM17_UP_2      STM32_DMA1_CHAN6

#  define DMACHAN_SPI3_RX         STM32_DMA2_CHAN1
#  define DMACHAN_TIM5_CH4        STM32_DMA2_CHAN1
#  define DMACHAN_TIM5_TRIG       STM32_DMA2_CHAN1

#  define DMACHAN_SPI3_TX         STM32_DMA2_CHAN2
#  define DMACHAN_TIM5_CH3        STM32_DMA2_CHAN2
#  define DMACHAN_TIM5_UP         STM32_DMA2_CHAN2

#  define DMACHAN_SDADC1          STM32_DMA2_CHAN3
#  define DMACHAN_TIM6_UP_2       STM32_DMA2_CHAN3
#  define DMACHAN_DAC1_CH1_2      STM32_DMA2_CHAN3

#  define DMACHAN_SDADC2          STM32_DMA2_CHAN4
#  define DMACHAN_TIM5_CH2        STM32_DMA2_CHAN4
#  define DMACHAN_TIM7_UP_2       STM32_DMA2_CHAN4
#  define DMACHAN_DAC1_CH2_2      STM32_DMA2_CHAN4

#  define DMACHAN_SDADC3          STM32_DMA2_CHAN5
#  define DMACHAN_TIM5_CH1        STM32_DMA2_CHAN5
#  define DMACHAN_TIM18_UP_2      STM32_DMA2_CHAN5
#  define DMACHAN_DAC2_CH1_2      STM32_DMA2_CHAN5

#elif defined(CONFIG_STM32_STM32G47XX)

  /* This family uses a DMAMUX. The code to support this needs to be ported
   * to this family from STM32L4R.
   */

#else
#  error "Unknown DMA channel assignments"
#endif

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DMA_V1_DMA_H */
