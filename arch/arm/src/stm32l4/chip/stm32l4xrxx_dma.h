/************************************************************************************
 * arch/arm/src/stm32l4/chip/stm32l4xrxx_dma.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_CHIP_STM32L4XRXX_DMA_H
#define __ARCH_ARM_SRC_STM32L4_CHIP_STM32L4XRXX_DMA_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* 14 Channels Total: 7 DMA1 Channels(1-7) and 7 DMA2 channels (1-7) */

#define DMA1 0
#define DMA2 1
#define DMA3 2
#define DMA4 3
#define DMA5 4
#define DMA6 5
#define DMA7 6

/* Register Offsets *****************************************************************/

#define STM32L4_DMA_ISR_OFFSET       0x0000 /* DMA interrupt status register */
#define STM32L4_DMA_IFCR_OFFSET      0x0004 /* DMA interrupt flag clear register */

#define STM32L4_DMACHAN_OFFSET(n)    (0x0014*(n))
#define STM32L4_DMACHAN1_OFFSET      0x0000
#define STM32L4_DMACHAN2_OFFSET      0x0014
#define STM32L4_DMACHAN3_OFFSET      0x0028
#define STM32L4_DMACHAN4_OFFSET      0x003c
#define STM32L4_DMACHAN5_OFFSET      0x0050
#define STM32L4_DMACHAN6_OFFSET      0x0064
#define STM32L4_DMACHAN7_OFFSET      0x0078

#define STM32L4_DMACHAN_CCR_OFFSET   0x0008 /* DMA channel configuration register */
#define STM32L4_DMACHAN_CNDTR_OFFSET 0x000c /* DMA channel number of data register */
#define STM32L4_DMACHAN_CPAR_OFFSET  0x0010 /* DMA channel peripheral address register */
#define STM32L4_DMACHAN_CMAR_OFFSET  0x0014 /* DMA channel memory address register */

#define STM32L4_DMA_CCR_OFFSET(n)   (STM32L4_DMACHAN_CCR_OFFSET+STM32L4_DMACHAN_OFFSET(n))
#define STM32L4_DMA_CNDTR_OFFSET(n) (STM32L4_DMACHAN_CNDTR_OFFSET+STM32L4_DMACHAN_OFFSET(n))
#define STM32L4_DMA_CPAR_OFFSET(n)  (STM32L4_DMACHAN_CPAR_OFFSET+STM32L4_DMACHAN_OFFSET(n))
#define STM32L4_DMA_CMAR_OFFSET(n)  (STM32L4_DMACHAN_CMAR_OFFSET+STM32L4_DMACHAN_OFFSET(n))

#define STM32L4_DMA_CCR1_OFFSET     0x0008 /* DMA channel 1 configuration register */
#define STM32L4_DMA_CCR2_OFFSET     0x001c /* DMA channel 2 configuration register */
#define STM32L4_DMA_CCR3_OFFSET     0x0030 /* DMA channel 3 configuration register */
#define STM32L4_DMA_CCR4_OFFSET     0x0044 /* DMA channel 4 configuration register */
#define STM32L4_DMA_CCR5_OFFSET     0x0058 /* DMA channel 5 configuration register */
#define STM32L4_DMA_CCR6_OFFSET     0x006c /* DMA channel 6 configuration register */
#define STM32L4_DMA_CCR7_OFFSET     0x0080 /* DMA channel 7 configuration register */

#define STM32L4_DMA_CNDTR1_OFFSET   0x000c /* DMA channel 1 number of data register */
#define STM32L4_DMA_CNDTR2_OFFSET   0x0020 /* DMA channel 2 number of data register */
#define STM32L4_DMA_CNDTR3_OFFSET   0x0034 /* DMA channel 3 number of data register */
#define STM32L4_DMA_CNDTR4_OFFSET   0x0048 /* DMA channel 4 number of data register */
#define STM32L4_DMA_CNDTR5_OFFSET   0x005c /* DMA channel 5 number of data register */
#define STM32L4_DMA_CNDTR6_OFFSET   0x0070 /* DMA channel 6 number of data register */
#define STM32L4_DMA_CNDTR7_OFFSET   0x0084 /* DMA channel 7 number of data register */

#define STM32L4_DMA_CPAR1_OFFSET    0x0010 /* DMA channel 1 peripheral address register */
#define STM32L4_DMA_CPAR2_OFFSET    0x0024 /* DMA channel 2 peripheral address register */
#define STM32L4_DMA_CPAR3_OFFSET    0x0038 /* DMA channel 3 peripheral address register */
#define STM32L4_DMA_CPAR4_OFFSET    0x004c /* DMA channel 4 peripheral address register */
#define STM32L4_DMA_CPAR5_OFFSET    0x0060 /* DMA channel 5 peripheral address register */
#define STM32L4_DMA_CPAR6_OFFSET    0x0074 /* DMA channel 6 peripheral address register */
#define STM32L4_DMA_CPAR7_OFFSET    0x0088 /* DMA channel 7 peripheral address register */

#define STM32L4_DMA_CMAR1_OFFSET    0x0014 /* DMA channel 1 memory address register */
#define STM32L4_DMA_CMAR2_OFFSET    0x0028 /* DMA channel 2 memory address register */
#define STM32L4_DMA_CMAR3_OFFSET    0x003c /* DMA channel 3 memory address register */
#define STM32L4_DMA_CMAR4_OFFSET    0x0050 /* DMA channel 4 memory address register */
#define STM32L4_DMA_CMAR5_OFFSET    0x0064 /* DMA channel 5 memory address register */
#define STM32L4_DMA_CMAR6_OFFSET    0x0078 /* DMA channel 6 memory address register */
#define STM32L4_DMA_CMAR7_OFFSET    0x008c /* DMA channel 7 memory address register */

#define STM32L4_DMA_CSELR_OFFSET    0x00a8 /* DMA channel selection register */

/* Register Addresses ***************************************************************/

#define STM32L4_DMA1_ISRC           (STM32L4_DMA1_BASE+STM32L4_DMA_ISR_OFFSET)
#define STM32L4_DMA1_IFCR           (STM32L4_DMA1_BASE+STM32L4_DMA_IFCR_OFFSET)

#define STM32L4_DMA1_CCR(n)         (STM32L4_DMA1_BASE+STM32L4_DMA_CCR_OFFSET(n))
#define STM32L4_DMA1_CCR1           (STM32L4_DMA1_BASE+STM32L4_DMA_CCR1_OFFSET)
#define STM32L4_DMA1_CCR2           (STM32L4_DMA1_BASE+STM32L4_DMA_CCR2_OFFSET)
#define STM32L4_DMA1_CCR3           (STM32L4_DMA1_BASE+STM32L4_DMA_CCR3_OFFSET)
#define STM32L4_DMA1_CCR4           (STM32L4_DMA1_BASE+STM32L4_DMA_CCR4_OFFSET)
#define STM32L4_DMA1_CCR5           (STM32L4_DMA1_BASE+STM32L4_DMA_CCR5_OFFSET)
#define STM32L4_DMA1_CCR6           (STM32L4_DMA1_BASE+STM32L4_DMA_CCR6_OFFSET)
#define STM32L4_DMA1_CCR7           (STM32L4_DMA1_BASE+STM32L4_DMA_CCR7_OFFSET)

#define STM32L4_DMA1_CNDTR(n)       (STM32L4_DMA1_BASE+STM32L4_DMA_CNDTR_OFFSET(n))
#define STM32L4_DMA1_CNDTR1         (STM32L4_DMA1_BASE+STM32L4_DMA_CNDTR1_OFFSET)
#define STM32L4_DMA1_CNDTR2         (STM32L4_DMA1_BASE+STM32L4_DMA_CNDTR2_OFFSET)
#define STM32L4_DMA1_CNDTR3         (STM32L4_DMA1_BASE+STM32L4_DMA_CNDTR3_OFFSET)
#define STM32L4_DMA1_CNDTR4         (STM32L4_DMA1_BASE+STM32L4_DMA_CNDTR4_OFFSET)
#define STM32L4_DMA1_CNDTR5         (STM32L4_DMA1_BASE+STM32L4_DMA_CNDTR5_OFFSET)
#define STM32L4_DMA1_CNDTR6         (STM32L4_DMA1_BASE+STM32L4_DMA_CNDTR6_OFFSET)
#define STM32L4_DMA1_CNDTR7         (STM32L4_DMA1_BASE+STM32L4_DMA_CNDTR7_OFFSET)

#define STM32L4_DMA1_CPAR(n)        (STM32L4_DMA1_BASE+STM32L4_DMA_CPAR_OFFSET(n))
#define STM32L4_DMA1_CPAR1          (STM32L4_DMA1_BASE+STM32L4_DMA_CPAR1_OFFSET)
#define STM32L4_DMA1_CPAR2          (STM32L4_DMA1_BASE+STM32L4_DMA_CPAR2_OFFSET)
#define STM32L4_DMA1_CPAR3          (STM32L4_DMA1_BASE+STM32L4_DMA_CPAR3_OFFSET)
#define STM32L4_DMA1_CPAR4          (STM32L4_DMA1_BASE+STM32L4_DMA_CPAR4_OFFSET)
#define STM32L4_DMA1_CPAR5          (STM32L4_DMA1_BASE+STM32L4_DMA_CPAR5_OFFSET)
#define STM32L4_DMA1_CPAR6          (STM32L4_DMA1_BASE+STM32L4_DMA_CPAR6_OFFSET)
#define STM32L4_DMA1_CPAR7          (STM32L4_DMA1_BASE+STM32L4_DMA_CPAR7_OFFSET)

#define STM32L4_DMA1_CMAR(n)        (STM32L4_DMA1_BASE+STM32L4_DMA_CMAR_OFFSET(n))
#define STM32L4_DMA1_CMAR1          (STM32L4_DMA1_BASE+STM32L4_DMA_CMAR1_OFFSET)
#define STM32L4_DMA1_CMAR2          (STM32L4_DMA1_BASE+STM32L4_DMA_CMAR2_OFFSET)
#define STM32L4_DMA1_CMAR3          (STM32L4_DMA1_BASE+STM32L4_DMA_CMAR3_OFFSET)
#define STM32L4_DMA1_CMAR4          (STM32L4_DMA1_BASE+STM32L4_DMA_CMAR4_OFFSET)
#define STM32L4_DMA1_CMAR5          (STM32L4_DMA1_BASE+STM32L4_DMA_CMAR5_OFFSET)
#define STM32L4_DMA1_CMAR6          (STM32L4_DMA1_BASE+STM32L4_DMA_CMAR6_OFFSET)
#define STM32L4_DMA1_CMAR7          (STM32L4_DMA1_BASE+STM32L4_DMA_CMAR7_OFFSET)

#define STM32L4_DMA2_ISRC           (STM32L4_DMA2_BASE+STM32L4_DMA_ISR_OFFSET)
#define STM32L4_DMA2_IFCR           (STM32L4_DMA2_BASE+STM32L4_DMA_IFCR_OFFSET)

#define STM32L4_DMA2_CCR(n)         (STM32L4_DMA2_BASE+STM32L4_DMA_CCR_OFFSET(n))
#define STM32L4_DMA2_CCR1           (STM32L4_DMA2_BASE+STM32L4_DMA_CCR1_OFFSET)
#define STM32L4_DMA2_CCR2           (STM32L4_DMA2_BASE+STM32L4_DMA_CCR2_OFFSET)
#define STM32L4_DMA2_CCR3           (STM32L4_DMA2_BASE+STM32L4_DMA_CCR3_OFFSET)
#define STM32L4_DMA2_CCR4           (STM32L4_DMA2_BASE+STM32L4_DMA_CCR4_OFFSET)
#define STM32L4_DMA2_CCR5           (STM32L4_DMA2_BASE+STM32L4_DMA_CCR5_OFFSET)
#define STM32L4_DMA2_CCR6           (STM32L4_DMA2_BASE+STM32L4_DMA_CCR6_OFFSET)
#define STM32L4_DMA2_CCR7           (STM32L4_DMA2_BASE+STM32L4_DMA_CCR7_OFFSET)

#define STM32L4_DMA2_CNDTR(n)       (STM32L4_DMA2_BASE+STM32L4_DMA_CNDTR_OFFSET(n))
#define STM32L4_DMA2_CNDTR1         (STM32L4_DMA2_BASE+STM32L4_DMA_CNDTR1_OFFSET)
#define STM32L4_DMA2_CNDTR2         (STM32L4_DMA2_BASE+STM32L4_DMA_CNDTR2_OFFSET)
#define STM32L4_DMA2_CNDTR3         (STM32L4_DMA2_BASE+STM32L4_DMA_CNDTR3_OFFSET)
#define STM32L4_DMA2_CNDTR4         (STM32L4_DMA2_BASE+STM32L4_DMA_CNDTR4_OFFSET)
#define STM32L4_DMA2_CNDTR5         (STM32L4_DMA2_BASE+STM32L4_DMA_CNDTR5_OFFSET)
#define STM32L4_DMA2_CNDTR6         (STM32L4_DMA2_BASE+STM32L4_DMA_CNDTR6_OFFSET)
#define STM32L4_DMA2_CNDTR7         (STM32L4_DMA2_BASE+STM32L4_DMA_CNDTR7_OFFSET)

#define STM32L4_DMA2_CPAR(n)        (STM32L4_DMA2_BASE+STM32L4_DMA_CPAR_OFFSET(n))
#define STM32L4_DMA2_CPAR1          (STM32L4_DMA2_BASE+STM32L4_DMA_CPAR1_OFFSET)
#define STM32L4_DMA2_CPAR2          (STM32L4_DMA2_BASE+STM32L4_DMA_CPAR2_OFFSET)
#define STM32L4_DMA2_CPAR3          (STM32L4_DMA2_BASE+STM32L4_DMA_CPAR3_OFFSET)
#define STM32L4_DMA2_CPAR4          (STM32L4_DMA2_BASE+STM32L4_DMA_CPAR4_OFFSET)
#define STM32L4_DMA2_CPAR5          (STM32L4_DMA2_BASE+STM32L4_DMA_CPAR5_OFFSET)
#define STM32L4_DMA2_CPAR6          (STM32L4_DMA2_BASE+STM32L4_DMA_CPAR6_OFFSET)
#define STM32L4_DMA2_CPAR7          (STM32L4_DMA2_BASE+STM32L4_DMA_CPAR7_OFFSET)

#define STM32L4_DMA2_CMAR(n)        (STM32L4_DMA2_BASE+STM32L4_DMA_CMAR_OFFSET(n))
#define STM32L4_DMA2_CMAR1          (STM32L4_DMA2_BASE+STM32L4_DMA_CMAR1_OFFSET)
#define STM32L4_DMA2_CMAR2          (STM32L4_DMA2_BASE+STM32L4_DMA_CMAR2_OFFSET)
#define STM32L4_DMA2_CMAR3          (STM32L4_DMA2_BASE+STM32L4_DMA_CMAR3_OFFSET)
#define STM32L4_DMA2_CMAR4          (STM32L4_DMA2_BASE+STM32L4_DMA_CMAR4_OFFSET)
#define STM32L4_DMA2_CMAR5          (STM32L4_DMA2_BASE+STM32L4_DMA_CMAR5_OFFSET)
#define STM32L4_DMA2_CMAR6          (STM32L4_DMA2_BASE+STM32L4_DMA_CMAR6_OFFSET)
#define STM32L4_DMA2_CMAR7          (STM32L4_DMA2_BASE+STM32L4_DMA_CMAR7_OFFSET)

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
#define DMA_IFCR_ALLCHANNELS      (0x0fffffff)

#define DMA_IFCR_CGIF(n)          (DMA_CHAN_GIF_BIT << DMA_IFCR_CHAN_SHIFT(n))
#define DMA_IFCR_CTCIF(n)         (DMA_CHAN_TCIF_BIT << DMA_IFCR_CHAN_SHIFT(n))
#define DMA_IFCR_CHTIF(n)         (DMA_CHAN_HTIF_BIT << DMA_IFCR_CHAN_SHIFT(n))
#define DMA_IFCR_CTEIF(n)         (DMA_CHAN_TEIF_BIT << DMA_IFCR_CHAN_SHIFT(n))

/* DMA channel configuration register */

#define DMA_CCR_EN                (1 << 0)  /* Bit 0: Channel enable */
#define DMA_CCR_TCIE              (1 << 1)  /* Bit 1: Transfer complete interrupt enable */
#define DMA_CCR_HTIE              (1 << 2)  /* Bit 2: Half Transfer interrupt enable */
#define DMA_CCR_TEIE              (1 << 3)  /* Bit 3: Transfer error interrupt enable */
#define DMA_CCR_DIR               (1 << 4)  /* Bit 4: Data transfer direction */
#define DMA_CCR_CIRC              (1 << 5)  /* Bit 5: Circular mode */
#define DMA_CCR_PINC              (1 << 6)  /* Bit 6: Peripheral increment mode */
#define DMA_CCR_MINC              (1 << 7)  /* Bit 7: Memory increment mode */
#define DMA_CCR_PSIZE_SHIFT       (8)       /* Bits 8-9: Peripheral size */
#define DMA_CCR_PSIZE_MASK        (3 << DMA_CCR_PSIZE_SHIFT)
#  define DMA_CCR_PSIZE_8BITS     (0 << DMA_CCR_PSIZE_SHIFT) /* 00: 8-bits */
#  define DMA_CCR_PSIZE_16BITS    (1 << DMA_CCR_PSIZE_SHIFT) /* 01: 16-bits */
#  define DMA_CCR_PSIZE_32BITS    (2 << DMA_CCR_PSIZE_SHIFT) /* 10: 32-bits */
#define DMA_CCR_MSIZE_SHIFT       (10)      /* Bits 10-11: Memory size */
#define DMA_CCR_MSIZE_MASK        (3 << DMA_CCR_MSIZE_SHIFT)
#  define DMA_CCR_MSIZE_8BITS     (0 << DMA_CCR_MSIZE_SHIFT) /* 00: 8-bits */
#  define DMA_CCR_MSIZE_16BITS    (1 << DMA_CCR_MSIZE_SHIFT) /* 01: 16-bits */
#  define DMA_CCR_MSIZE_32BITS    (2 << DMA_CCR_MSIZE_SHIFT) /* 10: 32-bits */
#define DMA_CCR_PL_SHIFT          (12)      /* Bits 12-13: Channel Priority level */
#define DMA_CCR_PL_MASK           (3 << DMA_CCR_PL_SHIFT)
#  define DMA_CCR_PRILO           (0 << DMA_CCR_PL_SHIFT) /* 00: Low */
#  define DMA_CCR_PRIMED          (1 << DMA_CCR_PL_SHIFT) /* 01: Medium */
#  define DMA_CCR_PRIHI           (2 << DMA_CCR_PL_SHIFT) /* 10: High */
#  define DMA_CCR_PRIVERYHI       (3 << DMA_CCR_PL_SHIFT) /* 11: Very high */
#define DMA_CCR_MEM2MEM           (1 << 14) /* Bit 14: Memory to memory mode */

#define DMA_CCR_ALLINTS           (DMA_CCR_TEIE|DMA_CCR_HTIE|DMA_CCR_TCIE)

/* DMA channel number of data register */

#define DMA_CNDTR_NDT_SHIFT       (0)       /* Bits 15-0: Number of data to Transfer */
#define DMA_CNDTR_NDT_MASK        (0xffff << DMA_CNDTR_NDT_SHIFT)

/* DMA Channel mapping.  Each DMA channel has a mapping to one of several
 * possible sources/sinks of data.  The requests from peripherals assigned to a
 * channel are multiplexed together before entering the DMA block. This means
 * that only one request on a given channel can be enabled at once.
 *
 * Alternative DMA channel selections are provided with a numeric suffix like _1,
 * _2, etc.  Drivers, however, will use the pin selection without the numeric suffix.
 * Additional definitions are required in the board.h file.
 */

#define STM32L4_DMA1_CHAN1          (0)
#define STM32L4_DMA1_CHAN2          (1)
#define STM32L4_DMA1_CHAN3          (2)
#define STM32L4_DMA1_CHAN4          (3)
#define STM32L4_DMA1_CHAN5          (4)
#define STM32L4_DMA1_CHAN6          (5)
#define STM32L4_DMA1_CHAN7          (6)

#define STM32L4_DMA2_CHAN1          (7)
#define STM32L4_DMA2_CHAN2          (8)
#define STM32L4_DMA2_CHAN3          (9)
#define STM32L4_DMA2_CHAN4          (10)
#define STM32L4_DMA2_CHAN5          (11)
#define STM32L4_DMA2_CHAN6          (12)
#define STM32L4_DMA2_CHAN7          (13)

/* DMA Channel settings include a channel and an alternative function.
 * Channel is in bits 0..7
 * Request number is in bits 8..15
 *
 * TODO: THIS IS WRONG! NEED TO USE DMAMUX_1!
 */

#define DMACHAN_SETTING(chan, req)     ((((req) & 0xff) << 8) | ((chan) & 0xff))
#define DMACHAN_SETTING_CHANNEL_MASK   0x00FF
#define DMACHAN_SETTING_CHANNEL_SHIFT  (0)
#define DMACHAN_SETTING_FUNCTION_MASK  0xFF00
#define DMACHAN_SETTING_FUNCTION_SHIFT (8)

/* ADC */

#define DMACHAN_ADC1_1          DMACHAN_SETTING(STM32L4_DMA1_CHAN1, 0)
#define DMACHAN_ADC1_2          DMACHAN_SETTING(STM32L4_DMA2_CHAN3, 0)

#define DMACHAN_ADC2_1          DMACHAN_SETTING(STM32L4_DMA1_CHAN1, 0)
#define DMACHAN_ADC2_2          DMACHAN_SETTING(STM32L4_DMA2_CHAN4, 0)

#define DMACHAN_ADC3_1          DMACHAN_SETTING(STM32L4_DMA1_CHAN1, 0)
#define DMACHAN_ADC3_2          DMACHAN_SETTING(STM32L4_DMA2_CHAN5, 0)

/* AES */

#define DMACHAN_AES_IN_1        DMACHAN_SETTING(STM32L4_DMA2_CHAN1, 6)
#define DMACHAN_AES_IN_2        DMACHAN_SETTING(STM32L4_DMA2_CHAN5, 6)
#define DMACHAN_AES_OUT_1       DMACHAN_SETTING(STM32L4_DMA2_CHAN2, 6)
#define DMACHAN_AES_OUT_2       DMACHAN_SETTING(STM32L4_DMA2_CHAN3, 6)

/* DAC */

#define DMACHAN_DAC1_1          DMACHAN_SETTING(STM32L4_DMA1_CHAN3, 6)
#define DMACHAN_DAC1_2          DMACHAN_SETTING(STM32L4_DMA2_CHAN4, 3)

#define DMACHAN_DAC2_1          DMACHAN_SETTING(STM32L4_DMA1_CHAN4, 5)
#define DMACHAN_DAC2_2          DMACHAN_SETTING(STM32L4_DMA2_CHAN5, 3)

/* DCMI */

#define DMACHAN_DCMI_1          DMACHAN_SETTING(STM32L4_DMA2_CHAN5, 4)
#define DMACHAN_DCMI_2          DMACHAN_SETTING(STM32L4_DMA2_CHAN6, 0)

/* DFSDM */

#define DMACHAN_DFSDM0          DMACHAN_SETTING(STM32L4_DMA1_CHAN4, 0)
#define DMACHAN_DFSDM1          DMACHAN_SETTING(STM32L4_DMA1_CHAN5, 0)
#define DMACHAN_DFSDM2          DMACHAN_SETTING(STM32L4_DMA1_CHAN6, 0)
#define DMACHAN_DFSDM3          DMACHAN_SETTING(STM32L4_DMA1_CHAN7, 0)

/* HASH */

#define DMACHAN_HASH_IN         DMACHAN_SETTING(STM32L4_DMA2_CHAN7, 6)

/* I2C */

#define DMACHAN_I2C1_RX_1       DMACHAN_SETTING(STM32L4_DMA1_CHAN7, 3)
#define DMACHAN_I2C1_RX_2       DMACHAN_SETTING(STM32L4_DMA2_CHAN6, 5)
#define DMACHAN_I2C1_TX_1       DMACHAN_SETTING(STM32L4_DMA1_CHAN6, 3)
#define DMACHAN_I2C1_TX_2       DMACHAN_SETTING(STM32L4_DMA2_CHAN7, 5)

#define DMACHAN_I2C2_RX         DMACHAN_SETTING(STM32L4_DMA1_CHAN5, 3)
#define DMACHAN_I2C2_TX         DMACHAN_SETTING(STM32L4_DMA1_CHAN4, 3)

#define DMACHAN_I2C3_RX         DMACHAN_SETTING(STM32L4_DMA1_CHAN3, 2)
#define DMACHAN_I2C3_TX         DMACHAN_SETTING(STM32L4_DMA1_CHAN2, 3)

#define DMACHAN_I2C4_RX         DMACHAN_SETTING(STM32L4_DMA2_CHAN1, 0)
#define DMACHAN_I2C4_TX         DMACHAN_SETTING(STM32L4_DMA2_CHAN2, 0)

/* QUADSPI */

#define DMACHAN_QUADSPI_1       DMACHAN_SETTING(STM32L4_DMA1_CHAN5, 5)
#define DMACHAN_QUADSPI_2       DMACHAN_SETTING(STM32L4_DMA2_CHAN7, 3)

/* SAI */

#define DMACHAN_SAI1_A_1        DMACHAN_SETTING(STM32L4_DMA2_CHAN1, 1)
#define DMACHAN_SAI1_A_2        DMACHAN_SETTING(STM32L4_DMA2_CHAN6, 1)
#define DMACHAN_SAI1_B_1        DMACHAN_SETTING(STM32L4_DMA2_CHAN2, 1)
#define DMACHAN_SAI1_B_2        DMACHAN_SETTING(STM32L4_DMA2_CHAN7, 1)

#define DMACHAN_SAI2_A_1        DMACHAN_SETTING(STM32L4_DMA1_CHAN6, 1)
#define DMACHAN_SAI2_A_2        DMACHAN_SETTING(STM32L4_DMA2_CHAN3, 1)
#define DMACHAN_SAI2_B_1        DMACHAN_SETTING(STM32L4_DMA1_CHAN7, 1)
#define DMACHAN_SAI2_B_2        DMACHAN_SETTING(STM32L4_DMA2_CHAN4, 1)

/* SDMMC */

#define DMACHAN_SDMMC_1         DMACHAN_SETTING(STM32L4_DMA2_CHAN4, 7)
#define DMACHAN_SDMMC_2         DMACHAN_SETTING(STM32L4_DMA2_CHAN5, 7)

/* SPI */

#define DMACHAN_SPI1_RX_1       DMACHAN_SETTING(STM32L4_DMA1_CHAN2, 1)
#define DMACHAN_SPI1_RX_2       DMACHAN_SETTING(STM32L4_DMA2_CHAN3, 4)
#define DMACHAN_SPI1_TX_1       DMACHAN_SETTING(STM32L4_DMA1_CHAN3, 1)
#define DMACHAN_SPI1_TX_2       DMACHAN_SETTING(STM32L4_DMA2_CHAN4, 4)

#define DMACHAN_SPI2_RX         DMACHAN_SETTING(STM32L4_DMA1_CHAN4, 1)
#define DMACHAN_SPI2_TX         DMACHAN_SETTING(STM32L4_DMA1_CHAN5, 1)

#define DMACHAN_SPI3_RX         DMACHAN_SETTING(STM32L4_DMA2_CHAN1, 3)
#define DMACHAN_SPI3_TX         DMACHAN_SETTING(STM32L4_DMA2_CHAN2, 3)

/* SWPMI */

#define DMACHAN_SWPMI_RX        DMACHAN_SETTING(STM32L4_DMA2_CHAN1, 4)
#define DMACHAN_SWPMI_TX        DMACHAN_SETTING(STM32L4_DMA2_CHAN2, 4)

/* TIM */

#define DMACHAN_TIM1_CH1        DMACHAN_SETTING(STM32L4_DMA1_CHAN2, 7)
#define DMACHAN_TIM1_CH2        DMACHAN_SETTING(STM32L4_DMA1_CHAN3, 7)
#define DMACHAN_TIM1_CH3        DMACHAN_SETTING(STM32L4_DMA1_CHAN7, 7)
#define DMACHAN_TIM1_CH4        DMACHAN_SETTING(STM32L4_DMA1_CHAN4, 7)
#define DMACHAN_TIM1_COM        DMACHAN_SETTING(STM32L4_DMA1_CHAN4, 7)
#define DMACHAN_TIM1_TRIG       DMACHAN_SETTING(STM32L4_DMA1_CHAN4, 7)
#define DMACHAN_TIM1_UP         DMACHAN_SETTING(STM32L4_DMA1_CHAN6, 7)

#define DMACHAN_TIM2_CH1        DMACHAN_SETTING(STM32L4_DMA1_CHAN5, 4)
#define DMACHAN_TIM2_CH2        DMACHAN_SETTING(STM32L4_DMA1_CHAN7, 4)
#define DMACHAN_TIM2_CH3        DMACHAN_SETTING(STM32L4_DMA1_CHAN1, 4)
#define DMACHAN_TIM2_CH4        DMACHAN_SETTING(STM32L4_DMA1_CHAN7, 4)
#define DMACHAN_TIM2_UP         DMACHAN_SETTING(STM32L4_DMA1_CHAN2, 4)

#define DMACHAN_TIM3_CH1        DMACHAN_SETTING(STM32L4_DMA1_CHAN6, 5)
#define DMACHAN_TIM3_CH3        DMACHAN_SETTING(STM32L4_DMA1_CHAN2, 5)
#define DMACHAN_TIM3_CH4        DMACHAN_SETTING(STM32L4_DMA1_CHAN3, 5)
#define DMACHAN_TIM3_TRIG       DMACHAN_SETTING(STM32L4_DMA1_CHAN6, 5)
#define DMACHAN_TIM3_UP         DMACHAN_SETTING(STM32L4_DMA1_CHAN3, 5)

#define DMACHAN_TIM4_CH1        DMACHAN_SETTING(STM32L4_DMA1_CHAN1, 6)
#define DMACHAN_TIM4_CH2        DMACHAN_SETTING(STM32L4_DMA1_CHAN4, 6)
#define DMACHAN_TIM4_CH3        DMACHAN_SETTING(STM32L4_DMA1_CHAN5, 6)
#define DMACHAN_TIM4_UP         DMACHAN_SETTING(STM32L4_DMA1_CHAN7, 6)

#define DMACHAN_TIM5_CH1        DMACHAN_SETTING(STM32L4_DMA2_CHAN5, 5)
#define DMACHAN_TIM5_CH2        DMACHAN_SETTING(STM32L4_DMA2_CHAN4, 5)
#define DMACHAN_TIM5_CH3        DMACHAN_SETTING(STM32L4_DMA2_CHAN2, 5)
#define DMACHAN_TIM5_CH4        DMACHAN_SETTING(STM32L4_DMA2_CHAN1, 5)
#define DMACHAN_TIM5_COM        DMACHAN_SETTING(STM32L4_DMA2_CHAN1, 5)
#define DMACHAN_TIM5_TRIG       DMACHAN_SETTING(STM32L4_DMA2_CHAN1, 5)
#define DMACHAN_TIM5_UP         DMACHAN_SETTING(STM32L4_DMA2_CHAN2, 5)

#define DMACHAN_TIM6_UP_1       DMACHAN_SETTING(STM32L4_DMA1_CHAN3, 6)
#define DMACHAN_TIM6_UP_2       DMACHAN_SETTING(STM32L4_DMA2_CHAN4, 3)

#define DMACHAN_TIM7_UP_1       DMACHAN_SETTING(STM32L4_DMA1_CHAN4, 5)
#define DMACHAN_TIM7_UP_2       DMACHAN_SETTING(STM32L4_DMA2_CHAN5, 3)

#define DMACHAN_TIM8_CH1        DMACHAN_SETTING(STM32L4_DMA2_CHAN6, 7)
#define DMACHAN_TIM8_CH2        DMACHAN_SETTING(STM32L4_DMA2_CHAN7, 7)
#define DMACHAN_TIM8_CH3        DMACHAN_SETTING(STM32L4_DMA2_CHAN1, 7)
#define DMACHAN_TIM8_CH4        DMACHAN_SETTING(STM32L4_DMA2_CHAN2, 7)
#define DMACHAN_TIM8_COM        DMACHAN_SETTING(STM32L4_DMA2_CHAN2, 7)
#define DMACHAN_TIM8_TRIG       DMACHAN_SETTING(STM32L4_DMA2_CHAN2, 7)
#define DMACHAN_TIM8_UP         DMACHAN_SETTING(STM32L4_DMA2_CHAN1, 7)

#define DMACHAN_TIM15_CH1       DMACHAN_SETTING(STM32L4_DMA1_CHAN5, 7)
#define DMACHAN_TIM15_COM       DMACHAN_SETTING(STM32L4_DMA1_CHAN5, 7)
#define DMACHAN_TIM15_TRIG      DMACHAN_SETTING(STM32L4_DMA1_CHAN5, 7)
#define DMACHAN_TIM15_UP        DMACHAN_SETTING(STM32L4_DMA1_CHAN5, 7)

#define DMACHAN_TIM16_CH1_1     DMACHAN_SETTING(STM32L4_DMA1_CHAN3, 4)
#define DMACHAN_TIM16_CH1_2     DMACHAN_SETTING(STM32L4_DMA1_CHAN6, 4)
#define DMACHAN_TIM16_UP_1      DMACHAN_SETTING(STM32L4_DMA1_CHAN3, 4)
#define DMACHAN_TIM16_UP_2      DMACHAN_SETTING(STM32L4_DMA1_CHAN6, 4)

#define DMACHAN_TIM17_CH1_1     DMACHAN_SETTING(STM32L4_DMA1_CHAN1, 5)
#define DMACHAN_TIM17_CH1_2     DMACHAN_SETTING(STM32L4_DMA1_CHAN7, 5)
#define DMACHAN_TIM17_UP_1      DMACHAN_SETTING(STM32L4_DMA1_CHAN1, 5)
#define DMACHAN_TIM17_UP_2      DMACHAN_SETTING(STM32L4_DMA1_CHAN7, 5)

/* UART */

#define DMACHAN_USART1_RX_1     DMACHAN_SETTING(STM32L4_DMA1_CHAN5, 2)
#define DMACHAN_USART1_RX_2     DMACHAN_SETTING(STM32L4_DMA2_CHAN7, 2)
#define DMACHAN_USART1_TX_1     DMACHAN_SETTING(STM32L4_DMA1_CHAN4, 2)
#define DMACHAN_USART1_TX_2     DMACHAN_SETTING(STM32L4_DMA2_CHAN6, 2)

#define DMACHAN_USART2_RX       DMACHAN_SETTING(STM32L4_DMA1_CHAN6, 2)
#define DMACHAN_USART2_TX       DMACHAN_SETTING(STM32L4_DMA1_CHAN7, 2)

#define DMACHAN_USART3_RX       DMACHAN_SETTING(STM32L4_DMA1_CHAN3, 2)
#define DMACHAN_USART3_TX       DMACHAN_SETTING(STM32L4_DMA1_CHAN2, 2)

#define DMACHAN_UART5_RX        DMACHAN_SETTING(STM32L4_DMA2_CHAN2, 2)
#define DMACHAN_UART5_TX        DMACHAN_SETTING(STM32L4_DMA2_CHAN1, 2)

#define DMACHAN_UART4_RX        DMACHAN_SETTING(STM32L4_DMA2_CHAN5, 2)
#define DMACHAN_UART4_TX        DMACHAN_SETTING(STM32L4_DMA2_CHAN3, 2)

#define DMACHAN_LPUART_RX       DMACHAN_SETTING(STM32L4_DMA2_CHAN7, 4)
#define DMACHAN_LPUART_TX       DMACHAN_SETTING(STM32L4_DMA2_CHAN6, 4)

#endif /* __ARCH_ARM_SRC_STM32L4_CHIP_STM32L4XRXX_DMA_H */
