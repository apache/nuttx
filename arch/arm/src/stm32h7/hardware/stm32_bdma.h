/************************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_bdma.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_BDMA_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_BDMA_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_BDMA_ISR_OFFSET       0x0000 /* BDMA interrupt status register */
#define STM32_BDMA_IFCR_OFFSET      0x0004 /* BDMA interrupt flag clear register */

#define STM32_BDMACH_CCR_OFFSET     0x0008 /* BDMA channel x configuration register */
#define STM32_BDMACH_CNDTR_OFFSET   0x000C /* BDMA channel x number of data to transfer register */
#define STM32_BDMACH_CPAR_OFFSET    0x0010 /* BDMA channel x peripheral address register */
#define STM32_BDMACH_CM0AR_OFFSET   0x0014 /* BDMA channel x memory 0 address register */
#define STM32_BDMACH_CM1AR_OFFSET   0x0018 /* BDMA channel x memory 1 address register */

#define STM32_BDMA_SPACING          0x14
#define STM32_BDMA_OFFSET(x)        (STM32_BDMA_SPACING * (x))

#define STM32_BDMA_CCRX_OFFSET(x)   (STM32_BDMACH_CCR_OFFSET + \
                                    STM32_BDMA_OFFSET(x))
#define STM32_BDMA_CCR0_OFFSET      STM32_BDMA_CCRX_OFFSET(0)
#define STM32_BDMA_CCR1_OFFSET      STM32_BDMA_CCRX_OFFSET(1)
#define STM32_BDMA_CCR2_OFFSET      STM32_BDMA_CCRX_OFFSET(2)
#define STM32_BDMA_CCR3_OFFSET      STM32_BDMA_CCRX_OFFSET(3)
#define STM32_BDMA_CCR4_OFFSET      STM32_BDMA_CCRX_OFFSET(4)
#define STM32_BDMA_CCR5_OFFSET      STM32_BDMA_CCRX_OFFSET(5)
#define STM32_BDMA_CCR6_OFFSET      STM32_BDMA_CCRX_OFFSET(6)
#define STM32_BDMA_CCR7_OFFSET      STM32_BDMA_CCRX_OFFSET(7)

#define STM32_BDMA_CNDTRX_OFFSET(x) (STM32_BDMACH_CNDTR_OFFSET + \
                                    STM32_BDMA_OFFSET(x))
#define STM32_BDMA_CNDTR0_OFFSET    STM32_BDMA_CNDTRX_OFFSET(0)
#define STM32_BDMA_CNDTR1_OFFSET    STM32_BDMA_CNDTRX_OFFSET(1)
#define STM32_BDMA_CNDTR2_OFFSET    STM32_BDMA_CNDTRX_OFFSET(2)
#define STM32_BDMA_CNDTR3_OFFSET    STM32_BDMA_CNDTRX_OFFSET(3)
#define STM32_BDMA_CNDTR4_OFFSET    STM32_BDMA_CNDTRX_OFFSET(4)
#define STM32_BDMA_CNDTR5_OFFSET    STM32_BDMA_CNDTRX_OFFSET(5)
#define STM32_BDMA_CNDTR6_OFFSET    STM32_BDMA_CNDTRX_OFFSET(6)
#define STM32_BDMA_CNDTR7_OFFSET    STM32_BDMA_CNDTRX_OFFSET(7)

#define STM32_BDMA_CPARX_OFFSET(x)  (STM32_BDMACH_CPAR_OFFSET + \
                                    STM32_BDMA_OFFSET(x))
#define STM32_BDMA_CPAR0_OFFSET     STM32_BDMA_CPARX_OFFSET(0)
#define STM32_BDMA_CPAR1_OFFSET     STM32_BDMA_CPARX_OFFSET(1)
#define STM32_BDMA_CPAR2_OFFSET     STM32_BDMA_CPARX_OFFSET(2)
#define STM32_BDMA_CPAR3_OFFSET     STM32_BDMA_CPARX_OFFSET(3)
#define STM32_BDMA_CPAR4_OFFSET     STM32_BDMA_CPARX_OFFSET(4)
#define STM32_BDMA_CPAR5_OFFSET     STM32_BDMA_CPARX_OFFSET(5)
#define STM32_BDMA_CPAR6_OFFSET     STM32_BDMA_CPARX_OFFSET(6)
#define STM32_BDMA_CPAR7_OFFSET     STM32_BDMA_CPARX_OFFSET(7)

#define STM32_BDMA_CM0ARX_OFFSET(x) (STM32_BDMACH_CM0AR_OFFSET + \
                                    STM32_BDMA_OFFSET(x))
#define STM32_BDMA_CM0AR0_OFFSET    STM32_BDMA_CM0ARX_OFFSET(0)
#define STM32_BDMA_CM0AR1_OFFSET    STM32_BDMA_CM0ARX_OFFSET(1)
#define STM32_BDMA_CM0AR2_OFFSET    STM32_BDMA_CM0ARX_OFFSET(2)
#define STM32_BDMA_CM0AR3_OFFSET    STM32_BDMA_CM0ARX_OFFSET(3)
#define STM32_BDMA_CM0AR4_OFFSET    STM32_BDMA_CM0ARX_OFFSET(4)
#define STM32_BDMA_CM0AR5_OFFSET    STM32_BDMA_CM0ARX_OFFSET(5)
#define STM32_BDMA_CM0AR6_OFFSET    STM32_BDMA_CM0ARX_OFFSET(6)
#define STM32_BDMA_CM0AR7_OFFSET    STM32_BDMA_CM0ARX_OFFSET(7)

#define STM32_BDMA_CM1ARX_OFFSET(x) (STM32_BDMACH_CM1AR_OFFSET + \
                                    STM32_BDMA_OFFSET(x))
#define STM32_BDMA_CM1AR0_OFFSET    STM32_BDMA_CM1ARX_OFFSET(0)
#define STM32_BDMA_CM1AR1_OFFSET    STM32_BDMA_CM1ARX_OFFSET(1)
#define STM32_BDMA_CM1AR2_OFFSET    STM32_BDMA_CM1ARX_OFFSET(2)
#define STM32_BDMA_CM1AR3_OFFSET    STM32_BDMA_CM1ARX_OFFSET(3)
#define STM32_BDMA_CM1AR4_OFFSET    STM32_BDMA_CM1ARX_OFFSET(4)
#define STM32_BDMA_CM1AR5_OFFSET    STM32_BDMA_CM1ARX_OFFSET(5)
#define STM32_BDMA_CM1AR6_OFFSET    STM32_BDMA_CM1ARX_OFFSET(6)
#define STM32_BDMA_CM1AR7_OFFSET    STM32_BDMA_CM1ARX_OFFSET(7)

/* Register Addresses ***************************************************************/

#define STM32_BDMA_ISR              (STM32_BDMA_BASE+STM32_BDMA_ISR_OFFSET)
#define STM32_BDMA_IFCR             (STM32_BDMA_BASE+STM32_BDMA_IFCR_OFFSET)

#define STM32_BDMA_CCRX(x)          (STM32_BDMA_BASE+STM32_BDMA_CCRX_OFFSET(x))
#define STM32_BDMA_CCR0             (STM32_BDMA_BASE+STM32_BDMA_CCR0_OFFSET)
#define STM32_BDMA_CCR1             (STM32_BDMA_BASE+STM32_BDMA_CCR1_OFFSET)
#define STM32_BDMA_CCR2             (STM32_BDMA_BASE+STM32_BDMA_CCR2_OFFSET)
#define STM32_BDMA_CCR3             (STM32_BDMA_BASE+STM32_BDMA_CCR3_OFFSET)
#define STM32_BDMA_CCR4             (STM32_BDMA_BASE+STM32_BDMA_CCR4_OFFSET)
#define STM32_BDMA_CCR5             (STM32_BDMA_BASE+STM32_BDMA_CCR5_OFFSET)
#define STM32_BDMA_CCR6             (STM32_BDMA_BASE+STM32_BDMA_CCR6_OFFSET)
#define STM32_BDMA_CCR7             (STM32_BDMA_BASE+STM32_BDMA_CCR7_OFFSET)

#define STM32_BDMA_CNDTRX(x)        (STM32_BDMA_BASE+STM32_BDMA_CNDTRX_OFFSET(x))
#define STM32_BDMA_CNDTR0           (STM32_BDMA_BASE+STM32_BDMA_CNDTR0_OFFSET)
#define STM32_BDMA_CNDTR1           (STM32_BDMA_BASE+STM32_BDMA_CNDTR1_OFFSET)
#define STM32_BDMA_CNDTR2           (STM32_BDMA_BASE+STM32_BDMA_CNDTR2_OFFSET)
#define STM32_BDMA_CNDTR3           (STM32_BDMA_BASE+STM32_BDMA_CNDTR3_OFFSET)
#define STM32_BDMA_CNDTR4           (STM32_BDMA_BASE+STM32_BDMA_CNDTR4_OFFSET)
#define STM32_BDMA_CNDTR5           (STM32_BDMA_BASE+STM32_BDMA_CNDTR5_OFFSET)
#define STM32_BDMA_CNDTR6           (STM32_BDMA_BASE+STM32_BDMA_CNDTR6_OFFSET)
#define STM32_BDMA_CNDTR7           (STM32_BDMA_BASE+STM32_BDMA_CNDTR7_OFFSET)

#define STM32_BDMA_CPARX(x)         (STM32_BDMA_BASE+STM32_BDMA_CPARX_OFFSET(x))
#define STM32_BDMA_CPAR0            (STM32_BDMA_BASE+STM32_BDMA_CPAR0_OFFSET)
#define STM32_BDMA_CPAR1            (STM32_BDMA_BASE+STM32_BDMA_CPAR1_OFFSET)
#define STM32_BDMA_CPAR2            (STM32_BDMA_BASE+STM32_BDMA_CPAR2_OFFSET)
#define STM32_BDMA_CPAR3            (STM32_BDMA_BASE+STM32_BDMA_CPAR3_OFFSET)
#define STM32_BDMA_CPAR4            (STM32_BDMA_BASE+STM32_BDMA_CPAR4_OFFSET)
#define STM32_BDMA_CPAR5            (STM32_BDMA_BASE+STM32_BDMA_CPAR5_OFFSET)
#define STM32_BDMA_CPAR6            (STM32_BDMA_BASE+STM32_BDMA_CPAR6_OFFSET)
#define STM32_BDMA_CPAR7            (STM32_BDMA_BASE+STM32_BDMA_CPAR7_OFFSET)

#define STM32_BDMA_CM0ARX(x)        (STM32_BDMA_BASE+STM32_BDMA_CM0ARX_OFFSET(x))
#define STM32_BDMA_CM0AR0           (STM32_BDMA_BASE+STM32_BDMA_CM0AR0_OFFSET)
#define STM32_BDMA_CM0AR1           (STM32_BDMA_BASE+STM32_BDMA_CM0AR1_OFFSET)
#define STM32_BDMA_CM0AR2           (STM32_BDMA_BASE+STM32_BDMA_CM0AR2_OFFSET)
#define STM32_BDMA_CM0AR3           (STM32_BDMA_BASE+STM32_BDMA_CM0AR3_OFFSET)
#define STM32_BDMA_CM0AR4           (STM32_BDMA_BASE+STM32_BDMA_CM0AR4_OFFSET)
#define STM32_BDMA_CM0AR5           (STM32_BDMA_BASE+STM32_BDMA_CM0AR5_OFFSET)
#define STM32_BDMA_CM0AR6           (STM32_BDMA_BASE+STM32_BDMA_CM0AR6_OFFSET)
#define STM32_BDMA_CM0AR7           (STM32_BDMA_BASE+STM32_BDMA_CM0AR7_OFFSET)

#define STM32_BDMA_CM1ARX(x)        (STM32_BDMA_BASE+STM32_BDMA_CM1ARX_OFFSET(x))
#define STM32_BDMA_CM1AR0           (STM32_BDMA_BASE+STM32_BDMA_CM1AR0_OFFSET)
#define STM32_BDMA_CM1AR1           (STM32_BDMA_BASE+STM32_BDMA_CM1AR1_OFFSET)
#define STM32_BDMA_CM1AR2           (STM32_BDMA_BASE+STM32_BDMA_CM1AR2_OFFSET)
#define STM32_BDMA_CM1AR3           (STM32_BDMA_BASE+STM32_BDMA_CM1AR3_OFFSET)
#define STM32_BDMA_CM1AR4           (STM32_BDMA_BASE+STM32_BDMA_CM1AR4_OFFSET)
#define STM32_BDMA_CM1AR5           (STM32_BDMA_BASE+STM32_BDMA_CM1AR5_OFFSET)
#define STM32_BDMA_CM1AR6           (STM32_BDMA_BASE+STM32_BDMA_CM1AR6_OFFSET)
#define STM32_BDMA_CM1AR7           (STM32_BDMA_BASE+STM32_BDMA_CM1AR7_OFFSET)

/* Register Bitfield Definitions ****************************************************/

#define BDMA_CHAN_SHIFT(n)         ((n) << 2)
#define BDMA_CHAN_MASK             0xf
#define BDMA_CHAN_CGIF             (1 << 0) /* Bit 0: Global interrupt flag */
#define BDMA_CHAN_TCIF             (1 << 1) /* Bit 1: Transfer complete flag */
#define BDMA_CHAN_HTIF             (1 << 2) /* Bit 2: half transfer complete flag */
#define BDMA_CHAN_TEIF             (1 << 3) /* Bit 3: Transfer error flag */
#define BDMA_CCR_ALLINTS           (BDMA_CHAN_TCIF | BDMA_CHAN_HTIF | BDMA_CHAN_TEIF)

/* BDMA interrupt status register */

#define BDMA_ISR_CHAN_SHIFT(n)     BDMA_CHAN_SHIFT(n)
#define BDMA_ISR_CHAN_MASK(n)      (BDMA_CHAN_MASK <<  BDMA_ISR_CHAN_SHIFT(n))
#define BDMA_ISR_CHAN0_SHIFT       (0)       /* Bits 3-0:  BDMA Channel 0 interrupt status */
#define BDMA_ISR_CHAN0_MASK        (BDMA_CHAN_MASK <<  BDMA_ISR_CHAN1_SHIFT)
#define BDMA_ISR_CHAN1_SHIFT       (4)       /* Bits 7-4:  BDMA Channel 1 interrupt status */
#define BDMA_ISR_CHAN1_MASK        (BDMA_CHAN_MASK <<  BDMA_ISR_CHAN2_SHIFT)
#define BDMA_ISR_CHAN2_SHIFT       (8)       /* Bits 11-8:  BDMA Channel 2 interrupt status */
#define BDMA_ISR_CHAN2_MASK        (BDMA_CHAN_MASK <<  BDMA_ISR_CHAN3_SHIFT)
#define BDMA_ISR_CHAN3_SHIFT       (12)      /* Bits 15-12:  BDMA Channel 3 interrupt status */
#define BDMA_ISR_CHAN3_MASK        (BDMA_CHAN_MASK <<  BDMA_ISR_CHAN4_SHIFT)
#define BDMA_ISR_CHAN4_SHIFT       (16)      /* Bits 19-16:  BDMA Channel 4 interrupt status */
#define BDMA_ISR_CHAN4_MASK        (BDMA_CHAN_MASK <<  BDMA_ISR_CHAN5_SHIFT)
#define BDMA_ISR_CHAN5_SHIFT       (20)      /* Bits 23-20:  BDMA Channel 5 interrupt status */
#define BDMA_ISR_CHAN5_MASK        (BDMA_CHAN_MASK <<  BDMA_ISR_CHAN6_SHIFT)
#define BDMA_ISR_CHAN6_SHIFT       (24)      /* Bits 27-24:  BDMA Channel 6 interrupt status */
#define BDMA_ISR_CHAN6_MASK        (BDMA_CHAN_MASK <<  BDMA_ISR_CHAN7_SHIFT)
#define BDMA_ISR_CHAN7_SHIFT       (28)      /* Bits 31-28:  BDMA Channel 7 interrupt status */
#define BDMA_ISR_CHAN7_MASK        (BDMA_CHAN_MASK <<  BDMA_ISR_CHAN7_SHIFT)

#define BDMA_ISR_CGIF(n)           (BDMA_CHAN_CGIF_BIT << BDMA_ISR_CHAN_SHIFT(n))
#define BDMA_ISR_TCIF(n)           (BDMA_CHAN_TCIF_BIT << BDMA_ISR_CHAN_SHIFT(n))
#define BDMA_ISR_HTIF(n)           (BDMA_CHAN_HTIF_BIT << BDMA_ISR_CHAN_SHIFT(n))
#define BDMA_ISR_TEIF(n)           (BDMA_CHAN_TEIF_BIT << BDMA_ISR_CHAN_SHIFT(n))

/* BDMA interrupt flag clear register */

#define BDMA_IFCR_CHAN_SHIFT(n)     BDMA_CHAN_SHIFT(n)
#define BDMA_IFCR_CHAN_MASK(n)      (BDMA_CHAN_MASK <<  BDMA_IFCR_CHAN_SHIFT(n))
#define BDMA_IFCR_CHAN0_SHIFT       (0)       /* Bits 3-0:  BDMA Channel 0 interrupt status */
#define BDMA_IFCR_CHAN0_MASK        (BDMA_CHAN_MASK <<  BDMA_IFCR_CHAN1_SHIFT)
#define BDMA_IFCR_CHAN1_SHIFT       (4)       /* Bits 7-4:  BDMA Channel 1 interrupt status */
#define BDMA_IFCR_CHAN1_MASK        (BDMA_CHAN_MASK <<  BDMA_IFCR_CHAN2_SHIFT)
#define BDMA_IFCR_CHAN2_SHIFT       (8)       /* Bits 11-8:  BDMA Channel 2 interrupt status */
#define BDMA_IFCR_CHAN2_MASK        (BDMA_CHAN_MASK <<  BDMA_IFCR_CHAN3_SHIFT)
#define BDMA_IFCR_CHAN3_SHIFT       (12)      /* Bits 15-12:  BDMA Channel 3 interrupt status */
#define BDMA_IFCR_CHAN3_MASK        (BDMA_CHAN_MASK <<  BDMA_IFCR_CHAN4_SHIFT)
#define BDMA_IFCR_CHAN4_SHIFT       (16)      /* Bits 19-16:  BDMA Channel 4 interrupt status */
#define BDMA_IFCR_CHAN4_MASK        (BDMA_CHAN_MASK <<  BDMA_IFCR_CHAN5_SHIFT)
#define BDMA_IFCR_CHAN5_SHIFT       (20)      /* Bits 23-20:  BDMA Channel 5 interrupt status */
#define BDMA_IFCR_CHAN5_MASK        (BDMA_CHAN_MASK <<  BDMA_IFCR_CHAN6_SHIFT)
#define BDMA_IFCR_CHAN6_SHIFT       (24)      /* Bits 27-24:  BDMA Channel 6 interrupt status */
#define BDMA_IFCR_CHAN6_MASK        (BDMA_CHAN_MASK <<  BDMA_IFCR_CHAN7_SHIFT)
#define BDMA_IFCR_CHAN7_SHIFT       (28)      /* Bits 31-28:  BDMA Channel 7 interrupt status */
#define BDMA_IFCR_CHAN7_MASK        (BDMA_CHAN_MASK <<  BDMA_IFCR_CHAN7_SHIFT)

#define BDMA_IFCR_CGIF(n)           (BDMA_CHAN_CGIF_BIT << BDMA_IFCR_CHAN_SHIFT(n))
#define BDMA_IFCR_TCIF(n)           (BDMA_CHAN_TCIF_BIT << BDMA_IFCR_CHAN_SHIFT(n))
#define BDMA_IFCR_HTIF(n)           (BDMA_CHAN_HTIF_BIT << BDMA_IFCR_CHAN_SHIFT(n))
#define BDMA_IFCR_TEIF(n)           (BDMA_CHAN_TEIF_BIT << BDMA_IFCR_CHAN_SHIFT(n))

/* BDMA channel x configuration register */

#define BDMA_CCR_EN                (1 << 0)  /* Bit 0: Channel enable */
#define BDMA_CCR_TCIE              (1 << 1)  /* Bit 1: Transfer complete interrupt enable */
#define BDMA_CCR_HTIE              (1 << 2)  /* Bit 2: Half Transfer interrupt enable */
#define BDMA_CCR_TEIE              (1 << 3)  /* Bit 3: Transfer error interrupt enable */
#define BDMA_CCR_DIR               (1 << 4)  /* Bit 4: Data transfer direction */
#define BDMA_CCR_CIRC              (1 << 5)  /* Bit 5: Circular mode */
#define BDMA_CCR_PINC              (1 << 6)  /* Bit 6: Peripheral increment mode */
#define BDMA_CCR_MINC              (1 << 7)  /* Bit 7: Memory increment */
#define BDMA_CCR_PSIZE_SHIFT       (8)       /* Bits 8-9: Peripheral size */
#define BDMA_CCR_PSIZE_MASK        (3 << BDMA_CCR_PSIZE_SHIFT)
#  define BDMA_CCR_PSIZE_8BITS     (0 << BDMA_CCR_PSIZE_SHIFT) /* 00: 8-bits */
#  define BDMA_CCR_PSIZE_16BITS    (1 << BDMA_CCR_PSIZE_SHIFT) /* 01: 16-bits */
#  define BDMA_CCR_PSIZE_32BITS    (2 << BDMA_CCR_PSIZE_SHIFT) /* 10: 32-bits */
#define BDMA_CCR_MSIZE_SHIFT       (10)                        /* Bits 10-11: Memory size*/
#define BDMA_CCR_MSIZE_MASK        (3 << BDMA_CCR_MSIZE_SHIFT)
#  define BDMA_CCR_MSIZE_8BITS     (0 << BDMA_CCR_MSIZE_SHIFT) /* 00: 8-bits */
#  define BDMA_CCR_MSIZE_16BITS    (1 << BDMA_CCR_MSIZE_SHIFT) /* 01: 16-bits */
#  define BDMA_CCR_MSIZE_32BITS    (2 << BDMA_CCR_MSIZE_SHIFT) /* 10: 32-bits */
#define BDMA_CCR_PL_SHIFT          (12)                        /* Bits 12-13: Priority level */
#define BDMA_CCR_PL_MASK           (3 << BDMA_CCR_PL_SHIFT)
#  define BDMA_CCR_PRILO           (0 << BDMA_CCR_PL_SHIFT) /* 00: Low */
#  define BDMA_CCR_PRIMED          (1 << BDMA_CCR_PL_SHIFT) /* 01: Medium */
#  define BDMA_CCR_PRIHI           (2 << BDMA_CCR_PL_SHIFT) /* 10: High */
#  define BDMA_CCR_PRIVERYHI       (3 << BDMA_CCR_PL_SHIFT) /* 11: Very high */
#define BDMA_CCR_M2M               (1 << 14)                /* Bit 14: Memory-to-memory mode */
#define BDMA_CCR_DBM               (1 << 15)                /* Bit 15: dobule buffer mode*/
#define BDMA_CCR_CT                (1 << 16)                /* Bit 16: Current target */

/* BDMA channel x number of data to transfer register */

#define BDMA_CNDTR_NDT_SHIFT       (0)       /* Bits 15-0: Number of data to Transfer */
#define BDMA_CNDTR_NDT_MASK        (0xffff << BDMA_CNDTR_NDT_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_BDMA_H */
