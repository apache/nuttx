/****************************************************************************
 * arch/arm/src/stm32l4/hardware/stm32l4_sai.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_SAI_H
#define __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_SAI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32L4_SAI_GCR_OFFSET     0x0000  /* SAI Global Configuration Register */

#define STM32L4_SAI_A_OFFSET       0x0004
#define STM32L4_SAI_B_OFFSET       0x0024

#define STM32L4_SAI_CR1_OFFSET     0x0000  /* SAI Configuration Register 1 A */
#define STM32L4_SAI_CR2_OFFSET     0x0004  /* SAI Configuration Register 2 A */
#define STM32L4_SAI_FRCR_OFFSET    0x0008  /* SAI Frame Configuration Register A */
#define STM32L4_SAI_SLOTR_OFFSET   0x000c  /* SAI Slot Register A */
#define STM32L4_SAI_IM_OFFSET      0x0010  /* SAI Interrupt Mask Register 2 A */
#define STM32L4_SAI_SR_OFFSET      0x0014  /* SAI Status Register A */
#define STM32L4_SAI_CLRFR_OFFSET   0x0018  /* SAI Clear Flag Register A */
#define STM32L4_SAI_DR_OFFSET      0x001c  /* SAI Data Register A */

/* Register Addresses *******************************************************/

#define STM32L4_SAI1_GCR           (STM32L4_SAI_GCR_OFFSET)

#define STM32L4_SAI1_A_BASE        (STM32L4_SAI1_BASE+STM32L4_SAI_A_OFFSET)
#define STM32L4_SAI1_B_BASE        (STM32L4_SAI1_BASE+STM32L4_SAI_B_OFFSET)

#define STM32L4_SAI1_ACR1          (STM32L4_SAI1_A_BASE+STM32L4_SAI_ACR1_OFFSET)
#define STM32L4_SAI1_ACR2          (STM32L4_SAI1_A_BASE+STM32L4_SAI_ACR2_OFFSET)
#define STM32L4_SAI1_AFRCR         (STM32L4_SAI1_A_BASE+STM32L4_SAI_AFRCR_OFFSET)
#define STM32L4_SAI1_ASLOTR        (STM32L4_SAI1_A_BASE+STM32L4_SAI_ASLOTR_OFFSET)
#define STM32L4_SAI1_AIM           (STM32L4_SAI1_A_BASE+STM32L4_SAI_AIM_OFFSET)
#define STM32L4_SAI1_ASR           (STM32L4_SAI1_A_BASE+STM32L4_SAI_ASR_OFFSET)
#define STM32L4_SAI1_ACLRFR        (STM32L4_SAI1_A_BASE+STM32L4_SAI_ACLRFR_OFFSET)
#define STM32L4_SAI1_ADR           (STM32L4_SAI1_A_BASE+STM32L4_SAI_ADR_OFFSET)

#define STM32L4_SAI1_BCR1          (STM32L4_SAI1_B_BASE+STM32L4_SAI_BCR1_OFFSET)
#define STM32L4_SAI1_BCR2          (STM32L4_SAI1_B_BASE+STM32L4_SAI_BCR2_OFFSET)
#define STM32L4_SAI1_BFRCR         (STM32L4_SAI1_B_BASE+STM32L4_SAI_BFRCR_OFFSET)
#define STM32L4_SAI1_BSLOTR        (STM32L4_SAI1_B_BASE+STM32L4_SAI_BSLOTR_OFFSET)
#define STM32L4_SAI1_BIM           (STM32L4_SAI1_B_BASE+STM32L4_SAI_BIM_OFFSET)
#define STM32L4_SAI1_BSR           (STM32L4_SAI1_B_BASE+STM32L4_SAI_BSR_OFFSET)
#define STM32L4_SAI1_BCLRFR        (STM32L4_SAI1_B_BASE+STM32L4_SAI_BCLRFR_OFFSET)
#define STM32L4_SAI1_BDR           (STM32L4_SAI1_B_BASE+STM32L4_SAI_BDR_OFFSET)

#define STM32L4_SAI2_GCR           (STM32L4_SAI2_BASE+STM32L4_SAI_GCR_OFFSET)

#define STM32L4_SAI2_A_BASE        (STM32L4_SAI2_BASE+STM32L4_SAI_A_OFFSET)
#define STM32L4_SAI2_B_BASE        (STM32L4_SAI2_BASE+STM32L4_SAI_B_OFFSET)

#define STM32L4_SAI2_ACR1          (STM32L4_SAI2_A_BASE+STM32L4_SAI_ACR1_OFFSET)
#define STM32L4_SAI2_ACR2          (STM32L4_SAI2_A_BASE+STM32L4_SAI_ACR2_OFFSET)
#define STM32L4_SAI2_AFRCR         (STM32L4_SAI2_A_BASE+STM32L4_SAI_AFRCR_OFFSET)
#define STM32L4_SAI2_ASLOTR        (STM32L4_SAI2_A_BASE+STM32L4_SAI_ASLOTR_OFFSET)
#define STM32L4_SAI2_AIM           (STM32L4_SAI2_A_BASE+STM32L4_SAI_AIM_OFFSET)
#define STM32L4_SAI2_ASR           (STM32L4_SAI2_A_BASE+STM32L4_SAI_ASR_OFFSET)
#define STM32L4_SAI2_ACLRFR        (STM32L4_SAI2_A_BASE+STM32L4_SAI_ACLRFR_OFFSET)
#define STM32L4_SAI2_ADR           (STM32L4_SAI2_A_BASE+STM32L4_SAI_ADR_OFFSET)

#define STM32L4_SAI2_BCR1          (STM32L4_SAI2_B_BASE+STM32L4_SAI_BCR1_OFFSET)
#define STM32L4_SAI2_BCR2          (STM32L4_SAI2_B_BASE+STM32L4_SAI_BCR2_OFFSET)
#define STM32L4_SAI2_BFRCR         (STM32L4_SAI2_B_BASE+STM32L4_SAI_BFRCR_OFFSET)
#define STM32L4_SAI2_BSLOTR        (STM32L4_SAI2_B_BASE+STM32L4_SAI_BSLOTR_OFFSET)
#define STM32L4_SAI2_BIM           (STM32L4_SAI2_B_BASE+STM32L4_SAI_BIM_OFFSET)
#define STM32L4_SAI2_BSR           (STM32L4_SAI2_B_BASE+STM32L4_SAI_BSR_OFFSET)
#define STM32L4_SAI2_BCLRFR        (STM32L4_SAI2_B_BASE+STM32L4_SAI_BCLRFR_OFFSET)
#define STM32L4_SAI2_BDR           (STM32L4_SAI2_B_BASE+STM32L4_SAI_BDR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SAI Global Configuration Register */

#define SAI_GCR_SYNCIN_SHIFT       (0)       /* Bits 0-1: Synchronization inputs */
#define SAI_GCR_SYNCIN_MASK        (3 << SAI_GCR_SYNCIN_SHIFT)
#  define SAI_GCR_SYNCIN(n)        ((uint32_t)(n) << SAI_GCR_SYNCIN_SHIFT)
                                             /* Bits 2-3: Reserved */
#define SAI_GCR_SYNCOUT_SHIFT      (4)       /* Bits 4-5: Synchronization outputs */
#define SAI_GCR_SYNCOUT_MASK       (3 << SAI_GCR_SYNCOUT_SHIFT)
#  define SAI_GCR_SYNCOUT          ((uint32_t)(n) << SAI_GCR_SYNCOUT_SHIFT)
                                             /* Bits 6-31: Reserved */

/* SAI Configuration Register 1 */

#define SAI_CR1_MODE_SHIFT         (0)       /* Bits 0-1: SAI audio block mode */
#define SAI_CR1_MODE_MASK          (3 << SAI_CR1_MODE_SHIFT)
#  define SAI_CR1_MODE_MASTER_TX   (0 << SAI_CR1_MODE_SHIFT) /* Master transmitter */
#  define SAI_CR1_MODE_MASTER_RX   (1 << SAI_CR1_MODE_SHIFT) /* Master receiver */
#  define SAI_CR1_MODE_SLAVE_TX    (2 << SAI_CR1_MODE_SHIFT) /* Slave transmitter */
#  define SAI_CR1_MODE_SLAVE_RX    (3 << SAI_CR1_MODE_SHIFT) /* Slave receiver */

#define SAI_CR1_PRTCFG_SHIFT       (2)       /* Bits 2-3: Protocol configuration */
#define SAI_CR1_PRTCFG_MASK        (3 << SAI_CR1_PRTCFG_SHIFT)
#  define SAI_CR1_PRTCFG_FREE      (0 << SAI_CR1_PRTCFG_SHIFT) /* Free protocol */
#  define SAI_CR1_PRTCFG_SPDIF     (1 << SAI_CR1_PRTCFG_SHIFT) /* SPDIF protocol */
#  define SAI_CR1_PRTCFG_AC97      (2 << SAI_CR1_PRTCFG_SHIFT) /* AC97 protocol */

                                             /* Bit 4: Reserved */
#define SAI_CR1_DS_SHIFT           (5)       /* Bits 5-7: Data size */
#define SAI_CR1_DS_MASK            (7 << SAI_CR1_DS_SHIFT)
#  define SAI_CR1_DS_8BITS         (2 << SAI_CR1_DS_SHIFT) /* 8 bits */
#  define SAI_CR1_DS_10BITS        (3 << SAI_CR1_DS_SHIFT) /* 10 bits */
#  define SAI_CR1_DS_16BITS        (4 << SAI_CR1_DS_SHIFT) /* 16 bits */
#  define SAI_CR1_DS_20BITS        (5 << SAI_CR1_DS_SHIFT) /* 20 bits */
#  define SAI_CR1_DS_24BITS        (6 << SAI_CR1_DS_SHIFT) /* 24 bits */
#  define SAI_CR1_DS_32BITS        (7 << SAI_CR1_DS_SHIFT) /* 32 bits */

#define SAI_CR1_LSBFIRST           (1 << 8)  /* Bit 8:  Least significant bit first */
#define SAI_CR1_CKSTR              (1 << 9)  /* Bit 9:  Clock strobing edge */
#define SAI_CR1_SYNCEN_SHIFT       (10)      /* Bits 10-11: Synchronization enable */
#define SAI_CR1_SYNCEN_MASK        (3 << SAI_CR1_SYNCEN_SHIFT)
#  define SAI_CR1_SYNCEN_ASYNCH    (0 << SAI_CR1_SYNCEN_SHIFT) /* Asynchronous mode */
#  define SAI_CR1_SYNCEN_INTERNAL  (1 << SAI_CR1_SYNCEN_SHIFT) /* Synchronous with other internal sub-block */
#  define SAI_CR1_SYNCEN_EXTERNAL  (2 << SAI_CR1_SYNCEN_SHIFT) /* Aynchronous with external SAI peripheral */

#define SAI_CR1_MONO               (1 << 12) /* Bit 12: Mono mode */
#define SAI_CR1_OUTDRIV            (1 << 13) /* Bit 13: Output drive */
                                             /* Bits 14-15: Reserved */
#define SAI_CR1_SAIEN              (1 << 16) /* Bit 16: Audio block enable */
#define SAI_CR1_DMAEN              (1 << 17) /* Bit 17: DMA enable */
                                             /* Bit 18: Reserved */
#define SAI_CR1_NODIV              (1 << 19) /* Bit 19: No divider */
#define SAI_CR1_MCKDIV_SHIFT       (20)      /* Bits 20-23: Master clock divider */
#define SAI_CR1_MCKDIV_MASK        (15 << SAI_CR1_MCKDIV_SHIFT)
#  define SAI_CR1_MCKDIV(n)        ((uint32_t)(n) << SAI_CR1_MCKDIV_SHIFT)
                                             /* Bits 24-31: Reserved */

/* SAI Configuration Register 2 */

#define SAI_CR2_FTH_SHIFT          (0)       /* Bits 0-2: FIFO threshold */
#define SAI_CR2_FTH_MASK           (7 << SAI_CR2_FTH_SHIFT)
#  define SAI_CR2_FTH_EMPTY        (0 << SAI_CR2_FTH_SHIFT) /* FIFO empty */
#  define SAI_CR2_FTH_1QF          (1 << SAI_CR2_FTH_SHIFT) /* 1/4 FIFO */
#  define SAI_CR2_FTH_HF           (2 << SAI_CR2_FTH_SHIFT) /* 1/2 FIFO */
#  define SAI_CR2_FTH_3QF          (3 << SAI_CR2_FTH_SHIFT) /* 3/4 FIFO */
#  define SAI_CR2_FTH_FULL         (4 << SAI_CR2_FTH_SHIFT) /* FIFO full */

#define SAI_CR2_FFLUSH             (1 << 3)  /* Bit 3:  FIFO flush */
#define SAI_CR2_TRIS               (1 << 4)  /* Bit 4:  Tristate management on data line */
#define SAI_CR2_MUTE               (1 << 5)  /* Bit 5:  Mute */
#define SAI_CR2_MUTEVAL            (1 << 6)  /* Bit 6:  Mute value */
#define SAI_CR2_MUTECNT_SHIFT      (7)       /* Bits 7-12: Mute counter */
#define SAI_CR2_MUTECNT_MASK       (0x3f << SAI_CR2_MUTECNT_SHIFT)
#  define SAI_CR2_MUTECNT(n)       ((uint32_t)(n) << SAI_CR2_MUTECNT_SHIFT)
#define SAI_CR2_CPL                (1 << 13) /* Bit 13: Complement */
#define SAI_CR2_COMP_SHIFT         (14)      /* Bits 14-15: Companding mode */
#define SAI_CR2_COMP_MASK          (3 << SAI_CR2_COMP_SHIFT)
#  define SAI_CR2_COMP_NONE        (0 << SAI_CR2_COMP_SHIFT) /* No companding algorithm */
#  define SAI_CR2_COMP_ULAW        (2 << SAI_CR2_COMP_SHIFT) /* μ-Law algorithm */
#  define SAI_CR2_COMP_ALAW        (3 << SAI_CR2_COMP_SHIFT) /* A-Law algorithm */

                                             /* Bits 16-31: Reserved */

/* SAI Frame Configuration Register */

#define SAI_FRCR_FRL_SHIFT         (0)       /* Bits 0-7: Frame length */
#define SAI_FRCR_FRL_MASK          (0xff << SAI_FRCR_FRL_SHIFT)
#  define SAI_FRCR_FRL(n)          ((uint32_t)((n) - 1) << SAI_FRCR_FRL_SHIFT)
#define SAI_FRCR_FSALL_SHIFT       (8)       /* Bits 8-14: Frame synchronization active level length */
#define SAI_FRCR_FSALL_MASK        (0x7f << SAI_FRCR_FSALL_SHIFT)
#  define SAI_FRCR_FSALL(n)        ((uint32_t)((n) - 1) << SAI_FRCR_FSALL_SHIFT)

#define SAI_FRCR_FSDEF             (1 << 16) /* Bit 16: Frame synchronization definition */

#  define SAI_FRCR_FSDEF_SF        (0)             /* FS signal is a start frame signal */
#  define SAI_FRCR_FSDEF_CHID      SAI_FRCR_FSDEF  /* FS signal is a start of frame + channel side ID */

#define SAI_FRCR_FSPOL             (1 << 17) /* Bit 17: Frame synchronization polarity */

#  define SAI_FRCR_FSPOL_LOW       (0)             /* FS is active low */
#  define SAI_FRCR_FSPOL_HIGH      SAI_FRCR_FSPOL  /* FS is active high */

#define SAI_FRCR_FSOFF             (1 << 18) /* Bit 18: Frame synchronization offset */

#  define SAI_FRCR_FSOFF_FB        (0)             /* FS on first bit of slot 0 */
#  define SAI_FRCR_FSOFF_BFB       SAI_FRCR_FSOFF  /* FS one bit before first bit of slot 0 */

                                             /* Bits 19-31: Reserved */

/* SAI Slot Register */

#define SAI_SLOTR_FBOFF_SHIFT      (0)       /* Bits 0-4: First bit offset */
#define SAI_SLOTR_FBOFF_MASK       (32 << SAI_SLOTR_FBOFF_SHIFT)
#  define SAI_SLOTR_FBOFF(n)       ((uint32_t)(n) << SAI_SLOTR_FBOFF_SHIFT)
                                             /* Bit 5: Reserved */
#define SAI_SLOTR_SLOTSZ_SHIFT     (6)       /* Bits 6-7: Slot size */
#define SAI_SLOTR_SLOTSZ_MASK      (3 << SAI_SLOTR_SLOTSZ_SHIFT)
#  define SAI_SLOTR_SLOTSZ_DATA    (0 << SAI_SLOTR_SLOTSZ_SHIFT) /* Same as data size */
#  define SAI_SLOTR_SLOTSZ_16BIT   (1 << SAI_SLOTR_SLOTSZ_SHIFT) /* 16-bit */
#  define SAI_SLOTR_SLOTSZ_32BIT   (2 << SAI_SLOTR_SLOTSZ_SHIFT) /* 32-bit */

#define SAI_SLOTR_NBSLOT_SHIFT     (0)       /* Bits 0-3: Number of slots in an audio frame */
#define SAI_SLOTR_NBSLOT_MASK      (15 << SAI_SLOTR_NBSLOT_SHIFT)
#  define SAI_SLOTR_NBSLOT(n)      ((uint32_t)((n) - 1) << SAI_SLOTR_NBSLOT_SHIFT)
                                             /* Bits 12-15: Reserved */
#define SAI_SLOTR_SLOTEN_SHIFT     (16)      /* Bits 16-31: Slot enable */
#define SAI_SLOTR_SLOTEN_MASK      (0xffff << SAI_SLOTR_SLOTEN_SHIFT)
#  define SAI_SLOTR_SLOTEN(n)      ((uint32_t)(n) << SAI_SLOTR_SLOTEN_SHIFT)
#  define SAI_SLOTR_SLOTEN_0       (1 << 16)  /* Bit 16: Slot 0 Enabled */
#  define SAI_SLOTR_SLOTEN_1       (1 << 17)  /* Bit 17: Slot 1 Enabled */
#  define SAI_SLOTR_SLOTEN_2       (1 << 18)  /* Bit 18: Slot 2 Enabled */
#  define SAI_SLOTR_SLOTEN_3       (1 << 19)  /* Bit 19: Slot 3 Enabled */
#  define SAI_SLOTR_SLOTEN_4       (1 << 20)  /* Bit 20: Slot 4 Enabled */
#  define SAI_SLOTR_SLOTEN_5       (1 << 21)  /* Bit 21: Slot 5 Enabled */
#  define SAI_SLOTR_SLOTEN_6       (1 << 22)  /* Bit 22: Slot 6 Enabled */
#  define SAI_SLOTR_SLOTEN_7       (1 << 23)  /* Bit 23: Slot 7 Enabled */
#  define SAI_SLOTR_SLOTEN_8       (1 << 24)  /* Bit 24: Slot 8 Enabled */
#  define SAI_SLOTR_SLOTEN_9       (1 << 25)  /* Bit 25: Slot 9 Enabled */
#  define SAI_SLOTR_SLOTEN_10      (1 << 26)  /* Bit 26: Slot 10 Enabled */
#  define SAI_SLOTR_SLOTEN_11      (1 << 27)  /* Bit 27: Slot 11 Enabled */
#  define SAI_SLOTR_SLOTEN_12      (1 << 28)  /* Bit 28: Slot 12 Enabled */
#  define SAI_SLOTR_SLOTEN_13      (1 << 29)  /* Bit 29: Slot 13 Enabled */
#  define SAI_SLOTR_SLOTEN_14      (1 << 30)  /* Bit 30: Slot 14 Enabled */
#  define SAI_SLOTR_SLOTEN_15      (1 << 31)  /* Bit 31: Slot 15 Enabled */

/* SAI Interrupt Mask Register 2, SAI Status Register,
 * and SAI Clear Flag Register
 */

#define SAI_INT_OVRUDR             (1 << 0)  /* Bit 0:  Overrun/underrun interrupt */
#define SAI_INT_MUTEDET            (1 << 1)  /* Bit 1:  Mute detection interrupt */
#define SAI_INT_WCKCFG             (1 << 2)  /* Bit 2:  Wrong clock configuration interrupt */
#define SAI_INT_FREQ               (1 << 3)  /* Bit 3:  FIFO request interrupt (not CLRFFR) */
#define SAI_INT_CNRDY              (1 << 4)  /* Bit 4:  Codec not ready interrupt (AC97). */
#define SAI_INT_AFSDET             (1 << 5)  /* Bit 5:  Anticipated frame synchronization detection interrupt */
#define SAI_INT_LFSDET             (1 << 6)  /* Bit 6:  Late frame synchronization detection interrupt */
                                             /* Bits 7-31: Reserved */

/* SAI Data Register (32-bit data) */

#endif /* __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_SAI_H */
