/****************************************************************************
 * arch/arm/src/sam34/hardware/sam_aes.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_AES_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_AES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AES register offsets *****************************************************/

#define SAM_AES_CR_OFFSET            0x0000 /* Control Register */
#define SAM_AES_MR_OFFSET            0x0004 /* Mode Register */
                                            /* 0x0008-0x000c: Reserved */
#define SAM_AES_IER_OFFSET           0x0010 /* Interrupt Enable Register */
#define SAM_AES_IDR_OFFSET           0x0014 /* Interrupt Disable Register */
#define SAM_AES_IMR_OFFSET           0x0018 /* Interrupt Mask Register */
#define SAM_AES_ISR_OFFSET           0x001c /* Interrupt Status Register */
#define SAM_AES_KEYWR0_OFFSET        0x0020 /* Key Word Register 0 */
#define SAM_AES_KEYWR1_OFFSET        0x0024 /* Key Word Register 1 */
#define SAM_AES_KEYWR2_OFFSET        0x0028 /* Key Word Register 2 */
#define SAM_AES_KEYWR3_OFFSET        0x002c /* Key Word Register 3 */
#define SAM_AES_KEYWR4_OFFSET        0x0030 /* Key Word Register 4 */
#define SAM_AES_KEYWR5_OFFSET        0x0034 /* Key Word Register 5 */
#define SAM_AES_KEYWR6_OFFSET        0x0038 /* Key Word Register 6 */
#define SAM_AES_KEYWR7_OFFSET        0x003c /* Key Word Register 7 */
#define SAM_AES_IDATAR0_OFFSET       0x0040 /* Input Data Register 0 */
#define SAM_AES_IDATAR1_OFFSET       0x0044 /* Input Data Register 1 */
#define SAM_AES_IDATAR2_OFFSET       0x0048 /* Input Data Register 2 */
#define SAM_AES_IDATAR3_OFFSET       0x004c /* Input Data Register 3 */
#define SAM_AES_ODATAR0_OFFSET       0x0050 /* Output Data Register 0 */
#define SAM_AES_ODATAR1_OFFSET       0x0054 /* Output Data Register 1 */
#define SAM_AES_ODATAR2_OFFSET       0x0058 /* Output Data Register 2 */
#define SAM_AES_ODATAR3_OFFSET       0x005c /* Output Data Register 3 */
#define SAM_AES_IVR0_OFFSET          0x0060 /* Initialization Vector Register 0 */
#define SAM_AES_IVR1_OFFSET          0x0064 /* Initialization Vector Register 1 */
#define SAM_AES_IVR2_OFFSET          0x0068 /* Initialization Vector Register 2 */
#define SAM_AES_IVR3_OFFSET          0x006c /* Initialization Vector Register 3 */
                                            /* 0x0070-0x00fc: Reserved */

/* AES register addresses ***************************************************/

#define SAM_AES_CR                   (SAM_AES_BASE+SAM_AES_CR_OFFSET)
#define SAM_AES_MR                   (SAM_AES_BASE+SAM_AES_MR_OFFSET)
#define SAM_AES_IER                  (SAM_AES_BASE+SAM_AES_IER_OFFSET)
#define SAM_AES_IDR                  (SAM_AES_BASE+SAM_AES_IDR_OFFSET)
#define SAM_AES_IMR                  (SAM_AES_BASE+SAM_AES_IMR_OFFSET)
#define SAM_AES_ISR                  (SAM_AES_BASE+SAM_AES_ISR_OFFSET)
#define SAM_AES_KEYWR0               (SAM_AES_BASE+SAM_AES_KEYWR0_OFFSET)
#define SAM_AES_KEYWR1               (SAM_AES_BASE+SAM_AES_KEYWR1_OFFSET)
#define SAM_AES_KEYWR2               (SAM_AES_BASE+SAM_AES_KEYWR2_OFFSET)
#define SAM_AES_KEYWR3               (SAM_AES_BASE+SAM_AES_KEYWR3_OFFSET)
#define SAM_AES_KEYWR4               (SAM_AES_BASE+SAM_AES_KEYWR4_OFFSET)
#define SAM_AES_KEYWR5               (SAM_AES_BASE+SAM_AES_KEYWR5_OFFSET)
#define SAM_AES_KEYWR6               (SAM_AES_BASE+SAM_AES_KEYWR6_OFFSET)
#define SAM_AES_KEYWR7               (SAM_AES_BASE+SAM_AES_KEYWR7_OFFSET)
#define SAM_AES_IDATAR0              (SAM_AES_BASE+SAM_AES_IDATAR0_OFFSET)
#define SAM_AES_IDATAR1              (SAM_AES_BASE+SAM_AES_IDATAR1_OFFSET)
#define SAM_AES_IDATAR2              (SAM_AES_BASE+SAM_AES_IDATAR2_OFFSET)
#define SAM_AES_IDATAR3              (SAM_AES_BASE+SAM_AES_IDATAR3_OFFSET)
#define SAM_AES_ODATAR0              (SAM_AES_BASE+SAM_AES_ODATAR0_OFFSET)
#define SAM_AES_ODATAR1              (SAM_AES_BASE+SAM_AES_ODATAR1_OFFSET)
#define SAM_AES_ODATAR2              (SAM_AES_BASE+SAM_AES_ODATAR2_OFFSET)
#define SAM_AES_ODATAR3              (SAM_AES_BASE+SAM_AES_ODATAR3_OFFSET)
#define SAM_AES_IVR0                 (SAM_AES_BASE+SAM_AES_IVR0_OFFSET)
#define SAM_AES_IVR1                 (SAM_AES_BASE+SAM_AES_IVR1_OFFSET)
#define SAM_AES_IVR2                 (SAM_AES_BASE+SAM_AES_IVR2_OFFSET)
#define SAM_AES_IVR3                 (SAM_AES_BASE+SAM_AES_IVR3_OFFSET)

/* AES register bit definitions *********************************************/

/* Control Register */

#define AES_CR_START                 (1 << 0)  /* Bit 0:  Start Processing */
#define AES_CR_SWRST                 (1 << 8)  /* Bit 8:  Software Reset */

/* Mode Register */

#define AES_MR_CIPHER                (1 << 0)  /* Bit 0:  Processing Mode */
#define AES_MR_DUALBUFF              (1 << 3)  /* Bit 3:  Dual input buffer */
#define AES_MR_PROCDLY_SHIFT         (4)       /* Bits 4-7: Processing delay */
#define AES_MR_PROCDLY_MASK          (15 << AES_MR_PROCDLY_SHIFT)
#  define AES_MR_PROCDLY(n)          ((uint32_t)(n) << AES_MR_PROCDLY_SHIFT)
#define AES_MR_SMOD_SHIFT            (8)       /* Bits 8-9: Start mode */
#define AES_MR_SMOD_MASK             (3 << AES_MR_SMOD_SHIFT)
#  define AES_MR_SMOD_MANUAL         (0 << AES_MR_SMOD_SHIFT) /* Manual Mode */
#  define AES_MR_SMOD_AUTO           (1 << AES_MR_SMOD_SHIFT) /* Auto Mode */
#  define AES_MR_SMOD_IDATR0         (2 << AES_MR_SMOD_SHIFT) /* AES_IDATAR0 access only Auto Mode */

#define AES_MR_KEYSIZE_SHIFT         (10)      /* Bits 10-11: Key Size */
#define AES_MR_KEYSIZE_MASK          (2 << AES_MR_KEYSIZE_SHIFT)
#  define AES_MR_KEYSIZE_AES128      (0 << AES_MR_KEYSIZE_SHIFT) /* AES Key Size is 128 bits */
#  define AES_MR_KEYSIZE_AES192      (1 << AES_MR_KEYSIZE_SHIFT) /* AES Key Size is 192 bits */
#  define AES_MR_KEYSIZE_AES256      (2 << AES_MR_KEYSIZE_SHIFT) /* AES Key Size is 256 bits */

#define AES_MR_OPMOD_SHIFT           (12)       /* Bits 12-14: Operation Mode */
#define AES_MR_OPMOD_MASK            (7 << AES_MR_OPMOD_SHIFT)
#  define AES_MR_OPMOD_ECB           (0 << AES_MR_OPMOD_SHIFT) /* ECB: Electronic Code Book mode */
#  define AES_MR_OPMOD_CBC           (1 << AES_MR_OPMOD_SHIFT) /* CBC: Cipher Block Chaining mode */
#  define AES_MR_OPMOD_OFB           (2 << AES_MR_OPMOD_SHIFT) /* OFB: Output Feedback mode */
#  define AES_MR_OPMOD_CFB           (3 << AES_MR_OPMOD_SHIFT) /* CFB: Cipher Feedback mode */
#  define AES_MR_OPMOD_CTR           (4 << AES_MR_OPMOD_SHIFT) /* CTR: Counter mode (16-bit counter) */

#define AES_MR_LOD                   (1 << 15) /* Bit 15:  Last Output Data Mode */
#define AES_MR_CFBS_SHIFT            (16)      /* Bits 16-18: Cipher Feedback Data Size */
#define AES_MR_CFBS_MASK             (7 << AES_MR_CFBS_SHIFT)
#  define AES_MR_CFBS_128BIT         (0 << AES_MR_CFBS_SHIFT) /* 128-bit */
#  define AES_MR_CFBS_64BIT          (1 << AES_MR_CFBS_SHIFT) /* 64-bit */
#  define AES_MR_CFBS_32BIT          (2 << AES_MR_CFBS_SHIFT) /* 32-bit */
#  define AES_MR_CFBS_16BIT          (3 << AES_MR_CFBS_SHIFT) /* 16-bit */
#  define AES_MR_CFBS_8BIT           (4 << AES_MR_CFBS_SHIFT) /* 8-bit */

#define AES_MR_CKEY_SHIFT            (20)       /* Bits 20-23: Key */
#define AES_MR_CKEY_MASK             (15 << AES_MR_CKEY_SHIFT)
#  define AES_MR_CKEY                (14 << AES_MR_CKEY_SHIFT)

/* Interrupt Enable, Interrupt Disable, Interrupt Mask,
 * and Interrupt Status Register
 */

#define AES_INT_DATRDY               (1 << 0)  /* Bit 0:  Data Ready Interrupt */
#define AES_INT_URAD                 (1 << 8)  /* Bit 8:  Unspecified Register Access Detection Interrupt */

/* Interrupt Status Register (only) */

#define AES_ISR_URAT_SHIFT           (12)       /* Bits 12-15: Unspecified Register Access */
#define AES_ISR_URAT_MASK            (15 << AES_ISR_URAT_SHIFT)
#  define AES_ISR_URAT_IDRWRPROC     (0 << AES_ISR_URAT_SHIFT) /* IDATAR written during data processing */
#  define AES_ISR_URAT_ODRRDPROC     (1 << AES_ISR_URAT_SHIFT) /* ODATAR read during data processing */
#  define AES_ISR_URAT_MRWRPROC      (2 << AES_ISR_URAT_SHIFT) /* MR written during the data processing */
#  define AES_ISR_URAT_ODRRDSUBKG    (3 << AES_ISR_URAT_SHIFT) /* ODATAR read during the sub-keys generation */
#  define AES_ISR_URAT_MRWRSUBKG     (4 << AES_ISR_URAT_SHIFT) /* MR written during the sub-keys generation */
#  define AES_ISR_URAT_WORRDACC      (5 << AES_ISR_URAT_SHIFT) /* WRONLY register read access */

/* Key Word Register 0-7 (32-bit value) */

/* Input Data Register 0-7 (32-bit value) */

/* Initialization Vector Register 0-7 (32-bit value) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_AES_H */
