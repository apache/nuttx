/****************************************************************************
 * arch/arm/src/sam34/hardware/sam4cm_aes.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_AES_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_AES_H

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

#define SAM_AES_CR_OFFSET                0x0000 /* Control Register */
#define SAM_AES_MR_OFFSET                0x0004 /* Control Register */
#define SAM_AES_IER_OFFSET               0x0010 /* Interrupt Enable Register */
#define SAM_AES_IDR_OFFSET               0x0014 /* Interrupt Disable Register */
#define SAM_AES_IMR_OFFSET               0x0018 /* Interrupt Mask Register */
#define SAM_AES_ISR_OFFSET               0x001C /* Interrupt Status Register */
#define SAM_AES_KEYWR_OFFSET             0x0020 /* Key Word Register */
#define SAM_AES_IDATAR_OFFSET            0x0040 /* Input Data Register */
#define SAM_AES_ODATAR_OFFSET            0x0050 /* Output Data Register */
#define SAM_AES_IVR_OFFSET               0x0060 /* Initialization Vector Register */
#define SAM_AES_AADLENR_OFFSET           0x0070 /* Additional Authenticated Data Length Register */
#define SAM_AES_CLENR_OFFSET             0x0074 /* Plaintext/Ciphertext Length Register */
#define SAM_AES_GHASHR_OFFSET            0x0078 /* GCM Intermediate Hash Word Register */
#define SAM_AES_TAGR_OFFSET              0x0088 /* GCM Authentication Tag Word Register */
#define SAM_AES_CTRR_OFFSET              0x0098 /* GCM Encryption Counter Value Register */
#define SAM_AES_GCMHR_OFFSET             0x009C /* GCM H World Register */

/* AES register addresses ***************************************************/

#define SAM_AES_CR                       (SAM_AES_BASE + SAM_AES_CR_OFFSET)
#define SAM_AES_MR                       (SAM_AES_BASE + SAM_AES_MR_OFFSET)
#define SAM_AES_IER                      (SAM_AES_BASE + SAM_AES_IER_OFFSET)
#define SAM_AES_IDR                      (SAM_AES_BASE + SAM_AES_IDR_OFFSET)
#define SAM_AES_IMR                      (SAM_AES_BASE + SAM_AES_IMR_OFFSET)
#define SAM_AES_ISR                      (SAM_AES_BASE + SAM_AES_ISR_OFFSET)
#define SAM_AES_KEYWR                    (SAM_AES_BASE + SAM_AES_KEYWR_OFFSET)
#define SAM_AES_IDATAR                   (SAM_AES_BASE + SAM_AES_IDATAR_OFFSET)
#define SAM_AES_ODATAR                   (SAM_AES_BASE + SAM_AES_ODATAR_OFFSET)
#define SAM_AES_IVR                      (SAM_AES_BASE + SAM_AES_IVR_OFFSET)
#define SAM_AES_AADLENR                  (SAM_AES_BASE + SAM_AES_AADLENR_OFFSET)
#define SAM_AES_CLENR                    (SAM_AES_BASE + SAM_AES_CLENR_OFFSET)
#define SAM_AES_GHASHR                   (SAM_AES_BASE + SAM_AES_GHASHR_OFFSET)
#define SAM_AES_TAGR                     (SAM_AES_BASE + SAM_AES_TAGR_OFFSET)
#define SAM_AES_CTRR                     (SAM_AES_BASE + SAM_AES_CTRR_OFFSET)
#define SAM_AES_GCMHR                    (SAM_AES_BASE + SAM_AES_GCMHR_OFFSET)

/* AES register bit definitions *********************************************/

/* AES Control Register */

#define AES_CR_START                     (1 << 0) /* Start Processing */
#define AES_CR_SWRST                     (1 << 8) /* Software Reset */

/* AES Mode Register */

#define AES_MR_CIPHER_OFSET              (0)
#define AES_MR_CIPHER_MASK               (0x1 << AES_MR_CIPHER_OFSET)
#  define AES_MR_CIPHER_DECRYPT          (0 << AES_MR_CIPHER_OFSET)
#  define AES_MR_CIPHER_ENCRYPT          (1 << AES_MR_CIPHER_OFSET)
#define AES_MR_GTAGEN                    (1 << 1)                     /* GCM Automatic Tag Generation Enable */
#define AES_MR_DUALBUFF                  (1 << 3)                     /* Dual Input Buffer (requires SMOD = 0x2) */
#define AES_MR_PROCDLY(n)                ((n) << 4)                   /* Processing Time = 12 × ( PROCDLY + 1 ) */
#define AES_MR_SMOD_OFSET                (8)
#define AES_MR_SMOD_MASK                 (0x3 << AES_MR_SMOD_OFSET)
#  define AES_MR_SMOD_MANUAL_START       (0 << AES_MR_SMOD_OFSET)     /* Manual Mode */
#  define AES_MR_SMOD_AUTO_START         (1 << AES_MR_SMOD_OFSET)     /* Auto Mode */
#  define AES_MR_SMOD_IDATAR0_START      (2 << AES_MR_SMOD_OFSET)     /* AES_IDATAR0 access only Auto Mode */
#define AES_MR_KEYSIZE_OFFSET            (10)
#define AES_MR_KEYSIZE_MASK              (0x3 << AES_MR_KEYSIZE_OFFSET)
#  define AES_MR_KEYSIZE_AES128          (0 << AES_MR_KEYSIZE_OFFSET) /* AES Key Size is 128 bits */
#  define AES_MR_KEYSIZE_AES192          (1 << AES_MR_KEYSIZE_OFFSET) /* AES Key Size is 192 bits */
#  define AES_MR_KEYSIZE_AES256          (2 << AES_MR_KEYSIZE_OFFSET) /* AES Key Size is 256 bits */
#define AES_MR_OPMOD_OFFSET              (12)
#define AES_MR_OPMOD_MASK                (0x7 << AES_MR_OPMOD_OFFSET)
#  define AES_MR_OPMOD_ECB               (0 << AES_MR_OPMOD_OFFSET)
#  define AES_MR_OPMOD_CBC               (1 << AES_MR_OPMOD_OFFSET)
#  define AES_MR_OPMOD_OFB               (2 << AES_MR_OPMOD_OFFSET)
#  define AES_MR_OPMOD_CFB               (3 << AES_MR_OPMOD_OFFSET)
#  define AES_MR_OPMOD_CTR               (4 << AES_MR_OPMOD_OFFSET)
#  define AES_MR_OPMOD_GCM               (5 << AES_MR_OPMOD_OFFSET)
#define AES_MR_LOD                       (1 << 15)                    /* Last Output Data Mode */
#define AES_MR_CFBS_OFFSET               (16)                         /* Cipher Feedback Data Size */
#define AES_MR_CFBS_MASK                 (0x7 << AES_MR_CFBS_OFFSET)
#  define AES_MR_CFBS_SIZE_128BIT        (0 << AES_MR_CFBS_OFFSET)
#  define AES_MR_CFBS_SIZE_64BIT         (1 << AES_MR_CFBS_OFFSET)
#  define AES_MR_CFBS_SIZE_32BIT         (2 << AES_MR_CFBS_OFFSET)
#  define AES_MR_CFBS_SIZE_16BIT         (3 << AES_MR_CFBS_OFFSET)
#  define AES_MR_CFBS_SIZE_8BIT          (4 << AES_MR_CFBS_OFFSET)
#define AES_MR_CKEY                      (0xE << 20)

/* AES Interrupt Status Register */

#define AES_ISR_DATRDY                   (1 << 0)

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_AES_H */
