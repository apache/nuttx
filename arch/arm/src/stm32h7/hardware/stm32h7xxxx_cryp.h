/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32h7xxxx_cryp.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_CRYP_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_CRYP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "chip.h"

/* The following STM32H7 devices have a cryptographic accelerator that
 * support AES, DES/TDES:
 *
 * STM32H735ZO, STM32H750XX, STM32H753XX, STM32H755XX, STM32H757XX,
 * STM32H7B0XX, STM32H7B3XX, STM32H7S3x8, STM32H7S7X8
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CRYP register offsets ****************************************************/

#define STM32_CRYP_CR_OFFSET                0x0000 /* Control Register */
#define STM32_CRYP_SR_OFFSET                0x0004 /* Status Register */
#define STM32_CRYP_DIN_OFFSET               0x0008 /* Data Input Register */
#define STM32_CRYP_DOUT_OFFSET              0x000C /* Data Output Register */
#define STM32_CRYP_DMACR_OFFSET             0x0010 /* DMA Control Register */
#define STM32_CRYP_IMSCR_OFFSET             0x0014 /* Interrupt Mask Register */
#define STM32_CRYP_RISR_OFFSET              0x0018 /* Raw Interrupt Status Register */
#define STM32_CRYP_MISR_OFFSET              0x001C /* Masked Interrupt Status Register */

#define STM32_CRYP_K0LR_OFFSET              0x0020 /* CRYP key 0L Register */
#define STM32_CRYP_K0RR_OFFSET              0x0024 /* CRYP key 0R Register */
#define STM32_CRYP_K1LR_OFFSET              0x0028 /* CRYP key 1L Register */
#define STM32_CRYP_K1RR_OFFSET              0x002c /* CRYP key 1R Register */
#define STM32_CRYP_K2LR_OFFSET              0x0030 /* CRYP key 2L Register */
#define STM32_CRYP_K2RR_OFFSET              0x0034 /* CRYP key 2R Register */
#define STM32_CRYP_K3LR_OFFSET              0x0038 /* CRYP key 3L Register */
#define STM32_CRYP_K3RR_OFFSET              0x003c /* CRYP key 3R Register */

#define STM32_CRYP_IV0LR_OFFSET             0x0040 /* CRYP Init Vector 0L Register */
#define STM32_CRYP_IV0RR_OFFSET             0x0044 /* CRYP Init Vector 0R Register */
#define STM32_CRYP_IV1LR_OFFSET             0x0048 /* CRYP Init Vector 1L Register */
#define STM32_CRYP_IV1RR_OFFSET             0x004C /* CRYP Init Vector 1R Register */

#define STM32_CRYP_CSGCMCCM0R_OFFSET        0x0050 /* CRYP Context Swap Register 0R */

/* CRYP register addresses **************************************************/

#define STM32_CRYP_CR           (STM32_CRYPTO_BASE + STM32_CRYP_CR_OFFSET)
#define STM32_CRYP_SR           (STM32_CRYPTO_BASE + STM32_CRYP_SR_OFFSET)
#define STM32_CRYP_DIN          (STM32_CRYPTO_BASE + STM32_CRYP_DIN_OFFSET)
#define STM32_CRYP_DOUT         (STM32_CRYPTO_BASE + STM32_CRYP_DOUT_OFFSET)
#define STM32_CRYP_DMACR        (STM32_CRYPTO_BASE + STM32_CRYP_DMACR_OFFSET)
#define STM32_CRYP_IMSCR        (STM32_CRYPTO_BASE + STM32_CRYP_IMSCR_OFFSET)
#define STM32_CRYP_RISR         (STM32_CRYPTO_BASE + STM32_CRYP_RISR_OFFSET)
#define STM32_CRYP_MISR         (STM32_CRYPTO_BASE + STM32_CRYP_MISR_OFFSET)
#define STM32_CRYP_K0LR         (STM32_CRYPTO_BASE + STM32_CRYP_K0LR_OFFSET)
#define STM32_CRYP_K0RR         (STM32_CRYPTO_BASE + STM32_CRYP_K0RR_OFFSET)
#define STM32_CRYP_K1LR         (STM32_CRYPTO_BASE + STM32_CRYP_K1LR_OFFSET)
#define STM32_CRYP_K1RR         (STM32_CRYPTO_BASE + STM32_CRYP_K1RR_OFFSET)
#define STM32_CRYP_K2LR         (STM32_CRYPTO_BASE + STM32_CRYP_K2LR_OFFSET)
#define STM32_CRYP_K2RR         (STM32_CRYPTO_BASE + STM32_CRYP_K2RR_OFFSET)
#define STM32_CRYP_K3LR         (STM32_CRYPTO_BASE + STM32_CRYP_K3LR_OFFSET)
#define STM32_CRYP_K3RR         (STM32_CRYPTO_BASE + STM32_CRYP_K3RR_OFFSET)
#define STM32_CRYP_IV0LR        (STM32_CRYPTO_BASE + STM32_CRYP_IV0LR_OFFSET)
#define STM32_CRYP_IV0RR        (STM32_CRYPTO_BASE + STM32_CRYP_IV0RR_OFFSET)
#define STM32_CRYP_IV1LR        (STM32_CRYPTO_BASE + STM32_CRYP_IV1LR_OFFSET)
#define STM32_CRYP_IV1RR        (STM32_CRYPTO_BASE + STM32_CRYP_IV1RR_OFFSET)
#define STM32_CRYP_CSGCMCCM0R   (STM32_CRYPTO_BASE + STM32_CRYP_CSGCMCCM0R_OFFSET)

/* CRYP register bit definitions ********************************************/

/* CRYP CR register */

#define CRYP_CR_GCMPH_SHIFT             16
#define CRYP_CR_GCMPH_MASK              (0x3 << CRYP_CR_GCMPH_SHIFT)
#  define CRYP_CR_GCMPH_INIT            (0 << CRYP_CR_GCMPH_SHIFT)
#  define CRYP_CR_GCMPH_HEADER          (1 << CRYP_CR_GCMPH_SHIFT)
#  define CRYP_CR_GCMPH_PAYLOAD         (2 << CRYP_CR_GCMPH_SHIFT)
#  define CRYP_CR_GCMPH_FINAL           (3 << CRYP_CR_GCMPH_SHIFT)
#define CRYP_CR_CRYPEN                  (1 << 15)
#define CRYP_CR_FFLUSH                  (1 << 14)
#define CRYP_CR_KEYSIZE_SHIFT           8
#define CRYP_CR_KEYSIZE_MASK            (0x3 << CRYP_CR_KEYSIZE_SHIFT)
#  define CRYP_CR_KEYSIZE_128           (0 << CRYP_CR_KEYSIZE_SHIFT)
#  define CRYP_CR_KEYSIZE_192           (1 << CRYP_CR_KEYSIZE_SHIFT)
#  define CRYP_CR_KEYSIZE_256           (2 << CRYP_CR_KEYSIZE_SHIFT)
#define CRYP_CR_DATATYPE_SHIFT          6
#define CRYP_CR_DATATYPE_MASK           (0x3 << CRYP_CR_DATATYPE_SHIFT)
#  define CRYP_CR_DATATYPE_LE           (0 << CRYP_CR_DATATYPE_SHIFT)
#  define CRYP_CR_DATATYPE_BE           (2 << CRYP_CR_DATATYPE_SHIFT)
/* ALGOMODE straddles bits [19, 5:3] of CRYP_CR */
#define CRYP_CR_ALGOMODE3_SHIFT         19
#define CRYP_CR_ALGOMODE3_MASK          (0x1 << CRYP_CR_ALGOMODE3_SHIFT)
#define CRYP_CR_ALGOMODE_SHIFT          3
#define CRYP_CR_ALGOMODE_MASK           ((0xf << CRYP_CR_ALGOMODE_SHIFT) \
                                         | CRYP_CR_ALGOMODE3_MASK)
#  define CRYP_CR_ALGOMODE_TDES_ECB     ((0 << CRYP_CR_ALGOMODE3_SHIFT) \
                                         | (0 << CRYP_CR_ALGOMODE_SHIFT))
#  define CRYP_CR_ALGOMODE_TDES_CBC     ((1 << CRYP_CR_ALGOMODE_SHIFT) \
                                         | (0 << CRYP_CR_ALGOMODE_SHIFT))
#  define CRYP_CR_ALGOMODE_DES_ECB      ((2 << CRYP_CR_ALGOMODE_SHIFT) \
                                         | (0 << CRYP_CR_ALGOMODE_SHIFT))
#  define CRYP_CR_ALGOMODE_DES_CBC      ((3 << CRYP_CR_ALGOMODE_SHIFT) \
                                         | (0 << CRYP_CR_ALGOMODE_SHIFT))
#  define CRYP_CR_ALGOMODE_AES_ECB      ((4 << CRYP_CR_ALGOMODE_SHIFT) \
                                         | (0 << CRYP_CR_ALGOMODE_SHIFT))
#  define CRYP_CR_ALGOMODE_AES_CBC      ((5 << CRYP_CR_ALGOMODE_SHIFT) \
                                         | (0 << CRYP_CR_ALGOMODE_SHIFT))
#  define CRYP_CR_ALGOMODE_AES_CTR      ((6 << CRYP_CR_ALGOMODE_SHIFT) \
                                         | (0 << CRYP_CR_ALGOMODE_SHIFT))
#  define CRYP_CR_ALGOMODE_AES_KEY_PREP ((7 << CRYP_CR_ALGOMODE_SHIFT)  \
                                         | (0 << CRYP_CR_ALGOMODE_SHIFT))
#  define CRYP_CR_ALGOMODE_AES_GCM      ((8 << CRYP_CR_ALGOMODE_SHIFT) \
                                         | (1 << CRYP_CR_ALGOMODE_SHIFT))
#  define CRYP_CR_ALGOMODE_AES_CCM      ((9 << CRYP_CR_ALGOMODE_SHIFT)  \
                                         | (1 << CRYP_CR_ALGOMODE_SHIFT))
#define CRYP_CR_ALGODIR_SHIFT           2
#define CRYP_CR_ALGODIR_MASK            (1 << CRYP_CR_ALGODIR_SHIFT)
#  define CRYP_CR_ALGODIR_ENCRYPT       (0 << CRYP_CR_ALGODIR_SHIFT)
#  define CRYP_CR_ALGODIR_DECRYPT       (1 << CRYP_CR_ALGODIR_SHIFT)

/* All non-defined bits in CRYP_CR have to be set
 * to the reset value (0x00000000). Following is mask for such
 */
#define CRYP_CR_RESET_MASK                                          \
  (CRYP_CR_ALGOMODE3_MASK | CRYP_CR_GCMPH_MASK | CRYP_CR_CRYPEN     \
   | CRYP_CR_FFLUSH | CRYP_CR_KEYSIZE_MASK | CRYP_CR_DATATYPE_MASK  \
   | CRYP_CR_ALGOMODE_MASK | CRYP_CR_ALGODIR_MASK)

/* CRYP SR register */

#define CRYP_SR_BUSY                    (1 << 4) /* BUSY */
#define CRYP_SR_OFFU                    (1 << 3) /* Output Fifo Full */
#define CRYP_SR_OFNE                    (1 << 2) /* Output Fifo Not Empty */
#define CRYP_SR_IFNF                    (1 << 1) /* Input Fifo Not Full */
#define CRYP_SR_IFEM                    (1 << 0) /* Input Fifo Empty */

/* CRYP DMACR register */

#define CRYP_DMACR_DOEN                 (1 << 1) /* DMA Output Enable */
#define CRYP_DMACR_DIEN                 (1 << 0) /* DMA Input Enable */

/* CRYP IMSCR register */

#define CRYP_IMSCR_OUTIM                (1 << 1) /* Output Fifo Service Mask */
#define CRYP_IMSCR_INIM                 (1 << 0) /* Input Fifo Service Mask */

/* CRYP RISR register */

#define CRYP_RISR_OUTRIS                (1 << 1)
#define CRYP_RISR_INRIS                 (1 << 0)

/* CRYP MISR register */

#define CRYP_MISR_OUTMIS                (1 << 1)
#define CRYP_MISR_INMIS                 (1 << 0)

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_CRYP_H */
