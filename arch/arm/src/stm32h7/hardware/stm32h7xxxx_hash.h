/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32h7xxxx_hash.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7XXXX_HASH_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7XXXX_HASH_H

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

/* HASH register offsets ****************************************************/

#define STM32_HASH_CR_OFFSET                0x0000 /* Control Register */
#define STM32_HASH_DIN_OFFSET               0x0004 /* Data Input Register */
#define STM32_HASH_STR_OFFSET               0x0008 /* Start Register */
#define STM32_HASH_HRA0_OFFSET              0x000C /* Alias Digest Reg 0 */
#define STM32_HASH_HRA1_OFFSET              0x0010 /* Alias Digest Reg 1 */
#define STM32_HASH_HRA2_OFFSET              0x0014 /* Alias Digest Reg 2 */
#define STM32_HASH_HRA3_OFFSET              0x0018 /* Alias Digest Reg 3 */
#define STM32_HASH_HRA4_OFFSET              0x001C /* Alias Digest Reg 4 */

#define STM32_HASH_IMR_OFFSET               0x0020 /* Interrupt Mask Register */
#define STM32_HASH_SR_OFFSET                0x0024 /* Status Register */

#define STM32_HASH_CSR0_OFFSET              0x00f8 /* Context swap Register 0 */
#define STM32_HASH_CSRx_OFFSET(x)           (STM32_HASH_CSR0_OFFSET + 4 * (x))

#define STM32_HASH_HR0_OFFSET               0x0310 /* Digest Reg 0 */
#define STM32_HASH_HR1_OFFSET               0x0314 /* Digest Reg 1 */
#define STM32_HASH_HR2_OFFSET               0x0318 /* Digest Reg 2 */
#define STM32_HASH_HR3_OFFSET               0x031c /* Digest Reg 3 */
#define STM32_HASH_HR4_OFFSET               0x0320 /* Digest Reg 4 */
#define STM32_HASH_HR5_OFFSET               0x0324 /* Supplemental Digest Reg 5 */
#define STM32_HASH_HR6_OFFSET               0x0328 /* Supplemental Digest Reg 6 */
#define STM32_HASH_HR7_OFFSET               0x032C /* Supplemental Digest Reg 7 */

/* HASH register addresses **************************************************/

#define STM32_HASH_CR           (STM32_HASH_BASE + STM32_HASH_CR_OFFSET)
#define STM32_HASH_DIN          (STM32_HASH_BASE + STM32_HASH_DIN_OFFSET)
#define STM32_HASH_STR          (STM32_HASH_BASE + STM32_HASH_STR_OFFSET)
#define STM32_HASH_DMACR        (STM32_HASH_BASE + STM32_HASH_DMACR_OFFSET)
#define STM32_HASH_HRA0         (STM32_HASH_BASE + STM32_HASH_HRA0_OFFSET)
#define STM32_HASH_HRA1         (STM32_HASH_BASE + STM32_HASH_HRA1_OFFSET)
#define STM32_HASH_HRA2         (STM32_HASH_BASE + STM32_HASH_HRA2_OFFSET)
#define STM32_HASH_HRA3         (STM32_HASH_BASE + STM32_HASH_HRA3_OFFSET)
#define STM32_HASH_HRA4         (STM32_HASH_BASE + STM32_HASH_HRA4_OFFSET)
#define STM32_HASH_IMR          (STM32_HASH_BASE + STM32_HASH_IMR_OFFSET)
#define STM32_HASH_SR           (STM32_HASH_BASE + STM32_HASH_SR_OFFSET)
#define STM32_HASH_CSR0         (STM32_HASH_BASE + STM32_HASH_CSR0_OFFSET)
#define STM32_HASH_HR0          (STM32_HASH_BASE + STM32_HASH_HR0_OFFSET)
#define STM32_HASH_HR1          (STM32_HASH_BASE + STM32_HASH_HR1_OFFSET)
#define STM32_HASH_HR2          (STM32_HASH_BASE + STM32_HASH_HR2_OFFSET)
#define STM32_HASH_HR3          (STM32_HASH_BASE + STM32_HASH_HR3_OFFSET)
#define STM32_HASH_HR4          (STM32_HASH_BASE + STM32_HASH_HR4_OFFSET)
#define STM32_HASH_HR5          (STM32_HASH_BASE + STM32_HASH_HR5_OFFSET)
#define STM32_HASH_HR6          (STM32_HASH_BASE + STM32_HASH_HR6_OFFSET)
#define STM32_HASH_HR7          (STM32_HASH_BASE + STM32_HASH_HR7_OFFSET)

/* HASH register bit definitions ********************************************/

/* HASH CR register */

#define HASH_CR_LKEY_SHIFT         16
#define HASH_CR_LKEY_MASK          (1 << HASH_CR_LKEY_SHIFT)
#define HASH_CR_LKEY_LE_64         (0 << HASH_CR_LKEY_SHIFT)
#define HASH_CR_LKEY_GT_64         (1 << HASH_CR_LKEY_SHIFT)
#define HASH_CR_MDMAT_SHIFT        13
#define HASH_CR_MDMAT_MASK         (1 << HASH_CR_MDMAT_SHIFT)
#define HASH_CR_MDMAT_AUTO_SET     (0 << HASH_CR_MDMAT_SHIFT)
#define HASH_CR_MDMAT_NOT_AUTO_SET (0 << HASH_CR_MDMAT_SHIFT)
#define HASH_CR_DINNE_SHIFT        12
#define HASH_CR_DINNE_MASK         (1 << HASH_CR_DINNE_SHIFT)
#define HASH_CR_DINNE_EMPTY        (0 << HASH_CR_DINNE_SHIFT)
#define HASH_CR_DINNE_NOT_EMPTY    (1 << HASH_CR_DINNE_SHIFT)
#define HASH_CR_NBW_SHIFT          8
/* If 0 && DINNE == 0 then no word pushed; if DINNEset then one word pushed.
 * If 1 then two words pushed (one in HASH_DIN and other in FIFO)
 */
#define HASH_CR_NBW_MASK           (0xf << HASH_CR_NBW_SHIFT)

#define HASH_CR_ALGO_MASK          ((1 << 18) | (1 << 7))
#define HASH_CR_ALGO_SHA1          ((0 << 18) | (0 << 7))
#define HASH_CR_ALGO_MD5           ((0 << 18) | (1 << 7))
#define HASH_CR_ALGO_SHA2_224      ((1 << 18) | (0 << 7))
#define HASH_CR_ALGO_SHA2_256      ((1 << 18) | (1 << 7))

#define HASH_CR_MODE_SHIFT         6
#define HASH_CR_MODE_MASK          (1 << HASH_CR_MODE_SHIFT)
#define HASH_CR_MODE_HASH          (0 << HASH_CR_MODE_SHIFT)
#define HASH_CR_MODE_HMAC          (1 << HASH_CR_MODE_SHIFT)
#define HASH_CR_DATATYPE_SHIFT     4
#define HASH_CR_DATATYPE_MASK      (0x3 << HASH_CR_DATATYPE_SHiFT)
#define HASH_CR_DATATYPE_32        (0 << HASH_CR_DATATYPE_SHIFT)
#define HASH_CR_DATATYPE_16        (1 << HASH_CR_DATATYPE_SHIFT)
#define HASH_CR_DATATYPE_8         (2 << HASH_CR_DATATYPE_SHIFT)
#define HASH_CR_DATATYPE_1         (3 << HASH_CR_DATATYPE_SHIFT)
#define HASH_CR_DMAE_SHIFT         3
#define HASH_CR_DMAE_MASK          (1 << HASH_CR_DMAE_SHIFT)
#define HASH_CR_DMAE_DISABLE       (0 << HASH_CR_DMAE_SHIFT)
#define HASH_CR_DMAE_ENABLE        (1 << HASH_CR_DMAE_SHIFT)
#define HASH_CR_INIT_SHIFT         2
#define HASH_CR_INIT_MASK          (1 << HASH_CR_INIT_SHIFT)
#define HASH_CR_INIT               HASH_CR_INIT_MASK

/* HASH STR register */

#define HASH_STR_DCAL_SHIFT        8
#define HASH_STR_DCAL              (1 << HASH_STR_DCAL_SHIFT)
#define HASH_STR_DCAL_START        HASH_STR_DCAL_MASK
#define HASH_STR_NBLW_SHIFT        0
#define HASH_STR_NBLW_MASK         (0x1F << HASH_STR_NBLW_SHIFT)
#define HASH_STR_NBLW(n)           (((n) << HASH_STR_NBLW_SHIFT) \
                                    & HASH_STR_NBLW_MASK)
#define HASH_STR_NBLW_BYTES(n)     ((((n) * 8) << HASH_STR_NBLW_SHIFT) \
                                    & HASH_STR_NBLW_MASK)

/* HASH IMR register */

#define HASH_IMR_DCIE_SHIFT        1
#define HASH_IMR_DCIE_MASK         (1 << HASH_IMR_DCIE_SHIFT)
#define HASH_IMR_DCIE_DISABLE      (0 << HASH_IMR_DCIE_SHIFT)
#define HASH_IMR_DCIE_ENABLE       (1 << HASH_IMR_DCIE_SHIFT)
#define HASH_IMR_DINIE_SHIFT       1
#define HASH_IMR_DINIE_MASK        (1 << HASH_IMR_DINIE_SHIFT)
#define HASH_IMR_DINIE_DISABLE     (0 << HASH_IMR_DINIE_SHIFT)
#define HASH_IMR_DINIE_ENABLE      (1 << HASH_IMR_DINIE_SHIFT)

/* HASH SR register */

#define HASH_SR_BUSY_SHIFT         3
#define HASH_SR_BUSY_MASK          (1 << HASH_SR_BUSY_SHIFT)
#define HASH_SR_BUSY               HASH_SR_BUSY_MASK
#define HASH_SR_DMAS_SHIFT         2
#define HASH_SR_DMAS_MASK          (1 << HASH_SR_DMAS_SHIFT)
#define HASH_SR_DMAS               HASH_SR_DMAS_MASK
#define HASH_SR_DCIS_SHIFT         1
#define HASH_SR_DCIS_MASK          (1 << HASH_SR_DCIS_SHIFT)
#define HASH_SR_DCIS               HASH_SR_DCIS_MASK
#define HASH_SR_DINIS_SHIFT        0
#define HASH_SR_DINIS_MASK         (1 << HASH_SR_DINIS_SHIFT)
#define HASH_SR_DINIS              HASH_SR_DINIS_MASK

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7XXXX_HASH_H */
