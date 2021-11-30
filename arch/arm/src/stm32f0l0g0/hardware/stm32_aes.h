/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32_aes.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_AES_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_AES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AES register offsets *****************************************************/

#define STM32_AES_CR_OFFSET                0x0000 /* Control Register */
#define STM32_AES_SR_OFFSET                0x0004 /* Status Register */
#define STM32_AES_DINR_OFFSET              0x0008 /* Data Input Register */
#define STM32_AES_DOUTR_OFFSET             0x000C /* Data Output Register */
#define STM32_AES_KEYR0_OFFSET             0x0010 /* AES Key Register 0 */
#define STM32_AES_KEYR1_OFFSET             0x0014 /* AES Key Register 1 */
#define STM32_AES_KEYR2_OFFSET             0x0018 /* AES Key Register 2 */
#define STM32_AES_KEYR3_OFFSET             0x001C /* AES Key Register 3 */
#define STM32_AES_IVR0_OFFSET              0x0020 /* AES Initialization Vector Register 0 */
#define STM32_AES_IVR1_OFFSET              0x0024 /* AES Initialization Vector Register 1 */
#define STM32_AES_IVR2_OFFSET              0x0028 /* AES Initialization Vector Register 2 */
#define STM32_AES_IVR3_OFFSET              0x002C /* AES Initialization Vector Register 3 */

/* AES register addresses ***************************************************/

#define STM32_AES_CR                       (STM32_AES_BASE + STM32_AES_CR_OFFSET)
#define STM32_AES_SR                       (STM32_AES_BASE + STM32_AES_SR_OFFSET)
#define STM32_AES_DINR                     (STM32_AES_BASE + STM32_AES_DINR_OFFSET)
#define STM32_AES_DOUTR                    (STM32_AES_BASE + STM32_AES_DOUTR_OFFSET)
#define STM32_AES_KEYR0                    (STM32_AES_BASE + STM32_AES_KEYR0_OFFSET)
#define STM32_AES_KEYR1                    (STM32_AES_BASE + STM32_AES_KEYR1_OFFSET)
#define STM32_AES_KEYR2                    (STM32_AES_BASE + STM32_AES_KEYR2_OFFSET)
#define STM32_AES_KEYR3                    (STM32_AES_BASE + STM32_AES_KEYR3_OFFSET)
#define STM32_AES_IVR0                     (STM32_AES_BASE + STM32_AES_IVR0_OFFSET)
#define STM32_AES_IVR1                     (STM32_AES_BASE + STM32_AES_IVR1_OFFSET)
#define STM32_AES_IVR2                     (STM32_AES_BASE + STM32_AES_IVR2_OFFSET)
#define STM32_AES_IVR3                     (STM32_AES_BASE + STM32_AES_IVR3_OFFSET)

/* AES register bit definitions *********************************************/

/* AES_CR register */

#define AES_CR_EN                          (1 << 0)       /* AES Enable */
#define AES_CR_DATATYPE                    (1 << 1)       /* Data type selection */
#  define AES_CR_DATATYPE_LE               (0x0 << 1)
#  define AES_CR_DATATYPE_BE               (0x2 << 1)

#define AES_CR_MODE                        (1 << 3)       /* AES Mode of operation */
#  define AES_CR_MODE_ENCRYPT              (0x0 << 3)
#  define AES_CR_MODE_KEYDERIV             (0x1 << 3)
#  define AES_CR_MODE_DECRYPT              (0x2 << 3)
#  define AES_CR_MODE_DECRYPT_KEYDERIV     (0x3 << 3)

#define AES_CR_CHMOD                       (1 << 5)       /* AES Chaining Mode */
#  define AES_CR_CHMOD_ECB                 (0x0 << 5)
#  define AES_CR_CHMOD_CBC                 (0x1 << 5)
#  define AES_CR_CHMOD_CTR                 (0x2 << 5)

#define AES_CR_CCFC                        (1 << 7)       /* Computation Complete Flag Clear */
#define AES_CR_ERRC                        (1 << 8)       /* Error Clear */
#define AES_CR_CCIE                        (1 << 9)       /* Computation Complete Interrupt Enable */
#define AES_CR_ERRIE                       (1 << 10)      /* Error Interrupt Enable */
#define AES_CR_DMAINEN                     (1 << 11)      /* DMA Enable Input */
#define AES_CR_DMAOUTEN                    (1 << 12)      /* DMA Enable Output */

/* AES_SR register */

#define AES_SR_CCF                         (1 << 0)       /* Computation Complete Flag */
#define AES_SR_RDERR                       (1 << 1)       /* Read Error Flag */
#define AES_SR_WRERR                       (1 << 2)       /* Write Error Flag */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_AES_H */
