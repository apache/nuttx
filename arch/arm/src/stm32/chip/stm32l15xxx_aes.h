/********************************************************************************************
 * arch/arm/src/stm32/chip/stm32l15xxx_aes.h
 * AES hardware accelerator for STM32L162xx advanced ARM-based
 * 32-bit MCUs
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32L15XXX_AES_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32L15XXX_AES_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/stm32l15xxx_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* AES register offsets *********************************************************************/

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

/* AES register addresses *******************************************************************/

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

/* AES register bit definitions *************************************************************/

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

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32L15XXX_AES_H */
