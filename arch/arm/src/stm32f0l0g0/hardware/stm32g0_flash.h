/************************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32g0_flash.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_FLASH_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_FLASH_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_FLASH_ACR_OFFSET       0x0000
#define STM32_FLASH_KEYR_OFFSET      0x0008
#define STM32_FLASH_OPTKEYR_OFFSET   0x000c
#define STM32_FLASH_SR_OFFSET        0x0010
#define STM32_FLASH_CR_OFFSET        0x0014
#define STM32_FLASH_ECCR_OFFSET      0x0018
#define STM32_FLASH_OPTR_OFFSET      0x0020
#define STM32_FLASH_PCROP1ASE_OFFSET 0x0024
#define STM32_FLASH_PCROP1AER_OFFSET 0x0028
#define STM32_FLASH_WPR1AR_OFFSET    0x002c
#define STM32_FLASH_WPR1BR_OFFSET    0x0030
#define STM32_FLASH_WPR1BSR_OFFSET   0x0034
#define STM32_FLASH_PCROP1BER_OFFSET 0x0038
#define STM32_FLASH_SECR_OFFSET      0x0080

/* Register Addresses ***************************************************************/

#define STM32_FLASH_ACR              (STM32_FLASHIF_BASE+STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_KEYR             (STM32_FLASHIF_BASE+STM32_FLASH_KEYR_OFFSET)
#define STM32_FLASH_OPTKEYR          (STM32_FLASHIF_BASE+STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_SR               (STM32_FLASHIF_BASE+STM32_FLASH_SR_OFFSET)
#define STM32_FLASH_CR               (STM32_FLASHIF_BASE+STM32_FLASH_CR_OFFSET)
#define STM32_FLASH_ECCR             (STM32_FLASHIF_BASE+STM32_FLASH_ECCR_OFFSET)
#define STM32_FLASH_OPTR             (STM32_FLASHIF_BASE+STM32_FLASH_OPTR_OFFSET)
#define STM32_FLASH_PCROP1ASE        (STM32_FLASHIF_BASE+STM32_FLASH_PCROP1ASE_OFFSET)
#define STM32_FLASH_PCROP1AER        (STM32_FLASHIF_BAER+STM32_FLASH_PCROP1AER_OFFSET)
#define STM32_FLASH_WPR1AR           (STM32_FLASHIF_BASE+STM32_FLASH_WPR1AR_OFFSET)
#define STM32_FLASH_WPR1BR           (STM32_FLASHIF_BASE+STM32_FLASH_WPR1BR_OFFSET)
#define STM32_FLASH_WPR1BSR          (STM32_FLASHIF_BASE+STM32_FLASH_WPR1BSR_OFFSET)
#define STM32_FLASH_PCROP1BER        (STM32_FLASHIF_BASE+STM32_FLASH_PCROP1BER_OFFSET)
#define STM32_FLASH_SECR             (STM32_FLASHIF_BASE+STM32_FLASH_SECR_OFFSET)

/* Register Bitfield Definitions ****************************************************/
/* Flash Access Control Register (ACR) */

#define FLASH_ACR_LATENCY_SHIFT     (0)       /* Bits 0-2: Flash memory access latency*/
#define FLASH_ACR_LATENCY_MASK      (7 << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY(n)      ((n) << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY_0       (0 << FLASH_ACR_LATENCY_SHIFT)    /* 000: Zero wait states  */
#  define FLASH_ACR_LATENCY_1       (1 << FLASH_ACR_LATENCY_SHIFT)    /* 001: One wait state    */
#  define FLASH_ACR_LATENCY_2       (2 << FLASH_ACR_LATENCY_SHIFT)    /* 010: Two wait states   */
                                              /* Bits 3-7: Reserved */
#define FLASH_ACR_PRFTEN            (1 << 8)  /* Bit 8: Prefetch enable */
#define FLASH_ACR_ICEN              (1 << 9)  /* Bit 9: Instruction cache enable */
                                              /* Bit 10:Reserved */
#define FLASH_ACR_ICRST             (1 << 11) /* Bit 11:Instruction cache reset */
                                              /* Bits 12-15: Reserved */
#define FLASH_ACR_EMPTY             (1 << 16) /* Bit 16: Main Flash memory area empty */
                                              /* Bit 17: Reserved */
#define FLASH_ACR_DBGSWEN           (1 << 18) /* Bit 18: Debug access software enable */
                                              /* Bits 19-31: Reserved */

/* TODO */


#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_FLASH_H */
