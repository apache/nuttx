/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32g0_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_FLASH_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

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

/* Register Addresses *******************************************************/

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

/* Register Bitfield Definitions ********************************************/

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
