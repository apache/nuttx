/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32f0_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32F0_FLASH_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32F0_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_FLASH_ACR_OFFSET      0x0000
#define STM32_FLASH_KEYR_OFFSET     0x0004
#define STM32_FLASH_OPTKEYR_OFFSET  0x0008
#define STM32_FLASH_SR_OFFSET       0x000c
#define STM32_FLASH_CR_OFFSET       0x0010
#define STM32_FLASH_AR_OFFSET       0x0014
#define STM32_FLASH_OBR_OFFSET      0x001c
#define STM32_FLASH_WRPR_OFFSET     0x0020

/* Register Addresses *******************************************************/

#define STM32_FLASH_ACR             (STM32_FLASHIF_BASE + STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_KEYR            (STM32_FLASHIF_BASE + STM32_FLASH_KEYR_OFFSET)
#define STM32_FLASH_OPTKEYR         (STM32_FLASHIF_BASE + STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_SR              (STM32_FLASHIF_BASE + STM32_FLASH_SR_OFFSET)
#define STM32_FLASH_CR              (STM32_FLASHIF_BASE + STM32_FLASH_CR_OFFSET)
#define STM32_FLASH_AR              (STM32_FLASHIF_BASE + STM32_FLASH_AR_OFFSET)
#define STM32_FLASH_OBR             (STM32_FLASHIF_BASE + STM32_FLASH_OBR_OFFSET)
#define STM32_FLASH_WRPR            (STM32_FLASHIF_BASE + STM32_FLASH_WRPR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Flash Access Control Register (ACR) */

#define FLASH_ACR_LATENCY_SHIFT     (0)
#define FLASH_ACR_LATENCY_MASK      (7 << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY(n)      ((n) << FLASH_ACR_LATENCY_SHIFT)  /* n wait states */
#  define FLASH_ACR_LATENCY_0       (0 << FLASH_ACR_LATENCY_SHIFT)    /* 000: Zero wait states */
#  define FLASH_ACR_LATENCY_1       (1 << FLASH_ACR_LATENCY_SHIFT)    /* 001: One wait state */

#define FLASH_ACR_PRTFBE            (1 << 4)  /* Bit 4: FLASH prefetch enable */
#define FLASH_ACR_PRFTBS            (1 << 5)  /* Bit 5: FLASH Prefetch buffer status */

/* Flash Status Register (SR) */

#define FLASH_SR_BSY                (1 << 0)  /* Bit 0:  Busy */
#define FLASH_SR_PGERR              (1 << 2)  /* Bit 2:  Programming Error */
#define FLASH_SR_WRPRT_ERR          (1 << 4)  /* Bit 3:  Write Protection Error */
#define FLASH_SR_EOP                (1 << 5)  /* Bit 4:  End of Operation */

/* Flash Control Register (CR) */

#define FLASH_CR_PG                 (1 << 0)  /* Bit 0:  Program Page */
#define FLASH_CR_PER                (1 << 1)  /* Bit 1:  Page Erase */
#define FLASH_CR_MER                (1 << 2)  /* Bit 2:  Mass Erase */
#define FLASH_CR_OPTPG              (1 << 4)  /* Bit 4:  Option Byte Programming */
#define FLASH_CR_OPTER              (1 << 5)  /* Bit 5:  Option Byte Erase */
#define FLASH_CR_STRT               (1 << 6)  /* Bit 6:  Start Erase */
#define FLASH_CR_LOCK               (1 << 7)  /* Bit 7:  Page Locked or Lock Page */
#define FLASH_CR_OPTWRE             (1 << 9)  /* Bit 8:  Option Bytes Write Enable */
#define FLASH_CR_ERRIE              (1 << 10) /* Bit 10: Error Interrupt Enable */
#define FLASH_CR_EOPIE              (1 << 12) /* Bit 12: End of Program Interrupt Enable */
#define FLASH_CR_OBLLAUNCH          (1 << 13) /* Bit 13: Force option byte loading */

/* Flash Option byte register */

#define FLASH_OBR_                            /* To be provided */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32F0_FLASH_H */
