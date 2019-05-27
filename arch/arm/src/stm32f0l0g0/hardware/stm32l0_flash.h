/************************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32l0_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32L0_FLASH_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32L0_FLASH_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_FLASH_ACR_OFFSET      0x0000
#define STM32_FLASH_PECR_OFFSET     0x0004
#define STM32_FLASH_PDKEYR_OFFSET   0x0008
#define STM32_FLASH_PEKEYR_OFFSET   0x000c
#define STM32_FLASH_PRGKEYR_OFFSET  0x0010
#define STM32_FLASH_OPTKEYR_OFFSET  0x0014
#define STM32_FLASH_SR_OFFSET       0x0018
#define STM32_FLASH_OBR_OFFSET      0x001c
#define STM32_FLASH_WRPR1_OFFSET    0x0020
#define STM32_FLASH_WRPR2_OFFSET    0x0080
#define STM32_FLASH_WRPR3_OFFSET    0x0084
#define STM32_FLASH_WRPR4_OFFSET    0x0088

/* Register Addresses ***************************************************************/

#define STM32_FLASH_ACR             (STM32_FLASHIF_BASE + STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_PECR            (STM32_FLASHIF_BASE + STM32_FLASH_PECR_OFFSET)
#define STM32_FLASH_PDKEYR          (STM32_FLASHIF_BASE + STM32_FLASH_PDKEYR_OFFSET)
#define STM32_FLASH_PEKEYR          (STM32_FLASHIF_BASE + STM32_FLASH_PEKEYR_OFFSET)
#define STM32_FLASH_PRGKEYR         (STM32_FLASHIF_BASE + STM32_FLASH_PRGKEYR_OFFSET)
#define STM32_FLASH_OPTKEYR         (STM32_FLASHIF_BASE + STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_SR              (STM32_FLASHIF_BASE + STM32_FLASH_SR_OFFSET)
#define STM32_FLASH_OBR             (STM32_FLASHIF_BASE + STM32_FLASH_OBR_OFFSET)
#define STM32_FLASH_WRPR1           (STM32_FLASHIF_BASE + STM32_FLASH_WRPR1_OFFSET)
#define STM32_FLASH_WRPR2           (STM32_FLASHIF_BASE + STM32_FLASH_WRPR2_OFFSET)
#define STM32_FLASH_WRPR3           (STM32_FLASHIF_BASE + STM32_FLASH_WRPR3_OFFSET)
#define STM32_FLASH_WRPR4           (STM32_FLASHIF_BASE + STM32_FLASH_WRPR4_OFFSET)

/* Register Bitfield Definitions ****************************************************/
/* Flash Access Control Register (ACR) */

#define FLASH_ACR_LATENCY           (1 << 0)  /* Bit 0: Latency */
#define FLASH_ACR_PRFTEN            (1 << 1)  /* Bit 1: Prefetch enable */
#define FLASH_ACR_ACC64             (1 << 2)  /* Bit 2: 64-bit access */
#define FLASH_ACR_SLEEP_PD          (1 << 3)  /* Bit 3: Flash mode during Sleep */
#define FLASH_ACR_RUN_PD            (1 << 4)  /* Bit 4: Flash mode during Run */

/* Program/Erase Control Register (PECR) */

#define FLASH_PECR_PELOCK           (1 << 0)  /* Bit 0: PECR and data EEPROM lock */
#define FLASH_PECR_PRGLOCK          (1 << 1)  /* Bit 1: Program memory lock */
#define FLASH_PECR_OPTLOCK          (1 << 2)  /* Bit 2: Option bytes block lock */
#define FLASH_PECR_PROG             (1 << 3)  /* Bit 3: Program memory selection */
#define FLASH_PECR_DATA             (1 << 4)  /* Bit 4: Data EEPROM selection */
#define FLASH_PECR_FTDW             (1 << 8)  /* Bit 8: Fixed time data write for Byte, Half Word and Word programming */
#define FLASH_PECR_ERASE            (1 << 9)  /* Bit 9: Page or Double Word erase mode */
#define FLASH_PECR_FPRG             (1 << 10) /* Bit 10: Half Page/Double Word programming mode */
#define FLASH_PECR_PARALLBANK       (1 << 15) /* Bit 15: Parallel bank mode */
#define FLASH_PECR_EOPIE            (1 << 16) /* Bit 16: End of programming interrupt enable */
#define FLASH_PECR_ERRIE            (1 << 17) /* Bit 17: Error interrupt enable */
#define FLASH_PECR_OBL_LAUNCH       (1 << 18) /* Bit 18: Launch the option byte loading */

/* Flash Status Register (SR) */

#define FLASH_SR_BSY                (1 << 0)  /* Bit 0: Busy */
#define FLASH_SR_EOP                (1 << 1)  /* Bit 1: End of operation */
#define FLASH_SR_ENDHV              (1 << 2)  /* Bit 2: End of high voltage */
#define FLASH_SR_READY              (1 << 3)  /* Bit 3: Flash memory module ready after low power mode */
#define FLASH_SR_WRPERR             (1 << 8)  /* Bit 8: Write protection error */
#define FLASH_SR_PGAERR             (1 << 9)  /* Bit 9: Programming alignment error  */
#define FLASH_SR_SIZERR             (1 << 10) /* Bit 10: Size error */
#define FLASH_SR_OPTVERR            (1 << 11) /* Bit 11: Option validity error */
#define FLASH_SR_OPTVERRUSR         (1 << 12) /* Bit 12: Option UserValidity Error */
#define FLASH_SR_RDERR              (1 << 13) /* Bit 13: Read protected error */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32L0_FLASH_H */
