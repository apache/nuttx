/************************************************************************************
 * arch/arm/src/stm32/chip/stm32_flash.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_FLASH_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_FLASH_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
 
#ifdef CONFIG_STM32_LOWDENSITY
#  define STM32_FLASH_NPAGES        32
#  define STM32_FLASH_PAGESIZE      1024
#elif  CONFIG_STM32_MEDIUMDENSITY
#  define STM32_FLASH_NPAGES        128
#  define STM32_FLASH_PAGESIZE      1024
#elif  CONFIG_STM32_CONNECTIVITYLINE
#  define STM32_FLASH_NPAGES        128
#  define STM32_FLASH_PAGESIZE      2048
#elif CONFIG_STM32_HIGHDENSITY
#  define STM32_FLASH_NPAGES        256
#  define STM32_FLASH_PAGESIZE      2048
#endif

#define STM32_FLASH_SIZE            (STM32_FLASH_NPAGES * STM32_FLASH_PAGESIZE)

/* Register Offsets *****************************************************************/

#define STM32_FLASH_ACR_OFFSET     0x0000
#define STM32_FLASH_KEYR_OFFSET    0x0004
#define STM32_FLASH_OPTKEYR_OFFSET 0x0008
#define STM32_FLASH_SR_OFFSET      0x000c
#define STM32_FLASH_CR_OFFSET      0x0010
#define STM32_FLASH_AR_OFFSET      0x0014
#define STM32_FLASH_OBR_OFFSET     0x001c
#define STM32_FLASH_WRPR_OFFSET    0x0020

/* Register Addresses ***************************************************************/

#define STM32_FLASH_ACR            (STM32_FLASHIF_BASE+STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_KEYR           (STM32_FLASHIF_BASE+STM32_FLASH_KEYR_OFFSET)
#define STM32_FLASH_OPTKEYR        (STM32_FLASHIF_BASE+STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_SR             (STM32_FLASHIF_BASE+STM32_FLASH_SR_OFFSET)
#define STM32_FLASH_CR             (STM32_FLASHIF_BASE+STM32_FLASH_CR_OFFSET)
#define STM32_FLASH_AR             (STM32_FLASHIF_BASE+STM32_FLASH_AR_OFFSET)
#define STM32_FLASH_OBR            (STM32_FLASHIF_BASE+STM32_FLASH_OBR_OFFSET)
#define STM32_FLASH_WRPR           (STM32_FLASHIF_BASE+STM32_FLASH_WRPR_OFFSET)

/* Register Bitfield Definitions ****************************************************/
/* TODO: Complete FLASH details from the STM32F10xxx Flash programming manual. */

/* Flash Status Register (SR) */

#define FLASH_SR_BSY                (1 << 0)                /* Busy */
#define FLASH_SR_PGERR              (1 << 2)                /* Programming Error */
#define FLASH_SR_WRPRT_ERR          (1 << 4)                /* Write Protection Error */
#define FLASH_SR_EOP                (1 << 5)                /* End of Operation */

/* Flash Control Register (CR) */

#define FLASH_CR_PG                 (1 << 0)                /* Program Page */
#define FLASH_CR_PER                (1 << 1)                /* Page Erase */
#define FLASH_CR_MER                (1 << 2)                /* Mass Erase */
#define FLASH_CR_OPTPG              (1 << 4)                /* Option Byte Programming */
#define FLASH_CR_OPTER              (1 << 5)                /* Option Byte Erase */
#define FLASH_CR_STRT               (1 << 6)                /* Start Erase */
#define FLASH_CR_LOCK               (1 << 7)                /* Page Locked or Lock Page */
#define FLASH_CR_OPTWRE             (1 << 9)                /* Option Bytes Write Enable */
#define FLASH_CR_ERRIE              (1 << 10)               /* Error Interrupt Enable */
#define FLASH_CR_EOPIE              (1 << 12)               /* End of Program Interrupt Enable */

/* Flash Access Control Register (ACR) */

#define FLASH_ACR_LATENCY_SHIFT     (0)
#define FLASH_ACR_LATENCY_MASK      (7 << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY_0       (0 << FLASH_ACR_LATENCY_SHIFT)    /* FLASH Zero Latency cycle */
#  define FLASH_ACR_LATENCY_1       (1 << FLASH_ACR_LATENCY_SHIFT)    /* FLASH One Latency cycle */
#  define FLASH_ACR_LATENCY_2       (2 << FLASH_ACR_LATENCY_SHIFT)    /* FLASH Two Latency cycles */
#define FLASH_ACR_HLFCYA            (1 << 3)                    /* FLASH half cycle access */
#define FLASH_ACR_PRTFBE            (1 << 4)                    /* FLASH prefetch enable */

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_FLASH_H */

