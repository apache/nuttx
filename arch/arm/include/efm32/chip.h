/************************************************************************************
 * arch/arm/include/efm32/chip.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_EFM32_CHIP_H
#define __ARCH_ARM_INCLUDE_EFM32_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* EFM32 EnergyMicro ****************************************************************/

/* Tiny Gecko with 32KiB FLASH and 4KiB RAM in a QFN64 package */

#if defined(CONFIG_ARCH_CHIP_EFM32TG840F32)

/* Gecko with 128KiB FLASH and 16KiB SRAM in LQFP100 (EFM32G880F128) or BGA112
 * (EFM32G890F128) package
 */

#elif defined(CONFIG_ARCH_CHIP_EFM32G880F128) || \
      defined(CONFIG_ARCH_CHIP_EFM32G890F128)

/* Giant Gecko with 1024KiB FLASH and 128KiB RAM in a QFP64 package
 * (EFM32GG332F1024) or BGA112 (EFM32GG990F1024) package
 */

#elif defined(CONFIG_ARCH_CHIP_EFM32GG332F1024) || \
      defined(CONFIG_ARCH_CHIP_EFM32GG990F1024)

#else
#  error "Unsupported EFM32 chip"
#endif

/* NVIC priority levels *************************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xe0 /* Bits [7:5] set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x20 /* Three bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_EFM32_CHIP_H */
