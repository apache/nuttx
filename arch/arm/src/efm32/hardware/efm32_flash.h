/************************************************************************************
 * arch/arm/src/efm32/chip/efm32_flash.h
 *
 *   Copyright (C) 2015 Bouteville Pierre-Noel. All rights reserved.
 *   Author: Bouteville Pierre-Noel <pnb990@gmail.com>
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

#ifndef __ARCH_ARM_SRC_EFM32_CHIP_EFM32_FLASH_H
#define __ARCH_ARM_SRC_EFM32_CHIP_EFM32_FLASH_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if defined(CONFIG_EFM32_EFM32GG)
#   define EFM32_FLASH_PAGESIZE    4096
#elif defined(CONFIG_EFM32_EFM32LG)
#   define EFM32_FLASH_PAGESIZE    2048
#elif defined(CONFIG_EFM32_EFM32WG)
#   define EFM32_FLASH_PAGESIZE    2048
#elif defined(CONFIG_EFM32_EFM32ZG)
#   define EFM32_FLASH_PAGESIZE    1024
#elif defined(CONFIG_EFM32_EFM32G)
#   define EFM32_FLASH_PAGESIZE    512
#elif defined(CONFIG_EFM32_EFM32TG)
#   define EFM32_FLASH_PAGESIZE    512
#endif

#endif /* __ARCH_ARM_SRC_EFM32_CHIP_EFM32_FLASH_H */
