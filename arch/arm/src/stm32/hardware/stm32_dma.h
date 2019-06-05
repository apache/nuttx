/************************************************************************************
 * arch/arm/src/stm32/hardware/stm32_dma.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DMA_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DMA_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* Include the correct DMA register definitions for selected STM32 DMA IP core:
 *   - STM32 DMA IP version 1 - F0, F1, F3, L0, L1, L4
 *   - STM32 DMA IP version 2 - F2, F4, F7, H7
 */

#if defined(CONFIG_STM32_HAVE_IP_DMA_V1) && defined(CONFIG_STM32_HAVE_IP_DMA_V2)
#  error Only one STM32 DMA IP version must be selected
#endif

#if defined(CONFIG_STM32_HAVE_IP_DMA_V1)
#  include "stm32_dma_v1.h"
#elif defined(CONFIG_STM32_HAVE_IP_DMA_V2)
#  include "stm32_dma_v2.h"
#else
#  error "STM32 DMA IP version not specified"
#endif

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DMA_H */
