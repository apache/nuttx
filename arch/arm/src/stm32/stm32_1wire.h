/************************************************************************************
 * arch/arm/src/stm32/stm32_1wire.h
 *
 *   Copyright (C) 2016 Aleksandr Vyhovanec. All rights reserved.
 *   Author: Aleksandr Vyhovanec <www.desh@gmail.com>
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_1WIRE_H
#define __ARCH_ARM_SRC_STM32_STM32_1WIRE_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(CONFIG_STM32_STM32L15XX)
#  include "chip/stm32l15xxx_uart.h"
#elif defined(CONFIG_STM32_STM32F10XX)
#  include "chip/stm32f10xxx_uart.h"
#elif defined(CONFIG_STM32_STM32F20XX)
#  include "chip/stm32f20xxx_uart.h"
#elif defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
#  include "chip/stm32f30xxx_uart.h"
#elif defined(CONFIG_STM32_STM32F40XX)
#  include "chip/stm32f40xxx_uart.h"
#else
#  error "Unsupported STM32 UART"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Check 1-Wire and U(S)ART conflicting */

#if defined(CONFIG_STM32_1WIRE1) && defined(CONFIG_STM32_USART1)
#  undef CONFIG_STM32_1WIRE1
#endif
#if defined(CONFIG_STM32_1WIRE2) && defined(CONFIG_STM32_USART2)
#  undef CONFIG_STM32_1WIRE2
#endif
#if defined(CONFIG_STM32_1WIRE3) && defined(CONFIG_STM32_USART3)
#  undef CONFIG_STM32_1WIRE3
#endif
#if defined(CONFIG_STM32_1WIRE4) && defined(CONFIG_STM32_UART4)
#  undef CONFIG_STM32_1WIRE4
#endif
#if defined(CONFIG_STM32_1WIRE5) && defined(CONFIG_STM32_UART5)
#  undef CONFIG_STM32_1WIRE5
#endif
#if defined(CONFIG_STM32_1WIRE6) && defined(CONFIG_STM32_USART6)
#  undef CONFIG_STM32_1WIRE6
#endif
#if defined(CONFIG_STM32_1WIRE7) && defined(CONFIG_STM32_UART7)
#  undef CONFIG_STM32_1WIRE7
#endif
#if defined(CONFIG_STM32_1WIRE8) && defined(CONFIG_STM32_UART8)
#  undef CONFIG_STM32_1WIRE8
#endif

/* Is there a 1-Wire enabled? */

#if defined(CONFIG_STM32_1WIRE1) || defined(CONFIG_STM32_1WIRE2) || \
    defined(CONFIG_STM32_1WIRE3) || defined(CONFIG_STM32_1WIRE4) || \
    defined(CONFIG_STM32_1WIRE5) || defined(CONFIG_STM32_1WIRE6) || \
    defined(CONFIG_STM32_1WIRE7) || defined(CONFIG_STM32_1WIRE8)
#  define HAVE_1WIRE 1
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_STM32_1WIRE_H */
