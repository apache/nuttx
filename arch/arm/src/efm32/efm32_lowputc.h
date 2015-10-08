/****************************************************************************
 * arch/arm/src/efm32/efm32_lowputc.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_LOWPUTC_H
#define __ARCH_ARM_SRC_EFM32_EFM32_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "efm32_config.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization including setup of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot sequence.
 *
 ****************************************************************************/

void efm32_lowsetup(void);

/****************************************************************************
 * Name: efm32_lowputc
 *
 * Description:
 *   Output one character to the UART using a simple polling method.
 *
 ****************************************************************************/

#if defined(HAVE_UART_CONSOLE) || defined(HAVE_LEUART_CONSOLE)
void efm32_lowputc(uint32_t ch);
#endif

/****************************************************************************
 * Name: efm32_uartconfigure
 *
 * Description:
 *   Configure a U[S]ART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void efm32_uartconfigure(uintptr_t base, uint32_t baud, unsigned int parity,
                         unsigned int nbits, bool stop2);
#endif

/****************************************************************************
 * Name: efm32_leuartconfigure
 *
 * Description:
 *   Configure a LEUART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_LEUART_DEVICE
void efm32_leuartconfigure(uintptr_t base, uint32_t baud, unsigned int parity,
                           unsigned int nbits, bool stop2);
#endif

/****************************************************************************
 * Name: efm32_uart_reset
 *
 * Description:
 *   Reset the USART/UART by disabling it and restoring all of the registers
 *   to the initial, reset value.  Only the ROUTE data set by efm32_lowsetup
 *   is preserved.
 *
 ****************************************************************************/

#if defined(HAVE_UART_DEVICE) || defined(HAVE_SPI_DEVICE)
void efm32_uart_reset(uintptr_t base);
#endif

/****************************************************************************
 * Name: efm32_uart_reset
 *
 * Description:
 *   Reset the USART/UART by disabling it and restoring all of the registers
 *   to the initial, reset value.  Only the ROUTE data set by efm32_lowsetup
 *   is preserved.
 *
 ****************************************************************************/

#ifdef HAVE_LEUART_DEVICE
void efm32_leuart_reset(uintptr_t base);
#endif

#endif /* __ARCH_ARM_SRC_EFM32_EFM32_LOWPUTC_H */
