/****************************************************************************
 * arch/arm/src/efm32/efm32_lowputc.h
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_LOWPUTC_H
#define __ARCH_ARM_SRC_EFM32_EFM32_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "efm32_config.h"

/****************************************************************************
 * Public Functions Prototypes
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
void efm32_leuartconfigure(uintptr_t base, uint32_t baud,
                           unsigned int parity,
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
