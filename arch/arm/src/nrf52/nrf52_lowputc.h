/****************************************************************************
 * arch/arm/src/nrf52/nrf52_lowputc.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_LOWPUTC_H
#define __ARCH_ARM_SRC_NRF52_NRF52_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <nrf52_gpio.h>
#include <nrf52_config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
/* This structure describes the configuration of an UART */

struct uart_config_s
{
  uint32_t baud;          /* Configured baud */
  uint8_t  parity;        /* 0=none, 1=odd, 2=even */
  uint8_t  bits;          /* Number of bits (5-9) */
  bool     stopbits2;     /* Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool     iflow;         /* Input flow control supported */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool     oflow;         /* Output flow control supported. */
#endif
  nrf52_pinset_t txpin;   /* TX pin */
  nrf52_pinset_t rxpin;   /* RX pin */
};
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start. Performs low level
 *   initialization including setup of the console UART. This UART
 *   initialization is done early so that the serial console is available
 *   for debugging very early in the boot sequence.
 *
 ****************************************************************************/

void nrf52_lowsetup(void);

/****************************************************************************
 * Name: nrf52_usart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void nrf52_usart_configure(uintptr_t base,
                           FAR const struct uart_config_s *config);
#endif

/****************************************************************************
 * Name: nrf52_usart_disable
 *
 * Description:
 *   Disable a UART.  it will be necessary to again call
 *   nrf52_usart_configure() in order to use this UART channel again.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void nrf52_usart_disable(uintptr_t base,
                         FAR const struct uart_config_s *config);
#endif

/****************************************************************************
 * Name: nrf52_usart_setformat
 *
 * Description:
 *   Set the USART line format and speed.
 *
 ****************************************************************************/

void nrf52_usart_setformat(uintptr_t base,
                           FAR const struct uart_config_s *config);

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_LOWPUTC_H */
