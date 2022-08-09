/****************************************************************************
 * arch/arm/src/nrf52/nrf52_lowputc.h
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
                           const struct uart_config_s *config);
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
                         const struct uart_config_s *config);
#endif

/****************************************************************************
 * Name: nrf52_usart_setformat
 *
 * Description:
 *   Set the USART line format and speed.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void nrf52_usart_setformat(uintptr_t base,
                           const struct uart_config_s *config);
#endif

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_LOWPUTC_H */
