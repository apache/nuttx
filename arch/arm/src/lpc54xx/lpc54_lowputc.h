/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_lowputc.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC54_LOWPUTC_H
#define __ARCH_ARM_SRC_LPC54XX_LPC54_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef HAVE_USART_DEVICE
/* This structure describes the configuration of an UART */

struct uart_config_s
{
  uint32_t baud;          /* Configured baud */
  uint32_t fclk;          /* Input Flexcomm function clock frequency */
  uint8_t  parity;        /* 0=none, 1=odd, 2=even */
  uint8_t  bits;          /* Number of bits (5-9) */
  uint8_t  txlevel;       /* TX level for event generation */
  uint8_t  rxlevel;       /* RX level for event generation */
  bool     stopbits2;     /* true: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool     iflow;         /* true: Input flow control supported */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool     oflow;         /* true: Output flow control supported. */
#endif
};
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.
 *   Performs low level initialization including setup of the console USART.
 *   This USART initialization is done early so that the serial console is
 *   available for debugging very early in the boot sequence.
 *
 ****************************************************************************/

void lpc54_lowsetup(void);

/****************************************************************************
 * Name: lpc54_usart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ****************************************************************************/

#ifdef HAVE_USART_DEVICE
void lpc54_usart_configure(uintptr_t base,
                           const struct uart_config_s *config);
#endif

/****************************************************************************
 * Name: lpc54_usart_disable
 *
 * Description:
 *   Disable a USART.  it will be necessary to again call
 *   lpc54_usart_configure() in order to use this USART channel again.
 *
 ****************************************************************************/

#ifdef HAVE_USART_DEVICE
void lpc54_usart_disable(uintptr_t base);
#endif

#endif /* __ARCH_ARM_SRC_LPC54XX_LPC54_LOWPUTC_H */
