/****************************************************************************
 * arch/arm/src/max326xx/max326_lowputc.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX326_LOWPUTC_H
#define __ARCH_ARM_SRC_MAX326XX_MAX326_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "max326_config.h"

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
  uint8_t  txlevel;       /* TX level for event generation */
  uint8_t  rxlevel;       /* RX level for event generation */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool     rtslevel;      /* RX level for RTS generation */
#endif
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
 * Name: max326_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization including setup of the console UART.  This UART
 *   initialization is done early so that the serial console is available
 *   for debugging very early in the boot sequence.
 *
 ****************************************************************************/

void max326_lowsetup(void);

/****************************************************************************
 * Name: max326_uart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void max326_uart_configure(uintptr_t base,
                           const struct uart_config_s *config);
#endif

/****************************************************************************
 * Name: max326_uart_disable
 *
 * Description:
 *   Disable a UART.  it will be necessary to again call
 *   max326_uart_configure() in order to use this UART channel again.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void max326_uart_disable(uintptr_t base);
#endif

#endif /* __ARCH_ARM_SRC_MAX326XX_MAX326_LOWPUTC_H */
