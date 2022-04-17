/****************************************************************************
 * arch/arm/src/xmc4/xmc4_lowputc.h
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

#ifndef __ARCH_ARM_SRC_XMC4_XMC4_LOWPUTC_H
#define __ARCH_ARM_SRC_XMC4_XMC4_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "arm_internal.h"
#include "xmc4_config.h"
#include "xmc4_usic.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure provides the configuration of one UART channel */

struct uart_config_s
{
  uint32_t baud;         /* Desired BAUD rate */
  uint8_t  dx;           /* Input pin 0=DXA, 1=DXB, ... 6=DXG */
  uint8_t  parity;       /* Parity selection:  0=none, 1=odd, 2=even */
  uint8_t  nbits;        /* Number of bits per word */
  bool     stop2;        /* true=2 stop bits; false=1 stop bit */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void xmc4_lowsetup(void);

/****************************************************************************
 * Name: xmc4_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before xmc4_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in xmc4_lowsetup() and main clock initialization
 *   performed in xmc4_clock_configure().
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void xmc4_earlyserialinit(void);
#endif

/****************************************************************************
 * Name: xmc4_uart_configure
 *
 * Description:
 *   Enable and configure a USIC channel as a RS-232 UART.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
int xmc4_uart_configure(enum usic_channel_e channel,
                        const struct uart_config_s *config);
#endif

/****************************************************************************
 * Name: xmc4_uart_disable
 *
 * Description:
 *   Disable a USIC channel previously configured as a RS-232 UART.  it will
 *   be necessary to again call xmc4_uart_configure() in order to use this
 *   UART channel again.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
#define xmc4_uart_disable(c) xmc4_disable_usic_channel(c)
#endif

#endif /* __ARCH_ARM_SRC_XMC4_XMC4_LOWPUTC_H */
