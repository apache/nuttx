/****************************************************************************
 * arch/arm/src/xmc4/xmc4_lowputc.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_XMC4_XMC4_LOWPUTC_H
#define __ARCH_ARM_SRC_XMC4_XMC4_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "up_internal.h"
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
 *   configuration performed in xmc_lowsetup() and main clock iniialization
 *   performed in xmc_clock_configure().
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
                        FAR const struct uart_config_s *config);
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
