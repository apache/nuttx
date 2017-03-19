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
#include "xmc4_config.h"

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
 * Name: xmc4_uart_reset
 *
 * Description:
 *   Reset a UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void xmc4_uart_reset(uintptr_t uart_base);
#endif

/****************************************************************************
 * Name: xmc4_uart_configure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void xmc4_uart_configure(uintptr_t uart_base, uint32_t baud,
                         uint32_t clock, unsigned int parity,
                         unsigned int nbits, unsigned int stop2);
#endif


#endif /* __ARCH_ARM_SRC_XMC4_XMC4_LOWPUTC_H */
