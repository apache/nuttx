/****************************************************************************
 * arch/arm/src/efm32/efm32_lowputc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "efm32_lowputc.h"

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

void efm32_lowsetup(void)
{
#warning Missing logic
}

/*****************************************************************************
 * Name: efm32_lowputc
 *
 * Description:
 *   Output one character to the UART using a simple polling method.
 *
 *****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
void efm32_lowputc(uint32_t ch)
{
#warning Missing logic
}
#endif

/*****************************************************************************
 * Name: efm32_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 *****************************************************************************/

#ifdef HAVE_UART_DEVICE
void efm32_uartconfigure(uintptr_t uart_base, uint32_t baud, uint32_t clock,
                         unsigned int parity, unsigned int nbits)
{
#warning Missing logic
}
#endif
