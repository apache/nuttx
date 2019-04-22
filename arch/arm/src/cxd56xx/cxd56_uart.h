/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_uart.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 *   Copyright (C) 2012, 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_UART_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "chip/cxd56_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization of the serial console.
 *
 ****************************************************************************/

void cxd56_lowsetup(void);

/****************************************************************************
 * Name: cxd56_uart_reset
 *
 * Description:
 *   Reset a U[S]ART.  These functions are used by the serial driver when a
 *   U[S]ART is closed.
 *
 ****************************************************************************/

void cxd56_uart_reset(int ch);

/****************************************************************************
 * Name: cxd56_uart_setup
 *
 * Description:
 *   Configure the UART.  This involves:
 *
 *   1. Connecting the input clock to the UART as specified in the
 *      board.h file,
 *   2. Configuring the UART pins
 *
 ****************************************************************************/

void cxd56_uart_setup(int);

/****************************************************************************
 * Name: cxd56_setbaud
 *
 * Description:
 *   Configure the U[S]ART divisors to accomplish the desired BAUD given the
 *   U[S]ART base frequency.
 *
 *   This computationally intensive algorithm is based on the same logic
 *   used in the NXP sample code.
 *
 ****************************************************************************/

void cxd56_setbaud(uintptr_t uartbase, uint32_t basefreq, uint32_t baud);

/****************************************************************************
 * Name: cxd56_uart_initialize
 *
 * Description:
 *   Various initial registration
 *
 ****************************************************************************/

int cxd56_uart_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_UART_H */
