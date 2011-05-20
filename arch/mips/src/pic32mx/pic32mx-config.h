/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-config.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************/

#ifndef __ARCH_MIPS_SRC_LPC17XX_LPC17_PIC32_H
#define __ARCH_MIPS_SRC_LPC17XX_LPC17_PIC32_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include "chip.h"
#include "pic32mx-memorymap.h"
#include "pic32mx-uart.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration *********************************************************************/
/* Don't enable UARTs not supported by the chip. */

#if CHIP_NEUARTS < 1
#  undef CONFIG_PIC32MX_UART1
#  undef CONFIG_PIC32MX_UART2
#endif
#if CHIP_NEUARTS < 2
#  undef CONFIG_PIC32MX_UART2
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART_DEVICE
#if defined(CONFIG_PIC32MX_UART1) || defined(CONFIG_LPC17_UART1)
#  define HAVE_UART_DEVICE 1
#endif

/* Is there a serial console?  There should be at most one defined.  It
 * could be on any UARTn, n=0,1
 */

#if defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_PIC32MX_UART1)
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_LPC17_UART2)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_MIPS_SRC_LPC17XX_LPC17_PIC32_H */
