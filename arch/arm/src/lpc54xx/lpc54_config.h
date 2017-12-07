/************************************************************************************
 * arch/arm/src/lpc54xx/lpc54_config.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC54_CONFIG_H
#define __ARCH_ARM_SRC_LPC54XX_LPC54_CONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration *********************************************************************/
/* Make sure that no unsupported UARTs are enabled */

#ifndef CONFIG_LPC54_FLEXCOMM0
#  undef CONFIG_LPC54_USART0
#endif
#ifndef CONFIG_LPC54_FLEXCOMM1
#  undef CONFIG_LPC54_USART1
#endif
#ifndef CONFIG_LPC54_FLEXCOMM2
#  undef CONFIG_LPC54_USART2
#endif
#ifndef CONFIG_LPC54_FLEXCOMM3
#  undef CONFIG_LPC54_USART3
#endif
#ifndef CONFIG_LPC54_FLEXCOMM4
#  undef CONFIG_LPC54_USART4
#endif
#ifndef CONFIG_LPC54_FLEXCOMM5
#  undef CONFIG_LPC54_USART5
#endif
#ifndef CONFIG_LPC54_FLEXCOMM6
#  undef CONFIG_LPC54_USART6
#endif
#ifndef CONFIG_LPC54_FLEXCOMM7
#  undef CONFIG_LPC54_USART7
#endif
#ifndef CONFIG_LPC54_FLEXCOMM8
#  undef CONFIG_LPC54_USART8
#endif
#ifndef CONFIG_LPC54_FLEXCOMM9
#  undef CONFIG_LPC54_USART9
#endif

/* Map logical UART names (Just for simplicity of naming) */

#undef HAVE_USART0
#undef HAVE_USART1
#undef HAVE_USART2
#undef HAVE_USART3
#undef HAVE_USART4
#undef HAVE_USART5
#undef HAVE_USART6
#undef HAVE_USART7
#undef HAVE_USART8
#undef HAVE_USART9

#ifdef CONFIG_LPC54_USART0
#  define HAVE_USART0
#endif
#ifdef CONFIG_LPC54_USART1
#  define HAVE_USART1
#endif
#ifdef CONFIG_LPC54_USART2
#  define HAVE_USART2
#endif
#ifdef CONFIG_LPC54_USART3
#  define HAVE_USART3
#endif
#ifdef CONFIG_LPC54_USART4
#  define HAVE_USART4
#endif
#ifdef CONFIG_LPC54_USART5
#  define HAVE_USART5
#endif
#ifdef CONFIG_LPC54_USART6
#  define HAVE_USART6
#endif
#ifdef CONFIG_LPC54_USART7
#  define HAVE_USART7
#endif
#ifdef CONFIG_LPC54_USART8
#  define HAVE_USART8
#endif
#ifdef CONFIG_LPC54_USART9
#  define HAVE_USART9
#endif

/* Are any UARTs enabled? */

#undef HAVE_USART_DEVICE
#if defined(HAVE_USART0) || defined(HAVE_USART1) || defined(HAVE_USART2) || \
    defined(HAVE_USART3) || defined(HAVE_USART4) || defined(HAVE_USART5) || \
    defined(HAVE_USART6) || defined(HAVE_USART7) || defined(HAVE_USART8) || \
    defined(HAVE_USART9)
#  define HAVE_USART_DEVICE 1
#endif

/* Is there a serial console? There should be at most one defined.  It could be on
 * any UARTn, n=0,1,2,3,4,5
 */

#undef HAVE_USART_CONSOLE

#if defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(HAVE_USART0)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  undef CONFIG_USART8_SERIAL_CONSOLE
#  undef CONFIG_USART9_SERIAL_CONSOLE
#  define HAVE_USART_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(HAVE_USART1)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  undef CONFIG_USART8_SERIAL_CONSOLE
#  undef CONFIG_USART9_SERIAL_CONSOLE
#  define HAVE_USART_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(HAVE_USART2)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  undef CONFIG_USART8_SERIAL_CONSOLE
#  undef CONFIG_USART9_SERIAL_CONSOLE
#  define HAVE_USART_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && defined(HAVE_USART3)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  undef CONFIG_USART8_SERIAL_CONSOLE
#  undef CONFIG_USART9_SERIAL_CONSOLE
#  define HAVE_USART_CONSOLE 1
#elif defined(CONFIG_USART4_SERIAL_CONSOLE) && defined(HAVE_USART4)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  undef CONFIG_USART8_SERIAL_CONSOLE
#  undef CONFIG_USART9_SERIAL_CONSOLE
#  define HAVE_USART_CONSOLE 1
#elif defined(CONFIG_USART5_SERIAL_CONSOLE) && defined(HAVE_USART5)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  undef CONFIG_USART8_SERIAL_CONSOLE
#  undef CONFIG_USART9_SERIAL_CONSOLE
#  define HAVE_USART_CONSOLE 1
#elif defined(CONFIG_USART6_SERIAL_CONSOLE) && defined(HAVE_USART6)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  undef CONFIG_USART8_SERIAL_CONSOLE
#  undef CONFIG_USART9_SERIAL_CONSOLE
#  define HAVE_USART_CONSOLE 1
#elif defined(CONFIG_USART7_SERIAL_CONSOLE) && defined(HAVE_USART7)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART8_SERIAL_CONSOLE
#  undef CONFIG_USART9_SERIAL_CONSOLE
#  define HAVE_USART_CONSOLE 1
#elif defined(CONFIG_USART8_SERIAL_CONSOLE) && defined(HAVE_USART8)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  undef CONFIG_USART9_SERIAL_CONSOLE
#  define HAVE_USART_CONSOLE 1
#elif defined(CONFIG_USART9_SERIAL_CONSOLE) && defined(HAVE_USART9)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  undef CONFIG_USART8_SERIAL_CONSOLE
#  define HAVE_USART_CONSOLE 1
#else
#  ifdef CONFIG_DEV_CONSOLE
#    warning "No valid CONFIG_[LP]UART[n]_SERIAL_CONSOLE Setting"
#  endif
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_USART7_SERIAL_CONSOLE
#  undef CONFIG_USART8_SERIAL_CONSOLE
#  undef CONFIG_USART9_SERIAL_CONSOLE
#endif

/* Check UART flow control (Not yet supported) */

# undef CONFIG_USART0_FLOWCONTROL
# undef CONFIG_USART1_FLOWCONTROL
# undef CONFIG_USART2_FLOWCONTROL
# undef CONFIG_USART3_FLOWCONTROL
# undef CONFIG_USART4_FLOWCONTROL
# undef CONFIG_USART5_FLOWCONTROL
# undef CONFIG_USART6_FLOWCONTROL
# undef CONFIG_USART7_FLOWCONTROL
# undef CONFIG_USART8_FLOWCONTROL
# undef CONFIG_USART9_FLOWCONTROL

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC54XX_LPC54_CONFIG_H */
