/************************************************************************************
 * arch/arm/src/tiva/tiva_enableclks.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_ENABLECLKS_H
#define __ARCH_ARM_SRC_TIVA_TIVA_ENABLECLKS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "up_arch.h"
#include "chip.h"
#include "chip/tiva_syscontrol.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Clocks are enabled or disabled by setting or clearing a bit (b) in a system
 * control register (a))
 */

#define tiva_enableclk(a,b)       modifyreg32((a),0,(b))
#define tiva_disableclk(a,b)      modifyreg32((a),(b),0)

/* GPIO clocking */

#ifdef TIVA_SYSCON_RCGCGPIO
#  define tiva_gpio_enableclk(p)  tiva_enableclk(TIVA_SYSCON_RCGCGPIO,SYSCON_RCGCGPIO(p))
#  define tiva_gpio_disableclk(p) tiva_disableclk(TIVA_SYSCON_RCGCGPIO,SYSCON_RCGCGPIO(p))
#else
#  define tiva_gpio_enableclk(p)  tiva_enableclk(TIVA_SYSCON_RCGC2,SYSCON_RCGC2_GPIO(p))
#  define tiva_gpio_disableclk(p) tiva_disableclk(TIVA_SYSCON_RCGC2,SYSCON_RCGC2_GPIO(p))
#endif

/* UART clocking */

#ifdef TIVA_SYSCON_RCGCUART
#  define tiva_uart_enableclk(p)  tiva_enableclk(TIVA_SYSCON_RCGCUART,SYSCON_RCGCUART(p))
#  define tiva_uart_disableclk(p) tiva_disableclk(TIVA_SYSCON_RCGCUART,SYSCON_RCGCUART(p))

#  define tiva_uart0_enableclk()  tiva_uart_enableclk(0)
#  define tiva_uart1_enableclk()  tiva_uart_enableclk(1)
#  define tiva_uart2_enableclk()  tiva_uart_enableclk(2)
#  define tiva_uart3_enableclk()  tiva_uart_enableclk(3)
#  define tiva_uart4_enableclk()  tiva_uart_enableclk(4)
#  define tiva_uart5_enableclk()  tiva_uart_enableclk(5)
#  define tiva_uart6_enableclk()  tiva_uart_enableclk(6)
#  define tiva_uart7_enableclk()  tiva_uart_enableclk(7)

#  define tiva_uart0_disableclk() tiva_uart_disableclk(0)
#  define tiva_uart1_disableclk() tiva_uart_disableclk(1)
#  define tiva_uart2_disableclk() tiva_uart_disableclk(2)
#  define tiva_uart3_disableclk() tiva_uart_disableclk(3)
#  define tiva_uart4_disableclk() tiva_uart_disableclk(4)
#  define tiva_uart5_disableclk() tiva_uart_disableclk(5)
#  define tiva_uart6_disableclk() tiva_uart_disableclk(6)
#  define tiva_uart7_disableclk() tiva_uart_disableclk(7)
#else
#  define tiva_uart0_enableclk()  tiva_enableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_UART0)
#  define tiva_uart1_enableclk()  tiva_enableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_UART1)
#  define tiva_uart2_enableclk()  tiva_enableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_UART2)

#  define tiva_uart0_disableclk() tiva_disableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_UART0)
#  define tiva_uart1_disableclk() tiva_disableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_UART1)
#  define tiva_uart2_disableclk() tiva_disableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_UART2)
#endif

#endif /* __ARCH_ARM_SRC_TIVA_TIVA_ENABLECLKS_H */
