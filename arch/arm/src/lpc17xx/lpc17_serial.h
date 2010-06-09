/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_serial.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_SERIAL_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_SERIAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include "lpc17_uart.h"
#include "lpc17_syscon.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration *********************************************************************/

/* Are any UARTs enabled? */

#undef HAVE_UART
#if defined(CONFIG_LPC17_UART0) || defined(CONFIG_LPC17_UART1) || \
    defined(CONFIG_LPC17_UART2) || defined(CONFIG_LPC17_UART3)
#  define HAVE_UART1
#endif

/* Is there a serial console? There should be at most one defined.  It could be on
 * any UARTn, n=0,1,2,3
 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_LPC17_UART0)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_LPC17_UART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_LPC17_UART2)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_LPC17_UART3)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#else
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef HAVE_CONSOLE
#endif

/* We cannot allow the DLM/DLL divisor to become to small or will will lose too
 * much accuracy.  This following is a "fudge factor" that represents the minimum
 * value of the divisor that we will permit.
 */

#define UART_MINDL 32

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
 * Name: lpc17_uartcclkdiv
 *
 * Descrption:
 *   Select a CCLK divider to produce the UART PCLK.  The stratey is to select the
 *   smallest divisor that results in an solution within range of the 16-bit
 *   DLM and DLL divisor:
 *
 *     PCLK = CCLK / divisor
 *     BAUD = PCLK / (16 * DL)
 *
 *   Ignoring the fractional divider for now.
 *
 *   NOTE:  This is an inline function.  If a typical optimization level is used and
 *   a constant is provided for the desired frequency, then most of the following
 *   logic will be optimized away.
 *
 ************************************************************************************/

static inline uint8_t lpc17_uartcclkdiv(uint32_t baud)
{
  /* Ignoring the fractional divider, the BAUD is given by:
   *
   *   BAUD = PCLK / (16 * DL), or
   *   DL   = PCLK / BAUD / 16
   *
   * Where:
   *
   *   PCLK = CCLK / divisor.
   *
   * Check divisor == 1.  This works if the upper limit is met	
   *
   *   DL < 0xffff, or
   *   PCLK / BAUD / 16 < 0xffff, or
   *   CCLK / BAUD / 16 < 0xffff, or
   *   CCLK < BAUD * 0xffff * 16
   *   BAUD > CCLK / 0xffff / 16
   *
   * And the lower limit is met (we can't allow DL to get very close to one).
   *
   *   DL >= MinDL
   *   CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 16 / MinDL
   */

  if (baud < (LPC17_CCLK / 16 / UART_MINDL ))
    {
      return SYSCON_PCLKSEL_CCLK;
    }
   
  /* Check divisor == 2.  This works if:
   *
   *   2 * CCLK / BAUD / 16 < 0xffff, or
   *   BAUD > CCLK / 0xffff / 8
   *
   * And
   *
   *   2 * CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 8 / MinDL
   */

  else if (baud < (LPC17_CCLK / 8 / UART_MINDL ))
    {
      return SYSCON_PCLKSEL_CCLK2;
    }

  /* Check divisor == 4.  This works if:
   *
   *   4 * CCLK / BAUD / 16 < 0xffff, or
   *   BAUD > CCLK / 0xffff / 4
   *
   * And
   *
   *   4 * CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 4 / MinDL 
   */

  else if (baud < (LPC17_CCLK / 4 / UART_MINDL ))
    {
      return SYSCON_PCLKSEL_CCLK4;
    }

  /* Check divisor == 8.  This works if:
   *
   *   8 * CCLK / BAUD / 16 < 0xffff, or
   *   BAUD > CCLK / 0xffff / 2
    *
   * And
   *
   *   8 * CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 2 / MinDL 
  */

  else /* if (baud < (LPC17_CCLK / 2 / UART_MINDL )) */
    {
      return SYSCON_PCLKSEL_CCLK8;
    }
}

/************************************************************************************
 * Name: lpc17_uartdl
 *
 * Descrption:
 *   Select a divider to produce the BAUD from the UART PCLK.
 *
 *     BAUD = PCLK / (16 * DL), or
 *     DL   = PCLK / BAUD / 16
 *
 *   Ignoring the fractional divider for now.
 *
 ************************************************************************************/

static inline uint32_t lpc17_uartdl(uint32_t baud, uint8_t divcode)
{
  uint32_t num;

  switch (divcode)
    {

    case SYSCON_PCLKSEL_CCLK4:   /* PCLK_peripheral = CCLK/4 */
      num = (LPC17_CCLK / 4);
      break;

    case SYSCON_PCLKSEL_CCLK:    /* PCLK_peripheral = CCLK */
      num = LPC17_CCLK;
      break;

    case SYSCON_PCLKSEL_CCLK2:   /* PCLK_peripheral = CCLK/2 */
      num = (LPC17_CCLK / 2);
      break;

    case SYSCON_PCLKSEL_CCLK8:   /* PCLK_peripheral = CCLK/8 (except CAN1, CAN2, and CAN) */
    default:
      num = (LPC17_CCLK / 8);
      break;
    }
  return num / (baud << 4);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_SERIAL_H */
