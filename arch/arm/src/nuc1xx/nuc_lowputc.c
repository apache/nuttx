/****************************************************************************
 * arch/arm/src/nuc1xx/nuc_lowputc.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "chip.h"
#include "nuc_config.h"
#include "chip/chip/nuc_clk.h"
#include "chip/chip/nuc_uart.h"

#include "nuc_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Here we assume that the default clock source for the UART modules is
 * the external high speed crystal.
 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nuc_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level initialization.
 *
 *****************************************************************************/

void nuc_lowsetup(void)
{
#ifdef HAVE_UART
  uint32_t regval;

  /* Configure the UART clock source 
   *
   * Here we assume that the UART clock source is the external high speed
   * crystal -- the power on default value in the CLKSEL0 register.
   */

  /* Enable UART clocking for the selected UARTs */

  regval = getreg32(NUC_CLK_APBCLK);
  regval &= ~(CLK_APBCLK_UART0_EN | CLK_APBCLK_UART1_EN | CLK_APBCLK_UART2_EN);

#ifdef CONFIG_NUC_UART0
  regval |= CLK_APBCLK_UART0_EN;
#endif
#ifdef CONFIG_NUC_UART1
  regval |= CLK_APBCLK_UART1_EN;
#endif
#ifdef CONFIG_NUC_UART2
  regval |= CLK_APBCLK_UART2_EN;
#endif

  putreg32(regval, NUC_CLK_APBCLK);

  /* Configure UART GPIO pins */
#warning "Missing logic"

  /* Configure the console UART */
#warning "Missing logic"

#endif /* HAVE_UART */
}