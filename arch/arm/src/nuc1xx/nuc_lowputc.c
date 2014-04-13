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

#include "chip/nuc_gcr.h"
#include "nuc_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Get the serial console UART configuration */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define NUC_CONSOLE_BASE     NUC_UART0_BASE
#    define NUC_CONSOLE_DEPTH    UART0_FIFO_DEPTH
#    define NUC_CONSOLE_BAUD     CONFIG_UART0_BAUD
#    define NUC_CONSOLE_BITS     CONFIG_UART0_BITS
#    define NUC_CONSOLE_PARITY   CONFIG_UART0_PARITY
#    define NUC_CONSOLE_2STOP    CONFIG_UART0_2STOP
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define NUC_CONSOLE_BASE     NUC_UART1_BASE
#    define NUC_CONSOLE_DEPTH    UART1_FIFO_DEPTH
#    define NUC_CONSOLE_BAUD     CONFIG_UART1_BAUD
#    define NUC_CONSOLE_BITS     CONFIG_UART1_BITS
#    define NUC_CONSOLE_PARITY   CONFIG_UART1_PARITY
#    define NUC_CONSOLE_2STOP    CONFIG_UART1_2STOP
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define NUC_CONSOLE_BASE     NUC_UART2_BASE
#    define NUC_CONSOLE_DEPTH    UART2_FIFO_DEPTH
#    define NUC_CONSOLE_BAUD     CONFIG_UART2_BAUD
#    define NUC_CONSOLE_BITS     CONFIG_UART2_BITS
#    define NUC_CONSOLE_PARITY   CONFIG_UART2_PARITY
#    define NUC_CONSOLE_2STOP    CONFIG_UART2_2STOP
#  endif
#endif

/* Select either the external high speed crystal, the PLL output, or
 * the internal high speed clock as the UART clock source.
 */

#if defined(CONFIG_NUC_UARTCLK_XTALHI)
#  define NUC_UART_CLK BOARD_XTALHI_FREQUENCY
#elif defined(CONFIG_NUC_UARTCLK_PLL)
#  define NUC_UART_CLK BOARD_PLL_FOUT
#elif defined(CONFIG_NUC_UARTCLK_INTHI)
#  define NUC_UART_CLK NUC_INTHI_FREQUENCY
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nuc_console_ready
 *
 * Description:
 *   Wait until the console is ready to add another character to the TX
 *   FIFO.
 *
 *****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
static inline void nuc_console_ready(void)
{
#if 1
  /* Wait for the TX FIFO to be empty (excessive!) */

  while ((getreg32(NUC_CONSOLE_BASE + NUC_UART_FSR_OFFSET) & UART_FSR_TX_EMPTY) == 0);
#else
  uint32_t depth;

  /* Wait until there is space in the TX FIFO */

  do
    {
      register uint32_t regval = getreg32(NUC_CONSOLE_BASE + NUC_UART_FSR_OFFSET);
      depth  = (regval & UART_FSR_TX_POINTER_MASK) >> UART_FSR_TX_POINTER_SHIFT;
    }
  while (depth >= (NUC_CONSOLE_DEPTH-1));
#endif
}
#endif /* HAVE_SERIAL_CONSOLE */

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

  /* Configure UART GPIO pins.
   *
   * Basic UART0 TX/RX requires that GPIOB MFP bits 0 and 1 be set. If flow
   * control is enabled, then GPIOB MFP bits 3 and 4 must also be set and ALT
   * MFP bits 11, 13,  and 14 must be cleared.
   */

#if defined(CONFIG_NUC_UART0) || defined(CONFIG_NUC_UART1)
  regval = getreg32(NUC_GCR_GPB_MFP);

#ifdef CONFIG_NUC_UART0
#ifdef CONFIG_UART0_FLOW_CONTROL
  regval |= (GCR_GPB_MFP0 | GCR_GPB_MFP1 | GCR_GPB_MFP2| GCR_GPB_MFP3);
#else
  regval |= (GCR_GPB_MFP0 | GCR_GPB_MFP1);
#endif
#endif /* CONFIG_NUC_UART0 */

  /* Basic UART1 TX/RX requires that GPIOB MFP bits 4 and 5 be set. If flow
   * control is enabled, then GPIOB MFP bits 6 and 7 must also be set and ALT
   * MFP bit 11 must be cleared.
   */

#ifdef CONFIG_NUC_UART1
#ifdef CONFIG_UART1_FLOW_CONTROL
  regval |= (GCR_GPB_MFP4 | GCR_GPB_MFP5 | GCR_GPB_MFP6| GCR_GPB_MFP7)
#else
  regval |= (GCR_GPB_MFP4 | GCR_GPB_MFP5);
#endif
#endif /* CONFIG_NUC_UART1 */

  putreg32(regval, NUC_GCR_GPB_MFP);

#if defined(CONFIG_UART0_FLOW_CONTROL) || defined(CONFIG_UART1_FLOW_CONTROL)
  regval = getreg32(NUC_GCR_ALT_MFP);
  regval &= ~GCR_ALT_MFP_EBI_EN;
#ifdef CONFIG_UART0_FLOW_CONTROL
  regval &= ~(GCR_ALT_MFP_EBI_NWRL_EN | GCR_ALT_MFP_EBI_NWRH_WN);
#endif
  putreg32(NUC_GCR_ALT_MFP);
#endif /* CONFIG_UART0_FLOW_CONTROL || CONFIG_UART1_FLOW_CONTROL */
#endif /* CONFIG_NUC_UART0 || CONFIG_NUC_UART1 */

  /* UART1 TX/RX support requires that GPIOD bits 14 and 15 be set.  UART2
   * does not support flow control.
   */

#ifdef CONFIG_NUC_UART2
  regval = getreg32(NUC_GCR_GPD_MFP);
  regval |= (GCR_GPD_MFP14 | GCR_GPD_MFP15);
  putreg32(regval, NUC_GCR_GPD_MFP);
#endif /* CONFIG_NUC_UART2 */

  /* Reset the UART peripheral(s) */

  regval = getreg32(NUC_GCR_IPRSTC2);
#ifdef CONFIG_NUC_UART0
  regval |= GCR_IPRSTC2_UART0_RST;
  putreg32(regval, NUC_GCR_IPRSTC2);
  regval &= ~GCR_IPRSTC2_UART0_RST;
  putreg32(regval, NUC_GCR_IPRSTC2);
#endif
#ifdef CONFIG_NUC_UART1
  regval |= GCR_IPRSTC2_UART1_RST;
  putreg32(regval, NUC_GCR_IPRSTC2);
  regval &= ~GCR_IPRSTC2_UART1_RST;
  putreg32(regval, NUC_GCR_IPRSTC2);
#endif
#ifdef CONFIG_NUC_UART2
  regval |= GCR_IPRSTC2_UART2_RST;
  putreg32(regval, NUC_GCR_IPRSTC2);
  regval &= ~GCR_IPRSTC2_UART2_RST;
  putreg32(regval, NUC_GCR_IPRSTC2);
#endif

  /* Configure the UART clock source. Set the UART clock source to either
   * the external high speed crystal (CLKSEL1 reset value), the PLL output,
   * or the internal high speed clock.
   */

  regval  = getreg32(NUC_CLK_CLKSEL1);
  regval &= ~CLK_CLKSEL1_UART_S_MASK;
#if defined(CONFIG_NUC_UARTCLK_XTALHI)
  regval |= CLK_CLKSEL1_UART_S_XTALHI;
#elif defined(CONFIG_NUC_UARTCLK_PLL)
  regval |= CLK_CLKSEL1_UART_S_PLL;
#elif defined(CONFIG_NUC_UARTCLK_INTHI)
  regval |= CLK_CLKSEL1_UART_S_INTHI;
#endif
  putreg32(regval, NUC_CLK_CLKSEL1);

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

  /* Configure the console UART */

#ifdef HAVE_SERIAL_CONSOLE

  /* Reset the TX FIFO */

  regval  = getreg32(NUC_CONSOLE_BASE + NUC_UART_FCR_OFFSET);
  regval &= ~(UART_FCR_TFR | UART_FCR_RFR);
  putreg32(regval | UART_FCR_TFR, NUC_CONSOLE_BASE + NUC_UART_FCR_OFFSET);

  /* Reset the RX FIFO */

  putreg32(regval | UART_FCR_RFR, NUC_CONSOLE_BASE + NUC_UART_FCR_OFFSET);

  /* Set Rx Trigger Level */

  regval &= ~UART_FCR_RFITL_MASK;
  regval |= UART_FCR_RFITL_4;
  putreg32(regval, NUC_CONSOLE_BASE + NUC_UART_FCR_OFFSET);

  /* Set Parity & Data bits and Stop bits */

  regval = 0;
#if NUC_CONSOLE_BITS  == 5
  regval |= UART_LCR_WLS_5;
#elif NUC_CONSOLE_BITS  == 6
  regval |= UART_LCR_WLS_6;
#elif NUC_CONSOLE_BITS  == 7
  regval |= UART_LCR_WLS_7;
#elif NUC_CONSOLE_BITS  == 8
  regval |= UART_LCR_WLS_8;
#else
  error "Unknown console UART data width"
#endif

#if NUC_CONSOLE_PARITY == 1
  regval |= UART_LCR_PBE;
#elif NUC_CONSOLE_PARITY == 2
  regval |= (UART_LCR_PBE | UART_LCR_EPE);
#endif

#if NUC_CONSOLE_2STOP != 0
  regval |= UART_LCR_NSB;
#endif

  putreg32(regval, NUC_CONSOLE_BASE + NUC_UART_LCR_OFFSET);

  /* Set Time-Out values */

  regval = UART_TOR_TOIC(40) |  UART_TOR_DLY(0);
  putreg32(regval, NUC_CONSOLE_BASE + NUC_UART_TOR_OFFSET);

  /* Set the baud */

  nuc_setbaud(NUC_CONSOLE_BASE, NUC_CONSOLE_BAUD);

#endif /* HAVE_SERIAL_CONSOLE */
#endif /* HAVE_UART */
}

/****************************************************************************
 * Name: nuc_lowputc
 *
 * Description:
 *   Output one character to the UART using a simple polling method.
 *
 *****************************************************************************/

void nuc_lowputc(uint32_t ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Wait for the TX FIFO to become available */

  nuc_console_ready();

  /* Then write the character to to the TX FIFO */

  putreg32(ch, NUC_CONSOLE_BASE + NUC_UART_THR_OFFSET);
#endif /* HAVE_SERIAL_CONSOLE */
}

/****************************************************************************
 * Name: nuc_setbaud
 *
 * Description:
 *   Set the BAUD divxisor for the selected UART
 *
 *   Mode DIV_X_EN DIV_X_ONE Divider X   BRD  (Baud rate equation)
 *   -------------------------------------------------------------
 *    0       0        0         B        A   UART_CLK / [16 * (A+2)]
 *    1       1        0         B        A   UART_CLK / [(B+1) * (A+2)] , B must >= 8
 *    2       1        1     Don't care   A   UART_CLK / (A+2), A must >=3
 *
 * Here we assume that the default clock source for the UART modules is
 * the external high speed crystal.
 *
 *****************************************************************************/

#ifdef HAVE_UART
void nuc_setbaud(uintptr_t base, uint32_t baud)
{
  uint32_t regval;
  uint32_t clksperbit;
  uint32_t brd;
  uint32_t divx;

  regval = getreg32(base + NUC_UART_BAUD_OFFSET);

   /* Mode 0: Source Clock mod 16 < 3 => Using Divider X = 16 */

  clksperbit = (NUC_UART_CLK + (baud >> 1)) / baud;
  if ((clksperbit & 15) < 3)
    {
      regval &= ~(UART_BAUD_DIV_X_ONE | UART_BAUD_DIV_X_EN);
      brd = (clksperbit >> 4) - 2;
    }

  /* Source Clock mod 16 >3 => Up 5% Error BaudRate */

  else
    {
      /* Mode 2: Try to Set Divider X = 1 */

      regval |= (UART_BAUD_DIV_X_ONE | UART_BAUD_DIV_X_EN);
      brd = clksperbit - 2;

      /* Check if the divxider exceeds the range */

      if (brd > 0xffff)
        {
          /* Mode 1: Try to Set Divider X up 10 */

          regval &= ~UART_BAUD_DIV_X_ONE;

          for (divx = 8; divx < 16; divx++)
            {
              brd = clksperbit % (divx + 1);
              if (brd < 3)
                {
                  regval &= ~UART_BAUD_DIVIDER_X_MASK;
                  regval |= UART_BAUD_DIVIDER_X(divx);

                  brd -= 2;
                  break;
                }
            }
        }
    }

  regval &= ~UART_BAUD_BRD_MASK;
  regval |= UART_BAUD_BRD(brd);
  putreg32(regval, base + NUC_UART_BAUD_OFFSET);
}
#endif /* HAVE_UART */
