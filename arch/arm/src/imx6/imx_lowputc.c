/****************************************************************************
 * arch/arm/src/imx6/imx_lowputc.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include "up_arch.h"

#include "chip/imx_iomuxc.h"
#include "chip/imx_pinmux.h"
#include "chip/imx_ccm.h"
#include "chip/imx_uart.h"
#include "imx_config.h"
#include "imx_gpio.h"
#include "imx_lowputc.h"

#include <arch/board/board.h> /* Include last:  has dependencies */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifdef IMX_HAVE_UART_CONSOLE
#  if defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define IMX_CONSOLE_VBASE    IMX_UART1_VBASE
#    define IMX_CONSOLE_BAUD     CONFIG_UART1_BAUD
#    define IMX_CONSOLE_BITS     CONFIG_UART1_BITS
#    define IMX_CONSOLE_PARITY   CONFIG_UART1_PARITY
#    define IMX_CONSOLE_2STOP    CONFIG_UART1_2STOP
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define IMX_CONSOLE_VBASE    IMX_UART2_VBASE
#    define IMX_CONSOLE_BAUD     CONFIG_UART2_BAUD
#    define IMX_CONSOLE_BITS     CONFIG_UART2_BITS
#    define IMX_CONSOLE_PARITY   CONFIG_UART2_PARITY
#    define IMX_CONSOLE_2STOP    CONFIG_UART2_2STOP
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define IMX_CONSOLE_VBASE    IMX_UART3_VBASE
#    define IMX_CONSOLE_BAUD     CONFIG_UART3_BAUD
#    define IMX_CONSOLE_BITS     CONFIG_UART3_BITS
#    define IMX_CONSOLE_PARITY   CONFIG_UART3_PARITY
#    define IMX_CONSOLE_2STOP    CONFIG_UART3_2STOP
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define IMX_CONSOLE_VBASE    IMX_UART4_VBASE
#    define IMX_CONSOLE_BAUD     CONFIG_UART4_BAUD
#    define IMX_CONSOLE_BITS     CONFIG_UART4_BITS
#    define IMX_CONSOLE_PARITY   CONFIG_UART4_PARITY
#    define IMX_CONSOLE_2STOP    CONFIG_UART4_2STOP
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define IMX_CONSOLE_VBASE    IMX_UART5_VBASE
#    define IMX_CONSOLE_BAUD     CONFIG_UART5_BAUD
#    define IMX_CONSOLE_BITS     CONFIG_UART5_BITS
#    define IMX_CONSOLE_PARITY   CONFIG_UART5_PARITY
#    define IMX_CONSOLE_2STOP    CONFIG_UART5_2STOP
#  endif
#endif

/* Clocking *****************************************************************/
/* the UART module receives two clocks, a peripheral_clock (ipg_clk) and the
 * module_clock (ipg_perclk).   The peripheral_clock is used as write clock
 * of the TxFIFO, read clock of the RxFIFO and synchronization of the modem
 * control input pins. It must always be running when UART is enabled.
 *
 * The default ipg_clk is 66MHz (max 66.5MHz).  ipg_clk is gated by
 * CCGR5[CG12], uart_clk_enable.  ipg_clk is shared among many modules and
 * should not be controlled by the UART logic.
 *
 * The module_clock is for all the state machines, writing RxFIFO, reading
 * TxFIFO, etc.  It must always be running when UART is sending or receiving
 * characters.This clock is used in order to allow frequency scaling on
 * peripheral_clock without changing configuration of baud rate.
 *
 * The default ipg_perclk is 80MHz (max 80MHz).  ipg_perclk is gated by
 * CCGR5[CG13], uart_serial_clk_enable.  The clock generation sequence is:
 *
 *   pll3_sw_clk (480M) -> CCGR5[CG13] -> 3 bit divider cg podf=6 ->
 *     PLL3_80M (80Mhz) -> CDCDR1: uart_clk_podf ->
 *       6 bit divider default=1 -> UART_CLK_ROOT
 *
 * REVISIT:  This logic assumes that all dividers are at the default value
 * and that the value of the ipg_perclk is 80MHz.
 */

#define IPG_PERCLK_FREQUENCY  80000000

/* The BRM sub-block receives ref_clk (module_clock clock after divider).
 * From this clock, and with integer and non-integer division, BRM generates
 * a 16x baud rate clock.
 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef IMX_HAVE_UART_CONSOLE
static const struct uart_config_s g_console_config =
{
  .baud      = IMX_CONSOLE_BAUD,    /* Configured baud */
  .parity    = IMX_CONSOLE_PARITY,  /* 0=none, 1=odd, 2=even */
  .bits      = IMX_CONSOLE_BITS,    /* Number of bits (5-9) */
  .stopbits2 = IMX_CONSOLE_2STOP,   /* true: Configure with 2 stop bits instead of 1 */
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization including setup of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot sequence.
 *
 ****************************************************************************/

void imx_lowsetup(void)
{
#ifdef IMX_HAVE_UART
  uint32_t regval;

  /* Make certain that the ipg_perclk is enabled.  The ipg_clk should already
   * have been enabled.  Here we set BOTH the ipg_clk and ipg_perclk so that
   * clocking is on in all modes (except STOP).
   */

  regval  = getreg32(IMX_CCM_CCGR5);
  regval &= (CCM_CCGR5_CG12_MASK | CCM_CCGR5_CG13_MASK);
  regval |= (CCM_CCGR5_CG12(CCM_CCGR_ALLMODES) | CCM_CCGR5_CG13(CCM_CCGR_ALLMODES));
  putreg32(regval, IMX_CCM_CCGR5);

#ifdef CONFIG_IMX6_UART1
  /* Disable and configure UART1 */

  putreg32(0, IMX_UART1_VBASE + UART_UCR1_OFFSET);
  putreg32(0, IMX_UART1_VBASE + UART_UCR2_OFFSET);
  putreg32(0, IMX_UART1_VBASE + UART_UCR3_OFFSET);
  putreg32(0, IMX_UART1_VBASE + UART_UCR4_OFFSET);

  /* Configure UART1 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.  REVISIT: DTR, DCD, RI, and DSR -- not configured.
   */

  (void)imx_config_gpio(GPIO_UART1_RX_DATA);
  (void)imx_config_gpio(GPIO_UART1_TX_DATA);
#ifdef CONFIG_UART1_OFLOWCONTROL
  (void)imx_config_gpio(GPIO_UART1_CTS);
#endif
#ifdef CONFIG_UART1_IFLOWCONTROL
  (void)imx_config_gpio(GPIO_UART1_RTS);
#endif
#endif

#ifdef CONFIG_IMX6_UART2
  /* Disable and configure UART2 */

  putreg32(0, IMX_UART2_VBASE + UART_UCR1_OFFSET);
  putreg32(0, IMX_UART2_VBASE + UART_UCR2_OFFSET);
  putreg32(0, IMX_UART2_VBASE + UART_UCR3_OFFSET);
  putreg32(0, IMX_UART2_VBASE + UART_UCR4_OFFSET);

  /* Configure UART2 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  (void)imx_config_gpio(GPIO_UART2_RX_DATA);
  (void)imx_config_gpio(GPIO_UART2_TX_DATA);
#ifdef CONFIG_UART1_OFLOWCONTROL
  (void)imx_config_gpio(GPIO_UART2_CTS);
#endif
#ifdef CONFIG_UART1_IFLOWCONTROL
  (void)imx_config_gpio(GPIO_UART2_RTS);
#endif
#endif

#ifdef CONFIG_IMX6_UART3
  /* Disable and configure UART3 */

  putreg32(0, IMX_UART3_VBASE + UART_UCR1_OFFSET);
  putreg32(0, IMX_UART3_VBASE + UART_UCR2_OFFSET);
  putreg32(0, IMX_UART3_VBASE + UART_UCR3_OFFSET);
  putreg32(0, IMX_UART3_VBASE + UART_UCR4_OFFSET);

  /* Configure UART3 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  (void)imx_config_gpio(GPIO_UART3_RX_DATA);
  (void)imx_config_gpio(GPIO_UART3_TX_DATA);
#ifdef CONFIG_UART1_OFLOWCONTROL
  (void)imx_config_gpio(GPIO_UART3_CTS);
#endif
#ifdef CONFIG_UART1_IFLOWCONTROL
  (void)imx_config_gpio(GPIO_UART3_RTS);
#endif
#endif

#ifdef CONFIG_IMX6_UART4
  /* Disable and configure UART4 */

  putreg32(0, IMX_UART4_VBASE + UART_UCR1_OFFSET);
  putreg32(0, IMX_UART4_VBASE + UART_UCR2_OFFSET);
  putreg32(0, IMX_UART4_VBASE + UART_UCR3_OFFSET);
  putreg32(0, IMX_UART4_VBASE + UART_UCR4_OFFSET);

  /* Configure UART4 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  (void)imx_config_gpio(GPIO_UART4_RX_DATA);
  (void)imx_config_gpio(GPIO_UART4_TX_DATA);
#ifdef CONFIG_UART1_OFLOWCONTROL
  (void)imx_config_gpio(GPIO_UART4_CTS);
#endif
#ifdef CONFIG_UART1_IFLOWCONTROL
  (void)imx_config_gpio(GPIO_UART4_RTS);
#endif
#endif

#ifdef CONFIG_IMX6_UART5
  /* Disable and configure UART5 */

  putreg32(0, IMX_UART5_VBASE + UART_UCR1_OFFSET);
  putreg32(0, IMX_UART5_VBASE + UART_UCR2_OFFSET);
  putreg32(0, IMX_UART5_VBASE + UART_UCR3_OFFSET);
  putreg32(0, IMX_UART5_VBASE + UART_UCR4_OFFSET);

  /* Configure UART5 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  (void)imx_config_gpio(GPIO_UART5_RX_DATA);
  (void)imx_config_gpio(GPIO_UART5_TX_DATA);
#ifdef CONFIG_UART1_OFLOWCONTROL
  (void)imx_config_gpio(GPIO_UART5_CTS);
#endif
#ifdef CONFIG_UART1_IFLOWCONTROL
  (void)imx_config_gpio(GPIO_UART5_RTS);
#endif
#endif

#ifdef IMX_HAVE_UART_CONSOLE
  /* Configure the serial console for initial, non-interrupt driver mode */

  (void)imx_uart_configure(IMX_CONSOLE_VBASE, &g_console_config);
#endif
#endif /* IMX_HAVE_UART */
}

/************************************************************************************
 * Name: imx_uart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ************************************************************************************/

#ifdef IMX_HAVE_UART
int imx_uart_configure(uint32_t base, FAR const struct uart_config_s *config)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint32_t regval;
  uint32_t ucr2;
  uint32_t div;
  uint32_t num;
  uint32_t den;

  /* Disable the UART */

  putreg32(0, base + UART_UCR1_OFFSET);
  putreg32(0, base + UART_UCR2_OFFSET);
  putreg32(0, base + UART_UCR3_OFFSET);
  putreg32(0, base + UART_UCR4_OFFSET);

  /* Set up UCR2 */

  ucr2  = getreg32(base + UART_UCR2_OFFSET);
  ucr2 |= (UART_UCR2_SRST | UART_UCR2_IRTS);

  /* Select the number of data bits */

  DEBUGASSERT(config->bits == 7 || config->bits == 8);
  if (config->bits == 8)
    {
      ucr2 |= UART_UCR2_WS;
    }

  /* Select the number of stop bits */

  if (config->stopbits2)
    {
      ucr2 |= UART_UCR2_STPB;
    }

  /* Select even/odd parity */

  if (config->parity)
    {
      DEBUGASSERT(config->parity == 1 || config->parity == 2);
      ucr2 |= UART_UCR2_PREN;
      if (config->parity == 1)
        {
          ucr2 |= UART_UCR2_PROE;
        }
    }

  /* Select RTS */

#if 0
  ucr2 &= ~UCR2_IRTS;
  ucr2 |= UCR2_CTSC;
#endif

  /* Setup hardware flow control */

  regval = 0;
#if 0
  if (config->hwfc)
    {
      ucr2 |= UART_UCR2_IRTS;

      /* CTS controled by Rx FIFO */

      ucr2 |= UART_UCR2_CTSC;

      /* Set CTS trigger level */

      regval |= 30 << UART_UCR4_CTSTL_SHIFT;
    }
#endif

  putreg32(regval, base + UART_UCR4_OFFSET);

  /* Setup the new UART configuration */

  putreg32(ucr2, base + UART_UCR2_OFFSET);

  /* Set the baud.
   *
   *   buad    = REFFREQ / (16 x NUM/DEM)
   *   baud * 16 / REFFREQ = NUM/DEN
   *   UBIR    = NUM-1;
   *   UMBR    = DEN-1
   *   REFFREQ = PERCLK1 / DIV,   DIV=1..7
   *   DIV     = RFDIV[2:0]
   *
   * First, select a closest value we can for the divider
   */

  div = (IPG_PERCLK_FREQUENCY >> 4) / config->baud;
  if (div > 7)
    {
      div = 7;
    }
  else if (div < 1)
    {
      div = 1;
    }

  /* Now find the numerator and denominator.  These must have
   * the ratio baud/(PERCLK / div / 16), but the values cannot
   * exceed 16 bits
   */

  num = config->baud;
  den = (IPG_PERCLK_FREQUENCY << 4) / div;

  if (num > den)
    {
      if (num > 0x00010000)
        {
          /* b16 is a scale such that b16*num = 0x10000 * 2**16 */

          uint32_t b16 = 0x100000000LL / num;
          num = 0x00010000;
          den = (b16 * den) >> 16;
        }
    }
  else
    {
      if (den > 0x0000ffff)
        {
          /* b16 is a scale such that b16*den = 0x10000 * 2**16 */

          uint32_t b16 = 0x100000000LL / den;
          num = (b16 * num) >> 16;
          den = 0x00010000;
        }
    }

  /* The actual values are we write to the registers need to be
   * decremented by 1.
   */

  if (num > 0)
    {
      num--;
    }

  if (den > 0)
    {
      den--;
    }

  /* The UBIR must be set before the UBMR register */

  putreg32(num, base + UART_UBIR_OFFSET);
  putreg32(den, base + UART_UBMR_OFFSET);

  /* Fixup the divisor, the value in the UFCR regiser is
   *
   *   000 = Divide input clock by 6
   *   001 = Divide input clock by 5
   *   010 = Divide input clock by 4
   *   011 = Divide input clock by 3
   *   100 = Divide input clock by 2
   *   101 = Divide input clock by 1
   *   110 = Divide input clock by 7
   */

  if (div == 7)
    {
      div = 6;
    }
  else
    {
      div = 6 - div;
    }

  regval = div << UART_UFCR_RFDIV_SHIFT;

  /* Set the TX trigger level to interrupt when the TxFIFO has 2 or fewer
   * characters.  Set the RX trigger level to interrupt when the RxFIFO has
   * 1 character.
   */

  regval |= ((2 << UART_UFCR_TXTL_SHIFT) | (1 << UART_UFCR_RXTL_SHIFT));
  putreg32(regval, base + UART_UFCR_OFFSET);

  /* Enable the TX and RX */

  ucr2 |= (UART_UCR2_TXEN | UART_UCR2_RXEN);
  putreg32(ucr2, base + UART_UCR2_OFFSET);
#endif

  return OK;
}
#endif /* IMX_HAVE_UART */