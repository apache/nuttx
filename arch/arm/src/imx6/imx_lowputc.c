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
#include <fixedmath.h>
#include <assert.h>

#include "up_arch.h"

#include "chip/imx_iomuxc.h"
#include "chip/imx_ccm.h"
#include "chip/imx_uart.h"
#include "imx_config.h"
#include "imx_iomuxc.h"
#include "imx_gpio.h"
#include "imx_lowputc.h"

#include "up_internal.h"

#include "chip/imx_pinmux.h"
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
 * characters.  This clock is used in order to allow frequency scaling on
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
#ifndef CONFIG_SUPPRESS_UART_CONFIG
#ifdef IMX_HAVE_UART
  uint32_t regval;

  /* Make certain that the ipg_clock and ipg_perclk are enabled for the UART
   * modules.  Here we set BOTH the ipg_clk and ipg_perclk so that clocking
   * is on in all modes (except STOP).
   */

  regval  = getreg32(IMX_CCM_CCGR5);
  regval &= ~(CCM_CCGR5_CG12_MASK | CCM_CCGR5_CG13_MASK);
  regval |= (CCM_CCGR5_CG12(CCM_CCGR_ALLMODES) |
             CCM_CCGR5_CG13(CCM_CCGR_ALLMODES));
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
#endif /* CONFIG_SUPPRESS_UART_CONFIG */
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
  uint64_t tmp;
  uint32_t regval;
  uint32_t ucr2;
  uint32_t refclk;
  uint32_t div;
  uint32_t num;
  uint32_t den;
  b16_t ratio;

  /* Disable the UART */

  putreg32(0, base + UART_UCR1_OFFSET);
  putreg32(0, base + UART_UCR2_OFFSET);
  putreg32(0, base + UART_UCR3_OFFSET);
  putreg32(0, base + UART_UCR4_OFFSET);

  /* Wait for the UART to come out of reset */

  while ((getreg32(base + UART_UCR2_OFFSET) & UART_UCR2_SRST) == 0);

  /* Set up UCR2, Clearing all bits that will be configured below. */

  ucr2  = getreg32(base + UART_UCR2_OFFSET);
  ucr2 &= ~(UART_UCR2_WS   | UART_UCR2_STPB | UART_UCR2_PREN |
            UART_UCR2_PROE | UART_UCR2_IRTS | UART_UCR2_CTSC);

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

  if (config->parity != 0)
    {
      DEBUGASSERT(config->parity == 1 || config->parity == 2);
      ucr2 |= UART_UCR2_PREN;
      if (config->parity == 1)
        {
          ucr2 |= UART_UCR2_PROE;
        }
    }

  /* Setup hardware flow control */

  regval = 0;

#if 0
  if (config->hwfc)
    {
      /* CTS controled by Rx FIFO */

      ucr2 |= UART_UCR2_CTSC;

      /* Set CTS trigger level */

      regval |= 30 << UART_UCR4_CTSTL_SHIFT;

      /* REVISIT:  There are other relevant bits that must be managed in
       * UCR1 and UCR3.
       */
    }
  else
#endif
    {
      /* Ignore RTS */

      ucr2 |= UART_UCR2_IRTS;
    }

  putreg32(regval, base + UART_UCR4_OFFSET);

  /* Setup the new UART configuration */

  putreg32(ucr2, base + UART_UCR2_OFFSET);

  /* Select a reference clock divider.
   * REVISIT:  For now we just use a divider of 2.  That might not be
   * optimal for very high or very low baud settings.
   */

  div    = 2;
  refclk = (IPG_PERCLK_FREQUENCY >> 1);

  /* Set the baud.
   *
   *   baud    = REFFREQ / (16 * NUM/DEN)
   *   baud    = REFFREQ / 16 / RATIO
   *   RATIO   = REFREQ / 16 / baud;
   *
   *   NUM     = SCALE * RATIO
   *   DEN     = SCALE
   *
   *   UMBR    = NUM-1
   *   UBIR    = DEN-1;
   */

  tmp   = ((uint64_t)refclk << (16 - 4)) / config->baud;
  DEBUGASSERT(tmp < 0x0000000100000000LL);
  ratio = (b16_t)tmp;

  /* Pick a scale factor that gives us about 14 bits of accuracy.
   * REVISIT:  Why not go all the way to 16-bits?
   */

  if (ratio < b16HALF)
    {
      den = (1 << 15);
      num = b16toi(ratio << 15);
      DEBUGASSERT(num > 0);
    }
  else if (ratio < b16ONE)
    {
      den = (1 << 14);
      num = b16toi(ratio << 14);
    }
  else if (ratio < itob16(2))
    {
      den = (1 << 13);
      num = b16toi(ratio << 13);
    }
  else if (ratio < itob16(4))
    {
      den = (1 << 12);
      num = b16toi(ratio << 12);
    }
  else if (ratio < itob16(8))
    {
      den = (1 << 11);
      num = b16toi(ratio << 11);
    }
  else if (ratio < itob16(16))
    {
      den = (1 << 10);
      num = b16toi(ratio << 10);
    }
  else if (ratio < itob16(32))
    {
      den = (1 << 9);
      num = b16toi(ratio << 9);
    }
  else if (ratio < itob16(64))
    {
      den = (1 << 8);
      num = b16toi(ratio << 8);
    }
  else if (ratio < itob16(128))
    {
      den = (1 << 7);
      num = b16toi(ratio << 7);
    }
  else if (ratio < itob16(256))
    {
      den = (1 << 6);
      num = b16toi(ratio << 6);
    }
  else if (ratio < itob16(512))
    {
      den = (1 << 5);
      num = b16toi(ratio << 5);
    }
  else if (ratio < itob16(1024))
    {
      den = (1 << 4);
      num = b16toi(ratio << 4);
    }
  else if (ratio < itob16(2048))
    {
      den = (1 << 3);
      num = b16toi(ratio << 3);
    }
  else if (ratio < itob16(4096))
    {
      den = (1 << 2);
      num = b16toi(ratio << 2);
    }
  else if (ratio < itob16(8192))
    {
      den = (1 << 1);
      num = b16toi(ratio << 1);
    }
  else /* if (ratio < itob16(16384)) */
    {
      DEBUGASSERT(ratio < itob16(16384));
      den = (1 << 0);
      num = b16toi(ratio);
    }

  /* Reduce if possible without losing accuracy. */

  while ((num & 1) == 0 && (den & 1) == 0)
    {
      num >>= 1;
      den >>= 1;
    }

  /* The actual values are we write to the registers need to be
   * decremented by 1.  NOTE that the UBIR must be set before
   * the UBMR.
   */

  putreg32(den - 1, base + UART_UBIR_OFFSET);
  putreg32(num - 1, base + UART_UBMR_OFFSET);

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

  /* Selected. Selects proper input pins for serial and Infrared input
   * signal.  NOTE: In this chip, UARTs are used in MUXED mode, so that this
   * bit should always be set.
   */

  putreg32(UART_UCR3_RXDMUXSEL, base + UART_UCR3_OFFSET);

  /* Enable the TX and RX */

  ucr2 |= (UART_UCR2_TXEN | UART_UCR2_RXEN);
  putreg32(ucr2, base + UART_UCR2_OFFSET);

  /* Enable the UART */

  regval  = getreg32(base + UART_UCR1_OFFSET);
  regval |= UART_UCR1_UARTEN;
  putreg32(regval, base + UART_UCR1_OFFSET);
#endif

  return OK;
}
#endif /* IMX_HAVE_UART */

/************************************************************************************
 * Name: imx_lowputc
 *
 * Description:
 *   Output a byte with as few system dependencies as possible.  This will even work
 *   BEFORE the console is initialized if we are booting from U-Boot (and the same
 *   UART is used for the console, of course.)
 *
 ************************************************************************************/

#if defined(IMX_HAVE_UART) && defined(CONFIG_DEBUG_FEATURES)
void imx_lowputc(int ch)
{
  /* Poll the TX fifo trigger level bit of the UART status register. When the TXFE
   * bit is non-zero, the TX Buffer FIFO is empty.
   */

  while ((getreg32(IMX_CONSOLE_VBASE + UART_USR2_OFFSET) & UART_USR2_TXFE) == 0);

  /* If the character to output is a newline, then pre-pend a carriage return */

  if (ch == '\n')
    {
      /* Send the carrage return by writing it into the UART_TXD register. */

      putreg32((uint32_t)'\r', IMX_CONSOLE_VBASE + UART_TXD_OFFSET);

      /* Wait for the tranmsit regiser to be emptied. When the TXFE bit is non-zero,
       * the TX Buffer FIFO is empty.
       */

      while ((getreg32(IMX_CONSOLE_VBASE + UART_USR2_OFFSET) & UART_USR2_TXFE) == 0);
    }

  /* Send the character by writing it into the UART_TXD register. */

  putreg32((uint32_t)ch, IMX_CONSOLE_VBASE + UART_TXD_OFFSET);

  /* Wait for the tranmsit regiser to be emptied. When the TXFE bit is non-zero,
   * the TX Buffer FIFO is empty.
   */

  while ((getreg32(IMX_CONSOLE_VBASE + UART_USR2_OFFSET) & UART_USR2_TXFE) == 0);
}
#endif
