/****************************************************************************
 * arch/arm/src/imxrt/imxrt_lowputc.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <fixedmath.h>
#include <assert.h>

#include "arm_arch.h"

#include "hardware/imxrt_iomuxc.h"
#include "hardware/imxrt_pinmux.h"
#include "hardware/imxrt_ccm.h"
#include "hardware/imxrt_lpuart.h"
#include "imxrt_config.h"
#include "imxrt_periphclks.h"
#include "imxrt_iomuxc.h"
#include "imxrt_gpio.h"
#include "imxrt_lowputc.h"

#include "arm_internal.h"

#include <arch/board/board.h> /* Include last:  has dependencies */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef HAVE_LPUART_CONSOLE
#  if defined(CONFIG_LPUART1_SERIAL_CONSOLE)
#    define IMXRT_CONSOLE_BASE     IMXRT_LPUART1_BASE
#    define IMXRT_CONSOLE_BAUD     CONFIG_LPUART1_BAUD
#    define IMXRT_CONSOLE_BITS     CONFIG_LPUART1_BITS
#    define IMXRT_CONSOLE_PARITY   CONFIG_LPUART1_PARITY
#    define IMXRT_CONSOLE_2STOP    CONFIG_LPUART1_2STOP
#  elif defined(CONFIG_LPUART2_SERIAL_CONSOLE)
#    define IMXRT_CONSOLE_BASE     IMXRT_LPUART2_BASE
#    define IMXRT_CONSOLE_BAUD     CONFIG_LPUART2_BAUD
#    define IMXRT_CONSOLE_BITS     CONFIG_LPUART2_BITS
#    define IMXRT_CONSOLE_PARITY   CONFIG_LPUART2_PARITY
#    define IMXRT_CONSOLE_2STOP    CONFIG_LPUART2_2STOP
#  elif defined(CONFIG_LPUART3_SERIAL_CONSOLE)
#    define IMXRT_CONSOLE_BASE     IMXRT_LPUART3_BASE
#    define IMXRT_CONSOLE_BAUD     CONFIG_LPUART3_BAUD
#    define IMXRT_CONSOLE_BITS     CONFIG_LPUART3_BITS
#    define IMXRT_CONSOLE_PARITY   CONFIG_LPUART3_PARITY
#    define IMXRT_CONSOLE_2STOP    CONFIG_LPUART3_2STOP
#  elif defined(CONFIG_LPUART4_SERIAL_CONSOLE)
#    define IMXRT_CONSOLE_BASE     IMXRT_LPUART4_BASE
#    define IMXRT_CONSOLE_BAUD     CONFIG_LPUART4_BAUD
#    define IMXRT_CONSOLE_BITS     CONFIG_LPUART4_BITS
#    define IMXRT_CONSOLE_PARITY   CONFIG_LPUART4_PARITY
#    define IMXRT_CONSOLE_2STOP    CONFIG_LPUART4_2STOP
#  elif defined(CONFIG_LPUART5_SERIAL_CONSOLE)
#    define IMXRT_CONSOLE_BASE     IMXRT_LPUART5_BASE
#    define IMXRT_CONSOLE_BAUD     CONFIG_LPUART5_BAUD
#    define IMXRT_CONSOLE_BITS     CONFIG_LPUART5_BITS
#    define IMXRT_CONSOLE_PARITY   CONFIG_LPUART5_PARITY
#    define IMXRT_CONSOLE_2STOP    CONFIG_LPUART5_2STOP
#  elif defined(CONFIG_LPUART6_SERIAL_CONSOLE)
#    define IMXRT_CONSOLE_BASE     IMXRT_LPUART6_BASE
#    define IMXRT_CONSOLE_BAUD     CONFIG_LPUART6_BAUD
#    define IMXRT_CONSOLE_BITS     CONFIG_LPUART6_BITS
#    define IMXRT_CONSOLE_PARITY   CONFIG_LPUART6_PARITY
#    define IMXRT_CONSOLE_2STOP    CONFIG_LPUART6_2STOP
#  elif defined(CONFIG_LPUART7_SERIAL_CONSOLE)
#    define IMXRT_CONSOLE_BASE     IMXRT_LPUART7_BASE
#    define IMXRT_CONSOLE_BAUD     CONFIG_LPUART7_BAUD
#    define IMXRT_CONSOLE_BITS     CONFIG_LPUART7_BITS
#    define IMXRT_CONSOLE_PARITY   CONFIG_LPUART7_PARITY
#    define IMXRT_CONSOLE_2STOP    CONFIG_LPUART7_2STOP
#  elif defined(CONFIG_LPUART8_SERIAL_CONSOLE)
#    define IMXRT_CONSOLE_BASE     IMXRT_LPUART8_BASE
#    define IMXRT_CONSOLE_BAUD     CONFIG_LPUART8_BAUD
#    define IMXRT_CONSOLE_BITS     CONFIG_LPUART8_BITS
#    define IMXRT_CONSOLE_PARITY   CONFIG_LPUART8_PARITY
#    define IMXRT_CONSOLE_2STOP    CONFIG_LPUART8_2STOP
#  endif
#endif

/* Clocking *****************************************************************/

/* The UART module receives two clocks, a peripheral_clock (ipg_clk) and the
 * module_clock (ipg_perclk).   The peripheral_clock is used as write clock
 * of the TxFIFO, read clock of the RxFIFO and synchronization of the modem
 * control input pins. It must always be running when UART is enabled.
 *
 * The default lpuart1 ipg_clk is 66MHz (max 66.5MHz).  ipg_clk is shared
 * among many modules and should not be controlled by the UART logic.
 *
 * The module_clock is for all the state machines, writing RxFIFO, reading
 * TxFIFO, etc.  It must always be running when UART is sending or receiving
 * characters.  This clock is used in order to allow frequency scaling on
 * peripheral_clock without changing configuration of baud rate.
 *
 * The default ipg_perclk is 80MHz (max 80MHz).  ipg_perclk is gated by
 * CCGR5[CG12], lpuart1_clk_enable.  The clock generation sequence is:
 *
 *   pll3_sw_clk (480M) -> CCGR5[CG12] -> 3 bit divider cg podf=6 ->
 *     PLL3_80M (80Mhz) -> CDCDR1: lpuart1_clk_podf ->
 *       6 bit divider default=1 -> LPUART1_CLK_ROOT
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

#ifdef HAVE_LPUART_CONSOLE
static const struct uart_config_s g_console_config =
{
  .baud      = IMXRT_CONSOLE_BAUD,    /* Configured baud */
  .parity    = IMXRT_CONSOLE_PARITY,  /* 0=none, 1=odd, 2=even */
  .bits      = IMXRT_CONSOLE_BITS,    /* Number of bits (5-9) */
  .stopbits2 = IMXRT_CONSOLE_2STOP,   /* true: Configure with 2 stop bits instead of 1 */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void imxrt_lpuart_clock_enable (uint32_t base)
{
  if (base == IMXRT_LPUART1_BASE)
    {
      imxrt_clockall_lpuart1();
    }
  else if (base == IMXRT_LPUART2_BASE)
    {
      imxrt_clockall_lpuart2();
    }
  else if (base == IMXRT_LPUART3_BASE)
    {
      imxrt_clockall_lpuart3();
    }
  else if (base == IMXRT_LPUART4_BASE)
    {
      imxrt_clockall_lpuart4();
    }
  else if (base == IMXRT_LPUART5_BASE)
    {
      imxrt_clockall_lpuart5();
    }
  else if (base == IMXRT_LPUART6_BASE)
    {
      imxrt_clockall_lpuart6();
    }
  else if (base == IMXRT_LPUART7_BASE)
    {
      imxrt_clockall_lpuart7();
    }
  else if (base == IMXRT_LPUART8_BASE)
    {
      imxrt_clockall_lpuart8();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization including setup of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot sequence.
 *
 ****************************************************************************/

void imxrt_lowsetup(void)
{
#ifndef CONFIG_SUPPRESS_LPUART_CONFIG
#ifdef HAVE_LPUART_DEVICE

#ifdef CONFIG_IMXRT_LPUART1

  /* Configure LPUART1 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  imxrt_config_gpio(GPIO_LPUART1_RX);
  imxrt_config_gpio(GPIO_LPUART1_TX);
#ifdef CONFIG_LPUART1_OFLOWCONTROL
  imxrt_config_gpio(GPIO_LPUART1_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART1_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART1_IFLOWCONTROL)))
  imxrt_config_gpio(GPIO_LPUART1_RTS);
#endif
#endif

#ifdef CONFIG_IMXRT_LPUART2

  /* Configure LPUART2 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  imxrt_config_gpio(GPIO_LPUART2_RX);
  imxrt_config_gpio(GPIO_LPUART2_TX);
#ifdef CONFIG_LPUART2_OFLOWCONTROL
  imxrt_config_gpio(GPIO_LPUART2_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART2_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART2_IFLOWCONTROL)))
  imxrt_config_gpio(GPIO_LPUART2_RTS);
#endif
#endif

#ifdef CONFIG_IMXRT_LPUART3

  /* Configure LPUART3 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  imxrt_config_gpio(GPIO_LPUART3_RX);
  imxrt_config_gpio(GPIO_LPUART3_TX);
#ifdef CONFIG_LPUART3_OFLOWCONTROL
  imxrt_config_gpio(GPIO_LPUART3_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART3_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART3_IFLOWCONTROL)))
  imxrt_config_gpio(GPIO_LPUART3_RTS);
#endif
#endif

#ifdef CONFIG_IMXRT_LPUART4

  /* Configure LPUART4 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  imxrt_config_gpio(GPIO_LPUART4_RX);
  imxrt_config_gpio(GPIO_LPUART4_TX);
#ifdef CONFIG_LPUART4_OFLOWCONTROL
  imxrt_config_gpio(GPIO_LPUART4_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART4_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART4_IFLOWCONTROL)))
  imxrt_config_gpio(GPIO_LPUART4_RTS);
#endif
#endif

#ifdef CONFIG_IMXRT_LPUART5

  /* Configure LPUART5 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  imxrt_config_gpio(GPIO_LPUART5_RX);
  imxrt_config_gpio(GPIO_LPUART5_TX);
#ifdef CONFIG_LPUART5_OFLOWCONTROL
  imxrt_config_gpio(GPIO_LPUART5_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART5_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART5_IFLOWCONTROL)))
  imxrt_config_gpio(GPIO_LPUART5_RTS);
#endif
#endif

#ifdef CONFIG_IMXRT_LPUART6

  /* Configure LPUART6 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  imxrt_config_gpio(GPIO_LPUART6_RX);
  imxrt_config_gpio(GPIO_LPUART6_TX);
#ifdef CONFIG_LPUART6_OFLOWCONTROL
  imxrt_config_gpio(GPIO_LPUART6_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART6_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART6_IFLOWCONTROL)))
  imxrt_config_gpio(GPIO_LPUART6_RTS);
#endif
#endif

#ifdef CONFIG_IMXRT_LPUART7

  /* Configure LPUART7 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  imxrt_config_gpio(GPIO_LPUART7_RX);
  imxrt_config_gpio(GPIO_LPUART7_TX);
#ifdef CONFIG_LPUART7_OFLOWCONTROL
  imxrt_config_gpio(GPIO_LPUART7_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART7_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART7_IFLOWCONTROL)))
  imxrt_config_gpio(GPIO_LPUART7_RTS);
#endif
#endif

#ifdef CONFIG_IMXRT_LPUART8

  /* Configure LPUART8 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  imxrt_config_gpio(GPIO_LPUART8_RX);
  imxrt_config_gpio(GPIO_LPUART8_TX);
#ifdef CONFIG_LPUART8_OFLOWCONTROL
  imxrt_config_gpio(GPIO_LPUART8_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART8_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART8_IFLOWCONTROL)))
  imxrt_config_gpio(GPIO_LPUART8_RTS);
#endif
#endif

#ifdef HAVE_LPUART_CONSOLE
  /* Configure the serial console for initial, non-interrupt driver mode */

  imxrt_lpuart_configure(IMXRT_CONSOLE_BASE, &g_console_config);
#endif
#endif /* HAVE_LPUART_DEVICE */
#endif /* CONFIG_SUPPRESS_LPUART_CONFIG */
}

/****************************************************************************
 * Name: imxrt_lpuart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ****************************************************************************/

#ifdef HAVE_LPUART_DEVICE
int imxrt_lpuart_configure(uint32_t base,
                           FAR const struct uart_config_s *config)
{
  uint32_t src_freq = 0;
  uint32_t pll3_div = 0;
  uint32_t uart_div = 0;
  uint32_t lpuart_freq = 0;
  uint16_t sbr;
  uint16_t temp_sbr;
  uint32_t osr;
  uint32_t temp_osr;
  uint32_t temp_diff;
  uint32_t calculated_baud;
  uint32_t baud_diff;
  uint32_t regval;
  uint32_t regval2;

  if ((getreg32(IMXRT_CCM_CSCDR1) & CCM_CSCDR1_UART_CLK_SEL) != 0)
    {
      src_freq = BOARD_XTAL_FREQUENCY;
    }
  else
    {
      if ((getreg32(IMXRT_CCM_ANALOG_PLL_USB1) &
           CCM_ANALOG_PLL_USB1_DIV_SELECT_MASK) != 0)
        {
          pll3_div = 22;
        }
      else
        {
          pll3_div = 20;
        }

      src_freq = (BOARD_XTAL_FREQUENCY * pll3_div) / 6;
    }

  uart_div    = (getreg32(IMXRT_CCM_CSCDR1) &
                 CCM_CSCDR1_UART_CLK_PODF_MASK) + 1;
  lpuart_freq = src_freq / uart_div;

  /* This LPUART instantiation uses a slightly different baud rate
   * calculation.  The idea is to use the best OSR (over-sampling rate)
   * possible.
   *
   * NOTE: OSR is typically hard-set to 16 in other LPUART instantiations
   * loop to find the best OSR value possible, one that generates minimum
   * baud_diff iterate through the rest of the supported values of OSR
   */

  baud_diff = config->baud;
  osr       = 0;
  sbr       = 0;

  for (temp_osr = 4; temp_osr <= 32; temp_osr++)
    {
      /* Calculate the temporary sbr value   */

      temp_sbr = (lpuart_freq / (config->baud * temp_osr));

      /* Set temp_sbr to 1 if the sourceClockInHz can not satisfy the
       * desired baud rate.
       */

      if (temp_sbr == 0)
        {
          temp_sbr = 1;
        }

      /* Calculate the baud rate based on the temporary OSR and SBR values */

      calculated_baud = (lpuart_freq / (temp_osr * temp_sbr));
      temp_diff       = calculated_baud - config->baud;

      /* Select the better value between srb and (sbr + 1) */

      if (temp_diff > (config->baud -
                      (lpuart_freq / (temp_osr * (temp_sbr + 1)))))
        {
          temp_diff = config->baud -
                      (lpuart_freq / (temp_osr * (temp_sbr + 1)));
          temp_sbr++;
        }

      if (temp_diff <= baud_diff)
        {
          baud_diff = temp_diff;
          osr       = temp_osr;
          sbr       = temp_sbr;
        }
    }

  if (baud_diff > ((config->baud / 100) * 3))
    {
      /* Unacceptable baud rate difference of more than 3% */

      return ERROR;
    }

  /* Enable lpuart clock */

  imxrt_lpuart_clock_enable(base);

  /* Reset all internal logic and registers, except the Global Register */

  regval  = getreg32(base + IMXRT_LPUART_GLOBAL_OFFSET);
  regval |= LPUART_GLOBAL_RST;
  putreg32(regval, base + IMXRT_LPUART_GLOBAL_OFFSET);

  regval &= ~LPUART_GLOBAL_RST;
  putreg32(regval, base + IMXRT_LPUART_GLOBAL_OFFSET);

  /* Construct MODIR register */

  regval = 0;

  if (config->userts)
    {
      regval |= LPUART_MODIR_RXRTSE;
    }
  else if (config->users485)
    {
      /* Both TX and RX side can't control RTS, so this gives
       * the RX side precedence. This should have been filtered
       * in layers above anyway, but it's just a precaution.
       */

      regval |= LPUART_MODIR_TXRTSE;
    }

  if (config->usects)
    {
      regval |= LPUART_MODIR_TXCTSE;
    }

  if (config->invrts)
    {
      regval |= LPUART_MODIR_TXRTSPOL;
    }

  putreg32(regval, base + IMXRT_LPUART_MODIR_OFFSET);

  regval = 0;

  if ((osr > 3) && (osr < 8))
    {
      regval |= LPUART_BAUD_BOTHEDGE;
    }

  if (config->stopbits2)
    {
      regval |= LPUART_BAUD_SBNS;
    }

  regval |= LPUART_BAUD_OSR(osr) | LPUART_BAUD_SBR(sbr);
  putreg32(regval, base + IMXRT_LPUART_BAUD_OFFSET);

  regval = 0;
  if (config->parity == 1)
    {
      regval |= LPUART_CTRL_PE | LPUART_CTRL_PT_ODD;
    }
  else if (config->parity == 2)
    {
      regval |= LPUART_CTRL_PE | LPUART_CTRL_PT_EVEN;
    }

  if (config->bits == 9 || (config->bits == 8 && config->parity != 0))
    {
      regval |= LPUART_CTRL_M;
    }
  else if ((config->bits == 8))
    {
      regval &= ~LPUART_CTRL_M;
    }
  else
    {
      /* Here should be added support of other bit modes. */

#warning missing logic
      return ERROR;
    }

  regval2  = getreg32(base + IMXRT_LPUART_FIFO_OFFSET);
  regval2 |= LPUART_FIFO_RXFLUSH | LPUART_FIFO_TXFLUSH |
             LPUART_FIFO_RXFE | LPUART_FIFO_RXIDEN_1 | LPUART_FIFO_TXFE;
  putreg32(regval2 , base + IMXRT_LPUART_FIFO_OFFSET);

  regval |= LPUART_CTRL_RE | LPUART_CTRL_TE;
  putreg32(regval, base + IMXRT_LPUART_CTRL_OFFSET);

  return OK;
}
#endif /* HAVE_LPUART_DEVICE */

/****************************************************************************
 * Name: imxrt_lowputc
 *
 * Description:
 *   Output a byte with as few system dependencies as possible.  This will
 *   even work BEFORE the console is initialized if we are booting from U-
 *   Boot (and the same UART is used for the console, of course.)
 *
 ****************************************************************************/

#if defined(HAVE_LPUART_DEVICE) && defined(CONFIG_DEBUG_FEATURES)
void imxrt_lowputc(int ch)
{
  while ((getreg32(IMXRT_CONSOLE_BASE + IMXRT_LPUART_STAT_OFFSET) &
         LPUART_STAT_TDRE) == 0)
    {
    }

  /* If the character to output is a newline, then pre-pend a carriage
   * return
   */

  if (ch == '\n')
    {
      /* Send the carriage return by writing it into the UART_TXD register. */

      putreg32((uint32_t)'\r', IMXRT_CONSOLE_BASE +
                               IMXRT_LPUART_DATA_OFFSET);

      /* Wait for the transmit register to be emptied. When the TXFE bit is
       * non-zero, the TX Buffer FIFO is empty.
       */

      while ((getreg32(IMXRT_CONSOLE_BASE + IMXRT_LPUART_STAT_OFFSET) &
             LPUART_STAT_TDRE) == 0)
        {
        }
    }

  /* Send the character by writing it into the UART_TXD register. */

  putreg32((uint32_t)ch, IMXRT_CONSOLE_BASE + IMXRT_LPUART_DATA_OFFSET);
}
#endif
