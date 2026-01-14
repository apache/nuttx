/****************************************************************************
 * arch/arm/src/mcx-nxxx/nxxx_lowputc.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include "nxxx_lowputc.h"

#include <nuttx/config.h>

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <errno.h>

#include "arm_internal.h"

#include <arch/board/board.h>
#include "hardware/nxxx_clock.h"
#include "hardware/nxxx_flexcomm.h"
#include "hardware/nxxx_lpuart.h"

#include "nxxx_clockconfig.h"
#include "nxxx_port.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if defined(CONFIG_LPUART0_SERIAL_CONSOLE)
#  define NXXX_CONSOLE_DEVNUM   0
#  define NXXX_CONSOLE_BASE     NXXX_LPUART0_BASE
#  define NXXX_CONSOLE_BAUD     CONFIG_LPUART0_BAUD
#  define NXXX_CONSOLE_BITS     CONFIG_LPUART0_BITS
#  define NXXX_CONSOLE_PARITY   CONFIG_LPUART0_PARITY
#  define NXXX_CONSOLE_2STOP    CONFIG_LPUART0_2STOP
#elif defined(CONFIG_LPUART2_SERIAL_CONSOLE)
#  define NXXX_CONSOLE_DEVNUM   1
#  define NXXX_CONSOLE_BASE     NXXX_LPUART1_BASE
#  define NXXX_CONSOLE_BAUD     CONFIG_LPUART1_BAUD
#  define NXXX_CONSOLE_BITS     CONFIG_LPUART1_BITS
#  define NXXX_CONSOLE_PARITY   CONFIG_LPUART1_PARITY
#  define NXXX_CONSOLE_2STOP    CONFIG_LPUART1_2STOP
#elif defined(CONFIG_LPUART2_SERIAL_CONSOLE)
#  define NXXX_CONSOLE_DEVNUM   2
#  define NXXX_CONSOLE_BASE     NXXX_LPUART2_BASE
#  define NXXX_CONSOLE_BAUD     CONFIG_LPUART2_BAUD
#  define NXXX_CONSOLE_BITS     CONFIG_LPUART2_BITS
#  define NXXX_CONSOLE_PARITY   CONFIG_LPUART2_PARITY
#  define NXXX_CONSOLE_2STOP    CONFIG_LPUART2_2STOP
#elif defined(CONFIG_LPUART3_SERIAL_CONSOLE)
#  define NXXX_CONSOLE_DEVNUM   3
#  define NXXX_CONSOLE_BASE     NXXX_LPUART3_BASE
#  define NXXX_CONSOLE_BAUD     CONFIG_LPUART3_BAUD
#  define NXXX_CONSOLE_BITS     CONFIG_LPUART3_BITS
#  define NXXX_CONSOLE_PARITY   CONFIG_LPUART3_PARITY
#  define NXXX_CONSOLE_2STOP    CONFIG_LPUART3_2STOP
#elif defined(CONFIG_LPUART4_SERIAL_CONSOLE)
#  define NXXX_CONSOLE_DEVNUM   4
#  define NXXX_CONSOLE_BASE     NXXX_LPUART4_BASE
#  define NXXX_CONSOLE_BAUD     CONFIG_LPUART4_BAUD
#  define NXXX_CONSOLE_BITS     CONFIG_LPUART4_BITS
#  define NXXX_CONSOLE_PARITY   CONFIG_LPUART4_PARITY
#  define NXXX_CONSOLE_2STOP    CONFIG_LPUART4_2STOP
#elif defined(CONFIG_LPUART5_SERIAL_CONSOLE)
#  define NXXX_CONSOLE_DEVNUM   5
#  define NXXX_CONSOLE_BASE     NXXX_LPUART5_BASE
#  define NXXX_CONSOLE_BAUD     CONFIG_LPUART5_BAUD
#  define NXXX_CONSOLE_BITS     CONFIG_LPUART5_BITS
#  define NXXX_CONSOLE_PARITY   CONFIG_LPUART5_PARITY
#  define NXXX_CONSOLE_2STOP    CONFIG_LPUART5_2STOP
#elif defined(CONFIG_LPUART6_SERIAL_CONSOLE)
#  define NXXX_CONSOLE_DEVNUM   6
#  define NXXX_CONSOLE_BASE     NXXX_LPUART6_BASE
#  define NXXX_CONSOLE_BAUD     CONFIG_LPUART6_BAUD
#  define NXXX_CONSOLE_BITS     CONFIG_LPUART6_BITS
#  define NXXX_CONSOLE_PARITY   CONFIG_LPUART6_PARITY
#  define NXXX_CONSOLE_2STOP    CONFIG_LPUART6_2STOP
#elif defined(CONFIG_LPUART7_SERIAL_CONSOLE)
#  define NXXX_CONSOLE_DEVNUM   7
#  define NXXX_CONSOLE_BASE     NXXX_LPUART7_BASE
#  define NXXX_CONSOLE_BAUD     CONFIG_LPUART7_BAUD
#  define NXXX_CONSOLE_BITS     CONFIG_LPUART7_BITS
#  define NXXX_CONSOLE_PARITY   CONFIG_LPUART7_PARITY
#  define NXXX_CONSOLE_2STOP    CONFIG_LPUART7_2STOP
#endif

/* Clocking *****************************************************************/

/* Functional clocking is provided via the  PCC.  The PCC clocking must
 * be configured by board-specific logic prior to using the LPUART.
 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef NXXX_CONSOLE_BASE
static const struct uart_config_s g_console_config =
{
  .baud      = NXXX_CONSOLE_BAUD,    /* Configured baud */
  .parity    = NXXX_CONSOLE_PARITY,  /* 0=none, 1=odd, 2=even */
  .bits      = NXXX_CONSOLE_BITS,    /* Number of bits (5-9) */
  .stopbits2 = NXXX_CONSOLE_2STOP,   /* true: Configure with 2 stop bits instead of 1 */
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxxx_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization including setup of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot sequence.
 *
 ****************************************************************************/

void nxxx_lowsetup(void)
{
#ifdef CONFIG_NXXX_LPUART0
  /* Configure LPUART0 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  nxxx_port_configure(PORT_LPUART0_RX);
  nxxx_port_configure(PORT_LPUART0_TX);
#ifdef CONFIG_LPUART1_OFLOWCONTROL
  nxxx_port_configure(PORT_LPUART0_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART1_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART1_IFLOWCONTROL)))
  nxxx_port_configure(PORT_LPUART0_RTS);
#endif
#endif

#ifdef CONFIG_NXXX_LPUART1
  /* Configure LPUART1 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  nxxx_port_configure(PORT_LPUART1_RX);
  nxxx_port_configure(PORT_LPUART1_TX);
#ifdef CONFIG_LPUART1_OFLOWCONTROL
  nxxx_port_configure(PORT_LPUART1_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART1_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART1_IFLOWCONTROL)))
  nxxx_port_configure(PORT_LPUART1_RTS);
#endif
#endif

#ifdef CONFIG_NXXX_LPUART2

  /* Configure LPUART2 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  nxxx_port_configure(PORT_LPUART2_RX);
  nxxx_port_configure(PORT_LPUART2_TX);
#ifdef CONFIG_LPUART2_OFLOWCONTROL
  nxxx_port_configure(PORT_LPUART2_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART2_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART2_IFLOWCONTROL)))
  nxxx_port_configure(PORT_LPUART2_RTS);
#endif
#endif

#ifdef CONFIG_NXXX_LPUART3

  /* Configure LPUART3 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  nxxx_port_configure(PORT_LPUART3_RX);
  nxxx_port_configure(PORT_LPUART3_TX);
#ifdef CONFIG_LPUART3_OFLOWCONTROL
  nxxx_port_configure(PORT_LPUART3_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART3_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART3_IFLOWCONTROL)))
  nxxx_port_configure(PORT_LPUART3_RTS);
#endif
#endif

#ifdef CONFIG_NXXX_LPUART4

  /* Configure LPUART4 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  nxxx_port_configure(PORT_LPUART4_RX);
  nxxx_port_configure(PORT_LPUART4_TX);
#ifdef CONFIG_LPUART4_OFLOWCONTROL
  nxxx_port_configure(PORT_LPUART4_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART4_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART4_IFLOWCONTROL)))
  nxxx_port_configure(PORT_LPUART4_RTS);
#endif
#endif

#ifdef CONFIG_NXXX_LPUART5

  /* Configure LPUART5 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  nxxx_port_configure(PORT_LPUART5_RX);
  nxxx_port_configure(PORT_LPUART5_TX);
#ifdef CONFIG_LPUART5_OFLOWCONTROL
  nxxx_port_configure(PORT_LPUART5_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART5_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART5_IFLOWCONTROL)))
  nxxx_port_configure(PORT_LPUART5_RTS);
#endif
#endif

#ifdef CONFIG_NXXX_LPUART6

  /* Configure LPUART6 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  nxxx_port_configure(PORT_LPUART6_RX);
  nxxx_port_configure(PORT_LPUART6_TX);
#ifdef CONFIG_LPUART6_OFLOWCONTROL
  nxxx_port_configure(PORT_LPUART6_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART6_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART6_IFLOWCONTROL)))
  nxxx_port_configure(PORT_LPUART6_RTS);
#endif
#endif

#ifdef CONFIG_NXXX_LPUART7

  /* Configure LPUART7 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  nxxx_port_configure(PORT_LPUART7_RX);
  nxxx_port_configure(PORT_LPUART7_TX);
#ifdef CONFIG_LPUART7_OFLOWCONTROL
  nxxx_port_configure(PORT_LPUART7_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART7_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART7_IFLOWCONTROL)))
  nxxx_port_configure(PORT_LPUART7_RTS);
#endif
#endif

#ifdef NXXX_CONSOLE_BASE
  /* Configure the serial console for initial, non-interrupt driver mode */

  nxxx_lpuart_configure(NXXX_CONSOLE_BASE, NXXX_CONSOLE_DEVNUM,
                        &g_console_config);
#endif
}

/****************************************************************************
 * Name: nxxx_lpuart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ****************************************************************************/

int nxxx_lpuart_configure(uint32_t base, int uartnum,
                          const struct uart_config_s *config)
{
  uint32_t lpuart_freq = 12000000u;
  struct clock_regs_s clock_regs;
  struct clock_gate_reg_s clock_gate;
  uint32_t clock_source;
  uint32_t psel_reg;
  uint16_t sbr;
  uint16_t temp_sbr;
  uint32_t osr;
  uint32_t temp_osr;
  int temp_diff;
  int configured_baud = config->baud;
  int calculated_baud;
  int baud_diff;
  uint32_t regval;

  switch (base)
    {
      case NXXX_LPUART0_BASE:
        clock_regs = SYSCON_FCCLK0;
        clock_source = FRO12M_TO_FLEXCOMM0;
        clock_gate = CLOCK_GATE_LPFLEXCOMM0;
        psel_reg = NXXX_FLEXCOMM0_PSELID;
        break;

      case NXXX_LPUART1_BASE:
        clock_regs = SYSCON_FCCLK1;
        clock_source = FRO12M_TO_FLEXCOMM1;
        clock_gate = CLOCK_GATE_LPFLEXCOMM1;
        psel_reg = NXXX_FLEXCOMM1_PSELID;
        break;

      case NXXX_LPUART2_BASE:
        clock_regs = SYSCON_FCCLK2;
        clock_source = FRO12M_TO_FLEXCOMM2;
        clock_gate = CLOCK_GATE_LPFLEXCOMM2;
        psel_reg = NXXX_FLEXCOMM2_PSELID;
        break;

      case NXXX_LPUART3_BASE:
        clock_regs = SYSCON_FCCLK3;
        clock_source = FRO12M_TO_FLEXCOMM3;
        clock_gate = CLOCK_GATE_LPFLEXCOMM3;
        psel_reg = NXXX_FLEXCOMM3_PSELID;
        break;

      case NXXX_LPUART4_BASE:
        clock_regs = SYSCON_FCCLK4;
        clock_source = FRO12M_TO_FLEXCOMM4;
        clock_gate = CLOCK_GATE_LPFLEXCOMM4;
        psel_reg = NXXX_FLEXCOMM4_PSELID;
        break;

      case NXXX_LPUART5_BASE:
        clock_regs = SYSCON_FCCLK5;
        clock_source = FRO12M_TO_FLEXCOMM5;
        clock_gate = CLOCK_GATE_LPFLEXCOMM5;
        psel_reg = NXXX_FLEXCOMM5_PSELID;
        break;

      case NXXX_LPUART6_BASE:
        clock_regs = SYSCON_FCCLK6;
        clock_source = FRO12M_TO_FLEXCOMM6;
        clock_gate = CLOCK_GATE_LPFLEXCOMM6;
        psel_reg = NXXX_FLEXCOMM6_PSELID;
        break;

      case NXXX_LPUART7_BASE:
        clock_regs = SYSCON_FCCLK7;
        clock_source = FRO12M_TO_FLEXCOMM7;
        clock_gate = CLOCK_GATE_LPFLEXCOMM7;
        psel_reg = NXXX_FLEXCOMM7_PSELID;
        break;

      default:
        return ERROR;
    }

  /* Set FRO12MHz with divider of 1 */

  nxxx_set_periphclock(clock_regs, clock_source, 1);
  nxxx_set_clock_gate(clock_gate, true);

  /* Set FLEXCOMM as LPUART */

  putreg32(FLEXCOMM_PSELID_PERSEL_USART, psel_reg);

  /* This LPUART instantiation uses a slightly different baud rate
   * calculation.  The idea is to use the best OSR (over-sampling rate)
   * possible.
   *
   * NOTE: OSR is typically hard-set to 16 in other LPUART instantiations
   * loop to find the best OSR value possible, one that generates minimum
   * baud_diff iterate through the rest of the supported values of OSR
   */

  baud_diff = configured_baud;
  osr       = 0;
  sbr       = 0;

  for (temp_osr = 4; temp_osr <= 32; temp_osr++)
    {
      /* Calculate the temporary sbr value   */

      temp_sbr = (lpuart_freq / (configured_baud * temp_osr));

      /* Set temp_sbr to 1 if the sourceClockInHz can not satisfy the
       * desired baud rate.
       */

      if (temp_sbr == 0)
        {
          temp_sbr = 1;
        }

      /* Calculate the baud rate based on the temporary OSR and SBR values */

      calculated_baud = (lpuart_freq / (temp_osr * temp_sbr));
      temp_diff       = abs(calculated_baud - configured_baud);

      /* Select the better value between srb and (sbr + 1) */

      calculated_baud = (lpuart_freq / (temp_osr * (temp_sbr + 1)));
      if (temp_diff >
          abs(calculated_baud - configured_baud))
        {
          temp_diff = abs(calculated_baud - configured_baud);
          temp_sbr++;
        }

      if (temp_diff <= baud_diff)
        {
          baud_diff = temp_diff;
          osr       = temp_osr;
          sbr       = temp_sbr;
        }
    }

  if (baud_diff > ((configured_baud * 3) / 100))
    {
      /* Unacceptable baud rate difference of more than 3% */

      return ERROR;
    }

  /* Reset all internal logic and registers, except the Global Register */

  regval  = getreg32(base + NXXX_LPUART_GLOBAL_OFFSET);
  regval |= LPUART_GLOBAL_RST;
  putreg32(regval, base + NXXX_LPUART_GLOBAL_OFFSET);

  regval &= ~LPUART_GLOBAL_RST;
  putreg32(regval, base + NXXX_LPUART_GLOBAL_OFFSET);

  /* Enable RX and TX FIFOs */

  putreg32(LPUART_FIFO_RXFE | LPUART_FIFO_TXFE,
           base + NXXX_LPUART_FIFO_OFFSET);

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

  putreg32(regval, base + NXXX_LPUART_MODIR_OFFSET);

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
  putreg32(regval, base + NXXX_LPUART_BAUD_OFFSET);

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
      /* REVISIT: Here should be added support of other bit modes. */

      return -ENOSYS;
    }

  regval |= LPUART_CTRL_RE | LPUART_CTRL_TE;
  putreg32(regval, base + NXXX_LPUART_CTRL_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: arm_earlyprintinit
 *
 * Description:
 *   Configure LPUART1 for non-interrupt driven operation
 *
 ****************************************************************************/

void arm_earlyprintinit(char ch)
{
  /* Assume bootloader has already set up the LPUART1 */
}

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output a byte with as few system dependencies as possible.  This will
 *   even work BEFORE the console is initialized if we are booting from U-
 *   Boot (and the same UART is used for the console, of course.)
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef NXXX_CONSOLE_BASE
  while ((getreg32(NXXX_CONSOLE_BASE + NXXX_LPUART_STAT_OFFSET) &
         LPUART_STAT_TDRE) == 0)
    {
    }

  /* If the character to output is a newline,
   * then prepend a carriage return.
   */

  if (ch == '\n')
    {
      /* Send the carriage return by writing it into the UART_TXD register. */

      putreg32((uint32_t)'\r',
                NXXX_CONSOLE_BASE + NXXX_LPUART_DATA_OFFSET);

      /* Wait for the transmit register to be emptied. When the TXFE bit is
       * non-zero, the TX Buffer FIFO is empty.
       */

      while ((getreg32(NXXX_CONSOLE_BASE + NXXX_LPUART_STAT_OFFSET) &
             LPUART_STAT_TDRE) == 0)
        {
        }
    }

  /* Send the character by writing it into the UART_TXD register. */

  putreg32((uint32_t)ch, NXXX_CONSOLE_BASE + NXXX_LPUART_DATA_OFFSET);

  /* Wait for the transmit register to be emptied. When the TXFE bit is
   * non-zero, the TX Buffer FIFO is empty.
   */

  while ((getreg32(NXXX_CONSOLE_BASE + NXXX_LPUART_STAT_OFFSET) &
         LPUART_STAT_TDRE) == 0)
    {
    }
#endif
}
