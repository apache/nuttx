/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_lowputc.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <fixedmath.h>
#include <assert.h>
#include <errno.h>

#include "hardware/s32k3xx_pinmux.h"
#include "hardware/s32k3xx_lpuart.h"

#include "s32k3xx_config.h"
#include "s32k3xx_pin.h"
#include "s32k3xx_lowputc.h"
#include "s32k3xx_clockconfig.h"
#include "s32k3xx_periphclocks.h"

#include "arm_internal.h"

#include <arch/board/board.h> /* Include last:  has dependencies */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef HAVE_LPUART_CONSOLE
#  if defined(CONFIG_LPUART0_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART0_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART0_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART0_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART0_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART0_2STOP
#  elif defined(CONFIG_LPUART1_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART1_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART1_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART1_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART1_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART1_2STOP
#  elif defined(CONFIG_LPUART2_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART2_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART2_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART2_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART2_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART2_2STOP
#  elif defined(CONFIG_LPUART3_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART3_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART3_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART3_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART3_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART3_2STOP
#  elif defined(CONFIG_LPUART4_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART4_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART4_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART4_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART4_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART4_2STOP
#  elif defined(CONFIG_LPUART5_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART5_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART5_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART5_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART5_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART5_2STOP
#  elif defined(CONFIG_LPUART6_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART6_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART6_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART6_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART6_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART6_2STOP
#  elif defined(CONFIG_LPUART7_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART7_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART7_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART7_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART7_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART7_2STOP
#  elif defined(CONFIG_LPUART8_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART8_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART8_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART8_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART8_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART8_2STOP
#  elif defined(CONFIG_LPUART9_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART9_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART9_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART9_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART9_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART9_2STOP
#  elif defined(CONFIG_LPUART10_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART10_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART10_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART10_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART10_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART10_2STOP
#  elif defined(CONFIG_LPUART11_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART11_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART11_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART11_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART11_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART11_2STOP
#  elif defined(CONFIG_LPUART12_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART12_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART12_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART12_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART12_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART12_2STOP
#  elif defined(CONFIG_LPUART13_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART13_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART13_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART13_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART13_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART13_2STOP
#  elif defined(CONFIG_LPUART14_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART14_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART14_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART14_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART14_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART14_2STOP
#  elif defined(CONFIG_LPUART15_SERIAL_CONSOLE)
#    define S32K3XX_CONSOLE_BASE     S32K3XX_LPUART15_BASE
#    define S32K3XX_CONSOLE_BAUD     CONFIG_LPUART15_BAUD
#    define S32K3XX_CONSOLE_BITS     CONFIG_LPUART15_BITS
#    define S32K3XX_CONSOLE_PARITY   CONFIG_LPUART15_PARITY
#    define S32K3XX_CONSOLE_2STOP    CONFIG_LPUART15_2STOP
#  endif
#endif

/* Clocking *****************************************************************/

/* Functional clocking is provided via the  PCC.  The PCC clocking must
 * be configured by board-specific logic prior to using the LPUART.
 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_LPUART_CONSOLE
static const struct uart_config_s g_console_config =
{
  .baud      = S32K3XX_CONSOLE_BAUD,    /* Configured baud */
  .parity    = S32K3XX_CONSOLE_PARITY,  /* 0=none, 1=odd, 2=even */
  .bits      = S32K3XX_CONSOLE_BITS,    /* Number of bits (5-9) */
  .stopbits2 = S32K3XX_CONSOLE_2STOP,   /* true: Configure with 2 stop bits instead of 1 */
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization including setup of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot sequence.
 *
 ****************************************************************************/

void s32k3xx_lowsetup(void)
{
#ifndef CONFIG_SUPPRESS_LPUART_CONFIG
#ifdef HAVE_LPUART_DEVICE

#ifdef CONFIG_S32K3XX_LPUART0

  /* Configure LPUART0 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART0_RX);
  s32k3xx_pinconfig(PIN_LPUART0_TX);
#ifdef CONFIG_LPUART0_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART0_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART0_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART0_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART0_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART1

  /* Configure LPUART1 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART1_RX);
  s32k3xx_pinconfig(PIN_LPUART1_TX);
#ifdef CONFIG_LPUART1_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART1_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART1_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART1_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART1_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART2

  /* Configure LPUART2 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART2_RX);
  s32k3xx_pinconfig(PIN_LPUART2_TX);
#ifdef CONFIG_LPUART2_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART2_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART2_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART2_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART2_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART3

  /* Configure LPUART3 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART3_RX);
  s32k3xx_pinconfig(PIN_LPUART3_TX);
#ifdef CONFIG_LPUART3_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART3_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART3_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART3_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART3_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART4

  /* Configure LPUART4 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART4_RX);
  s32k3xx_pinconfig(PIN_LPUART4_TX);
#ifdef CONFIG_LPUART4_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART4_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART4_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART4_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART4_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART5

  /* Configure LPUART5 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART5_RX);
  s32k3xx_pinconfig(PIN_LPUART5_TX);
#ifdef CONFIG_LPUART5_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART5_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART5_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART5_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART5_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART6

  /* Configure LPUART6 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART6_RX);
  s32k3xx_pinconfig(PIN_LPUART6_TX);
#ifdef CONFIG_LPUART6_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART6_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART6_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART6_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART6_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART7

  /* Configure LPUART7 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART7_RX);
  s32k3xx_pinconfig(PIN_LPUART7_TX);
#ifdef CONFIG_LPUART7_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART7_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART7_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART7_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART7_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART8

  /* Configure LPUART8 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART8_RX);
  s32k3xx_pinconfig(PIN_LPUART8_TX);
#ifdef CONFIG_LPUART8_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART8_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART8_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART8_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART8_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART9

  /* Configure LPUART9 pins: RXD and TXD.
   * Also configure RTS and CTS if flow control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART9_RX);
  s32k3xx_pinconfig(PIN_LPUART9_TX);
#ifdef CONFIG_LPUART9_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART9_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART9_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART9_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART9_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART10

  /* Configure LPUART10 pins: RXD and TXD.
   * Also configure RTS and CTS if flow control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART10_RX);
  s32k3xx_pinconfig(PIN_LPUART10_TX);
#ifdef CONFIG_LPUART10_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART10_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART10_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART10_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART10_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART11

  /* Configure LPUART11 pins: RXD and TXD.
   * Also configure RTS and CTS if flow control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART11_RX);
  s32k3xx_pinconfig(PIN_LPUART11_TX);
#ifdef CONFIG_LPUART11_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART11_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART11_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART11_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART11_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART12

  /* Configure LPUART12 pins: RXD and TXD.
   * Also configure RTS and CTS if flow control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART12_RX);
  s32k3xx_pinconfig(PIN_LPUART12_TX);
#ifdef CONFIG_LPUART12_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART12_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART12_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART12_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART12_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART13

  /* Configure LPUART13 pins: RXD and TXD.
   * Also configure RTS and CTS if flow control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART13_RX);
  s32k3xx_pinconfig(PIN_LPUART13_TX);
#ifdef CONFIG_LPUART13_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART13_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART13_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART13_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART13_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART14

  /* Configure LPUART14 pins: RXD and TXD.
   * Also configure RTS and CTS if flow control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART14_RX);
  s32k3xx_pinconfig(PIN_LPUART14_TX);
#ifdef CONFIG_LPUART14_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART14_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART14_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART14_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART14_RTS);
#endif
#endif

#ifdef CONFIG_S32K3XX_LPUART15

  /* Configure LPUART15 pins: RXD and TXD.
   * Also configure RTS and CTS if flow control is enabled.
   */

  s32k3xx_pinconfig(PIN_LPUART15_RX);
  s32k3xx_pinconfig(PIN_LPUART15_TX);
#ifdef CONFIG_LPUART15_OFLOWCONTROL
  s32k3xx_pinconfig(PIN_LPUART15_CTS);
#endif
#if ((defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART15_RS485RTSCONTROL)) || \
     (defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART15_IFLOWCONTROL)))
  s32k3xx_pinconfig(PIN_LPUART15_RTS);
#endif
#endif

#ifdef HAVE_LPUART_CONSOLE
  /* Configure the serial console for initial, non-interrupt driver mode */

  s32k3xx_lpuart_configure(S32K3XX_CONSOLE_BASE, &g_console_config);
#endif
#endif /* HAVE_LPUART_DEVICE */
#endif /* CONFIG_SUPPRESS_LPUART_CONFIG */
}

/****************************************************************************
 * Name: s32k3xx_lpuart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ****************************************************************************/

#ifdef HAVE_LPUART_DEVICE
int s32k3xx_lpuart_configure(uint32_t base,
                             const struct uart_config_s *config)
{
  uint32_t lpuart_freq;
  uint16_t sbr;
  uint16_t temp_sbr;
  uint32_t osr;
  uint32_t temp_osr;
  uint32_t temp_diff;
  uint32_t calculated_baud;
  uint32_t baud_diff;
  uint32_t regval;

  /* Functional clocking is provided via the  PCC.  The PCC clocking must
   * be configured by board-specific logic prior to using the LPUART.
   */

  /* Get the PCC source clock */

#ifdef CONFIG_S32K3XX_LPUART0
  if (base == S32K3XX_LPUART0_BASE)
    {
      lpuart_freq = s32k3xx_get_freq(AIPS_PLAT_CLK);
    }
  else
#endif
#ifdef CONFIG_S32K3XX_LPUART8
  if (base == S32K3XX_LPUART8_BASE)
    {
      lpuart_freq = s32k3xx_get_freq(AIPS_PLAT_CLK);
    }
  else
#endif
    {
      lpuart_freq = s32k3xx_get_freq(AIPS_SLOW_CLK);
    }

  DEBUGASSERT(lpuart_freq >= 0);

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

      if (temp_diff >
          (config->baud - (lpuart_freq / (temp_osr * (temp_sbr + 1)))))
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

  /* Reset all internal logic and registers, except the Global Register */

  regval  = getreg32(base + S32K3XX_LPUART_GLOBAL_OFFSET);
  regval |= LPUART_GLOBAL_RST;
  putreg32(regval, base + S32K3XX_LPUART_GLOBAL_OFFSET);

  regval &= ~LPUART_GLOBAL_RST;
  putreg32(regval, base + S32K3XX_LPUART_GLOBAL_OFFSET);

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

  putreg32(regval, base + S32K3XX_LPUART_MODIR_OFFSET);

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
  putreg32(regval, base + S32K3XX_LPUART_BAUD_OFFSET);

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
  putreg32(regval, base + S32K3XX_LPUART_CTRL_OFFSET);

  return OK;
}
#endif /* HAVE_LPUART_DEVICE */

/****************************************************************************
 * Name: s32k3xx_lowputc
 *
 * Description:
 *   Output a byte with as few system dependencies as possible.  This will
 *   even work BEFORE the console is initialized if we are booting from U-
 *   Boot (and the same UART is used for the console, of course.)
 *
 ****************************************************************************/

#if defined(HAVE_LPUART_DEVICE) && defined(CONFIG_DEBUG_FEATURES)
void s32k3xx_lowputc(int ch)
{
  while ((getreg32(S32K3XX_CONSOLE_BASE + S32K3XX_LPUART_STAT_OFFSET) &
         LPUART_STAT_TDRE) == 0)
    {
    }

  /* If the character to output is a newline,
   * then pre-pend a carriage return
   */

  if (ch == '\n')
    {
      /* Send the carriage return by writing it into the UART_TXD register. */

      putreg32((uint32_t)'\r',
                S32K3XX_CONSOLE_BASE + S32K3XX_LPUART_DATA_OFFSET);

      /* Wait for the transmit register to be emptied. When the TXFE bit is
       * non-zero, the TX Buffer FIFO is empty.
       */

      while ((getreg32(S32K3XX_CONSOLE_BASE + S32K3XX_LPUART_STAT_OFFSET) &
             LPUART_STAT_TDRE) == 0)
        {
        }
    }

  /* Send the character by writing it into the UART_TXD register. */

  putreg32((uint32_t)ch, S32K3XX_CONSOLE_BASE + S32K3XX_LPUART_DATA_OFFSET);

  /* Wait for the transmit register to be emptied. When the TXFE bit is
   * non-zero, the TX Buffer FIFO is empty.
   */

  while ((getreg32(S32K3XX_CONSOLE_BASE + S32K3XX_LPUART_STAT_OFFSET) &
         LPUART_STAT_TDRE) == 0)
    {
    }
}
#endif
