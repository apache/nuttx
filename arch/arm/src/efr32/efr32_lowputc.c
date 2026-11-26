/****************************************************************************
 * arch/arm/src/efr32/efr32_lowputc.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "efr32_config.h"
#include "efr32_gpio.h"
#include "efr32_lowputc.h"
#include "hardware/efr32_cmu.h"
#include "hardware/efr32_gpio.h"
#include "hardware/efr32_usart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_USART0_SERIAL_CONSOLE)
#    define CONSOLE_BASE     EFR32_USART0_BASE
#    define CONSOLE_BAUD     CONFIG_USART0_BAUD
#    define CONSOLE_PARITY   CONFIG_USART0_PARITY
#    define CONSOLE_BITS     CONFIG_USART0_BITS
#    define CONSOLE_2STOP    CONFIG_USART0_2STOP
#  endif
#endif

/* The USART is clocked by PCLK and oversamples 16 times */

#define USART_OVERSAMPLE     16

#ifdef CONFIG_EFR32_USART0
#  if !defined(GPIO_USART0_TX) || !defined(GPIO_USART0_RX)
#    error "board.h must define GPIO_USART0_TX and GPIO_USART0_RX"
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console.
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Wait until the transmit buffer has room */

  while ((getreg32(CONSOLE_BASE + EFR32_USART_STATUS_OFFSET) &
          USART_STATUS_TXBL) == 0)
    {
    }

  putreg32((uint32_t)ch, CONSOLE_BASE + EFR32_USART_TXDATA_OFFSET);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes.
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  arm_lowputc(ch);
#endif
}

/****************************************************************************
 * Name: efr32_usart_reset
 *
 * Description:
 *   Disable a USART and return its registers to the reset state.
 *
 ****************************************************************************/

void efr32_usart_reset(uintptr_t base)
{
  putreg32(USART_CMD_RXDIS | USART_CMD_TXDIS | USART_CMD_CLEARRX |
           USART_CMD_CLEARTX, base + EFR32_USART_CMD_OFFSET);

  /* The interrupt flags are only cleared through the CLR alias */

  putreg32(0, base + EFR32_USART_IEN_OFFSET);
  putreg32(USART_INT_ALL, base + EFR32_USART_IF_OFFSET + EFR32_CLR_OFFSET);

  /* Disable the module.  efr32_usart_configure() rewrites CTRL, FRAME
   * and CLKDIV after enabling it again.
   */

  putreg32(0, base + EFR32_USART_EN_OFFSET);
}

/****************************************************************************
 * Name: efr32_usart_configure
 *
 * Description:
 *   Configure a USART for asynchronous operation (baud rate, data bits,
 *   parity and stop bits) and enable its receiver and transmitter.  The
 *   USART must be disabled or idle.
 *
 ****************************************************************************/

void efr32_usart_configure(uintptr_t base, uint32_t baud,
                           unsigned int parity, unsigned int bits,
                           bool stop2)
{
  uint64_t clkdiv;
  uint32_t frame;

  efr32_usart_reset(base);
  putreg32(USART_EN_EN, base + EFR32_USART_EN_OFFSET);

  /* Asynchronous mode, 16x oversampling, TXBL set while the transmit
   * buffer has room for at least one frame.
   */

  putreg32(USART_CTRL_OVS_X16 | USART_CTRL_TXBIL,
           base + EFR32_USART_CTRL_OFFSET);

  /* Frame format */

  frame = USART_FRAME_DATABITS(bits);
  frame |= parity == 1 ? USART_FRAME_PARITY_ODD :
           parity == 2 ? USART_FRAME_PARITY_EVEN : USART_FRAME_PARITY_NONE;
  frame |= stop2 ? USART_FRAME_STOPBITS_TWO : USART_FRAME_STOPBITS_ONE;
  putreg32(frame, base + EFR32_USART_FRAME_OFFSET);

  /* baud = PCLK / (OVS * (1 + CLKDIV / 256)), where CLKDIV has five
   * fractional bits in DIV[7:3].  Round to the nearest divider.
   */

  clkdiv  = (uint64_t)32 * BOARD_PCLK_FREQUENCY +
            (USART_OVERSAMPLE * baud) / 2;
  clkdiv /= (uint64_t)USART_OVERSAMPLE * baud;
  clkdiv  = (clkdiv > 32 ? clkdiv - 32 : 0) << USART_CLKDIV_DIV_SHIFT;
  putreg32((uint32_t)clkdiv & USART_CLKDIV_DIV_MASK,
           base + EFR32_USART_CLKDIV_OFFSET);

  /* Enable the receiver and the transmitter */

  putreg32(USART_CMD_CLEARRX | USART_CMD_CLEARTX,
           base + EFR32_USART_CMD_OFFSET);
  putreg32(USART_CMD_RXEN | USART_CMD_TXEN, base + EFR32_USART_CMD_OFFSET);
}

/****************************************************************************
 * Name: efr32_lowsetup
 *
 * Description:
 *   Called early in the boot sequence, after efr32_clockconfig(), to
 *   enable the USART clocks, route the board's pins and configure the
 *   serial console so that debug output is available.
 *
 ****************************************************************************/

void efr32_lowsetup(void)
{
#ifdef CONFIG_EFR32_USART0
  /* Clock USART0, then route its RX and TX to the board's pins */

  putreg32(CMU_CLKEN0_USART0, EFR32_CMU_CLKEN0_SET);

  efr32_configgpio(GPIO_USART0_TX);
  efr32_configgpio(GPIO_USART0_RX);

  putreg32(GPIO_ROUTE(GPIO_PORTNO(GPIO_USART0_TX),
                      GPIO_PINNO(GPIO_USART0_TX)),
           EFR32_GPIO_USART0_TXROUTE);
  putreg32(GPIO_ROUTE(GPIO_PORTNO(GPIO_USART0_RX),
                      GPIO_PINNO(GPIO_USART0_RX)),
           EFR32_GPIO_USART0_RXROUTE);
  putreg32(GPIO_USART_ROUTEEN_TXPEN | GPIO_USART_ROUTEEN_RXPEN,
           EFR32_GPIO_USART0_ROUTEEN);
#endif

#ifdef HAVE_SERIAL_CONSOLE
  efr32_usart_configure(CONSOLE_BASE, CONSOLE_BAUD, CONSOLE_PARITY,
                        CONSOLE_BITS, CONSOLE_2STOP != 0);
#endif
}
