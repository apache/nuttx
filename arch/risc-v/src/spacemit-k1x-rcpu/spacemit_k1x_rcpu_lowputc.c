/****************************************************************************
 * arch/risc-v/src/spacemit-k1x-rcpu/spacemit_k1x_rcpu_lowputc.c
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

#include <nuttx/arch.h>

#include "riscv_internal.h"
#include "spacemit_k1x_rcpu_lowputc.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The PXA variant of the 16550 holds the whole block disabled until
 * IER.UUE is set, so every access to the block has to go through here.  A
 * plain 16550 write would configure a transmitter that never enables.
 */

#define SPACEMIT_K1X_RCPU_UART_ENABLE(base) \
  putreg32(getreg32((base) + SPACEMIT_K1X_RCPU_UART_IER_OFFSET) | SPACEMIT_K1X_RCPU_UART_IER_UUE, \
           (base) + SPACEMIT_K1X_RCPU_UART_IER_OFFSET)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_16550_UART0

/* Statically initialised so that riscv_lowputc() addresses a real UART even
 * before spacemit_k1x_rcpu_lowsetup() has run.  Without it the base is zero
 * until lowsetup(), and an early riscv_lowputc() polls the line status of
 * address 0x14 forever.  That failure was observed on hardware; see the call
 * ordering comment in spacemit_k1x_rcpu_start.c.
 *
 * The divisor and the frame format are only programmed by lowsetup(), so
 * output produced before that point is emitted at whatever the UART reset
 * state happens to be.
 */

static struct spacemit_k1x_rcpu_uart_config_s g_console_config =
{
  .base     = SPACEMIT_K1X_RCPU_RUART0_BASE,
  .baud     = CONFIG_SPACEMIT_K1X_RCPU_CONSOLE_BAUD,
  .clock    = CONFIG_SPACEMIT_K1X_RCPU_CONSOLE_CLOCK,
  .parity   = 0,
  .bits     = 8,
  .stopbits = 1
};

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spacemit_k1x_rcpu_uart_configure
 *
 * Description:
 *   Program the divisor latch and the frame format of one UART.
 *
 ****************************************************************************/

void spacemit_k1x_rcpu_uart_configure(
  const struct spacemit_k1x_rcpu_uart_config_s *config)
{
  uint32_t base = config->base;
  uint32_t divisor;
  uint32_t lcr;

  /* Divisor = fclk / (16 * baud).  Round to nearest: at 115200 baud an
   * unrounded divisor shifts the bit clock by up to 0.4 percent, which is
   * within tolerance but wastes margin over a long cable.
   */

  divisor = (config->clock + (config->baud * 8)) / (config->baud * 16);

  /* Safe minimum, and it must fit the 16 bit latch */

  if (divisor == 0)
    {
      divisor = 1;
    }
  else if (divisor > 0xffff)
    {
      divisor = 0xffff;
    }

  /* Start from a known state: FIFOs off, block enabled, no interrupts.
   *
   * The block has to be enabled (IER.UUE) before anything else in it can be
   * trusted, but a disabled block still reads back normally, so forgetting
   * this produces no diagnostic at all.
   */

  putreg32(0, base + SPACEMIT_K1X_RCPU_UART_FCR_OFFSET);
  /* UUE enables the block; RTOIE is set here as well so that the polled
   * path leaves the receiver time-out enabled for the interrupt driven
   * driver that takes the port over later.
   */

  putreg32(SPACEMIT_K1X_RCPU_UART_IER_UUE |
           SPACEMIT_K1X_RCPU_UART_IER_RTOIE,
           base + SPACEMIT_K1X_RCPU_UART_IER_OFFSET);

  lcr = SPACEMIT_K1X_RCPU_UART_LCR_WLS(config->bits - 5);

  if (config->stopbits == 2)
    {
      lcr |= SPACEMIT_K1X_RCPU_UART_LCR_STB;
    }

  if (config->parity != 0)
    {
      lcr |= SPACEMIT_K1X_RCPU_UART_LCR_PEN;

      if (config->parity == 2)
        {
          lcr |= SPACEMIT_K1X_RCPU_UART_LCR_EPS;
        }
    }

  /* Set the divisor through the DLAB window */

  putreg32(lcr | SPACEMIT_K1X_RCPU_UART_LCR_DLAB,
           base + SPACEMIT_K1X_RCPU_UART_LCR_OFFSET);
  putreg32(divisor & 0xff, base + SPACEMIT_K1X_RCPU_UART_DLL_OFFSET);
  putreg32((divisor >> 8) & 0xff, base + SPACEMIT_K1X_RCPU_UART_DLH_OFFSET);
  putreg32(lcr, base + SPACEMIT_K1X_RCPU_UART_LCR_OFFSET);

  /* MCR.OUT2 gates the transmitter and the interrupt output on this PXA
   * variant.  It is not a modem signal here.  With MCR left at its reset
   * value, THR writes are accepted and silently dropped: the transmitter
   * looks healthy, LSR.THRE even sets, and nothing ever leaves the pin.
   */

  putreg32(SPACEMIT_K1X_RCPU_UART_MCR_OUT2,
           base + SPACEMIT_K1X_RCPU_UART_MCR_OFFSET);

  /* FIFOs on and cleared, one byte receive trigger.
   *
   * BUS32 is deliberately not set.  It makes the block treat the FIFO ports
   * as 32 bit wide, so each write of one character ends up shifting four
   * bytes onto the wire and the terminal shows every character followed by
   * three NULs.  Measured on hardware, not inferred from the name.
   */

  putreg32(SPACEMIT_K1X_RCPU_UART_FCR_FIFO_EN |
           SPACEMIT_K1X_RCPU_UART_FCR_RXCLR |
           SPACEMIT_K1X_RCPU_UART_FCR_TXCLR |
           SPACEMIT_K1X_RCPU_UART_FCR_TRIG(0),
           base + SPACEMIT_K1X_RCPU_UART_FCR_OFFSET);
}

/****************************************************************************
 * Name: spacemit_k1x_rcpu_lowsetup
 *
 * Description:
 *   Configure the console UART so that riscv_lowputc() can be used.
 *
 ****************************************************************************/

void spacemit_k1x_rcpu_lowsetup(void)
{
#ifdef CONFIG_16550_UART0
  spacemit_k1x_rcpu_uart_configure(&g_console_config);

  /* riscv_lowputc() already expands a newline into CRLF, so emit the bare
   * newline here rather than a full CRLF sequence.
   */

  riscv_lowputs("\n");
#endif
}

/****************************************************************************
 * Name: riscv_lowputc
 *
 * Description:
 *   Write one character to the console, polled.  This is the earliest output
 *   path and has no locking: it is used before the scheduler exists.
 *
 ****************************************************************************/

#ifdef CONFIG_16550_UART0
void riscv_lowputc(char ch)
{
  uint32_t base = g_console_config.base;

  /* Map a bare newline to CRLF so that a serial terminal starts each line at
   * column zero.  Doing it here rather than at every call site keeps the
   * printable strings free of carriage returns.
   */

  while ((getreg32(base + SPACEMIT_K1X_RCPU_UART_LSR_OFFSET) &
          SPACEMIT_K1X_RCPU_UART_LSR_THRE) == 0)
    {
    }

  if (ch == '\n')
    {
      putreg32('\r', base + SPACEMIT_K1X_RCPU_UART_THR_OFFSET);

      while ((getreg32(base + SPACEMIT_K1X_RCPU_UART_LSR_OFFSET) &
          SPACEMIT_K1X_RCPU_UART_LSR_THRE) == 0)
        {
        }
    }

  putreg32((uint32_t)(uint8_t)ch, base + SPACEMIT_K1X_RCPU_UART_THR_OFFSET);
}
#endif

/****************************************************************************
 * Name: riscv_lowputs
 ****************************************************************************/

#ifdef CONFIG_16550_UART0
void riscv_lowputs(const char *str)
{
  while (*str != '\0')
    {
      riscv_lowputc(*str++);
    }
}
#endif

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Console output used by the syslog and assertion paths.
 *
 * Normally this belongs to whichever serial driver owns the console.  This
 * configuration has no interrupt driven serial driver (see
 * the defconfig), so the polled early console is the console
 * and this is a thin wrapper over it.  It is compiled only when there is no
 * serial driver, so that adding one later does not produce a duplicate
 * symbol.
 *
 ****************************************************************************/

#if defined(CONFIG_16550_UART0) && !defined(USE_SERIALDRIVER)
void up_putc(int ch)
{
  /* The lowputc layer expands a newline into CRLF, so a '\r' passed in by a
   * caller that already writes CRLF would double it.  Swallow a bare CR
   * instead of transmitting it.
   */

  if (ch != '\r')
    {
      riscv_lowputc((char)ch);
    }
}
#endif
