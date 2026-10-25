/****************************************************************************
 * arch/risc-v/src/ab32vg1/ab32vg1_lowputc.c
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

#include "riscv_internal.h"
#include "chip.h"
#include "ab32vg1.h"
#include "ab32vg1_lowputc.h"
#include "hardware/ab32vg1_uart.h"
#include "hardware/ab32vg1_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_UART0_SERIAL_CONSOLE
#  define HAVE_CONSOLE  1
#endif

#define UART0_CON       (AB32VG1_UART0_BASE + AB32VG1_UART_CON_OFFSET)
#define UART0_CPND      (AB32VG1_UART0_BASE + AB32VG1_UART_CPND_OFFSET)
#define UART0_BAUD      (AB32VG1_UART0_BASE + AB32VG1_UART_BAUD_OFFSET)
#define UART0_DATA      (AB32VG1_UART0_BASE + AB32VG1_UART_DATA_OFFSET)

/* PA7 is the UART0 TX pin of pin group G1 */

#define UART0_PIN       (1 << 7)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ab32vg1_uart0_configure
 ****************************************************************************/

void ab32vg1_uart0_configure(uint32_t baud, bool stop2)
{
  uint32_t div = (UART_INC_FREQUENCY + baud / 2) / baud - 1;
  uint32_t con;

  putreg32(0, UART0_CON);

  /* Clocks: uart_inc and the UART0 gate */

  modifyreg32(AB32VG1_CLKCON1, 0, CLKCON1_UART_INC);
  modifyreg32(AB32VG1_CLKGAT0, 0, CLKGAT0_UART0);

  /* PA7: digital input with pull-up, driven by the peripheral */

  modifyreg32(AB32VG1_GPIOA_BASE + AB32VG1_GPIO_FEN_OFFSET, 0, UART0_PIN);
  modifyreg32(AB32VG1_GPIOA_BASE + AB32VG1_GPIO_DE_OFFSET, 0, UART0_PIN);
  modifyreg32(AB32VG1_GPIOA_BASE + AB32VG1_GPIO_DIR_OFFSET, 0, UART0_PIN);
  modifyreg32(AB32VG1_GPIOA_BASE + AB32VG1_GPIO_PD_OFFSET, UART0_PIN, 0);
  modifyreg32(AB32VG1_GPIOA_BASE + AB32VG1_GPIO_PU_OFFSET, 0, UART0_PIN);

  /* TX0 on group G1 (PA7), RX0 on the TX pin */

  putreg32((FUNCMCON_CLEAR << FUNCMCON0_UT0RXMAP_SHIFT) |
           (FUNCMCON_CLEAR << FUNCMCON0_UT0TXMAP_SHIFT),
           AB32VG1_FUNCMCON0);
  putreg32((FUNCMCON_RX_ON_TX << FUNCMCON0_UT0RXMAP_SHIFT) |
           (FUNCMCON_G(1) << FUNCMCON0_UT0TXMAP_SHIFT),
           AB32VG1_FUNCMCON0);

  putreg32(UART_BAUD(div), UART0_BAUD);

  con = UART_CON_RXEN | UART_CON_ONELINE | UART_CON_CLKSRC | UART_CON_UTEN;
  if (stop2)
    {
      con |= UART_CON_SB2EN;
    }

  putreg32(UART_CPND_CRXPND | UART_CPND_CTXPND, UART0_CPND);
  putreg32(con, UART0_CON);
}

/****************************************************************************
 * Name: riscv_lowputc
 *
 * Description:
 *   Output one byte on the serial console.  TXPND is set when a byte has
 *   been shifted out and cleared by writing the data register; it reads 0
 *   before the first byte, so wait after writing rather than before.
 *
 ****************************************************************************/

void riscv_lowputc(char ch)
{
#ifdef HAVE_CONSOLE
  putreg32((uint8_t)ch, UART0_DATA);
  while ((getreg32(UART0_CON) & UART_CON_TXPND) == 0)
    {
    }
#endif
}

/****************************************************************************
 * Name: ab32vg1_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void ab32vg1_lowsetup(void)
{
#ifdef HAVE_CONSOLE
  ab32vg1_uart0_configure(CONFIG_UART0_BAUD, CONFIG_UART0_2STOP != 0);
#endif
}
