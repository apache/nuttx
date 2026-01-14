/****************************************************************************
 * arch/arm/src/csk6/csk6_lowputc.c
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

#include <arch/board/board.h>

#include "arm_internal.h"

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CSK6_IOMUX_BASE 0x46200000       // size=1MB
#define CSK6_SYSCTRL_BASE 0x46000000     // size=64KB
#define CSK6_SYSPLL_CTRL_BASE 0x46020000 // size=64KB
#define CSK6_UART0_BASE 0x45000000       // size=1MB

#define CSK6011A_NANO_BOARD_H_XTAL_SRC_FREQ 24000000UL
#define CSK6011A_NANO_BOARD_L_XTAL_SRC_FREQ 32768UL

#define CSK6_IOMUX_PIN_OFFSET(pin) (pin << 2)
#define CSK6_IOMUX_FSEL_MASK (0X0f)
#define CSK6_IOMUX_FSEL_UART0 (0x2)
#define CSK6_UART0_TX_GPIO_PIN (2)
#define CSK6_UART0_RX_GPIO_PIN (3)

#define REG_FIELD_MASK(w, s) (((1U << (w)) - 1) << (s))
#define REG_FIELD_EXTRACT(v, w, s) ((v & REG_FIELD_MASK(w, s)) >> s)
#define REG_VALUE_SHIFT(v, s) (v << s)

/* Select USART parameters for the selected console */
#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#define HAVE_CONSOLE
#define CSK6_CONSOLE_BASE() ((DW_UART_RegDef *)UART0_BASE)
#define CSK6_CONSOLE_BAUD CONFIG_USART0_BAUD
#define CSK6_CONSOLE_PARITY CONFIG_USART0_PARITY
#define CSK6_CONSOLE_NBITS CONFIG_USART0_BITS
#define CSK6_CONSOLE_2STOP CONFIG_USART0_2STOP
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline float uart_compute_div(uint32_t baudrate, uint32_t clk)
{
  uint32_t tmp = 16 * baudrate;
  uint32_t div_i = clk / tmp;

  float div_f = ((float)clk / (float)tmp) - div_i;
  float div1 = div_i + (float)((uint32_t)(div_f * 16)) / 16.0;
  float div2 = div_i + (float)((uint32_t)(div_f * 16) + 1) / 16.0;

  int32_t err = clk - div1 * tmp;
  int32_t err2 = div2 * tmp - clk;

  if (err2 < 0)
    {
      err2 = err2 * (-1);
    }

  return err > err2 ? div2 : div1;
}

static inline void csk6_lowsetup_gpio_init(void)
{
  modreg32(CSK6_IOMUX_FSEL_UART0,
        CSK6_IOMUX_FSEL_MASK,
        CSK6_IOMUX_BASE + CSK6_IOMUX_PIN_OFFSET(CSK6_UART0_TX_GPIO_PIN));

  modreg32(CSK6_IOMUX_FSEL_UART0,
        CSK6_IOMUX_FSEL_MASK,
        CSK6_IOMUX_BASE + CSK6_IOMUX_PIN_OFFSET(CSK6_UART0_RX_GPIO_PIN));
}

static inline uint32_t get_uart0_clock(void)
{
  uint32_t freq;
  uint32_t pll_freq;
  uint8_t div_n;
  uint8_t div_m;
  uint32_t reg_val;

  reg_val = getreg32(CSK6_SYSCTRL_BASE + 0x3c);
  div_n = REG_FIELD_EXTRACT(reg_val, 5, 25);
  div_m = REG_FIELD_EXTRACT(reg_val, 5, 20) + 1;

  if (!REG_FIELD_EXTRACT(reg_val, 1, 16)) /* CRM_IpSrcXtal */
    {
      freq = CSK6011A_NANO_BOARD_H_XTAL_SRC_FREQ;
    }
  else /* CRM_IpSrcSysPllPeri */
    {
      uint32_t div;
      reg_val = getreg32(CSK6_SYSCTRL_BASE + 0x30);
      div = REG_FIELD_EXTRACT(reg_val, 2, 29) + 10;
      pll_freq = CSK6011A_NANO_BOARD_H_XTAL_SRC_FREQ *
          REG_FIELD_EXTRACT(getreg32(CSK6_SYSPLL_CTRL_BASE + 0x14), 7, 1) /
          (REG_FIELD_EXTRACT(
            getreg32(CSK6_SYSPLL_CTRL_BASE + 0x14), 4, 8)
            + 1);
      freq = pll_freq / div;
    }

  return freq * div_n / div_m;
}

static inline void csk6_uart_clock_enable(void)
{
  modreg32(REG_VALUE_SHIFT(0x01, 19),
        REG_FIELD_MASK(1, 19),
        CSK6_SYSCTRL_BASE + 0x3c);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(HAVE_CONSOLE)

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
  while (!REG_FIELD_EXTRACT(getreg32(CSK6_UART0_BASE + 0x14), 1, 6))
    {
    }

  putreg32(ch, CSK6_UART0_BASE + 0x00);
}

/****************************************************************************
 * Name: csk6_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void csk6_lowsetup(void)
{
  uint32_t uart_clock = 0;
  float div;
  uint32_t div_i;
  uint32_t div_f;

  csk6_lowsetup_gpio_init();
  csk6_uart_clock_enable();
  uart_clock = get_uart0_clock();

  /* disable all */

  putreg32(0, CSK6_UART0_BASE + 0x04);

  div = uart_compute_div(CSK6_CONSOLE_BAUD, uart_clock);
  div_i = (uint32_t)div;
  div_f = (div - div_i) * 16;

  modreg32(REG_VALUE_SHIFT(0x01, 7),
    REG_FIELD_MASK(1, 7),
    CSK6_UART0_BASE + 0x0c);

  modreg32(REG_VALUE_SHIFT(div_i, 0),
    REG_FIELD_MASK(8, 0),
    CSK6_UART0_BASE + 0x00);

  modreg32(REG_VALUE_SHIFT(div_i >> 8, 0),
    REG_FIELD_MASK(8, 0),
    CSK6_UART0_BASE + 0x04);

  modreg32(REG_VALUE_SHIFT(div_f, 0),
    REG_FIELD_MASK(4, 0),
    CSK6_UART0_BASE + 0xc0);

  modreg32(REG_VALUE_SHIFT(0x00, 0),
    REG_FIELD_MASK(1, 7),
    CSK6_UART0_BASE + 0x0c);

#if (CSK6_CONSOLE_PARITY == 1)
  modreg32(REG_VALUE_SHIFT(0x01, 3),
    REG_FIELD_MASK(1, 3),
    CSK6_UART0_BASE + 0x0c);

  modreg32(REG_VALUE_SHIFT(0x01, 3),
    REG_FIELD_MASK(1, 4),
    CSK6_UART0_BASE + 0x0c);

#elif (CSK6_CONSOLE_PARITY == 2)
  modreg32(REG_VALUE_SHIFT(0x01, 3),
    REG_FIELD_MASK(1, 3),
    CSK6_UART0_BASE + 0x0c);

  modreg32(REG_VALUE_SHIFT(0x00, 3),
    REG_FIELD_MASK(1, 4),
    CSK6_UART0_BASE + 0x0c);

#else
  modreg32(REG_VALUE_SHIFT(0x00, 3),
    REG_FIELD_MASK(1, 3),
    CSK6_UART0_BASE + 0x0c);
#endif

#if (CSK6_CONSOLE_NBITS == 5)
  modreg32(REG_VALUE_SHIFT(0x00, 0),
    REG_FIELD_MASK(2, 0),
    CSK6_UART0_BASE + 0x0c);
#elif (CSK6_CONSOLE_NBITS == 6)
  modreg32(REG_VALUE_SHIFT(0x00, 0),
    REG_FIELD_MASK(2, 0),
    CSK6_UART0_BASE + 0x0c);
#elif (CSK6_CONSOLE_NBITS == 7)
  modreg32(REG_VALUE_SHIFT(0x00, 0),
    REG_FIELD_MASK(2, 0),
    CSK6_UART0_BASE + 0x0c);
#else
  modreg32(REG_VALUE_SHIFT(0x03, 0),
    REG_FIELD_MASK(2, 0),
    CSK6_UART0_BASE + 0x0c);
#endif

#if (CSK6_CONSOLE_2STOP)
  modreg32(REG_VALUE_SHIFT(0x01, 2),
    REG_FIELD_MASK(1, 2),
    CSK6_UART0_BASE + 0x0c);
#else
  modreg32(REG_VALUE_SHIFT(0x00, 2),
    REG_FIELD_MASK(1, 2),
    CSK6_UART0_BASE + 0x0c);
#endif

  putreg32(REG_VALUE_SHIFT(0x02, 4) | REG_VALUE_SHIFT(0x01, 0),
    CSK6_UART0_BASE + 0x08); /* set fifo */
}

#else

void arm_lowputc(char ch)
{
}

void csk6_lowsetup(void)
{
}

#endif
