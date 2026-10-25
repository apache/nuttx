/****************************************************************************
 * boards/risc-v/ab32vg1/ab32vg1-generic/tools/target/util.c
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

#include "target.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_EN         BIT(0)
#define UART_CLKSRC     BIT(5)          /* uart_inc = 26 MHz / 2 */
#define UART_ONELINE    BIT(6)
#define UART_RXEN       BIT(7)
#define UART_TXPND      BIT(8)
#define UART_RXPND      BIT(9)

#define UART_PIN        BIT(7)          /* PA7, TX0 of pin group G1 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* UART0 in single-wire mode on PA7 (same setup as the NuttX driver) */

void uart_init(uint32_t baud)
{
  uint32_t div = (13000000 + baud / 2) / baud - 1;

  UART0CON  = 0;
  CLKCON1  |= BIT(14);
  CLKGAT0  |= BIT(10);
  GPIOAFEN |= UART_PIN;
  GPIOADE  |= UART_PIN;
  GPIOADIR |= UART_PIN;
  GPIOAPD  &= ~UART_PIN;
  GPIOAPU  |= UART_PIN;
  FUNCMCON0 = (0xf << 12) | (0xf << 8);
  FUNCMCON0 = (0x7 << 12) | (0x1 << 8);
  UART0BAUD = (div << 16) | div;
  UART0CPND = UART_RXPND | UART_TXPND;
  UART0CON  = UART_RXEN | UART_ONELINE | UART_CLKSRC | UART_EN;
}

void uart_putc(int ch)
{
  UART0DATA = (uint8_t)ch;
  while ((UART0CON & UART_TXPND) == 0)
    {
    }
}

void uart_puts(const char *s)
{
  while (*s)
    {
      uart_putc(*s++);
    }
}

int uart_getc(void)
{
  int ch;

  while ((UART0CON & UART_RXPND) == 0)
    {
    }

  ch = UART0DATA & 0xff;
  UART0CPND = UART_RXPND;
  return ch;
}

int uart_trygetc(void)
{
  int ch;

  if ((UART0CON & UART_RXPND) == 0)
    {
      return -1;
    }

  ch = UART0DATA & 0xff;
  UART0CPND = UART_RXPND;
  return ch;
}

/* CRC-32 (IEEE 802.3, reflected, as zlib) without a table */

uint32_t crc32_update(uint32_t crc, const uint8_t *buf, uint32_t len)
{
  crc = ~crc;
  while (len-- > 0)
    {
      crc ^= *buf++;
      for (int i = 0; i < 8; i++)
        {
          crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1));
        }
    }

  return ~crc;
}

/* CRC-16 used by the mask ROM protocol: poly 0x1021, MSB first */

uint16_t crc16_update(uint16_t crc, const uint8_t *buf, uint32_t len)
{
  while (len-- > 0)
    {
      crc ^= (uint16_t)*buf++ << 8;
      for (int i = 0; i < 8; i++)
        {
          crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }

  return crc;
}
