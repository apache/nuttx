/****************************************************************************
 * boards/risc-v/ab32vg1/ab32vg1-generic/tools/target/spiflash.c
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

/* Minimal driver for the SPI NOR flash inside the AB32VG1 package.  The
 * flash is wired to SPI0 on port G (pin group G1): PG2 is the chip select,
 * PG0 and PG3 (WP#/HOLD#) are driven high, PG4/PG5 are the clock and data
 * out, PG1 is data in.  Transfers are one byte at a time.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "target.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI0CON_EN      BIT(0)
#define SPI0CON_RX      BIT(4)
#define SPI0CON_DONE    BIT(16)

#define CS_PIN          BIT(2)

#define CMD_WRSR        0x01
#define CMD_PP          0x02
#define CMD_READ        0x03
#define CMD_RDSR        0x05
#define CMD_WREN        0x06
#define CMD_SE          0x20            /* 4 KB sector erase */
#define CMD_BE          0xd8            /* 64 KB block erase */
#define CMD_RDID        0x9f

#define SR_WIP          BIT(0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sf_xfer_wait(void)
{
  while ((SPI0CON & SPI0CON_DONE) == 0)
    {
    }

  SPI0CPND = SPI0CON_DONE;
}

static void sf_send(uint8_t b)
{
  SPI0CON &= ~SPI0CON_RX;
  SPI0BUF = b;
  sf_xfer_wait();
}

static uint8_t sf_recv(void)
{
  SPI0CON |= SPI0CON_RX;
  SPI0BUF = 0xff;
  sf_xfer_wait();
  return SPI0BUF & 0xff;
}

static void sf_select(void)
{
  GPIOGCLR = CS_PIN;
}

static void sf_deselect(void)
{
  GPIOGSET = CS_PIN;
}

static void sf_cmd_addr(uint8_t cmd, uint32_t addr)
{
  sf_select();
  sf_send(cmd);
  sf_send(addr >> 16);
  sf_send(addr >> 8);
  sf_send(addr);
}

static void sf_write_enable(void)
{
  sf_select();
  sf_send(CMD_WREN);
  sf_deselect();
}

static void sf_wait_ready(void)
{
  while (sf_status() & SR_WIP)
    {
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sf_init(uint32_t baud)
{
  SPI0CON = 0;

  /* PG2 (CS) high, PG0/PG3 high, all three GPIO outputs */

  GPIOGSET  = CS_PIN;
  GPIOGDIR &= ~CS_PIN;
  GPIOGSET  = BIT(0) | BIT(3);
  GPIOGDIR &= ~(BIT(0) | BIT(3));

  /* SPI0 on pin group G1: PG4/PG5 outputs, PG1 input with pull-up */

  FUNCMCON0 = 0xf << 4;
  FUNCMCON0 = 0x1 << 4;
  GPIOGFEN |= BIT(1) | BIT(4) | BIT(5);
  GPIOGDIR &= ~(BIT(4) | BIT(5));
  GPIOGPU  |= BIT(1);
  GPIOGDIR |= BIT(1);

  /* SPI clock = sysclk / (baud + 1), plain SPI, on-the-fly descrambling
   * and CRC off.
   */

  SPI0BAUD = baud;
  SPI0CON |= SPI0CON_EN;
  CRCCON  &= ~5u;

  /* Leave any continuous read mode the mask ROM may have used: 0xff on
   * the bus for more than one address phase is not a valid command.
   */

  sf_select();
  for (int i = 0; i < 8; i++)
    {
      sf_send(0xff);
    }

  sf_deselect();
}

uint32_t sf_readid(void)
{
  uint32_t id;

  sf_select();
  sf_send(CMD_RDID);
  id  = (uint32_t)sf_recv() << 16;
  id |= (uint32_t)sf_recv() << 8;
  id |= sf_recv();
  sf_deselect();
  return id;
}

uint8_t sf_status(void)
{
  uint8_t sr;

  sf_select();
  sf_send(CMD_RDSR);
  sr = sf_recv();
  sf_deselect();
  return sr;
}

void sf_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
  sf_cmd_addr(CMD_READ, addr);
  while (len-- > 0)
    {
      *buf++ = sf_recv();
    }

  sf_deselect();
}

void sf_erase(uint32_t addr, bool block64k)
{
  sf_write_enable();
  sf_cmd_addr(block64k ? CMD_BE : CMD_SE, addr);
  sf_deselect();
  sf_wait_ready();
}

void sf_program(uint32_t addr, const uint8_t *buf, uint32_t len)
{
  uint32_t n;

  while (len > 0)
    {
      /* A page program must not cross a 256 byte page boundary */

      n = 256 - (addr & 255);
      if (n > len)
        {
          n = len;
        }

      sf_write_enable();
      sf_cmd_addr(CMD_PP, addr);
      for (uint32_t i = 0; i < n; i++)
        {
          sf_send(buf[i]);
        }

      sf_deselect();
      sf_wait_ready();

      addr += n;
      buf  += n;
      len  -= n;
    }
}
