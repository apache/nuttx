/****************************************************************************
 * boards/risc-v/ab32vg1/ab32vg1-generic/tools/target/stub.c
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

/* Flash programming stub.  Loaded into RAM by the mask ROM UART loader, it
 * takes over UART0 (single wire, PA7) and serves requests from the host
 * tool (ab32vg1_dl) until it is reset.
 *
 * Request:  5A cmd len16 addr32 payload[len] crc16
 * Response: A5 status len16 payload[len] crc16
 *
 * Multi-byte fields are little endian.  The CRC is the mask ROM CRC-16
 * (poly 0x1021, init 0xffff) over everything after the sync byte.  The
 * host never sends while a response is pending, so the half-duplex wire
 * never has two drivers.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "target.h"
#include "stub.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

extern uint32_t g_stub_param[2];

static uint8_t g_buf[STUB_MAXDATA];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void stub_reply(uint8_t status, const uint8_t *data, uint32_t len)
{
  uint8_t hdr[3];
  uint16_t crc;

  hdr[0] = status;
  hdr[1] = len;
  hdr[2] = len >> 8;
  crc = crc16_update(0xffff, hdr, 3);
  crc = crc16_update(crc, data, len);

  uart_putc(STUB_RSP_SYNC);
  for (int i = 0; i < 3; i++)
    {
      uart_putc(hdr[i]);
    }

  for (uint32_t i = 0; i < len; i++)
    {
      uart_putc(data[i]);
    }

  uart_putc(crc);
  uart_putc(crc >> 8);
}

static void stub_put32(uint8_t *p, uint32_t v)
{
  p[0] = v;
  p[1] = v >> 8;
  p[2] = v >> 16;
  p[3] = v >> 24;
}

static uint8_t stub_execute(uint8_t cmd, uint32_t addr, uint32_t len,
                            uint32_t *rlen)
{
  uint32_t v;

  *rlen = 0;

  switch (cmd)
    {
      case STUB_CMD_INFO:
        stub_put32(g_buf, sf_readid());
        g_buf[4] = sf_status();
        g_buf[5] = STUB_VERSION;
        *rlen = 6;
        return STUB_OK;

      case STUB_CMD_READ:

        /* "addr" is the flash address, the payload the 32-bit size */

        if (len != 4)
          {
            return STUB_EARG;
          }

        v = g_buf[0] | (g_buf[1] << 8) | (g_buf[2] << 16) |
            ((uint32_t)g_buf[3] << 24);
        if (v == 0 || v > STUB_MAXDATA)
          {
            return STUB_EARG;
          }

        sf_read(addr, g_buf, v);
        *rlen = v;
        return STUB_OK;

      case STUB_CMD_ERASE4K:
      case STUB_CMD_ERASE64K:
        sf_erase(addr, cmd == STUB_CMD_ERASE64K);
        return STUB_OK;

      case STUB_CMD_WRITE:
        sf_program(addr, g_buf, len);
        return STUB_OK;

      case STUB_CMD_CRC32:
        {
          uint32_t crc = 0;
          uint32_t n;

          /* "addr" is the flash address, the payload the 32-bit size */

          if (len != 4)
            {
              return STUB_EARG;
            }

          len = g_buf[0] | (g_buf[1] << 8) | (g_buf[2] << 16) |
                ((uint32_t)g_buf[3] << 24);
          while (len > 0)
            {
              n = len > STUB_MAXDATA ? STUB_MAXDATA : len;
              sf_read(addr, g_buf, n);
              crc = crc32_update(crc, g_buf, n);
              addr += n;
              len  -= n;
            }

          stub_put32(g_buf, crc);
          *rlen = 4;
          return STUB_OK;
        }

      case STUB_CMD_MEMREAD:
        if (len != 4)
          {
            return STUB_EARG;
          }

        v = g_buf[0] | (g_buf[1] << 8) | (g_buf[2] << 16) |
            ((uint32_t)g_buf[3] << 24);
        if (v == 0 || v > STUB_MAXDATA)
          {
            return STUB_EARG;
          }

        for (uint32_t i = 0; i < v; i++)
          {
            g_buf[i] = *(volatile uint8_t *)(uintptr_t)(addr + i);
          }

        *rlen = v;
        return STUB_OK;

      case STUB_CMD_PEEK:
        stub_put32(g_buf, SFR(addr & ~3u));
        *rlen = 4;
        return STUB_OK;

      case STUB_CMD_RESET:
        stub_reply(STUB_OK, g_buf, 0);
        WDTCON = WDTCON_RESET;
        for (; ; );

      default:
        return STUB_ECMD;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stub_main(void)
{
  uint8_t hdr[7];
  uint32_t addr;
  uint32_t len;
  uint32_t rlen;
  uint16_t crc;
  uint16_t rcrc;
  uint8_t status;

  WDTCON = WDTCON_DISABLE;
  uart_init(g_stub_param[1]);
  sf_init(1);                           /* 24 MHz at the ROM loader's 48 MHz */

  for (; ; )
    {
      while (uart_getc() != STUB_REQ_SYNC)
        {
        }

      for (int i = 0; i < 7; i++)
        {
          hdr[i] = uart_getc();
        }

      len  = hdr[1] | (hdr[2] << 8);
      addr = hdr[3] | (hdr[4] << 8) | (hdr[5] << 16) |
             ((uint32_t)hdr[6] << 24);

      if (len > STUB_MAXDATA)
        {
          stub_reply(STUB_EARG, g_buf, 0);
          continue;
        }

      for (uint32_t i = 0; i < len; i++)
        {
          g_buf[i] = uart_getc();
        }

      rcrc  = uart_getc();
      rcrc |= uart_getc() << 8;

      crc = crc16_update(0xffff, hdr, 7);
      crc = crc16_update(crc, g_buf, len);
      if (crc != rcrc)
        {
          stub_reply(STUB_ECRC, g_buf, 0);
          continue;
        }

      status = stub_execute(hdr[0], addr, len, &rlen);
      stub_reply(status, g_buf, rlen);
    }
}
