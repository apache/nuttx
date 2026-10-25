/****************************************************************************
 * boards/risc-v/ab32vg1/ab32vg1-generic/tools/target/boot.c
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

/* Stage-1 boot loader.  Reads the image table at IMG_TABLE, copies every
 * segment from flash to RAM, checks it and returns the entry point.  On
 * any error it prints a message on the console and returns 0, which makes
 * boot_start.S go back to the mask ROM.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "target.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The system clock set up by the mask ROM for the boot code is not known;
 * sysclk / 4 is what the vendor boot code uses.
 */

#define BOOT_SPI_BAUD   3

#define BOOT_DEBUG      0

/* Host listening window: 8 ms of system clock cycles (48 MHz when the
 * mask ROM starts the boot code)
 */

#define BOOT_WINDOW     (48000000 / 1000 * 8)

#if BOOT_DEBUG
#  define progress(c)   uart_putc(c)
#else
#  define progress(c)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_picen;
static uint32_t g_piccon;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uintptr_t boot_fail(const char *msg)
{
  uart_init(115200);
  uart_puts("\r\nab32vg1 boot: ");
  uart_puts(msg);
  uart_puts("\r\n");

  /* Give the mask ROM back its interrupts, stop the watchdog */

  WDTCON = WDTCON_DISABLE;

  PICEN = g_picen;
  PICCON = g_piccon;
  return 0;
}

/* Listen on the console pin for a host tool trying to reach the mask ROM
 * UART loader (its sync token, or the NuttX reset key, both sent
 * continuously while it waits).  Returns true if one was seen.
 */

static bool boot_host_waiting(void)
{
  static const uint8_t sync[4] =
    {
      0xa5, 0x96, 0x87, 0x5a
    };

  uint32_t match = 0;
  bool seen = false;
  int ch;

  uart_init(115200);
  TMR0CON = 0;
  TMR0CNT = 0;
  TMR0PR  = 0xffffffff;
  TMR0CON = BIT(0);                     /* Counts system clock cycles */

  while (!seen && TMR0CNT < BOOT_WINDOW)
    {
      ch = uart_trygetc();
      if (ch < 0)
        {
          continue;
        }

      match = (ch == sync[match]) ? match + 1 : (ch == sync[0]);
      seen = match == 4 || ch == 0xf5;
    }

  TMR0CON = 0;

  /* Release PA7 for the mask ROM loader (TX0/RX0 mapping cleared) */

  UART0CON  = 0;
  FUNCMCON0 = (0xf << 12) | (0xf << 8);
  return seen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uintptr_t boot_main(void)
{
  struct img_table_s tbl;
  uint32_t i;

  /* The mask ROM only opens its UART loader when the boot code does not
   * clear CPU register 0x508 in time: then it takes control back (a boot
   * code supervision; the chip looks dead from the boot code side, even
   * the watchdog does not reset it).  The vendor boot code clears 0x508
   * at once and calls a ROM routine to listen for the host.  Do the same
   * with our own short listening window: if a host is waiting, never
   * clear 0x508 and let the mask ROM take over.
   */

  if (boot_host_waiting())
    {
      for (; ; )
        {
        }
    }

  SFR(0x508) = 0;

  /* The image overwrites RAM used by the mask ROM: no ROM interrupt
   * handler may run from now on.
   */

  g_piccon = PICCON;
  g_picen  = PICEN;
  PICCONCLR = BIT(0);
  PICEN = 0;

  /* If anything below hangs, the watchdog resets the chip and the mask ROM
   * UART loader becomes reachable again.  NuttX disables it at start.
   */

  WDTCON = WDTCON_EN4S;

#if BOOT_DEBUG
  uart_init(115200);
#endif
  progress('1');

  sf_init(BOOT_SPI_BAUD);
  progress('2');

  sf_read(IMG_TABLE, (uint8_t *)&tbl, sizeof(tbl));
  if (tbl.magic != IMG_MAGIC || tbl.nseg == 0 || tbl.nseg > IMG_MAXSEG ||
      crc32_update(0, (const uint8_t *)&tbl,
                   offsetof(struct img_table_s, crc)) != tbl.crc)
    {
      return boot_fail("no valid image");
    }

  progress('3');

  for (i = 0; i < tbl.nseg; i++)
    {
      uint8_t *dest = (uint8_t *)(uintptr_t)tbl.seg[i].addr;
      uint32_t off;
      uint32_t n;

      progress('a' + i);
      for (off = 0; off < tbl.seg[i].size; off += n)
        {
          n = tbl.seg[i].size - off;
          if (n > 4096)
            {
              n = 4096;
            }

          sf_read(tbl.seg[i].flash + off, dest + off, n);
          WDTCON = WDTCON_KICK;
          if ((off & 0x3fff) == 0)
            {
              progress('.');
            }
        }

      if (crc32_update(0, dest, tbl.seg[i].size) != tbl.seg[i].crc)
        {
          return boot_fail("segment CRC error");
        }
    }

  progress('4');
  return tbl.entry;
}
