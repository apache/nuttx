/****************************************************************************
 * drivers/wireless/lpwan/sx1301/sx1301_reg.c
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

/* Page aware register access for the SX1301.
 *
 * The SX1301 uses a plain two byte SPI protocol:
 *
 *   write: [0x80 | addr] [data]
 *   read:  [0x00 | addr] [dummy]  -> data comes back in the second byte
 *
 * Bursts keep the same one byte header and let the chip auto increment the
 * address internally, which is how the 8 KiB MCU firmware images and the RX
 * FIFO are moved.  The chip select must stay asserted for the whole burst,
 * hence the explicit SPI_SELECT() around each transaction instead of one
 * SPI_SNDBLOCK() per buffer.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <string.h>

#include <nuttx/wireless/wireless.h>

#include "sx1301_priv.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* MOSI filler used during burst reads.  The reference HAL clocks out zeros
 * (the Linux spidev driver sends zeros for a NULL tx buffer), while
 * SPI_EXCHANGE() with a NULL tx buffer sends 0xff on most NuttX ports.  The
 * SX1301 ignores MOSI during the data phase of a read, but keeping the same
 * pattern as the reference removes one variable when debugging silence on
 * the bus.
 */

static const uint8_t g_sx1301_zerobuf[SX1301_ZEROBUF_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx1301_lock
 *
 * Description:
 *   Take the SPI bus and configure it for the SX1301.
 *
 ****************************************************************************/

static void sx1301_lock(FAR struct sx1301_dev_s *priv)
{
  SPI_LOCK(priv->spi, true);
  SPI_SETMODE(priv->spi, SPIDEV_MODE0);
  SPI_SETBITS(priv->spi, 8);
  SPI_SETFREQUENCY(priv->spi, CONFIG_LPWAN_SX1301_SPIFREQ);
}

/****************************************************************************
 * Name: sx1301_unlock
 ****************************************************************************/

static void sx1301_unlock(FAR struct sx1301_dev_s *priv)
{
  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: sx1301_rawwrite
 *
 * Description:
 *   Single register write without any page handling.  The bus must already
 *   be locked.
 *
 ****************************************************************************/

static void sx1301_rawwrite(FAR struct sx1301_dev_s *priv, uint8_t addr,
                            uint8_t value)
{
  uint8_t txbuf[2];

  txbuf[0] = SX1301_SPI_WRITE_FLAG | (addr & SX1301_SPI_ADDR_MASK);
  txbuf[1] = value;

  SPI_SELECT(priv->spi, SPIDEV_LPWAN(0), true);
  SPI_SNDBLOCK(priv->spi, txbuf, 2);
  SPI_SELECT(priv->spi, SPIDEV_LPWAN(0), false);
}

/****************************************************************************
 * Name: sx1301_rawread
 *
 * Description:
 *   Single register read without any page handling.  The bus must already
 *   be locked.
 *
 ****************************************************************************/

static void sx1301_rawread(FAR struct sx1301_dev_s *priv, uint8_t addr,
                           FAR uint8_t *value)
{
  uint8_t txbuf[2];
  uint8_t rxbuf[2];

  txbuf[0] = addr & SX1301_SPI_ADDR_MASK;
  txbuf[1] = 0;

  SPI_SELECT(priv->spi, SPIDEV_LPWAN(0), true);
  SPI_EXCHANGE(priv->spi, txbuf, rxbuf, 2);
  SPI_SELECT(priv->spi, SPIDEV_LPWAN(0), false);

  *value = rxbuf[1];
}

/****************************************************************************
 * Name: sx1301_setpage
 *
 * Description:
 *   Select a register page, if it is not the current one.  The bus must
 *   already be locked.  SX1301_PAGE_ANY leaves the page untouched, for the
 *   registers that are page independent.
 *
 ****************************************************************************/

static void sx1301_setpage(FAR struct sx1301_dev_s *priv, uint8_t page)
{
  if (page == SX1301_PAGE_ANY || page == priv->page)
    {
      return;
    }

  sx1301_rawwrite(priv, SX1301_REG_PAGE, page & 0x03);
  priv->page = page;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx1301_reg_write
 ****************************************************************************/

int sx1301_reg_write(FAR struct sx1301_dev_s *priv, uint8_t page,
                     uint8_t addr, uint8_t value)
{
  sx1301_lock(priv);
  sx1301_setpage(priv, page);
  sx1301_rawwrite(priv, addr, value);
  sx1301_unlock(priv);

  /* Writing the page register by hand invalidates the cached page */

  if (addr == SX1301_REG_PAGE)
    {
      priv->page = value & 0x03;
    }

  return OK;
}

/****************************************************************************
 * Name: sx1301_reg_read
 ****************************************************************************/

int sx1301_reg_read(FAR struct sx1301_dev_s *priv, uint8_t page,
                    uint8_t addr, FAR uint8_t *value)
{
  if (value == NULL)
    {
      return -EINVAL;
    }

  sx1301_lock(priv);
  sx1301_setpage(priv, page);
  sx1301_rawread(priv, addr, value);
  sx1301_unlock(priv);

  return OK;
}

/****************************************************************************
 * Name: sx1301_reg_wrburst
 *
 * Description:
 *   Burst write.  The address byte is followed by the whole buffer with the
 *   chip select held low; the SX1301 increments its internal pointer.
 *
 ****************************************************************************/

int sx1301_reg_wrburst(FAR struct sx1301_dev_s *priv, uint8_t page,
                       uint8_t addr, FAR const uint8_t *buffer,
                       size_t buflen)
{
  uint8_t hdr;

  if (buffer == NULL || buflen == 0)
    {
      return -EINVAL;
    }

  hdr = SX1301_SPI_WRITE_FLAG | (addr & SX1301_SPI_ADDR_MASK);

  sx1301_lock(priv);
  sx1301_setpage(priv, page);

  SPI_SELECT(priv->spi, SPIDEV_LPWAN(0), true);
  SPI_SNDBLOCK(priv->spi, &hdr, 1);
  SPI_SNDBLOCK(priv->spi, buffer, buflen);
  SPI_SELECT(priv->spi, SPIDEV_LPWAN(0), false);

  sx1301_unlock(priv);
  return OK;
}

/****************************************************************************
 * Name: sx1301_reg_rdburst
 *
 * Description:
 *   Burst read.  The first byte clocked back during the address phase is
 *   discarded, the rest is the payload.
 *
 ****************************************************************************/

int sx1301_reg_rdburst(FAR struct sx1301_dev_s *priv, uint8_t page,
                       uint8_t addr, FAR uint8_t *buffer, size_t buflen)
{
  uint8_t hdr;
  size_t chunk;
  size_t off;

  if (buffer == NULL || buflen == 0)
    {
      return -EINVAL;
    }

  hdr = addr & SX1301_SPI_ADDR_MASK;

  sx1301_lock(priv);
  sx1301_setpage(priv, page);

  SPI_SELECT(priv->spi, SPIDEV_LPWAN(0), true);
  SPI_SNDBLOCK(priv->spi, &hdr, 1);

  /* Clock out zeros while reading, in chunks of the filler buffer */

  for (off = 0; off < buflen; off += chunk)
    {
      chunk = buflen - off;
      if (chunk > SX1301_ZEROBUF_SIZE)
        {
          chunk = SX1301_ZEROBUF_SIZE;
        }

      SPI_EXCHANGE(priv->spi, g_sx1301_zerobuf, buffer + off, chunk);
    }

  SPI_SELECT(priv->spi, SPIDEV_LPWAN(0), false);

  sx1301_unlock(priv);
  return OK;
}

/****************************************************************************
 * Name: sx1301_reg_setbit
 *
 * Description:
 *   Read modify write of a single bit.
 *
 ****************************************************************************/

int sx1301_reg_setbit(FAR struct sx1301_dev_s *priv, uint8_t page,
                      uint8_t addr, uint8_t bit, bool value)
{
  uint8_t regval;
  int ret;

  ret = sx1301_reg_read(priv, page, addr, &regval);
  if (ret < 0)
    {
      return ret;
    }

  if (value)
    {
      regval |= 1 << bit;
    }
  else
    {
      regval &= ~(1 << bit);
    }

  return sx1301_reg_write(priv, page, addr, regval);
}

/****************************************************************************
 * Name: sx1301_reg_write13s
 *
 * Description:
 *   Write a 13 bit signed value spanning two consecutive registers, which is
 *   how the intermediate frequency of every demodulator is programmed.
 *
 ****************************************************************************/

int sx1301_reg_write13s(FAR struct sx1301_dev_s *priv, uint8_t page,
                        uint8_t addr, int32_t value)
{
  uint16_t raw = value & 0x1fff;
  int ret;

  ret = sx1301_reg_write(priv, page, addr, raw & 0xff);
  if (ret < 0)
    {
      return ret;
    }

  return sx1301_reg_write(priv, page, addr + 1, (raw >> 8) & 0x1f);
}

/****************************************************************************
 * Name: sx1301_reg_probe
 *
 * Description:
 *   Identify the chip on the bus.  Some shields come back with an
 *   unexpected version right after reset but still report the correct chip
 *   identifier, so both are checked.
 *
 * Returned Value:
 *   OK if an SX1301 answered, -ENODEV otherwise.
 *
 ****************************************************************************/

int sx1301_reg_probe(FAR struct sx1301_dev_s *priv)
{
  uint8_t version;
  uint8_t chipid;

  priv->page = 0;

  sx1301_reg_read(priv, SX1301_PAGE_ANY, SX1301_REG_VERSION, &version);
  if (version == SX1301_CHIP_VERSION)
    {
      wlinfo("SX1301 detected, version 0x%02x\n", version);
    }
  else
    {
      sx1301_reg_read(priv, SX1301_PAGE_ANY, SX1301_REG_CHIP_ID, &chipid);
      if (chipid != SX1301_CHIP_ID_VALUE)
        {
          wlerr("ERROR: no SX1301 found, version 0x%02x chipid 0x%02x\n",
                version, chipid);
          return -ENODEV;
        }

      wlwarn("WARNING: SX1301 version 0x%02x, chipid 0x%02x, going on\n",
             version, chipid);
    }

  /* Make sure we start from page 0 */

  sx1301_reg_write(priv, SX1301_PAGE_ANY, SX1301_REG_PAGE, 0);
  priv->page = 0;
  priv->connected = true;

  return OK;
}
