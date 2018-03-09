/****************************************************************************
 * drivers/lcd/ft80x_spi.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/lcd/ft80x.h>
#include <nuttx/spi/spi.h>
#include "ft80x.h"

#if defined(CONFIG_LCD_FT80X) && defined(CONFIG_LCD_FT80X_SPI)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ft80x_select
 *
 * Description:
 *   Select the FT80X part
 *
 ****************************************************************************/

static void ft80x_select(FAR struct ft80x_dev_s *priv)
{
  lcdinfo("Mode: %d Bits: 8 Frequency: %d\n", SPIDEV_MODE0, priv->frequency);

  DEBUGASSERT(priv != NULL);

  /* Lock the SPI bus */

  (void)SPI_LOCK(priv->spi, true);

  /* Configure SPI for the FT80X */

  SPI_SETMODE(priv->spi, SPIDEV_MODE0);
  SPI_SETBITS(priv->spi, 8);
  (void)SPI_HWFEATURES(priv->spi, 0);
  (void)SPI_SETFREQUENCY(priv->spi, priv->frequency);

  /* Select SPI device */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
}

/****************************************************************************
 * Name: ft80x_deselect
 *
 * Description:
 *   De-select the FT80X part
 *
 ****************************************************************************/

static void ft80x_deselect(FAR struct ft80x_dev_s *priv)
{
  /* Des-select the FT80x device */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);

  /* Unlock bus */

  (void)SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ft80x_host_command
 *
 * Description:
 *   Send a host command to the FT80x
 *
 *   For an SPI write command write transaction, the host writes a zero bit
 *   followed by a one bit, followed by the 5-bit command, followed by two
 *   bytes of zero. All data is streamed with a single chip select.
 *
 *   NOTE:  Commands are defined in ft80x.h with bit 7 = 0 and bit 6 = 1.
 *
 ****************************************************************************/

void ft80x_host_command(FAR struct ft80x_dev_s *priv, uint8_t cmd)
{
  struct ft80x_hostwrite_s hostwrite;

  DEBUGASSERT(priv != NULL && (cmd == 0x00 || (cmd & 0xc0) == 0x40));

  /* Format the host write command */

  hostwrite.cmd  = cmd;
  hostwrite.pad1 = 0;
  hostwrite.pad2 = 0;

  /* Select the FT80x */

  ft80x_select(priv);

  /* Send the host write command */

  SPI_SNDBLOCK(priv->spi, &hostwrite, sizeof(struct ft80x_hostwrite_s));

  /* De-select the FT80x */

  ft80x_deselect(priv);
}

/****************************************************************************
 * Name: ft80x_read_memory
 *
 * Description:
 *   Read from FT80X memory
 *
 *   For SPI memory read transaction, the host sends two zero bits, followed
 *   by the 22-bit address. This is followed by a dummy byte. After the dummy
 *   byte, the FT80x responds to each host byte with read data bytes.
 *
 ****************************************************************************/

void ft80x_read_memory(FAR struct ft80x_dev_s *priv, uint32_t addr,
                       FAR void *buffer, size_t buflen)
{
  struct ft80x_spiread_s spiread;

  DEBUGASSERT(priv != NULL && (addr & 0xffc00000) == 0 &&
              buffer != NULL && buflen > 0);

  /* Format the read header */

  spiread.addrh = (addr >> 16) & 0x3f;
  spiread.addrm = (addr >> 8)  & 0xff;
  spiread.addrl =  addr        & 0xff;
  spiread.dummy = 0xff;

  /* Select the FT80x */

  ft80x_select(priv);

  /* Send the read header */

  SPI_SNDBLOCK(priv->spi, &spiread, sizeof(struct ft80x_spiread_s));

  /* Then read the FT80x memory into the user provided buffer */

  SPI_RECVBLOCK(priv->spi, buffer, buflen);

  /* De-select the FT80x */

  ft80x_deselect(priv);
}

/****************************************************************************
 * Name: ft80x_read_byte, ft80x_read_hword, ft80x_read_word
 *
 * Description:
 *   Read an 8-, 16-, or 32-bt bit value from FT80X memory
 *
 *   For SPI memory read transaction, the host sends two zero bits, followed
 *   by the 22-bit address. This is followed by a dummy byte. After the dummy
 *   byte, the FT80x responds to each host byte with read data bytes.
 *
 ****************************************************************************/

uint8_t ft80x_read_byte(FAR struct ft80x_dev_s *priv, uint32_t addr)
{
  uint8_t data;
  ft80x_read_memory(priv, addr, (FAR void *)&data, 1);
  return data;
}

uint16_t ft80x_read_hword(FAR struct ft80x_dev_s *priv, uint32_t addr)
{
  uint16_t data;
  ft80x_read_memory(priv, addr, (FAR void *)&data, 2);
  return data;
}

uint32_t ft80x_read_word(FAR struct ft80x_dev_s *priv, uint32_t addr)
{
  uint32_t data;
  ft80x_read_memory(priv, addr, (FAR void *)&data, 4);
  return data;
}

/****************************************************************************
 * Name: ft80x_write_memory
 *
 * Description:
 *   Write to FT80X memory
 *
 *   For SPI memory write transaction, the host sends a '1' bit and '0' bit,
 *   followed by the 22-bit address. This is followed by the write data.
 *
 ****************************************************************************/

void ft80x_write_memory(FAR struct ft80x_dev_s *priv, uint32_t addr,
                        FAR const void *buffer, size_t buflen)
{
  struct ft80x_spiwrite_s spiwrite;

  DEBUGASSERT(priv != NULL && (addr & 0xffc00000) == 0 &&
              buffer != NULL && buflen > 0);

  /* Format the write header */

  spiwrite.addrh = 0x80 | ((addr >> 16) & 0x3f);
  spiwrite.addrm = (addr >> 8) & 0xff;
  spiwrite.addrl =  addr       & 0xff;

  /* Select the FT80x */

  ft80x_select(priv);

  /* Send the write header */

  SPI_SNDBLOCK(priv->spi, &spiwrite, sizeof(struct ft80x_spiwrite_s));

  /* Then write to the FT80x memory from the user provided buffer */

  SPI_SNDBLOCK(priv->spi, buffer, buflen);

  /* De-select the FT80x */

  ft80x_deselect(priv);
}

/****************************************************************************
 * Name: ft80x_write_byte, ft80x_write_hword, ft80x_write_word
 *
 * Description:
 *   Write an 8-, 16-, or 32-bt bit value to FT80X memory
 *
 *   For SPI memory write transaction, the host sends a '1' bit and '0' bit,
 *   followed by the 22-bit address. This is followed by the write data.
 *
 ****************************************************************************/

void ft80x_write_byte(FAR struct ft80x_dev_s *priv, uint32_t addr,
                      uint8_t data)
{
  struct ft80x_spiwrite8_s spiwrite;

  DEBUGASSERT(priv != NULL && (addr & 0xffc00000) == 0);

  /* Format the write header */

  spiwrite.addrh = 0x80 | ((addr >> 16) & 0x3f);
  spiwrite.addrm = (addr >> 8) & 0xff;
  spiwrite.addrl =  addr       & 0xff;
  spiwrite.data  =  data;

  /* Select the FT80x */

  ft80x_select(priv);

  /* Send the write header and 8-bit data */

  SPI_SNDBLOCK(priv->spi, &spiwrite, sizeof(struct ft80x_spiwrite8_s));

  /* De-select the FT80x */

  ft80x_deselect(priv);
}

void ft80x_write_hword(FAR struct ft80x_dev_s *priv, uint32_t addr,
                       uint16_t data)
{
  struct ft80x_spiwrite16_s spiwrite;

  DEBUGASSERT(priv != NULL && (addr & 0xffc00000) == 0);

  /* Format the write header */

  spiwrite.addrh   = 0x80 | ((addr >> 16) & 0x3f);
  spiwrite.addrm   = (addr >> 8) & 0xff;
  spiwrite.addrl   =  addr       & 0xff;
  spiwrite.data[0] =  data       & 0xff; /* Little endian */
  spiwrite.data[1] = (data >> 8) & 0xff;

  /* Select the FT80x */

  ft80x_select(priv);

  /* Send the write header and 16-bit data */

  SPI_SNDBLOCK(priv->spi, &spiwrite, sizeof(struct ft80x_spiwrite16_s));

  /* De-select the FT80x */

  ft80x_deselect(priv);
}

void ft80x_write_word(FAR struct ft80x_dev_s *priv, uint32_t addr,
                      uint32_t data)
{
  struct ft80x_spiwrite32_s spiwrite;

  DEBUGASSERT(priv != NULL && (addr & 0xffc00000) == 0);

  /* Format the write header */

  spiwrite.addrh   = 0x80 | ((addr >> 16) & 0x3f);
  spiwrite.addrm   = (addr >> 8)  & 0xff;
  spiwrite.addrl   =  addr        & 0xff;
  spiwrite.data[0] =  data        & 0xff; /* Little endian */
  spiwrite.data[1] = (data >> 8)  & 0xff;
  spiwrite.data[2] = (data >> 16) & 0xff;
  spiwrite.data[3] = (data >> 24) & 0xff;

  /* Select the FT80x */

  ft80x_select(priv);

  /* Send the write header and 32-bit data */

  SPI_SNDBLOCK(priv->spi, &spiwrite, sizeof(struct ft80x_spiwrite32_s));

  /* De-select the FT80x */

  ft80x_deselect(priv);
}

#endif /* CONFIG_LCD_FT80X && CONFIG_LCD_FT80X_SPI */
