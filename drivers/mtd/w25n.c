/****************************************************************************
 * drivers/mtd/w25n.c
 *
 * Driver for Winbond W25N SPI NAND flash.
 * Currently only W25N01GV (1Gbit/128MB) is supported and tested.
 *
 * Limitations:
 *   - No bad block management (BBM). Factory bad blocks and runtime bad
 *     blocks are not tracked.
 *   - No bad block table (BBT) scanning at initialization.
 *   - No Quad SPI support (standard SPI only).
 *   - No access to spare area (64 bytes/page) or OTP region.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_W25N_SPIMODE
#  define CONFIG_W25N_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_W25N_SPIFREQUENCY
#  define CONFIG_W25N_SPIFREQUENCY 20000000
#endif

/* W25N Commands ************************************************************/

#define W25N_RESET              0xff  /* Device reset */
#define W25N_READ_ID            0x9f  /* Read JEDEC ID */
#define W25N_READ_STATUS        0x0f  /* Read status register */
#define W25N_WRITE_STATUS       0x1f  /* Write status register */
#define W25N_WRITE_ENABLE       0x06  /* Write enable */
#define W25N_WRITE_DISABLE      0x04  /* Write disable */
#define W25N_PAGE_READ          0x13  /* Page data read (array to buffer) */
#define W25N_READ_DATA          0x03  /* Read data (from buffer) */
#define W25N_PROGRAM_LOAD       0x02  /* Program load (to buffer) */
#define W25N_PROGRAM_EXECUTE    0x10  /* Program execute (buffer to array) */
#define W25N_BLOCK_ERASE        0xd8  /* Block erase */

#define W25N_DUMMY              0x00  /* Dummy byte for SPI */

/* Status Register Addresses ************************************************/

#define W25N_SR1_ADDR           0xa0  /* Protection register */
#define W25N_SR2_ADDR           0xb0  /* Configuration register */
#define W25N_SR3_ADDR           0xc0  /* Status register */

/* Status Register 1 (Protection) bits **************************************/

#define W25N_SR1_SRP0           (1 << 7)
#define W25N_SR1_BP3            (1 << 6)
#define W25N_SR1_BP2            (1 << 5)
#define W25N_SR1_BP1            (1 << 4)
#define W25N_SR1_BP0            (1 << 3)
#define W25N_SR1_TB             (1 << 2)
#define W25N_SR1_WPE            (1 << 1)
#define W25N_SR1_SRP1           (1 << 0)

/* Status Register 2 (Configuration) bits ***********************************/

#define W25N_SR2_OTPL           (1 << 7)
#define W25N_SR2_OTPE           (1 << 6)
#define W25N_SR2_SR1L           (1 << 5)
#define W25N_SR2_ECCE           (1 << 4)
#define W25N_SR2_BUF            (1 << 3)

/* Status Register 3 (Status) bits ******************************************/

#define W25N_SR3_LUTF           (1 << 6)
#define W25N_SR3_ECC1           (1 << 5)
#define W25N_SR3_ECC0           (1 << 4)
#define W25N_SR3_PFAIL          (1 << 3)
#define W25N_SR3_EFAIL          (1 << 2)
#define W25N_SR3_WEL            (1 << 1)
#define W25N_SR3_BUSY           (1 << 0)

#define W25N_SR3_ECC_MASK       (W25N_SR3_ECC1 | W25N_SR3_ECC0)
#define W25N_SR3_ECC_OK         (0x00)
#define W25N_SR3_ECC_CORRECTED  (W25N_SR3_ECC0)
#define W25N_SR3_ECC_ERROR      (W25N_SR3_ECC1)

/* Device Identification ****************************************************/

#define W25N_MANUFACTURER_ID    0xef  /* Winbond */
#define W25N01GV_DEVICE_ID      0xaa21

/* Memory Organization ******************************************************/

#define W25N_PAGE_SIZE          2048  /* Bytes per page (data only) */
#define W25N_PAGE_SHIFT         11    /* 2^11 = 2048 */
#define W25N_PAGES_PER_BLOCK    64
#define W25N_BLOCK_SIZE         (W25N_PAGE_SIZE * W25N_PAGES_PER_BLOCK)
#define W25N_BLOCK_SHIFT        17    /* 2^17 = 128KB */
#define W25N01GV_BLOCKS         1024  /* Total blocks for 1Gbit device */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct w25n_dev_s
{
  struct mtd_dev_s       mtd;         /* MTD interface */
  FAR struct spi_dev_s  *spi;         /* SPI device */
  uint32_t               spi_devid;   /* SPI device ID for chip select */
  uint16_t               nblocks;     /* Number of erase blocks */
  uint8_t                blockshift;  /* Block size shift (17 = 128KB) */
  uint8_t                pageshift;   /* Page size shift (11 = 2KB) */
  uint8_t                eccstatus;   /* Last ECC status */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI helpers */

static void w25n_lock(FAR struct w25n_dev_s *priv);
static void w25n_unlock(FAR struct w25n_dev_s *priv);

/* Command helpers */

static void    w25n_reset(FAR struct w25n_dev_s *priv);
static int     w25n_readid(FAR struct w25n_dev_s *priv);
static uint8_t w25n_read_status(FAR struct w25n_dev_s *priv, uint8_t reg);
static void    w25n_write_status(FAR struct w25n_dev_s *priv,
                                 uint8_t reg, uint8_t val);
static void    w25n_writeenable(FAR struct w25n_dev_s *priv);
static void    w25n_writedisable(FAR struct w25n_dev_s *priv);
static int     w25n_waitready(FAR struct w25n_dev_s *priv);
static int     w25n_waitready_erase(FAR struct w25n_dev_s *priv);

/* Page operations */

static int     w25n_read_page(FAR struct w25n_dev_s *priv, uint32_t page);
static void    w25n_read_buffer(FAR struct w25n_dev_s *priv, uint16_t col,
                                FAR uint8_t *buf, size_t len);
static void    w25n_load_buffer(FAR struct w25n_dev_s *priv, uint16_t col,
                                FAR const uint8_t *buf, size_t len);
static int     w25n_program_execute(FAR struct w25n_dev_s *priv,
                                    uint32_t page);
static int     w25n_block_erase(FAR struct w25n_dev_s *priv, uint32_t block);

/* Configuration */

static void    w25n_enable_ecc(FAR struct w25n_dev_s *priv);
static void    w25n_unprotect(FAR struct w25n_dev_s *priv);

/* MTD driver methods */

static int     w25n_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks);
static ssize_t w25n_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buf);
static ssize_t w25n_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buf);
static ssize_t w25n_read(FAR struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, FAR uint8_t *buf);
static int     w25n_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: w25n_lock
 ****************************************************************************/

static void w25n_lock(FAR struct w25n_dev_s *priv)
{
  SPI_LOCK(priv->spi, true);
  SPI_SETMODE(priv->spi, CONFIG_W25N_SPIMODE);
  SPI_SETBITS(priv->spi, 8);
  SPI_HWFEATURES(priv->spi, 0);
  SPI_SETFREQUENCY(priv->spi, CONFIG_W25N_SPIFREQUENCY);
}

/****************************************************************************
 * Name: w25n_unlock
 ****************************************************************************/

static void w25n_unlock(FAR struct w25n_dev_s *priv)
{
  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: w25n_reset
 ****************************************************************************/

static void w25n_reset(FAR struct w25n_dev_s *priv)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, W25N_RESET);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);

  /* Wait for reset to complete (tRST max 500us) */

  nxsig_usleep(500);
}

/****************************************************************************
 * Name: w25n_readid
 ****************************************************************************/

static int w25n_readid(FAR struct w25n_dev_s *priv)
{
  uint8_t manufacturer;
  uint8_t device_hi;
  uint8_t device_lo;

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, W25N_READ_ID);
  SPI_SEND(priv->spi, W25N_DUMMY);
  manufacturer = SPI_SEND(priv->spi, W25N_DUMMY);
  device_hi = SPI_SEND(priv->spi, W25N_DUMMY);
  device_lo = SPI_SEND(priv->spi, W25N_DUMMY);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);

  finfo("W25N: Manufacturer=0x%02x Device=0x%02x%02x\n",
        manufacturer, device_hi, device_lo);

  if (manufacturer != W25N_MANUFACTURER_ID)
    {
      ferr("ERROR: Unexpected manufacturer ID: 0x%02x\n", manufacturer);
      return -ENODEV;
    }

  if (device_hi == 0xaa && device_lo == 0x21)
    {
      /* W25N01GV - 1Gbit */

      priv->nblocks = W25N01GV_BLOCKS;
      finfo("W25N01GV detected: %d blocks\n", priv->nblocks);
    }
  else
    {
      ferr("ERROR: Unsupported device ID: 0x%02x%02x\n",
           device_hi, device_lo);
      return -ENODEV;
    }

  priv->blockshift = W25N_BLOCK_SHIFT;
  priv->pageshift = W25N_PAGE_SHIFT;

  return OK;
}

/****************************************************************************
 * Name: w25n_read_status
 ****************************************************************************/

static uint8_t w25n_read_status(FAR struct w25n_dev_s *priv, uint8_t reg)
{
  uint8_t status;

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, W25N_READ_STATUS);
  SPI_SEND(priv->spi, reg);
  status = SPI_SEND(priv->spi, W25N_DUMMY);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);

  return status;
}

/****************************************************************************
 * Name: w25n_write_status
 ****************************************************************************/

static void w25n_write_status(FAR struct w25n_dev_s *priv,
                              uint8_t reg, uint8_t val)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, W25N_WRITE_STATUS);
  SPI_SEND(priv->spi, reg);
  SPI_SEND(priv->spi, val);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
}

/****************************************************************************
 * Name: w25n_writeenable
 ****************************************************************************/

static void w25n_writeenable(FAR struct w25n_dev_s *priv)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, W25N_WRITE_ENABLE);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
}

/****************************************************************************
 * Name: w25n_writedisable
 ****************************************************************************/

static void w25n_writedisable(FAR struct w25n_dev_s *priv)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, W25N_WRITE_DISABLE);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
}

/****************************************************************************
 * Name: w25n_waitready
 ****************************************************************************/

static int w25n_waitready(FAR struct w25n_dev_s *priv)
{
  uint8_t status;
  int timeout = 10000;

  /* Busy-wait for fast operations (page read ~60us, page program ~250us) */

  do
    {
      status = w25n_read_status(priv, W25N_SR3_ADDR);
      if ((status & W25N_SR3_BUSY) == 0)
        {
          priv->eccstatus = (status & W25N_SR3_ECC_MASK);
          return OK;
        }
    }
  while (--timeout > 0);

  ferr("ERROR: Timeout waiting for device ready\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: w25n_waitready_erase
 *
 * Description:
 *   Wait for block erase to complete. Uses sleep since erase is slow
 *   (2-10ms typical) and we can release the SPI bus for other tasks.
 *
 ****************************************************************************/

static int w25n_waitready_erase(FAR struct w25n_dev_s *priv)
{
  uint8_t status;
  int timeout = 100;  /* 100 iterations, ~1 second max */

  do
    {
      status = w25n_read_status(priv, W25N_SR3_ADDR);
      if ((status & W25N_SR3_BUSY) == 0)
        {
          priv->eccstatus = (status & W25N_SR3_ECC_MASK);
          return OK;
        }

      /* Unlock SPI, sleep, re-lock - allows other SPI access while waiting */

      w25n_unlock(priv);
      nxsched_usleep(1000);
      w25n_lock(priv);
    }
  while (--timeout > 0);

  ferr("ERROR: Timeout waiting for erase to complete\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: w25n_read_page
 *
 * Description:
 *   Load a page from the NAND array into the device buffer.
 *
 ****************************************************************************/

static int w25n_read_page(FAR struct w25n_dev_s *priv, uint32_t page)
{
  int ret;

  /* Send Page Data Read command with 16-bit page address */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, W25N_PAGE_READ);
  SPI_SEND(priv->spi, W25N_DUMMY);
  SPI_SEND(priv->spi, (page >> 8) & 0xff);
  SPI_SEND(priv->spi, page & 0xff);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);

  /* Wait for page read to complete (tRD max 60us with ECC) */

  ret = w25n_waitready(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Check ECC status */

  if (priv->eccstatus == W25N_SR3_ECC_ERROR)
    {
      ferr("ERROR: Uncorrectable ECC error on page %lu\n",
           (unsigned long)page);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: w25n_read_buffer
 *
 * Description:
 *   Read data from the device buffer (after page read).
 *
 ****************************************************************************/

static void w25n_read_buffer(FAR struct w25n_dev_s *priv, uint16_t col,
                             FAR uint8_t *buf, size_t len)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, W25N_READ_DATA);
  SPI_SEND(priv->spi, (col >> 8) & 0xff);
  SPI_SEND(priv->spi, col & 0xff);
  SPI_SEND(priv->spi, W25N_DUMMY);
  SPI_RECVBLOCK(priv->spi, buf, len);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
}

/****************************************************************************
 * Name: w25n_load_buffer
 *
 * Description:
 *   Load data into the device program buffer.
 *
 ****************************************************************************/

static void w25n_load_buffer(FAR struct w25n_dev_s *priv, uint16_t col,
                             FAR const uint8_t *buf, size_t len)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, W25N_PROGRAM_LOAD);
  SPI_SEND(priv->spi, (col >> 8) & 0xff);
  SPI_SEND(priv->spi, col & 0xff);
  SPI_SNDBLOCK(priv->spi, buf, len);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
}

/****************************************************************************
 * Name: w25n_program_execute
 *
 * Description:
 *   Program the buffer contents to the specified page.
 *
 ****************************************************************************/

static int w25n_program_execute(FAR struct w25n_dev_s *priv, uint32_t page)
{
  uint8_t status;
  int ret;

  /* Send Program Execute command with 16-bit page address */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, W25N_PROGRAM_EXECUTE);
  SPI_SEND(priv->spi, W25N_DUMMY);
  SPI_SEND(priv->spi, (page >> 8) & 0xff);
  SPI_SEND(priv->spi, page & 0xff);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);

  /* Wait for program to complete (tPP max 700us) */

  ret = w25n_waitready(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Check for program failure */

  status = w25n_read_status(priv, W25N_SR3_ADDR);
  if (status & W25N_SR3_PFAIL)
    {
      ferr("ERROR: Program failed on page %lu\n", (unsigned long)page);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: w25n_block_erase
 *
 * Description:
 *   Erase a 128KB block.
 *
 ****************************************************************************/

static int w25n_block_erase(FAR struct w25n_dev_s *priv, uint32_t block)
{
  uint32_t page;
  uint8_t status;
  int ret;

  /* Convert block number to page address (block * 64) */

  page = block << 6;

  finfo("Erasing block %lu (page %lu)\n",
        (unsigned long)block, (unsigned long)page);

  w25n_writeenable(priv);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, W25N_BLOCK_ERASE);
  SPI_SEND(priv->spi, W25N_DUMMY);
  SPI_SEND(priv->spi, (page >> 8) & 0xff);
  SPI_SEND(priv->spi, page & 0xff);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);

  /* Wait for erase to complete (2-10ms typical) */

  ret = w25n_waitready_erase(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Check for erase failure */

  status = w25n_read_status(priv, W25N_SR3_ADDR);
  if (status & W25N_SR3_EFAIL)
    {
      ferr("ERROR: Erase failed on block %lu\n", (unsigned long)block);
      w25n_writedisable(priv);
      return -EIO;
    }

  w25n_writedisable(priv);
  return OK;
}

/****************************************************************************
 * Name: w25n_enable_ecc
 ****************************************************************************/

static void w25n_enable_ecc(FAR struct w25n_dev_s *priv)
{
  uint8_t sr2;

  sr2 = w25n_read_status(priv, W25N_SR2_ADDR);
  sr2 |= W25N_SR2_ECCE;
  w25n_write_status(priv, W25N_SR2_ADDR, sr2);

  finfo("ECC enabled\n");
}

/****************************************************************************
 * Name: w25n_unprotect
 ****************************************************************************/

static void w25n_unprotect(FAR struct w25n_dev_s *priv)
{
  /* Clear all block protection bits */

  w25n_write_status(priv, W25N_SR1_ADDR, 0x00);

  finfo("All blocks unprotected\n");
}

/****************************************************************************
 * Name: w25n_erase
 ****************************************************************************/

static int w25n_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                      size_t nblocks)
{
  FAR struct w25n_dev_s *priv = (FAR struct w25n_dev_s *)dev;
  size_t i;
  int ret;

  finfo("startblock=%lu nblocks=%zu\n", (unsigned long)startblock, nblocks);

  w25n_lock(priv);

  ret = w25n_waitready(priv);
  if (ret < 0)
    {
      w25n_unlock(priv);
      return ret;
    }

  for (i = 0; i < nblocks; i++)
    {
      ret = w25n_block_erase(priv, startblock + i);
      if (ret < 0)
        {
          w25n_unlock(priv);
          return ret;
        }
    }

  w25n_unlock(priv);
  return (int)nblocks;
}

/****************************************************************************
 * Name: w25n_bread
 ****************************************************************************/

static ssize_t w25n_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buf)
{
  FAR struct w25n_dev_s *priv = (FAR struct w25n_dev_s *)dev;
  size_t i;
  int ret;

  finfo("startblock=%lu nblocks=%zu\n", (unsigned long)startblock, nblocks);

  w25n_lock(priv);

  for (i = 0; i < nblocks; i++)
    {
      uint32_t page = startblock + i;

      ret = w25n_read_page(priv, page);
      if (ret < 0)
        {
          w25n_unlock(priv);
          return ret;
        }

      w25n_read_buffer(priv, 0, buf + (i * W25N_PAGE_SIZE), W25N_PAGE_SIZE);
    }

  w25n_unlock(priv);
  return nblocks;
}

/****************************************************************************
 * Name: w25n_bwrite
 ****************************************************************************/

static ssize_t w25n_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct w25n_dev_s *priv = (FAR struct w25n_dev_s *)dev;
  size_t i;
  int ret;

  finfo("startblock=%lu nblocks=%zu\n", (unsigned long)startblock, nblocks);

  w25n_lock(priv);

  ret = w25n_waitready(priv);
  if (ret < 0)
    {
      w25n_unlock(priv);
      return ret;
    }

  for (i = 0; i < nblocks; i++)
    {
      uint32_t page = startblock + i;

      /* Write Enable must be set before Program Load */

      w25n_writeenable(priv);
      w25n_load_buffer(priv, 0, buf + (i * W25N_PAGE_SIZE), W25N_PAGE_SIZE);
      ret = w25n_program_execute(priv, page);
      if (ret < 0)
        {
          w25n_writedisable(priv);
          w25n_unlock(priv);
          return ret;
        }
    }

  w25n_writedisable(priv);
  w25n_unlock(priv);
  return nblocks;
}

/****************************************************************************
 * Name: w25n_read
 ****************************************************************************/

static ssize_t w25n_read(FAR struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, FAR uint8_t *buf)
{
  FAR struct w25n_dev_s *priv = (FAR struct w25n_dev_s *)dev;
  size_t bytesread = 0;
  int ret;

  finfo("offset=%lu nbytes=%zu\n", (unsigned long)offset, nbytes);

  w25n_lock(priv);

  while (bytesread < nbytes)
    {
      uint32_t page = offset >> priv->pageshift;
      uint16_t col = offset & ((1 << priv->pageshift) - 1);
      size_t chunk = nbytes - bytesread;

      if (chunk > W25N_PAGE_SIZE - col)
        {
          chunk = W25N_PAGE_SIZE - col;
        }

      ret = w25n_read_page(priv, page);
      if (ret < 0)
        {
          w25n_unlock(priv);
          return ret;
        }

      w25n_read_buffer(priv, col, buf + bytesread, chunk);

      bytesread += chunk;
      offset += chunk;
    }

  w25n_unlock(priv);
  return bytesread;
}

/****************************************************************************
 * Name: w25n_ioctl
 ****************************************************************************/

static int w25n_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct w25n_dev_s *priv = (FAR struct w25n_dev_s *)dev;
  int ret = -EINVAL;

  finfo("cmd=%d arg=%lu\n", cmd, arg);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo)
            {
              memset(geo, 0, sizeof(*geo));

              /* Block size is the page size (smallest writable unit) */

              geo->blocksize = W25N_PAGE_SIZE;

              /* Erase size is the block size (smallest erasable unit) */

              geo->erasesize = W25N_BLOCK_SIZE;

              /* Number of erase blocks */

              geo->neraseblocks = priv->nblocks;

              ret = OK;

              finfo("blocksize=%lu erasesize=%lu neraseblocks=%lu\n",
                    (unsigned long)geo->blocksize,
                    (unsigned long)geo->erasesize,
                    (unsigned long)geo->neraseblocks);
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
            (FAR struct partition_info_s *)((uintptr_t)arg);

          if (info != NULL)
            {
              info->numsectors = (uint32_t)priv->nblocks *
                                 W25N_PAGES_PER_BLOCK;
              info->sectorsize = W25N_PAGE_SIZE;
              info->startsector = 0;
              info->parent[0] = '\0';
              ret = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          ret = w25n_erase(dev, 0, priv->nblocks);
          if (ret >= 0)
            {
              ret = OK;
            }
        }
        break;

      case MTDIOC_ECCSTATUS:
        {
          FAR uint8_t *result = (FAR uint8_t *)((uintptr_t)arg);

          if (result)
            {
              *result = priv->eccstatus >> 4;
              ret = OK;
            }
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: w25n_initialize
 *
 * Description:
 *   Create an initialized MTD device instance for W25N SPI NAND FLASH.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *w25n_initialize(FAR struct spi_dev_s *spi,
                                      uint32_t spi_devid)
{
  FAR struct w25n_dev_s *priv;
  uint32_t actual_freq;
  int ret;

  finfo("spi=%p spi_devid=%lu\n", spi, (unsigned long)spi_devid);

  /* Allocate device structure */

  priv = kmm_zalloc(sizeof(struct w25n_dev_s));
  if (priv == NULL)
    {
      ferr("ERROR: Failed to allocate device structure\n");
      return NULL;
    }

  /* Initialize the device structure */

  priv->mtd.erase  = w25n_erase;
  priv->mtd.bread  = w25n_bread;
  priv->mtd.bwrite = w25n_bwrite;
  priv->mtd.read   = w25n_read;
  priv->mtd.ioctl  = w25n_ioctl;
  priv->mtd.name   = "w25n";
  priv->spi        = spi;
  priv->spi_devid  = spi_devid;

  /* Deselect the device */

  SPI_SELECT(spi, SPIDEV_FLASH(spi_devid), false);

  /* Lock and configure SPI */

  w25n_lock(priv);

  /* Reset the device */

  w25n_reset(priv);

  /* Wait for reset and read ID */

  ret = w25n_readid(priv);
  if (ret < 0)
    {
      w25n_unlock(priv);
      kmm_free(priv);
      return NULL;
    }

  /* Unprotect all blocks */

  w25n_unprotect(priv);

  /* Enable ECC */

  w25n_enable_ecc(priv);

  w25n_unlock(priv);

  /* Log actual SPI frequency */

  w25n_lock(priv);
  actual_freq = SPI_SETFREQUENCY(priv->spi, CONFIG_W25N_SPIFREQUENCY);
  w25n_unlock(priv);

  finfo("W25N: SPI freq: requested=%lu, actual=%lu Hz\n",
        (unsigned long)CONFIG_W25N_SPIFREQUENCY,
        (unsigned long)actual_freq);
  finfo("W25N initialized: %d blocks, %d bytes/block\n",
        priv->nblocks, W25N_BLOCK_SIZE);

  return &priv->mtd;
}
