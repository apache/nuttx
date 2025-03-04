/****************************************************************************
 * drivers/mtd/at25ee.c
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
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>

#ifdef CONFIG_MTD_AT25EE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_AT25EE_SPIMODE
#  define CONFIG_AT25EE_SPIMODE 0
#endif

/* EEPROM commands
 * High bit of low nibble used for A8 in 25xx040/at25040 products
 */

#define AT25EE_CMD_WRSR  0x01
#define AT25EE_CMD_WRITE 0x02
#define AT25EE_CMD_READ  0x03
#define AT25EE_CMD_WRDIS 0x04
#define AT25EE_CMD_RDSR  0x05
#define AT25EE_CMD_WREN  0x06

/* Following commands will be available some day via IOCTLs
 *   PE        0x42 Page erase (25xx512/1024)
 *   SE        0xD8 Sector erase (25xx512/1024)
 *   CE        0xC7 Chip erase (25xx512/1024)
 *   RDID      0xAB Wake up and read electronic signature (25xx512/1024)
 *   DPD       0xB9 Sleep (25xx512/1024)
 *
 * Identification page access for ST devices
 *   RDID/RDLS 0x83 Read identification page / Read ID page lock status
 *   WRID/LID  0x82 Write identification page / Lock ID page
 */

/* SR bits definitions */

#define AT25EE_SR_WIP  0x01 /* Write in Progress        */
#define AT25EE_SR_WEL  0x02 /* Write Enable Latch       */
#define AT25EE_SR_BP0  0x04 /* First Block Protect bit  */
#define AT25EE_SR_BP1  0x08 /* Second Block Protect bit */
#define AT25EE_SR_WPEN 0x80 /* Write Protect Enable     */

#define AT25EE_DUMMY   0xFF

/* For applications where a file system is used on the AT25EE, the tiny page
 * sizes will result in very inefficient EEPROM usage.  In such cases, it is
 * better if blocks are comprised of "clusters" of pages so that the file
 * system block size is, say, 256 or 512 bytes.
 * In any event, the block size *must* be an even multiple of the pages.
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Device geometry description, compact form (2 bytes per entry) */

struct at25ee_geom_s
{
  uint8_t bytes    : 4; /* Power of 2 of 128 bytes (0:128 1:256 2:512 etc)  */
  uint8_t pagesize : 4; /* Power of 2 of   8 bytes (0:8 1:16 2:32 3:64 etc) */
  uint8_t addrlen  : 4; /* Number of bytes in command address field         */
  uint8_t flags    : 4; /* Addr. management for 25xx040, 1=A8 in inst       */
};

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct at25ee_dev_s.
 */

struct at25ee_dev_s
{
  struct mtd_dev_s mtd;       /* MTD interface                              */
  struct spi_dev_s *spi;      /* SPI device where the EEPROM is attached    */
  uint32_t         size;      /* in bytes, expanded from geometry           */
  uint16_t         pgsize;    /* write block size, in bytes, expanded from
                               * geometry
                               */
  uint16_t         npages;    /* numpages, derived from geometry            */
  uint16_t         addrlen;   /* number of BITS in data addresses           */
  uint16_t         blocksize; /* Block sized to report                      */
  mutex_t          lock;      /* file access serialization                  */
  uint8_t          readonly;  /* Flags                                      */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void at25ee_lock(FAR struct spi_dev_s *dev);

/* MTD driver methods */

static int at25ee_erase(FAR struct mtd_dev_s *dev,
                        off_t startblock,
                        size_t nblocks);
static ssize_t at25ee_bread(FAR struct mtd_dev_s *dev,
                            off_t startblock,
                            size_t nblocks, FAR uint8_t *buf);
static ssize_t at25ee_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR const uint8_t *buf);
static ssize_t at25ee_read(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR uint8_t *buf);
static ssize_t at25ee_write(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR const uint8_t *buf);
static int at25ee_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                        unsigned long arg);
static void at25ee_writepage(FAR struct at25ee_dev_s *priv, uint32_t devaddr,
                             FAR const uint8_t *data, size_t len);
static void at25ee_writeenable(FAR struct at25ee_dev_s *priv, int enable);
static void at25ee_waitwritecomplete(struct at25ee_dev_s *priv);
static void at25ee_sendcmd(FAR struct spi_dev_s *spi, uint8_t cmd,
                           uint8_t addrlen, uint32_t addr);
static inline void at25ee_unlock(FAR struct spi_dev_s *dev);
static void at25ee_lock(FAR struct spi_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Supported device geometries.
 * One geometry can fit more than one device.
 * The user will use an enum'd index from include/eeprom/spi_xx25xx.h
 */

static const struct at25ee_geom_s g_at25ee_devices[] =
{
  /* Microchip devices */

  {
    0, 1, 1, 0
  }, /* 25xx010A     128   16     1 */
  {
    1, 1, 1, 0
  }, /* 25xx020A     256   16     1 */
  {
    2, 1, 1, 1
  }, /* 25xx040      512   16     1+bit */
  {
    3, 1, 1, 0
  }, /* 25xx080     1024   16     1 */
  {
    3, 2, 2, 0
  }, /* 25xx080B    1024   32     2 */
  {
    4, 1, 2, 0
  }, /* 25xx160     2048   16     2 */
  {
    4, 2, 2, 0
  }, /* 25xx160B/D  2048   32     2 */
  {
    5, 2, 2, 0
  }, /* 25xx320     4096   32     2 */
  {
    6, 2, 2, 0
  }, /* 25xx640     8192   32     2 */
  {
    7, 3, 2, 0
  }, /* 25xx128    16384   64     2 */
  {
    8, 3, 2, 0
  }, /* 25xx256    32768   64     2 */
  {
    9, 4, 2, 0
  }, /* 25xx512    65536  128     2 */
  {
    10, 5, 3, 0
  }, /* 25xx1024  131072  256     3 */

  /* Atmel devices */

  {
    0, 0, 1, 0
  }, /* AT25010B     128    8     1 */
  {
    1, 0, 1, 0
  }, /* AT25020B     256    8     1 */
  {
    2, 0, 1, 1
  }, /* AT25040B     512    8     1+bit */

  /* STM devices */

  {
    11, 5, 3, 0
  }, /* M95M02    262144  256     3 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at25ee_lock
 *
 * Description:
 *    On SPI buses where there are multiple devices, it will be necessary to
 *    lock SPI to have exclusive access to the buses for a sequence of
 *    transfers.  The bus should be locked before the chip is selected.
 *
 *    This is a blocking call and will not return until we have exclusive
 *    access to the SPI bus.  We will retain that exclusive access until the
 *    bus is unlocked.
 *
 *    After locking the SPI bus, the we also need call the setfrequency,
 *    setbits, and setmode methods to make sure that the SPI is properly
 * configured for the device.  If the SPI bus is being shared, then it may
 * have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev         - pointer to device structure
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void at25ee_lock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, true);
  SPI_SETMODE(dev, CONFIG_AT25EE_SPIMODE);
  SPI_SETBITS(dev, 8);
  SPI_HWFEATURES(dev, 0);
  SPI_SETFREQUENCY(dev, CONFIG_AT25EE_SPIFREQUENCY);
#ifdef CONFIG_SPI_DELAY_CONTROL
  SPI_SETDELAY(dev, CONFIG_AT25EE_START_DELAY, CONFIG_AT25EE_STOP_DELAY,
                    CONFIG_AT25EE_CS_DELAY, CONFIG_AT25EE_IFDELAY);
#endif
}

/****************************************************************************
 * Name: at25ee_unlock
 *
 * Description:
 *    Unlocks the SPI bus
 *
 * Input Parameters:
 *   dev         - pointer to device structure
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static inline void at25ee_unlock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, false);
}

/****************************************************************************
 * Name: at25ee_sendcmd
 *
 * Description:
 *    Send command and address as one transaction to take advantage
 *    of possible faster DMA transfers.
 *    Sending byte per byte is MUCH slower.
 *
 * Input Parameters:
 *   spi         - a reference to the spi device
 *   cmd         - SPI command to send
 *   addrlen     - length of the address, in bits
 *   addr        - address to write to
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void at25ee_sendcmd(FAR struct spi_dev_s *spi, uint8_t cmd,
                           uint8_t addrlen, uint32_t addr)
{
  uint8_t buf[4];
  int     cmdlen = 1;

  /* Store command */

  buf[0] = cmd;

  /* Store address according to its length */

  if (addrlen == 9)
    {
      buf[0] |= (((addr >> 8) & 1) << 3);
    }

  if (addrlen > 16)
    {
      buf[cmdlen++] = (addr >> 16) & 0xff;
    }

  if (addrlen > 9)
    {
      buf[cmdlen++] = (addr >>  8) & 0xff;
    }

  buf[cmdlen++]   =  addr        & 0xff;

  SPI_SNDBLOCK(spi, buf, cmdlen);
}

/****************************************************************************
 * Name: at25ee_waitwritecomplete
 *
 * Description:
 *   loop until the write operation is done.
 *
 * Input Parameters:
 *   priv        - a reference to the device structure
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void at25ee_waitwritecomplete(struct at25ee_dev_s *priv)
{
  uint8_t status;

  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Select this FLASH part */

      at25ee_lock(priv->spi);
      SPI_SELECT(priv->spi, SPIDEV_EEPROM(0), true);

      /* Send "Read Status Register (RDSR)" command */

      SPI_SEND(priv->spi, AT25EE_CMD_RDSR);

      /* Send a dummy byte to generate the clock needed to shift out the
       * status
       */

      status = SPI_SEND(priv->spi, AT25EE_DUMMY);

      /* Deselect the FLASH */

      SPI_SELECT(priv->spi, SPIDEV_EEPROM(0), false);
      at25ee_unlock(priv->spi);

      /* Given that writing could take up to a few milliseconds,
       * the following short delay in the "busy" case will allow
       * other peripherals to access the SPI bus.
       */

      if ((status & AT25EE_SR_WIP) != 0)
        {
          nxsig_usleep(1000);
        }
    }
  while ((status & AT25EE_SR_WIP) != 0);
}

/****************************************************************************
 * Name: at25ee_writeenable
 *
 * Description:
 *    Enable or disable write operations.
 *    This is required before any write, since a lot of operations
 *    automatically disable the write latch.
 *
 * Input Parameters:
 *   priv        - a reference to the device structure
 *   enable      - enable (true) or disable(false) write operations
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void at25ee_writeenable(FAR struct at25ee_dev_s *priv, int enable)
{
  at25ee_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_EEPROM(0), true);

  SPI_SEND(priv->spi, enable ? AT25EE_CMD_WREN : AT25EE_CMD_WRDIS);

  SPI_SELECT(priv->spi, SPIDEV_EEPROM(0), false);
  at25ee_unlock(priv->spi);
}

/****************************************************************************
 * Name: at25ee_writepage
 *
 * Description:
 *   Write data to the EEPROM, NOT crossing page boundaries.
 *
 * Input Parameters:
 *   priv        - a reference to the device structure
 *   devaddr     - the address to start the write
 *   data        - pointer to data buffer to write
 *   len         - length of the data to write
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void at25ee_writepage(FAR struct at25ee_dev_s *priv, uint32_t devaddr,
                             FAR const uint8_t *data, size_t len)
{
  at25ee_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_EEPROM(0), true);

  at25ee_sendcmd(priv->spi, AT25EE_CMD_WRITE, priv->addrlen, devaddr);
  SPI_SNDBLOCK(priv->spi, data, len);

  SPI_SELECT(priv->spi, SPIDEV_EEPROM(0), false);
  at25ee_unlock(priv->spi);
}

/****************************************************************************
 * Name: at25ee_eraseall
 *
 * Description:
 *   Erase all data in the device
 *
 * Input Parameters:
 *   priv        - a reference to the device structure
 *   devaddr     - the address to start the write
 *   data        - pointer to data buffer to write
 *   len         - length of the data to write
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static int at25ee_eraseall(FAR struct at25ee_dev_s *priv)
{
  uint8_t *buf;
  int startblock = 0;

  DEBUGASSERT(priv);

  buf = kmm_malloc(priv->pgsize);
  if (!buf)
    {
      ferr("ERROR: Failed to alloc memory for at25ee eraseall!\n");
      return -ENOMEM;
    }

  memset(buf, 0xff, priv->pgsize);

  for (startblock = 0; startblock < priv->npages; startblock++)
    {
      uint16_t offset = startblock * priv->pgsize;
      at25ee_write(&priv->mtd, offset, priv->pgsize, buf);
    }

  kmm_free(buf);
  return OK;
}

/****************************************************************************
 * Name: at25ee_erase
 *
 * Description:
 *   Erase a number of blocks of data.
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   startblock - start block of the erase
 *   nblocks    - nblocks to erase
 *
 * Returned Value:
 *   Success (OK) or fail (negated error code)
 ****************************************************************************/

static int at25ee_erase(FAR struct mtd_dev_s *dev,
                       off_t startblock,
                       size_t nblocks)
{
#ifndef CONFIG_AT25EE_ENABLE_BLOCK_ERASE
  return (int)nblocks;
#else
  FAR struct at25ee_dev_s *priv = (FAR struct at25ee_dev_s *)dev;
  uint8_t *buf;
  size_t blocksleft;

  DEBUGASSERT(dev);

  if (priv->blocksize > priv->pgsize)
    {
      startblock *= (priv->blocksize / priv->pgsize);
      nblocks    *= (priv->blocksize / priv->pgsize);
    }

  blocksleft = nblocks;

  if (startblock >= priv->npages)
    {
      return -E2BIG;
    }

  buf = kmm_malloc(priv->pgsize);
  if (!buf)
    {
      ferr("ERROR: Failed to alloc memory for at25ee erase!\n");
      return -ENOMEM;
    }

  memset(buf, 0xff, priv->pgsize);

  if (startblock + nblocks > priv->npages)
    {
      nblocks = priv->npages - startblock;
    }

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  while (blocksleft-- > 0)
    {
      off_t offset = startblock * priv->pgsize;

      finfo("startblock: %08lx offset: %d\n", (long)startblock, (int)offset);
      at25ee_write(dev, offset, priv->pgsize, buf);
      startblock++;
    }

  kmm_free(buf);
  if (priv->blocksize > priv->pgsize)
    {
      return (int)(nblocks / (priv->blocksize / priv->pgsize));
    }
  else
    {
      return (int)nblocks;
    }
#endif
}

/****************************************************************************
 * Name: at25ee_read
 *
 * Description:
 *   Read a number of bytes of data.
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   offset     - start of the memory to read
 *   nbytes     - number of bytes to read
 *   buffer     - pointer to variable to store the read data
 *
 * Returned Value:
 *   Size of the data read
 ****************************************************************************/

static ssize_t at25ee_read(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR uint8_t *buf)
{
  int ret;
  FAR struct at25ee_dev_s *priv = (FAR struct at25ee_dev_s *)dev;

  DEBUGASSERT(buf);
  DEBUGASSERT(dev);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if ((offset + nbytes) > priv->size)
    {
      return 0; /* end-of-file */
    }

  at25ee_lock(priv->spi);

  SPI_SELECT(priv->spi, SPIDEV_EEPROM(0), true);

  /* STM32F4Disco: There is a 25 us delay here */

  at25ee_sendcmd(priv->spi, AT25EE_CMD_READ, priv->addrlen, offset);

  SPI_RECVBLOCK(priv->spi, buf, nbytes);

  SPI_SELECT(priv->spi, SPIDEV_EEPROM(0), false);

  at25ee_unlock(priv->spi);

  nxmutex_unlock(&priv->lock);
  return nbytes;
}

/****************************************************************************
 * Name: at25ee_write
 *
 * Description:
 *   Write a number of bytes of data.
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   offset     - start of the memory to write
 *   nbytes     - number of bytes to write
 *   buf        - pointer to buffer of data to write
 *
 * Returned Value:
 *   Size of the data written
 ****************************************************************************/

static ssize_t at25ee_write(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR const uint8_t *buf)
{
  int                     ret   = -EACCES;
  FAR struct at25ee_dev_s *priv = (FAR struct at25ee_dev_s *)dev;
  int                     pageoff;
  size_t                  cnt;

  DEBUGASSERT(buf);
  DEBUGASSERT(dev);

  if (priv->readonly)
    {
      return -EPERM;
    }

  /* Forbid writes past the end of the device */

  if (nbytes + offset >= priv->size)
    {
      return 0;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return 0;
    }

  /* From this point no failure cannot be detected anymore.
   * The user should verify the write by rereading memory.
   */

  ret = nbytes; /* save number of bytes written */

  /* Writes can't happen in a row like the read does.
   * The EEPROM is made of pages, and write sequences
   * cannot cross page boundaries. So every time the last
   * byte of a page is programmed, the SPI transaction is
   * stopped, and the status register is read until the
   * write operation has completed.
   */

  /* First, write some page-unaligned data */

  pageoff = offset & (priv->pgsize - 1);
  cnt     = priv->pgsize - pageoff;
  if (cnt > nbytes)
    {
      cnt = nbytes;
    }

  if (pageoff > 0)
    {
      at25ee_writeenable(priv, true);
      at25ee_writepage(priv, offset, buf, cnt);
      at25ee_waitwritecomplete(priv);
      nbytes -= cnt;
      buf    += cnt;
      offset += cnt;
    }

  /* Then, write remaining bytes at page-aligned addresses */

  while (nbytes > 0)
    {
      cnt = nbytes;
      if (cnt > priv->pgsize)
        {
          cnt = priv->pgsize;
        }

      at25ee_writeenable(priv, true);
      at25ee_writepage(priv, offset, buf, cnt);
      at25ee_waitwritecomplete(priv);
      nbytes -= cnt;
      buf    += cnt;
      offset += cnt;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: at25ee_bread
 *
 * Description:
 *   Read a number of blocks of data.
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   startblock - start block of the read
 *   nblocks    - nblocks to read
 *   buf        - pointer to variable to store the read data
 *
 * Returned Value:
 *   Number of blocks written
 ****************************************************************************/

static ssize_t at25ee_bread(FAR struct mtd_dev_s *dev,
                           off_t startblock,
                           size_t nblocks, FAR uint8_t *buf)
{
  FAR struct at25ee_dev_s *priv = (FAR struct at25ee_dev_s *)dev;
  off_t offset;
  ssize_t nread;
  size_t i;

  DEBUGASSERT(dev);
  DEBUGASSERT(buf);

  if (priv->blocksize > priv->pgsize)
    {
      startblock *= (priv->blocksize / priv->pgsize);
      nblocks    *= (priv->blocksize / priv->pgsize);
    }

  finfo("startblock: %08lx nblocks: %lu\n",
        (unsigned long)startblock, (unsigned long)nblocks);

  if (startblock >= priv->npages)
    {
      return 0;
    }

  if (startblock + nblocks > priv->npages)
    {
      nblocks = priv->npages - startblock;
    }

  /* Convert the access from startblock and number of blocks to a byte
   * offset and number of bytes.
   */

  offset = startblock * priv->pgsize;

  /* Then perform the byte-oriented read for each block separately */

  for (i = 0; i < nblocks; i++)
    {
      nread = at25ee_read(dev, offset, priv->pgsize, buf);
      if (nread < 0)
        {
          return nread;
        }

      offset += priv->pgsize;
      buf += priv->pgsize;
    }

  if (priv->blocksize > priv->pgsize)
    {
      return nblocks / (priv->blocksize / priv->pgsize);
    }
  else
    {
      return nblocks;
    }
}

/****************************************************************************
 * Name: at25ee_bwrite
 *
 * Description:
 *   Write a number of blocks of data.
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   startblock - starting block to write to
 *   nblocks    - nblocks to write
 *   buf        - pointer to data buffer to write
 *
 * Returned Value:
 *   Size of the data written
 ****************************************************************************/

static ssize_t at25ee_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct at25ee_dev_s *priv = (FAR struct at25ee_dev_s *)dev;
  size_t blocksleft;

  DEBUGASSERT(dev);
  DEBUGASSERT(buf);

  if (priv->blocksize > priv->pgsize)
    {
      startblock *= (priv->blocksize / priv->pgsize);
      nblocks    *= (priv->blocksize / priv->pgsize);
    }

  blocksleft = nblocks;

  if (startblock >= priv->npages)
    {
      return 0;
    }

  if (startblock + nblocks > priv->npages)
    {
      nblocks = priv->npages - startblock;
    }

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  while (blocksleft-- > 0)
    {
      off_t offset = startblock * priv->pgsize;

      finfo("startblock: %08lx offset: %d\n", (long)startblock, (int)offset);
      at25ee_write(dev, offset, priv->pgsize, buf);
      startblock++;
      buf += priv->pgsize;
    }

  if (priv->blocksize > priv->pgsize)
    {
      return nblocks / (priv->blocksize / priv->pgsize);
    }
  else
    {
      return nblocks;
    }
}

/****************************************************************************
 * Name: at25ee_ioctl
 *  * Description:
 *   IOCTLS relating to the EEPROM mtd device
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   cmd        - ioctl command
 *   arg        - ioctl argument
 *
 * Returned Value:
 *   Success (OK) or fail (negated error code)
 ****************************************************************************/

static int at25ee_ioctl(FAR struct mtd_dev_s *dev,
                       int cmd,
                       unsigned long arg)
{
  FAR struct at25ee_dev_s *priv = (FAR struct at25ee_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  DEBUGASSERT(dev);

  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
         FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)
                                          ((uintptr_t)arg);
          if (geo)
            {
              memset(geo, 0, sizeof(*geo));

              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an array
               * of fixed size blocks.
               * That is most likely not true, but the client will expect the
               * device logic to do whatever is necessary to make it appear
               * so.
               *
               * blocksize:
               *   May be user defined.
               *   The block size for the at24XX devices may be larger than
               *   the page size in order to better support file systems.
               *   The read and write functions translate BLOCKS to pages
               *   for the small flash devices
               * erasesize:
               *   It has to be at least as big as the blocksize, bigger
               *   serves no purpose.
               * neraseblocks
               *   Note that the device size is in kilobits and must be
               *   scaled by 1024 / 8
               */

              if (priv->blocksize > priv->pgsize)
                {
                  geo->blocksize    = priv->blocksize;
                  geo->erasesize    = priv->blocksize;
                  geo->neraseblocks = priv->size / priv->blocksize;
                }
              else
                {
                  geo->blocksize    = priv->pgsize;
                  geo->erasesize    = priv->pgsize;
                  geo->neraseblocks = priv->npages;
                }

              ret = OK;

              finfo("blocksize: %" PRId32 " erasesize: %" PRId32
                    " neraseblocks: %" PRId32 "\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
              (FAR struct partition_info_s *)arg;
          if (info != NULL)
            {
              if (priv->blocksize > priv->pgsize)
                {
                  info->numsectors  = priv->size / priv->blocksize;
                  info->sectorsize  = priv->blocksize;
                }
              else
                {
                  info->numsectors  = priv->npages;
                  info->sectorsize  = priv->pgsize;
                }

              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        ret = at25ee_eraseall(priv);
        break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at25ee_initialize
 *
 * Description:
 *   Create an initialized MTD device instance for an AT25 SPI EEPROM
 *   MTD devices are not registered in the file system, but are created
 *   as instances that can be bound to other functions
 *   (such as a block or character driver front end).
 *
 * Input Parameters:
 *   dev        - a reference to the spi device structure
 *   devtype    - device type, from include/nuttx/eeprom/spi_xx25xx.h
 *   readonly   - sets block driver to be readonly
 *
 * Returned Value:
 *   Initialised device instance (success) or NULL (fail)
 *
 ****************************************************************************/

FAR struct mtd_dev_s *at25ee_initialize(FAR struct spi_dev_s *dev,
                                        int devtype, int readonly)
{
  FAR struct at25ee_dev_s *priv;

  DEBUGASSERT(dev);

  /* Check device type early */

  if ((devtype < 0) ||
      (devtype >= sizeof(g_at25ee_devices) / sizeof(g_at25ee_devices[0])))
    {
      return NULL;
    }

  priv = kmm_zalloc(sizeof(struct at25ee_dev_s));
  if (priv == NULL)
    {
      ferr("ERROR: Failed to allocate device structure\n");
      return NULL;
    }

  /* Initialize the allocated structure */

  nxmutex_init(&priv->lock);

  priv->spi      = dev;
  priv->size     = 128 << g_at25ee_devices[devtype].bytes;
  priv->pgsize   =   8 << g_at25ee_devices[devtype].pagesize;
  priv->addrlen  =        g_at25ee_devices[devtype].addrlen << 3;
  priv->npages   = priv->size / priv->pgsize;
#ifdef CONFIG_USE_NATIVE_AT25EE_BLOCK_SIZE
  priv->blocksize = priv->pgsize;
#else
  if ((CONFIG_MANUAL_AT25EE_BLOCK_SIZE % priv->pgsize) ||
      (CONFIG_MANUAL_AT25EE_BLOCK_SIZE > priv->size))
    {
      ferr("ERROR: Configured block size is incorrect!\n");
      DEBUGASSERT(0);
      priv->blocksize = priv->pgsize;
    }
  else
    {
      priv->blocksize = CONFIG_MANUAL_AT25EE_BLOCK_SIZE;
    }

#endif
  if ((g_at25ee_devices[devtype].flags & 1))
    {
      priv->addrlen = 9;
    }

  priv->readonly = !!readonly;

  finfo("EEPROM device, %"PRIu32" bytes, "
        "%u per page, addrlen %u, readonly %d\n",
        priv->size, priv->pgsize, priv->addrlen,
        priv->readonly);

  priv->mtd.erase  = at25ee_erase;
  priv->mtd.bread  = at25ee_bread;
  priv->mtd.bwrite = at25ee_bwrite;
  priv->mtd.read   = at25ee_read;
  priv->mtd.write  = at25ee_write;
  priv->mtd.ioctl  = at25ee_ioctl;
  priv->mtd.name   = "at25ee";

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}

#endif /* CONFIG_MTD_AT25EE */
