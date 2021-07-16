/***************************************************************************
 * drivers/mtd/gd25.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <inttypes.h>
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

/***************************************************************************
 * Configuration
 ***************************************************************************/

#ifndef CONFIG_GD25_SPIMODE
#  define CONFIG_GD25_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_GD25_SPIFREQUENCY
#  define CONFIG_GD25_SPIFREQUENCY 20000000
#endif

/***************************************************************************
 * GD25 Instructions
 ***************************************************************************/

/*      Command                    Value      Description                 */

#define GD25_WREN                   0x06    /* Write enable               */
#define GD25_WRDI                   0x04    /* Write Disable              */
#define GD25_RDSR                   0x05    /* Read status register       */
#define GD25_RDSR1                  0x35    /* Read status register-1     */
#define GD25_WRSR                   0x01    /* Write Status Register      */
#define GD25_RDDATA                 0x03    /* Read data bytes            */
#define GD25_FRD                    0x0b    /* Higher speed read          */
#define GD25_FRDD                   0x3b    /* Fast read, dual output     */
#define GD25_PP                     0x02    /* Program page               */
#define GD25_SE                     0x20    /* Sector erase (4KB)         */
#define GD25_BE                     0xd8    /* Block Erase (64KB)         */
#define GD25_CE                     0xc7    /* Chip erase                 */
#define GD25_PD                     0xb9    /* Power down                 */
#define GD25_PURDID                 0xab    /* Release PD, Device ID      */
#define GD25_RDMFID                 0x90    /* Read Manufacturer / Device */
#define GD25_JEDEC_ID               0x9f    /* JEDEC ID read              */
#define GD25_4BEN                   0xb7    /* Enable 4-byte Mode         */

/***************************************************************************
 * GD25 Registers
 ***************************************************************************/

/* JEDEC Read ID register values */

#define P25_JEDEC_MANUFACTURER      0x85
#define GD25_JEDEC_MANUFACTURER     0xc8  /* GigaDevice manufacturer ID */
#define GD25L_JEDEC_MEMORY_TYPE     0x60  /* GD25L memory type, 1.8V */
#define GD25Q_JEDEC_MEMORY_TYPE     0x40  /* GD25Q memory type, 3V */

#define GD25_JEDEC_CAPACITY_8MBIT   0x14  /* 256x4096 = 8Mbit memory capacity */
#define GD25_JEDEC_CAPACITY_16MBIT  0x15  /* 512x4096  = 16Mbit memory capacity */
#define GD25_JEDEC_CAPACITY_32MBIT  0x16  /* 1024x4096 = 32Mbit memory capacity */
#define GD25_JEDEC_CAPACITY_64MBIT  0x17  /* 2048x4096 = 64Mbit memory capacity */
#define GD25_JEDEC_CAPACITY_128MBIT 0x18  /* 4096x4096 = 128Mbit memory capacity */
#define GD25_JEDEC_CAPACITY_256MBIT 0x19  /* 8192x4096 = 256Mbit memory capacity */

#define GD25_NSECTORS_8MBIT         256   /* 256 sectors x 4096 bytes/sector = 1Mb */
#define GD25_NSECTORS_16MBIT        512   /* 512 sectors x 4096 bytes/sector = 2Mb */
#define GD25_NSECTORS_32MBIT        1024  /* 1024 sectors x 4096 bytes/sector = 4Mb */
#define GD25_NSECTORS_64MBIT        2048  /* 2048 sectors x 4096 bytes/sector = 8Mb */
#define GD25_NSECTORS_128MBIT       4096  /* 4096 sectors x 4096 bytes/sector = 16Mb */
#define GD25_NSECTORS_256MBIT       8192  /* 8192 sectors x 4096 bytes/sector = 32Mb */

/* Status register bit definitions */

#define GD25_SR_WIP                 (1 << 0)  /* Bit 0: Write in Progress */
#define GD25_SR_WEL                 (1 << 1)  /* Bit 1: Write Enable Latch */
#define GD25_SR1_EN4B               (1 << 3)  /* Bit 3: Enable 4byte address */

#define GD25_DUMMY                  0x00

/***************************************************************************
 * Chip Geometries
 ***************************************************************************/

/* All members of the family support uniform 4KB sectors and 256B pages */

#define GD25_SECTOR_SHIFT           12        /* Sector size 1 << 12 = 4Kb */
#define GD25_SECTOR_SIZE            (1 << 12) /* Sector size 1 << 12 = 4Kb */
#define GD25_PAGE_SHIFT             8         /* Sector size 1 << 8 = 256b */
#define GD25_PAGE_SIZE              (1 << 8)  /* Sector size 1 << 8 = 256b */

#define GD25_ERASED_STATE           0xff      /* State of FLASH when erased */

/***************************************************************************
 * Private Types
 ***************************************************************************/

/* This type represents the state of the MTD device. The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct gd25_dev_s.
 */

struct gd25_dev_s
{
  struct mtd_dev_s      mtd;         /* MTD interface */
  FAR struct spi_dev_s *spi;         /* Saved SPI interface instance */
  uint32_t              spi_devid;   /* Chip select inputs */
  uint16_t              nsectors;    /* Number of erase sectors */
  uint8_t               prev_instr;  /* Previous instruction given to GD25 device */
  bool                  addr_4byte;  /* True: Use Four-byte address */
};

/***************************************************************************
 * Private Function Prototypes
 ***************************************************************************/

/* Helpers */

static inline void gd25_purdid(FAR struct gd25_dev_s *priv);
static inline void gd25_pd(FAR struct gd25_dev_s *priv);
static void gd25_lock(FAR struct spi_dev_s *spi);
static inline void gd25_unlock(FAR struct spi_dev_s *spi);
static inline int gd25_readid(FAR struct gd25_dev_s *priv);
#ifndef CONFIG_GD25_READONLY
static void gd25_unprotect(FAR struct gd25_dev_s *priv);
#endif
static uint8_t gd25_waitwritecomplete(FAR struct gd25_dev_s *priv);
static inline void gd25_wren(FAR struct gd25_dev_s *priv);
static inline void gd25_wrdi(FAR struct gd25_dev_s *priv);
static bool gd25_is_erased(FAR struct gd25_dev_s *priv, off_t address,
        off_t size);
static void gd25_sectorerase(FAR struct gd25_dev_s *priv, off_t offset);
static inline int gd25_chiperase(FAR struct gd25_dev_s *priv);
static void gd25_byteread(FAR struct gd25_dev_s *priv, FAR uint8_t *buffer,
    off_t address, size_t nbytes);
#ifndef CONFIG_GD25_READONLY
static void gd25_pagewrite(FAR struct gd25_dev_s *priv,
    FAR const uint8_t *buffer, off_t address, size_t nbytes);
#endif
static inline uint8_t gd25_rdsr(FAR struct gd25_dev_s *priv, uint32_t id);
static inline void gd25_4ben(FAR struct gd25_dev_s *priv);

/* MTD driver methods */

static int gd25_erase(FAR struct mtd_dev_s *dev, off_t startblock,
    size_t nblocks);
static ssize_t gd25_bread(FAR struct mtd_dev_s *dev, off_t startblock,
    size_t nblocks, FAR uint8_t *buf);
static ssize_t gd25_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
    size_t nblocks, FAR const uint8_t *buf);
static ssize_t gd25_read(FAR struct mtd_dev_s *dev, off_t offset,
    size_t nbytes, FAR uint8_t *buffer);
static int gd25_ioctl(FAR struct mtd_dev_s *dev, int cmd,
    unsigned long arg);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t gd25_write(FAR struct mtd_dev_s *dev, off_t offset,
    size_t nbytes, FAR const uint8_t *buffer);
#endif

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/***************************************************************************
 * Name: gd25_purdid
 ***************************************************************************/

static inline void gd25_purdid(FAR struct gd25_dev_s *priv)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, GD25_PURDID);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
  up_udelay(20);
}

/***************************************************************************
 * Name: gd25_pd
 ***************************************************************************/

static inline void gd25_pd(FAR struct gd25_dev_s *priv)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, GD25_PD);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
}

/***************************************************************************
 * Name: gd25_lock
 ***************************************************************************/

static void gd25_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, true);

  SPI_SETMODE(spi, CONFIG_GD25_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_GD25_SPIFREQUENCY);
}

/***************************************************************************
 * Name: gd25_unlock
 ***************************************************************************/

static inline void gd25_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, false);
}

/***************************************************************************
 * Name: gd25_readid
 ***************************************************************************/

static inline int gd25_readid(FAR struct gd25_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t memory;
  uint16_t capacity;
  int ret = -ENODEV;

  /* Lock and configure the SPI bus */

  gd25_lock(priv->spi);
  gd25_purdid(priv);

  /* Select this FLASH part. */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the "Read ID (RDID)" command and read the first three ID bytes */

  SPI_SEND(priv->spi, GD25_JEDEC_ID);
  manufacturer = SPI_SEND(priv->spi, GD25_DUMMY);
  memory       = SPI_SEND(priv->spi, GD25_DUMMY);
  capacity     = SPI_SEND(priv->spi, GD25_DUMMY);

  /* Deselect the FLASH and unlock the bus */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);

  finfo("manufacturer: %02x memory: %02x capacity: %02x\n",
        manufacturer, memory, capacity);

  /* Check for a valid manufacturer and memory type */

  if ((manufacturer == GD25_JEDEC_MANUFACTURER ||
       manufacturer == P25_JEDEC_MANUFACTURER) &&
      (memory == GD25L_JEDEC_MEMORY_TYPE ||
       memory == GD25Q_JEDEC_MEMORY_TYPE))
    {
      if (capacity == GD25_JEDEC_CAPACITY_8MBIT)
        {
          priv->nsectors = GD25_NSECTORS_8MBIT;
        }
      else if (capacity == GD25_JEDEC_CAPACITY_16MBIT)
        {
          priv->nsectors = GD25_NSECTORS_16MBIT;
        }
      else if (capacity == GD25_JEDEC_CAPACITY_32MBIT)
        {
          priv->nsectors = GD25_NSECTORS_32MBIT;
        }
      else if (capacity == GD25_JEDEC_CAPACITY_64MBIT)
        {
          priv->nsectors = GD25_NSECTORS_64MBIT;
        }
      else if (capacity == GD25_JEDEC_CAPACITY_128MBIT)
        {
          priv->nsectors = GD25_NSECTORS_128MBIT;
        }
      else if (capacity == GD25_JEDEC_CAPACITY_256MBIT)
        {
          priv->nsectors = GD25_NSECTORS_256MBIT;
        }
      else
        {
          goto out;
        }

      /* Capacity greater than 16MB, Enable four-byte address */

      if (priv->nsectors > GD25_NSECTORS_128MBIT)
        {
          gd25_4ben(priv);

          if ((gd25_rdsr(priv, 1) & GD25_SR1_EN4B) != GD25_SR1_EN4B)
            {
              ferr("ERROR: capacity %02x: Can't enable 4-byte mode!\n",
                   capacity);
              ret = -EBUSY;
              goto out;
            }

          priv->addr_4byte = true;
        }

      ret = OK;
    }

out:
  /* We don't understand the manufacturer or the memory type.
   * Or enable four-byte address failed.
   * Or success.
   */

  gd25_pd(priv);
  gd25_unlock(priv->spi);
  return ret;
}

/***************************************************************************
 * Name: gd25_unprotect
 ***************************************************************************/

#ifndef CONFIG_GD25_READONLY
static void gd25_unprotect(FAR struct gd25_dev_s *priv)
{
  /* Lock and configure the SPI bus */

  gd25_lock(priv->spi);
  gd25_purdid(priv);

  /* Wait for any preceding write or erase operation to complete. */

  gd25_waitwritecomplete(priv);

  /* Send "Write enable (WREN)" */

  gd25_wren(priv);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send "Write enable status (EWSR)" */

  SPI_SEND(priv->spi, GD25_WRSR);

  /* Following by the new status value */

  SPI_SEND(priv->spi, 0);
  SPI_SEND(priv->spi, 0);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);

  /* Unlock the SPI bus */

  gd25_pd(priv);
  gd25_unlock(priv->spi);
}
#endif

/***************************************************************************
 * Name: gd25_waitwritecomplete
 ***************************************************************************/

static uint8_t gd25_waitwritecomplete(FAR struct gd25_dev_s *priv)
{
  uint8_t status;

  do
    {
      status = gd25_rdsr(priv, 0);
      if (priv->prev_instr != GD25_PP && (status & GD25_SR_WIP) != 0)
        {
          gd25_unlock(priv->spi);
          nxsig_usleep(1000);
          gd25_lock(priv->spi);
        }
    }
  while ((status & GD25_SR_WIP) != 0);

  return status;
}

/***************************************************************************
 * Name:  gd25_rdsr
 ***************************************************************************/

static inline uint8_t gd25_rdsr(FAR struct gd25_dev_s *priv, uint32_t id)
{
  uint8_t status;
  uint8_t rdsr[2] =
  {
    GD25_RDSR, GD25_RDSR1
  };

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, rdsr[id]);
  status = SPI_SEND(priv->spi, GD25_DUMMY);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);

  return status;
}

/***************************************************************************
 * Name:  gd25_4ben
 ***************************************************************************/

static inline void gd25_4ben(FAR struct gd25_dev_s *priv)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, GD25_4BEN);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
}

/***************************************************************************
 * Name:  gd25_wren
 ***************************************************************************/

static inline void gd25_wren(FAR struct gd25_dev_s *priv)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, GD25_WREN);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
}

/***************************************************************************
 * Name:  gd25_wrdi
 ***************************************************************************/

static inline void gd25_wrdi(FAR struct gd25_dev_s *priv)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, GD25_WRDI);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
}

/***************************************************************************
 * Name:  gd25_is_erased
 ***************************************************************************/

static bool gd25_is_erased(FAR struct gd25_dev_s *priv, off_t address,
                           off_t size)
{
  size_t npages = size >> GD25_PAGE_SHIFT;
  uint32_t erased_32;
  unsigned int i;
  uint32_t buf[GD25_PAGE_SIZE / sizeof(uint32_t)];

  DEBUGASSERT((address % GD25_PAGE_SIZE) == 0);
  DEBUGASSERT((size % GD25_PAGE_SIZE) == 0);

  memset(&erased_32, GD25_ERASED_STATE, sizeof(erased_32));

  /* Walk all pages in given area. */

  while (npages)
    {
      /* Check if all bytes of page is in erased state. */

      gd25_byteread(priv, (uint8_t *)buf, address, GD25_PAGE_SIZE);

      for (i = 0; i < GD25_PAGE_SIZE / sizeof(uint32_t); i++)
        {
          if (buf[i] != erased_32)
            {
              /* Page not in erased state! */

              return false;
            }
        }

      address += GD25_PAGE_SIZE;
      npages--;
    }

  return true;
}

/***************************************************************************
 * Name:  gd25_sectorerase
 ***************************************************************************/

static void gd25_sectorerase(FAR struct gd25_dev_s *priv, off_t sector)
{
  off_t address = sector << GD25_SECTOR_SHIFT;

  finfo("sector: %08lx\n", (long)sector);

  /* Check if sector is already erased. */

  if (gd25_is_erased(priv, address, GD25_SECTOR_SIZE))
    {
      /* Sector already in erased state, so skip erase. */

      return;
    }

  /* Wait for any preceding write or erase operation to complete. */

  gd25_waitwritecomplete(priv);

  /* Send write enable instruction */

  gd25_wren(priv);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the "Sector Erase (SE)" instruction */

  SPI_SEND(priv->spi, GD25_SE);
  priv->prev_instr = GD25_SE;

  /* Send the sector address high byte first.  Only the most significant
   * bits (those corresponding to the sector) have any meaning.
   */

  if (priv->addr_4byte)
    {
      SPI_SEND(priv->spi, (address >> 24) & 0xff);
    }

  SPI_SEND(priv->spi, (address >> 16) & 0xff);
  SPI_SEND(priv->spi, (address >> 8) & 0xff);
  SPI_SEND(priv->spi, address & 0xff);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
}

/***************************************************************************
 * Name:  gd25_chiperase
 ***************************************************************************/

static inline int gd25_chiperase(FAR struct gd25_dev_s *priv)
{
  /* Wait for any preceding write or erase operation to complete. */

  gd25_waitwritecomplete(priv);

  /* Send write enable instruction */

  gd25_wren(priv);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the "Chip Erase (CE)" instruction */

  SPI_SEND(priv->spi, GD25_CE);
  priv->prev_instr = GD25_CE;

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
  return OK;
}

/***************************************************************************
 * Name: gd25_byteread
 ***************************************************************************/

static void gd25_byteread(FAR struct gd25_dev_s *priv, FAR uint8_t *buffer,
                          off_t address, size_t nbytes)
{
  finfo("address: %08lx nbytes: %d\n", (long)address, (int)nbytes);

  /* Wait for any preceding write or erase operation to complete. */

  gd25_waitwritecomplete(priv);

  /* Make sure that writing is disabled */

  gd25_wrdi(priv);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send "Read from Memory " instruction */

#ifdef CONFIG_GD25_SLOWREAD
  SPI_SEND(priv->spi, GD25_RDDATA);
  priv->prev_instr = GD25_RDDATA;
#else
  SPI_SEND(priv->spi, GD25_FRD);
  priv->prev_instr = GD25_FRD;
#endif

  /* Send the address high byte first. */

  if (priv->addr_4byte)
    {
      SPI_SEND(priv->spi, (address >> 24) & 0xff);
    }

  SPI_SEND(priv->spi, (address >> 16) & 0xff);
  SPI_SEND(priv->spi, (address >> 8) & 0xff);
  SPI_SEND(priv->spi, address & 0xff);

  /* Send a dummy byte */

#ifndef CONFIG_GD25_SLOWREAD
  SPI_SEND(priv->spi, GD25_DUMMY);
#endif

  /* Then read all of the requested bytes */

  SPI_RECVBLOCK(priv->spi, buffer, nbytes);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
}

/***************************************************************************
 * Name:  gd25_pagewrite
 ***************************************************************************/

#ifndef CONFIG_GD25_READONLY
static void gd25_pagewrite(FAR struct gd25_dev_s *priv,
                           FAR const uint8_t *buffer, off_t address,
                           size_t nbytes)
{
  finfo("address: %08lx nwords: %d\n", (long)address, (int)nbytes);
  DEBUGASSERT(priv && buffer && (address & 0xff) == 0 &&
              (nbytes & 0xff) == 0);

  for (; nbytes > 0; nbytes -= GD25_PAGE_SIZE)
    {
      /* Wait for any preceding write or erase operation to complete. */

      gd25_waitwritecomplete(priv);

      /* Enable write access to the FLASH */

      gd25_wren(priv);

      SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);

      /* Send the "Page Program (GD25_PP)" Command */

      SPI_SEND(priv->spi, GD25_PP);
      priv->prev_instr = GD25_PP;

      /* Send the address high byte first. */

      if (priv->addr_4byte)
        {
          SPI_SEND(priv->spi, (address >> 24) & 0xff);
        }

      SPI_SEND(priv->spi, (address >> 16) & 0xff);
      SPI_SEND(priv->spi, (address >> 8) & 0xff);
      SPI_SEND(priv->spi, address & 0xff);

      /* Then send the page of data */

      SPI_SNDBLOCK(priv->spi, buffer, GD25_PAGE_SIZE);

      SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);

      /* Update addresses */

      address += GD25_PAGE_SIZE;
      buffer  += GD25_PAGE_SIZE;
    }
}
#endif

/***************************************************************************
 * Name:  gd25_bytewrite
 ***************************************************************************/

#if defined(CONFIG_MTD_BYTE_WRITE) && !defined(CONFIG_GD25_READONLY)
static inline void gd25_bytewrite(FAR struct gd25_dev_s *priv,
                                  FAR const uint8_t *buffer, off_t offset,
                                  uint16_t count)
{
  finfo("offset: %08lx  count:%d\n", (long)offset, count);

  /* Wait for any preceding write to complete.  We could simplify things by
   * perform this wait at the end of each write operation (rather than at
   * the beginning of ALL operations), but have the wait first will slightly
   * improve performance.
   */

  gd25_waitwritecomplete(priv);

  /* Enable the write access to the FLASH */

  gd25_wren(priv);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send "Page Program (PP)" command */

  SPI_SEND(priv->spi, GD25_PP);
  priv->prev_instr = GD25_PP;

  /* Send the page offset high byte first. */

  if (priv->addr_4byte)
    {
      SPI_SEND(priv->spi, (offset >> 24) & 0xff);
    }

  SPI_SEND(priv->spi, (offset >> 16) & 0xff);
  SPI_SEND(priv->spi, (offset >> 8) & 0xff);
  SPI_SEND(priv->spi, offset & 0xff);

  /* Then write the specified number of bytes */

  SPI_SNDBLOCK(priv->spi, buffer, count);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
  finfo("Written\n");
}
#endif /* defined(CONFIG_MTD_BYTE_WRITE) && !defined(CONFIG_GD25_READONLY) */

/***************************************************************************
 * Name: gd25_erase
 ***************************************************************************/

static int gd25_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                      size_t nblocks)
{
#ifdef CONFIG_GD25_READONLY
  return -EACESS
#else
  FAR struct gd25_dev_s *priv = (FAR struct gd25_dev_s *)dev;
  size_t blocksleft = nblocks;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  gd25_lock(priv->spi);
  gd25_purdid(priv);

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

      gd25_sectorerase(priv, startblock);
      startblock++;
    }

  gd25_pd(priv);
  gd25_unlock(priv->spi);
  return (int)nblocks;
#endif
}

/***************************************************************************
 * Name: gd25_bread
 ***************************************************************************/

static ssize_t gd25_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buffer)
{
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  nbytes = gd25_read(dev, startblock << GD25_PAGE_SHIFT,
                     nblocks << GD25_PAGE_SHIFT, buffer);
  if (nbytes > 0)
    {
      nbytes >>= GD25_PAGE_SHIFT;
    }

  return nbytes;
}

/***************************************************************************
 * Name: gd25_bwrite
 ***************************************************************************/

static ssize_t gd25_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buffer)
{
#ifdef CONFIG_GD25_READONLY
  return -EACCESS;
#else
  FAR struct gd25_dev_s *priv = (FAR struct gd25_dev_s *)dev;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the SPI bus and write all of the pages to FLASH */

  gd25_lock(priv->spi);
  gd25_purdid(priv);
  gd25_pagewrite(priv, buffer, startblock << GD25_PAGE_SHIFT,
                 nblocks << GD25_PAGE_SHIFT);
  gd25_pd(priv);
  gd25_unlock(priv->spi);

  return nblocks;
#endif
}

/***************************************************************************
 * Name: gd25_read
 ***************************************************************************/

static ssize_t gd25_read(FAR struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, FAR uint8_t *buffer)
{
  FAR struct gd25_dev_s *priv = (FAR struct gd25_dev_s *)dev;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the SPI bus and select this FLASH part */

  gd25_lock(priv->spi);
  gd25_purdid(priv);
  gd25_byteread(priv, buffer, offset, nbytes);
  gd25_pd(priv);
  gd25_unlock(priv->spi);

  finfo("return nbytes: %d,%x,%x\n", (int)nbytes, buffer[0], buffer[1]);
  return nbytes;
}

/***************************************************************************
 * Name: gd25_write
 ***************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t gd25_write(FAR struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, FAR const uint8_t *buffer)
{
#ifdef CONFIG_GD25_READONLY
  return -EACCESS;
#else
  FAR struct gd25_dev_s *priv = (FAR struct gd25_dev_s *)dev;
  int    startpage;
  int    endpage;
  int    count;
  int    index;
  int    bytestowrite;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* We must test if the offset + count crosses one or more pages
   * and perform individual writes.  The devices can only write in
   * page increments.
   */

  startpage = offset / GD25_PAGE_SIZE;
  endpage = (offset + nbytes) / GD25_PAGE_SIZE;

  gd25_lock(priv->spi);
  gd25_purdid(priv);
  if (startpage == endpage)
    {
      /* All bytes within one programmable page.  Just do the write. */

      gd25_bytewrite(priv, buffer, offset, nbytes);
    }
  else
    {
      /* Write the 1st partial-page */

      count = nbytes;
      bytestowrite = GD25_PAGE_SIZE - (offset & (GD25_PAGE_SIZE - 1));
      gd25_bytewrite(priv, buffer, offset, bytestowrite);

      /* Update offset and count */

      offset += bytestowrite;
      count -=  bytestowrite;
      index = bytestowrite;

      /* Write full pages */

      while (count >= GD25_PAGE_SIZE)
        {
          gd25_bytewrite(priv, &buffer[index], offset, GD25_PAGE_SIZE);

          /* Update offset and count */

          offset += GD25_PAGE_SIZE;
          count -= GD25_PAGE_SIZE;
          index += GD25_PAGE_SIZE;
        }

      /* Now write any partial page at the end */

      if (count > 0)
        {
          gd25_bytewrite(priv, &buffer[index], offset, count);
        }
    }

  gd25_pd(priv);
  gd25_unlock(priv->spi);
  return nbytes;
#endif
}
#endif

/***************************************************************************
 * Name: gd25_ioctl
 ***************************************************************************/

static int gd25_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct gd25_dev_s *priv = (FAR struct gd25_dev_s *)dev;
  int ret = -EINVAL;

  finfo("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              geo->blocksize    = GD25_PAGE_SIZE;
              geo->erasesize    = GD25_SECTOR_SIZE;
              geo->neraseblocks = priv->nsectors;
              ret               = OK;

              finfo("blocksize: %" PRIu32 " erasesize: %" PRIu32
                    " neraseblocks: %" PRIu32 "\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          gd25_lock(priv->spi);
          gd25_purdid(priv);
          ret = gd25_chiperase(priv);
          gd25_pd(priv);
          gd25_unlock(priv->spi);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result = GD25_ERASED_STATE;

          ret = OK;
        }
        break;

      case MTDIOC_XIPBASE:
      default:
        ret = -ENOTTY;
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: gd25_initialize
 *
 * Description:
 *   Create an initialize MTD device instance. MTD devices aren't registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ***************************************************************************/

FAR struct mtd_dev_s *gd25_initialize(FAR struct spi_dev_s *spi,
                                      uint32_t spi_devid)
{
  FAR struct gd25_dev_s *priv;
  int ret;

  priv = (FAR struct gd25_dev_s *)kmm_zalloc(sizeof(struct gd25_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = gd25_erase;
      priv->mtd.bread  = gd25_bread;
      priv->mtd.bwrite = gd25_bwrite;
      priv->mtd.read   = gd25_read;
      priv->mtd.ioctl  = gd25_ioctl;
#ifdef CONFIG_MTD_BYTE_WRITE
      priv->mtd.write  = gd25_write;
#endif
      priv->mtd.name   = "gd25";
      priv->spi        = spi;
      priv->spi_devid  = spi_devid;

      /* Deselect the FLASH */

      SPI_SELECT(spi, SPIDEV_FLASH(priv->spi_devid), false);

      /* Identify the FLASH chip and get its capacity */

      ret = gd25_readid(priv);
      if (ret != OK)
        {
          ferr("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }
      else
        {
          /* Make sure that the FLASH is unprotected
           * so that we can write into it
           */

#ifndef CONFIG_GD25_READONLY
          gd25_unprotect(priv);
#endif
        }
    }

  /* Return the implementation-specific state structure as the MTD device */

  return (FAR struct mtd_dev_s *)priv;
}
