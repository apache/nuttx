/***************************************************************************
 * drivers/mtd/gd25.c
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
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#ifdef CONFIG_GD25_QSPI
#  include <nuttx/spi/qspi.h>
#else
#  include <nuttx/spi/spi.h>
#endif
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

#ifndef CONFIG_GD25_QSPIMODE
#  define CONFIG_GD25_QSPIMODE 0
#endif

#ifndef CONFIG_GD25_QSPIFREQUENCY
#  define CONFIG_GD25_QSPIFREQUENCY 20000000
#endif

/***************************************************************************
 * GD25 Instructions
 ***************************************************************************/

/*      Command                    Value      Description                 */

#define GD25_WREN                   0x06    /* Write enable               */
#define GD25_WRDI                   0x04    /* Write Disable              */
#define GD25_RDSR                   0x05    /* Read status register       */
#define GD25_RDSR1                  0x35    /* Read status register-2     */
#define GD25_WRSR                   0x01    /* Write Status Register      */
#define GD25_RDDATA                 0x03    /* Read data bytes            */
#define GD25_FRD                    0x0b    /* Fast read (1-wire)         */
#define GD25_FRDD                   0x3b    /* Fast read, dual output     */
#define GD25_QOFRD                  0x6b    /* Quad output fast read      */
#define GD25_QIOFRD                 0xeb    /* Quad I/O fast read, 3-byte */
#define GD25_QIOFRD4B               0xec    /* Quad I/O fast read, 4-byte */
#define GD25_PP                     0x02    /* Program page (1-1-1), 3-byte */
#define GD25_PP4B                   0x12    /* Program page (1-1-1), 4-byte */
#define GD25_QPP                    0x32    /* Quad page program (1-1-4), 3-byte */
#define GD25_QPP4B                  0x34    /* Quad page program (1-1-4), 4-byte */
#define GD25_SE                     0x20    /* Sector erase (4KB), 3-byte */
#define GD25_SE4B                   0x21    /* Sector erase (4KB), 4-byte */
#define GD25_BE                     0xd8    /* Block Erase (64KB), 3-byte */
#define GD25_BE4B                   0xdc    /* Block Erase (64KB), 4-byte */
#define GD25_CE                     0xc7    /* Chip erase                 */
#define GD25_PD                     0xb9    /* Power down                 */
#define GD25_PURDID                 0xab    /* Release PD, Device ID      */
#define GD25_RDMFID                 0x90    /* Read Manufacturer / Device */
#define GD25_JEDEC_ID               0x9f    /* JEDEC ID read              */
#define GD25_4BEN                   0xb7    /* Enable 4-byte Mode         */
#define GD25_4BEXT                  0xe9    /* Exit 4-byte Mode           */

/* Dummy clock cycles for Quad I/O Fast Read (EBh/ECh).
 * Mode byte (2 clocks on 4-wire) + 4 additional = 6 total, matching
 * the MX25RXX driver's default and safe up to ~80MHz at VCC=3.3V.
 */

#define GD25_QIOFRD_DUMMIES         6

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
#define GD25Q_SR1_EN4B              (1 << 0)  /* Bit 0: Enable 4byte address GD25Q memories */

/* Status Register 2 (SR2) bit definitions */

#define GD25_SR2_QE                 (1 << 1)  /* Bit 1 of SR2 (S9): Quad Enable, non-volatile */
#define GD25_SR2_PRESERVE_MASK      0x78      /* Preserve SRP1 (bit6) and LB3-LB1 (bits5-3) */

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
  struct mtd_dev_s       mtd;         /* MTD interface */
#ifdef CONFIG_GD25_QSPI
  FAR struct qspi_dev_s *qspi;        /* Saved QSPI interface instance */
  FAR uint8_t           *cmdbuf;      /* DMA-safe command buffer */
#else
  FAR struct spi_dev_s  *spi;         /* Saved SPI interface instance */
  uint32_t               spi_devid;   /* Chip select inputs */
#endif
  uint16_t               nsectors;    /* Number of erase sectors */
  uint8_t                prev_instr;  /* Previous instruction given to GD25 device */
  bool                   addr_4byte;  /* True: Use Four-byte address */
  uint8_t                memory;      /* memory type read from device */
};

/***************************************************************************
 * Private Function Prototypes
 ***************************************************************************/

/* Helpers */

#ifdef CONFIG_GD25_QSPI
static void gd25_lock(FAR struct qspi_dev_s *qspi);
static inline void gd25_unlock(FAR struct qspi_dev_s *qspi);
#else
static void gd25_lock(FAR struct spi_dev_s *spi);
static inline void gd25_unlock(FAR struct spi_dev_s *spi);
#endif
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
static inline bool gd25_4ben(FAR struct gd25_dev_s *priv);

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
 * Name: gd25_lock
 ***************************************************************************/

#ifdef CONFIG_GD25_QSPI
static void gd25_lock(FAR struct qspi_dev_s *qspi)
{
  QSPI_LOCK(qspi, true);
  QSPI_SETMODE(qspi, CONFIG_GD25_QSPIMODE);
  QSPI_SETBITS(qspi, 8);
  QSPI_SETFREQUENCY(qspi, CONFIG_GD25_QSPIFREQUENCY);
}

static inline void gd25_unlock(FAR struct qspi_dev_s *qspi)
{
  QSPI_LOCK(qspi, false);
}
#else
static void gd25_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, true);

  SPI_SETMODE(spi, CONFIG_GD25_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_GD25_SPIFREQUENCY);
#ifdef CONFIG_SPI_DELAY_CONTROL
  SPI_SETDELAY(spi, CONFIG_GD25_START_DELAY, CONFIG_GD25_STOP_DELAY,
                    CONFIG_GD25_CS_DELAY, CONFIG_GD25_IFDELAY);
#endif
}

static inline void gd25_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, false);
}
#endif /* CONFIG_GD25_QSPI */

/***************************************************************************
 * Name: gd25_readid
 ***************************************************************************/

static inline int gd25_readid(FAR struct gd25_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t memory;
  uint16_t capacity;
  int ret = -ENODEV;

#ifdef CONFIG_GD25_QSPI
  struct qspi_cmdinfo_s cmdinfo;

  gd25_lock(priv->qspi);

  cmdinfo.flags   = QSPICMD_READDATA;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = GD25_JEDEC_ID;
  cmdinfo.buflen  = 3;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = priv->cmdbuf;
  QSPI_COMMAND(priv->qspi, &cmdinfo);

  manufacturer = priv->cmdbuf[0];
  memory       = priv->cmdbuf[1];
  capacity     = priv->cmdbuf[2];
#else
  /* Lock and configure the SPI bus */

  gd25_lock(priv->spi);

  /* Select this FLASH part. */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the "Read ID (RDID)" command and read the first three ID bytes */

  SPI_SEND(priv->spi, GD25_JEDEC_ID);
  manufacturer = SPI_SEND(priv->spi, GD25_DUMMY);
  memory       = SPI_SEND(priv->spi, GD25_DUMMY);
  capacity     = SPI_SEND(priv->spi, GD25_DUMMY);

  /* Deselect the FLASH and unlock the bus */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
#endif /* CONFIG_GD25_QSPI */

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

      priv->memory = memory;

      /* Capacity greater than 16MB, Enable four-byte address */

      if (priv->nsectors > GD25_NSECTORS_128MBIT)
        {
          if (!gd25_4ben(priv))
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

#ifdef CONFIG_GD25_QSPI
  gd25_unlock(priv->qspi);
#else
  gd25_unlock(priv->spi);
#endif
  return ret;
}

/***************************************************************************
 * Name: gd25_unprotect
 ***************************************************************************/

#ifndef CONFIG_GD25_READONLY
static void gd25_unprotect(FAR struct gd25_dev_s *priv)
{
#ifdef CONFIG_GD25_QSPI
  struct qspi_cmdinfo_s cmdinfo;
  uint8_t sr2;

  gd25_lock(priv->qspi);
  gd25_waitwritecomplete(priv);

  /* Read SR2 to preserve OTP/SRP bits before writing */

  sr2 = gd25_rdsr(priv, 1);

  gd25_wren(priv);

  /* SR1 = 0 (clear all block-protect bits); SR2 = preserve non-OTP bits,
   * set QE (S9 = bit1 of SR2) so quad I/O pins are active.
   */

  priv->cmdbuf[0] = 0;
  priv->cmdbuf[1] = (sr2 & GD25_SR2_PRESERVE_MASK) | GD25_SR2_QE;
  cmdinfo.flags   = QSPICMD_WRITEDATA;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = GD25_WRSR;
  cmdinfo.buflen  = 2;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = priv->cmdbuf;
  QSPI_COMMAND(priv->qspi, &cmdinfo);

  gd25_waitwritecomplete(priv);
  gd25_unlock(priv->qspi);
#else
  /* Lock and configure the SPI bus */

  gd25_lock(priv->spi);

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

  gd25_unlock(priv->spi);
#endif /* CONFIG_GD25_QSPI */
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
      if (priv->prev_instr != GD25_PP && priv->prev_instr != GD25_PP4B &&
          priv->prev_instr != GD25_QPP && priv->prev_instr != GD25_QPP4B &&
          (status & GD25_SR_WIP) != 0)
        {
#ifdef CONFIG_GD25_QSPI
          gd25_unlock(priv->qspi);
          nxsig_usleep(1000);
          gd25_lock(priv->qspi);
#else
          gd25_unlock(priv->spi);
          nxsig_usleep(1000);
          gd25_lock(priv->spi);
#endif
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
  uint8_t rdsr[2] =
  {
    GD25_RDSR, GD25_RDSR1
  };

#ifdef CONFIG_GD25_QSPI
  struct qspi_cmdinfo_s cmdinfo;
  cmdinfo.flags   = QSPICMD_READDATA;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = rdsr[id];
  cmdinfo.buflen  = 1;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = priv->cmdbuf;
  QSPI_COMMAND(priv->qspi, &cmdinfo);
  return priv->cmdbuf[0];
#else
  uint8_t status;
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, rdsr[id]);
  status = SPI_SEND(priv->spi, GD25_DUMMY);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
  return status;
#endif
}

/***************************************************************************
 * Name:  gd25_4ben
 *
 * Enable 4 byte memory addressing mode
 * Return success or not
 *
 ***************************************************************************/

static inline bool gd25_4ben(FAR struct gd25_dev_s *priv)
{
#ifdef CONFIG_GD25_QSPI
  struct qspi_cmdinfo_s cmdinfo;
  cmdinfo.flags   = 0;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = GD25_4BEN;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = NULL;
  QSPI_COMMAND(priv->qspi, &cmdinfo);
#else
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, GD25_4BEN);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
#endif
  if (priv->memory == GD25Q_JEDEC_MEMORY_TYPE)
    {
      return ((gd25_rdsr(priv, 1) & GD25Q_SR1_EN4B) == GD25Q_SR1_EN4B);
    }
  else
    {
      return ((gd25_rdsr(priv, 1) & GD25_SR1_EN4B) == GD25_SR1_EN4B);
    }
}

/***************************************************************************
 * Name:  gd25_wren
 ***************************************************************************/

static inline void gd25_wren(FAR struct gd25_dev_s *priv)
{
#ifdef CONFIG_GD25_QSPI
  struct qspi_cmdinfo_s cmdinfo;
  cmdinfo.flags   = 0;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = GD25_WREN;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = NULL;
  QSPI_COMMAND(priv->qspi, &cmdinfo);
#else
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, GD25_WREN);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
#endif
}

/***************************************************************************
 * Name:  gd25_wrdi
 ***************************************************************************/

static inline void gd25_wrdi(FAR struct gd25_dev_s *priv)
{
#ifdef CONFIG_GD25_QSPI
  struct qspi_cmdinfo_s cmdinfo;
  cmdinfo.flags   = 0;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = GD25_WRDI;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = NULL;
  QSPI_COMMAND(priv->qspi, &cmdinfo);
#else
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->spi, GD25_WRDI);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
#endif
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

      gd25_byteread(priv, (FAR uint8_t *)buf, address, GD25_PAGE_SIZE);

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
#ifdef CONFIG_GD25_QSPI
  struct qspi_cmdinfo_s cmdinfo;
#endif

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

#ifdef CONFIG_GD25_QSPI
  cmdinfo.flags   = QSPICMD_ADDRESS;
  cmdinfo.addrlen = priv->addr_4byte ? 4 : 3;
  cmdinfo.cmd     = priv->addr_4byte ? GD25_SE4B : GD25_SE;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = address;
  cmdinfo.buffer  = NULL;
  QSPI_COMMAND(priv->qspi, &cmdinfo);
  priv->prev_instr = cmdinfo.cmd;
#else
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
#endif /* CONFIG_GD25_QSPI */
}

/***************************************************************************
 * Name:  gd25_chiperase
 ***************************************************************************/

static inline int gd25_chiperase(FAR struct gd25_dev_s *priv)
{
#ifdef CONFIG_GD25_QSPI
  struct qspi_cmdinfo_s cmdinfo;
#endif

  /* Wait for any preceding write or erase operation to complete. */

  gd25_waitwritecomplete(priv);

  /* Send write enable instruction */

  gd25_wren(priv);

#ifdef CONFIG_GD25_QSPI
  cmdinfo.flags   = 0;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = GD25_CE;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = NULL;
  QSPI_COMMAND(priv->qspi, &cmdinfo);
  priv->prev_instr = GD25_CE;
#else
  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the "Chip Erase (CE)" instruction */

  SPI_SEND(priv->spi, GD25_CE);
  priv->prev_instr = GD25_CE;

  SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->spi_devid), false);
#endif /* CONFIG_GD25_QSPI */
  return OK;
}

/***************************************************************************
 * Name: gd25_byteread
 ***************************************************************************/

static void gd25_byteread(FAR struct gd25_dev_s *priv, FAR uint8_t *buffer,
                          off_t address, size_t nbytes)
{
#ifdef CONFIG_GD25_QSPI
  struct qspi_meminfo_s meminfo;
#endif

  finfo("address: %08lx nbytes: %d\n", (long)address, (int)nbytes);

  /* Wait for any preceding write or erase operation to complete. */

  gd25_waitwritecomplete(priv);

  /* Make sure that writing is disabled */

  gd25_wrdi(priv);

#ifdef CONFIG_GD25_QSPI
  meminfo.flags   = QSPIMEM_READ | QSPIMEM_QUADIO;
  meminfo.addrlen = priv->addr_4byte ? 4 : 3;
  meminfo.dummies = GD25_QIOFRD_DUMMIES;
  meminfo.buflen  = nbytes;
  meminfo.cmd     = priv->addr_4byte ? GD25_QIOFRD4B : GD25_QIOFRD;
  meminfo.addr    = address;
  meminfo.buffer  = buffer;
  QSPI_MEMORY(priv->qspi, &meminfo);
  priv->prev_instr = meminfo.cmd;
#else
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
#endif /* CONFIG_GD25_QSPI */
}

/***************************************************************************
 * Name:  gd25_pagewrite
 ***************************************************************************/

#ifndef CONFIG_GD25_READONLY
static void gd25_pagewrite(FAR struct gd25_dev_s *priv,
                           FAR const uint8_t *buffer, off_t address,
                           size_t nbytes)
{
#ifdef CONFIG_GD25_QSPI
  /* GD25 Quad Page Program (0x32/0x34) is a 1-1-4 transfer: the address
   * is clocked on a single line and only the data is quad.
   * QSPIMEM_QUADIO cannot express this (it forces the address quad too),
   * so use QSPIMEM_QUADDATA, which sets the data phase quad while leaving
   * the address phase single-line.
   */

  struct qspi_meminfo_s meminfo;
#endif

  finfo("address: %08lx nwords: %d\n", (long)address, (int)nbytes);
  DEBUGASSERT(priv && buffer && (address & 0xff) == 0 &&
              (nbytes & 0xff) == 0);

  for (; nbytes > 0; nbytes -= GD25_PAGE_SIZE)
    {
      /* Wait for any preceding write or erase operation to complete. */

      gd25_waitwritecomplete(priv);

      /* Enable write access to the FLASH */

      gd25_wren(priv);

#ifdef CONFIG_GD25_QSPI
      meminfo.flags   = QSPIMEM_WRITE | QSPIMEM_QUADDATA;
      meminfo.cmd     = priv->addr_4byte ? GD25_QPP4B : GD25_QPP;
      meminfo.addrlen = priv->addr_4byte ? 4 : 3;
      meminfo.buflen  = GD25_PAGE_SIZE;
      meminfo.dummies = 0;
      meminfo.addr    = address;
      meminfo.buffer  = (FAR void *)buffer;
      QSPI_MEMORY(priv->qspi, &meminfo);
      priv->prev_instr = meminfo.cmd;
#else
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
#endif /* CONFIG_GD25_QSPI */

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
#ifdef CONFIG_GD25_QSPI
  /* See gd25_pagewrite: GD25 Quad Page Program (0x32/0x34) is 1-1-4.
   * QSPIMEM_QUADDATA selects quad data with a single-line address.
   */

  struct qspi_meminfo_s meminfo;
#endif

  finfo("offset: %08lx  count:%d\n", (long)offset, count);

  /* Wait for any preceding write to complete.  We could simplify things by
   * perform this wait at the end of each write operation (rather than at
   * the beginning of ALL operations), but have the wait first will slightly
   * improve performance.
   */

  gd25_waitwritecomplete(priv);

  /* Enable the write access to the FLASH */

  gd25_wren(priv);

#ifdef CONFIG_GD25_QSPI
  meminfo.flags   = QSPIMEM_WRITE | QSPIMEM_QUADDATA;
  meminfo.cmd     = priv->addr_4byte ? GD25_QPP4B : GD25_QPP;
  meminfo.addrlen = priv->addr_4byte ? 4 : 3;
  meminfo.buflen  = count;
  meminfo.dummies = 0;
  meminfo.addr    = offset;
  meminfo.buffer  = (FAR void *)buffer;
  QSPI_MEMORY(priv->qspi, &meminfo);
  priv->prev_instr = meminfo.cmd;
#else
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
#endif /* CONFIG_GD25_QSPI */
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
  return -EACCES;
#else
  FAR struct gd25_dev_s *priv = (FAR struct gd25_dev_s *)dev;
  size_t blocksleft = nblocks;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the bus until we complete the erase */

#ifdef CONFIG_GD25_QSPI
  gd25_lock(priv->qspi);
#else
  gd25_lock(priv->spi);
#endif

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

      gd25_sectorerase(priv, startblock);
      startblock++;
    }

#ifdef CONFIG_GD25_QSPI
  gd25_unlock(priv->qspi);
#else
  gd25_unlock(priv->spi);
#endif
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

  /* Lock the bus and write all of the pages to FLASH */

#ifdef CONFIG_GD25_QSPI
  gd25_lock(priv->qspi);
#else
  gd25_lock(priv->spi);
#endif
  gd25_pagewrite(priv, buffer, startblock << GD25_PAGE_SHIFT,
                 nblocks << GD25_PAGE_SHIFT);
#ifdef CONFIG_GD25_QSPI
  gd25_unlock(priv->qspi);
#else
  gd25_unlock(priv->spi);
#endif

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

  /* Lock the bus and select this FLASH part */

#ifdef CONFIG_GD25_QSPI
  gd25_lock(priv->qspi);
#else
  gd25_lock(priv->spi);
#endif
  gd25_byteread(priv, buffer, offset, nbytes);
#ifdef CONFIG_GD25_QSPI
  gd25_unlock(priv->qspi);
#else
  gd25_unlock(priv->spi);
#endif

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

#ifdef CONFIG_GD25_QSPI
  gd25_lock(priv->qspi);
#else
  gd25_lock(priv->spi);
#endif
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

#ifdef CONFIG_GD25_QSPI
  gd25_unlock(priv->qspi);
#else
  gd25_unlock(priv->spi);
#endif
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

  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              memset(geo, 0, sizeof(*geo));

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

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
            (FAR struct partition_info_s *)arg;
          if (info != NULL)
            {
              info->numsectors  = priv->nsectors *
                                  GD25_SECTOR_SIZE / GD25_PAGE_SIZE;
              info->sectorsize  = GD25_PAGE_SIZE;
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

#ifdef CONFIG_GD25_QSPI
          gd25_lock(priv->qspi);
#else
          gd25_lock(priv->spi);
#endif
          ret = gd25_chiperase(priv);
#ifdef CONFIG_GD25_QSPI
          gd25_unlock(priv->qspi);
#else
          gd25_unlock(priv->spi);
#endif
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result = GD25_ERASED_STATE;

          ret = OK;
        }
        break;

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

#ifdef CONFIG_GD25_QSPI
FAR struct mtd_dev_s *gd25_initialize(FAR struct qspi_dev_s *qspi,
                                      bool unprotect)
{
  FAR struct gd25_dev_s *priv;
  int ret;

  priv = (FAR struct gd25_dev_s *)kmm_zalloc(sizeof(struct gd25_dev_s));
  if (priv)
    {
      priv->mtd.erase  = gd25_erase;
      priv->mtd.bread  = gd25_bread;
      priv->mtd.bwrite = gd25_bwrite;
      priv->mtd.read   = gd25_read;
      priv->mtd.ioctl  = gd25_ioctl;
#ifdef CONFIG_MTD_BYTE_WRITE
      priv->mtd.write  = gd25_write;
#endif
      priv->mtd.name   = "gd25";
      priv->qspi       = qspi;

      priv->cmdbuf = (FAR uint8_t *)QSPI_ALLOC(qspi, 4);
      if (!priv->cmdbuf)
        {
          kmm_free(priv);
          return NULL;
        }

      ret = gd25_readid(priv);
      if (ret != OK)
        {
          ferr("ERROR: Unrecognized\n");
          QSPI_FREE(qspi, priv->cmdbuf);
          kmm_free(priv);
          return NULL;
        }

      /* QE (Quad Enable) must be set before any quad I/O command is issued.
       * It is non-volatile, so only write if currently clear to avoid
       * unnecessary write cycles.
       */

      if (!(gd25_rdsr(priv, 1) & GD25_SR2_QE))
        {
          uint8_t sr2 = gd25_rdsr(priv, 1);

          gd25_lock(priv->qspi);
          gd25_waitwritecomplete(priv);
          gd25_wren(priv);

          priv->cmdbuf[0] = 0;
          priv->cmdbuf[1] = (sr2 & GD25_SR2_PRESERVE_MASK) | GD25_SR2_QE;

          struct qspi_cmdinfo_s cmdinfo;
          cmdinfo.flags   = QSPICMD_WRITEDATA;
          cmdinfo.addrlen = 0;
          cmdinfo.cmd     = GD25_WRSR;
          cmdinfo.buflen  = 2;
          cmdinfo.addr    = 0;
          cmdinfo.buffer  = priv->cmdbuf;
          QSPI_COMMAND(priv->qspi, &cmdinfo);

          gd25_waitwritecomplete(priv);
          gd25_unlock(priv->qspi);
        }

#ifndef CONFIG_GD25_READONLY
      if (unprotect)
        {
          gd25_unprotect(priv);
        }
#endif
    }

  return (FAR struct mtd_dev_s *)priv;
}
#else
FAR struct mtd_dev_s *gd25_initialize(FAR struct spi_dev_s *spi,
                                      uint32_t spi_devid)
{
  FAR struct gd25_dev_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(struct gd25_dev_s));
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
#endif /* CONFIG_GD25_QSPI */
