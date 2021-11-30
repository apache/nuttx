/****************************************************************************
 * drivers/mtd/ramtron.c
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

/* OPTIONS:
 *  - additional non-jedec standard device: FM25H20
 *    must be enabled with the CONFIG_RAMTRON_FRAM_NON_JEDEC=y
 *
 * NOTE:
 *  - frequency is fixed to desired max by RAMTRON_INIT_CLK_MAX if new
 *    devices with different speed arrive, use the table to handle freq
 *    change and to fit all devices. Note that STM32_SPI driver is prone
 *    to too high freq. parameters and limit it within physical constraints.
 *    The speed may be changed through ioctl MTDIOC_SETSPEED
 *
 * TODO:
 *  - add support for sleep
 *  - add support for faster read FSTRD command
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Used to abort the write wait */

#ifndef CONFIG_MTD_RAMTRON_WRITEWAIT_COUNT
#   define CONFIG_MTD_RAMTRON_WRITEWAIT_COUNT 100
#endif

/*  RAMTRON devices are flat!
 *  For purpose of the VFAT file system we emulate the following
 *  configuration:
 */

#define RAMTRON_EMULATE_SECTOR_SHIFT  9
#define RAMTRON_EMULATE_PAGE_SHIFT    9
#define RAMTRON_EMULATE_PAGE_SIZE     (1 << RAMTRON_EMULATE_PAGE_SHIFT)

/* RAMTRON Identification register values */

#define RAMTRON_MANUFACTURER         0x7f
#define RAMTRON_MEMORY_TYPE          0xc2

/* Instructions:
 *      Command          Value       N Description         Addr Dummy Data
 */

#define RAMTRON_WREN      0x06    /* 1 Write Enable          0   0     0 */
#define RAMTRON_WRDI      0x04    /* 1 Write Disable         0   0     0 */
#define RAMTRON_RDSR      0x05    /* 1 Read Status Register  0   0     >=1 */
#define RAMTRON_WRSR      0x01    /* 1 Write Status Register 0   0     1 */
#define RAMTRON_READ      0x03    /* 1 Read Data Bytes       A   0     >=1 */
#define RAMTRON_FSTRD     0x0b    /* 1 Higher speed read     A   1     >=1 */
#define RAMTRON_WRITE     0x02    /* 1 Write                 A   0     1-256 */
#define RAMTRON_SLEEP     0xb9    /* TODO: */
#define RAMTRON_RDID      0x9f    /* 1 Read Identification   0   0     1-3 */
#define RAMTRON_SN        0xc3    /* TODO: */

/* Status register bit definitions */

/*                                                            Bit 0: Res 0 */
#define RAMTRON_SR_WEL          (1 << 1)                   /* Bit 1: Write enable latch bit */
#define RAMTRON_SR_BP_SHIFT     (2)                        /* Bits 2-4: Block protect bits */
#define RAMTRON_SR_BP_MASK      (3 << RAMTRON_SR_BP_SHIFT)
#define RAMTRON_SR_BP_NONE      (0 << RAMTRON_SR_BP_SHIFT) /* Unprotected */
#define RAMTRON_SR_BP_UPPERQTR  (1 << RAMTRON_SR_BP_SHIFT) /* Upper quarter */
#define RAMTRON_SR_BP_UPPERHALF (2 << RAMTRON_SR_BP_SHIFT) /* Upper half */
#define RAMTRON_SR_BP_ALL       (3 << RAMTRON_SR_BP_SHIFT) /* All sectors */
#define RAMTRON_SR_BP_SHIFT     (2)                        /* Bits 4-6: Reserved Always 0 */
#define RAMTRON_SR_WPEN         (1 << 7)                   /* Bit 7: Status register write protect */

#define RAMTRON_DUMMY     0xa5

/* Defines the initial speed compatible with all devices. In case of RAMTRON
 * the defined devices within the part list have all the same speed.
 */

#define RAMTRON_INIT_CLK_MAX    40000000UL

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ramtron_parts_s
{
  FAR const char *name;
  uint8_t id1;
  uint8_t id2;
  uint32_t size;
  uint8_t addr_len;
  uint32_t speed;
#ifdef CONFIG_RAMTRON_CHUNKING
  bool chunked;                            /* True: write buffer size limitations */
  uint16_t chunksize;                      /* Write chunk Size */
#endif
};

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct ramtron_dev_s.
 */

struct ramtron_dev_s
{
  struct mtd_dev_s mtd;                    /* MTD interface */
  FAR struct spi_dev_s *dev;               /* Saved SPI interface instance */
  uint8_t sectorshift;
  uint8_t pageshift;
  uint16_t nsectors;
  uint32_t npages;
  uint32_t speed;                          /* Overridable via ioctl */
  FAR const struct ramtron_parts_s *part;  /* Part instance */
};

/****************************************************************************
 * Supported Part Lists
 ****************************************************************************/

static const struct ramtron_parts_s g_ramtron_parts[] =
{
  {
    "FM25V01",                    /* name */
    0x21,                         /* id1 */
    0x00,                         /* id2 */
    16L * 1024L,                  /* size */
    2,                            /* addr_len */
    RAMTRON_INIT_CLK_MAX          /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    RAMTRON_EMULATE_PAGE_SIZE     /* chunksize */
#endif
  },
  {
    "FM25V01A",                   /* name */
    0x21,                         /* id1 */
    0x08,                         /* id2 */
    16L * 1024L,                  /* size */
    2,                            /* addr_len */
    RAMTRON_INIT_CLK_MAX          /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    RAMTRON_EMULATE_PAGE_SIZE     /* chunksize */
#endif
  },
  {
    "FM25V02",                    /* name */
    0x22,                         /* id1 */
    0x00,                         /* id2 */
    32L * 1024L,                  /* size */
    2,                            /* addr_len */
    RAMTRON_INIT_CLK_MAX          /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    RAMTRON_EMULATE_PAGE_SIZE     /* chunksize */
#endif
  },
  {
    "FM25V02A",                   /* name */
    0x22,                         /* id1 */
    0x08,                         /* id2 */
    32L * 1024L,                  /* size */
    2,                            /* addr_len */
    RAMTRON_INIT_CLK_MAX          /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    RAMTRON_EMULATE_PAGE_SIZE     /* chunksize */
#endif
  },
  {
    "FM25VN02",                   /* name */
    0x22,                         /* id1 */
    0x01,                         /* id2 */
    32L * 1024L,                  /* size */
    2,                            /* addr_len */
    RAMTRON_INIT_CLK_MAX          /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    RAMTRON_EMULATE_PAGE_SIZE     /* chunksize */
#endif
  },
  {
    "FM25V05",                    /* name */
    0x23,                         /* id1 */
    0x00,                         /* id2 */
    64L * 1024L,                  /* size */
    2,                            /* addr_len */
    RAMTRON_INIT_CLK_MAX          /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    RAMTRON_EMULATE_PAGE_SIZE     /* chunksize */
#endif
  },
  {
    "FM25VN05",                   /* name */
    0x23,                         /* id1 */
    0x01,                         /* id2 */
    64L * 1024L,                  /* size */
    2,                            /* addr_len */
    RAMTRON_INIT_CLK_MAX          /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    RAMTRON_EMULATE_PAGE_SIZE     /* chunksize */
#endif
  },
  {
    "FM25V10",                    /* name */
    0x24,                         /* id1 */
    0x00,                         /* id2 */
    128L * 1024L,                 /* size */
    3,                            /* addr_len */
    RAMTRON_INIT_CLK_MAX          /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    RAMTRON_EMULATE_PAGE_SIZE     /* chunksize */
#endif
  },
  {
    "FM25VN10",                   /* name */
    0x24,                         /* id1 */
    0x01,                         /* id2 */
    128L * 1024L,                 /* size */
    3,                            /* addr_len */
    RAMTRON_INIT_CLK_MAX          /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    RAMTRON_EMULATE_PAGE_SIZE     /* chunksize */
#endif
  },
  {
    "FM25V20A",                   /* name */
    0x25,                         /* id1 */
    0x08,                         /* id2 */
    256L * 1024L,                 /* size */
    3,                            /* addr_len */
    RAMTRON_INIT_CLK_MAX          /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    RAMTRON_EMULATE_PAGE_SIZE     /* chunksize */
#endif
  },
  {
    "CY15B104Q",                  /* name */
    0x26,                         /* id1 */
    0x08,                         /* id2 */
    512L * 1024L,                 /* size */
    3,                            /* addr_len */
    RAMTRON_INIT_CLK_MAX          /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    RAMTRON_EMULATE_PAGE_SIZE     /* chunksize */
#endif
  },
  {
    "MB85RS1MT",                  /* name */
    0x27,                         /* id1 */
    0x03,                         /* id2 */
    128L * 1024L,                 /* size */
    3,                            /* addr_len */
    25000000                      /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    RAMTRON_EMULATE_PAGE_SIZE     /* chunksize */
#endif
  },
  {
    "MB85RS256B",                 /* name */
    0x05,                         /* id1 */
    0x09,                         /* id2 */
    32L * 1024L,                  /* size */
    3,                            /* addr_len */
    25000000                      /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    RAMTRON_EMULATE_PAGE_SIZE     /* chunksize */
#endif
  },
#ifdef CONFIG_RAMTRON_CHUNKING
  {
    "MB85AS4MT",                  /* name */
    0xc9,                         /* id1 */
    0x03,                         /* id2 */
    512L * 1024L,                 /* size */
    3,                            /* addr_len */
    RAMTRON_INIT_CLK_MAX,         /* speed */
    true,                         /* chunked */
    256                           /* chunksize */
  },
#endif
#ifdef CONFIG_RAMTRON_FRAM_NON_JEDEC
  {
    "FM25H20",                    /* name */
    0xff,                         /* id1 */
    0xff,                         /* id2 */
    256L * 1024L,                 /* size */
    3,                            /* addr_len */
    RAMTRON_INIT_CLK_MAX          /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    RAMTRON_EMULATE_PAGE_SIZE     /* chunksize */
#endif
  },
#endif
  {
    NULL,                         /* name */
    0,                            /* id1 */
    0,                            /* id2 */
    0,                            /* size */
    0,                            /* addr_len */
    0                             /* speed */
#ifdef CONFIG_RAMTRON_CHUNKING
    , false,                      /* chunked */
    0,                            /* chunksize */
#endif
  }
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static void ramtron_lock(FAR struct ramtron_dev_s *priv);
static inline void ramtron_unlock(FAR struct spi_dev_s *dev);
static inline int ramtron_readid(struct ramtron_dev_s *priv);
static void ramtron_writeenable(struct ramtron_dev_s *priv);
static inline int ramtron_pagewrite(struct ramtron_dev_s *priv,
                                    FAR const uint8_t *buffer,
                                    off_t offset,
                                    size_t pagesize);

/* MTD driver methods */

static int ramtron_erase(FAR struct mtd_dev_s *dev,
                         off_t startblock,
                         size_t nblocks);
static ssize_t ramtron_bread(FAR struct mtd_dev_s *dev,
                             off_t startblock,
                             size_t nblocks, FAR uint8_t *buf);
#ifdef CONFIG_RAMTRON_CHUNKING
static ssize_t ramtron_bwrite_nonchunked(FAR struct mtd_dev_s *dev,
                                         off_t startblock,
                                         size_t nblocks,
                                         FAR const uint8_t *buffer);
static ssize_t ramtron_bwrite_chunked(FAR struct mtd_dev_s *dev,
                                      off_t startblock,
                                      size_t nblocks,
                                      FAR const uint8_t *buf);
#endif
static ssize_t ramtron_bwrite(FAR struct mtd_dev_s *dev,
                              off_t startblock,
                              size_t nblocks,
                              FAR const uint8_t *buf);
static ssize_t ramtron_read(FAR struct mtd_dev_s *dev,
                            off_t offset,
                            size_t nbytes,
                            FAR uint8_t *buffer);
static int ramtron_ioctl(FAR struct mtd_dev_s *dev,
                         int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ramtron_lock
 ****************************************************************************/

static void ramtron_lock(FAR struct ramtron_dev_s *priv)
{
  FAR struct spi_dev_s *dev = priv->dev;

  /* On SPI buses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the buses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusive
   * access to the SPI bus.
   * We will retain that exclusive access until the bus is unlocked.
   */

  SPI_LOCK(dev, true);

  /* After locking the SPI bus, the we also need call the setfrequency,
   * setbits, and setmode methods to make sure that the SPI is properly
   * configured for the device.
   * If the SPI bus is being shared, then it may have been left in an
   * incompatible state.
   */

  SPI_SETMODE(dev, SPIDEV_MODE3);
  SPI_SETBITS(dev, 8);
  SPI_HWFEATURES(dev, 0);
  SPI_SETFREQUENCY(dev, priv->speed);
}

/****************************************************************************
 * Name: ramtron_unlock
 ****************************************************************************/

static inline void ramtron_unlock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, false);
}

/****************************************************************************
 * Name: ramtron_readid
 ****************************************************************************/

static inline int ramtron_readid(struct ramtron_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t memory;
  uint16_t capacity;
  uint16_t part;
  int i;

  finfo("priv: %p\n", priv);

  /* Lock the SPI bus, configure the bus, and select this FLASH part. */

  ramtron_lock(priv);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send the "Read ID (RDID)" command */

  SPI_SEND(priv->dev, RAMTRON_RDID);

  /* Read the first six manufacturer ID bytes. */

  for (i = 0; i < 6; i++)
    {
      /* Read the next manufacturer byte */

      manufacturer = SPI_SEND(priv->dev, RAMTRON_DUMMY);

      /* Fujitsu parts such as MB85RS1MT only have 1-byte for the
       * manufacturer ID.  The manufacturer code is "0x4".
       */

      if (i == 0 && manufacturer == 0x04)
        {
          break;
        }
    }

  memory   = SPI_SEND(priv->dev, RAMTRON_DUMMY);
  capacity = SPI_SEND(priv->dev, RAMTRON_DUMMY);  /* fram.id1 */
  part     = SPI_SEND(priv->dev, RAMTRON_DUMMY);  /* fram.id2 */

  /* Deselect the FLASH and unlock the bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
  ramtron_unlock(priv->dev);

  /* Select part from the part list */

  for (priv->part = g_ramtron_parts;
       priv->part->name != NULL &&
         !(priv->part->id1 == capacity && priv->part->id2 == part);
       priv->part++);

  if (priv->part->name != NULL)
    {
      UNUSED(manufacturer); /* Eliminate warnings when debug is off */
      UNUSED(memory);       /* Eliminate warnings when debug is off */

      finfo("RAMTRON %s of size %" PRIu32 " bytes (mf:%02" PRIx16 " mem:%02"
            PRIx16 " cap:%02" PRIx16 " part:%02" PRIx16 ")\n",
            priv->part->name, priv->part->size, manufacturer, memory,
            capacity, part);

      priv->sectorshift = RAMTRON_EMULATE_SECTOR_SHIFT;
      priv->nsectors    = priv->part->size /
                          (1 << RAMTRON_EMULATE_SECTOR_SHIFT);
      priv->pageshift   = RAMTRON_EMULATE_PAGE_SHIFT;
      priv->npages      = priv->part->size /
                          (1 << RAMTRON_EMULATE_PAGE_SHIFT);
      priv->speed       = priv->part->speed;
      return OK;
    }

  finfo("RAMTRON device not found\n");
  return -ENODEV;
}

/****************************************************************************
 * Name:  ramtron_writeenable
 ****************************************************************************/

static void ramtron_writeenable(struct ramtron_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send "Write Enable (WREN)" command */

  SPI_SEND(priv->dev, RAMTRON_WREN);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
  finfo("Enabled\n");
}

/****************************************************************************
 * Name:  ramtron_sendaddr
 ****************************************************************************/

static inline void ramtron_sendaddr(const struct ramtron_dev_s *priv,
                                    uint32_t addr)
{
  DEBUGASSERT(priv->part->addr_len == 3 || priv->part->addr_len == 2);

  if (priv->part->addr_len == 3)
    {
      SPI_SEND(priv->dev, (addr >> 16) & 0xff);
    }

  SPI_SEND(priv->dev, (addr >> 8) & 0xff);
  SPI_SEND(priv->dev, addr & 0xff);
}

/****************************************************************************
 * Name:  ramtron_pagewrite
 ****************************************************************************/

static inline int ramtron_pagewrite(struct ramtron_dev_s *priv,
                                    FAR const uint8_t *buffer, off_t page,
                                    size_t pagesize)
{
  off_t offset = page * pagesize;

  finfo("page: %08lx offset: %08lx\n", (long)page, (long)offset);

  /* Enable the write access to the FLASH */

  ramtron_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send "Page Program (PP)" command */

  SPI_SEND(priv->dev, RAMTRON_WRITE);

  /* Send the page offset high byte first. */

  ramtron_sendaddr(priv, offset);

  /* Then write the specified number of bytes */

  SPI_SNDBLOCK(priv->dev, buffer, pagesize);

  /* Deselect the FLASH: Chip Select high */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
  finfo("Written\n");

  return OK;
}

/****************************************************************************
 * Name: ramtron_erase
 ****************************************************************************/

static int ramtron_erase(FAR struct mtd_dev_s *dev,
                         off_t startblock,
                         size_t nblocks)
{
  finfo("startblock: %08lx nblocks: %d\n",
        (unsigned long)startblock,
        (int)nblocks);
  finfo("On RAMTRON devices erasing makes no sense, returning as OK\n");
  return (int)nblocks;
}

/****************************************************************************
 * Name: ramtron_bread
 ****************************************************************************/

static ssize_t ramtron_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR uint8_t *buffer)
{
  FAR struct ramtron_dev_s *priv = (FAR struct ramtron_dev_s *)dev;
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented
   * read
   */

  nbytes = ramtron_read(dev, startblock << priv->pageshift,
                        nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
      return nbytes >> priv->pageshift;
    }

  return (int)nbytes;
}

/****************************************************************************
 * Name: ramtron_bwrite/ramtron_bwrite_nonchunked
 ****************************************************************************/

#ifdef CONFIG_RAMTRON_CHUNKING
static ssize_t ramtron_bwrite_nonchunked(FAR struct mtd_dev_s *dev,
                                         off_t startblock,
                                         size_t nblocks,
                                         FAR const uint8_t *buffer)
#else
static ssize_t ramtron_bwrite(FAR struct mtd_dev_s *dev,
                              off_t startblock,
                              size_t nblocks,
                              FAR const uint8_t *buffer)
#endif
{
  FAR struct ramtron_dev_s *priv = (FAR struct ramtron_dev_s *)dev;
  size_t blocksleft = nblocks;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);
  DEBUGASSERT(priv != NULL && buffer != NULL);

  /* Lock the SPI bus and write each page to FLASH */

  ramtron_lock(priv);
  while (blocksleft-- > 0)
    {
      if (ramtron_pagewrite(priv, buffer, startblock, 1 << priv->pageshift))
        {
          nblocks = 0;
          break;
        }

      startblock++;
    }

  ramtron_unlock(priv->dev);
  return nblocks;
}

/****************************************************************************
 * Name: ramtron_bwrite_chunked
 ****************************************************************************/

#ifdef CONFIG_RAMTRON_CHUNKING
static ssize_t ramtron_bwrite_chunked(FAR struct mtd_dev_s *dev,
                                      off_t startblock,
                                      size_t nblocks,
                                      FAR const uint8_t *buffer)
{
  FAR struct ramtron_dev_s *priv = (FAR struct ramtron_dev_s *)dev;
  FAR const struct ramtron_parts_s *part;
  size_t blocksleft = nblocks;
  uint32_t p;
  uint32_t writesplits;
  off_t newstartblock;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  DEBUGASSERT(priv != NULL && priv->part != NULL && buffer != NULL);
  part = priv->part;

  writesplits   = (1 << priv->pageshift) / part->chunksize;
  newstartblock = startblock * writesplits;

  /* Lock the SPI bus and write each page to FLASH */

  ramtron_lock(priv);
  while (blocksleft-- > 0)
    {
      /* Split writes in chunksize chunks */

      for (p = 0; p < writesplits; p++)
        {
          if (ramtron_pagewrite(priv,
                                buffer + p * part->chunksize,
                                newstartblock,
                                part->chunksize))
            {
              nblocks = 0;
              goto out;
            }

          newstartblock++;
        }
    }

out:
  ramtron_unlock(priv->dev);
  return nblocks;
}
#endif

/****************************************************************************
 * Name: ramtron_bwrite
 ****************************************************************************/

#ifdef CONFIG_RAMTRON_CHUNKING
static ssize_t ramtron_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct ramtron_dev_s *priv = (FAR struct ramtron_dev_s *)dev;
  FAR const struct ramtron_parts_s *part;

  DEBUGASSERT(priv != NULL && priv->part != NULL && buffer != NULL);
  part = priv->part;

  /* Handle parts that require chunked output differently */

  if (part->chunked)
    {
      return ramtron_bwrite_chunked(dev, startblock, nblocks, buffer);
    }
  else
    {
      return ramtron_bwrite_nonchunked(dev, startblock, nblocks, buffer);
    }
}
#endif

/****************************************************************************
 * Name: ramtron_read
 ****************************************************************************/

static ssize_t ramtron_read(FAR struct mtd_dev_s *dev,
                            off_t offset,
                            size_t nbytes,
                            FAR uint8_t *buffer)
{
  FAR struct ramtron_dev_s *priv = (FAR struct ramtron_dev_s *)dev;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the SPI bus NOW because the ramtron_waitwritecomplete call must be
   * executed with the bus locked.
   */

  ramtron_lock(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send "Read from Memory " instruction */

  SPI_SEND(priv->dev, RAMTRON_READ);

  /* Send the page offset high byte first. */

  ramtron_sendaddr(priv, offset);

  /* Then read all of the requested bytes */

  SPI_RECVBLOCK(priv->dev, buffer, nbytes);

  /* Deselect the FLASH and unlock the SPI bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
  ramtron_unlock(priv->dev);

  finfo("return nbytes: %d\n", (int)nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: ramtron_ioctl
 ****************************************************************************/

static int ramtron_ioctl(FAR struct mtd_dev_s *dev,
                         int cmd,
                         unsigned long arg)
{
  FAR struct ramtron_dev_s *priv = (FAR struct ramtron_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  finfo("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
                    (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the
               * client will expect the device logic to do whatever is
               * necessary to make it appear so.
               */

              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;
              ret               = OK;

              finfo("blocksize: %ld erasesize: %ld neraseblocks: %ld\n",
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
                                  (priv->sectorshift - priv->pageshift);
              info->sectorsize  = 1 << priv->pageshift;
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        finfo("BULDERASE: Makes no sense in ramtron.\n");
        finfo("BULDERASE: Let's confirm operation as OK\n");
        ret = OK;
        break;

#ifdef CONFIG_RAMTRON_SETSPEED
      case MTDIOC_SETSPEED:
        {
          if (arg > 0 && arg <= RAMTRON_INIT_CLK_MAX)
            {
              priv->speed = arg;
              finfo("set bus speed to %lu\n", priv->speed);
              ret = OK;
            }
        }
        break;
#endif

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ramtron_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.
 *   MTD devices are not registered in the file system, but are created
 *   as instances that can be bound to other functions
 *   (such as a block or character driver front end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *ramtron_initialize(FAR struct spi_dev_s *dev)
{
  FAR struct ramtron_dev_s *priv;

  finfo("dev: %p\n", dev);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_FLASH(0) definition) and so would
   * have to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct ramtron_dev_s *)
          kmm_zalloc(sizeof(struct ramtron_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure. (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = ramtron_erase;
      priv->mtd.bread  = ramtron_bread;
      priv->mtd.bwrite = ramtron_bwrite;
      priv->mtd.read   = ramtron_read;
      priv->mtd.ioctl  = ramtron_ioctl;
      priv->mtd.name   = "ramtron";
      priv->dev        = dev;

      /* Deselect the FLASH */

      SPI_SELECT(dev, SPIDEV_FLASH(0), false);

      /* Identify the FLASH chip and get its capacity */

      if (ramtron_readid(priv) != OK)
        {
          /* Unrecognized! Discard all of that work we just did and
           * return NULL
           */

          kmm_free(priv);
          return NULL;
        }
    }

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
