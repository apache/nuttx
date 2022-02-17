/****************************************************************************
 * drivers/mtd/n25qxxx.c
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
#include <nuttx/spi/qspi.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* QuadSPI Mode.  Per data sheet, either Mode 0 or Mode 3 may be used. */

#ifndef CONFIG_N25QXXX_QSPIMODE
#define CONFIG_N25QXXX_QSPIMODE QSPIDEV_MODE0
#endif

/* QuadSPI Frequency per data sheet:
 *
 * In this implementation, only "Quad" reads are performed.
 */

#ifndef CONFIG_N25QXXX_QSPI_FREQUENCY
/* If you haven't specified frequency, default to 40 MHz which will work with
 * all commands.
 */
#define CONFIG_N25QXXX_QSPI_FREQUENCY 40000000
#endif

#ifndef CONFIG_N25QXXX_DUMMIES
/* If you haven't specified the number of dummy cycles for quad reads,
 * provide a reasonable default.
 * The actual number of dummies needed is clock and IO command dependent.
 */
#define CONFIG_N25QXXX_DUMMIES 6
#endif

/* N25QXXX Commands *********************************************************/

/* Configuration, Status, Erase, Program Commands ***************************/

/*      Command                 Value    Description:                       */

/*                                       Data sequence                      */

#define N25QXXX_READ_STATUS     0x05  /* Read status register:              *
                                       *   0x05 | SR                        */
#define N25QXXX_WRITE_STATUS    0x01  /* Write status register:             *
                                       *   0x01 | SR                        */
#define N25QXXX_READ_VOLCFG     0x85  /* Read volatile configuration register:   *
                                       *   0x85 | VCR                       */
#define N25QXXX_WRITE_VOLCFG    0x81  /* Write svolatile configuration register: *
                                       *   0x81 | VCR                       */
#define N25QXXX_WRITE_ENABLE    0x06  /* Write enable:                      *
                                       *   0x06                             */
#define N25QXXX_WRITE_DISABLE   0x04  /* Write disable command code:        *
                                       *   0x04                             */
#define N25QXXX_PAGE_PROGRAM    0x02  /* Page Program:
                                       *   0x02 | ADDR(MS) | ADDR(MID) |
                                       *   ADDR(LS) | data                  */
#define N25QXXX_SUBSECTOR_ERASE 0x20  /* Sub-sector Erase (4 kB)            *
                                       *   0x20 | ADDR(MS) | ADDR(MID) |    *
                                       *   ADDR(LS)                         */
#define N25QXXX_BULK_ERASE      0xc7  /* Bulk erase:                        *
                                       *   0xc7                             */

/* Read Commands ************************************************************/

/*      Command                  Value    Description:                      */

/*                                            Data sequence                 */
#define N25QXXX_FAST_READ_QUADIO 0xeb  /* Fast Read Quad I/O:               *
                                        *   0xeb | ADDR | data...           */

/* Reset Commands ***********************************************************/

/*      Command                  Value    Description:                      */

/*                                            Data sequence                 */

/* ID/Security Commands *****************************************************/

/*      Command                  Value    Description:                      */

/*                                          Data sequence                   */
#define N25QXXX_JEDEC_ID        0x9f  /* JEDEC ID:                          *
                                       * 0x9f | Manufacturer | MemoryType | *
                                       * Capacity                           */

/* Flash Manufacturer JEDEC IDs */

#define N25QXXX_JEDEC_ID_SPANSION  0x01
#define N25QXXX_JEDEC_ID_ATMEL     0x1f
#define N25QXXX_JEDEC_ID_ST        0x20
#define N25QXXX_JEDEC_ID_SST       0xbf
#define N25QXXX_JEDEC_ID_MACRONIX  0xc2
#define N25QXXX_JEDEC_ID_WINBOND   0xef

/* N25QXXX JEDIC IDs */

#define N25QXXX3V_JEDEC_DEVICE_TYPE 0xba  /* 3v memory device type */
#define N25QXXX2V_JEDEC_DEVICE_TYPE 0xbb  /* 2v memory device type */

#define N25Q016_JEDEC_CAPACITY    0x15  /* N25Q016 (2 MB) memory capacity */
#define N25Q032_JEDEC_CAPACITY    0x16  /* N25Q032 (4 MB) memory capacity */
#define N25Q064_JEDEC_CAPACITY    0x17  /* N25Q064 (8 MB) memory capacity */
#define N25Q128_JEDEC_CAPACITY    0x18  /* N25Q128 (16 MB) memory capacity */
#define N25Q256_JEDEC_CAPACITY    0x19  /* N25Q256 (32 MB) memory capacity */
#define N25Q512_JEDEC_CAPACITY    0x20  /* N25Q512 (64 MB) memory capacity */
#define N25Q00_JEDEC_CAPACITY     0x21  /* N25Q00 (128 MB) memory capacity */

/* N25QXXX Registers ********************************************************/

/* Status register bit definitions                                          */

#define STATUS_BUSY_MASK     (1 << 0) /* Bit 0: Device ready/busy status    */
#define STATUS_READY         (0 << 0) /*   0 = Not Busy                     */
#define STATUS_BUSY          (1 << 0) /*   1 = Busy                         */
#define STATUS_WEL_MASK      (1 << 1) /* Bit 1: Write enable latch status   */
#define STATUS_WEL_DISABLED  (0 << 1) /*   0 = Not Write Enabled            */
#define STATUS_WEL_ENABLED   (1 << 1) /*   1 = Write Enabled                */
#define STATUS_BP_SHIFT      (2)      /* Bits 2-4: Block protect bits       */
#define STATUS_BP_MASK       (7 << STATUS_BP_SHIFT)
#define STATUS_BP_NONE       (0 << STATUS_BP_SHIFT)
#define STATUS_BP_ALL        (7 << STATUS_BP_SHIFT)
#define STATUS_TB_MASK       (1 << 5) /* Bit 5: Top / Bottom Protect        */
#define STATUS_TB_TOP        (0 << 5) /*   0 = BP2-BP0 protect Top down     */
#define STATUS_TB_BOTTOM     (1 << 5) /*   1 = BP2-BP0 protect Bottom up    */
#define STATUS_BP3_MASK      (1 << 5) /* Bit 6: BP3                         */
#define STATUS_SRP0_MASK     (1 << 7) /* Bit 7: Status register protect 0   */
#define STATUS_SRP0_UNLOCKED (0 << 7) /*   0 = WP# no effect / PS Lock Down */
#define STATUS_SRP0_LOCKED   (1 << 7) /*   1 = WP# protect / OTP Lock Down  */

/* Chip Geometries **********************************************************/

/* All members of the family support uniform 4K-byte 'sub sectors'; they also
 * support 64k (and sometimes 32k) 'sectors' proper, but we won't be using
 * those here.
 */

/* N25Q016 (2 MB) memory capacity */

#define N25Q016_SECTOR_SIZE   (4*1024)
#define N25Q016_SECTOR_SHIFT  (12)
#define N25Q016_SECTOR_COUNT  (512)
#define N25Q016_PAGE_SIZE     (256)
#define N25Q016_PAGE_SHIFT    (8)

/* N25Q032 (4 MB) memory capacity */

#define N25Q032_SECTOR_SIZE   (4*1024)
#define N25Q032_SECTOR_SHIFT  (12)
#define N25Q032_SECTOR_COUNT  (1024)
#define N25Q032_PAGE_SIZE     (256)
#define N25Q032_PAGE_SHIFT    (8)

/* N25Q064 (8 MB) memory capacity */

#define N25Q064_SECTOR_SIZE   (4*1024)
#define N25Q064_SECTOR_SHIFT  (12)
#define N25Q064_SECTOR_COUNT  (2048)
#define N25Q064_PAGE_SIZE     (256)
#define N25Q064_PAGE_SHIFT    (8)

/* N25Q128 (16 MB) memory capacity */

#define N25Q128_SECTOR_SIZE   (4*1024)
#define N25Q128_SECTOR_SHIFT  (12)
#define N25Q128_SECTOR_COUNT  (4096)
#define N25Q128_PAGE_SIZE     (256)
#define N25Q128_PAGE_SHIFT    (8)

/* N25Q256 (32 MB) memory capacity */

#define N25Q256_SECTOR_SIZE   (4*1024)
#define N25Q256_SECTOR_SHIFT  (12)
#define N25Q256_SECTOR_COUNT  (8196)
#define N25Q256_PAGE_SIZE     (256)
#define N25Q256_PAGE_SHIFT    (8)

/* N25Q512 (64 MB) memory capacity */

#define N25Q512_SECTOR_SIZE   (4*1024)
#define N25Q512_SECTOR_SHIFT  (12)
#define N25Q512_SECTOR_COUNT  (16384)
#define N25Q512_PAGE_SIZE     (256)
#define N25Q512_PAGE_SHIFT    (8)

/* N25Q00 (128 MB) memory capacity */

#define N25Q00_SECTOR_SIZE    (4*1024)
#define N25Q00_SECTOR_SHIFT   (12)
#define N25Q00_SECTOR_COUNT   (32768)
#define N25Q00_PAGE_SIZE      (256)
#define N25Q00_PAGE_SHIFT     (8)

/* Cache flags **************************************************************/

#define N25QXXX_CACHE_VALID   (1 << 0)  /* 1=Cache has valid data */
#define N25QXXX_CACHE_DIRTY   (1 << 1)  /* 1=Cache is dirty */
#define N25QXXX_CACHE_ERASED  (1 << 2)  /* 1=Backing FLASH is erased */

#define IS_VALID(p)           ((((p)->flags) & N25QXXX_CACHE_VALID) != 0)
#define IS_DIRTY(p)           ((((p)->flags) & N25QXXX_CACHE_DIRTY) != 0)
#define IS_ERASED(p)          ((((p)->flags) & N25QXXX_CACHE_ERASED) != 0)

#define SET_VALID(p)          do { (p)->flags |= N25QXXX_CACHE_VALID; } while (0)
#define SET_DIRTY(p)          do { (p)->flags |= N25QXXX_CACHE_DIRTY; } while (0)
#define SET_ERASED(p)         do { (p)->flags |= N25QXXX_CACHE_ERASED; } while (0)

#define CLR_VALID(p)          do { (p)->flags &= ~N25QXXX_CACHE_VALID; } while (0)
#define CLR_DIRTY(p)          do { (p)->flags &= ~N25QXXX_CACHE_DIRTY; } while (0)
#define CLR_ERASED(p)         do { (p)->flags &= ~N25QXXX_CACHE_ERASED; } while (0)

/* 512 byte sector support **************************************************/

#define N25QXXX_SECTOR512_SHIFT     9
#define N25QXXX_SECTOR512_SIZE      (1 << 9)
#define N25QXXX_ERASED_STATE        0xff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.
 * The struct mtd_dev_s must appear at the beginning of the definition
 * so that you can freely cast between pointers to struct mtd_dev_s and
 * struct n25qxxx_dev_s.
 */

struct n25qxxx_dev_s
{
  struct mtd_dev_s       mtd;         /* MTD interface */
  FAR struct qspi_dev_s *qspi;        /* Saved QuadSPI interface instance */
  uint16_t               nsectors;    /* Number of erase sectors */
  uint8_t                sectorshift; /* Log2 of sector size */
  uint8_t                pageshift;   /* Log2 of page size */
  FAR uint8_t           *cmdbuf;      /* Allocated command buffer */
  FAR uint8_t           *readbuf;     /* Allocated status read buffer */

#ifdef CONFIG_N25QXXX_SECTOR512
  uint8_t                flags;       /* Buffered sector flags */
  uint16_t               esectno;     /* Erase sector number in the cache */
  FAR uint8_t           *sector;      /* Allocated sector data */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Locking */

static void n25qxxx_lock(FAR struct qspi_dev_s *qspi);
static inline void n25qxxx_unlock(FAR struct qspi_dev_s *qspi);

/* Low-level message helpers */

static int  n25qxxx_command(FAR struct qspi_dev_s *qspi, uint8_t cmd);
static int  n25qxxx_command_address(FAR struct qspi_dev_s *qspi,
                                    uint8_t cmd,
                                    off_t addr,
                                    uint8_t addrlen);
static int  n25qxxx_command_read(FAR struct qspi_dev_s *qspi,
                                 uint8_t cmd,
                                 FAR void *buffer,
                                 size_t buflen);
static int  n25qxxx_command_write(FAR struct qspi_dev_s *qspi,
                                  uint8_t cmd,
                                  FAR const void *buffer,
                                  size_t buflen);
static uint8_t n25qxxx_read_status(FAR struct n25qxxx_dev_s *priv);
static void n25qxxx_write_status(FAR struct n25qxxx_dev_s *priv);
static uint8_t n25qxxx_read_volcfg(FAR struct n25qxxx_dev_s *priv);
static void n25qxxx_write_volcfg(FAR struct n25qxxx_dev_s *priv);
static void n25qxxx_write_enable(FAR struct n25qxxx_dev_s *priv);
static void n25qxxx_write_disable(FAR struct n25qxxx_dev_s *priv);

static int  n25qxxx_readid(FAR struct n25qxxx_dev_s *priv);
static int  n25qxxx_protect(FAR struct n25qxxx_dev_s *priv,
                            off_t startblock,
                            size_t nblocks);
static int  n25qxxx_unprotect(FAR struct n25qxxx_dev_s *priv,
                              off_t startblock,
                              size_t nblocks);
static bool n25qxxx_isprotected(FAR struct n25qxxx_dev_s *priv,
                                uint8_t status,
                                off_t address);
static int  n25qxxx_erase_sector(FAR struct n25qxxx_dev_s *priv,
                                 off_t offset);
static int  n25qxxx_erase_chip(FAR struct n25qxxx_dev_s *priv);
static int  n25qxxx_read_byte(FAR struct n25qxxx_dev_s *priv,
                              FAR uint8_t *buffer,
                              off_t address,
                              size_t nbytes);
static int  n25qxxx_write_page(FAR struct n25qxxx_dev_s *priv,
                               FAR const uint8_t *buffer,
                               off_t address,
                               size_t nbytes);
#ifdef CONFIG_N25QXXX_SECTOR512
static int  n25qxxx_flush_cache(struct n25qxxx_dev_s *priv);
static FAR uint8_t *n25qxxx_read_cache(struct n25qxxx_dev_s *priv,
                                       off_t sector);
static void n25qxxx_erase_cache(struct n25qxxx_dev_s *priv,
                                off_t sector);
static int  n25qxxx_write_cache(FAR struct n25qxxx_dev_s *priv,
                                FAR const uint8_t *buffer,
                                off_t sector,
                                size_t nsectors);
#endif

/* MTD driver methods */

static int  n25qxxx_erase(FAR struct mtd_dev_s *dev,
                          off_t startblock,
                          size_t nblocks);
static ssize_t n25qxxx_bread(FAR struct mtd_dev_s *dev,
                             off_t startblock,
                             size_t nblocks,
                             FAR uint8_t *buf);
static ssize_t n25qxxx_bwrite(FAR struct mtd_dev_s *dev,
                              off_t startblock,
                              size_t nblocks,
                              FAR const uint8_t *buf);
static ssize_t n25qxxx_read(FAR struct mtd_dev_s *dev,
                            off_t offset,
                            size_t nbytes,
                            FAR uint8_t *buffer);
static int  n25qxxx_ioctl(FAR struct mtd_dev_s *dev,
                          int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n25qxxx_lock
 ****************************************************************************/

static void n25qxxx_lock(FAR struct qspi_dev_s *qspi)
{
  /* On QuadSPI buses where there are multiple devices, it will be necessary
   * to lock QuadSPI to have exclusive access to the buses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusive
   * access to the QuadSPI bus.  We will retain that exclusive access until
   * the bus is unlocked.
   */

  QSPI_LOCK(qspi, true);

  /* After locking the QuadSPI bus, the we also need call the setfrequency,
   * setbits, and setmode methods to make sure that the QuadSPI is properly
   * configured for the device. If the QuadSPI bus is being shared, then it
   * may have been left in an incompatible state.
   */

  QSPI_SETMODE(qspi, CONFIG_N25QXXX_QSPIMODE);
  QSPI_SETBITS(qspi, 8);
  QSPI_SETFREQUENCY(qspi, CONFIG_N25QXXX_QSPI_FREQUENCY);
}

/****************************************************************************
 * Name: n25qxxx_unlock
 ****************************************************************************/

static inline void n25qxxx_unlock(FAR struct qspi_dev_s *qspi)
{
  QSPI_LOCK(qspi, false);
}

/****************************************************************************
 * Name: n25qxxx_command
 ****************************************************************************/

static int n25qxxx_command(FAR struct qspi_dev_s *qspi, uint8_t cmd)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x\n", cmd);

  cmdinfo.flags   = 0;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = NULL;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
 * Name: n25qxxx_command_address
 ****************************************************************************/

static int n25qxxx_command_address(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                  off_t addr, uint8_t addrlen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x Address: %04lx addrlen=%d\n",
        cmd,
        (unsigned long)addr,
        addrlen);

  cmdinfo.flags   = QSPICMD_ADDRESS;
  cmdinfo.addrlen = addrlen;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = addr;
  cmdinfo.buffer  = NULL;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
 * Name: n25qxxx_command_read
 ****************************************************************************/

static int n25qxxx_command_read(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                FAR void *buffer, size_t buflen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x buflen: %lu\n", cmd, (unsigned long)buflen);

  cmdinfo.flags   = QSPICMD_READDATA;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = buflen;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = buffer;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
 * Name: n25qxxx_command_write
 ****************************************************************************/

static int n25qxxx_command_write(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                 FAR const void *buffer, size_t buflen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x buflen: %lu\n", cmd, (unsigned long)buflen);

  cmdinfo.flags   = QSPICMD_WRITEDATA;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = buflen;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = (FAR void *)buffer;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
 * Name: n25qxxx_read_status
 ****************************************************************************/

static uint8_t n25qxxx_read_status(FAR struct n25qxxx_dev_s *priv)
{
  DEBUGVERIFY(n25qxxx_command_read(priv->qspi, N25QXXX_READ_STATUS,
                                  (FAR void *)&priv->readbuf[0], 1));
  return priv->readbuf[0];
}

/****************************************************************************
 * Name:  n25qxxx_write_status
 ****************************************************************************/

static void n25qxxx_write_status(FAR struct n25qxxx_dev_s *priv)
{
  n25qxxx_write_enable(priv);

  /* take care to mask of the SRP bit; it is one-time-programmable */

  priv->cmdbuf[0] &= ~STATUS_SRP0_MASK;

  n25qxxx_command_write(priv->qspi, N25QXXX_WRITE_STATUS,
                       (FAR const void *)priv->cmdbuf, 1);
  n25qxxx_write_disable(priv);
}

/****************************************************************************
 * Name: n25qxxx_read_volcfg
 ****************************************************************************/

static uint8_t n25qxxx_read_volcfg(FAR struct n25qxxx_dev_s *priv)
{
  DEBUGVERIFY(n25qxxx_command_read(priv->qspi, N25QXXX_READ_VOLCFG,
                                  (FAR void *)&priv->readbuf[0], 1));
  return priv->readbuf[0];
}

/****************************************************************************
 * Name:  n25qxxx_write_volcfg
 ****************************************************************************/

static void n25qxxx_write_volcfg(FAR struct n25qxxx_dev_s *priv)
{
  n25qxxx_write_enable(priv);
  n25qxxx_command_write(priv->qspi, N25QXXX_WRITE_VOLCFG,
                       (FAR const void *)priv->cmdbuf, 1);
  n25qxxx_write_disable(priv);
}

/****************************************************************************
 * Name:  n25qxxx_write_enable
 ****************************************************************************/

static void n25qxxx_write_enable(FAR struct n25qxxx_dev_s *priv)
{
  uint8_t status;

  do
    {
      n25qxxx_command(priv->qspi, N25QXXX_WRITE_ENABLE);
      status = n25qxxx_read_status(priv);
    }
  while ((status & STATUS_WEL_MASK) != STATUS_WEL_ENABLED);
}

/****************************************************************************
 * Name:  n25qxxx_write_disable
 ****************************************************************************/

static void n25qxxx_write_disable(FAR struct n25qxxx_dev_s *priv)
{
  uint8_t status;

  do
    {
      n25qxxx_command(priv->qspi, N25QXXX_WRITE_DISABLE);
      status = n25qxxx_read_status(priv);
    }
  while ((status & STATUS_WEL_MASK) != STATUS_WEL_DISABLED);
}

/****************************************************************************
 * Name: n25qxxx_readid
 ****************************************************************************/

static inline int n25qxxx_readid(struct n25qxxx_dev_s *priv)
{
  /* Lock the QuadSPI bus and configure the bus. */

  n25qxxx_lock(priv->qspi);

  /* Read the JEDEC ID */

  n25qxxx_command_read(priv->qspi, N25QXXX_JEDEC_ID, priv->cmdbuf, 3);

  /* Unlock the bus */

  n25qxxx_unlock(priv->qspi);

  finfo("Manufacturer: %02x Device Type %02x, Capacity: %02x\n",
        priv->cmdbuf[0], priv->cmdbuf[1], priv->cmdbuf[2]);

  /* Check for a recognized memory device type */

  if (priv->cmdbuf[1] != N25QXXX3V_JEDEC_DEVICE_TYPE &&
      priv->cmdbuf[1] != N25QXXX2V_JEDEC_DEVICE_TYPE)
    {
      ferr("ERROR: Unrecognized device type: 0x%02x\n", priv->cmdbuf[1]);
      return -ENODEV;
    }

  /* Check for a supported capacity */

  switch (priv->cmdbuf[2])
    {
      case N25Q016_JEDEC_CAPACITY:
        priv->sectorshift = N25Q016_SECTOR_SHIFT;
        priv->pageshift   = N25Q016_PAGE_SHIFT;
        priv->nsectors    = N25Q016_SECTOR_COUNT;
        break;

      case N25Q032_JEDEC_CAPACITY:
        priv->sectorshift = N25Q032_SECTOR_SHIFT;
        priv->pageshift   = N25Q032_PAGE_SHIFT;
        priv->nsectors    = N25Q032_SECTOR_COUNT;
        break;

      case N25Q064_JEDEC_CAPACITY:
        priv->sectorshift = N25Q064_SECTOR_SHIFT;
        priv->pageshift   = N25Q064_PAGE_SHIFT;
        priv->nsectors    = N25Q064_SECTOR_COUNT;
        break;

      case N25Q128_JEDEC_CAPACITY:
        priv->sectorshift = N25Q128_SECTOR_SHIFT;
        priv->pageshift   = N25Q128_PAGE_SHIFT;
        priv->nsectors    = N25Q128_SECTOR_COUNT;
        break;

      case N25Q256_JEDEC_CAPACITY:
        priv->sectorshift = N25Q256_SECTOR_SHIFT;
        priv->pageshift   = N25Q256_PAGE_SHIFT;
        priv->nsectors    = N25Q256_SECTOR_COUNT;
        break;

      case N25Q512_JEDEC_CAPACITY:
        priv->sectorshift = N25Q512_SECTOR_SHIFT;
        priv->pageshift   = N25Q512_PAGE_SHIFT;
        priv->nsectors    = N25Q512_SECTOR_COUNT;
        break;

      case N25Q00_JEDEC_CAPACITY:
        priv->sectorshift = N25Q00_SECTOR_SHIFT;
        priv->pageshift   = N25Q00_PAGE_SHIFT;
        priv->nsectors    = N25Q00_SECTOR_COUNT;
        break;

      /* Support for this part is not implemented yet */

      default:
        ferr("ERROR: Unsupported memory capacity: %02x\n", priv->cmdbuf[2]);
        return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: n25qxxx_protect
 ****************************************************************************/

static int n25qxxx_protect(FAR struct n25qxxx_dev_s *priv,
                          off_t startblock, size_t nblocks)
{
  /* Get the status register value to check the current protection */

  priv->cmdbuf[0] = n25qxxx_read_status(priv);

  if ((priv->cmdbuf[0] & STATUS_BP_MASK) == STATUS_BP_NONE)
    {
      /* Protection already disabled */

      return 0;
    }

  /* Check if sector protection registers are locked */

  if ((priv->cmdbuf[0] & STATUS_SRP0_MASK) == STATUS_SRP0_LOCKED)
    {
      /* Yes.. unprotect section protection registers */

      priv->cmdbuf[0] &= ~STATUS_SRP0_MASK;
      n25qxxx_write_status(priv);
    }

  /* Set the protection mask to zero.
   * REVISIT:  This logic should really just set the BP bits as
   * necessary to protect the range of sectors.
   */

  priv->cmdbuf[0] |= (STATUS_BP3_MASK | STATUS_BP_MASK);
  n25qxxx_write_status(priv);

  /* Check the new status */

  priv->cmdbuf[0] = n25qxxx_read_status(priv);
  if ((priv->cmdbuf[0] & (STATUS_BP3_MASK | STATUS_BP_MASK)) !=
      (STATUS_BP3_MASK | STATUS_BP_MASK))
    {
      return -EACCES;
    }

  return OK;
}

/****************************************************************************
 * Name: n25qxxx_unprotect
 ****************************************************************************/

static int n25qxxx_unprotect(FAR struct n25qxxx_dev_s *priv,
                            off_t startblock, size_t nblocks)
{
  /* Get the status register value to check the current protection */

  priv->cmdbuf[0] = n25qxxx_read_status(priv);

  if ((priv->cmdbuf[0] & (STATUS_BP3_MASK | STATUS_BP_MASK)) == 0)
    {
      /* Protection already disabled */

      return 0;
    }

  /* Check if sector protection registers are locked */

  if ((priv->cmdbuf[0] & STATUS_SRP0_MASK) == STATUS_SRP0_LOCKED)
    {
      /* the SRP bit is one time programmable; if it's set, there's nothing
       * that you can do to unset it.
       */

      return -EACCES;
    }

  /* Set the protection mask to zero (and not complemented).
   * REVISIT:  This logic should really just re-write the BP bits as
   * necessary to unprotect the range of sectors.
   */

  priv->cmdbuf[0] &= ~(STATUS_BP3_MASK | STATUS_BP_MASK);
  n25qxxx_write_status(priv);

  /* Check the new status */

  priv->cmdbuf[0] = n25qxxx_read_status(priv);
  if ((priv->cmdbuf[0] &
      (STATUS_SRP0_MASK | STATUS_BP3_MASK | STATUS_BP_MASK)) != 0)
    {
      return -EACCES;
    }

  return OK;
}

/****************************************************************************
 * Name: n25qxxx_isprotected
 ****************************************************************************/

static bool n25qxxx_isprotected(FAR struct n25qxxx_dev_s *priv,
                                uint8_t status,
                                off_t address)
{
  off_t protstart;
  off_t protend;
  off_t protsize;
  unsigned int bp;

  /* The BP field is spread across non-contiguous bits */

  bp = (status & STATUS_BP_MASK) >> STATUS_BP_SHIFT;
  if (status & STATUS_BP3_MASK)
    {
      bp |= 8;
    }

  /* the BP field is essentially the power-of-two of the number of 64k
   * sectors, saturated to the device size.
   */

  if (0 == bp)
    {
      return false;
    }

  protsize = 0x00010000;
  protsize <<= (protsize << (bp - 1));
  protend = (1 << priv->sectorshift) * priv->nsectors;
  if (protsize > protend)
    {
      protsize = protend;
    }

  /* The final protection range then depends on if the protection region is
   * configured top-down or bottom up  (assuming CMP=0).
   */

  if ((status & STATUS_TB_MASK) != 0)
    {
      protstart = 0x00000000;
      protend   = protstart + protsize;
    }
  else
    {
      protstart = protend - protsize;

      /* protend already computed above */
    }

  return (address >= protstart && address < protend);
}

/****************************************************************************
 * Name:  n25qxxx_erase_sector
 ****************************************************************************/

static int n25qxxx_erase_sector(struct n25qxxx_dev_s *priv, off_t sector)
{
  off_t address;
  uint8_t status;

  finfo("sector: %08lx\n", (unsigned long)sector);

  /* Check that the flash is ready and unprotected */

  status = n25qxxx_read_status(priv);
  if ((status & STATUS_BUSY_MASK) != STATUS_READY)
    {
      ferr("ERROR: Flash busy: %02x", status);
      return -EBUSY;
    }

  /* Get the address associated with the sector */

  address = (off_t)sector << priv->sectorshift;

  if ((status & (STATUS_BP3_MASK | STATUS_BP_MASK)) != 0 &&
      n25qxxx_isprotected(priv, status, address))
    {
      ferr("ERROR: Flash protected: %02x", status);
      return -EACCES;
    }

  /* Send the sector erase command */

  n25qxxx_write_enable(priv);
  n25qxxx_command_address(priv->qspi, N25QXXX_SUBSECTOR_ERASE, address, 3);

  /* Wait for erasure to finish */

  while ((n25qxxx_read_status(priv) & STATUS_BUSY_MASK) != 0);

  return OK;
}

/****************************************************************************
 * Name:  n25qxxx_erase_chip
 ****************************************************************************/

static int n25qxxx_erase_chip(struct n25qxxx_dev_s *priv)
{
  uint8_t status;

  /* Check if the FLASH is protected */

  status = n25qxxx_read_status(priv);
  if ((status & (STATUS_BP3_MASK | STATUS_BP_MASK)) != 0)
    {
      ferr("ERROR: FLASH is Protected: %02x", status);
      return -EACCES;
    }

  /* Erase the whole chip */

  n25qxxx_write_enable(priv);
  n25qxxx_command(priv->qspi, N25QXXX_BULK_ERASE);

  /* Wait for the erasure to complete */

  status = n25qxxx_read_status(priv);
  while ((status & STATUS_BUSY_MASK) != 0)
    {
      nxsig_usleep(200 * 1000);
      status = n25qxxx_read_status(priv);
    }

  return OK;
}

/****************************************************************************
 * Name: n25qxxx_read_byte
 ****************************************************************************/

static int n25qxxx_read_byte(FAR struct n25qxxx_dev_s *priv,
                             FAR uint8_t *buffer,
                             off_t address,
                             size_t buflen)
{
  struct qspi_meminfo_s meminfo;

  finfo("address: %08lx nbytes: %d\n", (long)address, (int)buflen);

  meminfo.flags   = QSPIMEM_READ | QSPIMEM_QUADIO;
  meminfo.addrlen = 3;
  meminfo.dummies = CONFIG_N25QXXX_DUMMIES;
  meminfo.buflen  = buflen;
  meminfo.cmd     = N25QXXX_FAST_READ_QUADIO;
  meminfo.addr    = address;
  meminfo.buffer  = buffer;

  return QSPI_MEMORY(priv->qspi, &meminfo);
}

/****************************************************************************
 * Name:  n25qxxx_write_page
 ****************************************************************************/

static int n25qxxx_write_page(struct n25qxxx_dev_s *priv,
                              FAR const uint8_t *buffer,
                              off_t address,
                              size_t buflen)
{
  struct qspi_meminfo_s meminfo;
  unsigned int pagesize;
  unsigned int npages;
  int ret;
  int i;

  finfo("address: %08lx buflen: %u\n",
       (unsigned long)address,
       (unsigned)buflen);

  npages   = (buflen >> priv->pageshift);
  pagesize = (1 << priv->pageshift);

  /* Set up non-varying parts of transfer description */

  meminfo.flags   = QSPIMEM_WRITE;
  meminfo.cmd     = N25QXXX_PAGE_PROGRAM;
  meminfo.addrlen = 3;
  meminfo.buflen  = pagesize;
  meminfo.dummies = 0;

  /* Then write each page */

  for (i = 0; i < npages; i++)
    {
      /* Set up varying parts of the transfer description */

      meminfo.addr   = address;
      meminfo.buffer = (void *)buffer;

      /* Write one page */

      n25qxxx_write_enable(priv);
      ret = QSPI_MEMORY(priv->qspi, &meminfo);
      n25qxxx_write_disable(priv);

      if (ret < 0)
        {
          ferr("ERROR: QSPI_MEMORY failed writing address=%06jx\n",
               (intmax_t)address);
          return ret;
        }

      /* Update for the next time through the loop */

      buffer  += pagesize;
      address += pagesize;
      buflen  -= pagesize;
    }

  /* The transfer should always be an even number of sectors and hence also
   * pages.  There should be no remainder.
   */

  DEBUGASSERT(buflen == 0);

  return OK;
}

/****************************************************************************
 * Name: n25qxxx_flush_cache
 ****************************************************************************/

#ifdef CONFIG_N25QXXX_SECTOR512
static int n25qxxx_flush_cache(struct n25qxxx_dev_s *priv)
{
  int ret = OK;

  /* If the cache is dirty (meaning that it no longer matches the old FLASH
   * contents) or was erased (with the cache containing the correct FLASH
   * contents), then write the cached erase block to FLASH.
   */

  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      off_t address;

      /* Convert the erase sector number into a FLASH address */

      address = (off_t)priv->esectno << priv->sectorshift;

      /* Write entire erase block to FLASH */

      ret = n25qxxx_write_page(priv, priv->sector,
                               address, 1 << priv->sectorshift);
      if (ret < 0)
        {
          ferr("ERROR: n25qxxx_write_page failed: %d\n", ret);
        }

      /* The cache is no long dirty and the FLASH is no longer erased */

      CLR_DIRTY(priv);
      CLR_ERASED(priv);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: n25qxxx_read_cache
 ****************************************************************************/

#ifdef CONFIG_N25QXXX_SECTOR512
static FAR uint8_t *n25qxxx_read_cache(struct n25qxxx_dev_s *priv,
                                       off_t sector)
{
  off_t esectno;
  int   shift;
  int   index;
  int   ret;

  /* Convert from the 512 byte sector to the erase sector size of the device.
   * For example, if the actual erase sector size is 4Kb (1 << 12), then we
   * first shift to the right by 3 to get the sector number in 4096
   * increments.
   */

  shift    = priv->sectorshift - N25QXXX_SECTOR512_SHIFT;
  esectno  = sector >> shift;
  finfo("sector: %jd esectno: %jd shift=%d\n",
        (intmax_t)sector, (intmax_t)esectno, shift);

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || esectno != priv->esectno)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      ret = n25qxxx_flush_cache(priv);
      if (ret < 0)
        {
          ferr("ERROR: n25qxxx_flush_cache failed: %d\n", ret);
          return NULL;
        }

      /* Read the erase block into the cache */

      ret = n25qxxx_read_byte(priv, priv->sector,
                             (esectno << priv->sectorshift),
                             (1 << priv->sectorshift));
      if (ret < 0)
        {
          ferr("ERROR: n25qxxx_read_byte failed: %d\n", ret);
          return NULL;
        }

      /* Mark the sector as cached */

      priv->esectno = esectno;

      SET_VALID(priv);      /* The data in the cache is valid */
      CLR_DIRTY(priv);      /* It should match the FLASH contents */
      CLR_ERASED(priv);     /* The underlying FLASH has not been erased */
    }

  /* Get the index to the 512 sector in the erase block that holds the
   * argument
   */

  index = sector & ((1 << shift) - 1);

  /* Return the address in the cache that holds this sector */

  return &priv->sector[index << N25QXXX_SECTOR512_SHIFT];
}
#endif

/****************************************************************************
 * Name: n25qxxx_erase_cache
 ****************************************************************************/

#ifdef CONFIG_N25QXXX_SECTOR512
static void n25qxxx_erase_cache(struct n25qxxx_dev_s *priv, off_t sector)
{
  FAR uint8_t *dest;

  /* First, make sure that the erase block containing the 512 byte sector is
   * in the cache.
   */

  dest = n25qxxx_read_cache(priv, sector);

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase sector
   * is read into the cache and set here when we erase the block.
   */

  if (!IS_ERASED(priv))
    {
      off_t esectno  = sector >>
                       (priv->sectorshift - N25QXXX_SECTOR512_SHIFT);
      finfo("sector: %jd esectno: %jd\n",
            (intmax_t)sector, (intmax_t)esectno);

      DEBUGVERIFY(n25qxxx_erase_sector(priv, esectno));
      SET_ERASED(priv);
    }

  /* Put the cached sector data into the erase state and mark the cache as
   * dirty (but don't update the FLASH yet.  The caller will do that at a
   * more optimal time).
   */

  memset(dest, N25QXXX_ERASED_STATE, N25QXXX_SECTOR512_SIZE);
  SET_DIRTY(priv);
}
#endif

/****************************************************************************
 * Name: n25qxxx_write_cache
 ****************************************************************************/

#ifdef CONFIG_N25QXXX_SECTOR512
static int n25qxxx_write_cache(FAR struct n25qxxx_dev_s *priv,
                              FAR const uint8_t *buffer, off_t sector,
                              size_t nsectors)
{
  FAR uint8_t *dest;
  int ret;

  for (; nsectors > 0; nsectors--)
    {
      /* First, make sure that the erase block containing 512 byte sector is
       * in memory.
       */

      dest = n25qxxx_read_cache(priv, sector);

      /* Erase the block containing this sector if it is not already erased.
       * The erased indicated will be cleared when the data from the erase
       * sector is read into the cache and set here when we erase the sector.
       */

      if (!IS_ERASED(priv))
        {
          off_t esectno  = sector >>
                           (priv->sectorshift - N25QXXX_SECTOR512_SHIFT);
          finfo("sector: %jd esectno: %jd\n",
                (intmax_t)sector, (intmax_t)esectno);

          ret = n25qxxx_erase_sector(priv, esectno);
          if (ret < 0)
            {
              ferr("ERROR: n25qxxx_erase_sector failed: %d\n", ret);
              return ret;
            }

          SET_ERASED(priv);
        }

      /* Copy the new sector data into cached erase block */

      memcpy(dest, buffer, N25QXXX_SECTOR512_SIZE);
      SET_DIRTY(priv);

      /* Set up for the next 512 byte sector */

      buffer += N25QXXX_SECTOR512_SIZE;
      sector++;
    }

  /* Flush the last erase block left in the cache */

  return n25qxxx_flush_cache(priv);
}
#endif

/****************************************************************************
 * Name: n25qxxx_erase
 ****************************************************************************/

static int n25qxxx_erase(FAR struct mtd_dev_s *dev,
                         off_t startblock,
                         size_t nblocks)
{
  FAR struct n25qxxx_dev_s *priv = (FAR struct n25qxxx_dev_s *)dev;
  size_t blocksleft = nblocks;
#ifdef CONFIG_N25QXXX_SECTOR512
  int ret;
#endif

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  n25qxxx_lock(priv->qspi);

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

#ifdef CONFIG_N25QXXX_SECTOR512
      n25qxxx_erase_cache(priv, startblock);
#else
      n25qxxx_erase_sector(priv, startblock);
#endif
      startblock++;
    }

#ifdef CONFIG_N25QXXX_SECTOR512
  /* Flush the last erase block left in the cache */

  ret = n25qxxx_flush_cache(priv);
  if (ret < 0)
    {
      nblocks = ret;
    }
#endif

  n25qxxx_unlock(priv->qspi);

  return (int)nblocks;
}

/****************************************************************************
 * Name: n25qxxx_bread
 ****************************************************************************/

static ssize_t n25qxxx_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR uint8_t *buffer)
{
#ifndef CONFIG_N25QXXX_SECTOR512
  FAR struct n25qxxx_dev_s *priv = (FAR struct n25qxxx_dev_s *)dev;
#endif
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented
   * read
   */

#ifdef CONFIG_N25QXXX_SECTOR512
  nbytes = n25qxxx_read(dev, startblock << N25QXXX_SECTOR512_SHIFT,
                       nblocks << N25QXXX_SECTOR512_SHIFT, buffer);
  if (nbytes > 0)
    {
      nbytes >>= N25QXXX_SECTOR512_SHIFT;
    }
#else
  nbytes = n25qxxx_read(dev, startblock << priv->pageshift,
                       nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
      nbytes >>= priv->pageshift;
    }
#endif

  return nbytes;
}

/****************************************************************************
 * Name: n25qxxx_bwrite
 ****************************************************************************/

static ssize_t n25qxxx_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct n25qxxx_dev_s *priv = (FAR struct n25qxxx_dev_s *)dev;
  int ret = (int)nblocks;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the QuadSPI bus and write all of the pages to FLASH */

  n25qxxx_lock(priv->qspi);

#if defined(CONFIG_N25QXXX_SECTOR512)
  ret = n25qxxx_write_cache(priv, buffer, startblock, nblocks);
  if (ret < 0)
    {
      ferr("ERROR: n25qxxx_write_cache failed: %d\n", ret);
    }

#else
  ret = n25qxxx_write_page(priv, buffer, startblock << priv->pageshift,
                          nblocks << priv->pageshift);
  if (ret < 0)
    {
      ferr("ERROR: n25qxxx_write_page failed: %d\n", ret);
    }
#endif

  n25qxxx_unlock(priv->qspi);

  return ret < 0 ? ret : nblocks;
}

/****************************************************************************
 * Name: n25qxxx_read
 ****************************************************************************/

static ssize_t n25qxxx_read(FAR struct mtd_dev_s *dev,
                            off_t offset,
                            size_t nbytes,
                            FAR uint8_t *buffer)
{
  FAR struct n25qxxx_dev_s *priv = (FAR struct n25qxxx_dev_s *)dev;
  int ret;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the QuadSPI bus and select this FLASH part */

  n25qxxx_lock(priv->qspi);
  ret = n25qxxx_read_byte(priv, buffer, offset, nbytes);
  n25qxxx_unlock(priv->qspi);

  if (ret < 0)
    {
      ferr("ERROR: n25qxxx_read_byte returned: %d\n", ret);
      return (ssize_t)ret;
    }

  finfo("return nbytes: %d\n", (int)nbytes);
  return (ssize_t)nbytes;
}

/****************************************************************************
 * Name: n25qxxx_ioctl
 ****************************************************************************/

static int n25qxxx_ioctl(FAR struct mtd_dev_s *dev,
                         int cmd,
                         unsigned long arg)
{
  FAR struct n25qxxx_dev_s *priv = (FAR struct n25qxxx_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  finfo("cmd: %d\n", cmd);

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
               * of fixed size blocks.  That is most likely not true, but
               * the client will expect the device logic to do whatever is
               * necessary to make it appear so.
               */

#ifdef CONFIG_N25QXXX_SECTOR512
              geo->blocksize    = (1 << N25QXXX_SECTOR512_SHIFT);
              geo->erasesize    = (1 << N25QXXX_SECTOR512_SHIFT);
              geo->neraseblocks = priv->nsectors <<
                                  (priv->sectorshift -
                                   N25QXXX_SECTOR512_SHIFT);
#else
              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;
#endif
              ret               = OK;

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
#ifdef CONFIG_N25QXXX_SECTOR512
              info->numsectors  = priv->nsectors <<
                               (priv->sectorshift - N25QXXX_SECTOR512_SHIFT);
              info->sectorsize  = 1 << N25QXXX_SECTOR512_SHIFT;
#else
              info->numsectors  = priv->nsectors <<
                                  (priv->sectorshift - priv->pageshift);
              info->sectorsize  = 1 << priv->pageshift;
#endif
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          n25qxxx_lock(priv->qspi);
          ret = n25qxxx_erase_chip(priv);
          n25qxxx_unlock(priv->qspi);
        }
        break;

      case MTDIOC_PROTECT:
        {
          FAR const struct mtd_protect_s *prot =
            (FAR const struct mtd_protect_s *)((uintptr_t)arg);

          DEBUGASSERT(prot);
          ret = n25qxxx_protect(priv, prot->startblock, prot->nblocks);
        }
        break;

      case MTDIOC_UNPROTECT:
        {
          FAR const struct mtd_protect_s *prot =
            (FAR const struct mtd_protect_s *)((uintptr_t)arg);

          DEBUGASSERT(prot);
          ret = n25qxxx_unprotect(priv, prot->startblock, prot->nblocks);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result = N25QXXX_ERASED_STATE;

          ret = OK;
        }
        break;

      default:
        ret = -ENOTTY; /* Bad/unsupported command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n25qxxx_initialize
 *
 * Description:
 *   Create an initialize MTD device instance for the QuadSPI-based n25Qxxx
 *   FLASH part.
 *
 *   MTD devices are not registered in the file system, but are created as
 *   instances that can be bound to other functions (such as a block or
 *   character driver front end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *n25qxxx_initialize(FAR struct qspi_dev_s *qspi,
                                         bool unprotect)
{
  FAR struct n25qxxx_dev_s *priv;
  int ret;

  finfo("qspi: %p\n", qspi);
  DEBUGASSERT(qspi != NULL);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per QuadSPI
   * device (only because of the QSPIDEV_FLASH(0) definition) and so would
   * have to be extended to handle multiple FLASH parts on the same QuadSPI
   * bus.
   */

  priv = (FAR struct n25qxxx_dev_s *)
          kmm_zalloc(sizeof(struct n25qxxx_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = n25qxxx_erase;
      priv->mtd.bread  = n25qxxx_bread;
      priv->mtd.bwrite = n25qxxx_bwrite;
      priv->mtd.read   = n25qxxx_read;
      priv->mtd.ioctl  = n25qxxx_ioctl;
      priv->mtd.name   = "n25qxxx";
      priv->qspi       = qspi;

      /* Allocate a 4-byte buffer to support DMA-able command data */

      priv->cmdbuf = (FAR uint8_t *)QSPI_ALLOC(qspi, 4);
      if (priv->cmdbuf == NULL)
        {
          ferr("ERROR Failed to allocate command buffer\n");
          goto errout_with_priv;
        }

      /* Allocate a one-byte buffer to support DMA-able status read data */

      priv->readbuf = (FAR uint8_t *)QSPI_ALLOC(qspi, 1);
      if (priv->readbuf == NULL)
        {
          ferr("ERROR Failed to allocate read buffer\n");
          goto errout_with_cmdbuf;
        }

      /* Identify the FLASH chip and get its capacity */

      ret = n25qxxx_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and
           * return NULL
           */

          ferr("ERROR Unrecognized QSPI device\n");
          goto errout_with_readbuf;
        }

      /* Specify the number of dummy cycles via the 'volatile
       * configuration register'
       */

      priv->cmdbuf[0] = n25qxxx_read_volcfg(priv);
      priv->cmdbuf[0] &= 0x0f;
      priv->cmdbuf[0] |= (CONFIG_N25QXXX_DUMMIES << 4);
      n25qxxx_write_volcfg(priv);

      /* Unprotect FLASH sectors if so requested. */

      if (unprotect)
        {
          ret = n25qxxx_unprotect(priv, 0, priv->nsectors - 1);
          if (ret < 0)
            {
              ferr("ERROR: Sector unprotect failed\n");
            }
        }

#ifdef CONFIG_N25QXXX_SECTOR512  /* Simulate a 512 byte sector */
      /* Allocate a buffer for the erase block cache */

      priv->sector = (FAR uint8_t *)QSPI_ALLOC(qspi, 1 << priv->sectorshift);
      if (priv->sector == NULL)
        {
          /* Allocation failed! Discard all of that work we just did and
           * return NULL
           */

          ferr("ERROR: Sector allocation failed\n");
          goto errout_with_readbuf;
        }
#endif
    }

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;

errout_with_readbuf:
  QSPI_FREE(qspi, priv->readbuf);

errout_with_cmdbuf:
  QSPI_FREE(qspi, priv->cmdbuf);

errout_with_priv:
  kmm_free(priv);
  return NULL;
}
