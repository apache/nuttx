/****************************************************************************
 * drivers/mtd/sst26.c
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

/* Per the data sheet, SST26 parts can be driven with either SPI mode 0
 * (CPOL=0 and CPHA=0) or mode 3 (CPOL=1 and CPHA=1).
 * So you may need to specify CONFIG_SST26_SPIMODE to select the best mode
 * for your device. If CONFIG_SST26_SPIMODE is not defined, mode 0 will be
 * used.
 */

#ifndef CONFIG_SST26_SPIMODE
#define CONFIG_SST26_SPIMODE SPIDEV_MODE0
#endif

/* SPI Frequency.  May be up to 104 MHz. */

#ifndef CONFIG_SST26_SPIFREQUENCY
#define CONFIG_SST26_SPIFREQUENCY 20000000
#endif

/* Various manufacturers may have produced the parts.
 * 0xBF is the manufacturer ID for the SST serial FLASH.
 */

#ifndef CONFIG_SST26_MANUFACTURER
#define CONFIG_SST26_MANUFACTURER 0xBF
#endif

#ifndef CONFIG_SST26_MEMORY_TYPE
#define CONFIG_SST26_MEMORY_TYPE  0x26
#endif

/* SST26 Registers **********************************************************/

/* Identification register values */

#define SST26_MANUFACTURER         CONFIG_SST26_MANUFACTURER
#define SST26_MEMORY_TYPE          CONFIG_SST26_MEMORY_TYPE

#define SST26_SST26VF016_CAPACITY    0x41 /* 16 M-bit */
#define SST26_SST26VF032_CAPACITY    0x42 /* 32 M-bit */
#define SST26_SST26VF064_CAPACITY    0x43 /* 64 M-bit */

/* SST26VF016 capacity is 2,097,152 bytes:
 * (512 sectors) * (4,096 bytes per sector)
 * (8192 pages) * (256 bytes per page)
 */

#define SST26_SST26VF016_SECTOR_SHIFT  12    /* Sector size 1 << 15 = 65,536 */
#define SST26_SST26VF016_NSECTORS      512
#define SST26_SST26VF016_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define SST26_SST26VF016_NPAGES        8192

/* SST26VF032 capacity is 4,194,304 bytes:
 * (1,024 sectors) * (4,096 bytes per sector)
 * (16,384 pages) * (256 bytes per page)
 */

#define SST26_SST26VF032_SECTOR_SHIFT  12    /* Sector size 1 << 15 = 65,536 */
#define SST26_SST26VF032_NSECTORS      1024
#define SST26_SST26VF032_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define SST26_SST26VF032_NPAGES        16384

/* SST26VF064 capacity is 8,388,608 bytes:
 * (2,048 sectors) * (4,096 bytes per sector)
 * (32,768 pages) * (256 bytes per page)
 */

#define SST26_SST26VF064_SECTOR_SHIFT  12    /* Sector size 1 << 15 = 65,536 */
#define SST26_SST26VF064_NSECTORS      2048
#define SST26_SST26VF064_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define SST26_SST26VF064_NPAGES        32768

/* Instructions */

/*      Command         Value    NN Description          Addr Dummy  Data */

#define SST26_NOP       0x00 /* 14 No Operation             0   0   0     */
#define SST26_RSTEN     0x66 /* 14 Reset Enable             0   0   0     */
#define SST26_RST       0x99 /* 14 Reset Memory             0   0   0     */
#define SST26_EQIO      0x38 /* 1  Enable Quad I/O          0   0   0     */
#define SST26_RSTQIO    0xFF /*  4 Reset Quad I/O           0   0   0     */
#define SST26_RDSR      0x05 /* 1  Read Status Register     0   0   >=1   */
                             /*  4 Read Status Register     0   1   >=1   */
#define SST26_WRSR      0x01 /* 14 Write Status Register    0   0   2     */
#define SST26_RDCR      0x35 /* 1  Read Config Register     0   0   >=1   */
                             /*  4 Read Config Register     0   1   >=1   */
#define SST26_READ      0x03 /* 1  Read Data Bytes          3   0   >=1   */
#define SST26_FAST_READ 0x0b /* 1  Higher speed read        3   1   >=1   */
                             /*  4 Higher speed read        3   3   >=1   */
#define SST26_SQOR      0x6b /* 1  SQI Output Read          3   1   >=1   */
#define SST26_SQIOR     0xeb /* 1  SQI I/O Read             3   3   >=1   */
#define SST26_SDOR      0x3b /* 1  SDI Output Read          3   1   >=1   */
#define SST26_SDIOR     0xbb /* 1  SDI I/O Read             3   1   >=1   */
#define SST26_SB        0xc0 /* 14 Set Burst Length         0   0   1     */
#define SST26_RBSQI     0x0c /*  4 SQI Read Burst w/ Wrap   3   3   >=1   */
#define SST26_RBSPI     0xec /* 1  SPI Read Burst w/ Wrap   3   3   >=1   */
#define SST26_RDID      0x9f /* 1  Read Identification      0   0   >=3   */
#define SST26_QRDID     0xaf /*  4 Quad Read Identification 0   1   >=3   */
#define SST26_SFDP      0x5a /* 1  Serial Flash Discov. Par.3   1   >=1   */
#define SST26_WREN      0x06 /* 14 Write Enable             0   0   0     */
#define SST26_WRDI      0x04 /* 14 Write Disable            0   0   0     */
#define SST26_SE        0x20 /* 14 Sector Erase             3   0   0     */
#define SST26_BE        0xd8 /* 14 8/32/64K Block Erase     3   0   0     */
#define SST26_CE        0xc7 /* 14 Chip Erase               0   0   0     */
#define SST26_PP        0x02 /* 1  Page Program             3   0   1-256 */
#define SST26_QPP       0x32 /* 1  Quad Page Program        3   0   1-256 */
#define SST26_WRSU      0xb0 /* 14 Suspend Program/Erase    0   0   0     */
#define SST26_WRRE      0x30 /* 14 Resume Program/Erase     0   0   0     */
#define SST26_RBPR      0x72 /* 1  Read Block-Protection reg       0   0   1-18  */
                             /*  4 Read Block-Protection reg       0   1   1-18  */
#define SST26_WBPR      0x42 /* 14 Write Block-Protection reg      0   0   1-18  */
#define SST26_LBPR      0x8d /* 14 Lock down Block-Prot. reg       0   0   0     */
#define SST26_NVWLDR    0xe8 /* 14 non-Volatile Write L-D reg      0   0   1-18  */
#define SST26_ULBPR     0x98 /* 14 Global Block Protection unlock  0   0   0     */
#define SST26_RSID      0x88 /* 14 Read Security ID                2   1   1-2048 */
                             /*  4 Read Security ID                2   3   1-2048 */
#define SST26_PSID      0xa5 /* 14 Program User Security ID area   2   0   1-256 */
#define SST26_LSID      0x85 /* 14 Lockout Security ID programming 0   0   0     */

/* NOTE 1: All parts.
 * NOTE 2: In SST26VF064 terminology, 0xd8 is block erase and 0x20
 *         is a sector erase.  Block erase provides a faster way to erase
 *         multiple 4K sectors at once.
 */

/* Status register bit definitions */

#define SST26_SR_WIP              (1 << 0)    /* Bit 0: Write in progress */
#define SST26_SR_WEL              (1 << 1)    /* Bit 1: Write enable latch */
#define SST26_SR_WSE              (1 << 2)    /* Bit 2: Write Suspend-Erase Status */
#define SST26_SR_WSP              (1 << 3)    /* Bit 3: Write Suspend-Program Status */
#define SST26_SR_WPLD             (1 << 4)    /* Bit 4: Write Protection Lock-Down Status */
#define SST26_SR_SEC              (1 << 5)    /* Bit 5: Security ID status */
#define SST26_SR_RES              (1 << 6)    /* Bit 6: RFU */
#define SST26_SR_WIP2             (1 << 7)    /* Bit 7: Write in progress */

#define SST26_DUMMY     0xa5

/* Debug ********************************************************************/

#ifdef CONFIG_SST26_DEBUG
# define ssterr(format, ...)    _err(format, ##__VA_ARGS__)
# define sstinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
# define ssterr(x...)
# define sstinfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct sst26_dev_s.
 */

struct sst26_dev_s
{
  struct mtd_dev_s mtd;      /* MTD interface */
  FAR struct spi_dev_s *dev; /* Saved SPI interface instance */
  uint32_t npages;
  uint16_t nsectors;
  uint16_t devid;            /* SPI device ID to manage CS lines in board */
  uint8_t  sectorshift;
  uint8_t  pageshift;
  bool     lastwaswrite;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static void sst26_lock(FAR struct spi_dev_s *dev);
static inline void sst26_unlock(FAR struct spi_dev_s *dev);
static inline int sst26_readid(struct sst26_dev_s *priv);
static void sst26_waitwritecomplete(struct sst26_dev_s *priv);
static void sst26_writeenable(struct sst26_dev_s *priv);
static void sst26_writedisable(struct sst26_dev_s *priv);
static void sst26_globalunlock(struct sst26_dev_s *priv);
static inline void sst26_sectorerase(struct sst26_dev_s *priv,
                                     off_t offset,
                                     uint8_t type);
static inline int  sst26_chiperase(struct sst26_dev_s *priv);
static inline void sst26_pagewrite(struct sst26_dev_s *priv,
                                   FAR const uint8_t *buffer,
                                   off_t offset);

/* MTD driver methods */

static int sst26_erase(FAR struct mtd_dev_s *dev,
                       off_t startblock,
                       size_t nblocks);
static ssize_t sst26_bread(FAR struct mtd_dev_s *dev,
                           off_t startblock,
                           size_t nblocks,
                           FAR uint8_t *buf);
static ssize_t sst26_bwrite(FAR struct mtd_dev_s *dev,
                            off_t startblock,
                            size_t nblocks,
                            FAR const uint8_t *buf);
static ssize_t sst26_read(FAR struct mtd_dev_s *dev,
                          off_t offset,
                          size_t nbytes,
                          FAR uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t sst26_write(FAR struct mtd_dev_s *dev,
                           off_t offset,
                           size_t nbytes,
                           FAR const uint8_t *buffer);
#endif
static int sst26_ioctl(FAR struct mtd_dev_s *dev,
                       int cmd,
                       unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sst26_lock
 ****************************************************************************/

static void sst26_lock(FAR struct spi_dev_s *dev)
{
  /* On SPI buses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the buses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusive
   * access to the SPI bus.  We will retain that exclusive access until the
   * bus is unlocked.
   */

  SPI_LOCK(dev, true);

  /* After locking the SPI bus, then we also need to call the setfrequency,
   * setbits, and setmode methods to make sure that the SPI is properly
   * configured for the device.  If the SPI bus is being shared, then it
   * may have been left in an incompatible state.
   */

  SPI_SETMODE(dev, CONFIG_SST26_SPIMODE);
  SPI_SETBITS(dev, 8);
  SPI_HWFEATURES(dev, 0);
  SPI_SETFREQUENCY(dev, CONFIG_SST26_SPIFREQUENCY);
}

/****************************************************************************
 * Name: sst26_unlock
 ****************************************************************************/

static inline void sst26_unlock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, false);
}

/****************************************************************************
 * Name: sst26_readid
 ****************************************************************************/

static inline int sst26_readid(struct sst26_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t memory;
  uint16_t capacity;

  sstinfo("priv: %p\n", priv);

  /* Lock the SPI bus, configure the bus, and select this FLASH part. */

  sst26_lock(priv->dev);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), true);

  /* Send the "Read ID (RDID)" command and read the first three ID bytes */

  SPI_SEND(priv->dev, SST26_RDID);
  manufacturer = SPI_SEND(priv->dev, SST26_DUMMY);
  memory       = SPI_SEND(priv->dev, SST26_DUMMY);
  capacity     = SPI_SEND(priv->dev, SST26_DUMMY);

  /* De-select the FLASH and unlock the bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), false);
  sst26_unlock(priv->dev);

  sstinfo("manufacturer: %02x memory: %02x capacity: %02x\n",
          manufacturer, memory, capacity);

  /* Check for a valid manufacturer and memory type */

  if (manufacturer == SST26_MANUFACTURER && memory == SST26_MEMORY_TYPE)
    {
      /* Okay.. is it a FLASH capacity that we understand? */

      if (capacity == SST26_SST26VF064_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift = SST26_SST26VF064_SECTOR_SHIFT;
          priv->nsectors    = SST26_SST26VF064_NSECTORS;
          priv->pageshift   = SST26_SST26VF064_PAGE_SHIFT;
          priv->npages      = SST26_SST26VF064_NPAGES;
          return OK;
        }
      else if (capacity == SST26_SST26VF032_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift = SST26_SST26VF032_SECTOR_SHIFT;
          priv->nsectors    = SST26_SST26VF032_NSECTORS;
          priv->pageshift   = SST26_SST26VF032_PAGE_SHIFT;
          priv->npages      = SST26_SST26VF032_NPAGES;
          return OK;
        }
      else if (capacity == SST26_SST26VF016_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift = SST26_SST26VF016_SECTOR_SHIFT;
          priv->nsectors    = SST26_SST26VF016_NSECTORS;
          priv->pageshift   = SST26_SST26VF016_PAGE_SHIFT;
          priv->npages      = SST26_SST26VF016_NPAGES;
          return OK;
        }
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: sst26_waitwritecomplete
 ****************************************************************************/

static void sst26_waitwritecomplete(struct sst26_dev_s *priv)
{
  uint8_t status;

  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Select this FLASH part */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), true);

      /* Send "Read Status Register (RDSR)" command */

      SPI_SEND(priv->dev, SST26_RDSR);

      /* Send a dummy byte to generate the clock needed to shift out the
       * status
       */

      status = SPI_SEND(priv->dev, SST26_DUMMY);

      /* Deselect the FLASH */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), false);

      /* Given that writing could take up to few tens of milliseconds,
       * and erasing could take more.  The following short delay in the
       * "busy" case will allow other peripherals to access the SPI bus.
       */

      if ((status & SST26_SR_WIP) != 0)
        {
          sst26_unlock(priv->dev);
          nxsig_usleep(1000);
          sst26_lock(priv->dev);
        }
    }
  while ((status & SST26_SR_WIP) != 0);

  sstinfo("Complete\n");
}

/****************************************************************************
 * Name:  sst26_globalunlock
 * Description: SST26 flashes are globally locked after startup.
 * To allow writing, this command must be sent once.
 ****************************************************************************/

static void sst26_globalunlock(struct sst26_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), true);

  /* Send "Global Unlock (ULBPR)" command */

  SPI_SEND(priv->dev, SST26_ULBPR);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), false);

  sstinfo("Device unlocked.\n");
}

/****************************************************************************
 * Name:  sst26_writeenable
 ****************************************************************************/

static void sst26_writeenable(struct sst26_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), true);

  /* Send "Write Enable (WREN)" command */

  SPI_SEND(priv->dev, SST26_WREN);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), false);

  sstinfo("Enabled\n");
}

/****************************************************************************
 * Name:  sst26_writedisable
 ****************************************************************************/

static void sst26_writedisable(struct sst26_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), true);

  /* Send "Write Disable (WRDI)" command */

  SPI_SEND(priv->dev, SST26_WRDI);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), false);

  sstinfo("Disabled\n");
}

/****************************************************************************
 * Name:  sst26_sectorerase (4k)
 ****************************************************************************/

static void sst26_sectorerase(struct sst26_dev_s *priv,
                              off_t sector,
                              uint8_t type)
{
  off_t offset;

  offset = sector << priv->sectorshift;

  sstinfo("sector: %08lx\n", (long)sector);

  /* Send write enable instruction */

  sst26_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), true);

  /* Send the "Sector Erase (SE)" or "Block Erase (BE)" instruction
   * that was passed in as the erase type.
   */

  SPI_SEND(priv->dev, type);

  /* Send the sector offset high byte first.  For all of the supported
   * parts, the sector number is completely contained in the first byte
   * and the values used in the following two bytes don't really matter.
   */

  SPI_SEND(priv->dev, (offset >> 16) & 0xff);
  SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  SPI_SEND(priv->dev, offset & 0xff);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), false);

  sst26_waitwritecomplete(priv);

  sstinfo("Erased\n");
}

/****************************************************************************
 * Name:  sst26_chiperase
 ****************************************************************************/

static inline int sst26_chiperase(struct sst26_dev_s *priv)
{
  sstinfo("priv: %p\n", priv);

  /* Send write enable instruction */

  sst26_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), true);

  /* Send the "Chip Erase (CE)" instruction */

  SPI_SEND(priv->dev, SST26_CE);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), false);

  sst26_waitwritecomplete(priv);

  sstinfo("Return: OK\n");
  return OK;
}

/****************************************************************************
 * Name:  sst26_pagewrite
 ****************************************************************************/

static inline void sst26_pagewrite(struct sst26_dev_s *priv,
                                   FAR const uint8_t *buffer, off_t page)
{
  off_t offset = page << priv->pageshift;

  sstinfo("page: %08lx offset: %08lx\n", (long)page, (long)offset);

  /* Enable the write access to the FLASH */

  sst26_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), true);

  /* Send "Page Program (PP)" command */

  SPI_SEND(priv->dev, SST26_PP);

  /* Send the page offset high byte first. */

  SPI_SEND(priv->dev, (offset >> 16) & 0xff);
  SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  SPI_SEND(priv->dev, offset & 0xff);

  /* Then write the specified number of bytes */

  SPI_SNDBLOCK(priv->dev, buffer, 1 << priv->pageshift);

  /* Deselect the FLASH: Chip Select high */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), false);

  sst26_waitwritecomplete(priv);

  sstinfo("Written\n");
}

/****************************************************************************
 * Name:  sst26_bytewrite
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static inline void sst26_bytewrite(struct sst26_dev_s *priv,
                                   FAR const uint8_t *buffer, off_t offset,
                                   uint16_t count)
{
  sstinfo("offset: %08lx  count:%d\n", (long)offset, count);

  /* Enable the write access to the FLASH */

  sst26_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), true);

  /* Send "Page Program (PP)" command */

  SPI_SEND(priv->dev, SST26_PP);

  /* Send the page offset high byte first. */

  SPI_SEND(priv->dev, (offset >> 16) & 0xff);
  SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  SPI_SEND(priv->dev, offset & 0xff);

  /* Then write the specified number of bytes */

  SPI_SNDBLOCK(priv->dev, buffer, count);
  priv->lastwaswrite = true;

  /* Deselect the FLASH: Chip Select high */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), false);

  sst26_waitwritecomplete(priv);

  sstinfo("Written\n");
}
#endif

/* Driver routines */

/****************************************************************************
 * Name: sst26_erase
 ****************************************************************************/

static int sst26_erase(FAR struct mtd_dev_s *dev,
                       off_t startblock,
                       size_t nblocks)
{
  FAR struct sst26_dev_s *priv = (FAR struct sst26_dev_s *)dev;
  size_t blocksleft = nblocks;

  sstinfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  sst26_lock(priv->dev);
  while (blocksleft > 0)
    {
      /* SST26VF parts have complex block overlay structure for the moment
       * we just erase in 4k blocks.
       */

      sst26_sectorerase(priv, startblock, SST26_SE);
      startblock++;
      blocksleft--;
    }

  sst26_unlock(priv->dev);
  return (int)nblocks;
}

/****************************************************************************
 * Name: sst26_bread
 ****************************************************************************/

static ssize_t sst26_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR uint8_t *buffer)
{
  FAR struct sst26_dev_s *priv = (FAR struct sst26_dev_s *)dev;
  ssize_t nbytes;

  sstinfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented
   * read
   */

  nbytes = sst26_read(dev,
                      startblock << priv->pageshift,
                      nblocks << priv->pageshift,
                      buffer);
  if (nbytes > 0)
    {
      return nbytes >> priv->pageshift;
    }

  return (int)nbytes;
}

/****************************************************************************
 * Name: sst26_bwrite
 ****************************************************************************/

static ssize_t sst26_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct sst26_dev_s *priv = (FAR struct sst26_dev_s *)dev;
  size_t blocksleft = nblocks;
  size_t pagesize = 1 << priv->pageshift;

  sstinfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the SPI bus and write each page to FLASH */

  sst26_lock(priv->dev);
  while (blocksleft-- > 0)
    {
      sst26_pagewrite(priv, buffer, startblock);
      buffer += pagesize;
      startblock++;
    }

  sst26_unlock(priv->dev);
  return nblocks;
}

/****************************************************************************
 * Name: sst26_read
 ****************************************************************************/

static ssize_t sst26_read(FAR struct mtd_dev_s *dev,
                          off_t offset,
                          size_t nbytes,
                          FAR uint8_t *buffer)
{
  FAR struct sst26_dev_s *priv = (FAR struct sst26_dev_s *)dev;

  sstinfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the SPI bus and select this FLASH part */

  sst26_lock(priv->dev);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), true);

  /* Send "Read from Memory " instruction */

  SPI_SEND(priv->dev, SST26_FAST_READ);

  /* Send the page offset high byte first. */

  SPI_SEND(priv->dev, (offset >> 16) & 0xff);
  SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  SPI_SEND(priv->dev, offset & 0xff);

  /* Dummy read */

  SPI_SEND(priv->dev, SST26_DUMMY);

  /* Then read all of the requested bytes */

  SPI_RECVBLOCK(priv->dev, buffer, nbytes);

  /* Deselect the FLASH and unlock the SPI bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->devid), false);
  sst26_unlock(priv->dev);
  sstinfo("return nbytes: %d\n", (int)nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: sst26_write
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t sst26_write(FAR struct mtd_dev_s *dev,
                           off_t offset,
                           size_t nbytes,
                           FAR const uint8_t *buffer)
{
  FAR struct sst26_dev_s *priv = (FAR struct sst26_dev_s *)dev;
  int    startpage;
  int    endpage;
  int    count;
  int    index;
  int    pagesize;
  int    bytestowrite;

  sstinfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* We must test if the offset + count crosses one or more pages
   * and perform individual writes.  The devices can only write in
   * page increments.
   */

  startpage = offset / (1 << priv->pageshift);
  endpage = (offset + nbytes) / (1 << priv->pageshift);

  sst26_lock(priv->dev);
  if (startpage == endpage)
    {
      /* All bytes within one programmable page.  Just do the write. */

      sst26_bytewrite(priv, buffer, offset, nbytes);
    }
  else
    {
      /* Write the 1st partial-page */

      count = nbytes;
      pagesize = (1 << priv->pageshift);
      bytestowrite = pagesize - (offset & (pagesize - 1));
      sst26_bytewrite(priv, buffer, offset, bytestowrite);

      /* Update offset and count */

      offset += bytestowrite;
      count -=  bytestowrite;
      index = bytestowrite;

      /* Write full pages */

      while (count >= pagesize)
        {
          sst26_bytewrite(priv, &buffer[index], offset, pagesize);

          /* Update offset and count */

          offset += pagesize;
          count -= pagesize;
          index += pagesize;
        }

      /* Now write any partial page at the end */

      if (count > 0)
        {
          sst26_bytewrite(priv, &buffer[index], offset, count);
        }

      priv->lastwaswrite = true;
    }

  sst26_unlock(priv->dev);
  return nbytes;
}
#endif /* CONFIG_MTD_BYTE_WRITE */

/****************************************************************************
 * Name: sst26_ioctl
 ****************************************************************************/

static int sst26_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct sst26_dev_s *priv = (FAR struct sst26_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  sstinfo("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo != NULL)
            {
              /* Populate the geometry structure with information need to
               * know  the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but
               * the client will expect the device logic to do whatever is
               * necessary to make it appear so.
               */

              geo->blocksize = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;

              ret = OK;

              sstinfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
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
        {
            /* Erase the entire device */

            sst26_lock(priv->dev);
            ret = sst26_chiperase(priv);
            sst26_unlock(priv->dev);
        }
        break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  sstinfo("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sst26_initialize
 *
 * Description:
 *   Create an initialize MTD device instance. MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *sst26_initialize_spi(FAR struct spi_dev_s *dev,
                                           uint16_t spi_devid)
{
  FAR struct sst26_dev_s *priv;
  int ret;

  sstinfo("dev: %p\n", dev);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_FLASH(0) definition) and so would
   * have to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct sst26_dev_s *)kmm_zalloc(sizeof(struct sst26_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure. (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = sst26_erase;
      priv->mtd.bread  = sst26_bread;
      priv->mtd.bwrite = sst26_bwrite;
      priv->mtd.read   = sst26_read;
#ifdef CONFIG_MTD_BYTE_WRITE
      priv->mtd.write  = sst26_write;
#endif
      priv->mtd.ioctl  = sst26_ioctl;
      priv->mtd.name   = "sst26";
      priv->dev        = dev;
      priv->devid      = spi_devid;

      /* Deselect the FLASH */

      SPI_SELECT(dev, SPIDEV_FLASH(priv->devid), false);

      /* Identify the FLASH chip and get its capacity */

      ret = sst26_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and
           * return NULL
           */

          ssterr("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }
      else
        {
          /* Make sure that the FLASH is unprotected so that we can
           * write into it
           */

          sst26_writeenable(priv);
          sst26_globalunlock(priv);
          sst26_writedisable(priv);
        }
    }

  /* Return the implementation-specific state structure as the MTD device */

  sstinfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
