/************************************************************************************
 * drivers/mtd/sst25xx.c
 * Driver for SPI-based SST25VF parts 64MBit and larger.
 *
 * For smaller SST25 parts, use the sst25.c driver instead as support
 * a different program mechanism (byte or word writing vs page writing
 * supported in this driver).
 *
 *   Copyright (C) 2009-2011, 2013 Gregory Nutt. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
 *
 *   Copied from / based on m25px.c and sst25.c drivers written by
 *   Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

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

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Per the data sheet, SST25 parts can be driven with either SPI mode 0 (CPOL=0 and
 * CPHA=0) or mode 3 (CPOL=1 and CPHA=1).  So you may need to specify
 * CONFIG_SST25XX_SPIMODE to select the best mode for your device.  If
 * CONFIG_SST25XX_SPIMODE is not defined, mode 0 will be used.
 */

#ifndef CONFIG_SST25XX_SPIMODE
#  define CONFIG_SST25XX_SPIMODE SPIDEV_MODE0
#endif

/* SPI Frequency.  May be up to 25MHz. */

#ifndef CONFIG_SST25XX_SPIFREQUENCY
#  define CONFIG_SST25XX_SPIFREQUENCY 20000000
#endif

/* Various manufacturers may have produced the parts.  0xBF is the manufacturer ID
 * for the SST serial FLASH.
 */

#ifndef CONFIG_SST25XX_MANUFACTURER
#  define CONFIG_SST25XX_MANUFACTURER 0xBF
#endif

#ifndef CONFIG_SST25XX_MEMORY_TYPE
#  define CONFIG_SST25XX_MEMORY_TYPE  0x25
#endif

/* SST25 Registers *******************************************************************/
/* Indentification register values */

#define SST25_MANUFACTURER         CONFIG_SST25XX_MANUFACTURER
#define SST25_MEMORY_TYPE          CONFIG_SST25XX_MEMORY_TYPE

#define SST25_SST25064_CAPACITY    0x4b /* 64 M-bit */

/*  SST25064 capacity is 8,388,608 bytes:
 *  (2,0548 sectors) * (4,096 bytes per sector)
 *  (32,768 pages) * (256 bytes per page)
 */

#define SST25_SST25064_SECTOR_SHIFT  12    /* Sector size 1 << 15 = 65,536 */
#define SST25_SST25064_NSECTORS      2048
#define SST25_SST25064_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
#define SST25_SST25064_NPAGES        32768

/* Instructions */
/*      Command        Value      N Description             Addr Dummy  Data   */
#define SST25_WREN      0x06    /* 1 Write Enable              0   0     0     */
#define SST25_WRDI      0x04    /* 1 Write Disable             0   0     0     */
#define SST25_RDID      0x9f    /* 1 Read Identification       0   0     1-3   */
#define SST25_RDSR      0x05    /* 1 Read Status Register      0   0     >=1   */
#define SST25_EWSR      0x50    /* 1 Write enable status       0   0     0     */
#define SST25_WRSR      0x01    /* 1 Write Status Register     0   0     1     */
#define SST25_READ      0x03    /* 1 Read Data Bytes           3   0     >=1   */
#define SST25_FAST_READ 0x0b    /* 1 Higher speed read         3   1     >=1   */
#define SST25_PP        0x02    /* 1 Page Program              3   0     1-256 */
#define SST25_SE        0x20    /* 1 Sector Erase              3   0     0     */
#define SST25_BE32      0x52    /* 2 32K Block Erase           3   0     0     */
#define SST25_BE64      0xD8    /* 2 64K Block Erase           3   0     0     */
#define SST25_BE        0xc7    /* 1 Bulk Erase                0   0     0     */
#define SST25_RES       0xab    /* 1 Read Electronic Signature 0   3     >=1   */

/* NOTE 1: All parts.
 * NOTE 2: In SST25064 terminology, 0x52 and 0xd8 are block erase and 0x20
 *         is a sector erase.  Block erase provides a faster way to erase
 *         multiple 4K sectors at once.
 */

/* Status register bit definitions */

#define SST25_SR_WIP              (1 << 0)                /* Bit 0: Write in progress bit */
#define SST25_SR_WEL              (1 << 1)                /* Bit 1: Write enable latch bit */
#define SST25_SR_BP_SHIFT         (2)                     /* Bits 2-5: Block protect bits */
#define SST25_SR_BP_MASK          (15 << SST25_SR_BP_SHIFT)
#  define SST25_SR_BP_NONE        (0 << SST25_SR_BP_SHIFT) /* Unprotected */
#  define SST25_SR_BP_UPPER128th  (1 << SST25_SR_BP_SHIFT) /* Upper 128th */
#  define SST25_SR_BP_UPPER64th   (2 << SST25_SR_BP_SHIFT) /* Upper 64th */
#  define SST25_SR_BP_UPPER32nd   (3 << SST25_SR_BP_SHIFT) /* Upper 32nd */
#  define SST25_SR_BP_UPPER16th   (4 << SST25_SR_BP_SHIFT) /* Upper 16th */
#  define SST25_SR_BP_UPPER8th    (5 << SST25_SR_BP_SHIFT) /* Upper 8th */
#  define SST25_SR_BP_UPPERQTR    (6 << SST25_SR_BP_SHIFT) /* Upper quarter */
#  define SST25_SR_BP_UPPERHALF   (7 << SST25_SR_BP_SHIFT) /* Upper half */
#  define SST25_SR_BP_ALL         (8 << SST25_SR_BP_SHIFT) /* All sectors */
#define SST_SR_SEC                (1 << 6)                /* Security ID status */
#define SST25_SR_SRWD             (1 << 7)                /* Bit 7: Status register write protect */

#define SST25_DUMMY     0xa5

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct sst25xx_dev_s.
 */

struct sst25xx_dev_s
{
  struct mtd_dev_s mtd;      /* MTD interface */
  FAR struct spi_dev_s *dev; /* Saved SPI interface instance */
  uint8_t  sectorshift;      /* 16 or 18 */
  uint8_t  pageshift;        /* 8 */
  uint16_t nsectors;         /* 128 or 64 */
  uint32_t npages;           /* 32,768 or 65,536 */
  uint8_t  lastwaswrite;     /* Indicates if last operation was write */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */

static void sst25xx_lock(FAR struct spi_dev_s *dev);
static inline void sst25xx_unlock(FAR struct spi_dev_s *dev);
static inline int sst25xx_readid(struct sst25xx_dev_s *priv);
static void sst25xx_waitwritecomplete(struct sst25xx_dev_s *priv);
static void sst25xx_writeenable(struct sst25xx_dev_s *priv);
static inline void sst25xx_sectorerase(struct sst25xx_dev_s *priv, off_t offset, uint8_t type);
static inline int  sst25xx_bulkerase(struct sst25xx_dev_s *priv);
static inline void sst25xx_pagewrite(struct sst25xx_dev_s *priv, FAR const uint8_t *buffer,
                                  off_t offset);

/* MTD driver methods */

static int sst25xx_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t sst25xx_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buf);
static ssize_t sst25xx_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buf);
static ssize_t sst25xx_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t sst25xx_write(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR const uint8_t *buffer);
#endif
static int sst25xx_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sst25xx_lock
 ************************************************************************************/

static void sst25xx_lock(FAR struct spi_dev_s *dev)
{
  /* On SPI busses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the busses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusiv access to
   * the SPI buss.  We will retain that exclusive access until the bus is unlocked.
   */

  (void)SPI_LOCK(dev, true);

  /* After locking the SPI bus, the we also need call the setfrequency, setbits, and
   * setmode methods to make sure that the SPI is properly configured for the device.
   * If the SPI buss is being shared, then it may have been left in an incompatible
   * state.
   */

  SPI_SETMODE(dev, CONFIG_SST25XX_SPIMODE);
  SPI_SETBITS(dev, 8);
  (void)SPI_HWFEATURES(dev, 0);
  (void)SPI_SETFREQUENCY(dev, CONFIG_SST25XX_SPIFREQUENCY);
}

/************************************************************************************
 * Name: sst25xx_unlock
 ************************************************************************************/

static inline void sst25xx_unlock(FAR struct spi_dev_s *dev)
{
  (void)SPI_LOCK(dev, false);
}

/************************************************************************************
 * Name: sst25xx_readid
 ************************************************************************************/

static inline int sst25xx_readid(struct sst25xx_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t memory;
  uint16_t capacity;

  finfo("priv: %p\n", priv);

  /* Lock the SPI bus, configure the bus, and select this FLASH part. */

  sst25xx_lock(priv->dev);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send the "Read ID (RDID)" command and read the first three ID bytes */

  (void)SPI_SEND(priv->dev, SST25_RDID);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
  manufacturer = SPI_SEND(priv->dev, SST25_DUMMY);
  memory       = SPI_SEND(priv->dev, SST25_DUMMY);
  capacity     = SPI_SEND(priv->dev, SST25_DUMMY);

  /* Deselect the FLASH and unlock the bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
  sst25xx_unlock(priv->dev);

  finfo("manufacturer: %02x memory: %02x capacity: %02x\n",
        manufacturer, memory, capacity);

  /* Check for a valid manufacturer and memory type */

  if (manufacturer == SST25_MANUFACTURER && memory == SST25_MEMORY_TYPE)
    {
      /* Okay.. is it a FLASH capacity that we understand? */

      if (capacity == SST25_SST25064_CAPACITY)
        {
           /* Save the FLASH geometry */

           priv->sectorshift = SST25_SST25064_SECTOR_SHIFT;
           priv->nsectors    = SST25_SST25064_NSECTORS;
           priv->pageshift   = SST25_SST25064_PAGE_SHIFT;
           priv->npages      = SST25_SST25064_NPAGES;
           return OK;
        }
    }

  return -ENODEV;
}

/************************************************************************************
 * Name: sst25xx_waitwritecomplete
 ************************************************************************************/

static void sst25xx_waitwritecomplete(struct sst25xx_dev_s *priv)
{
  uint8_t status;

  /* No need to check if no write / erase was done */

#if 0
  if (!priv->lastwaswrite)
    {
      return;
    }
#endif

  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Select this FLASH part */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

      /* Send "Read Status Register (RDSR)" command */

      (void)SPI_SEND(priv->dev, SST25_RDSR);

      /* Send a dummy byte to generate the clock needed to shift out the status */

      status = SPI_SEND(priv->dev, SST25_DUMMY);

      /* Deselect the FLASH */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

      /* Given that writing could take up to few tens of milliseconds, and erasing
       * could take more.  The following short delay in the "busy" case will allow
       * other peripherals to access the SPI bus.
       */

      if ((status & SST25_SR_WIP) != 0)
        {
          sst25xx_unlock(priv->dev);
          nxsig_usleep(1000);
          sst25xx_lock(priv->dev);
        }
    }
  while ((status & SST25_SR_WIP) != 0);

  priv->lastwaswrite = false;

  finfo("Complete\n");
}

/************************************************************************************
 * Name:  sst25xx_writeenable
 ************************************************************************************/

static void sst25xx_writeenable(struct sst25xx_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send "Write Enable (WREN)" command */

  (void)SPI_SEND(priv->dev, SST25_WREN);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
  finfo("Enabled\n");
}

/************************************************************************************
 * Name: sst25xx_unprotect
 ************************************************************************************/

static void sst25xx_unprotect(struct sst25xx_dev_s *priv)
{
  /* Send "Write enable status (EWSR)" */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
  (void)SPI_SEND(priv->dev, SST25_EWSR);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

  /* Send "Write status (WRSR)" */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
  SPI_SEND(priv->dev, SST25_WRSR);

  /* Followed by the new status value */

  SPI_SEND(priv->dev, 0);

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
}

/************************************************************************************
 * Name:  sst25xx_sectorerase
 ************************************************************************************/

static void sst25xx_sectorerase(struct sst25xx_dev_s *priv, off_t sector, uint8_t type)
{
  off_t offset;

  offset = sector << priv->sectorshift;

  finfo("sector: %08lx\n", (long)sector);

  /* Wait for any preceding write to complete.  We could simplify things by
   * perform this wait at the end of each write operation (rather than at
   * the beginning of ALL operations), but have the wait first will slightly
   * improve performance.
   */

  sst25xx_waitwritecomplete(priv);

  /* Send write enable instruction */

  sst25xx_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send the "Sector Erase (SE)" or Sub-Sector Erase (SSE) instruction
   * that was passed in as the erase type.
   */

  (void)SPI_SEND(priv->dev, type);

  /* Send the sector offset high byte first.  For all of the supported
   * parts, the sector number is completely contained in the first byte
   * and the values used in the following two bytes don't really matter.
   */

  (void)SPI_SEND(priv->dev, (offset >> 16) & 0xff);
  (void)SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  (void)SPI_SEND(priv->dev, offset & 0xff);
  priv->lastwaswrite = true;

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
  finfo("Erased\n");
}

/************************************************************************************
 * Name:  sst25xx_bulkerase
 ************************************************************************************/

static inline int sst25xx_bulkerase(struct sst25xx_dev_s *priv)
{
  finfo("priv: %p\n", priv);

  /* Wait for any preceding write to complete.  We could simplify things by
   * perform this wait at the end of each write operation (rather than at
   * the beginning of ALL operations), but have the wait first will slightly
   * improve performance.
   */

  sst25xx_waitwritecomplete(priv);

  /* Send write enable instruction */

  sst25xx_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send the "Bulk Erase (BE)" instruction */

  (void)SPI_SEND(priv->dev, SST25_BE);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
  sst25xx_waitwritecomplete(priv);

  finfo("Return: OK\n");
  return OK;
}

/************************************************************************************
 * Name:  sst25xx_pagewrite
 ************************************************************************************/

static inline void sst25xx_pagewrite(struct sst25xx_dev_s *priv, FAR const uint8_t *buffer,
                                  off_t page)
{
  off_t offset = page << priv->pageshift;

  finfo("page: %08lx offset: %08lx\n", (long)page, (long)offset);

  /* Wait for any preceding write to complete.  We could simplify things by
   * perform this wait at the end of each write operation (rather than at
   * the beginning of ALL operations), but have the wait first will slightly
   * improve performance.
   */

  sst25xx_waitwritecomplete(priv);

  /* Enable the write access to the FLASH */

  sst25xx_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send "Page Program (PP)" command */

  (void)SPI_SEND(priv->dev, SST25_PP);

  /* Send the page offset high byte first. */

  (void)SPI_SEND(priv->dev, (offset >> 16) & 0xff);
  (void)SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  (void)SPI_SEND(priv->dev, offset & 0xff);

  /* Then write the specified number of bytes */

  SPI_SNDBLOCK(priv->dev, buffer, 1 << priv->pageshift);
  priv->lastwaswrite = true;

  /* Deselect the FLASH: Chip Select high */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
  finfo("Written\n");
}

/************************************************************************************
 * Name:  sst25xx_bytewrite
 ************************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static inline void sst25xx_bytewrite(struct sst25xx_dev_s *priv,
                                     FAR const uint8_t *buffer, off_t offset,
                                     uint16_t count)
{
  finfo("offset: %08lx  count:%d\n", (long)offset, count);

  /* Wait for any preceding write to complete.  We could simplify things by
   * perform this wait at the end of each write operation (rather than at
   * the beginning of ALL operations), but have the wait first will slightly
   * improve performance.
   */

  sst25xx_waitwritecomplete(priv);

  /* Enable the write access to the FLASH */

  sst25xx_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send "Page Program (PP)" command */

  (void)SPI_SEND(priv->dev, SST25_PP);

  /* Send the page offset high byte first. */

  (void)SPI_SEND(priv->dev, (offset >> 16) & 0xff);
  (void)SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  (void)SPI_SEND(priv->dev, offset & 0xff);

  /* Then write the specified number of bytes */

  SPI_SNDBLOCK(priv->dev, buffer, count);
  priv->lastwaswrite = true;

  /* Deselect the FLASH: Chip Select high */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
  finfo("Written\n");
}
#endif

/************************************************************************************
 * Name: sst25xx_erase
 ************************************************************************************/

static int sst25xx_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
  FAR struct sst25xx_dev_s *priv = (FAR struct sst25xx_dev_s *)dev;
  size_t blocksleft = nblocks;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  sst25xx_lock(priv->dev);
  while (blocksleft > 0)
    {
      size_t sectorboundry;
      size_t blkper;

      /* We will erase in either 4K sectors or 32K or 64K blocks depending
       * on the largest unit we can use given the startblock and nblocks.
       * This will reduce erase time (in the event we have partitions
       * enabled and are doing a bulk erase which is translated into
       * a block erase operation).
       */

      /* Test for 64K alignment */

      blkper = 64 / 4;
      sectorboundry = (startblock + blkper - 1) / blkper;
      sectorboundry *= blkper;

      /* If we are on a sector boundry and have at least a full sector
       * of blocks left to erase, then we can do a full sector erase.
       */

      if (startblock == sectorboundry && blocksleft >= blkper)
        {
          /* Do a 64k block erase */

          sst25xx_sectorerase(priv, startblock, SST25_BE64);
          startblock += blkper;
          blocksleft -= blkper;
          continue;
        }

      /* Test for 32K block alignment */

      blkper = 32 / 4;
      sectorboundry = (startblock + blkper - 1) / blkper;
      sectorboundry *= blkper;

      if (startblock == sectorboundry && blocksleft >= blkper)
        {
          /* Do a 32k block erase */

          sst25xx_sectorerase(priv, startblock, SST25_BE32);
          startblock += blkper;
          blocksleft -= blkper;
          continue;
        }
      else
        {
          /* Just do a sector erase */

          sst25xx_sectorerase(priv, startblock, SST25_SE);
          startblock++;
          blocksleft--;
          continue;
        }
    }

  sst25xx_unlock(priv->dev);
  return (int)nblocks;
}

/************************************************************************************
 * Name: sst25xx_bread
 ************************************************************************************/

static ssize_t sst25xx_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                             FAR uint8_t *buffer)
{
  FAR struct sst25xx_dev_s *priv = (FAR struct sst25xx_dev_s *)dev;
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented read */

  nbytes = sst25xx_read(dev, startblock << priv->pageshift,
                        nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
        return nbytes >> priv->pageshift;
    }

  return (int)nbytes;
}

/************************************************************************************
 * Name: sst25xx_bwrite
 ************************************************************************************/

static ssize_t sst25xx_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                              FAR const uint8_t *buffer)
{
  FAR struct sst25xx_dev_s *priv = (FAR struct sst25xx_dev_s *)dev;
  size_t blocksleft = nblocks;
  size_t pagesize = 1 << priv->pageshift;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the SPI bus and write each page to FLASH */

  sst25xx_lock(priv->dev);
  while (blocksleft-- > 0)
    {
      sst25xx_pagewrite(priv, buffer, startblock);
      buffer += pagesize;
      startblock++;
   }

  sst25xx_unlock(priv->dev);
  return nblocks;
}

/************************************************************************************
 * Name: sst25xx_read
 ************************************************************************************/

static ssize_t sst25xx_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                            FAR uint8_t *buffer)
{
  FAR struct sst25xx_dev_s *priv = (FAR struct sst25xx_dev_s *)dev;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the SPI bus NOW because the following conditional call to
   * sst25xx_waitwritecomplete must be executed with the bus locked.
   */

  sst25xx_lock(priv->dev);

  /* Wait for any preceding write to complete.  We could simplify things by
   * perform this wait at the end of each write operation (rather than at
   * the beginning of ALL operations), but have the wait first will slightly
   * improve performance.
   */

  if (priv->lastwaswrite)
    {
      sst25xx_waitwritecomplete(priv);
    }

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send "Read from Memory " instruction */

  (void)SPI_SEND(priv->dev, SST25_READ);

  /* Send the page offset high byte first. */

  (void)SPI_SEND(priv->dev, (offset >> 16) & 0xff);
  (void)SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  (void)SPI_SEND(priv->dev, offset & 0xff);

  /* Then read all of the requested bytes */

  SPI_RECVBLOCK(priv->dev, buffer, nbytes);

  /* Deselect the FLASH and unlock the SPI bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
  sst25xx_unlock(priv->dev);
  finfo("return nbytes: %d\n", (int)nbytes);
  return nbytes;
}

/************************************************************************************
 * Name: sst25xx_write
 ************************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t sst25xx_write(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR const uint8_t *buffer)
{
  FAR struct sst25xx_dev_s *priv = (FAR struct sst25xx_dev_s *)dev;
  int    startpage;
  int    endpage;
  int    count;
  int    index;
  int    pagesize;
  int    bytestowrite;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* We must test if the offset + count crosses one or more pages
   * and perform individual writes.  The devices can only write in
   * page increments.
   */

  startpage = offset / (1 << priv->pageshift);
  endpage = (offset + nbytes) / (1 << priv->pageshift);

  sst25xx_lock(priv->dev);
  if (startpage == endpage)
    {
      /* All bytes within one programmable page.  Just do the write. */

      sst25xx_bytewrite(priv, buffer, offset, nbytes);
    }
  else
    {
      /* Write the 1st partial-page */

      count = nbytes;
      pagesize = (1 << priv->pageshift);
      bytestowrite = pagesize - (offset & (pagesize-1));
      sst25xx_bytewrite(priv, buffer, offset, bytestowrite);

      /* Update offset and count */

      offset += bytestowrite;
      count -=  bytestowrite;
      index = bytestowrite;

      /* Write full pages */

      while (count >= pagesize)
        {
          sst25xx_bytewrite(priv, &buffer[index], offset, pagesize);

          /* Update offset and count */

          offset += pagesize;
          count -= pagesize;
          index += pagesize;
        }

      /* Now write any partial page at the end */

      if (count > 0)
        {
          sst25xx_bytewrite(priv, &buffer[index], offset, count);
        }

      priv->lastwaswrite = true;
    }

  sst25xx_unlock(priv->dev);
  return nbytes;
}
#endif /* CONFIG_MTD_BYTE_WRITE */

/************************************************************************************
 * Name: sst25xx_ioctl
 ************************************************************************************/

static int sst25xx_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct sst25xx_dev_s *priv = (FAR struct sst25xx_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  finfo("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              /* Populate the geometry structure with information need to know
               * the capacity and how to access the device.
               *
               * NOTE: that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the client
               * will expect the device logic to do whatever is necessary to make it
               * appear so.
               */

              geo->blocksize = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;

              ret = OK;

              finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
            /* Erase the entire device */

            sst25xx_lock(priv->dev);
            ret = sst25xx_bulkerase(priv);
            sst25xx_unlock(priv->dev);
        }
        break;

      case MTDIOC_XIPBASE:
      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sst25xx_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/

FAR struct mtd_dev_s *sst25xx_initialize(FAR struct spi_dev_s *dev)
{
  FAR struct sst25xx_dev_s *priv;
  int ret;

  finfo("dev: %p\n", dev);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_FLASH(0) definition) and so would have
   * to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct sst25xx_dev_s *)kmm_zalloc(sizeof(struct sst25xx_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure. (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = sst25xx_erase;
      priv->mtd.bread  = sst25xx_bread;
      priv->mtd.bwrite = sst25xx_bwrite;
      priv->mtd.read   = sst25xx_read;
#ifdef CONFIG_MTD_BYTE_WRITE
      priv->mtd.write  = sst25xx_write;
#endif
      priv->mtd.ioctl  = sst25xx_ioctl;
      priv->dev        = dev;
      priv->lastwaswrite = false;

      /* Deselect the FLASH */

      SPI_SELECT(dev, SPIDEV_FLASH(0), false);

      /* Identify the FLASH chip and get its capacity */

      ret = sst25xx_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and return NULL */

          ferr("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }
      else
        {
          /* Make sure that the FLASH is unprotected so that we can write into it */

          sst25xx_unprotect(priv);
        }
    }

  /* Register the MTD with the procfs system if enabled */

#ifdef CONFIG_MTD_REGISTRATION
  mtd_register(&priv->mtd, "sst25xx");
#endif

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
