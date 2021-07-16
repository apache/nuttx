/****************************************************************************
 * drivers/mtd/sst25.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Per the data sheet, the SST25 parts can be driven with either SPI mode 0
 * (CPOL=0 and CPHA=0) or mode 3 (CPOL=1 and CPHA=1).
 * But I have heard that other devices can operate in mode 0 or 1.
 * So you may need to specify CONFIG_SST25_SPIMODE to select the best mode
 * for your device.  If CONFIG_SST25_SPIMODE is not defined, mode 0 will
 * be used.
 */

#ifndef CONFIG_SST25_SPIMODE
#define CONFIG_SST25_SPIMODE SPIDEV_MODE0
#endif

/* SPI Frequency.  May be up to 25MHz. */

#ifndef CONFIG_SST25_SPIFREQUENCY
#define CONFIG_SST25_SPIFREQUENCY 20000000
#endif

/* SST25 Instructions *******************************************************/

/*    Command              Value      Description               Addr   Data */

/*                                                                 Dummy    */

#define SST25_READ          0x03    /* Read data bytes           3   0  >=1 */
#define SST25_FAST_READ     0x0b    /* Higher speed read         3   1  >=1 */
#define SST25_SE            0x20    /* 4Kb Sector erase          3   0   0  */
#define SST25_BE32          0x52    /* 32Kbit block Erase        3   0   0  */
#define SST25_BE64          0xd8    /* 64Kbit block Erase        3   0   0  */
#define SST25_CE            0xc7    /* Chip erase                0   0   0  */
#define SST25_CE_ALT        0x60    /* Chip erase (alternate)    0   0   0  */
#define SST25_BP            0x02    /* Byte program              3   0   1  */
#define SST25_AAI           0xad    /* Auto address increment    3   0  >=2 */
#define SST25_RDSR          0x05    /* Read status register      0   0  >=1 */
#define SST25_EWSR          0x50    /* Write enable status       0   0   0  */
#define SST25_WRSR          0x01    /* Write Status Register     0   0   1  */
#define SST25_WREN          0x06    /* Write Enable              0   0   0  */
#define SST25_WRDI          0x04    /* Write Disable             0   0   0  */
#define SST25_RDID          0xab    /* Read Identification       0   0  >=1 */
#define SST25_RDID_ALT      0x90    /* Read Identification (alt) 0   0  >=1 */
#define SST25_JEDEC_ID      0x9f    /* JEDEC ID read             0   0  >=3 */
#define SST25_EBSY          0x70    /* Enable SO RY/BY# status   0   0   0  */
#define SST25_DBSY          0x80    /* Disable SO RY/BY# status  0   0   0  */

/* SST25 Registers **********************************************************/

/* Read ID (RDID) register values */

#define SST25_MANUFACTURER          0xbf  /* SST manufacturer ID */
#define SST25_VF016_DEVID           0x25  /* SSTVF016B device ID */
#define SST25_VF032_DEVID           0x20  /* SSTVF032B device ID */

/* JEDEC Read ID register values */

#define SST25_JEDEC_MANUFACTURER    0xbf  /* SST manufacturer ID */
#define SST25_JEDEC_MEMORY_TYPE     0x25  /* SST25 memory type */
#define SST25_JEDEC_VF032_CAPACITY  0x4a  /* SST25VF032B memory capacity */
#define SST25_JEDEC_VF016_CAPACITY  0x41  /* SST25VF016B memory capacity */

/* Status register bit definitions */

#define SST25_SR_BUSY             (1 << 0)  /* Bit 0: Write in progress */
#define SST25_SR_WEL              (1 << 1)  /* Bit 1: Write enable latch bit */
#define SST25_SR_BP_SHIFT         (2)       /* Bits 2-5: Block protect bits */
#define SST25_SR_BP_MASK          (15 << SST25_SR_BP_SHIFT)
#define SST25_SR_BP_NONE          (0 << SST25_SR_BP_SHIFT) /* Unprotected */
#define SST25_SR_BP_UPPER64th     (1 << SST25_SR_BP_SHIFT) /* Upper 64th */
#define SST25_SR_BP_UPPER32nd     (2 << SST25_SR_BP_SHIFT) /* Upper 32nd */
#define SST25_SR_BP_UPPER16th     (3 << SST25_SR_BP_SHIFT) /* Upper 16th */
#define SST25_SR_BP_UPPER8th      (4 << SST25_SR_BP_SHIFT) /* Upper 8th */
#define SST25_SR_BP_UPPERQTR      (5 << SST25_SR_BP_SHIFT) /* Upper quarter */
#define SST25_SR_BP_UPPERHALF     (6 << SST25_SR_BP_SHIFT) /* Upper half */
#define SST25_SR_BP_ALL           (7 << SST25_SR_BP_SHIFT) /* All sectors */

#define SST25_SR_AAI              (1 << 6)  /* Bit 6: Auto Address increment programming */
#define SST25_SR_SRWD             (1 << 7)  /* Bit 7: Status register write protect */

#define SST25_DUMMY               0xa5

/* Chip Geometries **********************************************************/

/* SST25VF512 capacity is 512Kbit (64Kbit x 8)   =  64Kb (8Kb x 8) */

/* SST25VF010 capacity is 1Mbit   (128Kbit x 8)  = 128Kb (16Kb x 8 */

/* SST25VF520 capacity is 2Mbit   (256Kbit x 8)  = 256Kb (32Kb x 8) */

/* SST25VF540 capacity is 4Mbit   (512Kbit x 8)  = 512Kb (64Kb x 8) */

/* SST25VF080 capacity is 8Mbit   (1024Kbit x 8) =   1Mb (128Kb x 8) */

/* Not yet supported */

/* SST25VF016 capacity is 16Mbit  (2048Kbit x 8) =   2Mb (256Kb x 8) */

#define SST25_VF016_SECTOR_SHIFT  12         /* Sector size 1 << 12 = 4Kb */
#define SST25_VF016_NSECTORS      512        /* 512 sectors x 4096 bytes/sector = 2Mb */

/* SST25VF032 capacity is 32Mbit  (4096Kbit x 8) =   4Mb (512kb x 8) */

#define SST25_VF032_SECTOR_SHIFT  12          /* Sector size 1 << 12 = 4Kb */
#define SST25_VF032_NSECTORS      1024        /* 1024 sectors x 4096 bytes/sector = 4Mb */

#ifdef CONFIG_SST25_SECTOR512                 /* Simulate a 512 byte sector */
#define SST25_SECTOR_SHIFT      9           /* Sector size 1 << 9 = 512 bytes */
#define SST25_SECTOR_SIZE       512         /* Sector size = 512 bytes */
#endif

#define SST25_ERASED_STATE        0xff        /* State of FLASH when erased */

/* Cache flags */

#define SST25_CACHE_VALID         (1 << 0)    /* 1=Cache has valid data */
#define SST25_CACHE_DIRTY         (1 << 1)    /* 1=Cache is dirty */
#define SST25_CACHE_ERASED        (1 << 2)    /* 1=Backing FLASH is erased */

#define IS_VALID(p)                ((((p)->flags) & SST25_CACHE_VALID) != 0)
#define IS_DIRTY(p)                ((((p)->flags) & SST25_CACHE_DIRTY) != 0)
#define IS_ERASED(p)                ((((p)->flags) & SST25_CACHE_ERASED) != 0)

#define SET_VALID(p)               do { (p)->flags |= SST25_CACHE_VALID; } while (0)
#define SET_DIRTY(p)               do { (p)->flags |= SST25_CACHE_DIRTY; } while (0)
#define SET_ERASED(p)              do { (p)->flags |= SST25_CACHE_ERASED; } while (0)

#define CLR_VALID(p)               do { (p)->flags &= ~SST25_CACHE_VALID; } while (0)
#define CLR_DIRTY(p)               do { (p)->flags &= ~SST25_CACHE_DIRTY; } while (0)
#define CLR_ERASED(p)              do { (p)->flags &= ~SST25_CACHE_ERASED; } while (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.
 * The struct mtd_dev_s must appear at the beginning of the definition so
 * that you can freely cast between pointers to struct mtd_dev_s and struct
 * sst25_dev_s.
 */

struct sst25_dev_s
{
  struct mtd_dev_s      mtd;         /* MTD interface */
  FAR struct spi_dev_s *dev;         /* Saved SPI interface instance */
  uint16_t              nsectors;    /* Number of erase sectors */
  uint8_t               sectorshift; /* Log2 of erase sector size */

#if defined(CONFIG_SST25_SECTOR512) && !defined(CONFIG_SST25_READONLY)
  uint8_t               flags;       /* Buffered sector flags */
  uint16_t              esectno;     /* Erase sector number in the cache */
  FAR uint8_t          *sector;      /* Allocated sector data */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static void sst25_lock(FAR struct spi_dev_s *dev);
static inline void sst25_unlock(FAR struct spi_dev_s *dev);
static inline int sst25_readid(FAR struct sst25_dev_s *priv);
#ifndef CONFIG_SST25_READONLY
static void sst25_unprotect(FAR struct sst25_dev_s *priv);
#endif
static uint8_t sst25_waitwritecomplete(FAR struct sst25_dev_s *priv);
static inline void sst25_cmd(struct sst25_dev_s *priv, uint8_t cmd);
static inline void sst25_wren(FAR struct sst25_dev_s *priv);
#if !defined(CONFIG_SST25_SLOWWRITE) && !defined(CONFIG_SST25_READONLY)
static inline void sst25_wrdi(FAR struct sst25_dev_s *priv);
#endif
static void sst25_sectorerase(FAR struct sst25_dev_s *priv, off_t offset);
static inline int sst25_chiperase(FAR struct sst25_dev_s *priv);
static void sst25_byteread(FAR struct sst25_dev_s *priv,
                           FAR uint8_t *buffer,
                           off_t address,
                           size_t nbytes);
#ifndef CONFIG_SST25_READONLY
#ifdef CONFIG_SST25_SLOWWRITE
static void sst25_bytewrite(FAR struct sst25_dev_s *priv,
                            FAR const uint8_t *buffer,
                            off_t address,
                            size_t nbytes);
#else
static void sst25_wordwrite(FAR struct sst25_dev_s *priv,
                            FAR const uint8_t *buffer,
                            off_t address,
                            size_t nbytes);
#endif
#ifdef CONFIG_SST25_SECTOR512
static void sst25_cacheflush(struct sst25_dev_s *priv);
static FAR uint8_t *sst25_cacheread(struct sst25_dev_s *priv,
                                    off_t sector);
static void sst25_cacheerase(struct sst25_dev_s *priv,
                             off_t sector);
static void sst25_cachewrite(FAR struct sst25_dev_s *priv,
                             FAR const uint8_t *buffer,
                             off_t sector,
                             size_t nsectors);
#endif
#endif

/* MTD driver methods */

static int sst25_erase(FAR struct mtd_dev_s *dev,
                       off_t startblock,
                       size_t nblocks);
static ssize_t sst25_bread(FAR struct mtd_dev_s *dev,
                           off_t startblock,
                           size_t nblocks,
                           FAR uint8_t *buf);
static ssize_t sst25_bwrite(FAR struct mtd_dev_s *dev,
                            off_t startblock,
                            size_t nblocks,
                            FAR const uint8_t *buf);
static ssize_t sst25_read(FAR struct mtd_dev_s *dev,
                          off_t offset,
                          size_t nbytes,
                          FAR uint8_t *buffer);
static int sst25_ioctl(FAR struct mtd_dev_s *dev,
                       int cmd,
                       unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sst25_lock
 ****************************************************************************/

static void sst25_lock(FAR struct spi_dev_s *dev)
{
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

  SPI_SETMODE(dev, CONFIG_SST25_SPIMODE);
  SPI_SETBITS(dev, 8);
  SPI_HWFEATURES(dev, 0);
  SPI_SETFREQUENCY(dev, CONFIG_SST25_SPIFREQUENCY);
}

/****************************************************************************
 * Name: sst25_unlock
 ****************************************************************************/

static inline void sst25_unlock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, false);
}

/****************************************************************************
 * Name: sst25_readid
 ****************************************************************************/

static inline int sst25_readid(struct sst25_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t memory;
  uint16_t capacity;

  finfo("priv: %p\n", priv);

  /* Lock the SPI bus, configure the bus, and select this FLASH part. */

  sst25_lock(priv->dev);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send the "Read ID (RDID)" command and read the first three ID bytes */

  SPI_SEND(priv->dev, SST25_JEDEC_ID);
  manufacturer = SPI_SEND(priv->dev, SST25_DUMMY);
  memory       = SPI_SEND(priv->dev, SST25_DUMMY);
  capacity     = SPI_SEND(priv->dev, SST25_DUMMY);

  /* Deselect the FLASH and unlock the bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
  sst25_unlock(priv->dev);

  finfo("manufacturer: %02x memory: %02x capacity: %02x\n",
        manufacturer, memory, capacity);

  /* Check for a valid manufacturer and memory type */

  if (manufacturer == SST25_JEDEC_MANUFACTURER &&
      memory == SST25_JEDEC_MEMORY_TYPE)
    {
      /* Okay.. is it a FLASH capacity that we understand?  This should be
       * extended support other members of the SST25 family.  If so, save
       * the FLASH geometry.
       */

      switch (capacity)
        {
           case SST25_JEDEC_VF032_CAPACITY:
              priv->sectorshift = SST25_VF032_SECTOR_SHIFT;
              priv->nsectors    = SST25_VF032_NSECTORS;
              return OK;

           case SST25_JEDEC_VF016_CAPACITY:
              priv->sectorshift = SST25_VF016_SECTOR_SHIFT;
              priv->nsectors    = SST25_VF016_NSECTORS;
              return OK;

            /* Support for this part is not implemented yet */

            default:
              break;
        }
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: sst25_unprotect
 ****************************************************************************/

#ifndef CONFIG_SST25_READONLY
static void sst25_unprotect(struct sst25_dev_s *priv)
{
  /* Send "Write enable status (EWSR)" */

  sst25_cmd(priv, SST25_EWSR);

  /* Send "Write enable status (WRSR)" */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  SPI_SEND(priv->dev, SST25_WRSR);

  /* Followed by the new status value */

  SPI_SEND(priv->dev, 0);

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
}
#endif

/****************************************************************************
 * Name: sst25_waitwritecomplete
 ****************************************************************************/

static uint8_t sst25_waitwritecomplete(struct sst25_dev_s *priv)
{
  uint8_t status;

  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Select this FLASH part */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

      /* Send "Read Status Register (RDSR)" command */

      SPI_SEND(priv->dev, SST25_RDSR);

      /* Send a dummy byte to generate the clock needed to shift out the
       * status
       */

      status = SPI_SEND(priv->dev, SST25_DUMMY);

      /* Deselect the FLASH */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

      /* Given that writing could take up to few tens of milliseconds, and
       * erasing could take more. The following short delay in the "busy"
       * case will allow other peripherals to access the SPI bus.
       */

#if 0 /* Makes writes too slow */
      if ((status & SST25_SR_BUSY) != 0)
        {
          sst25_unlock(priv->dev);
          nxsig_usleep(1000);
          sst25_lock(priv->dev);
        }
#endif
    }
  while ((status & SST25_SR_BUSY) != 0);

  return status;
}

/****************************************************************************
 * Name:  sst25_cmd
 ****************************************************************************/

static inline void sst25_cmd(struct sst25_dev_s *priv, uint8_t cmd)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send command */

  SPI_SEND(priv->dev, cmd);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
}

/****************************************************************************
 * Name:  sst25_wren
 ****************************************************************************/

static inline void sst25_wren(struct sst25_dev_s *priv)
{
  /* Send "Write Enable (WREN)" command */

  sst25_cmd(priv, SST25_WREN);
}

/****************************************************************************
 * Name:  sst25_wrdi
 ****************************************************************************/

#if !defined(CONFIG_SST25_SLOWWRITE) && !defined(CONFIG_SST25_READONLY)
static inline void sst25_wrdi(struct sst25_dev_s *priv)
{
  /* Send "Write Disable (WRDI)" command */

  sst25_cmd(priv, SST25_WRDI);
}
#endif

/****************************************************************************
 * Name:  sst25_sectorerase
 ****************************************************************************/

static void sst25_sectorerase(struct sst25_dev_s *priv, off_t sector)
{
  off_t address = sector << priv->sectorshift;

  finfo("sector: %08lx\n", (long)sector);

  /* Wait for any preceding write or erase operation to complete. */

  sst25_waitwritecomplete(priv);

  /* Send write enable instruction */

  sst25_wren(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send the "Sector Erase (SE)" instruction */

  SPI_SEND(priv->dev, SST25_SE);

  /* Send the sector address high byte first. Only the most significant bits
   * (those corresponding to the sector) have any meaning.
   */

  SPI_SEND(priv->dev, (address >> 16) & 0xff);
  SPI_SEND(priv->dev, (address >> 8) & 0xff);
  SPI_SEND(priv->dev, address & 0xff);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
}

/****************************************************************************
 * Name:  sst25_chiperase
 ****************************************************************************/

static inline int sst25_chiperase(struct sst25_dev_s *priv)
{
  finfo("priv: %p\n", priv);

  /* Wait for any preceding write or erase operation to complete. */

  sst25_waitwritecomplete(priv);

  /* Send write enable instruction */

  sst25_wren(priv);

  /* Send the "Chip Erase (CE)" instruction */

  sst25_cmd(priv, SST25_CE);

  finfo("Return: OK\n");
  return OK;
}

/****************************************************************************
 * Name: sst25_byteread
 ****************************************************************************/

static void sst25_byteread(FAR struct sst25_dev_s *priv,
                           FAR uint8_t *buffer,
                           off_t address,
                           size_t nbytes)
{
  uint8_t status;

  finfo("address: %08lx nbytes: %d\n", (long)address, (int)nbytes);

  /* Wait for any preceding write or erase operation to complete. */

  status = sst25_waitwritecomplete(priv);
  DEBUGASSERT((status & (SST25_SR_WEL |
                         SST25_SR_BP_MASK |
                         SST25_SR_AAI)) == 0);
  UNUSED(status);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send "Read from Memory " instruction */

#ifdef CONFIG_SST25_SLOWREAD
  SPI_SEND(priv->dev, SST25_READ);
#else
  SPI_SEND(priv->dev, SST25_FAST_READ);
#endif

  /* Send the address high byte first. */

  SPI_SEND(priv->dev, (address >> 16) & 0xff);
  SPI_SEND(priv->dev, (address >> 8) & 0xff);
  SPI_SEND(priv->dev, address & 0xff);

  /* Send a dummy byte */

#ifndef CONFIG_SST25_SLOWREAD
  SPI_SEND(priv->dev, SST25_DUMMY);
#endif

  /* Then read all of the requested bytes */

  SPI_RECVBLOCK(priv->dev, buffer, nbytes);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
}

/****************************************************************************
 * Name:  sst25_bytewrite
 ****************************************************************************/

#if defined(CONFIG_SST25_SLOWWRITE) && !defined(CONFIG_SST25_READONLY)
static void sst25_bytewrite(struct sst25_dev_s *priv,
                            FAR const uint8_t *buffer,
                            off_t address,
                            size_t nbytes)
{
  uint8_t status;

  finfo("address: %08lx nwords: %d\n", (long)address, (int)nbytes);
  DEBUGASSERT(priv && buffer);

  /* Write each byte individually */

  for (; nbytes > 0; nbytes--)
    {
      /* Skip over bytes that are begin written to the erased state */

      if (*buffer != SST25_ERASED_STATE)
        {
          /* Wait for any preceding write or erase operation to complete. */

          status = sst25_waitwritecomplete(priv);
          DEBUGASSERT((status & (SST25_SR_WEL | SST25_SR_BP_MASK |
                                 SST25_SR_AAI)) == 0);

          /* Enable write access to the FLASH */

          sst25_wren(priv);

          /* Select this FLASH part */

          SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

          /* Send "Byte Program (BP)" command */

          SPI_SEND(priv->dev, SST25_BP);

          /* Send the byte address high byte first. */

          SPI_SEND(priv->dev, (address >> 16) & 0xff);
          SPI_SEND(priv->dev, (address >> 8) & 0xff);
          SPI_SEND(priv->dev, address & 0xff);

          /* Then write the single byte */

          SPI_SEND(priv->dev, *buffer);

          /* Deselect the FLASH and setup for the next pass through the
           * loop
           */

          SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
        }

      /* Advance to the next byte */

      buffer++;
      address++;
    }
}
#endif

/****************************************************************************
 * Name:  sst25_wordwrite
 ****************************************************************************/

#if !defined(CONFIG_SST25_SLOWWRITE) && !defined(CONFIG_SST25_READONLY)
static void sst25_wordwrite(struct sst25_dev_s *priv,
                            FAR const uint8_t *buffer,
                            off_t address,
                            size_t nbytes)
{
  size_t nwords = (nbytes + 1) >> 1;
  uint8_t status;

  finfo("address: %08lx nwords: %d\n", (long)address, (int)nwords);
  DEBUGASSERT(priv && buffer);

  /* Loop until all of the bytes have been written */

  while (nwords > 0)
    {
      /* Skip over any data that is being written to the erased state */

      while (nwords > 0 &&
             buffer[0] == SST25_ERASED_STATE &&
             buffer[1] == SST25_ERASED_STATE)
        {
          /* Decrement the word count and advance the write position */

          nwords--;
          buffer  += 2;
          address += 2;
        }

      /* If there are no further non-erased bytes in the user buffer, then
       * we are finished.
       */

      if (nwords < 1)
        {
          return;
        }

      /* Wait for any preceding write or erase operation to complete. */

      status = sst25_waitwritecomplete(priv);
      DEBUGASSERT((status & (SST25_SR_WEL |
                             SST25_SR_BP_MASK |
                             SST25_SR_AAI)) == 0);
      UNUSED(status);

      /* Enable write access to the FLASH */

      sst25_wren(priv);

      /* Select this FLASH part */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

      /* Send "Auto Address Increment (AAI)" command */

      SPI_SEND(priv->dev, SST25_AAI);

      /* Send the word address high byte first. */

      SPI_SEND(priv->dev, (address >> 16) & 0xff);
      SPI_SEND(priv->dev, (address >> 8) & 0xff);
      SPI_SEND(priv->dev, address & 0xff);

      /* Then write one 16-bit word */

      SPI_SNDBLOCK(priv->dev, buffer, 2);

      /* Deselect the FLASH: Chip Select high */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

      /* Wait for the preceding write to complete. */

      status = sst25_waitwritecomplete(priv);
      DEBUGASSERT((status & (SST25_SR_WEL |
                             SST25_SR_BP_MASK |
                             SST25_SR_AAI)) ==
                            (SST25_SR_WEL |
                             SST25_SR_AAI));
      UNUSED(status);

      /* Decrement the word count and advance the write position */

      nwords--;
      buffer  += 2;
      address += 2;

      /* Now loop, writing 16-bits of data on each pass through the loop
       * until all of the words have been transferred or until we encounter
       * data to be written to the erased state.
       */

      while (nwords > 0 &&
             (buffer[0] != SST25_ERASED_STATE ||
              buffer[1] != SST25_ERASED_STATE))
        {
          /* Select this FLASH part */

          SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

          /* Send "Auto Address Increment (AAI)" command with no address */

          SPI_SEND(priv->dev, SST25_AAI);

          /* Then write one 16-bit word */

          SPI_SNDBLOCK(priv->dev, buffer, 2);

          /* Deselect the FLASH: Chip Select high */

          SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

          /* Wait for the preceding write to complete. */

          status = sst25_waitwritecomplete(priv);
          DEBUGASSERT((status & (SST25_SR_WEL |
                                 SST25_SR_BP_MASK |
                                 SST25_SR_AAI)) ==
                                (SST25_SR_WEL |
                                 SST25_SR_AAI));
          UNUSED(status);

          /* Decrement the word count and advance the write position */

          nwords--;
          buffer  += 2;
          address += 2;
        }

      /* Disable writing */

      sst25_wrdi(priv);
    }
}
#endif

/****************************************************************************
 * Name: sst25_cacheflush
 ****************************************************************************/

#if defined(CONFIG_SST25_SECTOR512) && !defined(CONFIG_SST25_READONLY)
static void sst25_cacheflush(struct sst25_dev_s *priv)
{
  /* If the cached is dirty (meaning that it no longer matches the old FLASH
   * contents) or was erased (with the cache containing the correct FLASH
   * contents), then write the cached erase block to FLASH.
   */

  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      /* Write entire erase block to FLASH */

#ifdef CONFIG_SST25_SLOWWRITE
      sst25_bytewrite(priv, priv->sector,
                     (off_t)priv->esectno << priv->sectorshift,
                     (1 << priv->sectorshift));
#else
      sst25_wordwrite(priv, priv->sector,
                     (off_t)priv->esectno << priv->sectorshift,
                     (1 << priv->sectorshift));
#endif

      /* The case is no long dirty and the FLASH is no longer erased */

      CLR_DIRTY(priv);
      CLR_ERASED(priv);
    }
}
#endif

/****************************************************************************
 * Name: sst25_cacheread
 ****************************************************************************/

#if defined(CONFIG_SST25_SECTOR512) && !defined(CONFIG_SST25_READONLY)
static FAR uint8_t *sst25_cacheread(struct sst25_dev_s *priv, off_t sector)
{
  off_t esectno;
  int   shift;
  int   index;

  /* Convert from the 512 byte sector to the erase sector size of the device.
   * For exmample, if the actual erase sector size if 4Kb (1 << 12), then we
   * first shift to the right by 3 to get the sector number in 4096
   * increments.
   */

  shift    = priv->sectorshift - SST25_SECTOR_SHIFT;
  esectno  = sector >> shift;
  finfo("sector: %ld esectno: %d shift=%d\n", sector, esectno, shift);

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || esectno != priv->esectno)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      sst25_cacheflush(priv);

      /* Read the erase block into the cache */

      sst25_byteread(priv, priv->sector, (esectno << priv->sectorshift),
                     1 << priv->sectorshift);

      /* Mark the sector as cached */

      priv->esectno = esectno;

      SET_VALID(priv);          /* The data in the cache is valid */
      CLR_DIRTY(priv);          /* It should match the FLASH contents */
      CLR_ERASED(priv);         /* The underlying FLASH has not been erased */
    }

  /* Get the index to the 512 sector in the erase block that holds the
   * argument
   */

  index = sector & ((1 << shift) - 1);

  /* Return the address in the cache that holds this sector */

  return &priv->sector[index << SST25_SECTOR_SHIFT];
}
#endif

/****************************************************************************
 * Name: sst25_cacheerase
 ****************************************************************************/

#if defined(CONFIG_SST25_SECTOR512) && !defined(CONFIG_SST25_READONLY)
static void sst25_cacheerase(struct sst25_dev_s *priv, off_t sector)
{
  FAR uint8_t *dest;

  /* First, make sure that the erase block containing the 512 byte sector is
   * in the cache.
   */

  dest = sst25_cacheread(priv, sector);

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase sector
   * is read into the cache and set here when we erase the block.
   */

  if (!IS_ERASED(priv))
    {
      off_t esectno  = sector >> (priv->sectorshift - SST25_SECTOR_SHIFT);
      finfo("sector: %ld esectno: %d\n", sector, esectno);

      sst25_sectorerase(priv, esectno);
      SET_ERASED(priv);
    }

  /* Put the cached sector data into the erase state and mart the cache as
   * dirty (but don't update the FLASH yet. The caller will do that at a
   * more optimal time).
   */

  memset(dest, SST25_ERASED_STATE, SST25_SECTOR_SIZE);
  SET_DIRTY(priv);
}
#endif

/****************************************************************************
 * Name: sst25_cachewrite
 ****************************************************************************/

#if defined(CONFIG_SST25_SECTOR512) && !defined(CONFIG_SST25_READONLY)
static void sst25_cachewrite(FAR struct sst25_dev_s *priv,
                             FAR const uint8_t *buffer,
                             off_t sector,
                             size_t nsectors)
{
  FAR uint8_t *dest;

  for (; nsectors > 0; nsectors--)
    {
      /* First, make sure that the erase block containing 512 byte sector is
       * in memory.
       */

      dest = sst25_cacheread(priv, sector);

      /* Erase the block containing this sector if it is not already erased.
       * The erased indicated will be cleared when the data from the erase
       * sector is read into the cache and set here when we erase the sector.
       */

      if (!IS_ERASED(priv))
        {
          off_t esectno  = sector >>
                          (priv->sectorshift - SST25_SECTOR_SHIFT);
          finfo("sector: %ld esectno: %d\n", sector, esectno);

          sst25_sectorerase(priv, esectno);
          SET_ERASED(priv);
        }

      /* Copy the new sector data into cached erase block */

      memcpy(dest, buffer, SST25_SECTOR_SIZE);
      SET_DIRTY(priv);

      /* Set up for the next 512 byte sector */

      buffer += SST25_SECTOR_SIZE;
      sector++;
    }

  /* Flush the last erase block left in the cache */

  sst25_cacheflush(priv);
}
#endif

/****************************************************************************
 * Name: sst25_erase
 ****************************************************************************/

static int sst25_erase(FAR struct mtd_dev_s *dev,
                       off_t startblock,
                       size_t nblocks)
{
#ifdef CONFIG_SST25_READONLY
  return -EACESS
#else
  FAR struct sst25_dev_s *priv = (FAR struct sst25_dev_s *)dev;
  size_t blocksleft = nblocks;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  sst25_lock(priv->dev);

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

#ifdef CONFIG_SST25_SECTOR512
      sst25_cacheerase(priv, startblock);
#else
      sst25_sectorerase(priv, startblock);
#endif
      startblock++;
    }

#ifdef CONFIG_SST25_SECTOR512
  /* Flush the last erase block left in the cache */

  sst25_cacheflush(priv);
#endif

  sst25_unlock(priv->dev);
  return (int)nblocks;
#endif
}

/****************************************************************************
 * Name: sst25_bread
 ****************************************************************************/

static ssize_t sst25_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks,
                           FAR uint8_t *buffer)
{
#ifdef CONFIG_SST25_SECTOR512
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented
   * read
   */

  nbytes = sst25_read(dev, startblock << SST25_SECTOR_SHIFT,
                      nblocks << SST25_SECTOR_SHIFT, buffer);
  if (nbytes > 0)
    {
      return nbytes >> SST25_SECTOR_SHIFT;
    }

  return (int)nbytes;
#else
  FAR struct sst25_dev_s *priv = (FAR struct sst25_dev_s *)dev;
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented
   * read
   */

  nbytes = sst25_read(dev, startblock << priv->sectorshift,
                      nblocks << priv->sectorshift, buffer);
  if (nbytes > 0)
    {
      return nbytes >> priv->sectorshift;
    }

  return (int)nbytes;
#endif
}

/****************************************************************************
 * Name: sst25_bwrite
 ****************************************************************************/

static ssize_t sst25_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks,
                            FAR const uint8_t *buffer)
{
#ifdef CONFIG_SST25_READONLY
  return -EACCESS;
#else
  FAR struct sst25_dev_s *priv = (FAR struct sst25_dev_s *)dev;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the SPI bus and write all of the pages to FLASH */

  sst25_lock(priv->dev);

#if defined(CONFIG_SST25_SECTOR512)
  sst25_cachewrite(priv, buffer, startblock, nblocks);
#elif defined(CONFIG_SST25_SLOWWRITE)
  sst25_bytewrite(priv, buffer, startblock << priv->sectorshift,
                  nblocks << priv->sectorshift);
#else
  sst25_wordwrite(priv, buffer, startblock << priv->sectorshift,
                  nblocks << priv->sectorshift);
#endif
  sst25_unlock(priv->dev);

  return nblocks;
#endif
}

/****************************************************************************
 * Name: sst25_read
 ****************************************************************************/

static ssize_t sst25_read(FAR struct mtd_dev_s *dev,
                          off_t offset,
                          size_t nbytes,
                          FAR uint8_t *buffer)
{
  FAR struct sst25_dev_s *priv = (FAR struct sst25_dev_s *)dev;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the SPI bus and select this FLASH part */

  sst25_lock(priv->dev);
  sst25_byteread(priv, buffer, offset, nbytes);
  sst25_unlock(priv->dev);

  finfo("return nbytes: %d\n", (int)nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: sst25_ioctl
 ****************************************************************************/

static int sst25_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct sst25_dev_s *priv = (FAR struct sst25_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  finfo("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)
                                           ((uintptr_t)arg);
          if (geo)
            {
              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an array
               * of fixed size blocks. That is most likely not true, but the
               * client will expect the device logic to do whatever is
               * necessary to make it appear so.
               */

#ifdef CONFIG_SST25_SECTOR512
              geo->blocksize    = (1 << SST25_SECTOR_SHIFT);
              geo->erasesize    = (1 << SST25_SECTOR_SHIFT);
              geo->neraseblocks = priv->nsectors << (priv->sectorshift - 9);
#else
              geo->blocksize    = (1 << priv->sectorshift);
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

      case MTDIOC_BULKERASE:
        {
            /* Erase the entire device */

            sst25_lock(priv->dev);
            ret = sst25_chiperase(priv);
            sst25_unlock(priv->dev);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result = SST25_ERASED_STATE;

          ret = OK;
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sst25_initialize
 *
 * Description:
 *  Create an initialize MTD device instance. MTD devices are not
 *  registered in the file system, but are created as instances that can be
 *  bound to other functions (such as a block or character driver front end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *sst25_initialize(FAR struct spi_dev_s *dev)
{
  FAR struct sst25_dev_s *priv;
  int ret;

  finfo("dev: %p\n", dev);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_FLASH(0) definition) and so would
   * have to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct sst25_dev_s *)kmm_zalloc(sizeof(struct sst25_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure. (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = sst25_erase;
      priv->mtd.bread  = sst25_bread;
      priv->mtd.bwrite = sst25_bwrite;
      priv->mtd.read   = sst25_read;
      priv->mtd.ioctl  = sst25_ioctl;
      priv->mtd.name   = "sst25";
      priv->dev        = dev;

      /* Deselect the FLASH */

      SPI_SELECT(dev, SPIDEV_FLASH(0), false);

      /* Identify the FLASH chip and get its capacity */

      ret = sst25_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and
           * return NULL
           */

          ferr("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }
      else
        {
          /* Make sure that the FLASH is unprotected so that we can
           * write into it
           */

#ifndef CONFIG_SST25_READONLY
          sst25_unprotect(priv);
#endif

#ifdef CONFIG_SST25_SECTOR512        /* Simulate a 512 byte sector */
          /* Allocate a buffer for the erase block cache */

          priv->sector = (FAR uint8_t *)kmm_malloc(1 << priv->sectorshift);
          if (!priv->sector)
            {
              /* Allocation failed! Discard all of that work we just did and
               * return NULL
               */

              ferr("ERROR: Allocation failed\n");
              kmm_free(priv);
              return NULL;
            }
#endif
        }
    }

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
