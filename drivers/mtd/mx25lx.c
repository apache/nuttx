/****************************************************************************
 * drivers/mtd/mx25lx.c
 * Driver for SPI-based or QSPI-based MX25Lxx33L parts of 32 or 64MBit.
 *
 *   Copyright (C) 2016, 2019 Gregory Nutt. All rights reserved.
 *   Author: Aleksandr Vyhovanec <www.desh@gmail.com>
 *
 *   Copied from / based on sst25.c and w25.c drivers written by
 *   Gregory Nutt <gnutt@nuttx.org>
 *   Ken Pettit <pettitkd@gmail.com>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
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

/* Per the data sheet, MX25L parts can be driven with either SPI mode 0
 * (CPOL=0 and CPHA=0) or mode 3 (CPOL=1 and CPHA=1).
 * So you may need to specify:
 * CONFIG_MX25L_SPIMODE to select the best mode for your device.
 * If CONFIG_MX25L_SPIMODE is not defined, mode 0 will be used.
 */

#ifndef CONFIG_MX25L_SPIMODE
#  define CONFIG_MX25L_SPIMODE SPIDEV_MODE0
#endif

/* SPI Frequency.  May be up to 133 MHz. */

#ifndef CONFIG_MX25L_SPIFREQUENCY
#  define CONFIG_MX25L_SPIFREQUENCY 20000000
#endif

/* Chip Geometries **********************************************************/

/* MX25L3233F capacity is 32Mbit  (4096Kbit x 8) =   4Mb (512kb x 8) */

#define MX25L_MX25L3233F_SECTOR_SHIFT  12    /* Sector size 1 << 12 = 4Kb */
#define MX25L_MX25L3233F_NSECTORS      1024
#define MX25L_MX25L3233F_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */

/* MX25L6433F capacity is 32Mbit  (8192Kbit x 8) =   8Mb (1024kb x 8) */

#define MX25L_MX25L6433F_SECTOR_SHIFT  12    /* Sector size 1 << 12 = 4Kb */
#define MX25L_MX25L6433F_NSECTORS      2048
#define MX25L_MX25L6433F_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */

/* MX25L25635F capacity is 256Mbit */

#define MX25L_MX25L25635F_SECTOR_SHIFT  12    /* Sector size 1 << 12 = 4Kb */
#define MX25L_MX25L25635F_NSECTORS      8192
#define MX25L_MX25L25635F_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */

/* Parts larger than 128Mbit require 4-byte addressing */

#define MX25L_ADDRESSBYTES_3            3
#define MX25L_ADDRESSBYTES_4            4

#ifdef CONFIG_MX25L_SECTOR512                /* Simulate a 512 byte sector */
#  define MX25L_SECTOR512_SHIFT        9     /* Sector size 1 << 9 = 512 bytes */
#endif

#define MX25L_ERASED_STATE     0xff   /* State of FLASH when erased */

#define MX25L_CACHE_VALID    (1 << 0)   /* 1=Cache has valid data */
#define MX25L_CACHE_DIRTY    (1 << 1)   /* 1=Cache is dirty */
#define MX25L_CACHE_ERASED   (1 << 2)   /* 1=Backing FLASH is erased */

#define IS_VALID(p)      ((((p)->flags) & MX25L_CACHE_VALID) != 0)
#define IS_DIRTY(p)      ((((p)->flags) & MX25L_CACHE_DIRTY) != 0)
#define IS_ERASED(p)     ((((p)->flags) & MX25L_CACHE_ERASED) != 0)

#define SET_VALID(p)      do { (p)->flags |= MX25L_CACHE_VALID; } while (0)
#define SET_DIRTY(p)      do { (p)->flags |= MX25L_CACHE_DIRTY; } while (0)
#define SET_ERASED(p)     do { (p)->flags |= MX25L_CACHE_ERASED; } while (0)

#define CLR_VALID(p)      do { (p)->flags &= ~MX25L_CACHE_VALID; } while (0)
#define CLR_DIRTY(p)      do { (p)->flags &= ~MX25L_CACHE_DIRTY; } while (0)
#define CLR_ERASED(p)     do { (p)->flags &= ~MX25L_CACHE_ERASED; } while (0)

/* MX25L Instructions *******************************************************/

/* Command                 Value    Description               Addr   Data   */

/*                                                               Dummy      */

#define MX25L_READ          0x03  /* Read data bytes           3/4 0  >=1   */
#define MX25L_FAST_READ     0x0b  /* Higher speed read         3/4 1  >=1   */
#define MX25L_2READ         0xbb  /* 2 x I/O read command                   */
#define MX25L_DREAD         0x3b  /* 1I / 2O read command      3/4 1  >=1   */
#define MX25L_4READ         0xeb  /* 4 x I/O read command                   */
#define MX25L_QREAD         0x6b  /* 1I / 4O read command      3/4 1  >=1   */
#define MX25L_WREN          0x06  /* Write Enable              0   0  0     */
#define MX25L_WRDI          0x04  /* Write Disable             0   0  0     */
#define MX25L_RDSR          0x05  /* Read status register      0   0  >=1   */
#define MX25L_RDCR          0x15  /* Read config register      0   0  >=1   */
#define MX25L_WRSR          0x01  /* Write stat/conf register  0   0  2     */
#define MX25L_4PP           0x38  /* Quad page program         3/4 0  1-256 */
#define MX25L_SE            0x20  /* 4Kb Sector erase          3/4 0  0     */
#define MX25L_BE32          0x52  /* 32Kbit block Erase        3/4 0  0     */
#define MX25L_BE64          0xd8  /* 64Kbit block Erase        3/4 0  0     */
#define MX25L_CE            0xc7  /* Chip erase                0   0  0     */
#define MX25L_CE_ALT        0x60  /* Chip erase (alternate)    0   0  0     */
#define MX25L_PP            0x02  /* Page program              3   0  1-256 */
#define MX25L_DP            0xb9  /* Deep power down           0   0  0     */
#define MX25L_RDP           0xab  /* Release deep power down   0   0  0     */
#define MX25L_PGM_SUSPEND   0x75  /* Suspends program          0   0  0     */
#define MX25L_ERS_SUSPEND   0xb0  /* Suspends erase            0   0  0     */
#define MX25L_PGM_RESUME    0x7a  /* Resume program            0   0  0     */
#define MX25L_ERS_RESUME    0x30  /* Resume erase              0   0  0     */
#define MX25L_RDID          0x9f  /* Read identification       0   0  3     */
#define MX25L_RES           0xab  /* Read electronic ID        0   3  1     */
#define MX25L_REMS          0x90  /* Read manufacture and ID   1   2  >=2   */
#define MX25L_ENSO          0xb1  /* Enter secured OTP         0   0  0     */
#define MX25L_EXSO          0xc1  /* Exit secured OTP          0   0  0     */
#define MX25L_RDSCUR        0x2b  /* Read security register    0   0  0     */
#define MX25L_WRSCUR        0x2f  /* Write security register   0   0  0     */
#define MX25L_RSTEN         0x66  /* Reset Enable              0   0  0     */
#define MX25L_RST           0x99  /* Reset Memory              0   0  0     */
#define MX25L_EN4B          0xb7  /* Enter 4-byte mode         0   0  0     */
#define MX25L_EX4B          0xe9  /* Exit 4-byte mode          0   0  0     */
#define MX25L_READ4B        0x13  /* Read data (4 Byte mode)   4   0  >=1   */
#define MX25L_FAST_READ4B   0x0c  /* Higher speed read    (4B) 4   1  >=1   */
#define MX25L_2READ4B       0xbc  /* 2 x I/O read command (4B)              */
#define MX25L_DREAD4B       0x3c  /* 1I / 2O read command (4B) 4   1  >=1   */
#define MX25L_4READ4B       0xec  /* 4 x I/O read command (4B)              */
#define MX25L_QREAD4B       0x6c  /* 1I / 4O read command (4B) 4   1  >=1   */
#define MX25L_4PP4B         0x3e  /* Quad page program    (4B) 4   0  1-256 */
#define MX25L_SE4B          0x21  /* 4Kb Sector erase     (4B) 4   0  0     */
#define MX25L_BE32K4B       0x5c  /* 32Kbit block Erase   (4B) 4   0  0     */
#define MX25L_BE64K4B       0xdc  /* 64Kbit block Erase   (4B) 4   0  0     */
#define MX25L_PP4B          0x12  /* Page program         (4B) 4   0  1-256 */
#define MX25L_RDSFDP        0x5a  /* read out until CS# high                */
#define MX25L_SBL           0xc0  /* Set Burst Length                       */
#define MX25L_SBL_ALT       0x77  /* Set Burst Length                       */
#define MX25L_NOP           0x00  /* No Operation              0   0  0     */

/* MX25L Registers **********************************************************/

/* Read ID (RDID) register values */

#define MX25L_MANUFACTURER          0xc2  /* Macronix manufacturer ID */
#define MX25L3233F_DEVID            0x15  /* MX25L3233F device ID */

/* JEDEC Read ID register values */

#define MX25L_JEDEC_MANUFACTURER         0xc2  /* Macronix manufacturer ID */
#define MX25L_JEDEC_MEMORY_TYPE          0x20  /* MX25Lx  memory type */
#define MX25L_JEDEC_MX25L3233F_CAPACITY  0x16  /* MX25L3233F memory capacity */
#define MX25L_JEDEC_MX25L6433F_CAPACITY  0x17  /* MX25L6433F memory capacity */
#define MX25L_JEDEC_MX25L25635F_CAPACITY 0x19  /* MX25L25635F memory capacity */

/* Status register bit definitions */

#define MX25L_SR_WIP            (1 << 0)  /* Bit 0: Write in progress */
#define MX25L_SR_WEL            (1 << 1)  /* Bit 1: Write enable latch */
#define MX25L_SR_BP_SHIFT       (2)       /* Bits 2-5: Block protect bits */
#define MX25L_SR_BP_MASK        (15 << MX25L_SR_BP_SHIFT)
#define MX25L_SR_QE             (1 << 6)  /* Bit 6: Quad enable */
#define MX25L_SR_SRWD           (1 << 7)  /* Bit 7: Status register write protect */

/* Configuration register bit definitions */

#define MX25L_CR_ODS            (1 << 0)  /* Bit 0: Output driver strength */
#define MX25L_CR_TB             (1 << 3)  /* Bit 3: Top/bottom selected */
#define MX25L_CR_DC             (1 << 6)  /* Bit 6: Dummy cycle */

#define MX25L_DUMMY              MX25L_NOP

/* Debug ********************************************************************/

#ifdef CONFIG_MX25L_DEBUG
# define mxlerr(format, ...)    _err(format, ##__VA_ARGS__)
# define mxlinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
# define mxlerr(x...)
# define mxlinfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct mx25l_dev_s.
 */

struct mx25l_dev_s
{
  struct mtd_dev_s mtd;              /* MTD interface */
  FAR struct spi_dev_s *dev;         /* Saved SPI interface instance */
  uint8_t               sectorshift;
  uint8_t               pageshift;
  uint8_t               addressbytes; /* Number of address bytes required */
  uint16_t              nsectors;
#if defined(CONFIG_MX25L_SECTOR512)
  uint8_t               flags;       /* Buffered sector flags */
  uint16_t              esectno;     /* Erase sector number in the cache */
  FAR uint8_t          *sector;      /* Allocated sector data */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static void mx25l_lock(FAR struct spi_dev_s *dev);
static inline void mx25l_unlock(FAR struct spi_dev_s *dev);
static inline int mx25l_readid(FAR struct mx25l_dev_s *priv);
static void mx25l_waitwritecomplete(FAR struct mx25l_dev_s *priv);
static void mx25l_writeenable(FAR struct mx25l_dev_s *priv);
static void mx25l_writedisable(FAR struct mx25l_dev_s *priv);
static inline void mx25l_sectorerase(FAR struct mx25l_dev_s *priv,
                                     off_t offset);
static inline int  mx25l_chiperase(FAR struct mx25l_dev_s *priv);
static void mx25l_byteread(FAR struct mx25l_dev_s *priv,
                           FAR uint8_t *buffer,
                           off_t address,
                           size_t nbytes);
static inline void mx25l_pagewrite(FAR struct mx25l_dev_s *priv,
                                   FAR const uint8_t *buffer,
                                   off_t address, size_t nbytes);
#if defined(CONFIG_MX25L_SECTOR512)
static void mx25l_cacheflush(FAR struct mx25l_dev_s *priv);
static FAR uint8_t *mx25l_cacheread(FAR struct mx25l_dev_s *priv,
                                    off_t sector);
static void mx25l_cacheerase(FAR struct mx25l_dev_s *priv,
                             off_t sector);
static void mx25l_cachewrite(FAR struct mx25l_dev_s *priv,
                             FAR const uint8_t *buffer,
                             off_t sector, size_t nsectors);
#endif

/* MTD driver methods */

static int mx25l_erase(FAR struct mtd_dev_s *dev,
                       off_t startblock,
                       size_t nblocks);
static ssize_t mx25l_bread(FAR struct mtd_dev_s *dev,
                           off_t startblock,
                           size_t nblocks,
                           FAR uint8_t *buf);
static ssize_t mx25l_bwrite(FAR struct mtd_dev_s *dev,
                            off_t startblock,
                            size_t nblocks,
                            FAR const uint8_t *buf);
static ssize_t mx25l_read(FAR struct mtd_dev_s *dev,
                          off_t offset,
                          size_t nbytes,
                          FAR uint8_t *buffer);
static int mx25l_ioctl(FAR struct mtd_dev_s *dev,
                       int cmd,
                       unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx25l_lock
 ****************************************************************************/

static void mx25l_lock(FAR struct spi_dev_s *dev)
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

  SPI_SETMODE(dev, CONFIG_MX25L_SPIMODE);
  SPI_SETBITS(dev, 8);
  SPI_HWFEATURES(dev, 0);
  SPI_SETFREQUENCY(dev, CONFIG_MX25L_SPIFREQUENCY);
}

/****************************************************************************
 * Name: mx25l_unlock
 ****************************************************************************/

static inline void mx25l_unlock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, false);
}

/****************************************************************************
 * Name: mx25l_readid
 ****************************************************************************/

static inline int mx25l_readid(FAR struct mx25l_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t memory;
  uint16_t capacity;

  mxlinfo("priv: %p\n", priv);

  /* Lock the SPI bus, configure the bus, and select this FLASH part. */

  mx25l_lock(priv->dev);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send the "Read ID (RDID)" command and read the first three ID bytes */

  SPI_SEND(priv->dev, MX25L_RDID);
  manufacturer = SPI_SEND(priv->dev, MX25L_DUMMY);
  memory       = SPI_SEND(priv->dev, MX25L_DUMMY);
  capacity     = SPI_SEND(priv->dev, MX25L_DUMMY);

  /* Deselect the FLASH and unlock the bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
  mx25l_unlock(priv->dev);

  mxlinfo("manufacturer: %02x memory: %02x capacity: %02x\n",
          manufacturer, memory, capacity);

  /* Check for a valid manufacturer and memory type */

  if (manufacturer == MX25L_JEDEC_MANUFACTURER &&
      memory == MX25L_JEDEC_MEMORY_TYPE)
    {
      /* Okay.. is it a FLASH capacity that we understand? */

      if (capacity == MX25L_JEDEC_MX25L3233F_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift  = MX25L_MX25L3233F_SECTOR_SHIFT;
          priv->nsectors     = MX25L_MX25L3233F_NSECTORS;
          priv->pageshift    = MX25L_MX25L3233F_PAGE_SHIFT;
          priv->addressbytes = MX25L_ADDRESSBYTES_3;
          return OK;
        }
      else if (capacity == MX25L_JEDEC_MX25L6433F_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift  = MX25L_MX25L6433F_SECTOR_SHIFT;
          priv->nsectors     = MX25L_MX25L6433F_NSECTORS;
          priv->pageshift    = MX25L_MX25L6433F_PAGE_SHIFT;
          priv->addressbytes = MX25L_ADDRESSBYTES_3;
          return OK;
        }
      else if (capacity == MX25L_JEDEC_MX25L25635F_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->sectorshift  = MX25L_MX25L25635F_SECTOR_SHIFT;
          priv->nsectors     = MX25L_MX25L25635F_NSECTORS;
          priv->pageshift    = MX25L_MX25L25635F_PAGE_SHIFT;
          priv->addressbytes = MX25L_ADDRESSBYTES_4;
          return OK;
        }
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: mx25l_waitwritecomplete
 ****************************************************************************/

static void mx25l_waitwritecomplete(FAR struct mx25l_dev_s *priv)
{
  uint8_t status;

  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Select this FLASH part */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

      /* Send "Read Status Register (RDSR)" command */

      SPI_SEND(priv->dev, MX25L_RDSR);

      /* Send a dummy byte to generate the clock needed to shift out
       * the status
       */

      status = SPI_SEND(priv->dev, MX25L_DUMMY);

      /* Deselect the FLASH */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

      /* Given that writing could take up to few tens of milliseconds, and
       * erasing could take more.
       * The following short delay in the "busy" case will allow other
       * peripherals to access the SPI bus.
       */

      if ((status & MX25L_SR_WIP) != 0)
        {
          mx25l_unlock(priv->dev);
          nxsig_usleep(1000);
          mx25l_lock(priv->dev);
        }
    }
  while ((status & MX25L_SR_WIP) != 0);

  mxlinfo("Complete\n");
}

/****************************************************************************
 * Name:  mx25l_writeenable
 ****************************************************************************/

static void mx25l_writeenable(FAR struct mx25l_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send "Write Enable (WREN)" command */

  SPI_SEND(priv->dev, MX25L_WREN);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

  mxlinfo("Enabled\n");
}

/****************************************************************************
 * Name:  mx25l_writedisable
 ****************************************************************************/

static void mx25l_writedisable(FAR struct mx25l_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send "Write Disable (WRDI)" command */

  SPI_SEND(priv->dev, MX25L_WRDI);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

  mxlinfo("Disabled\n");
}

/****************************************************************************
 * Name:  mx25l_sectorerase (4k)
 ****************************************************************************/

static void mx25l_sectorerase(FAR struct mx25l_dev_s *priv, off_t sector)
{
  off_t offset;

  offset = sector << priv->sectorshift;

  mxlinfo("sector: %08lx\n", (long)sector);

  /* Send write enable instruction */

  mx25l_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send the "Sector Erase (SE)" or "Block Erase (BE)" instruction
   * that was passed in as the erase type.
   */

  /* The command we send varies depending on if we need 3 or 4 address
   * bytes
   */

  if (priv->addressbytes == MX25L_ADDRESSBYTES_4)
    {
      SPI_SEND(priv->dev, MX25L_SE4B);

      /* Send the sector offset high byte first. */

      SPI_SEND(priv->dev, (offset >> 24) & 0xff);
      SPI_SEND(priv->dev, (offset >> 16) & 0xff);
      SPI_SEND(priv->dev, (offset >> 8) & 0xff);
      SPI_SEND(priv->dev, offset & 0xff);
    }
  else
    {
      SPI_SEND(priv->dev, MX25L_SE);

      /* Send the sector offset high byte first.  For all of the supported
       * parts, the sector number is completely contained in the first byte
       * and the values used in the following two bytes don't really matter.
       */

      SPI_SEND(priv->dev, (offset >> 16) & 0xff);
      SPI_SEND(priv->dev, (offset >> 8) & 0xff);
      SPI_SEND(priv->dev, offset & 0xff);
    }

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

  mx25l_waitwritecomplete(priv);

  mxlinfo("Erased\n");
}

/****************************************************************************
 * Name:  mx25l_chiperase
 ****************************************************************************/

static inline int mx25l_chiperase(FAR struct mx25l_dev_s *priv)
{
  mxlinfo("priv: %p\n", priv);

  /* Send write enable instruction */

  mx25l_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send the "Chip Erase (CE)" instruction */

  SPI_SEND(priv->dev, MX25L_CE);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

  mx25l_waitwritecomplete(priv);

  mxlinfo("Return: OK\n");
  return OK;
}

/****************************************************************************
 * Name: mx25l_byteread
 ****************************************************************************/

static void mx25l_byteread(FAR struct mx25l_dev_s *priv, FAR uint8_t *buffer,
                           off_t address, size_t nbytes)
{
  mxlinfo("address: %08lx nbytes: %d\n", (long)address, (int)nbytes);

  /* Wait for any preceding write or erase operation to complete. */

  mx25l_waitwritecomplete(priv);

  /* Make sure that writing is disabled */

  mx25l_writedisable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* The command we send varies depending on if we need 3 or 4 address
   * bytes
   */

  if (priv->addressbytes == MX25L_ADDRESSBYTES_4)
    {
      /* Send "Read from Memory - 4 byte mode" instruction */

      SPI_SEND(priv->dev, MX25L_FAST_READ4B);

      /* Send the address high byte first. */

      SPI_SEND(priv->dev, (address >> 24) & 0xff);
      SPI_SEND(priv->dev, (address >> 16) & 0xff);
      SPI_SEND(priv->dev, (address >> 8) & 0xff);
      SPI_SEND(priv->dev, address & 0xff);
    }
  else
    {
      /* Send "Read from Memory " instruction */

      SPI_SEND(priv->dev, MX25L_FAST_READ);

      /* Send the address high byte first. */

      SPI_SEND(priv->dev, (address >> 16) & 0xff);
      SPI_SEND(priv->dev, (address >> 8) & 0xff);
      SPI_SEND(priv->dev, address & 0xff);
    }

  /* Send a dummy byte */

  SPI_SEND(priv->dev, MX25L_DUMMY);

  /* Then read all of the requested bytes */

  SPI_RECVBLOCK(priv->dev, buffer, nbytes);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
}

/****************************************************************************
 * Name:  mx25l_pagewrite
 ****************************************************************************/

static inline void mx25l_pagewrite(FAR struct mx25l_dev_s *priv,
                                   FAR const uint8_t *buffer,
                                   off_t address, size_t nbytes)
{
  mxlinfo("address: %08lx nwords: %d\n", (long)address, (int)nbytes);

  for (; nbytes > 0; nbytes -= (1 << priv->pageshift))
    {
      /* Enable the write access to the FLASH */

      mx25l_writeenable(priv);

      /* Select this FLASH part */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

      if (priv->addressbytes == MX25L_ADDRESSBYTES_4)
        {
          /* Send the "Page Program - 4 byte mode (MX25L_PP4B)" Command */

          SPI_SEND(priv->dev, MX25L_PP4B);

          /* Send the address high byte first. */

          SPI_SEND(priv->dev, (address >> 24) & 0xff);
          SPI_SEND(priv->dev, (address >> 16) & 0xff);
          SPI_SEND(priv->dev, (address >> 8) & 0xff);
          SPI_SEND(priv->dev, address & 0xff);
        }
      else
        {
          /* Send the "Page Program (MX25L_PP)" Command */

          SPI_SEND(priv->dev, MX25L_PP);

          /* Send the address high byte first. */

          SPI_SEND(priv->dev, (address >> 16) & 0xff);
          SPI_SEND(priv->dev, (address >> 8) & 0xff);
          SPI_SEND(priv->dev, address & 0xff);
        }

      /* Then send the page of data */

      SPI_SNDBLOCK(priv->dev, buffer, 1 << priv->pageshift);

      /* Deselect the FLASH and setup for the next pass through the loop */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

      /* Wait for any preceding write or erase operation to complete. */

      mx25l_waitwritecomplete(priv);

      /* Update addresses */

      address += 1 << priv->pageshift;
      buffer  += 1 << priv->pageshift;
    }

  mxlinfo("Written\n");
}

/****************************************************************************
 * Name: mx25l_cacheflush
 ****************************************************************************/

#if defined(CONFIG_MX25L_SECTOR512)
static void mx25l_cacheflush(FAR struct mx25l_dev_s *priv)
{
  /* If the cached is dirty (meaning that it no longer matches the old
   * FLASH contents)  or was erased (with the cache containing the correct
   * FLASH contents), then write the cached erase block to FLASH.
   */

  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      /* Write entire erase block to FLASH */

      mx25l_pagewrite(priv, priv->sector,
                     (off_t)priv->esectno << priv->sectorshift,
                     (1 << priv->sectorshift));

      /* The case is no long dirty and the FLASH is no longer erased */

      CLR_DIRTY(priv);
      CLR_ERASED(priv);
    }
}
#endif

/****************************************************************************
 * Name: mx25l_cacheread
 ****************************************************************************/

#if defined(CONFIG_MX25L_SECTOR512)
static FAR uint8_t *mx25l_cacheread(FAR struct mx25l_dev_s *priv,
                                    off_t sector)
{
  off_t esectno;
  int   shift;
  int   index;

  /* Convert from the 512 byte sector to the erase sector size of the device.
   * For exmample, if the actual erase sector size if 4Kb (1 << 12), then we
   * first shift to the right by 3 to get the sector number in 4096
   * increments.
   */

  shift    = priv->sectorshift - MX25L_SECTOR512_SHIFT;
  esectno  = sector >> shift;
  mxlinfo("sector: %ld esectno: %d shift=%d\n", sector, esectno, shift);

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || esectno != priv->esectno)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      mx25l_cacheflush(priv);

      /* Read the erase block into the cache */

      mx25l_byteread(priv, priv->sector, (esectno << priv->sectorshift),
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

  return &priv->sector[index << MX25L_SECTOR512_SHIFT];
}
#endif

/****************************************************************************
 * Name: mx25l_cacheerase
 ****************************************************************************/

#if defined(CONFIG_MX25L_SECTOR512)
static void mx25l_cacheerase(FAR struct mx25l_dev_s *priv, off_t sector)
{
  FAR uint8_t *dest;

  /* First, make sure that the erase block containing the 512 byte sector is
   * in the cache.
   */

  dest = mx25l_cacheread(priv, sector);

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase sector
   * is read into the cache and set here when we erase the block.
   */

  if (!IS_ERASED(priv))
    {
      off_t esectno  = sector >> (priv->sectorshift - MX25L_SECTOR512_SHIFT);
      mxlinfo("sector: %ld esectno: %d\n", sector, esectno);

      mx25l_sectorerase(priv, esectno);
      SET_ERASED(priv);
    }

  /* Put the cached sector data into the erase state and mart the cache as
   * dirty (but don't update the FLASH yet. The caller will do that at a more
   * optimal time).
   */

  memset(dest, MX25L_ERASED_STATE, 1 << MX25L_SECTOR512_SHIFT);
  SET_DIRTY(priv);
}
#endif

/****************************************************************************
 * Name: mx25l_cachewrite
 ****************************************************************************/

#if defined(CONFIG_MX25L_SECTOR512)
static void mx25l_cachewrite(FAR struct mx25l_dev_s *priv,
                             FAR const uint8_t *buffer,
                             off_t sector, size_t nsectors)
{
  FAR uint8_t *dest;

  for (; nsectors > 0; nsectors--)
    {
      /* First, make sure that the erase block containing 512 byte sector is
       * in memory.
       */

      dest = mx25l_cacheread(priv, sector);

      /* Erase the block containing this sector if it is not already erased.
       * The erased indicated will be cleared when the data from the erase
       * sector is read into the cache and set here when we erase the sector.
       */

      if (!IS_ERASED(priv))
        {
          off_t esectno  =
                sector >> (priv->sectorshift - MX25L_SECTOR512_SHIFT);
          mxlinfo("sector: %ld esectno: %d\n", sector, esectno);

          mx25l_sectorerase(priv, esectno);
          SET_ERASED(priv);
        }

      /* Copy the new sector data into cached erase block */

      memcpy(dest, buffer, 1 << MX25L_SECTOR512_SHIFT);
      SET_DIRTY(priv);

      /* Set up for the next 512 byte sector */

      buffer += 1 << MX25L_SECTOR512_SHIFT;
      sector++;
    }

  /* Flush the last erase block left in the cache */

  mx25l_cacheflush(priv);
}
#endif

/****************************************************************************
 * Name: mx25l_erase
 ****************************************************************************/

static int mx25l_erase(FAR struct mtd_dev_s *dev,
                       off_t startblock,
                       size_t nblocks)
{
  FAR struct mx25l_dev_s *priv = (FAR struct mx25l_dev_s *)dev;
  size_t blocksleft = nblocks;

  mxlinfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  mx25l_lock(priv->dev);

  while (blocksleft-- > 0)
    {
      /* MX25LVF parts have complex block overlay structure for the moment
       * we just erase in 4k blocks.
       */

#ifdef CONFIG_MX25L_SECTOR512
      mx25l_cacheerase(priv, startblock);
#else
      mx25l_sectorerase(priv, startblock);
#endif
      startblock++;
    }

#ifdef CONFIG_MX25L_SECTOR512
  /* Flush the last erase block left in the cache */

  mx25l_cacheflush(priv);
#endif

  mx25l_unlock(priv->dev);
  return (int)nblocks;
}

/****************************************************************************
 * Name: mx25l_bread
 ****************************************************************************/

static ssize_t mx25l_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR uint8_t *buffer)
{
  FAR struct mx25l_dev_s *priv = (FAR struct mx25l_dev_s *)dev;
  ssize_t nbytes;

  mxlinfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented
   * read
   */

#ifdef CONFIG_MX25L_SECTOR512
  nbytes = mx25l_read(dev, startblock << MX25L_SECTOR512_SHIFT,
                      nblocks << MX25L_SECTOR512_SHIFT, buffer);
  if (nbytes > 0)
    {
      return nbytes >> MX25L_SECTOR512_SHIFT;
    }
#else
  nbytes = mx25l_read(dev,
                      startblock << priv->pageshift,
                      nblocks << priv->pageshift,
                      buffer);
  if (nbytes > 0)
    {
      return nbytes >> priv->pageshift;
    }
#endif

  return (int)nbytes;
}

/****************************************************************************
 * Name: mx25l_bwrite
 ****************************************************************************/

static ssize_t mx25l_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct mx25l_dev_s *priv = (FAR struct mx25l_dev_s *)dev;

  mxlinfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the SPI bus and write all of the pages to FLASH */

  mx25l_lock(priv->dev);

#if defined(CONFIG_MX25L_SECTOR512)
  mx25l_cachewrite(priv, buffer, startblock, nblocks);
#else
  mx25l_pagewrite(priv, buffer, startblock << priv->pageshift,
                  nblocks << priv->pageshift);
#endif
  mx25l_unlock(priv->dev);

  return nblocks;
}

/****************************************************************************
 * Name: mx25l_read
 ****************************************************************************/

static ssize_t mx25l_read(FAR struct mtd_dev_s *dev,
                          off_t offset,
                          size_t nbytes,
                          FAR uint8_t *buffer)
{
  FAR struct mx25l_dev_s *priv = (FAR struct mx25l_dev_s *)dev;

  mxlinfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the SPI bus and select this FLASH part */

  mx25l_lock(priv->dev);
  mx25l_byteread(priv, buffer, offset, nbytes);
  mx25l_unlock(priv->dev);
  mxlinfo("return nbytes: %d\n", (int)nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: mx25l_ioctl
 ****************************************************************************/

static int mx25l_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct mx25l_dev_s *priv = (FAR struct mx25l_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  mxlinfo("cmd: %d \n", cmd);

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

#ifdef CONFIG_MX25L_SECTOR512
              geo->blocksize    = (1 << MX25L_SECTOR512_SHIFT);
              geo->erasesize    = (1 << MX25L_SECTOR512_SHIFT);
              geo->neraseblocks = priv->nsectors <<
                                (priv->sectorshift - MX25L_SECTOR512_SHIFT);
#else
              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;
#endif
              ret = OK;

              mxlinfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
                      geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
            /* Erase the entire device */

            mx25l_lock(priv->dev);
            ret = mx25l_chiperase(priv);
            mx25l_unlock(priv->dev);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result = MX25L_ERASED_STATE;

          ret = OK;
        }
        break;

      case MTDIOC_XIPBASE:
      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  mxlinfo("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx25l_initialize_spi
 *
 * Description:
 *   Create an initialize MTD device instance. MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *mx25l_initialize_spi(FAR struct spi_dev_s *dev)
{
  FAR struct mx25l_dev_s *priv;
  int ret;

  mxlinfo("dev: %p\n", dev);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_FLASH(0) definition) and so would
   * have to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct mx25l_dev_s *)kmm_zalloc(sizeof(struct mx25l_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure. (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = mx25l_erase;
      priv->mtd.bread  = mx25l_bread;
      priv->mtd.bwrite = mx25l_bwrite;
      priv->mtd.read   = mx25l_read;
      priv->mtd.ioctl  = mx25l_ioctl;
      priv->mtd.name   = "mx25l";
      priv->dev        = dev;

      /* Deselect the FLASH */

      SPI_SELECT(dev, SPIDEV_FLASH(0), false);

      /* Identify the FLASH chip and get its capacity */

      ret = mx25l_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and
           * return NULL
           */

          mxlerr("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }
      else
        {
#ifdef CONFIG_MX25L_SECTOR512        /* Simulate a 512 byte sector */
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

  mxlinfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
