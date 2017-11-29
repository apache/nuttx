/************************************************************************************
 * drivers/mtd/mx35.c
 * Driver for SPI-based MX35LFxGE4AB parts of 1 or 2GBit.
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Ekaterina Kovylova <fomalhaut.hm@gmail.com>
 *
 *   Copied from / based on mx25lx.c driver written by
 *   Aleksandr Vyhovanec <www.desh@gmail.com>
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
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Per the data sheet, MX35 parts can be driven with either SPI mode 0 (CPOL=0 and
 * CPHA=0) or mode 3 (CPOL=1 and CPHA=1). If CONFIG_MX35_SPIMODE is not defined,
 * mode 0 will be used.
 */

#ifndef CONFIG_MX35_SPIMODE
#  define CONFIG_MX35_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_MX35_SPIFREQUENCY
#  define CONFIG_MX35_SPIFREQUENCY 104000000
#endif

#ifndef CONFIG_MX35_MANUFACTURER
#  define CONFIG_MX35_MANUFACTURER 0xC2
#endif

/* Debug ****************************************************************************/

#ifdef CONFIG_MX35_DEBUG
# define mx35err(format, ...)    _err(format, ##__VA_ARGS__)
# define mx35info(format, ...)   _info(format, ##__VA_ARGS__)
#else
# define mx35err(x...)
# define mx35info(x...)
#endif

/* Indentification register values **************************************************/

#define MX35_MANUFACTURER              CONFIG_MX35_MANUFACTURER
#define MX35_MX35LF1GE4AB_CAPACITY     0x12  /* 1 Gb */
#define MX35_MX35LF2GE4AB_CAPACITY     0x22  /* 2 Gb */

/* Chip Geometries ******************************************************************/

/* MX35LF1GE4AB capacity is 1 G-bit */

#define MX35_MX35LF1GE4AB_SECTOR_SHIFT  17    /* Sector size 1 << 17 = 128 Kb */
#define MX35_MX35LF1GE4AB_NSECTORS      1024
#define MX35_MX35LF1GE4AB_PAGE_SHIFT    11     /* Page size 1 << 11 = 2 Kb */

/* MX35LF2GE4AB capacity is 2 G-bit */

#define MX35_MX35LF2GE4AB_SECTOR_SHIFT  17    /* Sector size 1 << 17 = 128 Kb */
#define MX35_MX35LF2GE4AB_NSECTORS      2048
#define MX35_MX35LF2GE4AB_PAGE_SHIFT    11    /* Page size 1 << 11 = 2 Kb */

/* MX35 Instructions ****************************************************************/
/*      Command                    Value     Description             Addr   Data    */
/*                                                                      Dummy       */
#define MX35_GET_FEATURE            0x0F   /* Get features           1   0   1      */
#define MX35_SET_FEATURE            0x1F   /* Set features           1   0   1      */
#define MX35_PAGE_READ              0x13   /* Array read             3   0   0      */
#define MX35_READ_FROM_CACHE        0x03   /* Output cache data
                                               on SO                 2   1   1-2112 */
#define MX35_READ_FROM_CACHE_X1     0x0B   /* Output cache data
                                               on SO                 2   1   1-2112 */
#define MX35_READ_FROM_CACHE_X2     0x3B   /* Output cache data
                                               on SI and SO          2   1   1-2112 */
#define MX35_READ_FROM_CACHE_X4     0x6B   /* Output cache data
                                               on SI, SO, WP, HOLD   2   1   1-2112 */
#define MX35_READ_ID                0x9F   /* Read device ID         0   1   2      */
#define MX35_ECC_STATUS_READ        0x7C   /* Internal ECC status
                                               output                0   1   1      */
#define MX35_BLOCK_ERASE            0xD8   /* Block erase            3   0   0      */
#define MX35_PROGRAM_EXECUTE        0x10   /* Enter block/page
                                               address, execute      3   0   0      */
#define MX35_PROGRAM_LOAD           0x02   /* Load program data with
                                               cache reset first     2   0   1-2112 */
#define MX35_PROGRAM_LOAD_RANDOM    0x84   /* Load program data
                                               without cache reset   2   0   1-2112 */
#define MX35_PROGRAM_LOAD_X4        0x32   /* Program load operation
                                               with x4 data input    2   0   1-2112 */
#define MX35_PROGRAM_LOAD_RANDOM_X4 0x34   /* Load random operation
                                               with x4 data input    2   0   1-2112 */
#define MX35_WRITE_ENABLE           0x06   /*                        0   0   0      */
#define MX35_WRITE_DISABLE          0x04   /*                        0   0   0      */
#define MX35_RESET                  0xFF   /* Reset the device       0   0   0      */

#define MX35_DUMMY                  0x00   /* No Operation           0   0   0      */

/* Feature register *****************************************************************/

/* Register address */

#define MX35_SECURE_OTP            0xB0
#define MX35_STATUS                0xC0
#define MX35_BLOCK_PROTECTION      0xA0

/* Bit definitions */

/* Secure OTP (On-Time-Programmable) register*/

#define MX35_SOTP_QE               (1 << 0)  /* Bit 0: Quad Enable */
#define MX35_SOTP_ECC              (1 << 4)  /* Bit 4: ECC enabled */
#define MX35_SOTP_SOTP_EN          (1 << 6)  /* Bit 6: Secure OTP Enable */
#define MX35_SOTP_SOTP_PROT        (1 << 7)  /* Bit 7: Secure OTP Protect */

/* Status register */

#define MX35_SR_OIP                (1 << 0)  /* Bit 0: Operation in progress */
#define MX35_SR_WEL                (1 << 1)  /* Bit 1: Write enable latch */
#define MX35_SR_E_FAIL             (1 << 2)  /* Bit 2: Erase fail */
#define MX35_SR_P_FAIL             (1 << 3)  /* Bit 3: Program Fail */
#define MX35_SR_ECC_S0             (1 << 4)  /* Bit 4-5: ECC Status  */
#define MX35_SR_ECC_S1             (1 << 5)

/* Block Protection register*/

#define MX35_BP_SP                 (1 << 0)  /* Bit 0: Solid-protection (1Gb only) */
#define MX35_BP_COMPL              (1 << 1)  /* Bit 1: Complementary (1Gb only) */
#define MX35_BP_INV                (1 << 2)  /* Bit 2: Invert (1Gb only) */
#define MX35_BP_BP0                (1 << 3)  /* Bit 3: Block Protection 0 */
#define MX35_BP_BP1                (1 << 4)  /* Bit 4: Block Protection 1 */
#define MX35_BP_BP2                (1 << 5)  /* Bit 5: Block Protection 2 */
#define MX35_BP_BPRWD              (1 << 7)  /* Bit 7: Block Protection Register
                                                 Write Disable */

/* ECC Status register */

#define MX35_FEATURE_ECC_MASK          (0x03 << 4)
#define MX35_FEATURE_ECC_INCORRECTABLE (0x02 << 4)
#define MX35_FEATURE_ECC_OFFSET        4
#define MX35_ECC_STATUS_MASK           0x0F
#define MX35_ECC_INCORRECTABLE         0x0F

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct m25p_dev_s.
 */

struct mx35_dev_s
{
  struct mtd_dev_s mtd;      /* MTD interface */
  FAR struct spi_dev_s *dev; /* Saved SPI interface instance */
  uint8_t highCapacity;
  uint8_t  sectorshift;      /* 17 */
  uint16_t nsectors;         /* 1024 or 2048 */
  uint8_t  pageshift;        /* 11 */
  uint8_t eccstatus;         /* Internal ECC status */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static inline void mx35_lock(FAR struct spi_dev_s *dev);
static inline void mx35_unlock(FAR struct spi_dev_s *dev);

static int mx35_readid(FAR struct mx35_dev_s *priv);
static bool mx35_waitstatus(FAR struct mx35_dev_s *priv, uint8_t mask,
                            bool successif);
static inline void mx35_writeenable(struct mx35_dev_s *priv);
static inline void mx35_writedisable(struct mx35_dev_s *priv);
static inline uint32_t mx35_addresstorow(FAR struct mx35_dev_s *priv,
                                         uint32_t address);
static inline uint32_t mx35_addresstocolumn(FAR struct mx35_dev_s *priv,
                                            uint32_t address);

static bool mx35_sectorerase(FAR struct mx35_dev_s *priv, off_t startsector);
static int mx35_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);

static void mx35_readbuffer(FAR struct mx35_dev_s *priv, uint32_t address,
                            uint8_t *buffer, size_t length);
static bool mx35_read_page(FAR struct mx35_dev_s *priv, uint32_t position);
static ssize_t mx35_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer);

static void mx35_write_to_cache(FAR struct mx35_dev_s *priv, uint32_t address,
                                const uint8_t *buffer, size_t length);
static bool mx35_execute_write(FAR struct mx35_dev_s *priv, uint32_t position);
static ssize_t mx35_write(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                          FAR const uint8_t *buffer);

static int mx35_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);
static inline void mx35_eccstatusread(struct mx35_dev_s *priv);
static inline void mx35_enableECC(struct mx35_dev_s *priv);
static inline void mx35_unlockblocks(struct mx35_dev_s *priv);

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: mx35_lock
 ************************************************************************************/

static inline void mx35_lock(FAR struct spi_dev_s *dev)
{
  /* On SPI busses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the busses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusive access to
   * the SPI buss.  We will retain that exclusive access until the bus is unlocked.
   */

  (void)SPI_LOCK(dev, true);

  /* After locking the SPI bus, the we also need call the setfrequency, setbits, and
   * setmode methods to make sure that the SPI is properly configured for the device.
   * If the SPI buss is being shared, then it may have been left in an incompatible
   * state.
   */

  SPI_SETMODE(dev, CONFIG_MX35_SPIMODE);
  SPI_SETBITS(dev, 8);
  (void)SPI_HWFEATURES(dev, 0);
  (void)SPI_SETFREQUENCY(dev, CONFIG_MX35_SPIFREQUENCY);
}

/************************************************************************************
 * Name: mx35_unlock
 ************************************************************************************/

static inline void mx35_unlock(FAR struct spi_dev_s *dev)
{
  (void)SPI_LOCK(dev, false);
}

/************************************************************************************
 * Name: m25p_readid
 ************************************************************************************/

static int mx35_readid(struct mx35_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t capacity;

  mx35info("priv: %p\n", priv);

  /* Lock the SPI bus, configure the bus, and select this FLASH part. */

  mx35_lock(priv->dev);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send the "Read ID" command and read two ID bytes */

  (void)SPI_SEND(priv->dev, MX35_READ_ID);
  (void)SPI_SEND(priv->dev, MX35_DUMMY);
  manufacturer = SPI_SEND(priv->dev, MX35_DUMMY);
  capacity     = SPI_SEND(priv->dev, MX35_DUMMY);

  /* Deselect the FLASH and unlock the bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
  mx35_unlock(priv->dev);

  mx35info("manufacturer: %02x capacity: %02x\n",
           manufacturer, capacity);

  /* Check for a valid manufacturer */

  if (manufacturer == MX35_MANUFACTURER)
    {
      /* Okay.. is it a FLASH capacity that we understand? */

      if (capacity == MX35_MX35LF1GE4AB_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->highCapacity = 0;
          priv->sectorshift = MX35_MX35LF1GE4AB_SECTOR_SHIFT;
          priv->nsectors    = MX35_MX35LF1GE4AB_NSECTORS;
          priv->pageshift   = MX35_MX35LF1GE4AB_PAGE_SHIFT;
          return OK;
        }
      else if (capacity == MX35_MX35LF2GE4AB_CAPACITY)
        {
          /* Save the FLASH geometry */

          priv->highCapacity = 1;
          priv->sectorshift = MX35_MX35LF2GE4AB_SECTOR_SHIFT;
          priv->nsectors    = MX35_MX35LF2GE4AB_NSECTORS;
          priv->pageshift   = MX35_MX35LF2GE4AB_PAGE_SHIFT;
          return OK;
        }
    }

  return -ENODEV;
}

/************************************************************************************
 * Name: mx35_waitstatus
 ************************************************************************************/

static bool mx35_waitstatus(FAR struct mx35_dev_s *priv, uint8_t mask, bool successif)
{
  uint8_t status;

  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Select this FLASH part */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

      /* Get feature command */

      (void)SPI_SEND(priv->dev, MX35_GET_FEATURE);
      (void)SPI_SEND(priv->dev, MX35_STATUS);
      status = SPI_SEND(priv->dev, MX35_DUMMY);

      /* Deselect the FLASH */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

      /* Given that writing could take up to few tens of milliseconds, and erasing
       * could take more.  The following short delay in the "busy" case will allow
       * other peripherals to access the SPI bus.
       */
    }
  while (((status & MX35_SR_OIP) != 0) && (!usleep(1000)));

  mx35info("Complete\n");
  return successif ? ((status & mask) != 0) : ((status & mask) == 0);
}

/************************************************************************************
 * Name:  mx35_writeenable
 ************************************************************************************/

static inline void mx35_writeenable(struct mx35_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send Write Enable command */

  (void)SPI_SEND(priv->dev, MX35_WRITE_ENABLE);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
}

/************************************************************************************
 * Name:  mx35_writedisable
 ************************************************************************************/

static inline void mx35_writedisable(struct mx35_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send Write Enable command */

  (void)SPI_SEND(priv->dev, MX35_WRITE_DISABLE);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
}

/************************************************************************************
 * Name:  mx35_addresstorow
 ************************************************************************************/

static inline uint32_t mx35_addresstorow(FAR struct mx35_dev_s *priv,
                        uint32_t address)
{
  /* Convert to page */

  uint32_t row = address >> priv->pageshift;

  if (priv->highCapacity)
    {
      const uint32_t plane = (row >> (16 - 6)) & 0x40;

      /* Shift block address */

      row = ((row & ~0x3F) << 1) | (row & 0x3F);

      /* Insert plane select bit */

      row = row | plane;
    }

  return row;
}

/************************************************************************************
 * Name:  mx35_addresstocolumn
 ************************************************************************************/

static inline uint32_t mx35_addresstocolumn(FAR struct mx35_dev_s *priv,
                                            uint32_t address)
{
  uint32_t column = address % (1 << priv->pageshift);

  if (priv->highCapacity)
    {
      /* Convert to page */

      const uint32_t row = address >> priv->pageshift;
      const uint32_t plane = (row >> (16 - 12)) & 0x1000;

      /* Insert plane select bit */

      column = column | plane;
    }
  else
    {
      uint16_t wraplength = 0x00;
      column |= (wraplength & 0xC000);
    }

  return column;
}

/************************************************************************************
 * Name:  mx35_sectorerase (128K)
 ************************************************************************************/

static bool mx35_sectorerase(FAR struct mx35_dev_s *priv, off_t startsector)
{
  off_t address = (off_t)startsector << priv->sectorshift;
  const uint32_t block = mx35_addresstorow(priv, address);

  mx35info("sector: %08lx\n", (long)startsector);

  /* Send write enable instruction */

  mx35_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send the Block Erase instruction */

  (void)SPI_SEND(priv->dev, MX35_BLOCK_ERASE);
  (void)SPI_SEND(priv->dev, (block >> 16) & 0xff);
  (void)SPI_SEND(priv->dev, (block >> 8) & 0xff);
  (void)SPI_SEND(priv->dev, block & 0xff);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

  mx35info("Erased\n");
  return mx35_waitstatus(priv, MX35_SR_E_FAIL, false);
}

/************************************************************************************
 * Name: mx35_erase
 ************************************************************************************/

static int mx35_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
  FAR struct mx35_dev_s *priv = (FAR struct mx35_dev_s *)dev;
  size_t blocksleft = nblocks;

  mx35info("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  mx35_lock(priv->dev);

  /* Wait all operations complete */

  mx35_waitstatus(priv, MX35_SR_OIP, false);

  while (blocksleft-- > 0)
    {
      mx35_sectorerase(priv, startblock);
      startblock++;
    }

  mx35_unlock(priv->dev);
  return (int)nblocks;
}

/************************************************************************************
 * Name: mx35_readbuffer
 ************************************************************************************/

static void mx35_readbuffer(FAR struct mx35_dev_s *priv, uint32_t address,
                            uint8_t *buffer, size_t length)
{
  const uint16_t offset = mx35_addresstocolumn(priv, address);

  /* Select the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  (void)SPI_SEND(priv->dev, MX35_READ_FROM_CACHE);

  /* Send the address high byte first. */

  (void)SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  (void)SPI_SEND(priv->dev, (offset) & 0xff);

  /* Send a dummy byte */

  (void)SPI_SEND(priv->dev, MX35_DUMMY);

  /* Then read all of the requested bytes */

  SPI_RECVBLOCK(priv->dev, buffer, length);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
}

/************************************************************************************
 * Name: mx35_read_page
 ************************************************************************************/

static bool mx35_read_page(FAR struct mx35_dev_s *priv, uint32_t pageaddress)
{
  const uint32_t row = mx35_addresstorow(priv, pageaddress);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send the Read Page instruction */

  (void)SPI_SEND(priv->dev, MX35_PAGE_READ);
  (void)SPI_SEND(priv->dev, (row >> 16) & 0xff);
  (void)SPI_SEND(priv->dev, (row >> 8) & 0xff);
  (void)SPI_SEND(priv->dev, row & 0xff);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

  mx35_waitstatus(priv, MX35_SR_OIP, false);

  mx35_eccstatusread(priv);
  if ((priv->eccstatus & MX35_FEATURE_ECC_MASK) == MX35_FEATURE_ECC_INCORRECTABLE)
    {
      return false;
    }

  return true;
}

/************************************************************************************
 * Name: mx35_read
 ************************************************************************************/

static ssize_t mx35_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer)
{
  FAR struct mx35_dev_s *priv = (FAR struct mx35_dev_s *)dev;
  size_t bytesleft = nbytes;
  uint32_t position = offset;

  mx35info("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the SPI bus and select this FLASH part */

  mx35_lock(priv->dev);

  /* Wait all operations complete */

  mx35_waitstatus(priv, MX35_SR_OIP, false);

  while(bytesleft)
    {
      const uint32_t pageaddress = (position >> priv->pageshift) << priv->pageshift;
      const uint32_t spaceleft = pageaddress + (1 << priv->pageshift) - position;
      const size_t chunklength = bytesleft < spaceleft ? bytesleft : spaceleft;

      if (!mx35_read_page(priv, pageaddress))
        {
          break;
        }

      mx35_readbuffer(priv, position, buffer, chunklength);

      position += chunklength;
      buffer += chunklength;
      bytesleft -= chunklength;
    }


  mx35_unlock(priv->dev);

  mx35info("return nbytes: %d\n", (int)(nbytes - bytesleft));
  return nbytes - bytesleft;
}

/************************************************************************************
 * Name: mx35_write_to_cache
 ************************************************************************************/

static void mx35_write_to_cache(FAR struct mx35_dev_s *priv, uint32_t address,
                                const uint8_t *buffer, size_t length)
{
  const uint16_t offset = mx35_addresstocolumn(priv, address);

  /* Select the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send the Program Load command */

  (void)SPI_SEND(priv->dev, MX35_PROGRAM_LOAD);

  /* Send the address high byte first. */

  (void)SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  (void)SPI_SEND(priv->dev, (offset) & 0xff);

  /* Send block of bytes */

  SPI_SNDBLOCK(priv->dev, buffer, length);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
}

/************************************************************************************
 * Name: mx35_write_to_cache
 ************************************************************************************/

static bool mx35_execute_write(FAR struct mx35_dev_s *priv, uint32_t pageaddress)
{
  const uint32_t row = mx35_addresstorow(priv, pageaddress);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);

  /* Send the Pragram Execute instruction */

  (void)SPI_SEND(priv->dev, MX35_PROGRAM_EXECUTE);
  (void)SPI_SEND(priv->dev, (row >> 16) & 0xff);
  (void)SPI_SEND(priv->dev, (row >> 8) & 0xff);
  (void)SPI_SEND(priv->dev, row & 0xff);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

  return mx35_waitstatus(priv, MX35_SR_P_FAIL, false);
}

/************************************************************************************
 * Name: mx35_write
 ************************************************************************************/

static ssize_t mx35_write(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                          FAR const uint8_t *buffer)
{
  FAR struct mx35_dev_s *priv = (FAR struct mx35_dev_s *)dev;
  size_t bytesleft = nbytes;
  uint32_t position = offset;

  mx35_lock(priv->dev);

  /* Wait all operations complete */

  mx35_waitstatus(priv, MX35_SR_OIP, false);

  while(bytesleft)
    {
      const uint32_t pageaddress = (position >> priv->pageshift) << priv->pageshift;
      const uint32_t spaceleft = pageaddress + (1 << priv->pageshift) - position;
      const size_t chunklength = bytesleft < spaceleft ? bytesleft : spaceleft;

      mx35_writeenable(priv);
      mx35_write_to_cache(priv, position, buffer, chunklength);
      if (!mx35_execute_write(priv, pageaddress))
        {
          continue;
        }

      position += chunklength;
      buffer += chunklength;
      bytesleft -= chunklength;
    }

  mx35_unlock(priv->dev);

  return nbytes - bytesleft;
}

/************************************************************************************
 * Name: mx25l_ioctl
 ************************************************************************************/

static int mx35_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct mx35_dev_s *priv = (FAR struct mx35_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  mx35info("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
                  (FAR struct mtd_geometry_s *)((uintptr_t)arg);
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

              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;

              ret = OK;

              mx35info("blocksize: %d erasesize: %d neraseblocks: %d\n",
                       geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */
          ret = mx35_erase(dev, 0, priv->nsectors);
        }
        break;

      case MTDIOC_ECCSTATUS:
        {
          uint8_t *result = (uint8_t *)arg;
          *result =
              (priv->eccstatus & MX35_FEATURE_ECC_MASK) >> MX35_FEATURE_ECC_OFFSET;

          ret = OK;
        }
      break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  mx35info("return %d\n", ret);
  return ret;
}

/************************************************************************************
 * Name:  mx35_eccstatusread
 ************************************************************************************/

static inline void mx35_eccstatusread(struct mx35_dev_s *priv)
{
  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
  (void)SPI_SEND(priv->dev, MX35_GET_FEATURE);
  (void)SPI_SEND(priv->dev, MX35_STATUS);
  priv->eccstatus = SPI_SEND(priv->dev, MX35_DUMMY);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);
}

/************************************************************************************
 * Name:  mx35_enableECC
 ************************************************************************************/

static inline void mx35_enableECC(struct mx35_dev_s *priv)
{
  uint8_t secureOTP = MX35_SOTP_ECC;

  mx35_lock(priv->dev);
  mx35_writeenable(priv);

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
  (void)SPI_SEND(priv->dev, MX35_SET_FEATURE);
  (void)SPI_SEND(priv->dev, MX35_SECURE_OTP);
  (void)SPI_SEND(priv->dev, secureOTP);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

  mx35_writedisable(priv);
  mx35_unlock(priv->dev);
}

/************************************************************************************
 * Name:  mx35_unlockblocks
 ************************************************************************************/

static inline void mx35_unlockblocks(struct mx35_dev_s *priv)
{
  uint8_t blockprotection = 0x00;

  mx35_lock(priv->dev);
  mx35_writeenable(priv);

  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
  (void)SPI_SEND(priv->dev, MX35_SET_FEATURE);
  (void)SPI_SEND(priv->dev, MX35_BLOCK_PROTECTION);
  (void)SPI_SEND(priv->dev, blockprotection);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

  mx35_writedisable(priv);
  mx35_unlock(priv->dev);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: mx35_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/

FAR struct mtd_dev_s *mx35_initialize(FAR struct spi_dev_s *dev)
{
  FAR struct mx35_dev_s *priv;
  int ret;

  mx35info("dev: %p\n", dev);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_FLASH(0) definition) and so would have
   * to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct mx35_dev_s *)kmm_zalloc(sizeof(struct mx35_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure. (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = mx35_erase;
      priv->mtd.read   = mx35_read;
      priv->mtd.write  = mx35_write;
      priv->mtd.ioctl  = mx35_ioctl;
      priv->dev        = dev;

      /* Deselect the FLASH */

      SPI_SELECT(dev, SPIDEV_FLASH(0), false);

      /* Reset the flash */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(0), true);
      (void)SPI_SEND(priv->dev, MX35_RESET);
      SPI_SELECT(priv->dev, SPIDEV_FLASH(0), false);

      /* Wait reset complete */

      mx35_waitstatus(priv, MX35_SR_OIP, false);

      /* Identify the FLASH chip and get its capacity */

      ret = mx35_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and return NULL */

          mx35err("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }
      else
        {

#ifdef CONFIG_MTD_REGISTRATION
       /* Register the MTD with the procfs system if enabled */

       mtd_register(&priv->mtd, "mx35");
#endif
        }

      mx35_enableECC(priv);
      mx35_unlockblocks(priv);
    }

  /* Return the implementation-specific state structure as the MTD device */

  mx35info("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
