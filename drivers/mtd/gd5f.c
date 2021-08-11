/****************************************************************************
 * drivers/mtd/gd5f.c
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

#ifndef CONFIG_GD5F_SPIMODE
#  define CONFIG_GD5F_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_GD5F_SPIFREQUENCY
#  define CONFIG_GD5F_SPIFREQUENCY  20000000
#endif

/* GD5F Instructions ********************************************************/

/*      Command                  Value     Description       Addr   Data    */

/*                                                                    Dummy */

#define GD5F_GET_FEATURE          0x0f /* Get features        1   0   1     */
#define GD5F_SET_FEATURE          0x1f /* Set features        1   0   1     */
#define GD5F_PAGE_READ            0x13 /* Array read          3   0   0     */
#define GD5F_READ_FROM_CACHE      0x03 /* Output cache data
                                        *  on SO              2   1   1-2112 */
#define GD5F_READ_ID              0x9f /* Read device ID      0   1   2     */
#define GD5F_ECC_STATUS_READ      0x7c /* Internal ECC status
                                        *  output             0   1   1     */
#define GD5F_BLOCK_ERASE          0xd8 /* Block erase         3   0   0     */
#define GD5F_PROGRAM_EXECUTE      0x10 /* Enter block/page
                                        * address, execute    3   0   0     */
#define GD5F_PROGRAM_LOAD         0x02 /* Load program data with
                                        * cache reset first   2   0   1-2112 */
#define GD5F_PROGRAM_LOAD_RANDOM  0x84 /* Load program data
                                        * without cache reset 2   0   1-2112 */
#define GD5F_WRITE_ENABLE         0x06 /*                     0   0   0     */
#define GD5F_WRITE_DISABLE        0x04 /*                     0   0   0     */
#define GD5F_RESET                0xff /* Reset the device    0   0   0     */
#define GD5F_DUMMY                0x00 /* No Operation        0   0   0     */

/* Feature register *********************************************************/

/* JEDEC Read ID register values */

#define GD5F_MANUFACTURER           0xc8
#define GD5F_GD5F_CAPACITY_MASK     0x0f
#define GD5F_CAPACITY_1GBIT         0x01  /* 1 Gb */
#define GD5F_CAPACITY_2GBIT         0x02  /* 2 Gb */
#define GD5F_CAPACITY_4GBIT         0x04  /* 4 Gb */

#define GD5F_NSECTORS_1GBIT         1024  /* 1024x131072 = 1Gbit memory capacity */
#define GD5F_NSECTORS_2GBIT         2048  /* 2048x131072 = 2Gbit memory capacity */
#define GD5F_NSECTORS_4GBIT         4096  /* 4096x131072 = 4Gbit memory capacity */

#define GD5F_SECTOR_SHIFT           17    /* 131072 byte */
#define GD5F_PAGE_SHIFT             11    /* 2048 */

/* Register address */

#define GD5F_SECURE_OTP             0xb0
#define GD5F_STATUS                 0xc0
#define GD5F_BLOCK_PROTECTION       0xa0

/* Bit definitions */

/* Secure OTP (On-Time-Programmable) register */

#define GD5F_SOTP_QE                (1 << 0)  /* Bit 0: Quad Enable */
#define GD5F_SOTP_ECC               (1 << 4)  /* Bit 4: ECC enabled */
#define GD5F_SOTP_SOTP_EN           (1 << 6)  /* Bit 6: Secure OTP Enable */
#define GD5F_SOTP_SOTP_PROT         (1 << 7)  /* Bit 7: Secure OTP Protect */

/* Status register */

#define GD5F_SR_OIP                 (1 << 0)  /* Bit 0: Operation in progress */
#define GD5F_SR_WEL                 (1 << 1)  /* Bit 1: Write enable latch */
#define GD5F_SR_E_FAIL              (1 << 2)  /* Bit 2: Erase fail */
#define GD5F_SR_P_FAIL              (1 << 3)  /* Bit 3: Program Fail */
#define GD5F_SR_ECC_S0              (1 << 4)  /* Bit 4-5: ECC Status  */
#define GD5F_SR_ECC_S1              (1 << 5)

/* Block Protection register */

#define GD5F_BP_SP                  (1 << 0)  /* Bit 0: Solid-protection (1Gb only) */
#define GD5F_BP_COMPL               (1 << 1)  /* Bit 1: Complementary (1Gb only) */
#define GD5F_BP_INV                 (1 << 2)  /* Bit 2: Invert (1Gb only) */
#define GD5F_BP_BP0                 (1 << 3)  /* Bit 3: Block Protection 0 */
#define GD5F_BP_BP1                 (1 << 4)  /* Bit 4: Block Protection 1 */
#define GD5F_BP_BP2                 (1 << 5)  /* Bit 5: Block Protection 2 */
#define GD5F_BP_BPRWD               (1 << 7)  /* Bit 7: Block Protection Register
                                               *        Write Disable */

/* ECC Status register */

#define GD5F_FEATURE_ECC_MASK       (0x03 << 4)
#define GD5F_FEATURE_ECC_ERROR      (0x02 << 4)
#define GD5F_FEATURE_ECC_OFFSET     4
#define GD5F_ECC_STATUS_MASK        0x0f

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct gd5f_dev_s.
 */

struct gd5f_dev_s
{
  struct mtd_dev_s     mtd;             /* MTD interface */
  FAR struct spi_dev_s *dev;            /* Saved SPI interface instance */
  uint32_t             spi_devid;       /* Chip select inputs */
  uint16_t             nsectors;        /* 1024 or 2048 */
  uint8_t              sectorshift;     /* 17 */
  uint8_t              pageshift;       /* 11 */
  uint8_t              eccstatus;       /* Internal ECC status */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline void gd5f_lock(FAR struct spi_dev_s *dev);
static inline void gd5f_unlock(FAR struct spi_dev_s *dev);

static int gd5f_readid(FAR struct gd5f_dev_s *priv);
static bool gd5f_waitstatus(FAR struct gd5f_dev_s *priv,
                            uint8_t mask,
                            bool successif);
static inline void gd5f_writeenable(FAR struct gd5f_dev_s *priv);
static inline void gd5f_writedisable(FAR struct gd5f_dev_s *priv);
static bool gd5f_sectorerase(FAR struct gd5f_dev_s *priv,
                             off_t startsector);
static void gd5f_readbuffer(FAR struct gd5f_dev_s *priv,
                            uint32_t address,
                            uint8_t *buffer,
                            size_t length);
static bool gd5f_read_page(FAR struct gd5f_dev_s *priv,
                           uint32_t position);

static void gd5f_write_to_cache(FAR struct gd5f_dev_s *priv,
                                uint32_t address,
                                const uint8_t *buffer,
                                size_t length);
static bool gd5f_execute_write(FAR struct gd5f_dev_s *priv,
                               uint32_t position);

static inline void gd5f_eccstatusread(FAR struct gd5f_dev_s *priv);
static inline void gd5f_enable_ecc(FAR struct gd5f_dev_s *priv);
static inline void gd5f_unlockblocks(FAR struct gd5f_dev_s *priv);

/* MTD driver methods */

static ssize_t gd5f_bread(FAR struct mtd_dev_s *dev,
                          off_t startblock,
                          size_t nblocks,
                          FAR uint8_t *buffer);
static ssize_t gd5f_read(FAR struct mtd_dev_s *dev,
                         off_t offset,
                         size_t nbytes,
                         FAR uint8_t *buffer);
static ssize_t gd5f_bwrite(FAR struct mtd_dev_s *dev,
                           off_t startblock,
                           size_t nblocks,
                           FAR const uint8_t *buffer);
static ssize_t gd5f_write(FAR struct mtd_dev_s *dev,
                          off_t offset,
                          size_t nbytes,
                          FAR const uint8_t *buffer);
static int gd5f_ioctl(FAR struct mtd_dev_s *dev,
                      int cmd,
                      unsigned long arg);
static int gd5f_erase(FAR struct mtd_dev_s *dev,
                      off_t startblock,
                      size_t nblocks);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd5f_lock
 ****************************************************************************/

static inline void gd5f_lock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, true);

  SPI_SETMODE(dev, CONFIG_GD5F_SPIMODE);
  SPI_SETBITS(dev, 8);
  SPI_HWFEATURES(dev, 0);
  SPI_SETFREQUENCY(dev, CONFIG_GD5F_SPIFREQUENCY);
}

/****************************************************************************
 * Name: gd5f_unlock
 ****************************************************************************/

static inline void gd5f_unlock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, false);
}

/****************************************************************************
 * Name: gd5f_readid
 ****************************************************************************/

static int gd5f_readid(FAR struct gd5f_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t deviceid;
  uint16_t capacity;

  finfo("priv: %p\n", priv);

  /* Lock the SPI bus, configure the bus, and select this FLASH part. */

  gd5f_lock(priv->dev);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the "Read ID" command and read two ID bytes */

  SPI_SEND(priv->dev, GD5F_READ_ID);
  SPI_SEND(priv->dev, GD5F_DUMMY);
  manufacturer = SPI_SEND(priv->dev, GD5F_DUMMY);
  deviceid     = SPI_SEND(priv->dev, GD5F_DUMMY);

  /* De-select the FLASH and unlock the bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
  gd5f_unlock(priv->dev);

  finfo("manufacturer: %02x deviceid: %02x\n",
           manufacturer, deviceid);

  /* Check for a valid manufacturer */

  if (manufacturer == GD5F_MANUFACTURER)
    {
      capacity = deviceid & GD5F_GD5F_CAPACITY_MASK;

      if (capacity == GD5F_CAPACITY_1GBIT)
        {
          priv->nsectors = GD5F_NSECTORS_1GBIT;
        }
      else if (capacity == GD5F_CAPACITY_2GBIT)
        {
          priv->nsectors = GD5F_NSECTORS_2GBIT;
        }
      else if (capacity == GD5F_CAPACITY_4GBIT)
        {
          priv->nsectors = GD5F_NSECTORS_4GBIT;
        }
      else
        {
          return -ENODEV;
        }

      priv->sectorshift = GD5F_SECTOR_SHIFT;
      priv->pageshift   = GD5F_PAGE_SHIFT;
      return OK;
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: gd5f_waitstatus
 ****************************************************************************/

static bool gd5f_waitstatus(FAR struct gd5f_dev_s *priv,
                            uint8_t mask,
                            bool successif)
{
  uint8_t status;

  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Select this FLASH part */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

      /* Get feature command */

      SPI_SEND(priv->dev, GD5F_GET_FEATURE);
      SPI_SEND(priv->dev, GD5F_STATUS);
      status = SPI_SEND(priv->dev, GD5F_DUMMY);

      /* Deselect the FLASH */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
      nxsig_usleep(1000);
    }
  while ((status & GD5F_SR_OIP) != 0);

  finfo("Complete %02x\n", status);

  return successif ? ((status & mask) != 0) : ((status & mask) == 0);
}

/****************************************************************************
 * Name:  gd5f_writeenable
 ****************************************************************************/

static inline void gd5f_writeenable(FAR struct gd5f_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send Write Enable command */

  SPI_SEND(priv->dev, GD5F_WRITE_ENABLE);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
}

/****************************************************************************
 * Name:  gd5f_writedisable
 ****************************************************************************/

static inline void gd5f_writedisable(FAR struct gd5f_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send Write Enable command */

  SPI_SEND(priv->dev, GD5F_WRITE_DISABLE);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
}

/****************************************************************************
 * Name:  gd5f_sectorerase (128K)
 ****************************************************************************/

static bool gd5f_sectorerase(FAR struct gd5f_dev_s *priv,
                             off_t startsector)
{
  const uint32_t block = startsector << (priv->sectorshift -
                                         priv->pageshift);

  finfo("block sector: %08lx\n", (long)block);

  /* Send write enable instruction */

  gd5f_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the Block Erase instruction */

  SPI_SEND(priv->dev, GD5F_BLOCK_ERASE);
  SPI_SEND(priv->dev, (block >> 16) & 0xff);
  SPI_SEND(priv->dev, (block >> 8) & 0xff);
  SPI_SEND(priv->dev, block & 0xff);

  /* De-select the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

  finfo("Erased\n");
  return gd5f_waitstatus(priv, GD5F_SR_E_FAIL, false);
}

/****************************************************************************
 * Name: gd5f_erase
 ****************************************************************************/

static int gd5f_erase(FAR struct mtd_dev_s *dev,
                      off_t startblock,
                      size_t nblocks)
{
  FAR struct gd5f_dev_s *priv = (FAR struct gd5f_dev_s *)dev;
  size_t blocksleft = nblocks;

  finfo("Erase: startblock: %08lx nblocks: %d\n",
        (long)startblock,
        (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  gd5f_lock(priv->dev);

  /* Wait all operations complete */

  gd5f_waitstatus(priv, GD5F_SR_OIP, false);

  while (blocksleft > 0)
    {
      if (!gd5f_sectorerase(priv, startblock))
        {
          break;
        }

      startblock++;
      blocksleft--;
    }

  gd5f_unlock(priv->dev);
  return nblocks - blocksleft;
}

/****************************************************************************
 * Name: gd5f_readbuffer
 ****************************************************************************/

static void gd5f_readbuffer(FAR struct gd5f_dev_s *priv,
                            uint32_t address,
                            uint8_t *buffer,
                            size_t length)
{
  const uint16_t offset = address & ((1 << priv->pageshift) - 1);

  /* Select the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  SPI_SEND(priv->dev, GD5F_READ_FROM_CACHE);

  /* Send the address high byte first. */

  SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  SPI_SEND(priv->dev, (offset) & 0xff);

  /* Send a dummy byte */

  SPI_SEND(priv->dev, GD5F_DUMMY);

  /* Then read all of the requested bytes */

  SPI_RECVBLOCK(priv->dev, buffer, length);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
}

/****************************************************************************
 * Name: gd5f_read_page
 ****************************************************************************/

static bool gd5f_read_page(FAR struct gd5f_dev_s *priv, uint32_t pageaddress)
{
  const uint32_t row = pageaddress >> priv->pageshift;

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the Read Page instruction */

  SPI_SEND(priv->dev, GD5F_PAGE_READ);
  SPI_SEND(priv->dev, (row >> 16) & 0xff);
  SPI_SEND(priv->dev, (row >> 8) & 0xff);
  SPI_SEND(priv->dev, row & 0xff);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

  /* Wait Page Read Complete */

  gd5f_waitstatus(priv, GD5F_SR_OIP, false);

  /* Check HardWare ECC result */

  gd5f_eccstatusread(priv);
  if ((priv->eccstatus & GD5F_FEATURE_ECC_MASK) == GD5F_FEATURE_ECC_ERROR)
    {
      /* ECC report uncorrectable, discard data */

      return false;
    }

  return true;
}

/****************************************************************************
 * Name: gd5f_read
 ****************************************************************************/

static ssize_t gd5f_read(FAR struct mtd_dev_s *dev,
                         off_t offset,
                         size_t nbytes,
                         FAR uint8_t *buffer)
{
  FAR struct gd5f_dev_s *priv = (FAR struct gd5f_dev_s *)dev;
  size_t bytesleft = nbytes;
  uint32_t position = offset;

  finfo("Read: offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the SPI bus and select this FLASH part */

  gd5f_lock(priv->dev);

  /* Wait all operations complete */

  gd5f_waitstatus(priv, GD5F_SR_OIP, false);

  while (bytesleft)
    {
      const uint32_t pageaddress =
                     (position >> priv->pageshift) << priv->pageshift;
      const uint32_t spaceleft =
                     pageaddress + (1 << priv->pageshift) - position;
      const size_t chunklength =
                   bytesleft < spaceleft ? bytesleft : spaceleft;

      if (!gd5f_read_page(priv, pageaddress))
        {
          break;
        }

      gd5f_readbuffer(priv, position, buffer, chunklength);

      position += chunklength;
      buffer += chunklength;
      bytesleft -= chunklength;
    }

  gd5f_unlock(priv->dev);

  finfo("return nbytes: %d\n", (int)(nbytes - bytesleft));
  return nbytes - bytesleft;
}

/****************************************************************************
 * Name: gd5f_bread
 ****************************************************************************/

static ssize_t gd5f_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buffer)
{
  ssize_t nbytes;
  FAR struct gd5f_dev_s *priv = (FAR struct gd5f_dev_s *)dev;

  finfo("Bread: startblock: %08lx nblocks: %d\n",
        (long)startblock, (int)nblocks);

  nbytes = gd5f_read(dev, startblock << priv->pageshift,
                     nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
      nbytes >>= priv->pageshift;
    }

  return nbytes;
}

/****************************************************************************
 * Name: gd5f_write_to_cache
 ****************************************************************************/

static void gd5f_write_to_cache(FAR struct gd5f_dev_s *priv,
                                uint32_t address,
                                const uint8_t *buffer,
                                size_t length)
{
  const uint16_t offset = address & ((1 << priv->pageshift) - 1);

  /* Select the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the Program Load command */

  SPI_SEND(priv->dev, GD5F_PROGRAM_LOAD);

  /* Send the address high byte first. */

  SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  SPI_SEND(priv->dev, (offset) & 0xff);

  /* Send block of bytes */

  SPI_SNDBLOCK(priv->dev, buffer, length);

  /* De-select the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
}

/****************************************************************************
 * Name: gd5f_execute_write
 ****************************************************************************/

static bool gd5f_execute_write(FAR struct gd5f_dev_s *priv,
                               uint32_t pageaddress)
{
  const uint32_t row = pageaddress >> priv->pageshift;

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the Program Execute instruction */

  SPI_SEND(priv->dev, GD5F_PROGRAM_EXECUTE);
  SPI_SEND(priv->dev, (row >> 16) & 0xff);
  SPI_SEND(priv->dev, (row >> 8) & 0xff);
  SPI_SEND(priv->dev, row & 0xff);

  /* De-select the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

  return gd5f_waitstatus(priv, GD5F_SR_P_FAIL, false);
}

/****************************************************************************
 * Name: gd5f_write
 ****************************************************************************/

static ssize_t gd5f_write(FAR struct mtd_dev_s *dev,
                          off_t offset,
                          size_t nbytes,
                          FAR const uint8_t *buffer)
{
  FAR struct gd5f_dev_s *priv = (FAR struct gd5f_dev_s *)dev;
  size_t bytesleft = nbytes;
  uint32_t position = offset;

  finfo("Write: offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);
  gd5f_lock(priv->dev);

  /* Wait all operations complete */

  gd5f_waitstatus(priv, GD5F_SR_OIP, false);

  while (bytesleft)
    {
      const uint32_t pageaddress =
                    (position >> priv->pageshift) << priv->pageshift;
      const uint32_t spaceleft =
                     pageaddress + (1 << priv->pageshift) - position;
      const size_t chunklength =
                   bytesleft < spaceleft ? bytesleft : spaceleft;

      gd5f_write_to_cache(priv, position, buffer, chunklength);
      gd5f_writeenable(priv);
      if (!gd5f_execute_write(priv, pageaddress))
        {
          break;
        }

      position += chunklength;
      buffer += chunklength;
      bytesleft -= chunklength;
    }

  gd5f_unlock(priv->dev);

  return nbytes - bytesleft;
}

/****************************************************************************
 * Name: gd5f_bwrite
 ****************************************************************************/

static ssize_t gd5f_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
    size_t nblocks, FAR const uint8_t *buffer)
{
  ssize_t nbytes;

  FAR struct gd5f_dev_s *priv = (FAR struct gd5f_dev_s *)dev;

  finfo("Bwrite: startblock: %08lx nblocks: %d\n",
        (long)startblock, (int)nblocks);

  /* Lock the SPI bus and write all of the pages to FLASH */

  nbytes = gd5f_write(dev, startblock << priv->pageshift,
                nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
      nbytes >>= priv->pageshift;
    }

  return nbytes;
}

/****************************************************************************
 * Name: mx25l_ioctl
 ****************************************************************************/

static int gd5f_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct gd5f_dev_s *priv = (FAR struct gd5f_dev_s *)dev;
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

              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;

              ret = OK;

              finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
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
              info->numsectors  = priv->nsectors <<
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

          ret = gd5f_erase(dev, 0, priv->nsectors);
        }
        break;

      case MTDIOC_ECCSTATUS:
        {
          uint8_t *result = (uint8_t *)arg;
          *result =
               (priv->eccstatus & GD5F_FEATURE_ECC_MASK)
                >> GD5F_FEATURE_ECC_OFFSET;

          ret = OK;
        }
      break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name:  gd5f_eccstatusread
 ****************************************************************************/

static inline void gd5f_eccstatusread(FAR struct gd5f_dev_s *priv)
{
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->dev, GD5F_GET_FEATURE);
  SPI_SEND(priv->dev, GD5F_STATUS);
  priv->eccstatus = SPI_SEND(priv->dev, GD5F_DUMMY);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
}

/****************************************************************************
 * Name:  gd5f_enable_ecc
 ****************************************************************************/

static inline void gd5f_enable_ecc(FAR struct gd5f_dev_s *priv)
{
  uint8_t secure_otp = GD5F_SOTP_ECC;

  gd5f_lock(priv->dev);
  gd5f_writeenable(priv);

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->dev, GD5F_SET_FEATURE);
  SPI_SEND(priv->dev, GD5F_SECURE_OTP);
  SPI_SEND(priv->dev, secure_otp);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

  gd5f_writedisable(priv);
  gd5f_unlock(priv->dev);
}

/****************************************************************************
 * Name:  gd5f_unlockblocks
 ****************************************************************************/

static inline void gd5f_unlockblocks(FAR struct gd5f_dev_s *priv)
{
  uint8_t blockprotection = 0x00;

  gd5f_lock(priv->dev);
  gd5f_writeenable(priv);

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->dev, GD5F_SET_FEATURE);
  SPI_SEND(priv->dev, GD5F_BLOCK_PROTECTION);
  SPI_SEND(priv->dev, blockprotection);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

  gd5f_writedisable(priv);
  gd5f_unlock(priv->dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd5f_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.
 *   MTD devices are not registered in the file system, but are created
 *   as instances that can be bound to other functions(such as a block
 *   or character driver front end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *gd5f_initialize(FAR struct spi_dev_s *dev,
                                      uint32_t spi_devid)
{
  FAR struct gd5f_dev_s *priv;
  int ret;

  finfo("dev: %p\n", dev);

  priv = (FAR struct gd5f_dev_s *)kmm_zalloc(sizeof(struct gd5f_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure. (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = gd5f_erase;
      priv->mtd.bread  = gd5f_bread;
      priv->mtd.bwrite = gd5f_bwrite;
      priv->mtd.ioctl  = gd5f_ioctl;
      priv->mtd.name   = "gd5f";
      priv->dev        = dev;
      priv->spi_devid  = spi_devid;

      /* De-select the FLASH */

      SPI_SELECT(dev, SPIDEV_FLASH(priv->spi_devid), false);

      /* Reset the flash */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);
      SPI_SEND(priv->dev, GD5F_RESET);
      SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

      /* Wait reset complete */

      gd5f_waitstatus(priv, GD5F_SR_OIP, false);

      /* Identify the FLASH chip and get its capacity */

      ret = gd5f_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and
           * return NULL
           */

          ferr("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }

      gd5f_enable_ecc(priv);
      gd5f_waitstatus(priv, GD5F_SR_OIP, false);
      gd5f_unlockblocks(priv);
    }

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
