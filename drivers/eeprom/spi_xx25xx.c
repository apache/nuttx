/****************************************************************************
 * drivers/eeprom/spi_xx25xx.c
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
 ****************************************************************************/

/* This is a driver for SPI EEPROMs that use the same commands as the
 * 25AA160.
 *
 * Write time 5ms, 6ms for 25xx1025 (determined automatically with polling)
 * Max SPI speed is :
 * 10 MHz for -A/B/C/D/E/UID versions
 *  1 MHz for 25AA versions
 *  2 MHz for 25LC versions
 *  3 MHz for 25C  versions
 * 10 MHz for 25xxN versions where N=128 and more
 * 20 MHz for 25AA512, 25LC512, 25xx1024
 * 20 MHz for Atmel devices (>4.5V)
 * 10 MHz for Atmel devices (>2.5V)
 * 20 MHz for <1Mbit STM devices (>4.5V)
 * 16 MHz for 1Mbit  STM devices (>4.5V)
 * 10 MHz for all    STM devices (>2.5V)
 *  5 MHz for 1Mbit  STM devices (>1.8V)
 *  2 MHz for 1Mbit  STM devices (>1.7V)
 *  5 MHz for 2Mbit  STM devices
 * All devices have the same instruction set.
 *
 * The following devices should be supported:
 *
 * Manufacturer Device     Bytes PgSize SecSize AddrLen
 * Microchip
 *              25xx010A     128     16      16       1
 *              25xx020A     256     16      16       1
 *              25AA02UID    256     16      16       1
 *              25AA02E48    256     16      16       1
 *              25AA02E64    256     16      16       1
 *              25xx040      512     16      16       1+bit
 *              25xx040A     512     16      16       1+bit
 *              25xx080     1024     16      16       1
 *              25xx080A    1024     16      16       2
 *              25xx080B    1024     32      32       2
 *              25xx080C    1024     16      16       x
 *              25xx080D    1024     32      32       x
 *              25xx160     2048     16      16       2
 *              25xx160A/C  2048     16      16       2
 *              25xx160B/D  2048     32      32       2
 *              25xx160C    2048     16      16       2
 *              25xx160D    2048     32      32       2
 *              25xx320     4096     32      32       2
 *              25xx320A    4096     32      32       2
 *              25xx640     8192     32      32       2
 *              25xx640A    8192     32      32       2
 *              25xx128    16384     64      64       2
 *              25xx256    32768     64      64       2
 *              25xx512    65536    128   16384       2
 *              25xx1024  131072    256   32768       3
 * Atmel
 *              AT25010B     128      8       8       1
 *              AT25020B     256      8       8       1
 *              AT25040B     512      8       8       1+bit
 *              AT25080B    1024     32      32       2
 *              AT25160B    2048     32      32       2
 *              AT25320B    4096     32      32       2
 *              AT25640B    8192     32      32       2
 *              AT25128B   16384     64      64       2
 *              AT25256B   32768     64      64       2
 *              AT25512    65536    128     128       2
 *              AT25M01   131072    256     256       3
 * ST Microelectronics
 *              M95010       128     16      16       1
 *              M95020       256     16      16       1
 *              M95040       512     16      16       1+bit
 *              M95080      1024     32      32       2
 *              M95160      2048     32      32       2
 *              M95320      4096     32      32       2
 *              M95640      8192     32      32       2
 *              M95128     16384     64      64       2
 *              M95256     32768     64      64       2
 *              M95512     65536    128     128       2
 *              M95M01    131072    256     256       3
 *              M95M02    262144    256     256       3
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/eeprom/eeprom.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EEPROM commands
 * High bit of low nibble used for A8 in 25xx040/at25040 products
 */

#define EE25XX_CMD_WRSR  0x01
#define EE25XX_CMD_WRITE 0x02
#define EE25XX_CMD_READ  0x03
#define EE25XX_CMD_WRDIS 0x04
#define EE25XX_CMD_RDSR  0x05
#define EE25XX_CMD_WREN  0x06

/* Following commands are available via IOCTL (on devices supporting them) */

#define EEP25XX_CMD_PE   0x42
#define EEP25XX_CMD_SE   0xD8
#define EEP25XX_CMD_CE   0xC7

/* Following commands will be available some day via IOCTLs
 *   RDID      0xAB Wake up and read electronic signature (25xx512/1024)
 *   DPD       0xB9 Sleep (25xx512/1024)
 *
 * Identification page access for ST devices
 *   RDID/RDLS 0x83 Read identification page / Read ID page lock status
 *   WRID/LID  0x82 Write identification page / Lock ID page
 */

/* SR bits definitions */

#define EE25XX_SR_WIP  0x01 /* Write in Progress */
#define EE25XX_SR_WEL  0x02 /* Write Enable Latch */
#define EE25XX_SR_BP0  0x04 /* First Block Protect bit */
#define EE25XX_SR_BP1  0x08 /* Second Block Protect bit */
#define EE25XX_SR_WPEN 0x80 /* Write Protect Enable */

#define EE25XX_DUMMY   0xFF

/****************************************************************************
 * Types
 ****************************************************************************/

/* Device geometry description, compact form (2 bytes per entry) */

struct ee25xx_geom_s
{
  uint8_t bytes    : 4; /* Power of two of 128 bytes (0:128 1:256 2:512 etc) */
  uint8_t pagesize : 4; /* Power of two of   8 bytes (0:8 1:16 2:32 3:64 etc) */
  uint8_t secsize  : 4; /* Power of two of the page size */
  uint8_t addrlen  : 4; /* Number of bytes in command address field */
  uint8_t flags    : 4; /* Special address management for 25xx040, 1=A8 in inst */
};

/* Private data attached to the inode */

struct ee25xx_dev_s
{
  FAR struct spi_dev_s *spi;   /* SPI device where the EEPROM is attached   */
  uint16_t              devid; /* SPI device ID to manage CS lines in board */
  uint32_t              freq;  /* SPI bus frequency in Hz                   */

  uint32_t size;     /* in bytes, expanded from geometry                    */
  uint16_t pgsize;   /* write block size, in bytes, expanded from geometry  */
  uint32_t secsize;  /* write sector size, in bytes, expanded from geometry */
  uint16_t addrlen;  /* number of BITS in data addresses                    */

  mutex_t lock;     /* file access serialization                            */
  uint8_t refs;     /* The number of times the device has been opened       */
  uint8_t readonly; /* Flags                                                */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     ee25xx_open(FAR struct file *filep);
static int     ee25xx_close(FAR struct file *filep);
static off_t   ee25xx_seek(FAR struct file *filep, off_t offset, int whence);
static ssize_t ee25xx_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t ee25xx_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     ee25xx_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Supported device geometries.
 * One geometry can fit more than one device.
 * The user will use an enum'ed index from include/eeprom/spi_xx25xx.h
 */

static const struct ee25xx_geom_s g_ee25xx_devices[] =
{
  /* Microchip devices */

  {
    0, 1, 0, 1, 0
  }, /* 25xx010A     128     16     16      1 */
  {
    1, 1, 0, 1, 0
  }, /* 25xx020A     256     16     16      1 */
  {
    2, 1, 0, 1, 1
  }, /* 25xx040      512     16     16      1+bit */
  {
    3, 1, 0, 1, 0
  }, /* 25xx080     1024     16     16      1 */
  {
    3, 2, 0, 2, 0
  }, /* 25xx080B    1024     32     32      2 */
  {
    4, 1, 0, 2, 0
  }, /* 25xx160     2048     16     16      2 */
  {
    4, 2, 0, 2, 0
  }, /* 25xx160B/D  2048     32     32      2 */
  {
    5, 2, 0, 2, 0
  }, /* 25xx320     4096     32     32      2 */
  {
    6, 2, 0, 2, 0
  }, /* 25xx640     8192     32     32      2 */
  {
    7, 3, 0, 2, 0
  }, /* 25xx128    16384     64     64      2 */
  {
    8, 3, 0, 2, 0
  }, /* 25xx256    32768     64     64      2 */
  {
    9, 4, 7, 2, 0
  }, /* 25xx512    65536    128  16384      2 */
  {
    10, 5, 7, 3, 0
  }, /* 25xx1024  131072    256  32768      3 */

  /* Atmel devices */

  {
    0, 0, 0, 1, 0
  }, /* AT25010B     128      8      8      1 */
  {
    1, 0, 0, 1, 0
  }, /* AT25020B     256      8      8      1 */
  {
    2, 0, 0, 1, 1
  }, /* AT25040B     512      8      8      1+bit */
  {
    9, 4, 0, 2, 0
  }, /* AT25512    65536    128    128      2 */
  {
    10, 5, 0, 3, 0
  }, /* AT25M01   131072    256    256      3 */

  /* STM devices */

  {
    11, 5, 0, 3, 0
  }, /* M95M02    262144    256    256      3 */
};

/* Driver operations */

static const struct file_operations g_ee25xx_fops =
{
  ee25xx_open,  /* open */
  ee25xx_close, /* close */
  ee25xx_read,  /* read */
  ee25xx_write, /* write */
  ee25xx_seek,  /* seek */
  ee25xx_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ee25xx_lock
 *
 * Description:
 *   Lock the SPI bus associated with the driver, set its mode and frequency
 *
 * Input Parameters
 *   priv - Device structure
 *
 ****************************************************************************/

static void ee25xx_lock(FAR struct ee25xx_dev_s *priv)
{
  /* On SPI buses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the buses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusive
   * access to the SPI bus.  We will retain that exclusive access until the
   * bus is unlocked.
   */

  SPI_LOCK(priv->spi, true);

  /* After locking the SPI bus, the we also need call the setfrequency,
   * setbits, and setmode methods to make sure that the SPI is properly
   * configured for the device.  If the SPI bus is being shared, then it may
   * have been left in an incompatible state.
   */

  SPI_SETMODE(priv->spi, CONFIG_EE25XX_SPIMODE);
  SPI_SETBITS(priv->spi, 8);
  SPI_HWFEATURES(priv->spi, 0);
  SPI_SETFREQUENCY(priv->spi, priv->freq);
#ifdef CONFIG_SPI_DELAY_CONTROL
  SPI_SETDELAY(priv->spi, CONFIG_EE25XX_START_DELAY,
               CONFIG_EE25XX_STOP_DELAY, CONFIG_EE25XX_CS_DELAY,
               CONFIG_EE25XX_IFDELAY);
#endif
}

/****************************************************************************
 * Name: ee25xx_unlock
 *
 * Description:
 *   Unlock the SPI bus associated with the driver
 *
 * Input Parameters:
 *   priv - Device structure
 *
 ****************************************************************************/

static inline void ee25xx_unlock(FAR struct ee25xx_dev_s *priv)
{
  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: ee25xx_sendcmd
 *
 * Description: Send command and address as one transaction to take advantage
 * of possible faster DMA transfers. Sending byte per byte is far far slower.
 *
 ****************************************************************************/

static void ee25xx_sendcmd(FAR struct spi_dev_s *spi, uint8_t cmd,
                           uint8_t addrlen, uint32_t addr)
{
  uint8_t buf[4];
  int     cmdlen = 1;

  /* Store command */

  buf[0] = cmd;

  /* Store address according to its length */

  if (addrlen == 9)
    {
      buf[0] |= (((addr >> 8) & 1) << 3);
    }

  if (addrlen > 16)
    {
      buf[cmdlen++] = (addr >> 16) & 0xff;
    }

  if (addrlen > 9)
    {
      buf[cmdlen++] = (addr >>  8) & 0xff;
    }

  buf[cmdlen++]   =  addr        & 0xff;

  SPI_SNDBLOCK(spi, buf, cmdlen);
}

/****************************************************************************
 * Name: ee25xx_waitwritecomplete
 *
 * Description: loop until the write operation is done.
 *
 ****************************************************************************/

static void ee25xx_waitwritecomplete(struct ee25xx_dev_s *priv)
{
  uint8_t status;

  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Select this FLASH part */

      ee25xx_lock(priv);
      SPI_SELECT(priv->spi, SPIDEV_EEPROM(priv->devid), true);

      /* Send "Read Status Register (RDSR)" command */

      SPI_SEND(priv->spi, EE25XX_CMD_RDSR);

      /* Send a dummy byte to generate the clock needed to shift out the
       * status
       */

      status = SPI_SEND(priv->spi, EE25XX_DUMMY);

      /* Deselect the FLASH */

      SPI_SELECT(priv->spi, SPIDEV_EEPROM(priv->devid), false);
      ee25xx_unlock(priv);

      /* Given that writing could take up to a few milliseconds,
       * the following short delay in the "busy" case will allow
       * other peripherals to access the SPI bus.
       */

      if ((status & EE25XX_SR_WIP) != 0)
        {
          nxsched_usleep(1000);
        }
    }
  while ((status & EE25XX_SR_WIP) != 0);
}

/****************************************************************************
 * Name: ee25xx_writeenable
 *
 * Description: Enable or disable write operations.
 * This is required before any write, since a lot of operations automatically
 * disables the write latch.
 *
 ****************************************************************************/

static void ee25xx_writeenable(FAR struct ee25xx_dev_s *eedev, int enable)
{
  ee25xx_lock(eedev);
  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), true);

  SPI_SEND(eedev->spi, enable ? EE25XX_CMD_WREN : EE25XX_CMD_WRDIS);

  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), false);
  ee25xx_unlock(eedev);
}

/****************************************************************************
 * Name: ee25xx_writepage
 *
 * Description: Write data to the EEPROM, NOT crossing page boundaries.
 *
 ****************************************************************************/

static void ee25xx_writepage(FAR struct ee25xx_dev_s *eedev,
                             uint32_t devaddr,
                             FAR const char *data,
                             size_t len)
{
  ee25xx_lock(eedev);
  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), true);

  ee25xx_sendcmd(eedev->spi, EE25XX_CMD_WRITE, eedev->addrlen, devaddr);
  SPI_SNDBLOCK(eedev->spi, data, len);

  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), false);
  ee25xx_unlock(eedev);
}

/****************************************************************************
 * Name: ee25xx_eraseall
 *
 * Description:
 *   Erase all data on the device
 *
 * Input Parameters:
 *   eedev - Device structure
 *
 ****************************************************************************/

static int ee25xx_eraseall(FAR struct ee25xx_dev_s *eedev)
{
  int       ret = OK;
  off_t     offset;
  FAR char *buf;

  DEBUGASSERT(eedev);
  DEBUGASSERT(eedev->pgsize > 0);

  if (eedev->readonly)
    {
      return -EACCES;
    }

  /* Devices with different page and sector sizes support a dedicated command
   * for chip erasure
   */

  if (eedev->pgsize != eedev->secsize)
    {
      ret = nxmutex_lock(&eedev->lock);
      if (ret < 0)
        {
          return ret;
        }

      ee25xx_writeenable(eedev, true);

      ee25xx_lock(eedev);
      SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), true);

      SPI_SEND(eedev->spi, EEP25XX_CMD_CE);

      SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), false);
      ee25xx_unlock(eedev);

      ee25xx_waitwritecomplete(eedev);

      nxmutex_unlock(&eedev->lock);
    }

  /* If there is no dedicated command for erasure, write the entire memory to
   * its default state
   */

  else
    {
      buf = kmm_malloc(eedev->pgsize);
      if (buf == NULL)
        {
          ferr("ERROR: Failed to allocate memory for ee25xx eraseall\n");
          return -ENOMEM;
        }

      memset(buf, EE25XX_DUMMY, eedev->pgsize);

      ret = nxmutex_lock(&eedev->lock);
      if (ret < 0)
        {
          goto free_buffer;
        }

      for (offset = 0; offset < eedev->size; offset += eedev->pgsize)
        {
          ee25xx_writeenable(eedev, true);
          ee25xx_writepage(eedev, offset, buf, eedev->pgsize);
          ee25xx_waitwritecomplete(eedev);
        }

      nxmutex_unlock(&eedev->lock);

free_buffer:
      kmm_free(buf);
    }

  return ret;
}

/****************************************************************************
 * Name: ee25xx_erasepage
 *
 * Description:
 *   Erase 1 page of data
 *
 * Input Parameters:
 *   eedev - Device structure
 *   index - Index of the page to erase
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ee25xx_erasepage(FAR struct ee25xx_dev_s *eedev,
                            unsigned long index)
{
  int       ret = OK;
  FAR char *buf;

  DEBUGASSERT(eedev);
  DEBUGASSERT(eedev->pgsize > 0);

  if (eedev->readonly)
    {
      return -EACCES;
    }

  if (index >= (eedev->size / eedev->pgsize))
    {
      return -EFBIG;
    }

  /* Devices with different page and sector sizes support a dedicated command
   * for page erasure
   */

  if (eedev->pgsize != eedev->secsize)
    {
      ret = nxmutex_lock(&eedev->lock);
      if (ret < 0)
        {
          return ret;
        }

      ee25xx_writeenable(eedev, true);

      ee25xx_lock(eedev);
      SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), true);

      ee25xx_sendcmd(eedev->spi, EEP25XX_CMD_PE, eedev->addrlen,
                     index * eedev->pgsize);

      SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), false);
      ee25xx_unlock(eedev);

      ee25xx_waitwritecomplete(eedev);

      nxmutex_unlock(&eedev->lock);
    }

  /* If there is no dedicated command for erasure, write the page to its
   * default state
   */

  else
    {
      buf = kmm_malloc(eedev->pgsize);
      if (buf == NULL)
        {
          ferr("ERROR: Failed to allocate memory for ee25xx_erasepage\n");
          return -ENOMEM;
        }

      memset(buf, EE25XX_DUMMY, eedev->pgsize);

      ret = nxmutex_lock(&eedev->lock);
      if (ret < 0)
        {
          goto free_buffer;
        }

      ee25xx_writeenable(eedev, true);
      ee25xx_writepage(eedev, index * eedev->pgsize, buf, eedev->pgsize);
      ee25xx_waitwritecomplete(eedev);

      nxmutex_unlock(&eedev->lock);

free_buffer:
      kmm_free(buf);
    }

  return ret;
}

/****************************************************************************
 * Name: ee25xx_erasesector
 *
 * Description:
 *   Erase 1 sector of data
 *
 * Input Parameters:
 *   eedev - Device structure
 *   index - Index of the sector to erase
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ee25xx_erasesector(FAR struct ee25xx_dev_s *eedev,
                              unsigned long index)
{
  int ret;

  DEBUGASSERT(eedev);
  DEBUGASSERT(eedev->secsize > 0);

  if (eedev->pgsize == eedev->secsize)
    {
      return ee25xx_erasepage(eedev, index);
    }

  if (eedev->readonly)
    {
      return -EACCES;
    }

  if (index >= (eedev->size / eedev->secsize))
    {
      return -EFBIG;
    }

  ret = nxmutex_lock(&eedev->lock);
  if (ret < 0)
    {
      return ret;
    }

  ee25xx_writeenable(eedev, true);

  ee25xx_lock(eedev);
  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), true);

  ee25xx_sendcmd(eedev->spi, EEP25XX_CMD_SE, eedev->addrlen,
                 index * eedev->secsize);

  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), false);
  ee25xx_unlock(eedev);

  ee25xx_waitwritecomplete(eedev);

  nxmutex_unlock(&eedev->lock);

  return ret;
}

/****************************************************************************
 * Name: ee25xx_rdsr
 *
 * Description: Read status register.
 *
 ****************************************************************************/

static uint8_t ee25xx_rdsr(FAR struct ee25xx_dev_s *eedev)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = EE25XX_CMD_RDSR;
  tx[1] = EE25XX_DUMMY;

  ee25xx_lock(eedev);
  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), true);
  SPI_EXCHANGE(eedev->spi, tx, rx, 2);
  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), false);
  ee25xx_unlock(eedev);
  return rx[1];
}

/****************************************************************************
 * Name: ee25xx_wrsr
 *
 * Description: Write status register.
 *
 ****************************************************************************/

static void ee25xx_wrsr(FAR struct ee25xx_dev_s *eedev, uint8_t what)
{
  uint8_t tx[2];

  tx[0] = EE25XX_CMD_WRSR;
  tx[1] = what;

  ee25xx_lock(eedev);
  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), true);
  SPI_EXCHANGE(eedev->spi, tx, NULL, 2);
  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), false);
  ee25xx_unlock(eedev);
}

/****************************************************************************
 * Driver Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ee25xx_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int ee25xx_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ee25xx_dev_s *eedev;
  int ret = OK;

  DEBUGASSERT(inode->i_private);
  eedev = inode->i_private;

  ret = nxmutex_lock(&eedev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the reference count */

  if ((eedev->refs + 1) == 0)
    {
      ret = -EMFILE;
    }
  else
    {
      eedev->refs += 1;
    }

  nxmutex_unlock(&eedev->lock);
  return ret;
}

/****************************************************************************
 * Name: ee25xx_close
 *
 * Description: Close the block device
 *
 ****************************************************************************/

static int ee25xx_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ee25xx_dev_s *eedev;
  int ret = OK;

  DEBUGASSERT(inode->i_private);
  eedev = inode->i_private;

  ret = nxmutex_lock(&eedev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the reference count. I want the entire close operation
   * to be atomic wrt other driver operations.
   */

  if (eedev->refs == 0)
    {
      ret = -EIO;
    }
  else
    {
      eedev->refs -= 1;
    }

  nxmutex_unlock(&eedev->lock);
  return ret;
}

/****************************************************************************
 * Name: ee25xx_seek
 *
 * Remark: Copied from bchlib
 *
 ****************************************************************************/

static off_t ee25xx_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct ee25xx_dev_s *eedev;
  off_t                   newpos;
  int                     ret;
  FAR struct inode        *inode = filep->f_inode;

  DEBUGASSERT(inode->i_private);
  eedev = inode->i_private;

  ret = nxmutex_lock(&eedev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Determine the new, requested file position */

  switch (whence)
    {
    case SEEK_CUR:
      newpos = filep->f_pos + offset;
      break;

    case SEEK_SET:
      newpos = offset;
      break;

    case SEEK_END:
      newpos = eedev->size + offset;
      break;

    default:

      /* Return EINVAL if the whence argument is invalid */

      nxmutex_unlock(&eedev->lock);
      return -EINVAL;
    }

  /* Opengroup.org:
   *
   *  "The lseek() function shall allow the file offset to be set beyond the
   *  end of the existing data in the file. If data is later written at this
   *  point, subsequent reads of data in the gap shall return bytes with the
   *  value 0 until data is actually written into the gap."
   *
   * We can conform to the first part, but not the second.
   * But return -EINVAL if
   *
   *  "...the resulting file offset would be negative for a regular file,
   *  block special file, or directory."
   */

  if (newpos >= 0)
    {
      filep->f_pos = newpos;
      ret = newpos;
    }
  else
    {
      ret = -EINVAL;
    }

  nxmutex_unlock(&eedev->lock);
  return ret;
}

/****************************************************************************
 * Name: ee25xx_read
 ****************************************************************************/

static ssize_t ee25xx_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  FAR struct ee25xx_dev_s *eedev;
  FAR struct inode        *inode = filep->f_inode;
  int ret;

  DEBUGASSERT(inode->i_private);
  eedev = inode->i_private;

  ret = nxmutex_lock(&eedev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* trim len if read would go beyond end of device */

  if ((filep->f_pos + len) > eedev->size)
    {
      len = eedev->size - filep->f_pos;
    }

  ee25xx_lock(eedev);
  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), true);

  /* STM32F4Disco: There is a 25 us delay here */

  ee25xx_sendcmd(eedev->spi, EE25XX_CMD_READ, eedev->addrlen, filep->f_pos);

  /* STM32F4Disco: There is a 42 us delay here */

  SPI_RECVBLOCK(eedev->spi, buffer, len);

  /* STM32F4Disco: There is a 20 us delay here */

  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), false);
  ee25xx_unlock(eedev);

  /* Update the file position */

  filep->f_pos += len;
  nxmutex_unlock(&eedev->lock);
  return len;
}

/****************************************************************************
 * Name: ee25xx_write
 ****************************************************************************/

static ssize_t ee25xx_write(FAR struct file *filep, FAR const char *buffer,
                            size_t len)
{
  FAR struct ee25xx_dev_s *eedev;
  size_t                  cnt;
  int                     pageoff;
  FAR struct inode        *inode = filep->f_inode;
  int                     ret    = -EACCES;

  DEBUGASSERT(inode->i_private);
  eedev = inode->i_private;

  if (eedev->readonly)
    {
      return ret;
    }

  /* Forbid writes past the end of the device */

  if (filep->f_pos >= eedev->size)
    {
      return -EFBIG;
    }

  /* Clamp len to avoid crossing the end of the memory */

  if ((len + filep->f_pos) > eedev->size)
    {
      len = eedev->size - filep->f_pos;
    }

  ret = nxmutex_lock(&eedev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* From this point no failure cannot be detected anymore.
   * The user should verify the write by rereading memory.
   */

  ret = len; /* save number of bytes written */

  /* Writes can't happen in a row like the read does.
   * The EEPROM is made of pages, and write sequences
   * cannot cross page boundaries. So every time the last
   * byte of a page is programmed, the SPI transaction is
   * stopped, and the status register is read until the
   * write operation has completed.
   */

  /* First, write some page-unaligned data */

  pageoff = filep->f_pos & (eedev->pgsize - 1);
  cnt     = eedev->pgsize - pageoff;
  if (cnt > len)
    {
      cnt = len;
    }

  if (pageoff > 0)
    {
      ee25xx_writeenable(eedev, true);
      ee25xx_writepage(eedev, filep->f_pos, buffer, cnt);
      ee25xx_waitwritecomplete(eedev);
      len          -= cnt;
      buffer       += cnt;
      filep->f_pos += cnt;
    }

  /* Then, write remaining bytes at page-aligned addresses */

  while (len > 0)
    {
      cnt = len;
      if (cnt > eedev->pgsize)
        {
          cnt = eedev->pgsize;
        }

      ee25xx_writeenable(eedev, true);
      ee25xx_writepage(eedev, filep->f_pos, buffer, cnt);
      ee25xx_waitwritecomplete(eedev);
      len          -= cnt;
      buffer       += cnt;
      filep->f_pos += cnt;
    }

  nxmutex_unlock(&eedev->lock);
  return ret;
}

/****************************************************************************
 * Name: ee25xx_ioctl
 *
 * Description: TODO: Read device ID.
 * This is completely optional and only applies to bigger devices.
 *
 ****************************************************************************/

static int ee25xx_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct ee25xx_dev_s *eedev;
  FAR struct inode        *inode = filep->f_inode;
  int                      ret   = -EINVAL;

  DEBUGASSERT(inode->i_private);
  eedev = inode->i_private;

  switch (cmd)
    {
      case EEPIOC_GEOMETRY:
        {
          FAR struct eeprom_geometry_s *geo =
            (FAR struct eeprom_geometry_s *)arg;
          if (geo != NULL)
            {
              geo->npages   = 0;
              geo->pagesize = eedev->pgsize;
              geo->sectsize = eedev->secsize;

              if (eedev->pgsize > 0)
                {
                  geo->npages = eedev->size / eedev->pgsize;
                }

              ret = OK;
            }
        }
        break;

      case EEPIOC_SETSPEED:
        {
          ret = nxmutex_lock(&eedev->lock);
          if (ret == OK)
          {
            eedev->freq = (uint32_t)arg;
            nxmutex_unlock(&eedev->lock);
          }
        }
        break;

      case EEPIOC_PAGEERASE:
        ret = ee25xx_erasepage(eedev, arg);
        break;

      case EEPIOC_SECTORERASE:
        ret = ee25xx_erasesector(eedev, arg);
        break;

      case EEPIOC_CHIPERASE:
        ret = ee25xx_eraseall(eedev);
        break;

      case EEPIOC_BLOCKPROTECT:
        {
          /* Get current value of status register. */

          ret = ee25xx_rdsr(eedev);

          /* Clear Block Protection bits. */

          ret &= ~(EE25XX_SR_BP0 | EE25XX_SR_BP1);

          /* Set Block Protection bits. */

          ret |= ((uint8_t)arg << 2) & (EE25XX_SR_BP0 | EE25XX_SR_BP1);

          /* Write status register. */

          ee25xx_writeenable(eedev, true);
          ee25xx_wrsr(eedev, ret);
          ee25xx_waitwritecomplete(eedev);
          ret = OK;
        }
        break;

      default:
        ret = -ENOTTY;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ee25xx_initialize
 *
 * Description:
 *   Bind an EEPROM driver to an SPI bus. The user MUST provide a description
 *   of the device geometry, since it is not possible to read this
 *   information from the device (contrary to the SPI flash devices).
 *
 * Parameters:
 *   dev       - Pointer to the SPI device instance
 *   spi_devid - SPI device ID to manage CS lines in board
 *   devname   - Device name
 *   devtype   - 25xx device type, the geometry is derived from it
 *   readonly  - Sets driver to be readonly
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int ee25xx_initialize(FAR struct spi_dev_s *dev, uint16_t spi_devid,
                      FAR char *devname, int devtype, int readonly)
{
  FAR struct ee25xx_dev_s *eedev;

  /* Check device type early */

  if ((devtype < 0) ||
      (devtype >= sizeof(g_ee25xx_devices) / sizeof(g_ee25xx_devices[0])))
    {
      return -EINVAL;
    }

  eedev = kmm_zalloc(sizeof(struct ee25xx_dev_s));

  if (!eedev)
    {
      return -ENOMEM;
    }

  nxmutex_init(&eedev->lock);

  eedev->spi      = dev;
  eedev->devid    = spi_devid;
  eedev->freq     = CONFIG_EE25XX_FREQUENCY;
  eedev->size     =           128 << g_ee25xx_devices[devtype].bytes;
  eedev->pgsize   =             8 << g_ee25xx_devices[devtype].pagesize;
  eedev->secsize  = eedev->pgsize << g_ee25xx_devices[devtype].secsize;
  eedev->addrlen  =                  g_ee25xx_devices[devtype].addrlen << 3;
  if ((g_ee25xx_devices[devtype].flags & 1))
    {
      eedev->addrlen = 9;
    }

  eedev->readonly = !!readonly;

  finfo("EEPROM device %s, %"PRIu32" bytes, "
        "%u per page, addrlen %u, readonly %d\n",
       devname, eedev->size, eedev->pgsize, eedev->addrlen, eedev->readonly);

  return register_driver_with_size(devname, &g_ee25xx_fops, 0666, eedev,
                                   eedev->size);
}
