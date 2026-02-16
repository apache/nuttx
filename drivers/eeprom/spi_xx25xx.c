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
#ifdef CONFIG_MTD_AT25EE
  struct eeprom_dev_s eepdev; /* Must appear at the beginning to make it
                               * possible to cast between struct eeprom_dev_s
                               * and struct ee25xx_dev_s
                               */
#endif

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

/* SPI lock/unlock */

static void ee25xx_lock(FAR struct ee25xx_dev_s *eedev);
static void ee25xx_unlock(FAR struct ee25xx_dev_s *eedev);

/* Trigger a read/write operation */

static void ee25xx_sendcmd(FAR struct spi_dev_s *spi, uint8_t cmd,
                           uint8_t addrlen, uint32_t addr);

/* Write/erase related functions */

static void    ee25xx_waitwritecomplete(FAR struct ee25xx_dev_s *eedev);
static void    ee25xx_writeenable(FAR struct ee25xx_dev_s *spi, int enable);
static void    ee25xx_writepage(FAR struct ee25xx_dev_s *eedev,
                                uint32_t devaddr, FAR const char *data,
                                size_t len);
static int     ee25xx_eraseall(FAR struct ee25xx_dev_s *eedev);
static int     ee25xx_erasepage(FAR struct ee25xx_dev_s *eedev,
                                unsigned long index);
static int     ee25xx_erasesector(FAR struct ee25xx_dev_s *eedev,
                                  unsigned long index);
static uint8_t ee25xx_rdsr(FAR struct ee25xx_dev_s *eedev);
static void    ee25xx_wrsr(FAR struct ee25xx_dev_s *eedev, uint8_t what);

/* Initialization function */

static int ee25xx_populatedev(FAR struct ee25xx_dev_s **eedev,
                              FAR struct spi_dev_s *dev, uint16_t spi_devid,
                              enum eeprom_25xx_e devtype, int readonly);

/* File operations */

static int     ee25xx_open(FAR struct file *filep);
static int     ee25xx_close(FAR struct file *filep);
static off_t   ee25xx_seek(FAR struct file *filep, off_t offset, int whence);
static ssize_t ee25xx_fread(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t ee25xx_fwrite(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int     ee25xx_fioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/* EEPROM operations (only needed for the MTD driver) */

#ifdef CONFIG_MTD_AT25EE
static ssize_t ee25xx_eepread(FAR struct eeprom_dev_s *eedev, off_t offset,
                              size_t nbytes, FAR uint8_t *buffer);
static ssize_t ee25xx_eepwrite(FAR struct eeprom_dev_s *eedev, off_t offset,
                               size_t nbytes, FAR const uint8_t *buffer);
static int     ee25xx_eepioctl(FAR struct eeprom_dev_s *dev, int cmd,
                               unsigned long arg);
#endif

/* Actual read, write and ioctl functions */

static ssize_t ee25xx_read(FAR struct ee25xx_dev_s *eedev, off_t offset,
                            size_t nbytes, FAR char *buffer);
static ssize_t ee25xx_write(FAR struct ee25xx_dev_s *eedev, off_t offset,
                            size_t nbytes, FAR const char *buffer);
static int     ee25xx_ioctl(FAR struct ee25xx_dev_s *dev, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Supported device geometries.
 * One geometry can fit more than one device.
 * The user will use an enum'ed index from include/eeprom/eeprom.h
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
  ee25xx_open,   /* open */
  ee25xx_close,  /* close */
  ee25xx_fread,  /* read */
  ee25xx_fwrite, /* write */
  ee25xx_seek,   /* seek */
  ee25xx_fioctl, /* ioctl */
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
 * Description:
 *   Send command and address on SPI as one transaction to take advantage of
 *   possible faster DMA transfers. Sending byte per byte is FAR FAR slower.
 *
 * Input Parameters:
 *   spi     - SPI bus
 *   cmd     - Command to be sent
 *   addrlen - Length of the address field in bits
 *   addr    - Address to be sent
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
 * Description:
 *   Loop until the write operation is done
 *
 * Input Parameters:
 *   eedev - Device structure
 *
 ****************************************************************************/

static void ee25xx_waitwritecomplete(FAR struct ee25xx_dev_s *eedev)
{
  uint8_t status;

  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Select this FLASH part */

      ee25xx_lock(eedev);
      SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), true);

      /* Send "Read Status Register (RDSR)" command */

      SPI_SEND(eedev->spi, EE25XX_CMD_RDSR);

      /* Send a dummy byte to generate the clock needed to shift out the
       * status
       */

      status = SPI_SEND(eedev->spi, EE25XX_DUMMY);

      /* Deselect the FLASH */

      SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), false);
      ee25xx_unlock(eedev);

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
 * Description:
 *   Enable or disable write operations.
 *   This is required before any write, since a lot of operations
 *   automatically disable the write latch.
 *
 * Input Parameters:
 *   eedev  - Device structure
 *   enable - Whether to enable or disable the write latch
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
 * Description:
 *   Write data to the EEPROM, NOT crossing page boundaries.
 *
 * Input Parameters:
 *   eedev   - Device structure
 *   devaddr - Address where to start writing
 *   data    - Data to be written
 *   len     - Length of the data to be written in bytes
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
      return -EROFS;
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
      return -EROFS;
    }

  if (index >= (eedev->size / eedev->pgsize))
    {
      return -EFAULT;
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

  if (eedev->readonly)
    {
      return -EROFS;
    }

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
      return -EFAULT;
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
 * Description:
 *   Read status register
 *
 * Input Parameters:
 *   eedev - Device structure
 *
 * Returned Value:
 *   Content of the status register
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
 * Description:
 *   Write status register
 *
 * Input Parameters:
 *   eedev - Device structure
 *   what  - Value to be written to the status register
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
 * Description:
 *   Open the character device
 *
 * Input Parameters
 *   filep - File structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ee25xx_open(FAR struct file *filep)
{
  FAR struct inode *       inode = filep->f_inode;
  FAR struct ee25xx_dev_s *eedev;
  int                      ret;

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
 * Description:
 *   Close the character device
 *
 * Input Parameters
 *   filep - File structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ee25xx_close(FAR struct file *filep)
{
  FAR struct inode *       inode = filep->f_inode;
  FAR struct ee25xx_dev_s *eedev;
  int                      ret;

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
 * Description:
 *   Reposition the offset of the given file structure
 *
 * Input Parameters
 *   filep  - File structure
 *   offset - Offset with respect to whence where to reposition
 *   whence - Reference point for the offset
 *
 * Returned Value:
 *   Offset location in bytes from the beginning of the EEPROM on success.
 *   Negated errno value on failure.
 *
 ****************************************************************************/

static off_t ee25xx_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct ee25xx_dev_s *eedev;
  off_t                    newpos;
  int                      ret;
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
 * Name: ee25xx_fread
 *
 * Description:
 *   Read bytes from the EEPROM at the file structure offset
 *   The offset is updated on success
 *
 * Input Parameters:
 *   filep  - File structure
 *   buffer - Buffer that will be populated with the read data
 *   len    - Number of bytes to read
 *
 * Returned Value:
 *   Number of read bytes on success; a negated error code on failure
 *
 ****************************************************************************/

static ssize_t ee25xx_fread(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  ssize_t ret;

  DEBUGASSERT(filep->f_inode);
  DEBUGASSERT(filep->f_inode->i_private);

  ret = ee25xx_read(filep->f_inode->i_private, filep->f_pos, len, buffer);

  if (ret > 0)
    {
      /* Update the file position */

      filep->f_pos += ret;
    }

  return ret;
}

/****************************************************************************
 * Name: ee25xx_eepread
 *
 * Description:
 *   Read bytes from the EEPROM at the given offset
 *
 * Input Parameters:
 *   dev    - Generic EEPROM device structure
 *   offset - Absolute offset where to start reading data
 *   nbytes - Number of bytes to read
 *   buffer - Buffer that will be populated with the read data
 *
 * Returned Value:
 *   Number of read bytes on success; a negated error code on failure
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_AT25EE
static ssize_t ee25xx_eepread(FAR struct eeprom_dev_s *dev, off_t offset,
                              size_t nbytes, FAR uint8_t *buffer)
{
  return ee25xx_read((FAR struct ee25xx_dev_s *)dev, offset, nbytes,
                     (FAR char *)buffer);
}
#endif

/****************************************************************************
 * Name: ee25xx_read
 *
 * Description:
 *   Read bytes from the EEPROM at the given offset
 *
 * Input Parameters:
 *   eedev  - Device structure
 *   offset - Absolute offset where to start reading data
 *   nbytes - Number of bytes to read
 *   buffer - Buffer that will be populated with the read data
 *
 * Returned Value:
 *   Number of read bytes on success; a negated error code on failure
 *
 ****************************************************************************/

static ssize_t ee25xx_read(FAR struct ee25xx_dev_s *eedev, off_t offset,
                            size_t nbytes, FAR char *buffer)
{
  if (offset > eedev->size)
    {
      return 0;
    }

  const int ret = nxmutex_lock(&eedev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* trim nbytes if read would go beyond end of device */

  if ((offset + nbytes) > eedev->size)
    {
      nbytes = eedev->size - offset;
    }

  ee25xx_lock(eedev);
  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), true);

  ee25xx_sendcmd(eedev->spi, EE25XX_CMD_READ, eedev->addrlen, offset);

  SPI_RECVBLOCK(eedev->spi, buffer, nbytes);

  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(eedev->devid), false);
  ee25xx_unlock(eedev);

  nxmutex_unlock(&eedev->lock);
  return nbytes;
}

/****************************************************************************
 * Name: ee25xx_fwrite
 *
 * Description:
 *   Write bytes to the EEPROM at the file structure offset
 *   The offset is updated on success
 *
 * Input Parameters:
 *   filep  - File structure
 *   buffer - Buffer containing the data to be written
 *   len    - Number of bytes to write
 *
 * Returned Value:
 *   Number of written bytes on success; a negated error code on failure
 *
 ****************************************************************************/

static ssize_t ee25xx_fwrite(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  ssize_t ret;

  DEBUGASSERT(filep->f_inode);
  DEBUGASSERT(filep->f_inode->i_private);

  ret = ee25xx_write(filep->f_inode->i_private, filep->f_pos, len, buffer);

  if (ret > 0)
    {
      /* Update the file position */

      filep->f_pos += ret;
    }

  return ret;
}

/****************************************************************************
 * Name: ee25xx_eepwrite
 *
 * Description:
 *   Write bytes to the EEPROM at the given offset
 *
 * Input Parameters:
 *   dev    - Generic EEPROM device structure
 *   offset - Absolute offset where to start writing data
 *   nbytes - Number of bytes to write
 *   buffer - Data to be written
 *
 * Returned Value:
 *   Number of written bytes on success; a negated error code on failure
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_AT25EE
static ssize_t ee25xx_eepwrite(FAR struct eeprom_dev_s *dev, off_t offset,
                               size_t nbytes, FAR const uint8_t *buffer)
{
  return ee25xx_write((FAR struct ee25xx_dev_s *)dev, offset, nbytes,
                      (FAR const char *)buffer);
}
#endif

/****************************************************************************
 * Name: ee25xx_write
 *
 * Description:
 *   Write bytes to the EEPROM at the given offset
 *
 * Input Parameters:
 *   dev    - Device structure
 *   offset - Absolute offset where to start writing data
 *   nbytes - Number of bytes to write
 *   buffer - Data to be written
 *
 * Returned Value:
 *   Number of written bytes on success; a negated error code on failure
 *
 ****************************************************************************/

static ssize_t ee25xx_write(FAR struct ee25xx_dev_s *eedev, off_t offset,
                            size_t nbytes, FAR const char *buffer)
{
  size_t cnt;
  int    pageoff;
  int    ret;

  if (eedev->readonly)
    {
      return -EROFS;
    }

  /* Forbid writes past the end of the device */

  if (offset >= eedev->size)
    {
      return 0;
    }

  /* Clamp nbytes to avoid crossing the end of the memory */

  if ((nbytes + offset) > eedev->size)
    {
      nbytes = eedev->size - offset;
    }

  ret = nxmutex_lock(&eedev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* From this point no failure cannot be detected anymore.
   * The user should verify the write by rereading memory.
   */

  ret = nbytes; /* save number of bytes written */

  /* Writes can't happen in a row like the read does.
   * The EEPROM is made of pages, and write sequences
   * cannot cross page boundaries. So every time the last
   * byte of a page is programmed, the SPI transaction is
   * stopped, and the status register is read until the
   * write operation has completed.
   */

  /* First, write some page-unaligned data */

  pageoff = offset & (eedev->pgsize - 1);
  cnt     = eedev->pgsize - pageoff;
  if (cnt > nbytes)
    {
      cnt = nbytes;
    }

  if (pageoff > 0)
    {
      ee25xx_writeenable(eedev, true);
      ee25xx_writepage(eedev, offset, buffer, cnt);
      ee25xx_waitwritecomplete(eedev);
      nbytes -= cnt;
      buffer += cnt;
      offset += cnt;
    }

  /* Then, write remaining bytes at page-aligned addresses */

  while (nbytes > 0)
    {
      cnt = nbytes;
      if (cnt > eedev->pgsize)
        {
          cnt = eedev->pgsize;
        }

      ee25xx_writeenable(eedev, true);
      ee25xx_writepage(eedev, offset, buffer, cnt);
      ee25xx_waitwritecomplete(eedev);
      nbytes -= cnt;
      buffer += cnt;
      offset += cnt;
    }

  nxmutex_unlock(&eedev->lock);
  return ret;
}

/****************************************************************************
 * Name: ee25xx_fioctl
 *
 * Description:
 *   Ioctl control for the device, the same as ioctl().
 *
 * Input Parameters:
 *   filep - File structure
 *   cmd   - Ioctl command.
 *   arg   - Ioctl argument.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ee25xx_fioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  DEBUGASSERT(filep->f_inode);
  DEBUGASSERT(filep->f_inode->i_private);
  return ee25xx_ioctl(filep->f_inode->i_private, cmd, arg);
}

/****************************************************************************
 * Name: ee25xx_eepioctl
 *
 * Description:
 *   Ioctl control for the device, the same as ioctl().
 *
 * Input Parameters:
 *   filep - Generic EEPROM device structure
 *   cmd   - Ioctl command.
 *   arg   - Ioctl argument.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_AT25EE
static int ee25xx_eepioctl(FAR struct eeprom_dev_s *dev, int cmd,
                           unsigned long arg)
{
  return ee25xx_ioctl((FAR struct ee25xx_dev_s *)dev, cmd, arg);
}
#endif

/****************************************************************************
 * Name: ee25xx_fioctl
 *
 * Description:
 *   Ioctl control for the device, the same as ioctl().
 *
 * Input Parameters:
 *   filep - Device structure
 *   cmd   - Ioctl command.
 *   arg   - Ioctl argument.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ee25xx_ioctl(FAR struct ee25xx_dev_s *eedev, int cmd,
                        unsigned long arg)
{
  int ret = -EINVAL;

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
 * Name: ee25xx_populatedev
 *
 * Description:
 *   Populate a device structure
 *
 * Input Parameters:
 *   eedev     - Pointer to the device structure to be populated
 *   dev       - SPI device instance
 *   spi_devid - SPI device ID to manage CS lines in board
 *   devtype   - 25xx device type, the geometry is derived from it
 *   readonly  - Sets driver to be readonly
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ee25xx_populatedev(FAR struct ee25xx_dev_s **eedev,
                              FAR struct spi_dev_s *dev, uint16_t spi_devid,
                              enum eeprom_25xx_e devtype, int readonly)
{
  /* Check the device type early */

  const int devtype_idx = (int)devtype;
  if (devtype_idx >=
      (sizeof(g_ee25xx_devices) / sizeof(g_ee25xx_devices[0])))
    {
      return -EINVAL;
    }

  *eedev = kmm_zalloc(sizeof(struct ee25xx_dev_s));

  if (!(*eedev))
    {
      return -ENOMEM;
    }

  nxmutex_init(&(*eedev)->lock);

  (*eedev)->spi      = dev;
  (*eedev)->devid    = spi_devid;
  (*eedev)->freq     = CONFIG_EE25XX_FREQUENCY;
  (*eedev)->readonly = !!readonly;

  (*eedev)->size    = 128 << g_ee25xx_devices[devtype_idx].bytes;
  (*eedev)->pgsize  =   8 << g_ee25xx_devices[devtype_idx].pagesize;
  (*eedev)->addrlen =        g_ee25xx_devices[devtype_idx].addrlen << 3;
  if ((g_ee25xx_devices[devtype_idx].flags & 1))
    {
      ++(*eedev)->addrlen;
    }

  (*eedev)->secsize =
    (*eedev)->pgsize << g_ee25xx_devices[devtype_idx].secsize;

  return OK;
}

/****************************************************************************
 * Name: ee25xx_initialize
 *
 * Description:
 *   Bind an EEPROM driver to an SPI bus
 *
 * Parameters:
 *   dev       - Pointer to the SPI device instance
 *   spi_devid - SPI device ID to manage CS lines in board
 *   devname   - Device name
 *   devtype   - 25xx device type, the geometry is derived from it
 *   readonly  - Sets driver to be readonly
 *
 * Returned Values:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ee25xx_initialize(FAR struct spi_dev_s *dev, uint16_t spi_devid,
                      FAR char *devname, enum eeprom_25xx_e devtype,
                      int readonly)
{
  FAR struct ee25xx_dev_s *eedev;
  int                      ret;

  ret = ee25xx_populatedev(&eedev, dev, spi_devid, devtype, readonly);

  if (ret != OK)
    {
      return ret;
    }

  finfo("EEPROM device %s, %"PRIu32" bytes, %u per page, addrlen %u, "
      "readonly %d\n", devname, eedev->size, eedev->pgsize,
      eedev->addrlen, eedev->readonly);

  return register_driver(devname, &g_ee25xx_fops, 0666, eedev);
}

/****************************************************************************
 * Name: ee25xx_initialize_as_eeprom_dev
 *
 * Description:
 *   Create an initialized EEPROM device instance for an xx25xx SPI EEPROM.
 *   The device is not registered in the file system, but created as an
 *   instance that can be bound to other functions.
 *
 * Parameters:
 *   dev       - Pointer to the SPI device instance
 *   spi_devid - SPI device ID to manage CS lines in board
 *   devtype   - 25xx device type, the geometry is derived from it
 *   readonly  - Sets driver to be readonly
 *
 * Returned Values:
 *   Initialised device structure (success) or NULL (fail)
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_AT25EE

FAR struct eeprom_dev_s *ee25xx_initialize_as_eeprom_dev(
                           FAR struct spi_dev_s *dev, uint16_t spi_devid,
                           enum eeprom_25xx_e devtype, int readonly)
{
  FAR struct ee25xx_dev_s *eedev;

  if (ee25xx_populatedev(&eedev, dev, spi_devid, devtype, readonly) != OK)
    {
      return NULL;
    }

  /* Populate the eeprom_dev_s struct */

  eedev->eepdev.read  = ee25xx_eepread;
  eedev->eepdev.write = ee25xx_eepwrite;
  eedev->eepdev.ioctl = ee25xx_eepioctl;

  return (FAR struct eeprom_dev_s *)eedev;
}

#endif

/****************************************************************************
 * Name: ee25xx_teardown
 *
 * Description:
 *   Teardown a previously created ee25xx device.
 *
 * Input Parameters:
 *   dev - Pointer to the driver instance.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_AT25EE

void ee25xx_teardown(FAR struct eeprom_dev_s *dev)
{
  kmm_free(dev);
}

#endif
