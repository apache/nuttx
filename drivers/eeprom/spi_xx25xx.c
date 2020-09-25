/****************************************************************************
 * drivers/eeprom/spi_xx25xx.c
 *
 *   Copyright (C) 2018 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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
 * Manufacturer Device     Bytes PgSize AddrLen
 * Microchip
 *              25xx010A     128   16     1
 *              25xx020A     256   16     1
 *              25AA02UID    256   16     1
 *              25AA02E48    256   16     1
 *              25AA02E64    256   16     1
 *              25xx040      512   16     1+bit
 *              25xx040A     512   16     1+bit
 *              25xx080     1024   16     1
 *              25xx080A    1024   16     2
 *              25xx080B    1024   32     2
 *              25xx080C    1024   16     x
 *              25xx080D    1024   32     x
 *              25xx160     2048   16     2
 *              25xx160A/C  2048   16     2    TESTED
 *              25xx160B/D  2048   32     2
 *              25xx160C    2048   16     2
 *              25xx160D    2048   32     2
 *              25xx320     4096   32     2
 *              25xx320A    4096   32     2
 *              25xx640     8192   32     2
 *              25xx640A    8192   32     2
 *              25xx128    16384   64     2
 *              25xx256    32768   64     2
 *              25xx512    65536  128     2
 *              25xx1024  131072  256     3
 * Atmel
 *              AT25010B     128    8     1
 *              AT25020B     256    8     1
 *              AT25040B     512    8     1+bit
 *              AT25080B    1024   32     2
 *              AT25160B    2048   32     2
 *              AT25320B    4096   32     2
 *              AT25640B    8192   32     2
 *              AT25128B   16384   64     2
 *              AT25256B   32768   64     2
 *              AT25512    65536  128     2
 *              AT25M01   131072  256     3
 * ST Microelectronics
 *              M95010       128   16     1
 *              M95020       256   16     1
 *              M95040       512   16     1+bit
 *              M95080      1024   32     2
 *              M95160      2048   32     2
 *              M95320      4096   32     2
 *              M95640      8192   32     2
 *              M95128     16384   64     2
 *              M95256     32768   64     2
 *              M95512     65536  128     2
 *              M95M01    131072  256     3
 *              M95M02    262144  256     3
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/fs/fs.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EE25XX_SPIMODE
#  define CONFIG_EE25XX_SPIMODE 0
#endif

/* EEPROM commands
 * High bit of low nibble used for A8 in 25xx040/at25040 products
 */

#define EE25XX_CMD_WRSR  0x01
#define EE25XX_CMD_WRITE 0x02
#define EE25XX_CMD_READ  0x03
#define EE25XX_CMD_WRDIS 0x04
#define EE25XX_CMD_RDSR  0x05
#define EE25XX_CMD_WREN  0x06

/* Following commands will be available some day via IOCTLs
 *   PE        0x42 Page erase (25xx512/1024)
 *   SE        0xD8 Sector erase (25xx512/1024)
 *   CE        0xC7 Chip erase (25xx512/1024)
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
  uint8_t addrlen  : 4; /* Number of bytes in command address field */
  uint8_t flags    : 4; /* Special address management for 25xx040, 1=A8 in inst */
};

/* Private data attached to the inode */

struct ee25xx_dev_s
{
  struct spi_dev_s *spi;     /* SPI device where the EEPROM is attached */
  uint32_t         size;     /* in bytes, expanded from geometry */
  uint16_t         pgsize;   /* write block size, in bytes, expanded from geometry */
  uint16_t         addrlen;  /* number of BITS in data addresses */
  sem_t            sem;      /* file access serialization */
  uint8_t          refs;     /* The number of times the device has been opened */
  uint8_t          readonly; /* Flags */
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
    0, 1, 1, 0
  }, /* 25xx010A     128   16     1 */
  {
    1, 1, 1, 0
  }, /* 25xx020A     256   16     1 */
  {
    2, 1, 1, 1
  }, /* 25xx040      512   16     1+bit */
  {
    3, 1, 1, 0
  }, /* 25xx080     1024   16     1 */
  {
    3, 2, 2, 0
  }, /* 25xx080B    1024   32     2 */
  {
    4, 1, 2, 0
  }, /* 25xx160     2048   16     2 */
  {
    4, 2, 2, 0
  }, /* 25xx160B/D  2048   32     2 */
  {
    5, 2, 2, 0
  }, /* 25xx320     4096   32     2 */
  {
    6, 2, 2, 0
  }, /* 25xx640     8192   32     2 */
  {
    7, 3, 2, 0
  }, /* 25xx128    16384   64     2 */
  {
    8, 3, 2, 0
  }, /* 25xx256    32768   64     2 */
  {
    9, 4, 2, 0
  }, /* 25xx512    65536  128     2 */
  {
    10, 5, 3, 0
  }, /* 25xx1024  131072  256     3 */

  /* Atmel devices */

  {
    0, 0, 1, 0
  }, /* AT25010B     128    8     1 */
  {
    1, 0, 1, 0
  }, /* AT25020B     256    8     1 */
  {
    2, 0, 1, 1
  }, /* AT25040B     512    8     1+bit */

  /* STM devices */

  {
    11, 5, 3, 0
  }, /* M95M02    262144  256     3 */
};

/* Driver operations */

static const struct file_operations ee25xx_fops =
{
  ee25xx_open,  /* open */
  ee25xx_close, /* close */
  ee25xx_read,  /* read */
  ee25xx_write, /* write */
  ee25xx_seek,  /* seek */
  ee25xx_ioctl, /* ioctl */
  NULL          /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ee25xx_lock
 ****************************************************************************/

static void ee25xx_lock(FAR struct spi_dev_s *dev)
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

  /* After locking the SPI bus, the we also need call the setfrequency,
   * setbits, and setmode methods to make sure that the SPI is properly
   * configured for the device.  If the SPI bus is being shared, then it may
   * have been left in an incompatible state.
   */

  SPI_SETMODE(dev, CONFIG_EE25XX_SPIMODE);
  SPI_SETBITS(dev, 8);
  SPI_HWFEATURES(dev, 0);
  SPI_SETFREQUENCY(dev, CONFIG_EE25XX_FREQUENCY);
}

/****************************************************************************
 * Name: ee25xx_unlock
 ****************************************************************************/

static inline void ee25xx_unlock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, false);
}

/****************************************************************************
 * Name: ee25xx_sendcmd
 *
 * Description: Send command and address as one transaction to take advantage
 * of possible faster DMA transfers. Sending byte per byte is FAR FAR slower.
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

      SPI_SELECT(priv->spi, SPIDEV_EEPROM(0), true);

      /* Send "Read Status Register (RDSR)" command */

      SPI_SEND(priv->spi, EE25XX_CMD_RDSR);

      /* Send a dummy byte to generate the clock needed to shift out the
       * status
       */

      status = SPI_SEND(priv->spi, EE25XX_DUMMY);

      /* Deselect the FLASH */

      SPI_SELECT(priv->spi, SPIDEV_EEPROM(0), false);

      /* Given that writing could take up to a few milliseconds,
       * the following short delay in the "busy" case will allow
       * other peripherals to access the SPI bus.
       */

      if ((status & EE25XX_SR_WIP) != 0)
        {
          ee25xx_unlock(priv->spi);
          nxsig_usleep(1000);
          ee25xx_lock(priv->spi);
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

static void ee25xx_writeenable(FAR struct spi_dev_s *spi, int enable)
{
  ee25xx_lock(spi);
  SPI_SELECT(spi, SPIDEV_EEPROM(0), true);

  SPI_SEND(spi, enable ? EE25XX_CMD_WREN : EE25XX_CMD_WRDIS);

  SPI_SELECT(spi, SPIDEV_EEPROM(0), false);
  ee25xx_unlock(spi);
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
  ee25xx_lock(eedev->spi);
  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(0), true);

  ee25xx_sendcmd(eedev->spi, EE25XX_CMD_WRITE, eedev->addrlen, devaddr);
  SPI_SNDBLOCK(eedev->spi, data, len);

  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(0), false);
  ee25xx_unlock(eedev->spi);
}

/****************************************************************************
 * Name: ee25xx_semtake
 *
 * Acquire a resource to access the device.
 * The purpose of the semaphore is to block tasks that try to access the
 * EEPROM while another task is actively using it.
 *
 ****************************************************************************/

static int ee25xx_semtake(FAR struct ee25xx_dev_s *eedev)
{
  return nxsem_wait_uninterruptible(&eedev->sem);
}

/****************************************************************************
 * Name: ee25xx_semgive
 *
 * Release a resource to access the device.
 *
 ****************************************************************************/

static inline void ee25xx_semgive(FAR struct ee25xx_dev_s *eedev)
{
  nxsem_post(&eedev->sem);
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

  DEBUGASSERT(inode && inode->i_private);
  eedev = (FAR struct ee25xx_dev_s *)inode->i_private;

  ret = ee25xx_semtake(eedev);
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

  ee25xx_semgive(eedev);
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

  DEBUGASSERT(inode && inode->i_private);
  eedev = (FAR struct ee25xx_dev_s *)inode->i_private;

  ret = ee25xx_semtake(eedev);
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

  ee25xx_semgive(eedev);
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

  DEBUGASSERT(inode && inode->i_private);
  eedev = (FAR struct ee25xx_dev_s *)inode->i_private;

  ret = ee25xx_semtake(eedev);
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

      ee25xx_semgive(eedev);
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
   * But return EINVAL if
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

  ee25xx_semgive(eedev);
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

  DEBUGASSERT(inode && inode->i_private);
  eedev = (FAR struct ee25xx_dev_s *)inode->i_private;

  ret = ee25xx_semtake(eedev);
  if (ret < 0)
    {
      return ret;
    }

  /* trim len if read would go beyond end of device */

  if ((filep->f_pos + len) > eedev->size)
    {
      len = eedev->size - filep->f_pos;
    }

  ee25xx_lock(eedev->spi);
  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(0), true);

  /* STM32F4Disco: There is a 25 us delay here */

  ee25xx_sendcmd(eedev->spi, EE25XX_CMD_READ, eedev->addrlen, filep->f_pos);

  /* STM32F4Disco: There is a 42 us delay here */

  SPI_RECVBLOCK(eedev->spi, buffer, len);

  /* STM32F4Disco: There is a 20 us delay here */

  SPI_SELECT(eedev->spi, SPIDEV_EEPROM(0), false);
  ee25xx_unlock(eedev->spi);

  /* Update the file position */

  filep->f_pos += len;
  ee25xx_semgive(eedev);
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

  DEBUGASSERT(inode && inode->i_private);
  eedev = (FAR struct ee25xx_dev_s *)inode->i_private;

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

  ret = ee25xx_semtake(eedev);
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
      ee25xx_writeenable(eedev->spi, true);
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

      ee25xx_writeenable(eedev->spi, true);
      ee25xx_writepage(eedev, filep->f_pos, buffer, cnt);
      ee25xx_waitwritecomplete(eedev);
      len          -= cnt;
      buffer       += cnt;
      filep->f_pos += cnt;
    }

  ee25xx_semgive(eedev);

  return ret;
}

/****************************************************************************
 * Name: ee25xx_ioctl
 *
 * Description: TODO: Erase a sector/page/device or read device ID.
 * This is completely optional and only applies to bigger devices.
 *
 ****************************************************************************/

static int ee25xx_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct ee25xx_dev_s *eedev;
  FAR struct inode        *inode = filep->f_inode;
  int                     ret    = 0;

  DEBUGASSERT(inode && inode->i_private);
  eedev = (FAR struct ee25xx_dev_s *)inode->i_private;
  UNUSED(eedev);

  switch (cmd)
    {
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
 * Description: Bind a EEPROM driver to an SPI bus. The user MUST provide
 * a description of the device geometry, since it is not possible to read
 * this information from the device (contrary to the SPI flash devices).
 *
 ****************************************************************************/

int ee25xx_initialize(FAR struct spi_dev_s *dev, FAR char *devname,
                      int devtype, int readonly)
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

  nxsem_init(&eedev->sem, 0, 1);

  eedev->spi      = dev;
  eedev->size     = 128 << g_ee25xx_devices[devtype].bytes;
  eedev->pgsize   =   8 << g_ee25xx_devices[devtype].pagesize;
  eedev->addrlen  =        g_ee25xx_devices[devtype].addrlen << 3;
  if ((g_ee25xx_devices[devtype].flags & 1))
    {
      eedev->addrlen = 9;
    }

  eedev->readonly = !!readonly;

  finfo("EEPROM device %s, %d bytes, %d per page, addrlen %d, readonly %d\n",
       devname, eedev->size, eedev->pgsize, eedev->addrlen, eedev->readonly);

  return register_driver(devname, &ee25xx_fops, 0666, eedev);
}
