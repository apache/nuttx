/****************************************************************************
 * drivers/spi/ice40.c
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>

#include <nuttx/spi/ice40.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int ice40_open(FAR struct file *filep);

static int ice40_close(FAR struct file *filep);

static ssize_t ice40_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t ice40_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int ice40_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/* Helper functions */

static int ice40_init_fpga(FAR struct ice40_dev_s *dev);

static int ice40_writeblk(FAR struct ice40_dev_s *dev,
                           FAR const char *buffer,
                           size_t buflen);

static int ice40_endwrite(FAR struct ice40_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ice40_fops =
{
  ice40_open,  /* open */
  ice40_close, /* close */
  ice40_read,  /* read */
  ice40_write, /* write */
  NULL,        /* seek */
  ice40_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ice40_open
 *
 * Description:
 *  This function is called whenever the ICE40 device is opened.
 *
 ****************************************************************************/

static int
ice40_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ice40_dev_s *dev = inode->i_private;

  DEBUGASSERT(dev != NULL);

  if (dev->is_open)
    {
      return -EBUSY;
    }

  dev->is_open = true;

  return OK;
}

/****************************************************************************
 * Name: ice40_close
 *
 * Description:
 *  This function is called whenever the ICE40 device is closed.
 *
 ****************************************************************************/

static int
ice40_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ice40_dev_s *dev = inode->i_private;

  DEBUGASSERT(dev != NULL);

  if (dev->in_progress)
    {
      if (ice40_endwrite(dev))
        {
          _err("ERROR: Failed to end writing to FPGA\n");
          dev->is_open = false;
          return -EIO;
        }
    }

  dev->is_open = false;

  return OK;
}

/****************************************************************************
 * Name: ice40_configspi
 *
 * Description:
 *   Configure the SPI instance for to match the DAT-31R5-SP+
 *   specifications
 *
 ****************************************************************************/

static inline void
ice40_configspi(FAR struct spi_dev_s *spi)
{
  DEBUGASSERT(spi != NULL);

  /* Configure SPI Mode for the ICE40 */

  SPI_SETMODE(spi, ICE40_SPI_MODE);
  SPI_SETBITS(spi, 8);

  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_ICE40_SPI_FREQUENCY);
}

/****************************************************************************
 * Name: ice40_init_fpga
 *
 * Description:
 *  Initialize the FPGA - set it to SPI Master load mode
 *  Reset the FPGA with the CS pin active low
 *  and send 8 dummy bits with CS high to start the SPI transfer.
 *
 ****************************************************************************/

static int
ice40_init_fpga(FAR struct ice40_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(dev->spi != NULL);

  SPI_LOCK(dev->spi, true);

  ice40_configspi(dev->spi);

  dev->ops->reset(dev, true);
  up_udelay(2);
  dev->ops->select(dev, true);
  up_udelay(2);
  dev->ops->reset(dev, false);
  up_udelay(1200);

  dev->ops->select(dev, false);
  SPI_SEND(dev->spi, 0xff);
  dev->ops->select(dev, true);

  dev->in_progress = true;

  return 0;
}

/****************************************************************************
 * Name: ice_v_writeblk
 *
 * Description:
 *  Write block to the ICE40 FPGA, max 4096 bytes
 ****************************************************************************/

static inline int
ice40_writeblk(FAR struct ice40_dev_s *dev, FAR const char *buffer,
                size_t buflen)
{
  uint32_t nbytes;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(dev->spi != NULL);

  DEBUGASSERT(buffer != NULL);
  DEBUGASSERT(buflen > 0);

  if (!dev->in_progress)
    {
      _err("ERROR: FPGA not initialized\n");
      return -EINVAL;
    }

  ice40_configspi(dev->spi);

  while (buflen > 0)
    {
      nbytes = buflen;
      if (nbytes >= ICE_SPI_MAX_XFER)
        {
          nbytes = ICE_SPI_MAX_XFER;
        }

      SPI_SNDBLOCK(dev->spi, buffer, nbytes);

      buffer += nbytes;
      buflen -= nbytes;
    }

  return 0;
}

/****************************************************************************
 * Name: ice_v_endwrite
 *
 * Description:
 *  End writing bitstream to the ICE40 FPGA
 ****************************************************************************/

static int
ice40_endwrite(FAR struct ice40_dev_s *dev)
{
  ice40_configspi(dev->spi);
  int cdone = 0;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(dev->spi != NULL);

  if (!dev->in_progress)
    {
      _err("ERROR: FPGA not initialized\n");
      return -EINVAL;
    }

  dev->ops->select(dev, false);

  for (size_t i = 0; i < ICE40_SPI_FINAL_CLK_CYCLES + 7 / 8; i++)
    {
      SPI_SEND(dev->spi, 0xff);
    }

  cdone = dev->ops->get_status(dev);
  if (cdone == 0)
    {
      _err("ERROR: CDONE not high after writing to FPGA\n");
      SPI_LOCK(dev->spi, false);
      return -ENODEV;
    }

  SPI_LOCK(dev->spi, false);

  dev->in_progress = false;

  return 0;
}

/****************************************************************************
 * Name: ice40_write
 *
 * Description:
 *  Write buffer to the ICE40 FPGA
 ****************************************************************************/

static ssize_t
ice40_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  int ret;

  DEBUGASSERT(buffer != NULL);
  DEBUGASSERT(filep != NULL);

  FAR struct inode *inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);
  FAR struct ice40_dev_s *dev = inode->i_private;
  DEBUGASSERT(dev != NULL);

  DEBUGASSERT(dev->spi != NULL);

  if (!dev->in_progress)
    {
      ret = ice40_init_fpga(dev);
      if (ret < 0)
        {
          _err("ERROR: Failed to initialize FPGA: %d\n", ret);
          return ret;
        }
    }

  ret = ice40_writeblk(dev, buffer, buflen);
  if (ret < 0)
    {
      _err("ERROR: Failed to write to FPGA: %d\n", ret);
      return ret;
    }

  return buflen;
}

/****************************************************************************
 * Name: ice40_read
 *
 * Description:
 *   Read is ignored.
 ****************************************************************************/

static ssize_t
ice40_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: ice40_ioctl
 *
 * Description:
 *   The only available ICTL is RFIOC_SETATT. It expects a struct
 *   attenuator_control* as the argument to set the attenuation
 *   level. The channel is ignored as the DAT-31R5-SP+ has just a
 *   single attenuator.
 ****************************************************************************/

static int
ice40_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ice40_dev_s *dev = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
    case FPGAIOC_WRITE_INIT:
      ret = ice40_init_fpga(dev);
      break;

    case FPGAIOC_WRITE:
      ret = ice40_writeblk(dev, (FAR const char *)arg, sizeof(arg));
      break;
    case FPGAIOC_WRITE_COMPLETE:
      ret = ice40_endwrite(dev);
      break;

    default:
      sninfo("Unrecognized cmd: %d\n", cmd);
      ret = -EINVAL;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ice40_register
 *
 * Description:
 *   Register the ice_v character device as 'devpath'.
 *
 ****************************************************************************/

int ice40_register(FAR const char *path, FAR struct ice40_dev_s *dev)
{
  int ret;

  /* Sanity check */

  DEBUGASSERT(dev != NULL);

  /* Register the character driver */

  ret = register_driver(path, &g_ice40_fops, 0666, dev);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
    }

  return ret;
}
