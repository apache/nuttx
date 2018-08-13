/****************************************************************************
 * drivers/sensors/adxl372.c
 * Character driver for the ST ADXL372 Tri-axis accelerometer and gyroscope.
 *
 *   Copyright (C) 2017-2018 RAF Research LLC. All rights reserved.
 *   Author: Bob Feretich <bob.feretich@rafresearch.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright+
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
 *****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_ADXL372) \
    && defined(CONFIG_SPI_EXCHANGE)

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <semaphore.h>

#include <nuttx/kmalloc.h>

#include <nuttx/fs/fs.h>
#include <nuttx/sensors/adxl372.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADXL372_INITIAL_CR_SIZE 7

/****************************************************************************
 * Private structure definitions
 ****************************************************************************/

struct sensor_data_s
{
  int16_t x_gyr;                       /* Measurement result for x axis */
  int16_t y_gyr;                       /* Measurement result for y axis */
  int16_t z_gyr;                       /* Measurement result for z axis */
};

struct adxl372_dev_s
{
  FAR struct adxl372_dev_s *flink;     /* Supports a singly linked list of
                                        * drivers */
  FAR struct spi_dev_s *spi;           /* Pointer to the SPI instance */
  FAR struct adxl372_config_s *config; /* Pointer to the configuration of the
                                        * ADXL372 sensor */
  sem_t devicesem;                     /* Manages exclusive access to this
                                        * device */
  sem_t datasem;                       /* Manages exclusive access to this
                                        * structure */
  struct sensor_data_s data;           /* The data as measured by the sensor */
  uint8_t seek_address;                /* Current device address. */
  uint8_t readonly;                    /* 0 = writing to the device in enabled */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t  adxl372_read_register(FAR struct adxl372_dev_s *dev,
                                     uint8_t reg_addr);
static void     adxl372_read_registerblk(FAR struct adxl372_dev_s *dev,
                                    uint8_t reg_addr,
                                    FAR uint8_t *reg_data,
                                    uint8_t xfercnt);
static void     adxl372_write_register(FAR struct adxl372_dev_s *dev,
                                    uint8_t reg_addr,
                                    uint8_t reg_data);
static void     adxl372_write_registerblk(FAR struct adxl372_dev_s *dev,
                                    uint8_t reg_addr,
                                    FAR uint8_t *reg_data,
                                    uint8_t xfercnt);
static void     adxl372_reset(FAR struct adxl372_dev_s *dev);

static int      adxl372_open(FAR struct file *filep);
static int      adxl372_close(FAR struct file *filep);
static ssize_t  adxl372_read(FAR struct file *, FAR char *, size_t);
static ssize_t  adxl372_write(FAR struct file *filep,
                              FAR const char *buffer, size_t buflen);
static off_t    adxl372_seek(FAR struct file *filep, off_t offset,
                             int whence);
static int      adxl372_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg);

static int      adxl372_dvr_open(FAR void *instance_handle, int32_t arg);
static int      adxl372_dvr_close(FAR void *instance_handle, int32_t arg);
static ssize_t  adxl372_dvr_read(FAR void *instance_handle,
                                 FAR char *buffer, size_t buflen);
static ssize_t  adxl372_dvr_write(FAR void *instance_handle,
                                  FAR const char *buffer, size_t buflen);
static off_t    adxl372_dvr_seek(FAR void *instance_handle, off_t offset,
                                 int whence);
static int      adxl372_dvr_ioctl(FAR void *instance_handle, int cmd,
                                  unsigned long arg);
static void     adxl372_dvr_exchange(FAR void *instance_handle,
                                     FAR const void *txbuffer,
                                     FAR void *rxbuffer, size_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_adxl372_fops =
{
  adxl372_open,
  adxl372_close,
  adxl372_read,
  adxl372_write,
  adxl372_seek,
  adxl372_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};

static const struct adxl372_dvr_entry_vector_s g_adxl372_dops =
{
  /* Standard sensor cluster driver entry-vector */

  {
    .driver_open    = adxl372_dvr_open,
    .driver_close   = adxl372_dvr_close,
    .driver_read    = adxl372_dvr_read,
    .driver_write   = adxl372_dvr_write,
    .driver_seek    = adxl372_dvr_seek,
    .driver_ioctl   = adxl372_dvr_ioctl,
    .driver_suspend = 0,
    .driver_resume  = 0,
    },

  /* adxl372 extensions follow */

  .driver_spiexc = adxl372_dvr_exchange,
};

/****************************************************************************
 * Private data storage
 ****************************************************************************/

/* Single linked list to store instances of drivers */

static struct adxl372_dev_s *g_adxl372_list = NULL;

/* Default accelerometer initialization sequence */

/* Configure ADXL372 to read live data (not using FIFO).
 * 1. Set to standby mode. The below can't be set while running.
 * 2. Configure the FIFO to be bypassed.
 * 3. Configure interrupts as disabled, because ADXL372 irpts are used.
 * 4. Configure the Output Data Rate (ODR) as 1600 Hz.
 * 5. Configure normal mode (vs low noise) and 800Hz bandwidth.
 * 6. Set to operational mode; 370ms filter settle; LPF=enb; HPF=dis;
 */

static struct adxl372_reg_pair_s g_initial_adxl372_cr_values[] =
{
  /* Set to standby mode */

  {
    .addr  = ADXL372_POWER_CTL,
    .value = 0
  },
  {
    .addr  = ADXL372_FIFO_CTL,
    .value = ADXL372_FIFO_BYPASSED
  },

  /* Interrupts disabled. */

  {
    .addr  = ADXL372_INT1_MAP,
    .value = 0
  },
  {
    .addr  = ADXL372_TIMING,
    .value = ADXL372_TIMING_ODR1600
  },
  {
    .addr  = ADXL372_MEASURE,
    .value = ADXL372_MEAS_BW800
  },
  {
    .addr  = ADXL372_POWER_CTL,
    .value = ADXL372_POWER_HPF_DISABLE | ADXL372_POWER_MODE_MEASURE
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adxl372_read_register
 ****************************************************************************/

static uint8_t adxl372_read_register(FAR struct adxl372_dev_s *dev,
                                     uint8_t reg_addr)
{
  uint8_t reg_data;

  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, ADXL372_SPI_FREQUENCY);
  SPI_SETMODE(dev->spi, ADXL372_SPI_MODE);

  /* Set CS to low to select the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read. */

  SPI_SEND(dev->spi, reg_addr | ADXL372_READ);

  /* Write an idle byte while receiving the requested data */

  reg_data = (uint8_t) (SPI_SEND(dev->spi, 0xff));

  /* Set CS to high to deselect the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);

  return reg_data;
}

/******************************************************************************
 * Name: adxl372_read_registerblk
 ******************************************************************************/

 static void adxl372_read_registerblk(FAR struct adxl372_dev_s *dev,
                                      uint8_t reg_addr,
                                      FAR uint8_t *reg_data,
                                      uint8_t xfercnt)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, ADXL372_SPI_FREQUENCY);
  SPI_SETMODE(dev->spi, ADXL372_SPI_MODE);

  /* Set CS to low to select the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading */

  SPI_SEND(dev->spi, reg_addr | ADXL372_READ);

  /* Write idle bytes while receiving the requested data */

  while ( 0 != xfercnt-- )
    {
      *reg_data++ = (uint8_t)SPI_SEND(dev->spi, 0xff);
    }

  /* Set CS to high to deselect the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: adxl372_write_register
 ****************************************************************************/

static void adxl372_write_register(FAR struct adxl372_dev_s *dev,
                                   uint8_t reg_addr, uint8_t reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, ADXL372_SPI_FREQUENCY);
  SPI_SETMODE(dev->spi, ADXL372_SPI_MODE);

  /* Set CS to low to select the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address to where we want to write */

  SPI_SEND(dev->spi, reg_addr | ADXL372_WRITE);

  /* Transmit the content which should be written into the register */

  SPI_SEND(dev->spi, reg_data);

  /* Set CS to high to deselect the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: adxl372_write_registerblk
 ****************************************************************************/

 static void adxl372_write_registerblk(FAR struct adxl372_dev_s *dev,
                                       uint8_t reg_addr,
                                       FAR uint8_t *reg_data,
                                       uint8_t xfercnt)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, ADXL372_SPI_FREQUENCY);
  SPI_SETMODE(dev->spi, ADXL372_SPI_MODE);

  /* Set CS to low which selects the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address to where we want to start writing */

  SPI_SEND(dev->spi, reg_addr | ADXL372_WRITE);

  /* Transmit the content which should be written in the register block */

  while ( 0 != xfercnt-- )
    {
      SPI_SEND(dev->spi, *reg_data++);
    }

  /* Set CS to high to deselect the ADXL372 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: adxl372_reset
 *
 * Description:
 *   ADXL Accelerometer Reset
 *   1. Make sure that a reset is not in progress.
 *   2. Write ADXL372_RESET_VALUE (0x52) to ADXL372_RESET register.
 *   3. Wait for the reset to finish.
 *
 ****************************************************************************/

static void adxl372_reset(FAR struct adxl372_dev_s *dev)
{
  uint wdcnt = 10;

  /* Wait for boot to finish (15 ms error timeout) */

  up_mdelay(5);
  while (wdcnt > 0 && (0 != adxl372_read_register(dev, ADXL372_RESET)))
    {
      up_mdelay(1);
    }

  /* Reset ADXL372 Accelerometer. Write only. Begin a boot. */

  adxl372_write_register(dev, ADXL372_RESET, ADXL372_RESET_VALUE);

  /* Wait for boot to finish (15 ms error timeout) */

  up_mdelay(5);
  wdcnt = 10;
  while (wdcnt>0 && (0 != adxl372_read_register(dev, ADXL372_RESET)))
    {
      up_mdelay(1);
    }
}

/****************************************************************************
 * Name: adxl372_read_id
 *
 * Description:
 *
 *   Read the ADXL372 Accelerometer's ID Registers.
 *   There are 4 ID Registers...
 *
 *     Manufacturer should be ADXL372_DEVID_AD_VALUE (0xAD).
 *     Family should be ADXL372_DEVID_MST_VALUE (0x1D).
 *     Part ID should be ADXL372_PARTID_VALUE (0xFA, Octal 372)
 *     Revision is returned, but not expected to be checked.
 *     All of the above are returned as an uint32_t. Should be 0xAD1DFAxx.
 *
 ****************************************************************************/

static uint32_t adxl372_read_id(FAR struct adxl372_dev_s *dev)
{
  union
  {
    uint32_t adxl_devid32;
    uint8_t  adxl_devid[4];
  } un;

  un.adxl_devid[3] = adxl372_read_register(dev, ADXL372_DEVID_AD);
  un.adxl_devid[2] = adxl372_read_register(dev, ADXL372_DEVID_MST);
  un.adxl_devid[1] = adxl372_read_register(dev, ADXL372_PARTID);
  un.adxl_devid[0] = adxl372_read_register(dev, ADXL372_REVID);
  return un.adxl_devid32;
}

/****************************************************************************
 * Name: adxl372_dvr_open
 ****************************************************************************/

static int adxl372_dvr_open(FAR void *instance_handle, int32_t arg)
{
  FAR struct adxl372_dev_s *priv = (FAR struct adxl372_dev_s *)instance_handle;
  FAR struct adxl372_reg_pair_s *initp;
  uint32_t pnpid;
  int sz;
  int ret;
  int i;

#ifdef CONFIG_DEBUG_SENSORS_INFO
  uint8_t reg_content;
#endif

  sninfo("adxl372_open: entered...\n");

  DEBUGASSERT(priv != NULL);
  UNUSED(arg);

  ret = nxsem_trywait(&priv->devicesem);
  if (ret < 0)
    {
      sninfo("INFO: ADXL372 Accelerometer is already opened.\n");
      return -EBUSY;
    }

  /* Read the ID registers */

  pnpid = adxl372_read_id(priv);
  priv->readonly = false;

  sninfo("ADXL372_ID = 0x%08x\n", pnpid);

  if ((pnpid & 0xffffff00) != (ADXL372_DEVID_AD_VALUE << 24 |
                               ADXL372_DEVID_MST_VALUE << 16 |
                               ADXL372_PARTID_VALUE << 8))
    {
      snwarn("ERROR: Invalid ADXL372_ID = 0x%08x\n", pnpid);

      priv->readonly = true;
      set_errno(ENODEV);
    }
  else /* ID matches */
    {
      adxl372_reset(priv);   /* Perform a sensor reset */

      /* Choose the initialization sequence */

      if (priv->config->initial_cr_values_size == 0 ||
          priv->config->initial_cr_values == NULL)
        {
          initp = g_initial_adxl372_cr_values;      /* Default values */
          sz = ADXL372_INITIAL_CR_SIZE;
        }
      else
        {
          initp = priv->config->initial_cr_values;  /* User supplied values */
          sz = priv->config->initial_cr_values_size;
        }

      /* Apply the initialization sequence */

      for (i = 0; i < sz; i++)
        {
          adxl372_write_register(priv, initp[i].addr, initp[i].value);
        }

#ifdef CONFIG_DEBUG_SENSORS_INFO
      /* Read back the content of all control registers for debug purposes */

      reg_content = adxl372_read_register(priv, ADXL372_FIFO_CTL);
      sninfo("ADXL372_FIFO_CTL = 0x%02x\n", reg_content);

      reg_content = adxl372_read_register(priv, ADXL372_INT1_MAP);
      sninfo("ADXL372_INT1_MAP = 0x%02x\n", reg_content);

      reg_content = adxl372_read_register(priv, ADXL372_TIMING);
      sninfo("ADXL372_TIMING = 0x%02x\n", reg_content);

      reg_content = adxl372_read_register(priv, ADXL372_MEASURE);
      sninfo("ADXL372_MEASURE = 0x%02x\n", reg_content);

      reg_content = adxl372_read_register(priv, ADXL372_POWER_CTL);
      sninfo("ADXL372_POWER_CTL = 0x%02x\n", reg_content);
#endif
    }

  priv->seek_address = (uint8_t) ADXL372_XDATA_H;
  return OK;
}

/****************************************************************************
 * Name: adxl372_dvr_close
 ****************************************************************************/

static int adxl372_dvr_close(FAR void *instance_handle, int32_t arg)
{
  FAR struct adxl372_dev_s *priv = (FAR struct adxl372_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);
  UNUSED(arg);

  /* Perform a reset to place the sensor in standby mode.*/

  adxl372_reset(priv);

  /* Release the sensor */

  nxsem_post(&priv->devicesem);
  return OK;
}

/****************************************************************************
 * Name: adxl372_dvr_read
 ****************************************************************************/

static ssize_t adxl372_dvr_read(FAR void *instance_handle, FAR char *buffer,
                                size_t buflen)
{
  FAR struct adxl372_dev_s *priv = ((FAR struct adxl372_dev_s *)instance_handle);
  union
  {
    int16_t d16;
    char    d8[2];
  } un;
  FAR char *p1;
  FAR char *p2;
  int i;

  DEBUGASSERT(priv != NULL);

  adxl372_read_registerblk(priv, priv->seek_address, (uint8_t *)buffer,
                          buflen);

  /* Permute accelerometer data out fields */

  if (priv->seek_address == ADXL372_XDATA_H && buflen >= 6)
    {
      p1 = p2 = buffer;
      for (i=0; i<3; i++)
        {
          un.d8[1] = *p1++;
          un.d8[0] = *p1++;
          un.d16   = un.d16 >> 4;
          *p2++    = un.d8[0];
          *p2++    = un.d8[1];
        }
    }

  return buflen;
}

/****************************************************************************
 * Name: adxl372_dvr_write
 ****************************************************************************/

static ssize_t adxl372_dvr_write(FAR void *instance_handle,
                                 FAR const char *buffer, size_t buflen)
{
  FAR struct adxl372_dev_s *priv = (FAR struct adxl372_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);

  if (priv->readonly)
    {
      set_errno(EROFS);
      return -1;
    }

  adxl372_write_registerblk(priv, priv->seek_address, (uint8_t *)buffer,
                            buflen);

  return buflen;
}

/****************************************************************************
 * Name: adxl372_dvr_seek
 ****************************************************************************/

static off_t adxl372_dvr_seek(FAR void *instance_handle, off_t offset,
                              int whence)
{
  off_t reg;
  FAR struct adxl372_dev_s *priv = (FAR struct adxl372_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);

  switch (whence)
    {
      case SEEK_CUR:  /* Incremental seek */
        reg = priv->seek_address + offset;
        if (0 > reg || reg > ADXL372_LAST)
          {
            set_errno(-EINVAL);
            return -1;
          }

        priv->seek_address = reg;
        break;

      case SEEK_END:  /* Seek to the 1st X-data register */
        priv->seek_address = ADXL372_XDATA_H;
        break;

      case SEEK_SET:  /* Seek to designated address */
        if (0 > offset || offset > ADXL372_LAST)
          {
            set_errno(-EINVAL);
            return -1;
          }

        priv->seek_address = offset;
        break;

      default:        /* invalid whence */
        set_errno(-EINVAL);
        return -1;
    }

  return priv->seek_address;
}

/****************************************************************************
 * Name: adxl372_dvr_ioctl
 ****************************************************************************/

static int adxl372_dvr_ioctl(FAR void *instance_handle, int cmd,
                             unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      /* Command was not recognized */

    default:
      snerr("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: adxl372_dvr_exchange (with SPI DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   instance_handle - Pointer to struct adxl372_dev_s.
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void adxl372_dvr_exchange(FAR void *instance_handle,
                                 FAR const void *txbuffer,
                                 FAR void *rxbuffer, size_t nwords)
{
  FAR struct adxl372_dev_s *priv = (FAR struct adxl372_dev_s *)instance_handle;
  FAR struct spi_dev_s *spi = priv->spi;

  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(spi, true);

  SPI_SETFREQUENCY(spi, ADXL372_SPI_FREQUENCY);
  SPI_SETMODE(spi, ADXL372_SPI_MODE);

  /* Set CS to low which selects the ADXL372 */

  SPI_SELECT(spi, priv->config->spi_devid, true);

  /* Perform an SPI exchange block operation. */

  SPI_EXCHANGE(spi, txbuffer, rxbuffer, nwords);

  /* Set CS to high to deselect the ADXL372 */

  SPI_SELECT(spi, priv->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(spi, false);
 }

/****************************************************************************
 * Name: adxl372_open
 ****************************************************************************/

 static int adxl372_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adxl372_dev_s *priv = inode->i_private;
  int ret;

  ret = adxl372_dvr_open((FAR void *)priv, 0);
  return ret;
}

/****************************************************************************
 * Name: adxl372_close
 ****************************************************************************/

static int adxl372_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adxl372_dev_s *priv = inode->i_private;
  int ret;

  ret = adxl372_dvr_close((FAR void *)priv, 0);
  return ret;
}

/****************************************************************************
 * Name: adxl372_read
 ****************************************************************************/

static ssize_t adxl372_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adxl372_dev_s *priv = inode->i_private;

  return adxl372_dvr_read(priv, buffer, buflen);
}

/****************************************************************************
 * Name: adxl372_write
 ****************************************************************************/

static ssize_t adxl372_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adxl372_dev_s *priv = inode->i_private;

  return adxl372_dvr_write(priv, buffer, buflen);
}

/****************************************************************************
 * Name: adxl372_seek
 ****************************************************************************/

static off_t adxl372_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adxl372_dev_s *priv = inode->i_private;

  return adxl372_dvr_seek(priv, offset, whence);
}

/****************************************************************************
 * Name: adxl372_ioctl
 ****************************************************************************/

static int adxl372_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adxl372_dev_s *priv = inode->i_private;

  return adxl372_dvr_ioctl(priv, cmd, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adxl372_register
 *
 * Description:
 *   Register the ADXL372 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath  - The full path to the driver to register. E.g., "/dev/acl0"
 *   spi      - An instance of the SPI interface to use to communicate with
 *              ADXL372
 *   config   - Configuration for the ADXL372 accelerometer driver.  For
 *              details see description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int adxl372_register(FAR const char *devpath,
                     FAR struct spi_dev_s *spi,
                     FAR struct adxl372_config_s *config)
{
  FAR struct adxl372_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(config != NULL);

  /* Initialize the ADXL372 accelerometer device structure. */

  priv = (FAR struct adxl372_dev_s *)kmm_malloc(sizeof(struct adxl372_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate accelerometer instance\n");
      return -ENOMEM;
    }

  priv->spi           = spi;
  priv->config        = config;
  config->leaf_handle = NULL;
  config->sc_ops      = NULL;

  /* Initialize sensor and sensor data access semaphore */

  nxsem_init(&priv->devicesem, 0, 1);
  nxsem_init(&priv->datasem, 0, 1);

  /* Register the character driver */

  ret = register_driver(devpath, &g_adxl372_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register accelerometer driver: %d\n", ret);

      nxsem_destroy(&priv->datasem);
      kmm_free(priv);
      return ret;
    }

  /* Since we support multiple ADXL372 devices, we will need to add this new
   * instance to a list of device instances so that it can be found by the
   * interrupt handler based on the received IRQ number.
   */

  priv->flink         = g_adxl372_list;
  g_adxl372_list      = priv;
  config->leaf_handle = (void *) priv;
  config->sc_ops      = &g_adxl372_dops;

  return OK;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_ADXL372 && CONFIG_SPI_EXCHANGE */
