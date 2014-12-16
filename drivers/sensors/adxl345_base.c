/****************************************************************************
 * drivers/sensors/adxl345.c
 *
 *   Copyright (C) 2014 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *   Based on STME811 driver
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sensors/adxl345.h>

#include "adxl345.h"

#if defined(CONFIG_SENSORS_ADXL345)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Character driver methods */

static int     adxl345_open(FAR struct file *filep);
static int     adxl345_close(FAR struct file *filep);
static ssize_t adxl345_read(FAR struct file *filep, FAR char *buffer,
                            size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct adxl345_dev_s g_adxl345;

/* This the vtable that supports the character driver interface */

static const struct file_operations g_adxl345fops =
{
  adxl345_open,    /* open */
  adxl345_close,   /* close */
  adxl345_read,    /* read */
  0,               /* write */
  0,               /* seek */
  0,               /* ioctl */
};

/****************************************************************************
 * Name: adxl345_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int adxl345_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: adxl345_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int adxl345_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: adxl345_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t adxl345_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode         *inode;
  FAR struct adxl345_dev_s *priv;
  struct adxl345_sample_s   sample;
  int                       ret;

  ivdbg("len=%d\n", len);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct adxl345_dev_s *)inode->i_private;

  /* Verify that the caller has provided a buffer large enough to receive
   * the accelerometer data.
   */

  if (len < sizeof(struct adxl345_sample_s))
    {
      /* We could provide logic to break up a touch report into segments and
       * handle smaller reads... but why?
       */

      return -ENOSYS;
    }

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Read accelerometer X Y Z axes */

  sample.data_x =  adxl345_getreg8(priv, ADXL345_DATAX1);
  sample.data_x = (sample.data_x << 8) | adxl345_getreg8(priv, ADXL345_DATAX0);
  sample.data_y =  adxl345_getreg8(priv, ADXL345_DATAY1);
  sample.data_y = (sample.data_y << 8) | adxl345_getreg8(priv, ADXL345_DATAY0);
  sample.data_z =  adxl345_getreg8(priv, ADXL345_DATAZ1);
  sample.data_z = (sample.data_z << 8) | adxl345_getreg8(priv, ADXL345_DATAZ0);

  /* Return read sample */

  buffer = (FAR char *) &sample;

  sem_post(&priv->exclsem);
  return sizeof(struct adxl345_sample_s);
}

/****************************************************************************
 * Name: adxl345_register
 *
 * Description:
 *  This function will register the touchscreen driver as /dev/accelN where N
 *  is the minor device number
 *
 * Input Parameters:
 *   handle    - The handle previously returned by adxl345_register
 *   minor     - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int adxl345_register(ADXL345_HANDLE handle, int minor)
{
  FAR struct adxl345_dev_s *priv = (FAR struct adxl345_dev_s *)handle;
  char devname[DEV_NAMELEN];
  int ret;

  ivdbg("handle=%p minor=%d\n", handle, minor);
  DEBUGASSERT(priv);

  /* Get exclusive access to the device structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      int errval = errno;
      idbg("ERROR: sem_wait failed: %d\n", errval);
      return -errval;
    }

  /* Initialize the structure fields to their default values */

  priv->ofsx      = 0;
  priv->ofsy      = 0;
  priv->ofsz      = 0;

  /* Register the character driver */

  snprintf(devname, DEV_NAMELEN, DEV_FORMAT, minor);
  ret = register_driver(devname, &g_adxl345fops, 0666, priv);
  if (ret < 0)
    {
      idbg("ERROR: Failed to register driver %s: %d\n", devname, ret);
      sem_post(&priv->exclsem);
      return ret;
    }

  /* Indicate that the accelerometer was successfully initialized */

  priv->status |= ADXL345_STAT_INITIALIZED;  /* Accelerometer is initialized */
  sem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: adxl345_worker
 *
 * Description:
 *   This is the "bottom half" of the ADXL345 interrupt handler
 *
 ****************************************************************************/

static void adxl345_worker(FAR void *arg)
{
  FAR struct adxl345_dev_s *priv = (FAR struct adxl345_dev_s *)arg;
  uint8_t regval;

  DEBUGASSERT(priv && priv->config);

  /* Get the global interrupt status */

  regval =  adxl345_getreg8(priv, ADXL345_INT_SOURCE);

  /* Check for a data ready interrupt */

  if ((regval & INT_DATA_READY) != 0)
    {
      /* Read accelerometer data to sample */

      priv->sample.data_x =  adxl345_getreg8(priv, ADXL345_DATAX1);
      priv->sample.data_x = (priv->sample.data_x << 8) | adxl345_getreg8(priv, ADXL345_DATAX0);
      priv->sample.data_y =  adxl345_getreg8(priv, ADXL345_DATAY1);
      priv->sample.data_y = (priv->sample.data_y << 8) | adxl345_getreg8(priv, ADXL345_DATAY0);
      priv->sample.data_z =  adxl345_getreg8(priv, ADXL345_DATAZ1);
      priv->sample.data_z = (priv->sample.data_z << 8) | adxl345_getreg8(priv, ADXL345_DATAZ0);
    }

  /* Re-enable the ADXL345 GPIO interrupt */

  priv->config->enable(priv->config, true);
}

/****************************************************************************
 * Name: adxl345_interrupt
 *
 * Description:
 *  The ADXL345 interrupt handler
 *
 ****************************************************************************/

static int adxl345_interrupt(int irq, FAR void *context)
{
  FAR struct adxl345_dev_s    *priv;
  FAR struct adxl345_config_s *config;
  int                          ret;

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* Disable further interrupts */

  config->enable(config, false);

  /* Check if interrupt work is already queue.  If it is already busy, then
   * we already have interrupt processing in the pipeline and we need to do
   * nothing more.
   */

  if (work_available(&priv->work))
    {
      /* Yes.. Transfer processing to the worker thread.  Since ADXL345
       * interrupts are disabled while the work is pending, no special
       * action should be required to protect the work queue.
       */

      ret = work_queue(HPWORK, &priv->work, adxl345_worker, priv, 0);
      if (ret != 0)
        {
          illdbg("Failed to queue work: %d\n", ret);
        }
    }

  /* Clear any pending interrupts and return success */

  config->clear(config);
  return OK;
}

/****************************************************************************
 * Name: adxl345_checkid
 *
 * Description:
 *   Read and verify the ADXL345 chip ID
 *
 ****************************************************************************/

static int adxl345_checkid(FAR struct adxl345_dev_s *priv)
{
  uint8_t devid = 0;

  /* Read device ID  */

  devid = adxl345_getreg8(priv, ADXL345_DEVID);
  ivdbg("devid: %04x\n", devid);

  if (devid != (uint16_t) DEVID)
    {
      /* ID is not Correct */

      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: adxl345_reset
 *
 * Description:
 *  Reset the ADXL345
 *
 ****************************************************************************/

static void adxl345_reset(FAR struct adxl345_dev_s *priv)
{
  /* ADXL345 doesn't have software reset */

  /* Wait a bit to make the GOD of TIME happy */

  usleep(20*1000);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adxl345_instantiate
 *
 * Description:
 *   Instantiate and configure the ADXL345 device driver to use the provided
 *   I2C or SPIdevice instance.
 *
 * Input Parameters:
 *   dev     - An I2C or SPI driver instance
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   A non-zero handle is returned on success.  This handle may then be used
 *   to configure the ADXL345 driver as necessary.  A NULL handle value is
 *   returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADXL345_SPI
ADXL345_HANDLE adxl345_instantiate(FAR struct spi_dev_s *dev,
                                     FAR struct adxl345_config_s *config)
#else
ADXL345_HANDLE adxl345_instantiate(FAR struct i2c_dev_s *dev,
                                     FAR struct adxl345_config_s *config)
#endif
{
  FAR struct adxl345_dev_s *priv;
  uint8_t regval;
  int ret;

  /* Use the one-and-only ADXL345 driver instance */

  priv = &g_adxl345;

  /* Initialize the device state structure */

  sem_init(&priv->exclsem, 0, 1);
  priv->config = config;

#ifdef CONFIG_ADXL345_SPI
  priv->spi = dev;

  /* If this SPI bus is not shared, then we can config it now.
   * If it is shared, then other device could change our config,
   * then just configure before sending data.
   */

  #ifdef CONFIG_SPI_OWNBUS
    /* Configure SPI for the ADXL345 */

    SPI_SELECT(priv->spi, SPIDEV_GSENSOR, true);
    SPI_SETMODE(priv->spi, SPIDEV_MODE3);
    SPI_SETBITS(priv->spi, 8);
    SPI_SETFREQUENCY(priv->spi, ADXL345_SPI_MAXFREQUENCY);
    SPI_SELECT(priv->spi, SPIDEV_GSENSOR, false);
  #endif
#else
  priv->i2c = dev;

  /* Set the I2C address and frequency.  REVISIT:  This logic would be
   * insufficient if we share the I2C bus with any other devices that also
   * modify the address and frequency.
   */

  I2C_SETADDRESS(dev, config->address, 7);
  I2C_SETFREQUENCY(dev, config->frequency);
#endif

  /* Read and verify the ADXL345 device ID */

  ret = adxl345_checkid(priv);
  if (ret < 0)
    {
      illdbg("Wrong Device ID!\n");
      return NULL;
    }

  /* Generate ADXL345 Software reset */

  adxl345_reset(priv);

  /* Configure the interrupt output pin to generate interrupts on high or low level. */

  regval  = adxl345_getreg8(priv, ADXL345_DATA_FORMAT);
#ifdef CONFIG_ADXL345_ACTIVELOW
  regval |= DATA_FMT_INT_INVERT; /* Pin polarity: Active low / falling edge */
#else
  regval &= ~DATA_FMT_INT_INVERT;  /* Pin polarity: Active high / rising edge */
#endif
  adxl345_putreg8(priv, ADXL345_DATA_FORMAT, regval);

  /* Attach the ADXL345 interrupt handler. */

  config->attach(config, adxl345_interrupt);

  /* Leave standby mode */

  adxl345_putreg8(priv, ADXL345_POWER_CTL, POWER_CTL_MEASURE);

  config->clear(config);
  config->enable(config, true);

  /* Enable interrupts */

  adxl345_putreg8(priv, ADXL345_INT_ENABLE, INT_DATA_READY);

  /* Return our private data structure as an opaque handle */
  return (ADXL345_HANDLE)priv;
}

#endif /* CONFIG_SENSORS_ADXL345 */
