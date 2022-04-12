/****************************************************************************
 * drivers/sensors/adxl345_base.c
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

#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
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

static ssize_t adxl345_read(FAR struct file *filep, FAR char *buffer,
                            size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_adxl345fops =
{
  NULL,            /* open */
  NULL,            /* close */
  adxl345_read,    /* read */
  NULL,            /* write */
  NULL,            /* seek */
  NULL,            /* ioctl */
  NULL             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

/****************************************************************************
 * Name: adxl345_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t adxl345_read(FAR struct file *filep,
                            FAR char *buffer, size_t len)
{
  FAR struct inode         *inode;
  FAR struct adxl345_dev_s *priv;
  struct adxl345_sample_s   sample;
  int                       ret;

  sninfo("len=%d\n", len);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct adxl345_dev_s *)inode->i_private;

  /* Verify that the caller has provided a buffer large enough to receive
   * the accelerometer data.
   */

  if (len < sizeof(struct adxl345_sample_s))
    {
      /* We could provide logic to break up a sample into segments and
       * handle smaller reads... but why?
       */

      return -ENOSYS;
    }

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Read accelerometer X Y Z axes */

  sample.data_x =  adxl345_getreg8(priv, ADXL345_DATAX1);
  sample.data_x = (sample.data_x << 8) |
                   adxl345_getreg8(priv, ADXL345_DATAX0);
  sample.data_y =  adxl345_getreg8(priv, ADXL345_DATAY1);
  sample.data_y = (sample.data_y << 8) |
                   adxl345_getreg8(priv, ADXL345_DATAY0);
  sample.data_z =  adxl345_getreg8(priv, ADXL345_DATAZ1);
  sample.data_z = (sample.data_z << 8) |
                   adxl345_getreg8(priv, ADXL345_DATAZ0);

  add_sensor_randomness(sample.data_x);
  add_sensor_randomness((sample.data_z << 16) | sample.data_y);

  /* Return read sample */

  buffer = (FAR char *) &sample;

  nxsem_post(&priv->exclsem);
  return sizeof(struct adxl345_sample_s);
}

/****************************************************************************
 * Name: adxl345_register
 *
 * Description:
 *  This function will register the accelerometer driver as /dev/accelN where
 *  N is the minor device number
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

  sninfo("handle=%p minor=%d\n", handle, minor);
  DEBUGASSERT(priv);

  /* Get exclusive access to the device structure */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      snerr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Initialize the structure fields to their default values */

  priv->ofsx      = 0;
  priv->ofsy      = 0;
  priv->ofsz      = 0;

  /* Register the character driver */

  snprintf(devname, sizeof(devname), DEV_FORMAT, minor);
  ret = register_driver(devname, &g_adxl345fops, 0444, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver %s: %d\n", devname, ret);
      nxsem_post(&priv->exclsem);
      return ret;
    }

  /* Indicate that the accelerometer was successfully initialized */

  priv->status |= ADXL345_STAT_INITIALIZED;  /* Accelerometer is initialized */
  nxsem_post(&priv->exclsem);
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

      priv->sample.data_x = adxl345_getreg8(priv, ADXL345_DATAX1);
      priv->sample.data_x = (priv->sample.data_x << 8) |
                            adxl345_getreg8(priv, ADXL345_DATAX0);
      priv->sample.data_y = adxl345_getreg8(priv, ADXL345_DATAY1);
      priv->sample.data_y = (priv->sample.data_y << 8) |
                            adxl345_getreg8(priv, ADXL345_DATAY0);
      priv->sample.data_z = adxl345_getreg8(priv, ADXL345_DATAZ1);
      priv->sample.data_z = (priv->sample.data_z << 8) |
                            adxl345_getreg8(priv, ADXL345_DATAZ0);
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

static void adxl345_interrupt(FAR struct adxl345_config_s *config,
                              FAR void *arg)
{
  FAR struct adxl345_dev_s *priv = (FAR struct adxl345_dev_s *)arg;
  int ret;

  DEBUGASSERT(priv && priv->config == config);

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
          snerr("ERROR: Failed to queue work: %d\n", ret);
        }
    }

  /* Clear any pending interrupts and return success */

  config->clear(config);
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
  sninfo("devid: %04x\n", devid);

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

  nxsig_usleep(20 * 1000);
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
ADXL345_HANDLE adxl345_instantiate(FAR struct i2c_master_s *dev,
                                   FAR struct adxl345_config_s *config)
#endif
{
  FAR struct adxl345_dev_s *priv;
  uint8_t regval;
  int ret;

  /* Allocate the ADXL345 driver instance */

  priv = (FAR struct adxl345_dev_s *)
              kmm_zalloc(sizeof(struct adxl345_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate the device structure!\n");
      return NULL;
    }

  /* Initialize the device state structure */

  nxsem_init(&priv->exclsem, 0, 1);
  priv->config = config;

#ifdef CONFIG_ADXL345_SPI
  priv->spi = dev;

#else
  priv->i2c = dev;
#endif

  /* Read and verify the ADXL345 device ID */

  ret = adxl345_checkid(priv);
  if (ret < 0)
    {
      snerr("ERROR: Wrong Device ID!\n");
      kmm_free(priv);
      return NULL;
    }

  /* Generate ADXL345 Software reset */

  adxl345_reset(priv);

  /* Configure the interrupt output pin to generate interrupts on high or low
   * level.
   */

  regval  = adxl345_getreg8(priv, ADXL345_DATA_FORMAT);
#ifdef CONFIG_ADXL345_ACTIVELOW
  regval |= DATA_FMT_INT_INVERT; /* Pin polarity: Active low / falling edge */
#else
  regval &= ~DATA_FMT_INT_INVERT;  /* Pin polarity: Active high / rising edge */
#endif
  adxl345_putreg8(priv, ADXL345_DATA_FORMAT, regval);

  /* Attach the ADXL345 interrupt handler. */

  config->attach(config,
                (adxl345_handler_t)adxl345_interrupt,
                (FAR void *)priv);

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
