/****************************************************************************
 * drivers/sensors/xen1210.c
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

/* This driver is used to interface with Sensixs XEN1210 3D-board. */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <inttypes.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/xen1210.h>
#include <nuttx/random.h>

#include "xen1210.h"

#if defined(CONFIG_SENSORS_XEN1210)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static ssize_t xen1210_read(FAR struct file *filep, FAR char *buffer,
                            size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_xen1210fops =
{
  NULL,            /* open */
  NULL,            /* close */
  xen1210_read,    /* read */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xen1210_configspi
 *
 * Description:
 *
 ****************************************************************************/

static inline void xen1210_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the XEN1210 */

  SPI_SETMODE(spi, SPIDEV_MODE1);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, XEN1210_SPI_MAXFREQUENCY);
}

/****************************************************************************
 * Name: xen1210_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t xen1210_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  FAR struct inode         *inode;
  FAR struct xen1210_dev_s *priv;
  int                       ret;

  sninfo("len=%d\n", len);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct xen1210_dev_s *)inode->i_private;

  /* Verify that the caller has provided a buffer large enough to receive
   * the magnetometer data.
   */

  if (len < sizeof(struct xen1210_sample_s))
    {
      /* We could provide logic to break up a touch report into segments and
       * handle smaller reads... but why?
       */

      snerr("Failed: Trying to read less bytes than sensor sample!\n");
      return -ENOSYS;
    }

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      snerr("Failed: Cannot get exclusive access to driver structure!\n");
      return ret;
    }

  sninfo("X = 0x%06" PRIX32 "\n", priv->sample.data_x);
  sninfo("Y = 0x%06" PRIX32 "\n", priv->sample.data_y);
  sninfo("Z = 0x%06" PRIX32 "\n", priv->sample.data_z);

  /* Return read sample */

  buffer = (FAR char *) &priv->sample;

  nxmutex_unlock(&priv->lock);
  return sizeof(struct xen1210_sample_s);
}

/****************************************************************************
 * Name: xen1210_worker
 *
 * Description:
 *   This is the "bottom half" of the XEN1210 interrupt handler
 *
 ****************************************************************************/

static void xen1210_worker(FAR void *arg)
{
  FAR struct xen1210_dev_s *priv = (FAR struct xen1210_dev_s *)arg;

  DEBUGASSERT(priv && priv->config);

  /* Read the sensors */

  xen1210_getdata(priv);

  /* Re-enable the XEN1210 GPIO interrupt */

  priv->config->enable(priv->config, true);
}

/****************************************************************************
 * Name: xen1210_interrupt
 *
 * Description:
 *  The XEN1210 interrupt handler
 *
 ****************************************************************************/

static void xen1210_interrupt(FAR struct xen1210_config_s *config,
                              FAR void *arg)
{
  FAR struct xen1210_dev_s *priv = (FAR struct xen1210_dev_s *)arg;
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
      /* Yes.. Transfer processing to the worker thread.  Since XEN1210
       * interrupts are disabled while the work is pending, no special
       * action should be required to protect the work queue.
       */

      ret = work_queue(HPWORK, &priv->work, xen1210_worker, priv, 0);
      if (ret != 0)
        {
          snerr("ERROR: Failed to queue work: %d\n", ret);
        }
    }

  /* Clear any pending interrupts and return success */

  config->clear(config);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xen1210_instantiate
 *
 * Description:
 *   Instantiate and configure the XEN1210 device driver to use the provided
 *   SPI device instance.
 *
 * Input Parameters:
 *   dev     - An I2C or SPI driver instance
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   A non-zero handle is returned on success.  This handle may then be used
 *   to configure the XEN1210 driver as necessary.  A NULL handle value is
 *   returned on failure.
 *
 ****************************************************************************/

XEN1210_HANDLE xen1210_instantiate(FAR struct spi_dev_s *dev,
                                   FAR struct xen1210_config_s *config)
{
  FAR struct xen1210_dev_s *priv;
  uint32_t regval;

  /* Allocate the XEN1210 driver instance */

  priv = (FAR struct xen1210_dev_s *)
         kmm_zalloc(sizeof(struct xen1210_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate the device structure!\n");
      return NULL;
    }

  /* Initialize the device state structure */

  nxmutex_init(&priv->lock);
  priv->config = config;

  priv->spi = dev;

  /* Attach the XEN1210 interrupt handler. */

  config->attach(config, (xen1210_handler_t)xen1210_interrupt,
                 (FAR void *)priv);

  /* Device initialization sequence */

  /* Power off */

  regval = (XEN1210_POWEROFF);

  xen1210_putdata(priv, regval);

  /* Timing */

  regval  = (XEN1210_TIMING);
  regval |= 0x131100;

  xen1210_putdata(priv, regval);

  /* Test */

  regval  = (XEN1210_TEST);
  regval |= 0x003a00;

  xen1210_putdata(priv, regval);

  /* Power on */

  regval = (XEN1210_POWERON);

  xen1210_putdata(priv, regval); /* X axis */

  config->clear(config);
  config->enable(config, true);

  /* Return our private data structure as an opaque handle */

  return (XEN1210_HANDLE)priv;
}

/****************************************************************************
 * Name: xen1210_register
 *
 * Description:
 *  This function will register the touchscreen driver as /dev/accelN where N
 *  is the minor device number
 *
 * Input Parameters:
 *   handle    - The handle previously returned by xen1210_register
 *   minor     - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int xen1210_register(XEN1210_HANDLE handle, int minor)
{
  FAR struct xen1210_dev_s *priv = (FAR struct xen1210_dev_s *)handle;
  char devname[DEV_NAMELEN];
  int ret;

  sninfo("handle=%p minor=%d\n", handle, minor);
  DEBUGASSERT(priv);

  /* Get exclusive access to the device structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      snerr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Register the character driver */

  snprintf(devname, sizeof(devname), DEV_FORMAT, minor);
  ret = register_driver(devname, &g_xen1210fops, 0444, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver %s: %d\n", devname, ret);
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  /* Indicate that the accelerometer was successfully initialized */

  priv->status |= XEN1210_STAT_INITIALIZED;  /* Accelerometer is initialized */
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: xen1210_getdata
 *
 * Description:
 *   Read 24-bit from XEN1210 buffer, read three times (3 sensors)
 *
 ****************************************************************************/

void xen1210_getdata(FAR struct xen1210_dev_s *priv)
{
  uint32_t regval;

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  xen1210_configspi(priv->spi);

  /* Select the XEN1210 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), true);

  /* Read three times 3 bytes = 24 bits * 3 */

  SPI_RECVBLOCK(priv->spi, &regval, 3);
  priv->sample.data_x = regval & 0xffffff;

  SPI_RECVBLOCK(priv->spi, &regval, 3);
  priv->sample.data_y = regval & 0xffffff;

  SPI_RECVBLOCK(priv->spi, &regval, 3);
  priv->sample.data_z = regval & 0xffffff;

  /* Deselect the XEN1210 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

#ifdef CONFIG_XEN1210_REGDEBUG
  _err("%02x->%02x\n", regaddr, regval);
#endif

  /* Feed sensor data to entropy pool */

  add_sensor_randomness((priv->sample.data_x << 8) ^
                        (priv->sample.data_y << 4) ^
                        (priv->sample.data_z << 4));
}

/****************************************************************************
 * Name: xen1210_putdata
 *
 * Description:
 *   Write 24-bit to XEN1210 buffer, write three times (3 sensors)
 *
 ****************************************************************************/

void xen1210_putdata(FAR struct xen1210_dev_s *priv, uint32_t regval)
{
#ifdef CONFIG_XEN1210_REGDEBUG
  _err("%02x<-%02x\n", regaddr, regval);
#endif

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  xen1210_configspi(priv->spi);

  /* Select the XEN1210 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), true);

  /* We need to write to 3 sensors in the daisy-chain */

  /* Write three times 3 bytes */

  SPI_SNDBLOCK(priv->spi, &regval, 3);
  SPI_SNDBLOCK(priv->spi, &regval, 3);
  SPI_SNDBLOCK(priv->spi, &regval, 3);

  /* Deselect the XEN1210 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
}

#endif /* CONFIG_SENSORS_XEN1210 */
