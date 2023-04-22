/****************************************************************************
 * drivers/video/mipidsi/mipi_dsi_device_driver.c
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

#include <debug.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>

#include "mipi_dsi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device naming ************************************************************/

#define MIPI_DSI_DEVNAME_FMT "/dev/dsi%d/dev.%d.%s"
#define MIPI_DSI_DEVNAME_LEN  128

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mipi_dsi_device_driver_s
{
  FAR struct mipi_dsi_device *dsi_dev;
  mutex_t lock;                           /* Mutual exclusion */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t dsi_dev_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t dsi_dev_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int     dsi_dev_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_dsi_dev_fops =
{
  NULL,            /* open */
  NULL,            /* close */
  dsi_dev_read,    /* read */
  dsi_dev_write,   /* write */
  NULL,            /* seek */
  dsi_dev_ioctl,   /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dsi_dev_read
 ****************************************************************************/

static ssize_t dsi_dev_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  return 0; /* Return EOF */
}

/****************************************************************************
 * Name: dsi_dev_write
 ****************************************************************************/

static ssize_t dsi_dev_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return buflen; /* Say that everything was written */
}

/****************************************************************************
 * Name: dsi_dev_ioctl
 ****************************************************************************/

static int dsi_dev_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct mipi_dsi_device_driver_s *priv;
  int ret = OK;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct mipi_dsi_device_driver_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Get exclusive access to the DSI device driver state structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Process the IOCTL command */

  switch (cmd)
    {
      case MIPIDSI_GETDEVLANES:
        {
          FAR uint16_t *planes = (FAR uint16_t *)((uintptr_t)arg);
          DEBUGASSERT(planes != NULL);

          *planes = priv->dsi_dev->lanes;
        }
        break;
      case MIPIDSI_GETDEVFMT:
        {
          FAR uint32_t *fmt = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(fmt != NULL);

          *fmt = priv->dsi_dev->format;
        }
        break;
      case MIPIDSI_GETDEVMODE:
        {
          FAR uint32_t *mode = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(mode != NULL);

          *mode = priv->dsi_dev->mode_flags;
        }
        break;
      case MIPIDSI_GETDEVHSRATE:
        {
          FAR uint32_t *hsrate = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(hsrate != NULL);

          *hsrate = priv->dsi_dev->hs_rate;
        }
        break;
      case MIPIDSI_GETDEVLPRATE:
        {
          FAR uint32_t *lprate = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(lprate != NULL);

          *lprate = priv->dsi_dev->lp_rate;
        }
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mipi_dsi_device_driver_register
 *
 * Description:
 *   Create and register the dsi device character driver.
 *
 *   The dsi device character driver is a simple character driver that
 *   supports get dsi device params.
 *
 * Input Parameters:
 *   device - An instance of the struct mipi_dsi_device
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int mipi_dsi_device_driver_register(FAR struct mipi_dsi_device *device)
{
  FAR struct mipi_dsi_device_driver_s *priv;
  FAR struct mipi_dsi_host *host;
  char devpath[MIPI_DSI_DEVNAME_LEN];
  int ret = OK;

  DEBUGASSERT(device != NULL && device->host != NULL);

  priv = kmm_zalloc(sizeof(struct mipi_dsi_device_driver_s));
  if (priv == NULL)
    {
      verr("mipi dsi device driver register failed, no memory.\n");
      return -ENOMEM;
    }

  priv->dsi_dev = device;
  nxmutex_init(&priv->lock);

  host = device->host;
  snprintf(devpath, sizeof(devpath), MIPI_DSI_DEVNAME_FMT, host->bus,
           device->channel, device->name);

  ret = register_driver(devpath, &g_dsi_dev_fops, 0666, priv);
  if (ret < 0)
    {
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
    }

  return ret;
}
