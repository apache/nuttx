/****************************************************************************
 * drivers/video/mipidsi/mipi_dsi_host_driver.c
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

#define MIPI_DSI_HOSTNAME_FMT "/dev/dsi%d/host"
#define MIPI_DSI_HOSTNAME_LEN 128

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Driver state structure */

struct mipi_dsi_host_driver_s
{
  FAR struct mipi_dsi_host *host;
  mutex_t lock;                           /* Mutual exclusion */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t dsi_host_read(FAR struct file *filep, FAR char *buffer,
                             size_t len);
static ssize_t dsi_host_write(FAR struct file *filep, FAR const char *buffer,
                              size_t len);
static int     dsi_host_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations dsi_host_fops =
{
  NULL,             /* open */
  NULL,             /* close */
  dsi_host_read,    /* read */
  dsi_host_write,   /* write */
  NULL,             /* seek */
  dsi_host_ioctl,   /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dsi_host_read
 ****************************************************************************/

static ssize_t dsi_host_read(FAR struct file *filep, FAR char *buffer,
                             size_t len)
{
  return 0; /* Return EOF */
}

/****************************************************************************
 * Name: DSI hostdrvr_write
 ****************************************************************************/

static ssize_t dsi_host_write(FAR struct file *filep, FAR const char *buffer,
                              size_t len)
{
  return len; /* Say that everything was written */
}

/****************************************************************************
 * Name: dsi_host_ioctl
 ****************************************************************************/

static int dsi_host_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct mipi_dsi_host_driver_s *priv;
  FAR struct inode *inode;
  FAR struct mipi_dsi_msg *msg;
  FAR struct mipi_dsi_host *host;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = inode->i_private;
  DEBUGASSERT(priv);

  /* Get exclusive access to the dsi host driver state structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Process the IOCTL command */

  switch (cmd)
    {
      case MIPIDSI_TRANSFER:
        {
          /* Get the reference to the mipi_dsi_msg structure */

          msg = (FAR struct mipi_dsi_msg *)((uintptr_t)arg);

          /* Get the reference to the mipi_dsi_host structure */

          host = priv->host;
          DEBUGASSERT(host != NULL && msg != NULL);

          ret = host->ops->transfer(host, msg);
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
 * Name: mipi_dsi_host_driver_register
 *
 * Description:
 *   Create and register the dsi host character driver.
 *
 *   The dsi host character driver is a simple character driver that
 *   supports dsi transfer.
 *
 * Input Parameters:
 *   host - An instance of the struct mipi_dsi_host
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int mipi_dsi_host_driver_register(FAR struct mipi_dsi_host *host)
{
  FAR struct mipi_dsi_host_driver_s *priv;
  char name[MIPI_DSI_HOSTNAME_LEN];
  int ret = -ENOMEM;

  DEBUGASSERT(host != NULL);

  priv = kmm_zalloc(sizeof(struct mipi_dsi_host_driver_s));
  if (priv != NULL)
    {
      priv->host = host;
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
      nxmutex_init(&priv->lock);
#endif

      snprintf(name, sizeof(name), MIPI_DSI_HOSTNAME_FMT, host->bus);
      ret = register_driver(name, &dsi_host_fops, 0666, priv);
      if (ret < 0)
        {
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
          nxmutex_destroy(&priv->lock);
#endif
          kmm_free(priv);
        }
    }

  return ret;
}
