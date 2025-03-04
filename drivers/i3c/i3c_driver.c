/****************************************************************************
 * drivers/i3c/i3c_driver.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/i3c/master.h>
#include <nuttx/i3c/i3c_driver.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVNAME_FMT    "/dev/i3c%d"
#define DEVNAME_FMTLEN (8 + 3 + 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Driver state structure */

struct i3c_driver_s
{
  /* Contained I3C lower half driver */

  FAR struct i3c_master_controller *master;

  /* Mutual exclusion */

  mutex_t lock;

  /* Number of open references */

  int16_t crefs;

  /* True, driver has been unlinked */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  i3cdrvr_open(FAR struct file *filep);
static int  i3cdrvr_close(FAR struct file *filep);
static ssize_t  i3cdrvr_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t  i3cdrvr_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);
static int  i3cdrvr_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);
static int i3cdrvr_unlink(FAR struct inode *inode);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_i3cdrvr_fops =
{
  i3cdrvr_open,    /* open */
  i3cdrvr_close,   /* close */
  i3cdrvr_read,    /* read */
  i3cdrvr_write,   /* write */
  NULL,            /* seek */
  i3cdrvr_ioctl,   /* ioctl */
  NULL,            /* mmap */
  NULL,            /* truncate */
  NULL,            /* poll */
  NULL,            /* readv */
  NULL             /* writev */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , i3cdrvr_unlink /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i3cdrvr_open
 ****************************************************************************/

static int i3cdrvr_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct i3c_driver_s *priv;
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = inode->i_private;
  DEBUGASSERT(priv);

  /* Get exclusive access to the I3C driver state structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: i3cdrvr_close
 ****************************************************************************/

static int i3cdrvr_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct i3c_driver_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = inode->i_private;
  DEBUGASSERT(priv);

  /* Get exclusive access to the I3C driver state structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If the count has decremented to zero,then commit Hara-Kiri now. */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->crefs <= 0 && priv->unlinked)
#else
  if (priv->crefs <= 0)
#endif
    {
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return OK;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: i3cdrvr_read
 ****************************************************************************/

static ssize_t i3cdrvr_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  return 0; /* Return EOF */
}

/****************************************************************************
 * Name: i3cdrvr_write
 ****************************************************************************/

static ssize_t i3cdrvr_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  return len; /* Say that everything was written */
}

/****************************************************************************
 * Name: i3cdrvr_ioctl
 ****************************************************************************/

static int i3cdrvr_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct i3c_driver_s *priv;
  FAR struct i3c_transfer_s *transfer;
  FAR struct i3c_dev_desc *desc;
  bool obtain = false;
  uint16_t manufid;
  uint16_t partid;
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = inode->i_private;
  DEBUGASSERT(priv);

  /* Get exclusive access to the I3C driver state structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  transfer = (FAR struct i3c_transfer_s *)((uintptr_t)arg);
  DEBUGASSERT(transfer != NULL);

  i3c_bus_normaluse_lock(&priv->master->bus);
  i3c_bus_for_each_i3cdev(&priv->master->bus, desc)
    {
      manufid = I3C_PID_MANUF_ID(desc->info.pid);
      partid = I3C_PID_PART_ID(desc->info.pid);

      if (manufid == transfer->manufid && partid == transfer->partid)
        {
          obtain = true;
          break;
        }
    }

  i3c_bus_normaluse_unlock(&priv->master->bus);
  if (!obtain)
    {
      nxmutex_unlock(&priv->lock);
      return -ENXIO;
    }

  /* Process the IOCTL command */

  switch (cmd)
    {
      case I3CIOC_PRIV_XFERS:

        DEBUGASSERT(transfer->xfers != NULL);

         /* Perform the transfer */

        ret = i3c_device_do_priv_xfers(desc->dev, transfer->xfers,
                                       transfer->nxfers);
        break;
      case I3CIOC_GET_DEVINFO:

        DEBUGASSERT(transfer->info != NULL);

        /* Perform the get specified i3c device operating */

        i3c_device_get_info(desc->dev, transfer->info);
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: i3cdrvr_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int i3cdrvr_unlink(FAR struct inode *inode)
{
  FAR struct i3c_driver_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = (FAR struct i3c_driver_s *)inode->i_private;

  /* Get exclusive access to the I3C driver state structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Are there open references to the driver data structure? */

  if (priv->crefs <= 0)
    {
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return OK;
    }

  /* No... just mark the driver as unlinked and free the resources when the
   * last client closes their reference to the driver.
   */

  priv->unlinked = true;
  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i3c_register
 *
 * Description:
 *   Create and register the I3C character driver.
 *
 *   The I3C character driver is a simple character driver that supports I3C
 *   transfers.  The intent of this driver is to support I3C testing.  It is
 *   not suitable for use in any real driver application.
 *
 * Input Parameters:
 *   master - the lower half of an the controller object about I3C master
 *     driver.
 *   bus    - The I3C bus number.  This will be used as the I3C device minor
 *     number.  The I3C character device will be registered as /dev/i3cN
 *     where N is the minor number
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int i3c_register(FAR struct i3c_master_controller *master, int bus)
{
  FAR struct i3c_driver_s *priv;
  char devname[DEVNAME_FMTLEN];
  int ret;

  /* Sanity check */

  DEBUGASSERT(master != NULL && (unsigned)bus < 1000);

  priv = kmm_zalloc(sizeof(struct i3c_driver_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->master = master;
  nxmutex_init(&priv->lock);

  /* Create the character device name */

  snprintf(devname, sizeof(devname), DEVNAME_FMT, bus);
  ret = register_driver(devname, &g_i3cdrvr_fops, 0666, priv);
  if (ret < 0)
    {
      /* Free the device structure if we failed to create the character
       * device.
       */

      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
    }

  /* Return the result of the registration */

  return ret;
}

