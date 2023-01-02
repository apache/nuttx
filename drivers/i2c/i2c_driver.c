/****************************************************************************
 * drivers/i2c/i2c_driver.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mutex.h>

#ifdef CONFIG_I2C_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device naming ************************************************************/

#define DEVNAME_FMT    "/dev/i2c%d"
#define DEVNAME_FMTLEN (8 + 3 + 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Driver state structure */

struct i2c_driver_s
{
  FAR struct i2c_master_s *i2c;  /* Contained I2C lower half driver */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  mutex_t lock;                  /* Mutual exclusion */
  int16_t crefs;                 /* Number of open references */
  bool unlinked;                 /* True, driver has been unlinked */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     i2cdrvr_open(FAR struct file *filep);
static int     i2cdrvr_close(FAR struct file *filep);
#endif
static ssize_t i2cdrvr_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t i2cdrvr_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     i2cdrvr_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     i2cdrvr_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations i2cdrvr_fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  i2cdrvr_open,    /* open */
  i2cdrvr_close,   /* close */
#else
  NULL,            /* open */
  NULL,            /* close */
#endif
  i2cdrvr_read,    /* read */
  i2cdrvr_write,   /* write */
  NULL,            /* seek */
  i2cdrvr_ioctl,   /* ioctl */
  NULL,            /* mmap */
  NULL,            /* truncate */
  NULL             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , i2cdrvr_unlink /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2cdrvr_open
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int i2cdrvr_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct i2c_driver_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct i2c_driver_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Get exclusive access to the I2C driver state structure */

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
#endif

/****************************************************************************
 * Name: i2cdrvr_close
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int i2cdrvr_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct i2c_driver_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct i2c_driver_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Get exclusive access to the I2C driver state structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If the count has decremented to zero and the driver has been unlinked,
   * then commit Hara-Kiri now.
   */

  if (priv->crefs <= 0 && priv->unlinked)
    {
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return OK;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: i2cdrvr_read
 ****************************************************************************/

static ssize_t i2cdrvr_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  return 0; /* Return EOF */
}

/****************************************************************************
 * Name: i2cdrvr_write
 ****************************************************************************/

static ssize_t i2cdrvr_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  return len; /* Say that everything was written */
}

/****************************************************************************
 * Name: i2cdrvr_ioctl
 ****************************************************************************/

static int i2cdrvr_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct i2c_driver_s *priv;
  FAR struct i2c_transfer_s *transfer;
  int ret;

  i2cinfo("cmd=%x arg=%08lx\n", cmd, arg);

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct i2c_driver_s *)inode->i_private;
  DEBUGASSERT(priv);

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  /* Get exclusive access to the I2C driver state structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* Process the IOCTL command */

  switch (cmd)
    {
      /* Command:      I2CIOC_TRANSFER
       * Description:  Perform an I2C transfer
       * Argument:     A reference to an instance of struct i2c_transfer_s.
       * Dependencies: CONFIG_I2C_DRIVER
       */

      case I2CIOC_TRANSFER:
        {
          /* Get the reference to the i2c_transfer_s structure */

          transfer = (FAR struct i2c_transfer_s *)((uintptr_t)arg);
          DEBUGASSERT(transfer != NULL);

          /* Perform the transfer */

          ret = I2C_TRANSFER(priv->i2c, transfer->msgv, transfer->msgc);
        }
        break;

#ifdef CONFIG_I2C_RESET
      /* Command:      I2CIOC_RESET
       * Description:  Perform an I2C bus reset in an attempt to break loose
       *               stuck I2C devices.
       * Argument:     None
       * Dependencies: CONFIG_I2C_DRIVER && CONFIG_I2C_RESET
       */

      case I2CIOC_RESET:
        {
          ret = I2C_RESET(priv->i2c);
        }
        break;
#endif

      default:
        ret = -ENOTTY;
        break;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  nxmutex_unlock(&priv->lock);
#endif
  return ret;
}

/****************************************************************************
 * Name: i2cdrvr_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int i2cdrvr_unlink(FAR struct inode *inode)
{
  FAR struct i2c_driver_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = (FAR struct i2c_driver_s *)inode->i_private;

  /* Get exclusive access to the I2C driver state structure */

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
 * Name: i2c_register
 *
 * Description:
 *   Create and register the I2C character driver.
 *
 *   The I2C character driver is a simple character driver that supports I2C
 *   transfers.  The intent of this driver is to support I2C testing.  It is
 *   not suitable for use in any real driver application.
 *
 * Input Parameters:
 *   i2c - An instance of the lower half I2C driver
 *   bus - The I2C bus number.  This will be used as the I2C device minor
 *     number.  The I2C character device will be registered as /dev/i2cN
 *     where N is the minor number
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int i2c_register(FAR struct i2c_master_s *i2c, int bus)
{
  FAR struct i2c_driver_s *priv;
  char devname[DEVNAME_FMTLEN];
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL && (unsigned)bus < 1000);

  /* Allocate a I2C character device structure */

  priv = (FAR struct i2c_driver_s *)kmm_zalloc(sizeof(struct i2c_driver_s));
  if (priv)
    {
      /* Initialize the I2C character device structure */

      priv->i2c = i2c;
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
      nxmutex_init(&priv->lock);
#endif

      /* Create the character device name */

      snprintf(devname, DEVNAME_FMTLEN, DEVNAME_FMT, bus);
      ret = register_driver(devname, &i2cdrvr_fops, 0666, priv);
      if (ret < 0)
        {
          /* Free the device structure if we failed to create the character
           * device.
           */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
          nxmutex_destroy(&priv->lock);
#endif
          kmm_free(priv);
          return ret;
        }

      /* Return the result of the registration */

      return OK;
    }

  return -ENOMEM;
}

#endif /* CONFIG_I2C_DRIVER */
