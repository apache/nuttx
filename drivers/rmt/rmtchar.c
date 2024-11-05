/****************************************************************************
 * drivers/rmt/rmtchar.c
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

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/circbuf.h>
#include <nuttx/mutex.h>
#include <nuttx/rmt/rmt.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVNAME_FMT    "/dev/rmt%d"
#define DEVNAME_FMTLEN (8 + 3 + 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rmt_driver_s
{
  /* The lower half RMT driver */

  FAR struct rmt_dev_s *rmt;

  /* The minor identification of the driver. It's provided by the lower half
   * driver and it can represent the channel being used.
   */

  int minor;

  mutex_t lock;               /* Assures mutually exclusive access */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rmt_open(FAR struct file *filep);
static int rmt_close(FAR struct file *filep);
static ssize_t rmt_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t rmt_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_rmt_channel_fops =
{
  rmt_open,     /* open  */
  rmt_close,    /* close */
  rmt_read,     /* read  */
  rmt_write,    /* write */
  NULL,         /* seek  */
  NULL,         /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rmt_open
 *
 * Description:
 *   Prepare the RMT peripheral for use. This method just calls the lower
 *   half `open` routine if it exists.
 *
 * Input Parameters:
 *   filep - Pointer system file data
 *
 * Returned Value:
 *   The return code of the lower half `open` routine if it exists.
 *   Please check device-specific driver for more information.
 *
 ****************************************************************************/

static int rmt_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct rmt_driver_s *priv;
  int ret = OK;

  /* Get our private data structure */

  inode = filep->f_inode;

  priv = inode->i_private;
  DEBUGASSERT(priv);

  if (priv->rmt->ops->open)
    {
      ret = priv->rmt->ops->open(priv->rmt);
    }

  return ret;
}

/****************************************************************************
 * Name: rmt_close
 *
 * Description:
 *   Close the RMT peripheral after use. This method just calls the lower
 *   half `close` routine if it exists.
 *
 * Input Parameters:
 *   filep - Pointer system file data
 *
 * Returned Value:
 *   The return code of the lower half `close` routine if it exists.
 *   Please check device-specific driver for more information.
 *
 ****************************************************************************/

static int rmt_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct rmt_driver_s *priv;
  int ret = OK;

  /* Get our private data structure */

  inode = filep->f_inode;

  priv = inode->i_private;
  DEBUGASSERT(priv);

  if (priv->rmt->ops->close)
    {
      ret = priv->rmt->ops->close(priv->rmt);
    }

  return ret;
}

/****************************************************************************
 * Name: rmt_read
 *
 * Description:
 *   This function reads data from the RMT device into the provided buffer.
 *   The read operation is performed by the read function pointer in the
 *   RMT device's operations structure, if it exists.
 *
 * Input Parameters:
 *   filep   - Pointer to the file structure.
 *   buffer  - Pointer to the buffer where the read data should be stored.
 *   buflen  - The maximum amount of data to be read.
 *
 * Returned Value:
 *   Returns the number of bytes read from the RMT device; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static ssize_t rmt_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct inode *inode;
  FAR struct rmt_driver_s *priv;
  ssize_t nread = 0;

  /* Get our private data structure */

  inode = filep->f_inode;

  priv = inode->i_private;
  DEBUGASSERT(priv);

  if (priv->rmt->ops->read)
    {
      int ret = priv->rmt->ops->read(priv->rmt, buffer, buflen);
      if (ret < 0)
        {
          return ret;
        }

      for (; ; )
        {
          nread = circbuf_read(priv->rmt->circbuf , buffer, buflen);
          if (nread != 0 || (filep->f_oflags & O_NONBLOCK))
            {
              break;
            }

          while (circbuf_is_empty(priv->rmt->circbuf))
            {
              nxsem_wait_uninterruptible(priv->rmt->recvsem);
            }
        }
    }

  return nread;
}

/****************************************************************************
 * Name: rmt_write
 *
 * Description:
 *   Write to the RMT peripheral. This method just calls the lower half
 *   `write` routine if it exists.
 *
 * Input Parameters:
 *   filep  - Pointer system file data
 *   buffer - Data to write to the RMT device
 *   buflen - Number of bytes requested to write
 *
 * Returned Value:
 *   Number of bytes that has been successfully written, or 0 when no
 *   bytes could be written for any reason.
 *
 ****************************************************************************/

static ssize_t rmt_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  FAR struct inode *inode;
  FAR struct rmt_driver_s *priv;
  ssize_t nwritten = 0;

  /* Get our private data structure */

  inode = filep->f_inode;

  priv = inode->i_private;
  DEBUGASSERT(priv);

  if (priv->rmt->ops->write)
    {
      nwritten = priv->rmt->ops->write(priv->rmt, buffer, buflen);
    }

  return nwritten;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rmtchar_register
 *
 * Description:
 *   Create and register the RMT character driver.
 *
 *   The RMT character driver is a simple character driver that supports RMT
 *   transfers via read() and write(). This driver is primarily intended to
 *   support RMT testing. It is not suitable for use in any real driver
 *   application in its current form because its buffer management heuristics
 *   are dependent on the lower half driver (device-specific).
 *
 * Input Parameters:
 *   rmt - An instance of the lower half RMT driver
 *
 * Returned Value:
 *   OK if the driver was successfully registered; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int rmtchar_register(FAR struct rmt_dev_s *rmt)
{
  FAR struct rmt_driver_s *priv;
  char devname[DEVNAME_FMTLEN];
  size_t dev_size = sizeof(struct rmt_driver_s);
  int ret;

  /* Sanity check */

  DEBUGASSERT(rmt != NULL && (unsigned)rmt->minor < 1000);

  /* Allocate a RMT character device structure */

  priv = kmm_zalloc(dev_size);
  if (priv)
    {
      /* Initialize the RMT character device structure */

      priv->rmt = rmt;
      priv->minor = rmt->minor;
      nxmutex_init(&priv->lock);

      /* Create the character device name */

      snprintf(devname, sizeof(devname), DEVNAME_FMT, priv->minor);
      ret = register_driver(devname, &g_rmt_channel_fops, 0666, priv);
      if (ret < 0)
        {
          /* Free the device structure if we failed to create the character
           * device.
           */

          nxmutex_destroy(&priv->lock);
          kmm_free(priv);
          return ret;
        }

      /* Return the result of the registration */

      return ret;
    }

  return -ENOMEM;
}
