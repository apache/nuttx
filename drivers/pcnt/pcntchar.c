/****************************************************************************
 * drivers/pcnt/pcntchar.c
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
#include <nuttx/mutex.h>
#include <nuttx/pcnt/pcnt.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVNAME_FMT    "/dev/pcnt%d"
#define DEVNAME_FMTLEN (9 + 3 + 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pcnt_driver_s
{
  /* The lower half PCNT driver */

  FAR struct pcnt_dev_s *pcnt;

  /* The minor identification of the driver. It's provided by the lower half
   * driver and it can represent the channel being used.
   */

  int minor;

  mutex_t lock;               /* Assures mutually exclusive access */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pcnt_open(FAR struct file *filep);
static int pcnt_close(FAR struct file *filep);
static ssize_t pcnt_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static int pcnt_ioctl(struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pcnt_channel_fops =
{
  pcnt_open,  /* open  */
  pcnt_close, /* close */
  pcnt_read,  /* read  */
  NULL,       /* write */
  NULL,       /* seek  */
  pcnt_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcnt_open
 *
 * Description:
 *   Prepare the pcnt peripheral for use. This method just calls the lower
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

static int pcnt_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct pcnt_driver_s *priv;
  int ret = OK;

  /* Get our private data structure */

  inode = filep->f_inode;

  priv = inode->i_private;
  DEBUGASSERT(priv);

  if (priv->pcnt->ops->open)
    {
      ret = priv->pcnt->ops->open(priv->pcnt);
    }

  return ret;
}

/****************************************************************************
 * Name: pcnt_close
 *
 * Description:
 *   Close the pcnt peripheral after use. This method just calls the lower
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

static int pcnt_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct pcnt_driver_s *priv;
  int ret = OK;

  /* Get our private data structure */

  inode = filep->f_inode;

  priv = inode->i_private;
  DEBUGASSERT(priv);

  if (priv->pcnt->ops->close)
    {
      ret = priv->pcnt->ops->close(priv->pcnt);
    }

  return ret;
}

/****************************************************************************
 * Name: pcnt_read
 *
 * Description:
 *   This function reads data from the pcnt device into the provided buffer.
 *   The read operation is performed by the read function pointer in the
 *   pcnt device's operations structure, if it exists.
 *
 * Input Parameters:
 *   filep   - Pointer to the file structure.
 *   buffer  - Pointer to the buffer where the read data should be stored.
 *   buflen  - The maximum amount of data to be read.
 *
 * Returned Value:
 *   Returns the number of bytes read from the pcnt device; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static ssize_t pcnt_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct inode *inode;
  FAR struct pcnt_driver_s *priv;
  ssize_t nread = 0;

  /* Get our private data structure */

  inode = filep->f_inode;

  priv = inode->i_private;
  DEBUGASSERT(priv);

  if (priv->pcnt->ops->read)
    {
      nread = priv->pcnt->ops->read(priv->pcnt, buffer, buflen);
    }

  return nread;
}

/****************************************************************************
 * Name: pcnt_ioctl
 *
 * Description:
 *   Perform a device ioctl
 *
 * Input Parameters:
 *   filep - The pointer of file, represents each user using the sensor
 *   cmd   - The ioctl command
 *   arg   - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pcnt_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct pcnt_driver_s *priv = inode->i_private;
  int ret;

  ret = priv->pcnt->ops->ioctl(priv->pcnt, cmd, arg);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcntchar_register
 *
 * Description:
 *   Create and register the pcnt character driver.
 *
 * Input Parameters:
 *   pcnt - An instance of the lower half pcnt driver
 *
 * Returned Value:
 *   OK if the driver was successfully registered; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int pcntchar_register(FAR struct pcnt_dev_s *pcnt)
{
  FAR struct pcnt_driver_s *priv;
  char devname[DEVNAME_FMTLEN];
  size_t dev_size = sizeof(struct pcnt_driver_s);
  int ret;

  /* Sanity check */

  DEBUGASSERT(pcnt != NULL && (unsigned)pcnt->minor < 1000);

  /* Allocate a pcnt character device structure */

  priv = kmm_zalloc(dev_size);
  if (priv)
    {
      /* Initialize the pcnt character device structure */

      priv->pcnt = pcnt;
      priv->minor = pcnt->minor;
      nxmutex_init(&priv->lock);

      /* Create the character device name */

      snprintf(devname, sizeof(devname), DEVNAME_FMT, priv->minor);
      ret = register_driver(devname, &g_pcnt_channel_fops, 0666, priv);
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
