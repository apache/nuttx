/****************************************************************************
 * drivers/i2c/i2c_slave_driver.c
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

#include <assert.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/param.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_slave.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVNAME_FMT    "/dev/i2cslv%d"
#define DEVNAME_FMTLEN (11 + 3 + 1)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int i2c_slave_open(FAR struct file *filep);
static int i2c_slave_close(FAR struct file *filep);
static ssize_t i2c_slave_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t i2c_slave_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static int i2c_slave_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup);

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int i2c_slave_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct i2c_slave_driver_s
{
  /* Slave private data */

  FAR struct i2c_slave_s *dev;

  /* Slave send buffer */

  uint8_t write_buffer[CONFIG_I2C_SLAVE_WRITEBUFSIZE];

  /* Slave receive buffer */

  uint8_t read_buffer[CONFIG_I2C_SLAVE_READBUFSIZE];

  /* Slave receive buffer length */

  size_t read_length;

  /* Read buffer index */

  size_t read_index;

  /* Wait for transfer to complete */

  sem_t wait;

  /* I2C Slave write flag */

  bool writeable;

  /* Mutual exclusion */

  mutex_t lock;

  /* The poll waiter */

  FAR struct pollfd *fds[CONFIG_I2C_SLAVE_NPOLLWAITERS];

  /* Number of open references */

  int16_t crefs;

  /* I2C Slave address */

  int addr;

  /* The number of address bits provided (7 or 10) */

  int nbits;

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked; /* Indicates if the driver has been unlinked */
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_i2cslavefops =
{
  i2c_slave_open,     /* open */
  i2c_slave_close,    /* close */
  i2c_slave_read,     /* read */
  i2c_slave_write,    /* write */
  NULL,               /* seek */
  NULL,               /* ioctl */
  NULL,               /* mmap */
  NULL,               /* truncate */
  i2c_slave_poll,     /* poll */
  NULL,               /* readv */
  NULL                /* writev */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , i2c_slave_unlink  /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_slave_open
 *
 * Description:
 *   This function is called whenever the I2C Slave device is opened.
 *
 * Input Parameters:
 *   filep  - File structure instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

static int i2c_slave_open(FAR struct file *filep)
{
  FAR struct i2c_slave_driver_s *priv;
  int ret;

  DEBUGASSERT(filep->f_inode->i_private != NULL);

  /* Get our private data structure */

  priv = filep->f_inode->i_private;

  /* Get exclusive access to the I2C Slave driver state structure */

  nxmutex_lock(&priv->lock);

  /* I2c slave initialize */

  if (priv->dev->ops->setup != NULL && priv->crefs == 0)
    {
      ret = I2CS_SETUP(priv->dev);
      if (ret < 0)
        {
          goto out;
        }
    }

  /* Set i2c slave address */

  ret = I2CS_SETOWNADDRESS(priv->dev, priv->addr, priv->nbits);
  if (ret < 0)
    {
      if (priv->dev->ops->shutdown != NULL)
        {
          ret = I2CS_SHUTDOWN(priv->dev);
        }
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: i2c_slave_close
 *
 * Description:
 *   This routine is called when the I2C Slave device is closed.
 *
 * Input Parameters:
 *   filep  - File structure instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

static int i2c_slave_close(FAR struct file *filep)
{
  FAR struct i2c_slave_driver_s *priv;
  int ret = OK;

  DEBUGASSERT(filep->f_inode->i_private != NULL);

  /* Get our private data structure */

  priv = filep->f_inode->i_private;

  /* Get exclusive access to the I2C slave driver state structure */

  nxmutex_lock(&priv->lock);

  /* I2c slave uninitialize */

  if (priv->dev->ops->shutdown != NULL && priv->crefs == 1)
    {
      ret = I2CS_SHUTDOWN(priv->dev);
      if (ret < 0)
        {
          goto out;
        }
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->crefs <= 0 && priv->unlinked)
    {
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      filep->f_inode->i_private = NULL;
      return OK;
    }
#endif

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: i2c_slave_read
 *
 * Description:
 *   This routine is called when the application requires to read the data
 *   received by the I2C Slave device.
 *
 * Input Parameters:
 *   filep  - File structure instance
 *   buffer - User-provided to save the data
 *   buflen - The maximum size of the user-provided buffer
 *
 * Returned Value:
 *   The positive non-zero number of bytes read on success, 0 on if an
 *   end-of-file condition, or a negated errno value on any failure.
 *
 ****************************************************************************/

static ssize_t i2c_slave_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  FAR struct i2c_slave_driver_s *priv;
  int ret;

  DEBUGASSERT(filep->f_inode->i_private != NULL);

  /* Get our private data structure */

  priv = filep->f_inode->i_private;

  /* Get exclusive access to the I2C Slave driver state structure */

  nxmutex_lock(&priv->lock);

  while (priv->read_length == 0)
    {
      nxmutex_unlock(&priv->lock);
      if (filep->f_oflags & O_NONBLOCK)
        {
          return -EAGAIN;
        }

      ret = nxsem_wait(&priv->wait);
      if (ret < 0)
        {
          return ret;
        }

      nxmutex_lock(&priv->lock);
    }

  buflen = MIN(buflen, priv->read_length);
  memcpy(buffer, priv->read_buffer + priv->read_index, buflen);
  priv->read_index += buflen;
  priv->read_length -= buflen;
  nxmutex_unlock(&priv->lock);
  return buflen;
}

/****************************************************************************
 * Name: i2c_slave_write
 *
 * Description:
 *   This routine is called when the application needs to enqueue data to be
 *   transferred at the next leading clock edge of the I2C Slave controller.
 *
 * Input Parameters:
 *   filep  - Instance of struct file to use with the write
 *   buffer - Data to write
 *   buflen - Length of data to write in bytes
 *
 * Returned Value:
 *   On success, the number of bytes written are returned (zero indicates
 *   nothing was written). On any failure, a negated errno value is returned.
 *
 ****************************************************************************/

static ssize_t i2c_slave_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen)
{
  FAR struct i2c_slave_driver_s *priv;
  size_t write_bytes;
  int ret;

  DEBUGASSERT(filep->f_inode->i_private != NULL);

  /* Get our private data structure */

  priv = filep->f_inode->i_private;

  /* Get exclusive access to the I2C Slave driver state structure */

  nxmutex_lock(&priv->lock);

  write_bytes = MIN(buflen, CONFIG_I2C_SLAVE_WRITEBUFSIZE);
  memcpy(priv->write_buffer, buffer, write_bytes);
  ret = I2CS_WRITE(priv->dev, priv->write_buffer, write_bytes);
  if (ret >= 0)
    {
      priv->writeable = false;
    }

  nxmutex_unlock(&priv->lock);
  return ret < 0 ? ret : write_bytes;
}

/****************************************************************************
 * Name: i2c_slave_poll
 ****************************************************************************/

static int i2c_slave_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup)
{
  FAR struct i2c_slave_driver_s *priv;
  int ret = OK;
  int i;

  DEBUGASSERT(filep->f_inode->i_private != NULL);

  /* Get our private data structure */

  priv = filep->f_inode->i_private;

  /* Get exclusive access to the I2C Slave driver state structure */

  nxmutex_lock(&priv->lock);
  if (setup)
    {
      pollevent_t eventset = 0;

      for (i = 0; i < CONFIG_I2C_SLAVE_NPOLLWAITERS; i++)
        {
          if (!priv->fds[i])
            {
              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i == CONFIG_I2C_SLAVE_NPOLLWAITERS)
        {
          ret = -EBUSY;
          goto out;
        }

      if (priv->read_length > 0)
        {
          eventset |= POLLIN;
        }

      if (priv->writeable)
        {
          eventset |= POLLOUT;
        }

      poll_notify(priv->fds, CONFIG_I2C_SLAVE_NPOLLWAITERS, eventset);
    }
  else if (fds->priv != NULL)
    {
      FAR struct pollfd **slot = fds->priv;
      *slot     = NULL;
      fds->priv = NULL;
    }

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: i2c_slave_unlink
 *
 * Description:
 *   This routine is called when the I2C Slave device is unlinked.
 *
 * Input Parameters:
 *   inode  - The inode associated with the I2C Slave device
 *
 * Returned Value:
 *   Zero is returned on success; a negated value is returned on any failure.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int i2c_slave_unlink(FAR struct inode *inode)
{
  FAR struct i2c_slave_driver_s *priv;

  DEBUGASSERT(inode->i_private != NULL);

  /* Get our private data structure */

  priv = inode->i_private;

  /* Get exclusive access to the I2C Slave driver state structure */

  nxmutex_lock(&priv->lock);

  /* Are there open references to the driver data structure? */

  if (priv->crefs <= 0)
    {
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      inode->i_private = NULL;
      return OK;
    }

  /* No... just mark the driver as unlinked and free the resources when the
   * last client closes their reference to the driver.
   */

  priv->unlinked = true;
  nxmutex_unlock(&priv->lock);
  return OK;
}
#endif

static int i2c_slave_callback(FAR void *arg, i2c_slave_complete_t status,
                              size_t length)
{
  FAR struct i2c_slave_driver_s *priv = arg;
  pollevent_t events;
  int semcount;

  /* Get exclusive access to the I2C Slave driver state structure */

  nxmutex_lock(&priv->lock);

  if (status == I2CS_RX_COMPLETE)
    {
      events = POLLIN;
      priv->read_index = 0;
      priv->read_length = length;

      while (nxsem_get_value(&priv->wait, &semcount) >= 0 && semcount <= 0)
        {
          nxsem_post(&priv->wait);
        }
    }
  else
    {
      events = POLLOUT;
      priv->writeable = true;
    }

  nxmutex_unlock(&priv->lock);
  poll_notify(priv->fds, CONFIG_I2C_SLAVE_NPOLLWAITERS, events);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_slave_register
 *
 * Description:
 *   Register the I2C Slave character device driver as 'devpath'.
 *
 * Input Parameters:
 *   dev   - An instance of the I2C Slave interface to use to communicate
 *           with the I2C Slave device
 *   bus   - The I2C Slave bus number. This will be used as the I2C device
 *           minor number. The I2C Slave character device will be
 *           registered as /dev/i2cslvN where N is the minor number
 *   addr  - I2C Slave address
 *   nbits - The number of address bits provided (7 or 10)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int i2c_slave_register(FAR struct i2c_slave_s *dev, int bus, int addr,
                       int nbits)
{
  FAR struct i2c_slave_driver_s *priv;
  char devname[DEVNAME_FMTLEN];
  int ret;

  /* Sanity check */

  DEBUGASSERT(dev != NULL && (unsigned int)bus < 1000);

  priv = kmm_zalloc(sizeof(struct i2c_slave_driver_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  snprintf(devname, sizeof(devname), DEVNAME_FMT, bus);
  ret = register_driver(devname, &g_i2cslavefops, 0666, priv);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  nxsem_init(&priv->wait, 0, 0);
  nxmutex_init(&priv->lock);
  priv->dev = dev;
  priv->addr = addr;
  priv->nbits = nbits;
  priv->writeable = true;

  ret = I2CS_READ(priv->dev, priv->read_buffer,
                  CONFIG_I2C_SLAVE_READBUFSIZE);
  if (ret < 0)
    {
      goto out;
    }

  ret = I2CS_REGISTERCALLBACK(priv->dev, i2c_slave_callback, priv);
  if (ret >= 0)
    {
      return OK;
    }

out:
  nxmutex_destroy(&priv->lock);
  nxsem_destroy(&priv->wait);
  unregister_driver(devname);
  kmm_free(priv);
  return ret;
}
