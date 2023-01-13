/****************************************************************************
 * drivers/usrsock/usrsock_dev.c
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
#if defined(CONFIG_NET_USRSOCK_DEVICE)

#include <sys/types.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/net/net.h>
#include <nuttx/net/usrsock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NET_USRSOCKDEV_NPOLLWAITERS
#  define CONFIG_NET_USRSOCKDEV_NPOLLWAITERS 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct usrsockdev_s
{
  mutex_t devlock; /* Lock for device node */
  uint8_t ocount;  /* The number of times the device has been opened */
  struct
  {
    FAR const struct iovec *iov;    /* Pending request buffers */
    int                     iovcnt; /* Number of request buffers */
    size_t                  pos;    /* Reader position on request buffer */
  } req;
  FAR struct pollfd *pollfds[CONFIG_NET_USRSOCKDEV_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static ssize_t usrsockdev_read(FAR struct file *filep, FAR char *buffer,
                               size_t len);

static ssize_t usrsockdev_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len);

static off_t usrsockdev_seek(FAR struct file *filep, off_t offset,
                             int whence);

static int usrsockdev_open(FAR struct file *filep);

static int usrsockdev_close(FAR struct file *filep);

static int usrsockdev_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_usrsockdevops =
{
  usrsockdev_open,    /* open */
  usrsockdev_close,   /* close */
  usrsockdev_read,    /* read */
  usrsockdev_write,   /* write */
  usrsockdev_seek,    /* seek */
  NULL,               /* ioctl */
  NULL,               /* mmap */
  NULL,               /* truncate */
  usrsockdev_poll     /* poll */
};

static struct usrsockdev_s g_usrsockdev =
{
  NXMUTEX_INITIALIZER
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsockdev_is_opened
 ****************************************************************************/

static bool usrsockdev_is_opened(FAR struct usrsockdev_s *dev)
{
  bool ret = true;

  if (dev->ocount == 0)
    {
      ret = false; /* No usrsock daemon running. */
    }

  return ret;
}

/****************************************************************************
 * Name: usrsockdev_read
 ****************************************************************************/

static ssize_t usrsockdev_read(FAR struct file *filep, FAR char *buffer,
                               size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usrsockdev_s *dev;
  int ret;

  if (len == 0)
    {
      return 0;
    }

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  DEBUGASSERT(inode);

  dev = inode->i_private;

  DEBUGASSERT(dev);

  ret = nxmutex_lock(&dev->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Is request available? */

  if (dev->req.iov)
    {
      ssize_t rlen;

      /* Copy request to user-space. */

      rlen = usrsock_iovec_get(buffer, len, dev->req.iov, dev->req.iovcnt,
                               dev->req.pos, NULL);
      if (rlen < 0)
        {
          /* Tried reading beyond buffer. */

          len = 0;
        }
      else
        {
          dev->req.pos += rlen;
          len = rlen;
        }
    }
  else
    {
      len = 0;
    }

  nxmutex_unlock(&dev->devlock);
  return len;
}

/****************************************************************************
 * Name: usrsockdev_seek
 ****************************************************************************/

static off_t usrsockdev_seek(FAR struct file *filep, off_t offset,
                             int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usrsockdev_s *dev;
  off_t pos;
  int ret;

  if (whence != SEEK_CUR && whence != SEEK_SET)
    {
      return -EINVAL;
    }

  DEBUGASSERT(inode);

  dev = inode->i_private;

  DEBUGASSERT(dev);

  ret = nxmutex_lock(&dev->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Is request available? */

  if (dev->req.iov)
    {
      ssize_t rlen;

      if (whence == SEEK_CUR)
        {
          pos = dev->req.pos + offset;
        }
      else
        {
          pos = offset;
        }

      /* Copy request to user-space. */

      rlen = usrsock_iovec_get(NULL, 0, dev->req.iov, dev->req.iovcnt,
                               pos, NULL);
      if (rlen < 0)
        {
          /* Tried seek beyond buffer. */

          pos = -EINVAL;
        }
      else
        {
          dev->req.pos = pos;
        }
    }
  else
    {
      pos = 0;
    }

  nxmutex_unlock(&dev->devlock);
  return pos;
}

/****************************************************************************
 * Name: usrsockdev_write
 ****************************************************************************/

static ssize_t usrsockdev_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usrsockdev_s *dev;
  bool req_done = false;
  ssize_t ret = 0;

  if (len == 0)
    {
      return 0;
    }

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  DEBUGASSERT(inode);

  dev = inode->i_private;

  DEBUGASSERT(dev);

  ret = nxmutex_lock(&dev->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = usrsock_response(buffer, len, &req_done);
  if (req_done && dev->req.iov)
    {
      dev->req.iov = NULL;
      dev->req.pos = 0;
      dev->req.iovcnt = 0;
    }

  nxmutex_unlock(&dev->devlock);
  return ret;
}

/****************************************************************************
 * Name: usrsockdev_open
 ****************************************************************************/

static int usrsockdev_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct usrsockdev_s *dev;
  int ret;
  int tmp;

  DEBUGASSERT(inode);

  dev = inode->i_private;

  DEBUGASSERT(dev);

  ret = nxmutex_lock(&dev->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ninfo("opening /dev/usrsock\n");

  /* Increment the count of references to the device. */

  tmp = dev->ocount + 1;
  if (tmp > 1)
    {
      /* Only one reference is allowed. */

      nwarn("failed to open\n");

      ret = -EPERM;
    }
  else
    {
      dev->ocount = tmp;
      ret = OK;
    }

  nxmutex_unlock(&dev->devlock);
  return ret;
}

/****************************************************************************
 * Name: usrsockdev_close
 ****************************************************************************/

static int usrsockdev_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usrsockdev_s *dev;
  int ret;

  DEBUGASSERT(inode);

  dev = inode->i_private;

  DEBUGASSERT(dev);

  ret = nxmutex_lock(&dev->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ninfo("closing /dev/usrsock\n");

  /* Decrement the references to the driver. */

  dev->ocount--;
  DEBUGASSERT(dev->ocount == 0);
  ret = OK;
  dev->req.iov = NULL;
  dev->req.iovcnt = 0;
  dev->req.pos = 0;

  nxmutex_unlock(&dev->devlock);
  usrsock_abort();

  return ret;
}

/****************************************************************************
 * Name: usrsockdev_poll
 ****************************************************************************/

static int usrsockdev_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usrsockdev_s *dev;
  pollevent_t eventset;
  int ret;
  int i;

  DEBUGASSERT(inode);

  dev = inode->i_private;

  DEBUGASSERT(dev);

  /* Some sanity checking */

  if (!dev || !fds)
    {
      return -ENODEV;
    }

  /* Are we setting up the poll?  Or tearing it down? */

  ret = nxmutex_lock(&dev->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < ARRAY_SIZE(dev->pollfds); i++)
        {
          /* Find an available slot */

          if (!dev->pollfds[i])
            {
              /* Bind the poll structure and this slot */

              dev->pollfds[i] = fds;
              fds->priv = &dev->pollfds[i];
              break;
            }
        }

      if (i >= ARRAY_SIZE(dev->pollfds))
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto errout;
        }

      /* Should immediately notify on any of the requested events? */

      eventset = 0;

      /* Notify the POLLIN event if pending request. */

      if (dev->req.iov != NULL &&
          !(usrsock_iovec_get(NULL, 0, dev->req.iov,
                              dev->req.iovcnt, dev->req.pos, NULL) < 0))
        {
          eventset |= POLLIN;
        }

      poll_notify(dev->pollfds, ARRAY_SIZE(dev->pollfds), eventset);
    }
  else
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      if (!slot)
        {
          ret = -EIO;
          goto errout;
        }

      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }

errout:
  nxmutex_unlock(&dev->devlock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_request
 ****************************************************************************/

int usrsock_request(FAR struct iovec *iov, unsigned int iovcnt)
{
  FAR struct usrsockdev_s *dev = &g_usrsockdev;
  int ret = 0;

  /* Set outstanding request for daemon to handle. */

  net_sem_wait_uninterruptible(&dev->devlock);

  if (usrsockdev_is_opened(dev))
    {
      DEBUGASSERT(dev->req.iov == NULL);
      dev->req.iov = iov;
      dev->req.pos = 0;
      dev->req.iovcnt = iovcnt;

      /* Notify daemon of new request. */

      poll_notify(dev->pollfds, ARRAY_SIZE(dev->pollfds), POLLIN);
    }
  else
    {
      ninfo("daemon abruptly closed /dev/usrsock.\n");
      ret = -ENETDOWN;
    }

  nxmutex_unlock(&dev->devlock);
  return ret;
}

/****************************************************************************
 * Name: usrsock_register
 *
 * Description:
 *   Register /dev/usrsock
 *
 ****************************************************************************/

void usrsock_register(void)
{
  register_driver("/dev/usrsock", &g_usrsockdevops, 0666,
                  &g_usrsockdev);
}

#endif /* CONFIG_NET_USRSOCK_DEVICE */
