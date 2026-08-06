/****************************************************************************
 * drivers/misc/rpmsgdev.c
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

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <poll.h>
#include <limits.h>
#include <nuttx/debug.h>
#include <net/if.h>

#include <nuttx/list.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/video/fb.h>
#include <nuttx/mutex.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/net/ioctl.h>
#include <nuttx/drivers/rpmsgdev.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/input/mouse.h>
#include <nuttx/mtd/configdata.h>

#include "rpmsgdev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef rpmsgdeverr
#  define rpmsgdeverr(fmt, ...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The private data slots are preallocated in the device instance and the
 * unused ones are kept on the 'free' list.  A slot is claimed by removing it
 * from that list under the lock, which is atomic with respect to any other
 * open(), so no in-use flag is needed.  The opened slots are reachable from
 * filep->f_priv and therefore do not need to be linked anywhere.
 */

struct rpmsgdev_priv_s
{
  uint64_t         filep;    /* store server file pointer */
  bool             nonblock; /* true: open with O_NONBLOCK */
  struct list_node node;     /* Free list node, unlinked while in use */
  mutex_t          lock;     /* Mutex used to protect the file operation */
};

struct rpmsgdev_s
{
  struct rpmsg_endpoint  ept;         /* Rpmsg endpoint */

  /* The server cpu name and the device path in the server cpu */

  char                   remotecpu[RPMSG_NAME_SIZE];
  char                   remotepath[RPMSG_NAME_SIZE -
                                    RPMSGDEV_NAME_PREFIX_LEN];

  sem_t                  wait;        /* Wait sem, used for preventing any
                                       * operation until the connection
                                       * between two cpu established.
                                       */
  struct list_node       free;        /* List head of the free slots */
  struct list_node       cookies;     /* Requests waiting for a reply */
  spinlock_t             lock;        /* Protect the free and cookie lists */
  struct rpmsgdev_priv_s priv[CONFIG_DEV_RPMSG_OPEN_MAX];
};

/* Rpmsg device cookie used to handle the response from the remote cpu */

struct rpmsgdev_cookie_s
{
  struct list_node node;    /* Node in rpmsgdev_s::cookies */
  sem_t            sem;     /* Semaphore used for rpmsg */
  int              result;  /* The return value of the remote call */
  FAR void        *data;    /* The return data buffer of the remote call */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* The file operation functions */

static int     rpmsgdev_open(FAR struct file *filep);
static int     rpmsgdev_close(FAR struct file *filep);
static void    rpmsgdev_wait_cb(FAR struct pollfd_s *fds);
static int     rpmsgdev_wait(FAR struct file *filep, pollevent_t events);
static ssize_t rpmsgdev_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t rpmsgdev_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);
static off_t   rpmsgdev_seek(FAR struct file *filep, off_t offset,
                             int whence);
static ssize_t rpmsgdev_ioctl_arglen(int cmd, unsigned long arg);
static int     rpmsgdev_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg);
static int rpmsgdev_poll(FAR struct file *filep, FAR struct pollfd_s *fds,
                         bool setup);

/* Functions for sending data to the remote cpu */

static int     rpmsgdev_send_recv(FAR struct rpmsgdev_s *priv,
                                  uint32_t command, bool copy,
                                  FAR struct rpmsgdev_header_s *msg,
                                  int len, FAR void *data);
static FAR void *rpmsgdev_get_tx_payload_buffer(FAR struct rpmsgdev_s *priv,
                                                FAR uint32_t *len);

/* Functions handle the responses from the remote cpu */

static int     rpmsgdev_default_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv);
static int     rpmsgdev_read_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv);
static int     rpmsgdev_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv);
static int     rpmsgdev_notify_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv);

/* Functions for creating communication with remote cpu */

static void    rpmsgdev_device_created(struct rpmsg_device *rdev,
                                       FAR void *priv_);
static void    rpmsgdev_device_destroy(struct rpmsg_device *rdev,
                                       FAR void *priv_);
static int     rpmsgdev_ept_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len, uint32_t src,
                               FAR void *priv);
static void    rpmsgdev_ns_bound(struct rpmsg_endpoint *ept);
static void    rpmsgdev_ns_unbind(FAR struct rpmsg_endpoint *ept);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Rpmsg device response handler table */

static const rpmsg_ept_cb g_rpmsgdev_handler[] =
{
  [RPMSGDEV_OPEN]          = rpmsgdev_default_handler,
  [RPMSGDEV_CLOSE]         = rpmsgdev_default_handler,
  [RPMSGDEV_READ_NONBLOCK] = rpmsgdev_read_handler,
  [RPMSGDEV_READ]          = rpmsgdev_read_handler,
  [RPMSGDEV_WRITE]         = rpmsgdev_default_handler,
  [RPMSGDEV_LSEEK]         = rpmsgdev_default_handler,
  [RPMSGDEV_IOCTL]         = rpmsgdev_ioctl_handler,
  [RPMSGDEV_POLL]          = rpmsgdev_default_handler,
  [RPMSGDEV_NOTIFY]        = rpmsgdev_notify_handler,
};

/* File operations */

static const struct file_operations g_rpmsgdev_ops =
{
  rpmsgdev_open,          /* open */
  rpmsgdev_close,         /* close */
  rpmsgdev_read,          /* read */
  rpmsgdev_write,         /* write */
  rpmsgdev_seek,          /* seek */
  rpmsgdev_ioctl,         /* ioctl */
  NULL,                   /* mmap */
  NULL,                   /* truncate */
  rpmsgdev_poll           /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsgdev_open
 *
 * Description:
 *   Rpmsg-device open operation
 *
 * Parameters:
 *   filep  - the file instance
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgdev_open(FAR struct file *filep)
{
  FAR struct rpmsgdev_s *dev;
  FAR struct rpmsgdev_priv_s *priv;
  struct rpmsgdev_open_s msg;
  irqstate_t flags;
  int ret;

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  dev = filep->f_inode->i_private;
  DEBUGASSERT(dev != NULL);

  /* Claim a free private data slot by removing it from the free list */

  flags = spin_lock_irqsave(&dev->lock);
  priv = list_remove_head_type(&dev->free, struct rpmsgdev_priv_s, node);
  spin_unlock_irqrestore(&dev->lock, flags);

  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* Try to open the device in the remote cpu, open with O_NONBLOCK
   * by default to avoid the server rptun thread blocked in read/write
   * operations.
   */

  msg.flags = filep->f_oflags | O_NONBLOCK;
  ret = rpmsgdev_send_recv(dev, RPMSGDEV_OPEN, true, &msg.header,
                           sizeof(msg), NULL);
  if (ret < 0)
    {
      rpmsgdeverr("open failed, ret=%d\n", ret);
      flags = spin_lock_irqsave(&dev->lock);
      list_add_tail(&dev->free, &priv->node);
      spin_unlock_irqrestore(&dev->lock, flags);
      return ret;
    }

  priv->filep    = msg.filep;
  priv->nonblock = !!(filep->f_oflags & O_NONBLOCK);
  nxmutex_init(&priv->lock);

  /* Attach the private data to the struct file instance */

  filep->f_priv = priv;
  return ret;
}

/****************************************************************************
 * Name: rpmsgdev_close
 *
 * Description:
 *   Rpmsg-device close operation
 *
 * Parameters:
 *   filep  - the file instance
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgdev_close(FAR struct file *filep)
{
  FAR struct rpmsgdev_s *dev;
  FAR struct rpmsgdev_priv_s *priv;
  struct rpmsgdev_close_s msg;
  irqstate_t flags;
  int ret;

  /* Recover our private data from the struct file instance */

  dev  = filep->f_inode->i_private;
  priv = filep->f_priv;
  DEBUGASSERT(dev != NULL && priv != NULL);

  /* Try to close the device in the remote cpu */

  msg.filep = priv->filep;
  ret = rpmsgdev_send_recv(dev, RPMSGDEV_CLOSE, true, &msg.header,
                           sizeof(msg), NULL);
  if (ret < 0)
    {
      rpmsgdeverr("close failed, ret=%d\n", ret);
      return ret;
    }

  filep->f_priv = NULL;
  nxmutex_destroy(&priv->lock);
  flags = spin_lock_irqsave(&dev->lock);
  list_add_tail(&dev->free, &priv->node);
  spin_unlock_irqrestore(&dev->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: rpmsgdev_wait_cb
 *
 * Description:
 *   Rpmsg-device read/write operation wait callback function
 *
 * Parameters:
 *   fds  - The structure describing the events to be monitored.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsgdev_wait_cb(FAR struct pollfd_s *fds)
{
  int semcount = 0;
  FAR sem_t *pollsem = (FAR sem_t *)fds->arg;

  nxsem_get_value(pollsem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(pollsem);
    }
}

/****************************************************************************
 * Name: rpmsgdev_wait
 *
 * Description:
 *   Rpmsg-device read/write operation wait function, this function will be
 *   called in the rpmsgdev_read()/rpmsgdev_write() when the open flags is
 *   not NONBLOCKED to avoid the server rptun thread blocked in file_read()
 *   or file_write(). By calling this function before sending the READ or
 *   WRITE command to server, a simulated blocked read/write operation is
 *   achieved.
 *
 * Parameters:
 *   filep  - the file instance
 *   events - the events to be monitored
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgdev_wait(FAR struct file *filep, pollevent_t events)
{
  int ret;
  sem_t sem;
  struct pollfd_s fds;

  nxsem_init(&sem, 0, 0);

  fds.events  = events;
  fds.arg     = &sem;
  fds.cb      = rpmsgdev_wait_cb;
  events     |= POLLERR | POLLHUP;

  for (; ; )
    {
      ret = rpmsgdev_poll(filep, &fds, true);
      if (ret < 0)
        {
          break;
        }

      ret = nxsem_wait(&sem);
      rpmsgdev_poll(filep, &fds, false);
      if (ret < 0)
        {
          break;
        }

      if ((fds.revents & events) != 0)
        {
          break;
        }
    }

  nxsem_destroy(&sem);
  return ret;
}

/****************************************************************************
 * Name: rpmsgdev_read
 *
 * Description:
 *   Rpmsg-device read operation
 *
 * Parameters:
 *   filep  - the file instance
 *   buffer - the read buffer pointer
 *   buflen - the read buffer length
 *
 * Returned Values:
 *   The positive non-zero number of bytes read on success, 0 on if an
 *   end-of-file condition, or a negated errno value on any failure.
 *
 ****************************************************************************/

static ssize_t rpmsgdev_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct rpmsgdev_s *dev;
  FAR struct rpmsgdev_priv_s *priv;
  FAR struct rpmsgdev_read_s *msg;
  uint32_t command;
  uint32_t space;
  ssize_t ret;

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  /* Recover our private data from the struct file instance */

  dev  = filep->f_inode->i_private;
  priv = filep->f_priv;
  DEBUGASSERT(dev != NULL && priv != NULL);

  command = priv->nonblock ? RPMSGDEV_READ_NONBLOCK : RPMSGDEV_READ;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  msg = rpmsgdev_get_tx_payload_buffer(dev, &space);
  if (msg == NULL)
    {
      nxmutex_unlock(&priv->lock);
      return -ENOMEM;
    }

  msg->filep = priv->filep;
  msg->count = buflen;

  ret = rpmsgdev_send_recv(dev, command, false, &msg->header,
                           sizeof(*msg) - 1, buffer);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgdev_write
 *
 * Description:
 *   Rpmsg-device write operation
 *
 * Parameters:
 *   filep  - the file instance
 *   buffer - the write buffer pointer
 *   buflen - the write buffer length
 *
 * Returned Values:
 *   On success, the number of bytes written are returned (zero indicates
 *   nothing was written).  On any failure, a negated errno value is returned
 *   (see comments withwrite() for a description of the appropriate errno
 *   values).
 *
 ****************************************************************************/

static ssize_t rpmsgdev_write(FAR struct file *filep, const char *buffer,
                              size_t buflen)
{
  FAR struct rpmsgdev_s *dev;
  FAR struct rpmsgdev_priv_s *priv;
  FAR struct rpmsgdev_write_s *msg;
  uint32_t space;
  size_t written = 0;
  int ret = 0;

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  /* Recover our private data from the struct file instance */

  dev  = filep->f_inode->i_private;
  priv = filep->f_priv;
  DEBUGASSERT(dev != NULL && priv != NULL);

  /* Perform the rpmsg write */

  while (written < buflen)
    {
      msg = rpmsgdev_get_tx_payload_buffer(dev, &space);
      if (msg == NULL)
        {
          ret = -ENOMEM;
          break;
        }

      space -= sizeof(*msg) - 1;
      if (space >= buflen - written)
        {
          space = buflen - written;
        }

      msg->filep = priv->filep;
      msg->count = space;
      memcpy(msg->buf, buffer + written, space);
      ret = rpmsgdev_send_recv(dev, RPMSGDEV_WRITE, false, &msg->header,
                               sizeof(*msg) - 1 + space, NULL);
      if (ret > 0)
        {
          written += ret;
          continue;
        }
      else if (ret != -EAGAIN || priv->nonblock || written != 0)
        {
          break;
        }

      /* If open with block mode and return -EAGAIN and no data
       * written to this device, should wait peer device ready and
       * try again.
       */

      ret = rpmsgdev_wait(filep, POLLOUT);
      if (ret < 0)
        {
          rpmsgerr("write wait failed, ret=%d\n", ret);
          break;
        }
    }

  return written != 0 ? written : ret;
}

/****************************************************************************
 * Name: rpmsgdev_seek
 *
 * Description:
 *   Rpmsg-device seek operation
 *
 * Parameters:
 *   file     File structure instance
 *   offset   Defines the offset to position to
 *   whence   Defines how to use offset
 *
 * Returned Values:
 *   The resulting offset on success.  A negated errno value is returned on
 *   any failure (see lseek comments).
 *
 ****************************************************************************/

static off_t rpmsgdev_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct rpmsgdev_s *dev;
  FAR struct rpmsgdev_priv_s *priv;
  struct rpmsgdev_lseek_s msg;
  int ret;

  /* Recover our private data from the struct file instance */

  dev  = filep->f_inode->i_private;
  priv = filep->f_priv;
  DEBUGASSERT(dev != NULL && priv != NULL);

  /* Call our internal routine to perform the seek */

  msg.filep  = priv->filep;
  msg.offset = offset;
  msg.whence = whence;

  ret = rpmsgdev_send_recv(dev, RPMSGDEV_LSEEK, true, &msg.header,
                           sizeof(msg), NULL);
  if (ret >= 0)
    {
      filep->f_pos = msg.offset;
    }

  return ret;
}

/****************************************************************************
 * Name: rpmsgdev_marshal_cfgdata
 ****************************************************************************/

static void rpmsgdev_marshal_cfgdata(FAR void *buf, FAR void *cfg)
{
  FAR struct config_data_s *src = (FAR struct config_data_s *)cfg;
  FAR struct config_data_s *dst = (FAR struct config_data_s *)buf;

#ifdef CONFIG_MTD_CONFIG_NAMED
  memcpy(dst->name, src->name, sizeof(dst->name));
#else
  dst->id = src->id;
  dst->instance = src->instance;
#endif
  dst->len = src->len;
  dst->configdata = (FAR uint8_t *)(dst + 1);
  if (src->configdata != NULL && src->len > 0)
    {
      memcpy(dst->configdata, src->configdata, src->len);
    }
}

/****************************************************************************
 * Name: rpmsgdev_unmarshal_cfgdata
 ****************************************************************************/

static void rpmsgdev_unmarshal_cfgdata(FAR void *buf, FAR void *cfg)
{
  FAR struct config_data_s *src = (FAR struct config_data_s *)buf;
  FAR struct config_data_s *dst = (FAR struct config_data_s *)cfg;

  if (dst->configdata != NULL && src->len > 0)
    {
      memcpy(dst->configdata, (FAR uint8_t *)(src + 1),
             MIN(src->len, dst->len));
    }

#ifdef CONFIG_MTD_CONFIG_NAMED
  memcpy(dst->name, src->name, sizeof(dst->name));
#else
  dst->id = src->id;
  dst->instance = src->instance;
#endif
  dst->len = src->len;
}

/****************************************************************************
 * Name: rpmsgdev_ioctl_arglen
 *
 * Description:
 *   Get rpmsg device ioctl argument length according to the command
 *
 * Parameters:
 *   cmd - the ioctl command
 *   arg - the ioctl arguments
 *
 * Returned Values:
 *   0        - ioctl command not support
 *   positive - the argument length
 *
 ****************************************************************************/

static ssize_t rpmsgdev_ioctl_arglen(int cmd, unsigned long arg)
{
  switch (cmd)
    {
      case FIONWRITE:
      case FIONREAD:
      case FIONSPACE:
      case FBIOSET_POWER:
      case FBIOGET_POWER:
      case BATIOC_CURRENT:
      case BATIOC_VOLTAGE:
      case BATIOC_TEMPERATURE:
      case BATIOC_INPUT_CURRENT:
      case BATIOC_STATE:
      case BATIOC_ONLINE:
      case BATIOC_CAPACITY:
      case BATIOC_CHIPID:
        return sizeof(int);
      case TUNSETIFF:
      case TUNGETIFF:
        return sizeof(struct ifreq);
      case FIOC_FILEPATH:
        return PATH_MAX;
      case BATIOC_GET_PROTOCOL:
      case BATIOC_OPERATE:
        return sizeof(struct batio_operate_msg_s);
      case MSIOC_VENDOR:
        return sizeof(struct mouse_vendor_cmd_s) +
               ((FAR struct mouse_vendor_cmd_s *)(uintptr_t)arg)->len;
      case MSIOC_GRAB:
        return 0;
      case CFGDIOC_GETCONFIG:
      case CFGDIOC_SETCONFIG:
      case CFGDIOC_FIRSTCONFIG:
      case CFGDIOC_NEXTCONFIG:
        return sizeof(struct config_data_s) +
               ((FAR struct config_data_s *)(uintptr_t)arg)->len;
      case CFGDIOC_DELCONFIG:
        return sizeof(struct config_data_s);
      case RPMSGDEV_PRIVATE_IOCTL:
        return sizeof(struct rpmsgdev_private_ioctl) +
               ((FAR struct rpmsgdev_private_ioctl *)(uintptr_t)arg)->len;
      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: rpmsgdev_ioctl
 *
 * Description:
 *   Rpmsg-device ioctl operation
 *
 * Parameters:
 *   filep  - the file instance
 *   cmd    - the ioctl command
 *   arg    - the ioctl arguments
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgdev_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct rpmsgdev_s *dev;
  FAR struct rpmsgdev_priv_s *priv;
  FAR struct rpmsgdev_ioctl_s *msg;
  FAR void *argp = (FAR void *)(uintptr_t)arg;
  uint32_t space;
  ssize_t arglen;
  size_t msglen;

  /* Recover our private data from the struct file instance */

  dev  = filep->f_inode->i_private;
  priv = filep->f_priv;
  DEBUGASSERT(dev != NULL && priv != NULL);

  /* Always handle FIONBIO locally since the remote side
   * must always be in non-blocking mode to avoid blocking
   * the rpmsg work thread.
   */

  if (cmd == FIONBIO)
    {
      FAR int *nonblock = (FAR int *)argp;

      priv->nonblock = *nonblock;
      return 0;
    }

  /* Call our internal routine to perform the ioctl */

  arglen = rpmsgdev_ioctl_arglen(cmd, arg);
  if (arglen < 0)
    {
      return arglen;
    }

  msg = rpmsgdev_get_tx_payload_buffer(dev, &space);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  msglen = sizeof(*msg) + arglen - 1;
  if (space < msglen)
    {
      rpmsg_release_tx_buffer(&dev->ept, msg);
      return -EMSGSIZE;
    }

  msg->filep   = priv->filep;
  msg->request = cmd;
  msg->arg     = arg;
  msg->arglen  = arglen;

  if (arglen > 0)
    {
      if (cmd >= CFGDIOC_GETCONFIG && cmd <= CFGDIOC_NEXTCONFIG)
        {
          rpmsgdev_marshal_cfgdata(msg->buf, argp);
        }
      else
        {
          memcpy(msg->buf, argp, arglen);
        }
    }

  return rpmsgdev_send_recv(dev, RPMSGDEV_IOCTL, false, &msg->header,
                            msglen, arglen > 0 ? argp : NULL);
}

/****************************************************************************
 * Name: rpmsgdev_poll
 *
 * Description:
 *   Rpmsg-device poll operation
 *
 * Parameters:
 *   filep - the file instance
 *   fds   - The structure describing the events to be monitored.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgdev_poll(FAR struct file *filep, FAR struct pollfd_s *fds,
                         bool setup)
{
  FAR struct rpmsgdev_s *dev;
  FAR struct rpmsgdev_priv_s *priv;
  struct rpmsgdev_poll_s msg;
  int ret;

  /* Recover our private data from the struct file instance */

  dev  = filep->f_inode->i_private;
  priv = filep->f_priv;
  DEBUGASSERT(dev != NULL && priv != NULL);

  /* Setup or teardown the poll */

  msg.filep  = priv->filep;
  msg.events = fds->events;
  msg.setup  = setup;
  msg.fds    = (uint64_t)(uintptr_t)fds;

  ret = rpmsgdev_send_recv(dev, RPMSGDEV_POLL, true, &msg.header,
                           sizeof(msg), NULL);
  return ret;
}

/****************************************************************************
 * Name: rpmsgdev_get_tx_payload_buffer
 *
 * Description:
 *   Get the rpmsg device tx payload, the buffer is from the rpmsg
 *   share memory that can be accessed by local and remote cpu.
 *
 * Parameters:
 *   priv  - The rpmsg-device handle
 *   len   - The got memory size
 *
 * Returned Values:
 *   NULL     - failure
 *   not NULL - success
 *
 ****************************************************************************/

static FAR void *rpmsgdev_get_tx_payload_buffer(FAR struct rpmsgdev_s *priv,
                                                FAR uint32_t *len)
{
  int sval = 0;

  nxsem_get_value(&priv->wait, &sval);
  if (sval <= 0)
    {
      rpmsg_wait(&priv->ept, &priv->wait);
      rpmsg_post(&priv->ept, &priv->wait);
    }

  return rpmsg_get_tx_payload_buffer(&priv->ept, len, true);
}

/****************************************************************************
 * Name: rpmsgdev_send_recv
 *
 * Description:
 *   Send and receive the rpmsg data.
 *
 * Parameters:
 *   priv    - rpmsg device handle
 *   command - the command, RPMSGDEV_OPEN, RPMSGDEV_CLOSE, RPMSGDEV_READ,
 *                          RPMSGDEV_WRITE, RPMSGDEV_IOCTL
 *   copy    - true, send a message across to the remote processor, and the
 *                   tx buffer will be alloced inside function rpmsg_send()
 *             false, send a message in tx buffer reserved by
 *                    rpmsg_get_tx_payload_buffer() across to the remote
 *                    processor.
 *   msg     - the message header
 *   len     - length of the payload
 *   data    - the data
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgdev_send_recv(FAR struct rpmsgdev_s *priv,
                              uint32_t command, bool copy,
                              FAR struct rpmsgdev_header_s *msg,
                              int len, FAR void *data)
{
  struct rpmsgdev_cookie_s cookie;
  irqstate_t flags;
  int ret;

  memset(&cookie, 0, sizeof(cookie));
  nxsem_init(&cookie.sem, 0, 0);

  if (data != NULL)
    {
      cookie.data = data;
    }
  else if (copy)
    {
      cookie.data = msg;
    }

  msg->command = command;
  msg->result  = -ENXIO;
  msg->cookie  = (uintptr_t)&cookie;

  flags = spin_lock_irqsave(&priv->lock);
  list_add_tail(&priv->cookies, &cookie.node);
  spin_unlock_irqrestore(&priv->lock, flags);

  if (copy)
    {
      uint32_t buflen;
      FAR void *buf;

      buf = rpmsgdev_get_tx_payload_buffer(priv, &buflen);
      if (buf == NULL)
        {
          ret = -ENOMEM;
          goto fail;
        }

      DEBUGASSERT(len <= buflen);
      memcpy(buf, msg, len);
      msg = buf;
    }

  ret = rpmsg_send_nocopy(&priv->ept, msg, len);
  if (ret < 0)
    {
      rpmsg_release_tx_buffer(&priv->ept, msg);
      goto fail;
    }

  ret = rpmsg_wait(&priv->ept, &cookie.sem);
  if (ret >= 0)
    {
      ret = cookie.result;
    }

fail:
  flags = spin_lock_irqsave(&priv->lock);
  list_delete(&cookie.node);
  spin_unlock_irqrestore(&priv->lock, flags);
  nxsem_destroy(&cookie.sem);
  return ret;
}

/****************************************************************************
 * Name: rpmsgdev_default_handler
 *
 * Description:
 *   Default rpmsg-device response handler, this function will be called to
 *   process the return message of rpmsgdev_open(), rpmsgdev_close() and
 *   rpmsgdev_write().
 *
 * Parameters:
 *   ept  - The rpmsg endpoint
 *   data - The return message
 *   len  - The return message length
 *   src  - unknown
 *   priv - unknown
 *
 * Returned Values:
 *   Always OK
 *
 ****************************************************************************/

static int rpmsgdev_default_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv)
{
  FAR struct rpmsgdev_header_s *header = data;
  FAR struct rpmsgdev_cookie_s *cookie =
      (FAR struct rpmsgdev_cookie_s *)(uintptr_t)header->cookie;

  cookie->result = header->result;
  if (cookie->result >= 0 && cookie->data)
    {
      memcpy(cookie->data, data, len);
    }

  rpmsg_post(ept, &cookie->sem);
  return 0;
}

/****************************************************************************
 * Name: rpmsgdev_read_handler
 *
 * Description:
 *   Rpmsg-device read response handler, this function will be called to
 *   process the return message of rpmsgdev_read().
 *
 * Parameters:
 *   ept  - The rpmsg endpoint
 *   data - The return message
 *   len  - The return message length
 *   src  - unknown
 *   priv - unknown
 *
 * Returned Values:
 *   Always OK
 *
 ****************************************************************************/

static int rpmsgdev_read_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgdev_header_s *header = data;
  FAR struct rpmsgdev_cookie_s *cookie =
      (FAR struct rpmsgdev_cookie_s *)(uintptr_t)header->cookie;
  FAR struct rpmsgdev_read_s *rsp = data;

  if (header->result > 0)
    {
      memcpy(cookie->data, rsp->buf, header->result);
      cookie->data += header->result;
      cookie->result += header->result;
    }
  else if (cookie->result == 0)
    {
      cookie->result = header->result;
    }

  if (rsp->count == 0 || header->result <= 0)
    {
      rpmsg_post(ept, &cookie->sem);
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsgdev_ioctl_handler
 *
 * Description:
 *   Rpmsg-device ioctl response handler, this function will be called to
 *   process the return message of rpmsgdev_ioctl().
 *
 * Parameters:
 *   ept  - The rpmsg endpoint
 *   data - The return message
 *   len  - The return message length
 *   src  - unknown
 *   priv - unknown
 *
 * Returned Values:
 *   Always OK
 *
 ****************************************************************************/

static int rpmsgdev_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgdev_header_s *header = data;
  FAR struct rpmsgdev_cookie_s *cookie =
      (FAR struct rpmsgdev_cookie_s *)(uintptr_t)header->cookie;
  FAR struct rpmsgdev_ioctl_s *rsp = data;

  cookie->result = header->result;
  if (cookie->result >= 0 && rsp->arglen > 0)
    {
      if (rsp->request >= CFGDIOC_GETCONFIG &&
          rsp->request <= CFGDIOC_NEXTCONFIG)
        {
          rpmsgdev_unmarshal_cfgdata(rsp->buf, cookie->data);
        }
      else
        {
          memcpy(cookie->data, rsp->buf, rsp->arglen);
        }
    }

  rpmsg_post(ept, &cookie->sem);
  return 0;
}

/****************************************************************************
 * Name: rpmsgdev_notify_handler
 *
 * Description:
 *   Rpmsg-device poll notify handler.
 *
 * Parameters:
 *   ept  - The rpmsg endpoint
 *   data - The return message
 *   len  - The return message length
 *   src  - unknown
 *   priv - unknown
 *
 * Returned Values:
 *   Always OK
 *
 ****************************************************************************/

static int rpmsgdev_notify_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv)
{
  FAR struct rpmsgdev_notify_s *rsp = data;
  FAR struct pollfd_s *fds;

  fds = (FAR struct pollfd_s *)(uintptr_t)rsp->fds;
  poll_notify(&fds, 1, rsp->revents);

  return 0;
}

/****************************************************************************
 * Name: rpmsgdev_ns_bound
 *
 * Description:
 *   Rpmsg device end point service bound callback function , called when
 *   remote end point address is received.
 *
 * Parameters:
 *   ept  - The rpmsg-device end point
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsgdev_ns_bound(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsgdev_s *priv = ept->priv;

  rpmsg_post(&priv->ept, &priv->wait);
}

/****************************************************************************
 * Name: rpmsgdev_fail_cookies
 *
 * Description:
 *   Complete every request still waiting for a reply with -ENOTCONN.  The
 *   server is gone and no reply will come, so without this a blocking
 *   read would never return.
 *
 ****************************************************************************/

static void rpmsgdev_fail_cookies(FAR struct rpmsgdev_s *priv)
{
  FAR struct rpmsgdev_cookie_s *cookie;
  irqstate_t flags;

  /* rpmsg_post() may switch to a waiter, which then takes priv->lock */

  flags = spin_lock_irqsave_nopreempt(&priv->lock);
  list_for_every_entry(&priv->cookies, cookie, struct rpmsgdev_cookie_s,
                       node)
    {
      cookie->result = -ENOTCONN;
      rpmsg_post(&priv->ept, &cookie->sem);
    }

  spin_unlock_irqrestore_nopreempt(&priv->lock, flags);
}

/****************************************************************************
 * Name: rpmsgdev_ns_unbind
 ****************************************************************************/

static void rpmsgdev_ns_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsgdev_s *priv = ept->priv;

  rpmsgdev_fail_cookies(priv);
}

/****************************************************************************
 * Name: rpmsgdev_device_created
 *
 * Description:
 *   Rpmsg device create function, this function will be called by rptun to
 *   create a rpmsg-device end point.
 *
 * Parameters:
 *   rdev  - The rpmsg-device end point
 *   priv_ - Rpmsg-device handle
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsgdev_device_created(FAR struct rpmsg_device *rdev,
                                    FAR void *priv_)
{
  FAR struct rpmsgdev_s *priv = priv_;
  char buf[RPMSG_NAME_SIZE];

  if (strcmp(priv->remotecpu, rpmsg_get_cpuname(rdev)) == 0)
    {
      priv->ept.priv = priv;
      priv->ept.ns_bound_cb = rpmsgdev_ns_bound;
      snprintf(buf, sizeof(buf), "%s%s", RPMSGDEV_NAME_PREFIX,
               priv->remotepath);
      rpmsg_set_priority(&priv->ept, RPMSG_PRIO_RT);
      rpmsg_create_ept(&priv->ept, rdev, buf,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       rpmsgdev_ept_cb, rpmsgdev_ns_unbind);
    }
}

/****************************************************************************
 * Name: rpmsgdev_device_destroy
 *
 * Description:
 *   Rpmsg device destroy function, this function will be called by rptun to
 *   destroy rpmsg-device end point.
 *
 * Parameters:
 *   rdev  - The rpmsg-device end point
 *   priv_ - Rpmsg-device handle
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsgdev_device_destroy(FAR struct rpmsg_device *rdev,
                                    FAR void *priv_)
{
  FAR struct rpmsgdev_s *priv = priv_;

  if (strcmp(priv->remotecpu, rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsgdev_fail_cookies(priv);
      rpmsg_wait(&priv->ept, &priv->wait);
      rpmsg_destroy_ept(&priv->ept);
    }
}

/****************************************************************************
 * Name: rpmsgdev_ept_cb
 *
 * Description:
 *   Rpmsg device end point callback function, this function will be called
 *   when receive the remote cpu message.
 *
 * Parameters:
 *   ept  - The rpmsg-device end point
 *   data - The received data
 *   len  - The received data length
 *   src  - unknown
 *   priv - unknown
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgdev_ept_cb(FAR struct rpmsg_endpoint *ept,
                           FAR void *data, size_t len, uint32_t src,
                           FAR void *priv)
{
  FAR struct rpmsgdev_header_s *header = data;
  uint32_t command = header->command;

  if (command < nitems(g_rpmsgdev_handler))
    {
      return g_rpmsgdev_handler[command](ept, data, len, src, priv);
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsgdev_register
 *
 * Description:
 *   Rpmsg-device client initialize function, the client cpu should call
 *   this function in the board initialize process.
 *
 * Parameters:
 *   remotecpu  - the server cpu name
 *   remotepath - the device you want to access in the remote cpu
 *   localpath  - the device path in local cpu, if NULL, the localpath is
 *                same as the remotepath, provide this argument to support
 *                custom device path
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmsgdev_register(FAR const char *remotecpu, FAR const char *remotepath,
                      FAR const char *localpath)
{
  FAR struct rpmsgdev_s *dev;
  int ret;
  int i;

  /* Arguments check */

  if (remotecpu == NULL || remotepath == NULL)
    {
      return -EINVAL;
    }

  DEBUGASSERT(strlen(remotepath) + RPMSGDEV_NAME_PREFIX_LEN <
              RPMSG_NAME_SIZE);

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize the rpmsg device */

  strlcpy(dev->remotecpu, remotecpu, sizeof(dev->remotecpu));
  strlcpy(dev->remotepath, remotepath, sizeof(dev->remotepath));

  nxsem_init(&dev->wait, 0, 0);
  spin_lock_init(&dev->lock);

  list_initialize(&dev->free);
  list_initialize(&dev->cookies);
  for (i = 0; i < CONFIG_DEV_RPMSG_OPEN_MAX; i++)
    {
      list_add_tail(&dev->free, &dev->priv[i].node);
    }

  /* Register the rpmsg callback */

  ret = rpmsg_register_callback(dev,
                                rpmsgdev_device_created,
                                rpmsgdev_device_destroy,
                                NULL,
                                NULL);
  if (ret < 0)
    {
      rpmsgdeverr("register callback failed, ret=%d\n", ret);
      goto fail;
    }

  /* Register driver, using the remotepath if localpath is NULL */

  if (localpath == NULL)
    {
      localpath = remotepath;
    }

  ret = register_driver(localpath, &g_rpmsgdev_ops, 0600, dev);
  if (ret < 0)
    {
      rpmsgdeverr("register driver failed, ret=%d\n", ret);
      goto fail_with_rpmsg;
    }

  return 0;

fail_with_rpmsg:
  rpmsg_unregister_callback(dev,
                            rpmsgdev_device_created,
                            rpmsgdev_device_destroy,
                            NULL,
                            NULL);

fail:
  nxsem_destroy(&dev->wait);
  kmm_free(dev);

  return ret;
}

/****************************************************************************
 * Name: rpmsgdev_unregister
 *
 * Description:
 *   Rpmsg-device client uninitialize function, the client cpu should call
 *   this function to unregister the rpmsg device.
 *
 * Parameters:
 *   localpath  - the device path which bindded to the rpmsg device
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmsgdev_unregister(FAR const char *localpath)
{
  FAR struct rpmsgdev_s *dev;
  struct file file;
  int ret;

  if (localpath == NULL)
    {
      return -EINVAL;
    }

  /* Open the device to get the inode reference */

  ret = file_open(&file, localpath, O_RDWR);
  if (ret < 0)
    {
      rpmsgdeverr("Failed to open %s, ret=%d\n", localpath, ret);
      return ret;
    }

  /* Get the private data from the inode */

  dev = file.f_inode->i_private;
  file_close(&file);

  /* Unregister the driver */

  ret = unregister_driver(localpath);
  if (ret < 0)
    {
      rpmsgdeverr("unregister driver failed, ret=%d\n", ret);
      return ret;
    }

  /* Unregister the rpmsg callback */

  rpmsg_unregister_callback(dev,
                            rpmsgdev_device_created,
                            rpmsgdev_device_destroy,
                            NULL,
                            NULL);

  nxsem_destroy(&dev->wait);
  kmm_free(dev);
  return ret;
}

