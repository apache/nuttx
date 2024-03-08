/****************************************************************************
 * drivers/misc/rpmsgdev.c
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
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <poll.h>
#include <limits.h>
#include <debug.h>
#include <net/if.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/video/fb.h>
#include <nuttx/mutex.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/net/ioctl.h>
#include <nuttx/drivers/rpmsgdev.h>
#include <nuttx/power/battery_ioctl.h>

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

struct rpmsgdev_priv_s
{
  uint64_t filep;    /* store server file pointer */
  bool     nonblock; /* true: open with O_NONBLOCK */
};

struct rpmsgdev_s
{
  struct rpmsg_endpoint ept;         /* Rpmsg endpoint */
  FAR const char       *remotecpu;   /* The server cpu name */
  FAR const char       *remotepath;  /* The device path in the server cpu */
  sem_t                 wait;        /* Wait sem, used for preventing any
                                      * opreation until the connection
                                      * between two cpu established.
                                      */
  uint32_t              flags;       /* Read and write special handle flags */
};

/* Rpmsg device cookie used to handle the response from the remote cpu */

struct rpmsgdev_cookie_s
{
  sem_t     sem;     /* Semaphore used fo rpmsg */
  int       result;  /* The return value of the remote call */
  FAR void *data;    /* The return data buffer of the remote call */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* The file operation functions */

static int     rpmsgdev_open(FAR struct file *filep);
static int     rpmsgdev_close(FAR struct file *filep);
static void    rpmsgdev_wait_cb(FAR struct pollfd *fds);
static int     rpmsgdev_wait(FAR struct file *filep, pollevent_t events);
static ssize_t rpmsgdev_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t rpmsgdev_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);
static off_t   rpmsgdev_seek(FAR struct file *filep, off_t offset,
                             int whence);
static ssize_t rpmsgdev_ioctl_arglen(int cmd);
static int     rpmsgdev_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg);
static int     rpmsgdev_poll(FAR struct file *filep, FAR struct pollfd *fds,
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

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Rpmsg device response handler table */

static const rpmsg_ept_cb g_rpmsgdev_handler[] =
{
  [RPMSGDEV_OPEN]        = rpmsgdev_default_handler,
  [RPMSGDEV_CLOSE]       = rpmsgdev_default_handler,
  [RPMSGDEV_READ]        = rpmsgdev_read_handler,
  [RPMSGDEV_READ_NOFRAG] = rpmsgdev_read_handler,
  [RPMSGDEV_WRITE]       = rpmsgdev_default_handler,
  [RPMSGDEV_LSEEK]       = rpmsgdev_default_handler,
  [RPMSGDEV_IOCTL]       = rpmsgdev_ioctl_handler,
  [RPMSGDEV_POLL]        = rpmsgdev_default_handler,
  [RPMSGDEV_NOTIFY]      = rpmsgdev_notify_handler,
};

/* File operations */

const struct file_operations g_rpmsgdev_ops =
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
  int ret;

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  dev = filep->f_inode->i_private;
  DEBUGASSERT(dev != NULL);

  priv = kmm_zalloc(sizeof(*priv));
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
      kmm_free(priv);
      return ret;
    }

  priv->filep    = msg.filep;
  priv->nonblock = (filep->f_oflags & O_NONBLOCK) != 0;

  /* Attach the private date to the struct file instance */

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
  kmm_free(priv);

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

static void rpmsgdev_wait_cb(FAR struct pollfd *fds)
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
 *   achived.
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
  struct pollfd fds;

  nxsem_init(&sem, 0, 0);

  fds.events  = events;
  fds.arg     = &sem;
  fds.cb      = rpmsgdev_wait_cb;
  events     |= POLLERR | POLLHUP;

  while (1)
    {
      ret = rpmsgdev_poll(filep, &fds, true);
      if (ret < 0)
        {
          return ret;
        }

      ret = nxsem_wait(&sem);
      rpmsgdev_poll(filep, &fds, false);
      if (ret < 0)
        {
          return ret;
        }

      if ((fds.revents & events) != 0)
        {
          break;
        }
    }

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
  struct rpmsgdev_read_s msg;
  struct iovec read;
  uint32_t command;
  ssize_t ret;

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  /* Recover our private data from the struct file instance */

  dev  = filep->f_inode->i_private;
  priv = filep->f_priv;
  DEBUGASSERT(dev != NULL && priv != NULL);

  /* If the open flags is not nonblock, should poll the device for
   * read ready first to avoid the server rptun thread blocked in
   * device read operation.
   */

  if (priv->nonblock == false)
    {
      ret = rpmsgdev_wait(filep, POLLIN);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Call the host to perform the read */

  read.iov_base = buffer;
  read.iov_len  = 0;

  msg.filep = priv->filep;
  msg.count = buflen;
  command   = dev->flags & RPMSGDEV_NOFRAG_READ ?
              RPMSGDEV_READ_NOFRAG : RPMSGDEV_READ;

  ret = rpmsgdev_send_recv(dev, command, true, &msg.header,
                           sizeof(msg) - 1, &read);

  return read.iov_len ? read.iov_len : ret;
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
  struct rpmsgdev_cookie_s cookie;
  uint32_t space;
  size_t written = 0;
  int ret;

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  /* Recover our private data from the struct file instance */

  dev  = filep->f_inode->i_private;
  priv = filep->f_priv;
  DEBUGASSERT(dev != NULL && priv != NULL);

  /* If the open flags is not nonblock, should poll the device for
   * write ready first to avoid the server rptun thread blocked in
   * device write operation.
   */

  if (priv->nonblock == false)
    {
      ret = rpmsgdev_wait(filep, POLLOUT);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Perform the rpmsg write */

  memset(&cookie, 0, sizeof(cookie));
  nxsem_init(&cookie.sem, 0, 0);

  while (written < buflen)
    {
      msg = rpmsgdev_get_tx_payload_buffer(dev, &space);
      if (msg == NULL)
        {
          ret = -ENOMEM;
          goto out;
        }

      space -= sizeof(*msg) - 1;
      if (space >= buflen - written)
        {
          /* Send complete, set cookie is valid, need ack */

          space = buflen - written;
          msg->header.cookie = (uintptr_t)&cookie;
        }
      else if ((dev->flags & RPMSGDEV_NOFRAG_WRITE) != 0)
        {
          rpmsg_release_tx_buffer(&dev->ept, msg);
          ret = -EMSGSIZE;
          goto out;
        }
      else
        {
          /* Not send complete, set cookie invalid, do not need ack */

          msg->header.cookie = 0;
        }

      msg->header.command = RPMSGDEV_WRITE;
      msg->header.result  = -ENXIO;
      msg->filep          = priv->filep;
      msg->count          = space;
      memcpy(msg->buf, buffer + written, space);

      ret = rpmsg_send_nocopy(&dev->ept, msg, sizeof(*msg) - 1 + space);
      if (ret < 0)
        {
          goto out;
        }

      written += space;
    }

  ret = rpmsg_wait(&dev->ept, &cookie.sem);
  if (ret < 0)
    {
      goto out;
    }

  ret = cookie.result;

out:
  nxsem_destroy(&cookie.sem);
  return ret < 0 ? ret : buflen;
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
 * Name: rpmsgdev_ioctl_arglen
 *
 * Description:
 *   Get rpmsg device ioctl argument length according to the command
 *
 * Parameters:
 *   cmd - the ioctl command
 *
 * Returned Values:
 *   0        - ioctl command not support
 *   positive - the argument length
 *
 ****************************************************************************/

static ssize_t rpmsgdev_ioctl_arglen(int cmd)
{
  switch (cmd)
    {
      case FIONBIO:
      case FIONWRITE:
      case FIONREAD:
      case FIONSPACE:
      case FBIOSET_POWER:
      case FBIOGET_POWER:
      case BATIOC_STATE:
        return sizeof(int);
      case TUNSETIFF:
      case TUNGETIFF:
        return sizeof(struct ifreq);
      case FIOC_FILEPATH:
        return PATH_MAX;
      case BATIOC_GET_PROTOCOL:
      case BATIOC_OPERATE:
        return sizeof(struct batio_operate_msg_s);
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
  uint32_t space;
  ssize_t arglen;
  size_t msglen;
  int ret;

  /* Recover our private data from the struct file instance */

  dev  = filep->f_inode->i_private;
  priv = filep->f_priv;
  DEBUGASSERT(dev != NULL && priv != NULL);

  /* Call our internal routine to perform the ioctl */

  arglen = rpmsgdev_ioctl_arglen(cmd);
  if (arglen < 0)
    {
      return arglen;
    }

  msglen = sizeof(*msg) + arglen - 1;

  msg = rpmsgdev_get_tx_payload_buffer(dev, &space);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  msg->filep   = priv->filep;
  msg->request = cmd;
  msg->arg     = arg;
  msg->arglen  = arglen;

  if (arglen > 0)
    {
      memcpy(msg->buf, (FAR void *)(uintptr_t)arg, arglen);
    }

  ret = rpmsgdev_send_recv(dev, RPMSGDEV_IOCTL, false, &msg->header,
                           msglen, arglen > 0 ? (FAR void *)arg : NULL);
  if (cmd == FIONBIO && ret >= 0)
    {
      FAR int *nonblock = (FAR int *)(uintptr_t)arg;
      priv->nonblock = *nonblock;
    }

  return ret;
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

static int rpmsgdev_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  FAR struct rpmsgdev_s *dev;
  FAR struct rpmsgdev_priv_s *priv;
  struct rpmsgdev_poll_s msg;

  /* Recover our private data from the struct file instance */

  dev  = filep->f_inode->i_private;
  priv = filep->f_priv;
  DEBUGASSERT(dev != NULL && priv != NULL);

  /* Setup or teardown the poll */

  msg.filep  = priv->filep;
  msg.events = fds->events;
  msg.setup  = setup;
  msg.fds    = (uint64_t)(uintptr_t)fds;

  return rpmsgdev_send_recv(dev, RPMSGDEV_POLL, true, &msg.header,
                            sizeof(msg), NULL);
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
 *   len   - The got memroy size
 *
 * Returned Values:
 *   NULL     - failure
 *   not NULL - success
 *
 ****************************************************************************/

static FAR void *rpmsgdev_get_tx_payload_buffer(FAR struct rpmsgdev_s *priv,
                                                FAR uint32_t *len)
{
  int sval;

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

  if (copy)
    {
      ret = rpmsg_send(&priv->ept, msg, len);
    }
  else
    {
      ret = rpmsg_send_nocopy(&priv->ept, msg, len);
    }

  if (ret < 0)
    {
      goto fail;
    }

  ret = rpmsg_wait(&priv->ept, &cookie.sem);
  if (ret >= 0)
    {
      ret = cookie.result;
    }

fail:
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
 *   src  - unknow
 *   priv - unknow
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
 *   src  - unknow
 *   priv - unknow
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
  FAR struct iovec *read = cookie->data;

  cookie->result = header->result;
  if (cookie->result > 0)
    {
      memcpy(read->iov_base + read->iov_len, rsp->buf, cookie->result);
      read->iov_len += cookie->result;
    }

  if (header->command == RPMSGDEV_READ_NOFRAG ||
      cookie->result <= 0 || read->iov_len >= rsp->count)
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
 *   src  - unknow
 *   priv - unknow
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
      memcpy(cookie->data, (FAR void *)(uintptr_t)rsp->buf, rsp->arglen);
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
 *   src  - unknow
 *   priv - unknow
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
  FAR struct pollfd *fds;

  fds = (FAR struct pollfd *)(uintptr_t)rsp->fds;
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
      rpmsg_create_ept(&priv->ept, rdev, buf,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       rpmsgdev_ept_cb, NULL);
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
 *   src  - unknow
 *   priv - unknow
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
 *                same as the remotepath, provide this argument to supoort
 *                custom device path
 *   flags      - RPMSGDEV_NOFRAG_READ and RPMSGDEV_NOFRAG_WRITE can be set
 *                to indicates that the read and write data of the device
 *                cannot be split or aggregated
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmsgdev_register(FAR const char *remotecpu, FAR const char *remotepath,
                      FAR const char *localpath, uint32_t flags)
{
  FAR struct rpmsgdev_s *dev;
  int ret;

  /* Arguments check */

  if (remotecpu == NULL || remotepath == NULL)
    {
      return -EINVAL;
    }

  DEBUGASSERT(strlen(remotepath) + RPMSGDEV_NAME_PREFIX_LEN <=
              RPMSG_NAME_SIZE);

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize the rpmsg device */

  dev->remotecpu  = remotecpu;
  dev->remotepath = remotepath;
  dev->flags      = flags;

  nxsem_init(&dev->wait, 0, 0);

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

  ret = register_driver(localpath, &g_rpmsgdev_ops, 0666, dev);
  if (ret < 0)
    {
      rpmsgdeverr("register driver failed, ret=%d\n", ret);
      goto fail_with_rpmsg;
    }

  return OK;

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
