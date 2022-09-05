/****************************************************************************
 * drivers/mtd/rpmsgmtd.c
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
#include <limits.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mutex.h>
#include <nuttx/rptun/openamp.h>

#include "rpmsgmtd.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsgmtd_s
{
  struct mtd_dev_s      mtd;         /* MTD device */
  struct rpmsg_endpoint ept;         /* Rpmsg endpoint */
  FAR const char       *remotecpu;   /* The server cpu name */
  FAR const char       *remotepath;  /* The device path in the server cpu */
  sem_t                 wait;        /* Wait sem, used for preventing any
                                      * opreation until the connection
                                      * between two cpu established.
                                      */
  mutex_t               geoexcl;     /* Get mtd geometry operation mutex */
  struct mtd_geometry_s geo;         /* MTD geomerty */
};

/* Rpmsg device cookie used to handle the response from the remote cpu */

struct rpmsgmtd_cookie_s
{
  sem_t     sem;     /* Semaphore used fo rpmsg */
  int       result;  /* The return value of the remote call */
  FAR void *data;    /* The return data buffer of the remote call */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* The mtd operation functions */

static int     rpmsgmtd_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks);
static int     rpmsgmtd_get_geometry(FAR struct rpmsgmtd_s *dev);
static ssize_t rpmsgmtd_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, FAR uint8_t *buffer);
static ssize_t rpmsgmtd_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                               size_t nblocks, FAR const uint8_t *buffer);
static ssize_t rpmsgmtd_read(FAR struct mtd_dev_s *dev, off_t offset,
                             size_t nbytes, FAR uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t rpmsgmtd_write(FAR struct mtd_dev_s *dev, off_t offset,
                              size_t nbytes, FAR const uint8_t *buffer);
#endif
static size_t  rpmsgmtd_ioctl_arglen(int cmd);
static int     rpmsgmtd_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                              unsigned long arg);

/* Functions for sending data to the remote cpu */

static int     rpmsgmtd_send_recv(FAR struct rpmsgmtd_s *priv,
                                  uint32_t command, bool copy,
                                  FAR struct rpmsgmtd_header_s *msg,
                                  int len, FAR void *data);
static FAR void *rpmsgmtd_get_tx_payload_buffer(FAR struct rpmsgmtd_s *priv,
                                                FAR uint32_t *len);

/* Functions handle the responses from the remote cpu */

static int     rpmsgmtd_default_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv);
static int     rpmsgmtd_bread_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv);
static int     rpmsgmtd_read_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv);
static int     rpmsgmtd_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv);

/* Functions for creating communication with remote cpu */

static void    rpmsgmtd_device_created(struct rpmsg_device *rdev,
                                       FAR void *priv_);
static void    rpmsgmtd_device_destroy(struct rpmsg_device *rdev,
                                       FAR void *priv_);
static int     rpmsgmtd_ept_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len, uint32_t src,
                               FAR void *priv);
static void    rpmsgmtd_ns_bound(struct rpmsg_endpoint *ept);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Rpmsg device response handler table */

static const rpmsg_ept_cb g_rpmsgmtd_handler[] =
{
  [RPMSGMTD_ERASE]  = rpmsgmtd_default_handler,
  [RPMSGMTD_BREAD]  = rpmsgmtd_bread_handler,
  [RPMSGMTD_BWRITE] = rpmsgmtd_default_handler,
  [RPMSGMTD_READ]   = rpmsgmtd_read_handler,
  [RPMSGMTD_WRITE]  = rpmsgmtd_default_handler,
  [RPMSGMTD_IOCTL]  = rpmsgmtd_ioctl_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsgmtd_erase
 *
 * Description:
 *   Rpmsg-mtd erase operation
 *
 * Parameters:
 *   dev        - the mtd device
 *   startblock - erase start block
 *   nblocks    - erase block number
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgmtd_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks)
{
  FAR struct rpmsgmtd_s *priv = (FAR struct rpmsgmtd_s *)dev;
  struct rpmsgmtd_erase_s msg;

  /* Sanity checks */

  DEBUGASSERT(priv != NULL);

  msg.startblock = startblock;
  msg.nblocks    = nblocks;

  return rpmsgmtd_send_recv(priv, RPMSGMTD_ERASE, true, &msg.header,
                            sizeof(msg), NULL);
}

/****************************************************************************
 * Name: rpmsgmtd_get_geometry
 *
 * Description:
 *   Rpmsg-mtd get the server mtd device geometry
 *
 * Parameters:
 *   dev - the mtd device
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgmtd_get_geometry(FAR struct rpmsgmtd_s *dev)
{
  int ret;

  ret = nxmutex_lock(&dev->geoexcl);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->geo.blocksize == 0)
    {
      /* Get the server mtd device geometry */

      ret = rpmsgmtd_ioctl(&dev->mtd, MTDIOC_GEOMETRY,
                           (unsigned long)&dev->geo);
    }

  nxmutex_unlock(&dev->geoexcl);
  return ret;
}

/****************************************************************************
 * Name: rpmsgmtd_bread
 *
 * Description:
 *   Rpmsg-mtd block read operation
 *
 * Parameters:
 *   dev        - the mtd device
 *   startblock - read start block
 *   nblocks    - read block number
 *   buffer     - read buffer
 *
 * Returned Values:
 *   The positive non-zero number of blocks read on success, 0 on if an
 *   end-of-file condition, or a negated errno value on any failure.
 *
 ****************************************************************************/

static ssize_t rpmsgmtd_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, FAR uint8_t *buffer)
{
  FAR struct rpmsgmtd_s *priv = (FAR struct rpmsgmtd_s *)dev;
  struct rpmsgmtd_bread_s msg;
  struct iovec iov;
  int ret;

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  /* Sanity checks */

  DEBUGASSERT(priv != NULL);

  /* Get the server mtd geometry */

  ret = rpmsgmtd_get_geometry(priv);
  if (ret < 0)
    {
      ferr("Get geometry failed, ret=%d\n", ret);
      return ret;
    }

  /* In block read, iov_len represent the received block number */

  iov.iov_base = buffer;
  iov.iov_len  = 0;

  msg.startblock = startblock;
  msg.nblocks    = nblocks;
  msg.blocksize  = priv->geo.blocksize;

  ret = rpmsgmtd_send_recv(priv, RPMSGMTD_BREAD, true, &msg.header,
                           sizeof(msg) - 1, &iov);

  return ret < 0 ? ret : iov.iov_len;
}

/****************************************************************************
 * Name: rpmsgmtd_bwrite
 *
 * Description:
 *   Rpmsg-mtd block write operation
 *
 * Parameters:
 *   dev        - the mtd device
 *   startblock - write start block
 *   nblocks    - write block number
 *   buffer     - write buffer
 *
 * Returned Values:
 *   On success, the number of blocks written are returned (zero indicates
 *   nothing was written).  On any failure, a negated errno value is returned
 *   (see comments withwrite() for a description of the appropriate errno
 *   values).
 *
 ****************************************************************************/

static ssize_t rpmsgmtd_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                               size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct rpmsgmtd_s *priv = (FAR struct rpmsgmtd_s *)dev;
  FAR struct rpmsgmtd_bwrite_s *msg;
  struct rpmsgmtd_cookie_s cookie;
  uint32_t blocksize;
  uint32_t space;
  size_t written = 0;
  int ret;

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  /* Sanity checks */

  DEBUGASSERT(priv != NULL);

  /* Get the server mtd geometry */

  ret = rpmsgmtd_get_geometry(priv);
  if (ret < 0)
    {
      ferr("Get geometry failed, ret=%d\n", ret);
      return ret;
    }

  /* Perform the rpmsg write */

  memset(&cookie, 0, sizeof(cookie));
  nxsem_init(&cookie.sem, 0, 0);
  nxsem_set_protocol(&cookie.sem, SEM_PRIO_NONE);

  blocksize = priv->geo.blocksize;
  while (written < nblocks)
    {
      msg = rpmsgmtd_get_tx_payload_buffer(priv, &space);
      if (msg == NULL)
        {
          ret = -ENOMEM;
          goto out;
        }

      DEBUGASSERT(sizeof(*msg) - 1 + blocksize <= space);

      msg->nblocks = (space - sizeof(*msg) + 1) / blocksize;
      if (msg->nblocks >= nblocks - written)
        {
          /* Send complete, set cookie is valid, need ack */

          msg->nblocks = nblocks - written;
          msg->header.cookie = (uintptr_t)&cookie;
        }
      else
        {
          /* Not send complete, set cookie invalid, do not need ack */

          msg->header.cookie = 0;
        }

      msg->header.command = RPMSGMTD_BWRITE;
      msg->header.result  = -ENXIO;
      msg->startblock     = startblock;
      msg->blocksize      = blocksize;
      memcpy(msg->buf, buffer, msg->nblocks * blocksize);

      buffer     += msg->nblocks * blocksize;
      startblock += msg->nblocks;
      written    += msg->nblocks;

      ret = rpmsg_send_nocopy(&priv->ept, msg,
                              sizeof(*msg) - 1 + msg->nblocks * blocksize);
      if (ret < 0)
        {
          goto out;
        }
    }

  ret = rpmsg_wait(&priv->ept, &cookie.sem);
  if (ret < 0)
    {
      goto out;
    }

  ret = cookie.result;

out:
  nxsem_destroy(&cookie.sem);
  return ret < 0 ? ret : nblocks;
}

/****************************************************************************
 * Name: rpmsgmtd_read
 *
 * Description:
 *   Rpmsg-mtd read operation
 *
 * Parameters:
 *   dev    - the mtd device
 *   offset - read offset address in the mtd device
 *   nbytes - read number in bytes
 *   buffer - read buffer
 *
 * Returned Values:
 *   The positive non-zero number of bytes read on success, 0 on if an
 *   end-of-file condition, or a negated errno value on any failure.
 *
 ****************************************************************************/

static ssize_t rpmsgmtd_read(FAR struct mtd_dev_s *dev, off_t offset,
                             size_t nbytes, FAR uint8_t *buffer)
{
  FAR struct rpmsgmtd_s *priv = (FAR struct rpmsgmtd_s *)dev;
  struct rpmsgmtd_read_s msg;
  struct iovec iov;
  ssize_t ret;

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  /* Sanity checks */

  DEBUGASSERT(priv != NULL);

  /* Call the host to perform the read */

  iov.iov_base = buffer;
  iov.iov_len  = 0;

  msg.offset = offset;
  msg.nbytes = nbytes;

  ret = rpmsgmtd_send_recv(priv, RPMSGMTD_READ, true, &msg.header,
                           sizeof(msg) - 1, &iov);

  return ret < 0 ? ret : iov.iov_len;
}

/****************************************************************************
 * Name: rpmsgmtd_write
 *
 * Description:
 *   Rpmsg-mtd write operation
 *
 * Parameters:
 *   dev    - the mtd device
 *   offset - write offset address in the mtd device
 *   nbytes - write number in bytes
 *   buffer - write buffer
 *
 * Returned Values:
 *   On success, the number of bytes written are returned (zero indicates
 *   nothing was written).  On any failure, a negated errno value is returned
 *   (see comments withwrite() for a description of the appropriate errno
 *   values).
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t rpmsgmtd_write(FAR struct mtd_dev_s *dev, off_t offset,
                              size_t nbytes, FAR const uint8_t *buffer)
{
  FAR struct rpmsgmtd_s *priv = (FAR struct rpmsgmtd_s *)dev;
  FAR struct rpmsgmtd_write_s *msg;
  struct rpmsgmtd_cookie_s cookie;
  uint32_t space;
  size_t written = 0;
  int ret;

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  /* Sanity checks */

  DEBUGASSERT(priv != NULL);

  /* Perform the rpmsg write */

  memset(&cookie, 0, sizeof(cookie));
  nxsem_init(&cookie.sem, 0, 0);
  nxsem_set_protocol(&cookie.sem, SEM_PRIO_NONE);

  while (written < nbytes)
    {
      msg = rpmsgmtd_get_tx_payload_buffer(priv, &space);
      if (msg == NULL)
        {
          ret = -ENOMEM;
          goto out;
        }

      space -= sizeof(*msg) - 1;
      if (space >= nbytes - written)
        {
          /* Send complete, set cookie is valid, need ack */

          space = nbytes - written;
          msg->header.cookie = (uintptr_t)&cookie;
        }
      else
        {
          /* Not send complete, set cookie invalid, do not need ack */

          msg->header.cookie = 0;
        }

      msg->header.command = RPMSGMTD_WRITE;
      msg->header.result  = -ENXIO;
      msg->offset         = offset;
      msg->nbytes         = space;
      memcpy(msg->buf, buffer, space);

      ret = rpmsg_send_nocopy(&priv->ept, msg, sizeof(*msg) - 1 + space);
      if (ret < 0)
        {
          goto out;
        }

      buffer  += space;
      offset  += space;
      written += space;
    }

  ret = rpmsg_wait(&priv->ept, &cookie.sem);
  if (ret < 0)
    {
      goto out;
    }

  ret = cookie.result;

out:
  nxsem_destroy(&cookie.sem);
  return ret < 0 ? ret : nbytes;
}
#endif

/****************************************************************************
 * Name: rpmsgmtd_ioctl_arglen
 *
 * Description:
 *   Get rpmsg mtd ioctl argument length according to the command
 *
 * Parameters:
 *   cmd - the ioctl command
 *
 * Returned Values:
 *   0        - ioctl command not support
 *   positive - the argument length
 *
 ****************************************************************************/

static size_t rpmsgmtd_ioctl_arglen(int cmd)
{
  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        return sizeof(struct mtd_geometry_s);
      case MTDIOC_PROTECT:
      case MTDIOC_UNPROTECT:
        return sizeof(struct mtd_protect_s);
      default:
        return 0;
    }
}

/****************************************************************************
 * Name: rpmsgmtd_ioctl
 *
 * Description:
 *   Rpmsg-mtd ioctl operation
 *
 * Parameters:
 *   dev - the mtd device
 *   cmd - the ioctl command
 *   arg - the ioctl arguments
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgmtd_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                          unsigned long arg)
{
  FAR struct rpmsgmtd_s *priv = (FAR struct rpmsgmtd_s *)dev;
  FAR struct rpmsgmtd_ioctl_s *msg;
  uint32_t space;
  size_t arglen;
  size_t msglen;

  /* Sanity checks */

  DEBUGASSERT(priv != NULL);

  /* Call our internal routine to perform the ioctl */

  arglen = rpmsgmtd_ioctl_arglen(cmd);
  msglen = sizeof(*msg) + arglen - 1;

  msg = rpmsgmtd_get_tx_payload_buffer(priv, &space);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  msg->request = cmd;
  msg->arg     = arg;
  msg->arglen  = arglen;

  if (arglen > 0)
    {
      memcpy(msg->buf, (FAR void *)(uintptr_t)arg, arglen);
    }

  return rpmsgmtd_send_recv(priv, RPMSGMTD_IOCTL, false, &msg->header,
                            msglen, arglen > 0 ? (FAR void *)arg : NULL);
}

/****************************************************************************
 * Name: rpmsgmtd_get_tx_payload_buffer
 *
 * Description:
 *   Get the rpmsg mtd tx payload, the buffer is from the rpmsg share memory
 *   that can be accessed by local and remote cpu.
 *
 * Parameters:
 *   priv  - The rpmsg-mtd handle
 *   len   - The got memroy size
 *
 * Returned Values:
 *   NULL     - failure
 *   not NULL - success
 *
 ****************************************************************************/

static FAR void *rpmsgmtd_get_tx_payload_buffer(FAR struct rpmsgmtd_s *priv,
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
 * Name: rpmsgmtd_send_recv
 *
 * Description:
 *   Send and receive the rpmsg data.
 *
 * Parameters:
 *   priv    - rpmsg mtd handle
 *   command - the command, RPMSGMTD_OPEN, RPMSGMTD_CLOSE, RPMSGMTD_READ,
 *                          RPMSGMTD_WRITE, RPMSGMTD_IOCTL
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

static int rpmsgmtd_send_recv(FAR struct rpmsgmtd_s *priv,
                              uint32_t command, bool copy,
                              FAR struct rpmsgmtd_header_s *msg,
                              int len, FAR void *data)
{
  struct rpmsgmtd_cookie_s cookie;
  int ret;

  memset(&cookie, 0, sizeof(cookie));
  nxsem_init(&cookie.sem, 0, 0);
  nxsem_set_protocol(&cookie.sem, SEM_PRIO_NONE);

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
 * Name: rpmsgmtd_default_handler
 *
 * Description:
 *   Default rpmsg-mtd response handler, this function will be called to
 *   process the return message of rpmsgmtd_open(), rpmsgmtd_close() and
 *   rpmsgmtd_write().
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

static int rpmsgmtd_default_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv)
{
  FAR struct rpmsgmtd_header_s *header = data;
  FAR struct rpmsgmtd_cookie_s *cookie =
      (FAR struct rpmsgmtd_cookie_s *)(uintptr_t)header->cookie;

  cookie->result = header->result;
  if (cookie->result >= 0 && cookie->data)
    {
      memcpy(cookie->data, data, len);
    }

  rpmsg_post(ept, &cookie->sem);
  return 0;
}

/****************************************************************************
 * Name: rpmsgmtd_bread_handler
 *
 * Description:
 *   Rpmsg-mtd block read response handler, this function will be called to
 *   process the return message of rpmsgmtd_bread().
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

static int rpmsgmtd_bread_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgmtd_header_s *header = data;
  FAR struct rpmsgmtd_cookie_s *cookie =
      (FAR struct rpmsgmtd_cookie_s *)(uintptr_t)header->cookie;
  FAR struct rpmsgmtd_bread_s *rsp = data;
  FAR struct iovec *iov = cookie->data;
  size_t read;

  cookie->result = header->result;
  if (cookie->result > 0)
    {
      read = cookie->result * rsp->blocksize;
      memcpy(iov->iov_base, rsp->buf, read);
      iov->iov_base += read;
      iov->iov_len  += cookie->result;
    }

  if (cookie->result <= 0 || iov->iov_len >= rsp->nblocks)
    {
      rpmsg_post(ept, &cookie->sem);
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsgmtd_read_handler
 *
 * Description:
 *   Rpmsg-mtd read response handler, this function will be called to
 *   process the return message of rpmsgmtd_read().
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

static int rpmsgmtd_read_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgmtd_header_s *header = data;
  FAR struct rpmsgmtd_cookie_s *cookie =
      (FAR struct rpmsgmtd_cookie_s *)(uintptr_t)header->cookie;
  FAR struct rpmsgmtd_read_s *rsp = data;
  FAR struct iovec *iov = cookie->data;

  cookie->result = header->result;
  if (cookie->result > 0)
    {
      memcpy(iov->iov_base + iov->iov_len, rsp->buf, cookie->result);
      iov->iov_len += cookie->result;
    }

  if (cookie->result <= 0 || iov->iov_len >= rsp->nbytes)
    {
      rpmsg_post(ept, &cookie->sem);
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsgmtd_ioctl_handler
 *
 * Description:
 *   Rpmsg-mtd ioctl response handler, this function will be called to
 *   process the return message of rpmsgmtd_ioctl().
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

static int rpmsgmtd_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgmtd_header_s *header = data;
  FAR struct rpmsgmtd_cookie_s *cookie =
      (FAR struct rpmsgmtd_cookie_s *)(uintptr_t)header->cookie;
  FAR struct rpmsgmtd_ioctl_s *rsp = data;

  if (cookie->result >= 0 && rsp->arglen > 0)
    {
      memcpy(cookie->data, (FAR void *)(uintptr_t)rsp->buf, rsp->arglen);
    }

  rpmsg_post(ept, &cookie->sem);
  return 0;
}

/****************************************************************************
 * Name: rpmsgmtd_ns_bound
 *
 * Description:
 *   Rpmsg mtd end point service bound callback function , called when
 *   remote end point address is received.
 *
 * Parameters:
 *   ept  - The rpmsg-mtd end point
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsgmtd_ns_bound(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsgmtd_s *priv = ept->priv;

  rpmsg_post(&priv->ept, &priv->wait);
}

/****************************************************************************
 * Name: rpmsgmtd_device_created
 *
 * Description:
 *   Rpmsg mtd create function, this function will be called by rptun to
 *   create a rpmsg-mtd end point.
 *
 * Parameters:
 *   rdev  - The rpmsg-mtd end point
 *   priv_ - Rpmsg-mtd handle
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsgmtd_device_created(FAR struct rpmsg_device *rdev,
                                    FAR void *priv_)
{
  FAR struct rpmsgmtd_s *priv = priv_;
  char buf[RPMSG_NAME_SIZE];

  if (strcmp(priv->remotecpu, rpmsg_get_cpuname(rdev)) == 0)
    {
      priv->ept.priv = priv;
      priv->ept.ns_bound_cb = rpmsgmtd_ns_bound;
      snprintf(buf, sizeof(buf), "%s%s", RPMSGMTD_NAME_PREFIX,
               priv->remotepath);
      rpmsg_create_ept(&priv->ept, rdev, buf,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       rpmsgmtd_ept_cb, NULL);
    }
}

/****************************************************************************
 * Name: rpmsgmtd_device_destroy
 *
 * Description:
 *   Rpmsg mtd destroy function, this function will be called by rptun to
 *   destroy rpmsg-mtd end point.
 *
 * Parameters:
 *   rdev  - The rpmsg-mtd end point
 *   priv_ - Rpmsg-mtd handle
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsgmtd_device_destroy(FAR struct rpmsg_device *rdev,
                                    FAR void *priv_)
{
  FAR struct rpmsgmtd_s *priv = priv_;

  if (strcmp(priv->remotecpu, rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_destroy_ept(&priv->ept);
    }
}

/****************************************************************************
 * Name: rpmsgmtd_ept_cb
 *
 * Description:
 *   Rpmsg mtd end point callback function, this function will be called
 *   when receive the remote cpu message.
 *
 * Parameters:
 *   ept  - The rpmsg-mtd end point
 *   data - The received data
 *   len  - The received data length
 *   src  - unknow
 *   priv - unknow
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgmtd_ept_cb(FAR struct rpmsg_endpoint *ept,
                           FAR void *data, size_t len, uint32_t src,
                           FAR void *priv)
{
  FAR struct rpmsgmtd_header_s *header = data;
  uint32_t command = header->command;

  if (command < ARRAY_SIZE(g_rpmsgmtd_handler))
    {
      return g_rpmsgmtd_handler[command](ept, data, len, src, priv);
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsgmtd_register
 *
 * Description:
 *   Rpmsg-mtd client initialize function, the client cpu should call
 *   this function in the board initialize process.
 *
 * Parameters:
 *   remotecpu  - the server cpu name
 *   remotepath - the device you want to access in the remote cpu
 *   localpath  - the device path in local cpu, if NULL, the localpath is
 *                same as the remotepath, provide this argument to supoort
 *                custom device path
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmsgmtd_register(FAR const char *remotecpu, FAR const char *remotepath,
                      FAR const char *localpath)
{
  FAR struct rpmsgmtd_s *dev;
  int ret;

  /* Arguments check */

  if (remotecpu == NULL || remotepath == NULL)
    {
      ferr("ERROR: Input arguments null\n");
      return -EINVAL;
    }

  /* Create an instance of the RPMSG MTD device */

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      ferr("ERROR: Failed to allocate the RPMSG MTD degice\n");
      return -ENOMEM;
    }

  /* Perform initialization as necessary. (unsupported methods were
   * nullified by kmm_zalloc).
   */

  dev->mtd.erase  = rpmsgmtd_erase;
  dev->mtd.bread  = rpmsgmtd_bread;
  dev->mtd.bwrite = rpmsgmtd_bwrite;
  dev->mtd.read   = rpmsgmtd_read;
#ifdef CONFIG_MTD_BYTE_WRITE
  dev->mtd.write  = rpmsgmtd_write;
#endif
  dev->mtd.ioctl  = rpmsgmtd_ioctl;
  dev->mtd.name   = "rpmsgmtd";

  /* Initialize the rpmsg device */

  dev->remotecpu  = remotecpu;
  dev->remotepath = remotepath;

  nxsem_init(&dev->wait, 0, 0);
  nxsem_set_protocol(&dev->wait, SEM_PRIO_NONE);
  nxmutex_init(&dev->geoexcl);

  /* Register the rpmsg callback */

  ret = rpmsg_register_callback(dev,
                                rpmsgmtd_device_created,
                                rpmsgmtd_device_destroy,
                                NULL,
                                NULL);
  if (ret < 0)
    {
      ferr("ERROR: register callback failed, ret=%d\n", ret);
      goto fail;
    }

  /* Register driver, using the remotepath if localpath is NULL */

  if (localpath == NULL)
    {
      localpath = remotepath;
    }

  ret = register_mtddriver(localpath, &dev->mtd, 0755, dev);
  if (ret < 0)
    {
      ferr("ERROR: register driver failed, ret=%d\n", ret);
      goto fail_with_rpmsg;
    }

  return OK;

fail_with_rpmsg:
  rpmsg_unregister_callback(dev,
                            rpmsgmtd_device_created,
                            rpmsgmtd_device_destroy,
                            NULL,
                            NULL);

fail:
  nxsem_destroy(&dev->wait);
  kmm_free(dev);
  return ret;
}
