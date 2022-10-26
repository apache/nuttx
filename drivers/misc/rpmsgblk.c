/****************************************************************************
 * drivers/misc/rpmsgblk.c
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
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/smart.h>
#include <nuttx/mtd/smart.h>
#include <nuttx/mutex.h>
#include <nuttx/rptun/openamp.h>

#include "rpmsgblk.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsgblk_s
{
  struct block_operations blk;         /* Rpmsg-Block device operation */
  struct rpmsg_endpoint   ept;         /* Rpmsg endpoint */
  FAR const char         *remotecpu;   /* The server cpu name */
  FAR const char         *remotepath;  /* The device path in the server cpu */
  sem_t                   wait;        /* Wait sem, used for preventing any
                                        * opreation until the connection
                                        * between two cpu established.
                                        */
  mutex_t                 lock;        /* Lock for thread-safe */
  struct geometry         geo;         /* block geomerty */
  int                     refs;        /* refence count */
};

/* Rpmsg device cookie used to handle the response from the remote cpu */

struct rpmsgblk_cookie_s
{
  sem_t     sem;     /* Semaphore used fo rpmsg */
  int       result;  /* The return value of the remote call */
  FAR void *data;    /* The return data buffer of the remote call */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* The block operation functions */

static int     rpmsgblk_open(FAR struct inode *inode);
static int     rpmsgblk_close(FAR struct inode *inode);
static ssize_t rpmsgblk_read(FAR struct inode *inode,
                             FAR unsigned char *buffer,
                             blkcnt_t start_sector, unsigned int nsectors);
static ssize_t rpmsgblk_write(FAR struct inode *inode,
                              FAR const unsigned char *buffer,
                              blkcnt_t start_sector, unsigned int nsectors);
static int     rpmsgblk_geometry(FAR struct inode *inode,
                                 FAR struct geometry *geometry);
static size_t  rpmsgblk_ioctl_arglen(int cmd);
static int     rpmsgblk_ioctl(FAR struct inode *inode, int cmd,
                              unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     rpmsgblk_unlink(FAR struct inode *inode);
#endif

/* Functions for sending data to the remote cpu */

static int     rpmsgblk_send_recv(FAR struct rpmsgblk_s *priv,
                                  uint32_t command, bool copy,
                                  FAR struct rpmsgblk_header_s *msg,
                                  int len, FAR void *data);
static FAR void *rpmsgblk_get_tx_payload_buffer(FAR struct rpmsgblk_s *priv,
                                                FAR uint32_t *len);

/* Functions handle the responses from the remote cpu */

static int     rpmsgblk_default_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv);
static int     rpmsgblk_read_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv);
static int     rpmsgblk_geometry_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv);
static int     rpmsgblk_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv);

/* Functions for creating communication with remote cpu */

static void    rpmsgblk_device_created(struct rpmsg_device *rdev,
                                       FAR void *priv_);
static void    rpmsgblk_device_destroy(struct rpmsg_device *rdev,
                                       FAR void *priv_);
static int     rpmsgblk_ept_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len, uint32_t src,
                               FAR void *priv);
static void    rpmsgblk_ns_bound(struct rpmsg_endpoint *ept);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Rpmsg device response handler table */

static const rpmsg_ept_cb g_rpmsgblk_handler[] =
{
  [RPMSGBLK_OPEN]     = rpmsgblk_default_handler,
  [RPMSGBLK_CLOSE]    = rpmsgblk_default_handler,
  [RPMSGBLK_READ]     = rpmsgblk_read_handler,
  [RPMSGBLK_WRITE]    = rpmsgblk_default_handler,
  [RPMSGBLK_GEOMETRY] = rpmsgblk_geometry_handler,
  [RPMSGBLK_IOCTL]    = rpmsgblk_ioctl_handler,
  [RPMSGBLK_UNLINK]   = rpmsgblk_default_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsgblk_open
 *
 * Description:
 *   Rpmsg-blk open operation
 *
 * Parameters:
 *   inode
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgblk_open(FAR struct inode *inode)
{
  FAR struct rpmsgblk_s *priv = (FAR struct rpmsgblk_s *)inode->i_private;
  struct rpmsgblk_open_s msg;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(priv != NULL);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      ferr("lock failed, ret=%d\n", ret);
      return ret;
    }

  /* Check if this is the first time that the driver has been opened. */

  if (priv->refs++ == 0)
    {
      /* Try to open the block device in the remote cpu */

      ret = rpmsgblk_send_recv(priv, RPMSGBLK_OPEN, true, &msg.header,
                               sizeof(msg), NULL);
      if (ret < 0)
        {
          ferr("open failed, ret=%d\n", ret);
          priv->refs--;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgblk_close
 *
 * Description:
 *   Rpmsg-blk close operation
 *
 * Parameters:
 *   inode
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgblk_close(FAR struct inode *inode)
{
  FAR struct rpmsgblk_s *priv = (FAR struct rpmsgblk_s *)inode->i_private;
  struct rpmsgblk_close_s msg;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      ferr("lock failed, ret=%d\n", ret);
      return ret;
    }

  /* There are no more references to the port */

  if (--priv->refs == 0)
    {
      ret = rpmsgblk_send_recv(priv, RPMSGBLK_CLOSE, true, &msg.header,
                               sizeof(msg), NULL);
      if (ret < 0)
        {
          ferr("close failed, ret=%d\n", ret);
          priv->refs++;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgblk_read
 *
 * Description:
 *   Rpmsg-blk read operation
 *
 * Parameters:
 *   dev    - the blk device
 *   offset - read offset address in the blk device
 *   nbytes - read number in bytes
 *   buffer - read buffer
 *
 * Returned Values:
 *   The positive non-zero number of bytes read on success, 0 on if an
 *   end-of-file condition, or a negated errno value on any failure.
 *
 ****************************************************************************/

static ssize_t rpmsgblk_read(FAR struct inode *inode,
                             FAR unsigned char *buffer,
                             blkcnt_t start_sector, unsigned int nsectors)
{
  FAR struct rpmsgblk_s *priv = (FAR struct rpmsgblk_s *)inode->i_private;
  struct rpmsgblk_read_s msg;
  struct iovec iov;
  int ret;

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  /* Sanity checks */

  DEBUGASSERT(priv != NULL);

  ret = rpmsgblk_geometry(inode, &priv->geo);
  if (ret < 0)
    {
      ferr("Get geometry failed, ret=%d\n", ret);
      return ret;
    }

  /* In block read, iov_len represent the received block number */

  iov.iov_base = buffer;
  iov.iov_len  = 0;

  msg.startsector = start_sector;
  msg.nsectors    = nsectors;
  msg.sectorsize  = priv->geo.geo_sectorsize;

  ret = rpmsgblk_send_recv(priv, RPMSGBLK_READ, true, &msg.header,
                           sizeof(msg) - 1, &iov);

  return ret < 0 ? ret : iov.iov_len;
}

/****************************************************************************
 * Name: rpmsgblk_write
 *
 * Description:
 *   Rpmsg-blk write operation
 *
 * Parameters:
 *   dev    - the blk device
 *   offset - write offset address in the blk device
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

static ssize_t rpmsgblk_write(FAR struct inode *inode,
                              FAR const unsigned char *buffer,
                              blkcnt_t start_sector, unsigned int nsectors)
{
  FAR struct rpmsgblk_s *priv = (FAR struct rpmsgblk_s *)inode->i_private;
  FAR struct rpmsgblk_write_s *msg;
  struct rpmsgblk_cookie_s cookie;
  uint32_t sectorsize;
  uint32_t space;
  size_t written = 0;
  int ret;

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  /* Sanity checks */

  DEBUGASSERT(priv != NULL);

  ret = rpmsgblk_geometry(inode, &priv->geo);
  if (ret < 0)
    {
      ferr("Get geometry failed, ret=%d\n", ret);
      return ret;
    }

  /* Perform the rpmsg write */

  memset(&cookie, 0, sizeof(cookie));
  nxsem_init(&cookie.sem, 0, 0);

  sectorsize = priv->geo.geo_sectorsize;
  while (written < nsectors)
    {
      msg = rpmsgblk_get_tx_payload_buffer(priv, &space);
      if (msg == NULL)
        {
          ret = -ENOMEM;
          goto out;
        }

      DEBUGASSERT(sizeof(*msg) - 1 + sectorsize <= space);

      msg->nsectors = (space - sizeof(*msg) + 1) / sectorsize;
      if (msg->nsectors >= nsectors - written)
        {
          /* Send complete, set cookie is valid, need ack */

          msg->nsectors = nsectors - written;
          msg->header.cookie = (uintptr_t)&cookie;
        }
      else
        {
          /* Not send complete, set cookie invalid, do not need ack */

          msg->header.cookie = 0;
        }

      msg->header.command = RPMSGBLK_WRITE;
      msg->header.result  = -ENXIO;
      msg->startsector    = start_sector;
      msg->sectorsize     = sectorsize;
      memcpy(msg->buf, buffer, msg->nsectors * sectorsize);

      buffer       += msg->nsectors * sectorsize;
      start_sector += msg->nsectors;
      written      += msg->nsectors;

      ret = rpmsg_send_nocopy(&priv->ept, msg,
                              sizeof(*msg) - 1 + msg->nsectors * sectorsize);
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
  return ret < 0 ? ret : nsectors;
}

/****************************************************************************
 * Name: rpmsgblk_geometry
 *
 * Description:
 *   Rpmsg-blk geometry operation
 *
 * Parameters:
 *   inode    - the blk device inode
 *   geometry - pointer to the application geomoetry struct
 *
 * Returned Values:
 *   On success, the number of bytes written are returned (zero indicates
 *   nothing was written).  On any failure, a negated errno value is returned
 *   (see comments withwrite() for a description of the appropriate errno
 *   values).
 *
 ****************************************************************************/

static int rpmsgblk_geometry(FAR struct inode *inode,
                             FAR struct geometry *geometry)
{
  FAR struct rpmsgblk_s *priv = (FAR struct rpmsgblk_s *)inode->i_private;
  struct rpmsgblk_geometry_s *msg;
  uint32_t space;
  int msglen;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(priv != NULL);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Return the perviously got geometry */

  if (priv->geo.geo_sectorsize != 0)
    {
      memcpy(geometry, &priv->geo, sizeof(*geometry));
      goto out;
    }

  msglen = sizeof(*msg) + sizeof(*geometry) - 1;

  msg = rpmsgblk_get_tx_payload_buffer(priv, &space);
  if (msg == NULL)
    {
      ret = -ENOMEM;
      goto out;
    }

  DEBUGASSERT(space > msglen);

  msg->arg    = (uintptr_t)geometry;
  msg->arglen = sizeof(*geometry);
  memcpy(msg->buf, geometry, sizeof(*geometry));

  ret = rpmsgblk_send_recv(priv, RPMSGBLK_GEOMETRY, false, &msg->header,
                           msglen, geometry);
  if (ret >= 0)
    {
      memcpy(&priv->geo, geometry, sizeof(priv->geo));
    }

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgblk_ioctl_arglen
 *
 * Description:
 *   Get rpmsg blk ioctl argument length according to the command
 *
 * Parameters:
 *   cmd - the ioctl command
 *
 * Returned Values:
 *   0        - ioctl command not support
 *   positive - the argument length
 *
 ****************************************************************************/

static size_t rpmsgblk_ioctl_arglen(int cmd)
{
  switch (cmd)
    {
      case BIOC_XIPBASE:
        return sizeof(uintptr_t);
      case BIOC_PROBE:
      case BIOC_EJECT:
      case BIOC_LLFORMAT:
      case BIOC_GETFORMAT:
      case BIOC_ALLOCSECT:
      case BIOC_FREESECT:
      case BIOC_FLUSH:
        return 0;
      case BIOC_READSECT:
      case BIOC_WRITESECT:
        return sizeof(struct smart_read_write_s);
      case BIOC_GETPROCFSD:
        return sizeof(struct mtd_smart_procfs_data_s);
      case BIOC_DEBUGCMD:
        return sizeof(struct mtd_smart_debug_data_s);
      case BIOC_GEOMETRY:
        return sizeof(struct geometry);
      case BIOC_PARTINFO:
        return sizeof(struct partition_info_s);
      case BIOC_BLKSSZGET:
        return sizeof(blksize_t);
      default:
        return 0;
    }
}

/****************************************************************************
 * Name: rpmsgblk_ioctl
 *
 * Description:
 *   Rpmsg-blk ioctl operation
 *
 * Parameters:
 *   inode - the blk device inode
 *   cmd   - the ioctl command
 *   arg   - the ioctl arguments
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgblk_ioctl(FAR struct inode *inode, int cmd,
                          unsigned long arg)
{
  FAR struct rpmsgblk_s *priv = (FAR struct rpmsgblk_s *)inode->i_private;
  FAR struct rpmsgblk_ioctl_s *msg;
  uint32_t space;
  size_t arglen;
  size_t msglen;

  /* Sanity checks */

  DEBUGASSERT(priv != NULL);

  /* Call our internal routine to perform the ioctl */

  arglen = rpmsgblk_ioctl_arglen(cmd);
  msglen = sizeof(*msg) + arglen - 1;

  msg = rpmsgblk_get_tx_payload_buffer(priv, &space);
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

  return rpmsgblk_send_recv(priv, RPMSGBLK_IOCTL, false, &msg->header,
                            msglen, arglen > 0 ? (FAR void *)arg : NULL);
}

/****************************************************************************
 * Name: rpmsgblk_unlink
 *
 * Description:
 *   Rpmsg-blk ioctl operation
 *
 * Parameters:
 *   inode - the blk device inode
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rpmsgblk_unlink(FAR struct inode *inode)
{
  FAR struct rpmsgblk_s *priv = (FAR struct rpmsgblk_s *)inode->i_private;
  struct rpmsgblk_unlink_s msg;
  int ret;

  ret = rpmsgblk_send_recv(priv, RPMSGBLK_UNLINK, true, &msg.header,
                           sizeof(msg), NULL);
  if (ret < 0)
    {
      ferr("unlink failed, ret=%d\n", ret);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: rpmsgblk_get_tx_payload_buffer
 *
 * Description:
 *   Get the rpmsg blk tx payload, the buffer is from the rpmsg share memory
 *   that can be accessed by local and remote cpu.
 *
 * Parameters:
 *   priv  - The rpmsg-blk handle
 *   len   - The got memroy size
 *
 * Returned Values:
 *   NULL     - failure
 *   not NULL - success
 *
 ****************************************************************************/

static FAR void *rpmsgblk_get_tx_payload_buffer(FAR struct rpmsgblk_s *priv,
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
 * Name: rpmsgblk_send_recv
 *
 * Description:
 *   Send and receive the rpmsg data.
 *
 * Parameters:
 *   priv    - rpmsg blk handle
 *   command - the command, RPMSGBLK_OPEN, RPMSGBLK_CLOSE, RPMSGBLK_READ,
 *                          RPMSGBLK_WRITE, RPMSGBLK_IOCTL
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

static int rpmsgblk_send_recv(FAR struct rpmsgblk_s *priv,
                              uint32_t command, bool copy,
                              FAR struct rpmsgblk_header_s *msg,
                              int len, FAR void *data)
{
  struct rpmsgblk_cookie_s cookie;
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
 * Name: rpmsgblk_default_handler
 *
 * Description:
 *   Default rpmsg-blk response handler, this function will be called to
 *   process the return message of rpmsgblk_open(), rpmsgblk_close() and
 *   rpmsgblk_write().
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

static int rpmsgblk_default_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv)
{
  FAR struct rpmsgblk_header_s *header = data;
  FAR struct rpmsgblk_cookie_s *cookie =
      (FAR struct rpmsgblk_cookie_s *)(uintptr_t)header->cookie;

  cookie->result = header->result;
  if (cookie->result >= 0 && cookie->data)
    {
      memcpy(cookie->data, data, len);
    }

  return rpmsg_post(ept, &cookie->sem);
}

/****************************************************************************
 * Name: rpmsgblk_read_handler
 *
 * Description:
 *   Rpmsg-blk block read response handler, this function will be called to
 *   process the return message of rpmsgblk_bread().
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

static int rpmsgblk_read_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgblk_header_s *header = data;
  FAR struct rpmsgblk_cookie_s *cookie =
      (FAR struct rpmsgblk_cookie_s *)(uintptr_t)header->cookie;
  FAR struct rpmsgblk_read_s *rsp = data;
  FAR struct iovec *iov = cookie->data;
  size_t read;

  cookie->result = header->result;
  if (cookie->result > 0)
    {
      read = cookie->result * rsp->sectorsize;
      memcpy(iov->iov_base, rsp->buf, read);
      iov->iov_base += read;
      iov->iov_len  += cookie->result;
    }

  if (cookie->result <= 0 || iov->iov_len >= rsp->nsectors)
    {
      return rpmsg_post(ept, &cookie->sem);
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsgblk_geometry_handler
 *
 * Description:
 *   Rpmsg-blk geometry response handler, this function will be called to
 *   process the return message of rpmsgblk_geometry().
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

static int rpmsgblk_geometry_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv)
{
  FAR struct rpmsgblk_header_s *header = data;
  FAR struct rpmsgblk_cookie_s *cookie =
      (FAR struct rpmsgblk_cookie_s *)(uintptr_t)header->cookie;
  FAR struct rpmsgblk_geometry_s *rsp = data;

  if (cookie->result >= 0 && rsp->arglen > 0)
    {
      memcpy(cookie->data, rsp->buf, rsp->arglen);
    }

  return rpmsg_post(ept, &cookie->sem);
}

/****************************************************************************
 * Name: rpmsgblk_ioctl_handler
 *
 * Description:
 *   Rpmsg-blk ioctl response handler, this function will be called to
 *   process the return message of rpmsgblk_ioctl().
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

static int rpmsgblk_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgblk_header_s *header = data;
  FAR struct rpmsgblk_cookie_s *cookie =
      (FAR struct rpmsgblk_cookie_s *)(uintptr_t)header->cookie;
  FAR struct rpmsgblk_ioctl_s *rsp = data;

  if (cookie->result >= 0 && rsp->arglen > 0)
    {
      memcpy(cookie->data, rsp->buf, rsp->arglen);
    }

  return rpmsg_post(ept, &cookie->sem);
}

/****************************************************************************
 * Name: rpmsgblk_ns_bound
 *
 * Description:
 *   Rpmsg blk end point service bound callback function , called when
 *   remote end point address is received.
 *
 * Parameters:
 *   ept  - The rpmsg-blk end point
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsgblk_ns_bound(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsgblk_s *priv = ept->priv;

  rpmsg_post(&priv->ept, &priv->wait);
}

/****************************************************************************
 * Name: rpmsgblk_device_created
 *
 * Description:
 *   Rpmsg blk create function, this function will be called by rptun to
 *   create a rpmsg-blk end point.
 *
 * Parameters:
 *   rdev  - The rpmsg-blk end point
 *   priv_ - Rpmsg-blk handle
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsgblk_device_created(FAR struct rpmsg_device *rdev,
                                    FAR void *priv_)
{
  FAR struct rpmsgblk_s *priv = priv_;
  char buf[RPMSG_NAME_SIZE];

  if (strcmp(priv->remotecpu, rpmsg_get_cpuname(rdev)) == 0)
    {
      priv->ept.priv = priv;
      priv->ept.ns_bound_cb = rpmsgblk_ns_bound;
      snprintf(buf, sizeof(buf), "%s%s", RPMSGBLK_NAME_PREFIX,
               priv->remotepath);
      rpmsg_create_ept(&priv->ept, rdev, buf,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       rpmsgblk_ept_cb, NULL);
    }
}

/****************************************************************************
 * Name: rpmsgblk_device_destroy
 *
 * Description:
 *   Rpmsg blk destroy function, this function will be called by rptun to
 *   destroy rpmsg-blk end point.
 *
 * Parameters:
 *   rdev  - The rpmsg-blk end point
 *   priv_ - Rpmsg-blk handle
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsgblk_device_destroy(FAR struct rpmsg_device *rdev,
                                    FAR void *priv_)
{
  FAR struct rpmsgblk_s *priv = priv_;

  if (strcmp(priv->remotecpu, rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_destroy_ept(&priv->ept);
    }
}

/****************************************************************************
 * Name: rpmsgblk_ept_cb
 *
 * Description:
 *   Rpmsg blk end point callback function, this function will be called
 *   when receive the remote cpu message.
 *
 * Parameters:
 *   ept  - The rpmsg-blk end point
 *   data - The received data
 *   len  - The received data length
 *   src  - unknow
 *   priv - unknow
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsgblk_ept_cb(FAR struct rpmsg_endpoint *ept,
                           FAR void *data, size_t len, uint32_t src,
                           FAR void *priv)
{
  FAR struct rpmsgblk_header_s *header = data;
  uint32_t command = header->command;

  if (command < ARRAY_SIZE(g_rpmsgblk_handler))
    {
      return g_rpmsgblk_handler[command](ept, data, len, src, priv);
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsgblk_register
 *
 * Description:
 *   Rpmsg-blk client initialize function, the client cpu should call
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

int rpmsgblk_register(FAR const char *remotecpu, FAR const char *remotepath,
                      FAR const char *localpath)
{
  FAR struct rpmsgblk_s *dev;
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

  dev->blk.open     = rpmsgblk_open;
  dev->blk.close    = rpmsgblk_close;
  dev->blk.read     = rpmsgblk_read;
  dev->blk.write    = rpmsgblk_write;
  dev->blk.geometry = rpmsgblk_geometry;
  dev->blk.ioctl    = rpmsgblk_ioctl;
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  dev->blk.unlink   = rpmsgblk_unlink;
#endif

  /* Initialize the rpmsg device */

  dev->remotecpu  = remotecpu;
  dev->remotepath = remotepath;

  nxsem_init(&dev->wait, 0, 0);
  nxmutex_init(&dev->lock);

  /* Register the rpmsg callback */

  ret = rpmsg_register_callback(dev,
                                rpmsgblk_device_created,
                                rpmsgblk_device_destroy,
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

  ret = register_blockdriver(localpath, &dev->blk, 0755, dev);
  if (ret < 0)
    {
      ferr("ERROR: register driver failed, ret=%d\n", ret);
      goto fail_with_rpmsg;
    }

  return OK;

fail_with_rpmsg:
  rpmsg_unregister_callback(dev,
                            rpmsgblk_device_created,
                            rpmsgblk_device_destroy,
                            NULL,
                            NULL);

fail:
  nxsem_destroy(&dev->wait);
  nxmutex_destroy(&dev->lock);
  kmm_free(dev);
  return ret;
}
