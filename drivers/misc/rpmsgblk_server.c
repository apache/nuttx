/****************************************************************************
 * drivers/misc/rpmsgblk_server.c
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

#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mmcsd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/rpmsg/rpmsg.h>

#include "inode.h"
#include "rpmsgblk.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsgblk_server_s
{
  struct rpmsg_endpoint              ept;
  FAR struct inode                  *blknode;
  FAR const struct block_operations *bops;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Functions handle the messages from the client cpu */

static int rpmsgblk_open_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv);
static int rpmsgblk_close_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv);
static int rpmsgblk_read_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv);
static int rpmsgblk_write_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv);
static int rpmsgblk_geometry_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv);
static int rpmsgblk_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv);

/* Functions for creating communication with client cpu */

static bool rpmsgblk_ns_match(FAR struct rpmsg_device *rdev,
                              FAR void *priv, FAR const char *name,
                              uint32_t dest);
static void rpmsgblk_ns_bind(FAR struct rpmsg_device *rdev,
                             FAR void *priv, FAR const char *name,
                             uint32_t dest);
static int  rpmsgblk_ept_cb(FAR struct rpmsg_endpoint *ept,
                            FAR void *data, size_t len, uint32_t src,
                            FAR void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const rpmsg_ept_cb g_rpmsgblk_handler[] =
{
  [RPMSGBLK_OPEN]     = rpmsgblk_open_handler,
  [RPMSGBLK_CLOSE]    = rpmsgblk_close_handler,
  [RPMSGBLK_READ]     = rpmsgblk_read_handler,
  [RPMSGBLK_WRITE]    = rpmsgblk_write_handler,
  [RPMSGBLK_GEOMETRY] = rpmsgblk_geometry_handler,
  [RPMSGBLK_IOCTL]    = rpmsgblk_ioctl_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsgblk_open_handler
 ****************************************************************************/

static int rpmsgblk_open_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgblk_server_s *server = ept->priv;
  FAR struct rpmsgblk_open_s *msg = data;

  /* To check if the block device has been removed by unlink operation. */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (server->blknode->i_peer == NULL)
    {
      msg->header.result = -ENODEV;
      return rpmsg_send(ept, msg, sizeof(*msg));
    }
#endif

  if (server->bops->open != NULL)
    {
      msg->header.result = server->bops->open(server->blknode);
      if (msg->header.result < 0)
        {
          ferr("block device open failed, ret=%d\n", msg->header.result);
        }
    }
  else
    {
      msg->header.result = 0;
    }

  return rpmsg_send(ept, msg, sizeof(*msg));
}

/****************************************************************************
 * Name: rpmsgblk_close_handler
 ****************************************************************************/

static int rpmsgblk_close_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgblk_server_s *server = ept->priv;
  FAR struct rpmsgblk_close_s *msg = data;

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (server->blknode->i_peer == NULL)
    {
      msg->header.result = -ENODEV;
      return rpmsg_send(ept, msg, sizeof(*msg));
    }
#endif

  if (server->bops->close != NULL)
    {
      msg->header.result = server->bops->close(server->blknode);
      if (msg->header.result < 0)
        {
          ferr("block device close failed, ret=%d\n", msg->header.result);
        }
    }
  else
    {
      msg->header.result = 0;
    }

  return rpmsg_send(ept, msg, sizeof(*msg));
}

/****************************************************************************
 * Name: rpmsgblk_read_handler
 ****************************************************************************/

static int rpmsgblk_read_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgblk_server_s *server = ept->priv;
  FAR struct rpmsgblk_read_s *msg = data;
  FAR struct rpmsgblk_read_s *rsp;
  int ret = -ENOENT;
  size_t read = 0;
  size_t nsectors;
  uint32_t space;

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (server->blknode->i_peer == NULL)
    {
      msg->header.result = -ENODEV;
      return rpmsg_send(ept, msg, sizeof(*msg) - 1);
    }
#endif

  while (read < msg->nsectors)
    {
      rsp = rpmsg_get_tx_payload_buffer(ept, &space, true);
      if (rsp == NULL)
        {
          ferr("get tx payload failed or no enough space\n");
          return -ENOMEM;
        }

      DEBUGASSERT(space >= sizeof(*msg) - 1 + msg->sectorsize);

      *rsp = *msg;

      nsectors = (space - sizeof(*msg) + 1) / msg->sectorsize;
      if (nsectors > msg->nsectors - read)
        {
          nsectors = msg->nsectors - read;
        }

      ret = server->bops->read(server->blknode,
                               (FAR unsigned char *)rsp->buf,
                               msg->startsector, nsectors);
      rsp->header.result = ret;
      if (rpmsg_send_nocopy(ept, rsp, (ret < 0 ? 0 : ret * msg->sectorsize) +
                                      sizeof(*rsp) - 1) < 0)
        {
          rpmsg_release_tx_buffer(ept, rsp);
        }

      if (ret <= 0)
        {
          ferr("mtd block read failed\n");
          break;
        }

      read += ret;
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsgblk_write_handler
 ****************************************************************************/

static int rpmsgblk_write_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgblk_server_s *server = ept->priv;
  FAR struct rpmsgblk_write_s *msg = data;
  int ret;

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (server->blknode->i_peer == NULL)
    {
      msg->header.result = -ENODEV;
      return rpmsg_send(ept, msg, sizeof(*msg) - 1);
    }
#endif

  ret = server->bops->write(server->blknode, (FAR unsigned char *)msg->buf,
                            msg->startsector, msg->nsectors);
  if (ret <= 0)
    {
      ferr("mtd block write failed\n");
    }

  /* cookie != 0 indicate the data has been sent complete, so send back
   * the total written blocks.
   */

  if (msg->header.cookie != 0)
    {
      msg->header.result = ret;
      return rpmsg_send(ept, msg, sizeof(*msg) - 1);
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsgblk_ioctl_handler
 ****************************************************************************/

static int rpmsgblk_geometry_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv)
{
  FAR struct rpmsgblk_server_s *server = ept->priv;
  FAR struct rpmsgblk_geometry_s *msg = data;
  struct geometry geo;

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (server->blknode->i_peer == NULL)
    {
      msg->header.result = -ENODEV;
      return rpmsg_send(ept, msg, len);
    }
#endif

  msg->header.result = server->bops->geometry(server->blknode, &geo);

  DEBUGASSERT(strlen(geo.geo_model) <= RPMSGBLK_NAME_MAX);

  msg->available = geo.geo_available;
  msg->mediachanged = geo.geo_mediachanged;
  msg->writeenabled = geo.geo_writeenabled;
  msg->nsectors = geo.geo_nsectors;
  msg->sectorsize = geo.geo_sectorsize;
  strlcpy(msg->model, geo.geo_model, sizeof(msg->model));

  return rpmsg_send(ept, msg, len);
}

/****************************************************************************
 * Name: rpmsgblk_mmc_cmd_handler
 ****************************************************************************/

static int rpmsgblk_mmc_cmd_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR struct rpmsgblk_ioctl_s *msg)
{
  FAR struct rpmsgblk_server_s *server = ept->priv;
  FAR struct mmc_ioc_cmd *ioc =
    (FAR struct mmc_ioc_cmd *)(uintptr_t)msg->buf;
  FAR struct rpmsgblk_ioctl_s *rsp;
  FAR struct mmc_ioc_cmd *ioc_rsp;
  size_t rsplen;
  size_t arglen;
  uint32_t space;
  int ret;

  arglen = sizeof(struct mmc_ioc_cmd);
  if (!ioc->write_flag)
    {
      arglen += ioc->blksz * ioc->blocks;
    }

  rsplen = sizeof(*rsp) + arglen - 1;
  rsp = rpmsg_get_tx_payload_buffer(ept, &space, true);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(space >= rsplen);

  memcpy(rsp, msg, sizeof(*rsp) + sizeof(struct mmc_ioc_cmd) - 1);
  rsp->arglen = arglen;
  ioc_rsp = (FAR struct mmc_ioc_cmd *)(uintptr_t)rsp->buf;

  if (ioc_rsp->write_flag)
    {
      ioc_rsp->data_ptr = (uint64_t)(uintptr_t)(msg->buf + sizeof(*ioc_rsp));
    }
  else
    {
      ioc_rsp->data_ptr = (uint64_t)(uintptr_t)(rsp->buf + sizeof(*ioc_rsp));
    }

  rsp->header.result = server->bops->ioctl(server->blknode, rsp->request,
                                           (unsigned long)rsp->buf);
  ret = rpmsg_send_nocopy(ept, rsp, rsplen);
  if (ret < 0)
    {
      rpmsg_release_tx_buffer(ept, rsp);
    }

  return ret;
}

/****************************************************************************
 * Name: rpmsgblk_mmc_cmd_handler
 ****************************************************************************/

static int rpmsgblk_mmc_multi_cmd_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR struct rpmsgblk_ioctl_s *msg)
{
  FAR struct rpmsgblk_server_s *server = ept->priv;
  FAR struct mmc_ioc_multi_cmd *mioc =
    (FAR struct mmc_ioc_multi_cmd *)(uintptr_t)msg->buf;
  FAR struct rpmsgblk_ioctl_s *rsp;
  FAR struct mmc_ioc_multi_cmd *mioc_rsp;
  size_t rsplen;
  size_t arglen;
  size_t off;
  size_t rsp_off;
  uint32_t space;
  uint64_t i;
  int ret;

  arglen = sizeof(struct mmc_ioc_multi_cmd) +
           mioc->num_of_cmds * sizeof(struct mmc_ioc_cmd);
  for (i = 0; i < mioc->num_of_cmds; i++)
    {
      if (!mioc->cmds[i].write_flag)
        {
          arglen += mioc->cmds[i].blksz * mioc->cmds[i].blocks;
        }
    }

  rsplen = sizeof(*rsp) + arglen - 1;
  rsp = rpmsg_get_tx_payload_buffer(ept, &space, true);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(space >= rsplen);

  off = sizeof(struct mmc_ioc_multi_cmd) +
        mioc->num_of_cmds * sizeof(struct mmc_ioc_cmd);

  /* Consist of the rsp msg */

  memcpy(rsp, msg, sizeof(*rsp) + off - 1);
  rsp->arglen = arglen;
  mioc_rsp = (FAR struct mmc_ioc_multi_cmd *)(uintptr_t)rsp->buf;
  rsp_off = off;
  for (i = 0; i < mioc_rsp->num_of_cmds; i++)
    {
      if (mioc_rsp->cmds[i].write_flag)
        {
          mioc_rsp->cmds[i].data_ptr = (uint64_t)(uintptr_t)
                                       (msg->buf + off);
          off += mioc_rsp->cmds[i].blksz * mioc_rsp->cmds[i].blocks;
        }
      else
        {
          mioc_rsp->cmds[i].data_ptr = (uint64_t)(uintptr_t)
                                       (rsp->buf + rsp_off);
          rsp_off += mioc_rsp->cmds[i].blksz * mioc_rsp->cmds[i].blocks;
        }
    }

  rsp->header.result = server->bops->ioctl(server->blknode, rsp->request,
                                           (unsigned long)rsp->buf);
  ret = rpmsg_send_nocopy(ept, rsp, rsplen);
  if (ret < 0)
    {
      rpmsg_release_tx_buffer(ept, rsp);
    }

  return ret;
}

/****************************************************************************
 * Name: rpmsgblk_ioctl_handler
 ****************************************************************************/

static int rpmsgblk_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgblk_server_s *server = ept->priv;
  FAR struct rpmsgblk_ioctl_s *msg = data;

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (server->blknode->i_peer == NULL)
    {
      msg->header.result = -ENODEV;
      return rpmsg_send(ept, msg, len);
    }
#endif

  switch (msg->request)
    {
      case MMC_IOC_CMD:
        return rpmsgblk_mmc_cmd_handler(ept, data);

      case MMC_IOC_MULTI_CMD:
        return rpmsgblk_mmc_multi_cmd_handler(ept, data);

      default:
        break;
    }

  msg->header.result = server->bops->ioctl(server->blknode, msg->request,
                                           msg->arglen > 0 ?
                                           (unsigned long)msg->buf :
                                           msg->arg);

  return rpmsg_send(ept, msg, len);
}

/****************************************************************************
 * Name: rpmsgblk_ns_match
 ****************************************************************************/

static bool rpmsgblk_ns_match(FAR struct rpmsg_device *rdev,
                              FAR void *priv, FAR const char *name,
                              uint32_t dest)
{
  return !strncmp(name, RPMSGBLK_NAME_PREFIX, RPMSGBLK_NAME_PREFIX_LEN);
}

/****************************************************************************
 * Name: rpmsgblk_ept_release
 ****************************************************************************/

static void rpmsgblk_ept_release(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsgblk_server_s *server = ept->priv;

  inode_release(server->blknode);
  kmm_free(server);
}

/****************************************************************************
 * Name: rpmsgblk_ns_bind
 ****************************************************************************/

static void rpmsgblk_ns_bind(FAR struct rpmsg_device *rdev,
                             FAR void *priv, FAR const char *name,
                             uint32_t dest)
{
  FAR struct rpmsgblk_server_s *server;
  int ret;

  server = kmm_zalloc(sizeof(*server));
  if (server == NULL)
    {
      ferr("mtd server malloced failed\n");
      return;
    }

  ret = find_blockdriver(&name[RPMSGBLK_NAME_PREFIX_LEN], 0,
                         &server->blknode);
  if (ret < 0)
    {
      ferr("ERROR: Failed to find %s block driver\n",
           &name[RPMSGBLK_NAME_PREFIX_LEN]);
      kmm_free(server);
      return;
    }

  server->ept.priv = server;
  server->ept.release_cb = rpmsgblk_ept_release;
  server->bops = server->blknode->u.i_bops;

  ret = rpmsg_create_ept(&server->ept, rdev, name,
                         RPMSG_ADDR_ANY, dest,
                         rpmsgblk_ept_cb, rpmsg_destroy_ept);
  if (ret < 0)
    {
      ferr("endpoint create failed, ret=%d\n", ret);
      inode_release(server->blknode);
      kmm_free(server);
    }
}

/****************************************************************************
 * Name: rpmsgblk_ept_cb
 ****************************************************************************/

static int rpmsgblk_ept_cb(FAR struct rpmsg_endpoint *ept,
                           FAR void *data, size_t len, uint32_t src,
                           FAR void *priv)
{
  FAR struct rpmsgblk_header_s *header = data;
  uint32_t command = header->command;

  if (command < nitems(g_rpmsgblk_handler))
    {
      return g_rpmsgblk_handler[command](ept, data, len, src, priv);
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsgblk_server_init
 *
 * Description:
 *   Rpmsg-mtd server initialize function, the server cpu should call
 *   this function.
 *
 * Parameters:
 *   None
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmsgblk_server_init(void)
{
  return rpmsg_register_callback(NULL,
                                 NULL,
                                 NULL,
                                 rpmsgblk_ns_match,
                                 rpmsgblk_ns_bind);
}
