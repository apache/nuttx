/****************************************************************************
 * drivers/vhost/vhost-blk.c
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

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/vhost/vhost.h>
#include <nuttx/virtio/virtio-blk.h>
#include <nuttx/virtio/virtio-config.h>

#include "vhost-blk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vhost_blk_priv_s
{
  FAR struct vhost_device *hdev;           /* Virtio device */
  sem_t                    semrx;
  pid_t                    pid;
  struct file              bdev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* BLK block_operations functions and they helper function */

static ssize_t vhost_blk_rdwr_handler(FAR struct vhost_blk_priv_s *priv,
                                      FAR struct virtio_blk_req_s *req,
                                      uint16_t idx, FAR uint16_t *next_idx,
                                      bool write);

/* Other functions */

static int  vhost_blk_probe(FAR struct vhost_device *vdev);
static void vhost_blk_remove(FAR struct vhost_device *vdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct vhost_driver g_vhost_blk_driver =
{
  LIST_INITIAL_VALUE(g_vhost_blk_driver.node),  /* node */
  VIRTIO_ID_BLOCK,                              /* device id */
  vhost_blk_probe,                              /* probe */
  vhost_blk_remove,                             /* remove */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_blk_read_handler
 ****************************************************************************/

static ssize_t vhost_blk_rdwr_handler(FAR struct vhost_blk_priv_s *priv,
                                      FAR struct virtio_blk_req_s *req,
                                      uint16_t idx, FAR uint16_t *next_idx,
                                      bool write)
{
  FAR struct virtqueue *vq = priv->hdev->vrings_info[0].vq;
  struct virtqueue_buf vb;
  ssize_t ret = -EIO;
  uint32_t len;

  vb.buf = virtqueue_get_next_avail_buffer(vq, idx, next_idx, &len);
  if (vb.buf == NULL)
    {
      return -ENOBUFS;
    }

  file_seek(&priv->bdev,
            req->sector << VIRTIO_BLK_SECTOR_BITS, SEEK_SET);

  if (write)
    {
      ret = file_write(&priv->bdev, vb.buf, len);
    }
  else
    {
      ret = file_read(&priv->bdev, vb.buf, len);
    }

  if (ret < 0)
    {
      vhosterr("%s error ret=%zd\n", write ? "Write" : "Read", ret);
    }

  vb.len = ret;
  return ret;
}

/****************************************************************************
 * Name: vhost_blk_handler
 ****************************************************************************/

static int vhost_blk_handler(FAR struct vhost_blk_priv_s *priv,
                             FAR struct virtqueue *vq,
                             FAR void *buf, uint16_t idx)
{
  FAR struct virtio_blk_req_s *req = (FAR struct virtio_blk_req_s *)buf;
  FAR struct virtio_blk_resp_s *resp;
  int ret = -EINVAL;

  switch (req->type)
    {
      case VIRTIO_BLK_T_IN:
        ret = vhost_blk_rdwr_handler(priv, req, idx, &idx, false);
        break;
      case VIRTIO_BLK_T_OUT:
        ret = vhost_blk_rdwr_handler(priv, req, idx, &idx, true);
        break;
      case VIRTIO_BLK_T_FLUSH:
        if (vhost_has_feature(priv->hdev, VIRTIO_BLK_F_FLUSH))
          {
            ret = file_ioctl(&priv->bdev, BIOC_FLUSH, 0);
          }
        break;
    }

  resp = virtqueue_get_next_avail_buffer(vq, idx, &idx, NULL);
  if (resp == NULL)
    {
      return -ENOBUFS;
    }

  resp->status = ret < 0 ? VIRTIO_BLK_S_IOERR : VIRTIO_BLK_S_OK;
  return ret;
}

/****************************************************************************
 * Name: vhost_blk_work
 ****************************************************************************/

static void vhost_blk_work(FAR struct vhost_blk_priv_s *priv)
{
  FAR struct virtqueue *vq;
  FAR void *buf;
  uint32_t len;
  uint16_t idx;
  int ret;

  vq = priv->hdev->vrings_info[0].vq;
  for (; ; )
    {
      buf = virtqueue_get_first_avail_buffer(vq, &idx, &len);
      if (buf == NULL)
        {
          break;
        }

      ret = vhost_blk_handler(priv, vq, buf, idx);
      if (ret < 0)
        {
          vhosterr("handler failed, ret=%d\n", ret);
        }

      virtqueue_add_consumed_buffer(vq, idx, len);
    }

  virtqueue_kick(vq);
}

/****************************************************************************
 * Name: vhost_blk_thread
 ****************************************************************************/

static int vhost_blk_thread(int argc, FAR char *argv[])
{
  FAR struct vhost_blk_priv_s *priv =
    (FAR struct vhost_blk_priv_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

  while (1)
    {
      nxsem_wait_uninterruptible(&priv->semrx);
      vhost_blk_work(priv);
    }

  return 0;
}

/****************************************************************************
 * Name: vhost_blk_callback
 ****************************************************************************/

static void vhost_blk_callback(FAR struct virtqueue *vq)
{
  FAR struct vhost_blk_priv_s *priv = vq->vq_dev->priv;
  int semcount;

  nxsem_get_value(&priv->semrx, &semcount);
  while (semcount++ < 1)
    {
      nxsem_post(&priv->semrx);
    }
}

/****************************************************************************
 * Name: vhost_blk_probe
 ****************************************************************************/

static int vhost_blk_probe(FAR struct vhost_device *hdev)
{
  FAR struct vhost_blk_priv_s *priv;
  FAR const char *vqname[1];
  vq_callback callback[1];
  FAR char *argv[2];
  char path[PATH_MAX];
  char arg0[32];
  int ret;

  /* Alloc the vhost block driver private data */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vhosterr("No enough memory\n");
      return -ENOMEM;
    }

  /* Init the vhost block driver */

  priv->hdev = hdev;
  hdev->priv = priv;
  nxsem_init(&priv->semrx, 0, 0);

  /* Read the block config path */

  vhost_read_config_member(priv->hdev, struct vhost_blk_config_s,
                           path, &path);
  ret = file_open(&priv->bdev, path, O_RDWR | O_CLOEXEC);
  if (ret < 0)
    {
      vhosterr("ERROR: Failed to open block driver\n");
      goto err_init;
    }

  vqname[0]   = "virtio_blk";
  callback[0] = vhost_blk_callback;
  ret = vhost_create_virtqueues(hdev, 0, 1, vqname, callback, NULL);
  if (ret < 0)
    {
      vhosterr("vhost_device_create_virtqueue failed, ret=%d\n", ret);
      goto err_vq;
    }

  snprintf(arg0, sizeof(arg0), "%p", priv);
  argv[0] = arg0;
  argv[1] = NULL;

  priv->pid = kthread_create("vhost-blk",
                             CONFIG_DRIVERS_VHOST_BLK_PRIORITY,
                             CONFIG_DRIVERS_VHOST_BLK_STACKSIZE,
                             vhost_blk_thread, argv);
  if (priv->pid < 0)
    {
      vhosterr("vhost blk thread create failed %d\n", priv->pid);
      ret = priv->pid;
      goto err_thread;
    }

  virtqueue_enable_cb(hdev->vrings_info[0].vq);
  return ret;

err_thread:
  vhost_delete_virtqueues(hdev);
err_vq:
  file_close(&priv->bdev);
  nxsem_destroy(&priv->semrx);
err_init:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: vhost_blk_remove
 ****************************************************************************/

static void vhost_blk_remove(FAR struct vhost_device *hdev)
{
  FAR struct vhost_blk_priv_s *priv = hdev->priv;

  virtqueue_disable_cb(hdev->vrings_info[0].vq);
  kthread_delete(priv->pid);
  vhost_delete_virtqueues(hdev);
  file_close(&priv->bdev);
  nxsem_destroy(&priv->semrx);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_register_blk_driver
 ****************************************************************************/

int vhost_register_blk_driver(void)
{
  return vhost_register_driver(&g_vhost_blk_driver);
}
