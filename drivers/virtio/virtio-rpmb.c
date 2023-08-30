/****************************************************************************
 * drivers/virtio/virtio-rpmb.c
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
#include <stdio.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>
#include <nuttx/mmcsd.h>
#include <nuttx/virtio/virtio.h>

#include "virtio-rpmb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_RPMB_DEVNAME "/dev/rpmb"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct virtio_rpmb_priv_s
{
  /* The virtio device we're associated with */

  FAR struct virtio_device *vdev;
};

struct virtio_rpmb_cookie_s
{
  sem_t  sem;
  size_t len;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int virtio_rpmb_probe(FAR struct virtio_device *vdev);
static void virtio_rpmb_remove(FAR struct virtio_device *vdev);
static int virtio_rpmb_ioctl(FAR struct file *, int, unsigned long);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct virtio_driver g_virtio_rpmb_driver =
{
  LIST_INITIAL_VALUE(g_virtio_rpmb_driver.node), /* node */
  VIRTIO_ID_RPMB,                                /* device id */
  virtio_rpmb_probe,                             /* probe */
  virtio_rpmb_remove,                            /* remove */
};

static const struct file_operations g_virtio_rpmb_ops =
{
  NULL,               /* open */
  NULL,               /* close */
  NULL,               /* read */
  NULL,               /* write */
  NULL,               /* seek */
  virtio_rpmb_ioctl,  /* ioctl */
  NULL,               /* mmap */
  NULL,               /* truncate */
  NULL,               /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL,               /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* virtio_rpmb_done() - virtual queue completion callback */

static void virtio_rpmb_done(FAR struct virtqueue *vq)
{
  FAR struct virtio_rpmb_cookie_s *cookie;
  uint32_t len;

  cookie = virtqueue_get_buffer(vq, &len, NULL);
  if (cookie != NULL)
    {
      /* Assign the return length */

      cookie->len = len;

      /* Read completed, post the sem */

      nxsem_post(&cookie->sem);
    }
}

/* This is just a simple helper for processing the virtqueue buf. It will
 * block until the resp arrives. Returns number of bytes written back
 * or negative if it failed.
 */

static int virtio_rpmb_transact(FAR struct virtio_rpmb_priv_s *priv,
                                FAR struct virtqueue_buf *vb,
                                int out, int in)
{
  FAR struct virtqueue *vq = priv->vdev->vrings_info[0].vq;
  struct virtio_rpmb_cookie_s cookie;
  int ret;

  /* Init the cookie */

  cookie.len = 0;
  nxsem_init(&cookie.sem, 0, 0);

  /* Add the input buffer to the virtqueue, and the cookie as the virtqueue
   * cookie. (virtqueue_get_buffer() will return cookie).
   */

  ret = virtqueue_add_buffer(vq, vb, out, in, &cookie);
  if (ret < 0)
    {
      return ret;
    }

  /* Notify the other side to process the added virtqueue buffer */

  virtqueue_kick(vq);

  /* Wait fot completion */

  nxsem_wait_uninterruptible(&cookie.sem);
  return cookie.len;
}

/****************************************************************************
 * Name: virtio_blk_ioctl
 ****************************************************************************/

static int virtio_rpmb_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  FAR struct virtio_rpmb_priv_s *priv;
  FAR struct mmc_ioc_multi_cmd *mioc = (FAR struct mmc_ioc_multi_cmd *)arg;
  struct virtqueue_buf vb[mioc->num_of_cmds];
  int ret;
  int i;

  priv = (FAR struct virtio_rpmb_priv_s *)filep->f_inode->i_private;

  for (i = 0; i < mioc->num_of_cmds; i++)
    {
      vb[i].buf = (FAR void *)mioc->cmds[i].data_ptr;
      vb[i].len = mioc->cmds[i].blocks * mioc->cmds[i].blksz;
    }

  ret = virtio_rpmb_transact(priv, vb, mioc->num_of_cmds - 1, 1);
  return ret < 0 ? ret : 0;
}

static int virtio_rpmb_init(FAR struct virtio_rpmb_priv_s *priv,
                            FAR struct virtio_device *vdev)
{
  FAR const char *vqname[1];
  vq_callback callback[1];
  int ret;

  priv->vdev = vdev;
  vdev->priv = priv;

  /* Initialize the virtio device */

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
  virtio_set_features(vdev, 0);
  virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);

  vqname[0]   = "virtio_rpmb_vq";
  callback[0] = virtio_rpmb_done;
  ret = virtio_create_virtqueues(vdev, 0, 1, vqname, callback);
  if (ret < 0)
    {
      vrterr("virtio_device_create_virtqueue failed, ret=%d\n", ret);
      goto err_with_lock;
    }

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);
  virtqueue_enable_cb(vdev->vrings_info[0].vq);
  return OK;

err_with_lock:
  return ret;
}

static void virtio_rpmb_uninit(FAR struct virtio_rpmb_priv_s *priv)
{
  FAR struct virtio_device *vdev = priv->vdev;

  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
}

static int virtio_rpmb_probe(FAR struct virtio_device *vdev)
{
  FAR struct virtio_rpmb_priv_s *priv;
  int ret;

  /* Alloc the virtio block driver private data */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vrterr("No enough memory\n");
      return -ENOMEM;
    }

  /* Init the virtio block driver */

  ret = virtio_rpmb_init(priv, vdev);
  if (ret < 0)
    {
      vrterr("virtio_rpmb_init failed, ret=%d\n", ret);
      goto err_with_priv;
    }

  ret = register_driver(VIRTIO_RPMB_DEVNAME, &g_virtio_rpmb_ops, 0666,
                        priv);
  if (ret < 0)
    {
      vrterr("Register NuttX driver failed, ret=%d\n", ret);
      goto err_with_init;
    }

  return ret;

err_with_init:
  virtio_rpmb_uninit(priv);
err_with_priv:
  kmm_free(priv);
  return ret;
}

static void virtio_rpmb_remove(FAR struct virtio_device *vdev)
{
  FAR struct virtio_rpmb_priv_s *priv = vdev->priv;

  unregister_driver(VIRTIO_RPMB_DEVNAME);
  virtio_rpmb_uninit(priv);
  kmm_free(priv);
}

int virtio_register_rpmb_driver(void)
{
  return virtio_register_driver(&g_virtio_rpmb_driver);
}
