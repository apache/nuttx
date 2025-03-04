/****************************************************************************
 * drivers/vhost/vhost-rng.c
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

#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/vhost/vhost.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vhost_rng_priv_s
{
  FAR struct vhost_device *hdev;
  struct work_s            work;
  spinlock_t               lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Vhost driver functions */

static int vhost_rng_probe(FAR struct vhost_device *hdev);
static void vhost_rng_remove(FAR struct vhost_device *hdev);

static void vhost_rng_work(FAR void *arg);
static void vhost_rng_handler(FAR struct virtqueue *vq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct vhost_driver g_vhost_rng_driver =
{
  LIST_INITIAL_VALUE(g_vhost_rng_driver.node), /* Node */
  VIRTIO_ID_ENTROPY,                           /* Device id */
  vhost_rng_probe,                             /* Probe */
  vhost_rng_remove,                            /* Remove */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_rng_thread
 ****************************************************************************/

static void vhost_rng_work(FAR void *arg)
{
  FAR struct vhost_rng_priv_s *priv = arg;
  FAR struct virtqueue *vq;
  FAR void *buf;
  irqstate_t flags;
  uint16_t idx;
  uint32_t len;
  ssize_t ret;

  vq = priv->hdev->vrings_info[0].vq;
  flags = spin_lock_irqsave(&priv->lock);
  for (; ; )
    {
      buf = virtqueue_get_available_buffer(vq, &idx, &len);
      if (buf == NULL)
        {
          break;
        }

      spin_unlock_irqrestore(&priv->lock, flags);
      arc4random_buf(buf, len);

      flags = spin_lock_irqsave(&priv->lock);
      virtqueue_add_consumed_buffer(vq, idx, (uint32_t)ret);
      virtqueue_kick(vq);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: vhost_rng_handler
 ****************************************************************************/

static void vhost_rng_handler(FAR struct virtqueue *vq)
{
  FAR struct vhost_rng_priv_s *priv = vq->vq_dev->priv;

  if (work_available(&priv->work))
    {
      work_queue(HPWORK, &priv->work, vhost_rng_work, priv, 0);
    }
}

/****************************************************************************
 * Name: vhost_rng_probe
 ****************************************************************************/

static int vhost_rng_probe(FAR struct vhost_device *hdev)
{
  FAR struct vhost_rng_priv_s *priv;
  FAR const char *vqnames[1];
  vq_callback callback[1];
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vhosterr("No enough memory\n");
      return -ENOMEM;
    }

  spin_lock_init(&priv->lock);
  priv->hdev = hdev;
  hdev->priv = priv;

  /* Create the virtqueues */

  vqnames[0]  = "virtio_rng";
  callback[0] = vhost_rng_handler;
  ret = vhost_create_virtqueues(hdev, 0, 1, vqnames, callback, NULL);
  if (ret < 0)
    {
      vhosterr("virtio_device_create_virtqueue failed, ret=%d\n", ret);
      goto err;
    }

  virtqueue_enable_cb(hdev->vrings_info[0].vq);
  return ret;

err:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: vhost_rng_remove
 ****************************************************************************/

static void vhost_rng_remove(FAR struct vhost_device *hdev)
{
  FAR struct vhost_rng_priv_s *priv = hdev->priv;

  virtqueue_disable_cb(hdev->vrings_info[0].vq);
  work_cancel_sync(HPWORK, &priv->work);
  vhost_delete_virtqueues(hdev);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_register_rng_driver
 ****************************************************************************/

int vhost_register_rng_driver(void)
{
  return vhost_register_driver(&g_vhost_rng_driver);
}
