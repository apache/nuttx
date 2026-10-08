/****************************************************************************
 * fs/v9fs/virtio_9p.c
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

#include <nuttx/debug.h>
#include <errno.h>

#include <nuttx/nuttx.h>
#include <nuttx/spinlock.h>
#ifdef CONFIG_ARCH_ADDRENV
#  include <nuttx/arch.h>
#  include <nuttx/pgalloc.h>
#endif
#include <nuttx/virtio/virtio.h>

#include "client.h"
#include "fs_heap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_9P_MOUNT_TAG 0

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct virtio_9p_config_s
{
  uint16_t tag_len;
  char tag[NAME_MAX];
};

struct virtio_9p_priv_s
{
  FAR struct virtio_device *vdev;
  struct v9fs_transport_s   transport;
  struct virtio_driver      vdrv;
  spinlock_t                lock;
  char                      tag[0];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* VirtIO transport api for 9p */

static int virtio_9p_create(FAR struct v9fs_transport_s **transport,
                            FAR const char *args);
static void virtio_9p_destroy(FAR struct v9fs_transport_s *transport);
static int virtio_9p_request(FAR struct v9fs_transport_s *transport,
                             FAR struct v9fs_payload_s *payload);
static void virtio_9p_done(FAR struct virtqueue *vq);
static int virtio_9p_probe(FAR struct virtio_device *vdev);
static void virtio_9p_remove(FAR struct virtio_device *vdev);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct v9fs_transport_ops_s g_virtio_9p_transport_ops =
{
  virtio_9p_create,  /* create */
  virtio_9p_request, /* request */
  virtio_9p_destroy, /* close */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Name: virtio_9p_split
 *
 * Description:
 *   A buffer in a user address environment is contiguous only in virtual
 *   memory, but the device reads and writes physical memory, and one
 *   descriptor covers one physical range.  Split the buffer where its
 *   physical pages are not adjacent.  With vb NULL, only count.
 *
 * Returned Value:
 *   The number of descriptors the buffer needs.
 *
 ****************************************************************************/

static size_t virtio_9p_split(FAR void *buf, size_t len,
                              FAR struct virtqueue_buf *vb)
{
  uintptr_t va = (uintptr_t)buf;
  uintptr_t next = 0;
  size_t n = 0;

  while (len > 0)
    {
      size_t chunk = MM_PGSIZE - (va & MM_PGMASK);
      uintptr_t pa;

      if (chunk > len)
        {
          chunk = len;
        }

      pa = up_addrenv_va_to_pa((FAR void *)va);
      if (n > 0 && pa == next)
        {
          if (vb != NULL)
            {
              vb[n - 1].len += chunk;
            }
        }
      else
        {
          if (vb != NULL)
            {
              vb[n].buf = (FAR void *)va;
              vb[n].len = chunk;
            }

          n++;
        }

      next = pa + chunk;
      va  += chunk;
      len -= chunk;
    }

  return n;
}
#endif

/****************************************************************************
 * Name: virtio_9p_create
 ****************************************************************************/

static int virtio_9p_create(FAR struct v9fs_transport_s **transport,
                            FAR const char *args)
{
  FAR struct virtio_9p_priv_s *priv;
  FAR const char *start;
  FAR const char *end;
  size_t length;
  int ret;

  /* Parse device name */

  start = strstr(args, "tag=");
  if (start == NULL)
    {
      return -EINVAL;
    }

  start += 4;
  end = strchr(start, ',');
  length = end ? end - start : strlen(start);
  priv = fs_heap_zalloc(sizeof(struct virtio_9p_priv_s) + length + 1);
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  spin_lock_init(&priv->lock);
  memcpy(priv->tag, start, length);
  priv->vdrv.device = VIRTIO_ID_9P;
  priv->vdrv.probe = virtio_9p_probe;
  priv->vdrv.remove = virtio_9p_remove;
  priv->transport.ops = &g_virtio_9p_transport_ops;
  *transport = &priv->transport;
  ret = virtio_register_driver(&priv->vdrv);
  if (priv->vdev == NULL)
    {
      /* No corresponding driver was found, we should return an error */

      ret = -ENODEV;
    }

  if (ret < 0)
    {
      fs_heap_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_9p_destroy
 ****************************************************************************/

static void virtio_9p_destroy(FAR struct v9fs_transport_s *transport)
{
  FAR struct virtio_9p_priv_s *priv =
            container_of(transport, struct virtio_9p_priv_s, transport);

  virtio_unregister_driver(&priv->vdrv);
  fs_heap_free(priv);
}

/****************************************************************************
 * Name: virtio_9p_request
 ****************************************************************************/

static int virtio_9p_request(FAR struct v9fs_transport_s *transport,
                             FAR struct v9fs_payload_s *payload)
{
  FAR struct virtio_9p_priv_s *priv =
             container_of(transport, struct virtio_9p_priv_s, transport);
  FAR struct virtqueue *vq = priv->vdev->vrings_info[0].vq;
#ifdef CONFIG_ARCH_ADDRENV
  size_t wcount = 0;
  size_t rcount = 0;
  size_t n;
#else
  size_t wcount = payload->wcount;
  size_t rcount = payload->rcount;
#endif
  irqstate_t flags;
  size_t i;
  int ret;

#ifdef CONFIG_ARCH_ADDRENV
  for (i = 0; i < payload->wcount; i++)
    {
      wcount += virtio_9p_split(payload->wiov[i].iov_base,
                                payload->wiov[i].iov_len, NULL);
    }

  for (i = 0; i < payload->rcount; i++)
    {
      rcount += virtio_9p_split(payload->riov[i].iov_base,
                                payload->riov[i].iov_len, NULL);
    }
#endif

  struct virtqueue_buf vb[wcount + rcount];

#ifdef CONFIG_ARCH_ADDRENV
  n = 0;
  for (i = 0; i < payload->wcount; i++)
    {
      n += virtio_9p_split(payload->wiov[i].iov_base,
                           payload->wiov[i].iov_len, &vb[n]);
    }

  for (i = 0; i < payload->rcount; i++)
    {
      n += virtio_9p_split(payload->riov[i].iov_base,
                           payload->riov[i].iov_len, &vb[n]);
    }
#else
  for (i = 0; i < wcount; i++)
    {
      vb[i].buf = payload->wiov[i].iov_base;
      vb[i].len = payload->wiov[i].iov_len;
    }

  for (i = 0; i < rcount; i++)
    {
      vb[wcount + i].buf = payload->riov[i].iov_base;
      vb[wcount + i].len = payload->riov[i].iov_len;
    }
#endif

  flags = spin_lock_irqsave(&priv->lock);
  ret = virtqueue_add_buffer(vq, vb, wcount, rcount, payload);
  if (ret < 0)
    {
      vrterr("virtqueue_add_buffer failed, ret=%d\n", ret);
      goto out;
    }

  virtqueue_kick(vq);
out:
  spin_unlock_irqrestore(&priv->lock, flags);
  return ret;
}

/****************************************************************************
 * Name: virtio_9p_done
 ****************************************************************************/

static void virtio_9p_done(FAR struct virtqueue *vq)
{
  FAR struct virtio_9p_priv_s *priv = vq->vq_dev->priv;
  FAR struct v9fs_payload_s *payload;

  for (; ; )
    {
      payload = virtqueue_get_buffer_lock(vq, NULL, NULL, &priv->lock);
      if (payload == NULL)
        {
          break;
        }

      v9fs_transport_done(payload, 0);
    }
}

/****************************************************************************
 * Name: virtio_9p_probe
 ****************************************************************************/

static int virtio_9p_probe(FAR struct virtio_device *vdev)
{
  FAR struct virtio_9p_priv_s *priv = container_of(vdev->priv,
                                      struct virtio_9p_priv_s, vdrv);
  struct virtio_9p_config_s config;
  FAR const char *vqname[1];
  vq_callback callback[1];
  int ret;

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
  virtio_negotiate_features(vdev, 1 << VIRTIO_9P_MOUNT_TAG, NULL);
  virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);

  if (!virtio_has_feature(vdev, VIRTIO_9P_MOUNT_TAG))
    {
      virtio_reset_device(vdev);
      return -EINVAL;
    }

  virtio_read_config_member(vdev, struct virtio_9p_config_s, tag_len,
                            &config.tag_len);
  virtio_read_config_bytes(vdev, offsetof(struct virtio_9p_config_s, tag),
                           &config.tag, config.tag_len);
  config.tag[config.tag_len] = '\0';
  if (strcmp(config.tag, priv->tag))
    {
      virtio_reset_device(vdev);
      return -ENOENT;
    }

  vqname[0] = "virtio_9p_vq";
  callback[0] = virtio_9p_done;
  ret = virtio_create_virtqueues(vdev, 0, 1, vqname, callback, NULL);
  if (ret < 0)
    {
      vrterr("virtio_device_create_virtqueue failed, ret=%d\n", ret);
      virtio_reset_device(vdev);
      return ret;
    }

  priv->vdev = vdev;
  vdev->priv = priv;
  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);
  virtqueue_enable_cb(vdev->vrings_info[0].vq);
  return OK;
}

/****************************************************************************
 * Name: virtio_9p_remove
 ****************************************************************************/

static void virtio_9p_remove(FAR struct virtio_device *vdev)
{
  virtio_delete_virtqueues(vdev);
  virtio_reset_device(vdev);
}
