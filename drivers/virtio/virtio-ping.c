/****************************************************************************
 * drivers/virtio/virtio-ping.c
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

#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/virtio/virtio-ping.h>

#include "virtio-ping.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Alignment applied to the buffer pool */

#define VIRTIO_PING_ALIGN     16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A buffer is passed around as void *, so it reaches the free list as a node
 * and the wire as a header without a cast in either direction.  An idle
 * buffer carries a dq_entry_s at its start, which is safe because nothing
 * reads the header of a buffer nobody owns, and VIRTIO_PING_ALIGN keeps the
 * start aligned for the node.  Casting the packed header to the node type
 * instead would trip -Waddress-of-packed-member, which GCC also reports
 * through an intermediate (void *) or (uintptr_t).
 */

struct virtio_ping_priv_s
{
  FAR struct virtio_device *vdev;
  char                      name[VIRTIO_PING_NAMELEN];
  spinlock_t                lock;     /* Protects both virtqueues */
  FAR char                 *pool;     /* Every buffer, in one allocation */
  struct dq_queue_s         txfree;   /* Idle outbound buffers */
  sem_t                     txavail;  /* Counts the buffers on txfree */
};

/* Threaded through virtio_foreach_device() to the handler below */

struct virtio_ping_info_s
{
  FAR const struct virtio_ping_s *ping;
  bool                            all;   /* Every ping device was asked for */
  int                             count; /* Devices the name matched */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int virtio_ping_probe(FAR struct virtio_device *vdev);
static void virtio_ping_remove(FAR struct virtio_device *vdev);
static void virtio_ping_dump(FAR struct virtio_device *vdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct virtio_driver g_virtio_ping_driver =
{
  VIRTIO_ID_PING,     /* device id */
  virtio_ping_probe,  /* probe */
  virtio_ping_remove, /* remove */
  virtio_ping_dump,   /* dump */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_ping_add_buffer
 *
 * Description:
 *   Put one buffer on a queue as a single segment and kick.  A buffer the
 *   device fills is writable on the RX queue, one we filled is readable on
 *   the TX queue, which is what the queue index selects.
 *
 ****************************************************************************/

static inline_function int
virtio_ping_add_buffer(FAR struct virtio_ping_priv_s *priv,
                       unsigned int qidx, FAR void *buf, uint32_t len)
{
  FAR struct virtqueue *vq = priv->vdev->vrings_info[qidx].vq;
  struct virtqueue_buf vb;
  irqstate_t flags;
  int ret;

  vb.buf = buf;
  vb.len = len;

  /* The buffer is the cookie, so a completion hands the whole buffer back */

  flags = spin_lock_irqsave(&priv->lock);
  ret = virtqueue_add_buffer(vq, &vb, 1 - qidx, qidx, buf);
  if (ret >= 0)
    {
      virtqueue_kick(vq);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
  return ret;
}

/****************************************************************************
 * Name: virtio_ping_get_tx_buffer
 *
 * Description:
 *   Take an idle outbound buffer, reclaiming completed ones when the free
 *   list has run dry.  The callback side passes wait false, since it may not
 *   block; NULL then means none could be had.
 *
 ****************************************************************************/

static FAR void *
virtio_ping_get_tx_buffer(FAR struct virtio_ping_priv_s *priv, bool wait)
{
  FAR struct virtqueue *vq =
    priv->vdev->vrings_info[VIRTIO_PING_VQ_TX].vq;
  FAR struct dq_entry_s *node;
  irqstate_t flags;

  for (; ; )
    {
      flags = spin_lock_irqsave(&priv->lock);
      node = dq_remfirst(&priv->txfree);
      spin_unlock_irqrestore(&priv->lock, flags);

      if (node != NULL)
        {
          break;
        }

      /* Nothing idle, so reclaim whatever the device has finished with */

      node = virtqueue_get_buffer_lock(vq, NULL, NULL, &priv->lock);
      if (node != NULL)
        {
          break;
        }

      if (!wait)
        {
          return NULL;
        }

      /* Re-arm before sleeping.  Suppression is explicit here: the queue
       * is left unarmed while buffers are reclaimed, so it has to be armed
       * again before this side can rely on a notification.  A non zero
       * return means a buffer completed while the queue was unarmed, so
       * retry rather than sleep on it.
       */

      if (virtqueue_enable_cb_lock(vq, &priv->lock) != 0)
        {
          continue;
        }

      nxsem_wait_uninterruptible(&priv->txavail);
    }

  /* A buffer is in hand, so stop asking to be notified until this side runs
   * dry again and arms the queue right before sleeping.
   */

  virtqueue_disable_cb_lock(vq, &priv->lock);
  return node;
}

/****************************************************************************
 * Name: virtio_ping_put_tx_buffer
 ****************************************************************************/

static inline_function void
virtio_ping_put_tx_buffer(FAR struct virtio_ping_priv_s *priv,
                          FAR void *buf)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  dq_addlast(buf, &priv->txfree);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: virtio_ping_recv
 *
 * Description:
 *   Handle one message the peer sent us: a response releases the initiator
 *   waiting on the cookie, a request is answered with the cookie echoed
 *   back.  The answer needs a buffer of our own, the received one belongs to
 *   the peer's ring.
 *
 ****************************************************************************/

static void virtio_ping_recv(FAR struct virtio_ping_priv_s *priv,
                             FAR struct virtio_ping_hdr_s *hdr, uint32_t len)
{
  FAR struct virtio_ping_hdr_s *rsp;

  DEBUGASSERT(len >= VIRTIO_PING_HDRSIZE);

  if (hdr->cmd == VIRTIO_PING_CMD_RSP)
    {
      nxsem_post((FAR sem_t *)(uintptr_t)hdr->cookie);
      return;
    }

  rsp = virtio_ping_get_tx_buffer(priv, false);
  if (rsp == NULL)
    {
      vrtwarn("no buffer to reply on\n");
      return;
    }

  rsp->cmd    = VIRTIO_PING_CMD_RSP;
  rsp->cookie = hdr->cookie;

  if (virtio_ping_add_buffer(priv, VIRTIO_PING_VQ_TX, rsp,
                             VIRTIO_PING_HDRSIZE) < 0)
    {
      virtio_ping_put_tx_buffer(priv, rsp);
    }
}

/****************************************************************************
 * Name: virtio_ping_rxready
 *
 * Description:
 *   Inbound queue callback, taking peer requests and replies to our own.
 *
 *   The drain / re-arm loop is what suppresses notifications: the queue is
 *   left unarmed while draining, and virtqueue_enable_cb() re-arms it and
 *   reports whether more arrived.
 *
 ****************************************************************************/

static void virtio_ping_rxready(FAR struct virtqueue *vq)
{
  FAR struct virtio_ping_priv_s *priv = vq->vq_dev->priv;
  FAR void *buf;
  uint32_t len;

  virtqueue_disable_cb_lock(vq, &priv->lock);

  do
    {
      for (; ; )
        {
          buf = virtqueue_get_buffer_lock(vq, &len, NULL, &priv->lock);
          if (buf == NULL)
            {
              break;
            }

          virtio_ping_recv(priv, buf, len);

          /* The reply, if any, went out in a buffer of its own, so this one
           * goes straight back.
           */

          virtio_ping_add_buffer(priv, VIRTIO_PING_VQ_RX, buf,
                                 VIRTIO_PING_BUFLEN);
        }
    }
  while (virtqueue_enable_cb_lock(vq, &priv->lock) != 0);
}

/****************************************************************************
 * Name: virtio_ping_txdone
 *
 * Description:
 *   Outbound queue callback.  Buffers are claimed back in
 *   virtio_ping_get_tx_buffer, so nothing is drained here.
 *
 ****************************************************************************/

static void virtio_ping_txdone(FAR struct virtqueue *vq)
{
  FAR struct virtio_ping_priv_s *priv = vq->vq_dev->priv;
  int semcount = 0;

  /* Completed buffers are reclaimed by whoever needs one, so the only work
   * here is releasing an initiator that found none.  Capping the count at
   * one token keeps a buffer that completes just before the initiator sleeps
   * from being missed, without letting tokens pile up.
   */

  nxsem_get_value(&priv->txavail, &semcount);
  while (semcount++ < 1)
    {
      nxsem_post(&priv->txavail);
    }
}

/****************************************************************************
 * Name: virtio_ping_once
 *
 * Description:
 *   Send one request and wait for the peer to answer it.  The cookie is this
 *   semaphore, which the peer echoes, so the reply needs no state of ours to
 *   find its way back.
 *
 ****************************************************************************/

static int virtio_ping_once(FAR struct virtio_ping_priv_s *priv,
                            uint32_t len)
{
  FAR struct virtio_ping_hdr_s *hdr;
  sem_t sem;
  int ret;

  hdr = virtio_ping_get_tx_buffer(priv, true);
  if (hdr == NULL)
    {
      return -ENOMEM;
    }

  nxsem_init(&sem, 0, 0);

  hdr->cmd    = VIRTIO_PING_CMD_REQ;
  hdr->cookie = (uintptr_t)&sem;
  memset(hdr->data, 0, len - VIRTIO_PING_HDRSIZE + 1);

  ret = virtio_ping_add_buffer(priv, VIRTIO_PING_VQ_TX, hdr, len);
  if (ret >= 0)
    {
      nxsem_wait_uninterruptible(&sem);
    }
  else
    {
      /* No completion will arrive for a buffer that never went out */

      virtio_ping_put_tx_buffer(priv, hdr);
    }

  nxsem_destroy(&sem);
  return ret;
}

/****************************************************************************
 * Name: virtio_ping_run
 ****************************************************************************/

static int virtio_ping_run(FAR struct virtio_ping_priv_s *priv,
                           FAR const struct virtio_ping_s *ping)
{
  clock_t min = CLOCK_MAX;
  clock_t max = 0;
  uint64_t total = 0;
  int len;
  int ret;
  int i;

  /* Every buffer is VIRTIO_PING_BUFLEN on both sides, so that is the cap */

  len = MAX(ping->len, VIRTIO_PING_HDRSIZE);
  len = MIN(len, VIRTIO_PING_BUFLEN);

  for (i = 0; i < ping->times; i++)
    {
      clock_t tm = perf_gettime();

      ret = virtio_ping_once(priv, len);
      if (ret < 0)
        {
          return ret;
        }

      tm     = perf_gettime() - tm;
      min    = MIN(min, tm);
      max    = MAX(max, tm);
      total += tm;

      if (ping->sleep > 0)
        {
          nxsig_usleep(ping->sleep * USEC_PER_MSEC);
        }
    }

  _emerg("ping device: %s\n", priv->name);
  _emerg("ping times: %d, send_len: %d\n",
         ping->times, len);

  virtio_ping_logout("avg", total / ping->times);
  virtio_ping_logout("min", min);
  virtio_ping_logout("max", max);

  /* Bytes on the wire per round trip, reply header included */

  virtio_ping_logout_rate(len + VIRTIO_PING_HDRSIZE, total / ping->times);

  return OK;
}

/****************************************************************************
 * Name: virtio_ping_bufs_free
 ****************************************************************************/

static void virtio_ping_bufs_free(FAR struct virtio_ping_priv_s *priv)
{
  virtio_free_buf(priv->vdev, priv->pool);
  dq_init(&priv->txfree);
  priv->pool = NULL;
}

/****************************************************************************
 * Name: virtio_ping_bufs_alloc
 *
 * Description:
 *   Build every buffer this device will ever use in one allocation, sized by
 *   what the two queues can hold, then hand the inbound half to the device.
 *
 ****************************************************************************/

static int virtio_ping_bufs_alloc(FAR struct virtio_ping_priv_s *priv)
{
  FAR struct virtqueue *vq =
    priv->vdev->vrings_info[VIRTIO_PING_VQ_RX].vq;
  struct virtqueue_buf vb;
  FAR char *buf;
  uint16_t ntxbufs;
  uint16_t nrxbufs;
  uint16_t i;
  int ret;

  ntxbufs = priv->vdev->vrings_info[VIRTIO_PING_VQ_TX].vq->vq_free_cnt;
  nrxbufs = priv->vdev->vrings_info[VIRTIO_PING_VQ_RX].vq->vq_free_cnt;

  if (ntxbufs == 0 || nrxbufs == 0)
    {
      return -ENOSPC;
    }

  dq_init(&priv->txfree);

  priv->pool = virtio_zalloc_buf(priv->vdev,
                                 (ntxbufs + nrxbufs) *
                                 VIRTIO_PING_BUFLEN, VIRTIO_PING_ALIGN);
  if (priv->pool == NULL)
    {
      return -ENOMEM;
    }

  for (buf = priv->pool, i = 0; i < ntxbufs;
       i++, buf += VIRTIO_PING_BUFLEN)
    {
      dq_addlast((FAR struct dq_entry_s *)buf, &priv->txfree);
    }

  /* Counts nothing, it only parks an initiator that has run dry */

  nxsem_init(&priv->txavail, 0, 0);

  for (i = 0; i < nrxbufs; i++, buf += VIRTIO_PING_BUFLEN)
    {
      /* The buffer is the cookie, so a completion hands the whole buffer
       * back.  One kick covers the batch.
       */

      vb.buf = buf;
      vb.len = VIRTIO_PING_BUFLEN;

      ret = virtqueue_add_buffer(vq, &vb, 0, 1, buf);
      if (ret < 0)
        {
          nxsem_destroy(&priv->txavail);
          virtio_ping_bufs_free(priv);
          return ret;
        }
    }

  virtqueue_kick(vq);

  return OK;
}

/****************************************************************************
 * Name: virtio_ping_probe
 ****************************************************************************/

static int virtio_ping_probe(FAR struct virtio_device *vdev)
{
  FAR struct virtio_ping_priv_s *priv;
  FAR const char *vqnames[VIRTIO_PING_VQ_NUM];
  vq_callback callback[VIRTIO_PING_VQ_NUM];
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vrterr("No enough memory\n");
      return -ENOMEM;
    }

  spin_lock_init(&priv->lock);
  priv->vdev = vdev;
  vdev->priv = priv;

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);

  /* No features to negotiate: notifications are suppressed explicitly
   * through virtqueue_disable_cb() / virtqueue_enable_cb().  Still tell the
   * peer, rather than leaning on the transport having done it, so the
   * handshake holds whichever transport this device turns up on.
   */

  virtio_negotiate_features(vdev, 0, NULL);
  virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);

  vqnames[VIRTIO_PING_VQ_TX]  = "virtio_ping_tx";
  vqnames[VIRTIO_PING_VQ_RX]  = "virtio_ping_rx";
  callback[VIRTIO_PING_VQ_TX] = virtio_ping_txdone;
  callback[VIRTIO_PING_VQ_RX] = virtio_ping_rxready;
  ret = virtio_create_virtqueues(vdev, 0, VIRTIO_PING_VQ_NUM, vqnames,
                                 callback, NULL);
  if (ret < 0)
    {
      vrterr("virtio_create_virtqueues failed, ret=%d\n", ret);
      goto err_with_priv;
    }

  /* The name is how a user picks this device out of several, so a device
   * without one is rejected rather than falling back to an ordinal.
   */

  virtio_read_config_member(vdev, struct virtio_ping_config_s,
                            name, &priv->name);
  priv->name[VIRTIO_PING_NAMELEN - 1] = '\0';
  if (priv->name[0] == '\0')
    {
      vrterr("device published no name in its config space\n");
      ret = -EINVAL;
      goto err_with_virtqueue;
    }

  ret = virtio_ping_bufs_alloc(priv);
  if (ret < 0)
    {
      vrterr("buffer setup failed, ret=%d\n", ret);
      goto err_with_virtqueue;
    }

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);
  virtqueue_enable_cb(vdev->vrings_info[VIRTIO_PING_VQ_RX].vq);

  return OK;

err_with_virtqueue:
  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
err_with_priv:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: virtio_ping_remove
 *
 * Description:
 *   Reached through virtio_unregister_device(), which holds the virtio bus
 *   lock.  virtio_foreach_device() holds that same lock for the whole walk,
 *   so a remove racing against a running ping blocks until it finishes.
 *
 ****************************************************************************/

static void virtio_ping_remove(FAR struct virtio_device *vdev)
{
  FAR struct virtio_ping_priv_s *priv = vdev->priv;

  virtqueue_disable_cb(vdev->vrings_info[VIRTIO_PING_VQ_RX].vq);
  virtqueue_disable_cb(vdev->vrings_info[VIRTIO_PING_VQ_TX].vq);
  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
  nxsem_destroy(&priv->txavail);
  virtio_ping_bufs_free(priv);
  kmm_free(priv);
}

/****************************************************************************
 * Name: virtio_ping_dump
 ****************************************************************************/

static void virtio_ping_dump(FAR struct virtio_device *vdev)
{
  FAR struct virtio_ping_priv_s *priv = vdev->priv;

  _emerg("    ping: name=%s txbufs=%" PRIu16 " rxbufs=%" PRIu16 "\n",
         priv->name,
         vdev->vrings_info[VIRTIO_PING_VQ_TX].vq->vq_nentries,
         vdev->vrings_info[VIRTIO_PING_VQ_RX].vq->vq_nentries);
}

/****************************************************************************
 * Name: virtio_ping_handler
 *
 * Description:
 *   Called by virtio_foreach_device() once per ping device on the bus.
 *
 ****************************************************************************/

static int virtio_ping_handler(FAR struct virtio_device *vdev, FAR void *arg)
{
  FAR struct virtio_ping_info_s *info = arg;
  FAR struct virtio_ping_priv_s *priv = vdev->priv;

  if (!info->all &&
      strncmp(priv->name, info->ping->name, VIRTIO_PING_NAMELEN) != 0)
    {
      return OK;
    }

  info->count++;
  return virtio_ping_run(priv, info->ping);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_ping
 *
 * Description:
 *   Run a ping measurement against the device named by ping->name, or every
 *   ping device when that name is "all".  Called from the VIRTIOIOC_PING
 *   handler in virtio.c.  The device list walked is the bus's own, through
 *   virtio_foreach_device(), so this driver keeps no list of its own.
 *
 ****************************************************************************/

int virtio_ping(FAR const struct virtio_ping_s *ping)
{
  struct virtio_ping_info_s info;
  int ret;

  if (ping == NULL || ping->times <= 0 || ping->len < 0 ||
      ping->name[0] == '\0')
    {
      return -EINVAL;
    }

  info.ping  = ping;
  info.all   = strncmp(ping->name, "all", VIRTIO_PING_NAMELEN) == 0;
  info.count = 0;

  ret = virtio_foreach_device(VIRTIO_ID_PING, virtio_ping_handler, &info);
  if (ret >= 0 && info.count == 0)
    {
      ret = -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_register_ping_driver
 ****************************************************************************/

int virtio_register_ping_driver(void)
{
  return virtio_register_driver(&g_virtio_ping_driver);
}
