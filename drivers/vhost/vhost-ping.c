/****************************************************************************
 * drivers/vhost/vhost-ping.c
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
#include <nuttx/percpu.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>
#include <nuttx/vhost/vhost.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/virtio/virtio-ping.h>

#include "vhost-ping.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vhost_ping_priv_s
{
  FAR struct vhost_device *hdev;
  char                     name[VIRTIO_PING_NAMELEN];
  spinlock_t               lock;      /* Protects both virtqueues */

  /* Posted when the driver offers fresh outbound buffers, waited on by an
   * initiator that finds none.
   */

  sem_t                    txbufs;
};

/* Threaded through vhost_foreach_device() to the handler below */

struct vhost_ping_info_s
{
  FAR const struct vhost_ping_s *ping;
  bool                           all;   /* Every ping device was asked for */
  int                            count; /* Devices the name matched */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int vhost_ping_probe(FAR struct vhost_device *hdev);
static void vhost_ping_remove(FAR struct vhost_device *hdev);
static void vhost_ping_dump(FAR struct vhost_device *hdev);

static void vhost_ping_rxready(FAR struct virtqueue *vq);
static void vhost_ping_txdone(FAR struct virtqueue *vq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static DEFINE_PER_CPU_BMP(struct vhost_driver, g_vhost_ping_driver) =
{
  VIRTIO_ID_PING,    /* Device id */
  vhost_ping_probe,  /* Probe */
  vhost_ping_remove, /* Remove */
  vhost_ping_dump,   /* Dump */
};
#define g_vhost_ping_driver this_cpu_var_bmp(g_vhost_ping_driver)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_ping_add_buffer
 *
 * Description:
 *   Return a consumed descriptor to the driver and kick.  One lock for both
 *   calls, the locked helpers would take it twice.
 *
 ****************************************************************************/

static void vhost_ping_add_buffer(FAR struct vhost_ping_priv_s *priv,
                                  unsigned int qidx, uint16_t head,
                                  uint32_t len)
{
  FAR struct virtqueue *vq = priv->hdev->vrings_info[qidx].vq;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  virtqueue_add_consumed_buffer(vq, head, len);
  virtqueue_kick(vq);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: vhost_ping_get_tx_buffer
 *
 * Description:
 *   Take one of the writable buffers the driver pre-posted, the only way
 *   this side can put a message on the wire: a device may not produce into
 *   an available ring.  The callback side passes wait false, since it may
 *   not block; NULL then means the driver has offered none.
 *
 ****************************************************************************/

static FAR struct vhost_ping_hdr_s *
vhost_ping_get_tx_buffer(FAR struct vhost_ping_priv_s *priv,
                         FAR uint32_t *len, FAR uint16_t *head, bool wait)
{
  FAR struct virtqueue *vq =
    priv->hdev->vrings_info[VHOST_PING_VQ_TX].vq;
  FAR void *buf;

  for (; ; )
    {
      buf = virtqueue_get_available_buffer_lock(vq, head, len, &priv->lock);
      if (buf != NULL)
        {
          /* A buffer is in hand, so stop asking to be notified until
           * this side runs dry again.
           */

          virtqueue_disable_cb_lock(vq, &priv->lock);
          break;
        }

      if (!wait)
        {
          return NULL;
        }

      /* Re-arm before sleeping.  Suppression is explicit here: the queue
       * is left unarmed while buffers are claimed, so it has to be armed
       * again before this side can rely on the driver kicking it.  A non
       * zero return means buffers arrived while the queue was unarmed, so
       * retry rather than sleep on them.
       */

      if (virtqueue_enable_cb_lock(vq, &priv->lock) != 0)
        {
          continue;
        }

      nxsem_wait_uninterruptible(&priv->txbufs);
    }

  DEBUGASSERT(*len >= VHOST_PING_HDRSIZE);

  return buf;
}

/****************************************************************************
 * Name: vhost_ping_get_rx_buffer
 *
 * Description:
 *   Take the next request or reply the peer produced.  NULL means the queue
 *   is empty.
 *
 ****************************************************************************/

static FAR struct vhost_ping_hdr_s *
vhost_ping_get_rx_buffer(FAR struct vhost_ping_priv_s *priv,
                         FAR uint16_t *head)
{
  FAR struct virtqueue *vq =
    priv->hdev->vrings_info[VHOST_PING_VQ_RX].vq;
  FAR void *buf;
  uint32_t len;

  buf = virtqueue_get_available_buffer_lock(vq, head, &len, &priv->lock);
  if (buf == NULL)
    {
      return NULL;
    }

  DEBUGASSERT(len >= VHOST_PING_HDRSIZE);

  return buf;
}

/****************************************************************************
 * Name: vhost_ping_recv
 *
 * Description:
 *   Handle one message the peer sent us: a response releases the initiator
 *   waiting on the cookie, a request is answered with the cookie echoed
 *   back.  The answer goes out in a buffer of the driver's, so it needs one
 *   of its own.
 *
 ****************************************************************************/

static void vhost_ping_recv(FAR struct vhost_ping_priv_s *priv,
                            FAR struct vhost_ping_hdr_s *hdr)
{
  FAR struct vhost_ping_hdr_s *rsp;
  uint32_t rsplen;
  uint16_t rsphead;

  if (hdr->cmd == VIRTIO_PING_CMD_RSP)
    {
      nxsem_post((FAR sem_t *)(uintptr_t)hdr->cookie);
      return;
    }

  rsp = vhost_ping_get_tx_buffer(priv, &rsplen, &rsphead, false);
  if (rsp == NULL)
    {
      vhostwarn("no buffer to reply on\n");
      return;
    }

  rsp->cmd    = VIRTIO_PING_CMD_RSP;
  rsp->cookie = hdr->cookie;

  vhost_ping_add_buffer(priv, VHOST_PING_VQ_TX, rsphead,
                        VHOST_PING_HDRSIZE);
}

/****************************************************************************
 * Name: vhost_ping_rxready
 *
 * Description:
 *   Peer outbound queue callback, taking peer requests and replies to our
 *   own.
 *
 *   The drain / re-arm loop is what suppresses notifications: the queue is
 *   left unarmed while draining, and virtqueue_enable_cb() re-arms it and
 *   reports whether more arrived.
 *
 ****************************************************************************/

static void vhost_ping_rxready(FAR struct virtqueue *vq)
{
  FAR struct vhost_ping_priv_s *priv = vq->vq_dev->priv;
  FAR struct vhost_ping_hdr_s *hdr;
  uint16_t head;

  virtqueue_disable_cb_lock(vq, &priv->lock);

  do
    {
      for (; ; )
        {
          hdr = vhost_ping_get_rx_buffer(priv, &head);
          if (hdr == NULL)
            {
              break;
            }

          vhost_ping_recv(priv, hdr);

          /* Returned only after the payload is consumed, the buffer
           * belongs to the peer.
           */

          vhost_ping_add_buffer(priv, VHOST_PING_VQ_RX, head, 0);
        }
    }
  while (virtqueue_enable_cb_lock(vq, &priv->lock) != 0);
}

/****************************************************************************
 * Name: vhost_ping_txdone
 *
 * Description:
 *   The driver posted fresh outbound buffers.  They are claimed on demand,
 *   so the only work is to wake an initiator that found none.
 *
 ****************************************************************************/

static void vhost_ping_txdone(FAR struct virtqueue *vq)
{
  FAR struct vhost_ping_priv_s *priv = vq->vq_dev->priv;
  int semcount = 0;

  /* Capping the count at one token releases every initiator that is already
   * parked, and keeps a batch that arrives just before one of them sleeps
   * from being missed, without letting tokens pile up.
   */

  nxsem_get_value(&priv->txbufs, &semcount);
  while (semcount++ < 1)
    {
      nxsem_post(&priv->txbufs);
    }
}

/****************************************************************************
 * Name: vhost_ping_once
 *
 * Description:
 *   Send one request and wait for the peer to answer it.  The cookie is this
 *   semaphore, which the peer echoes, so the reply needs no state of ours to
 *   find its way back.
 *
 ****************************************************************************/

static int vhost_ping_once(FAR struct vhost_ping_priv_s *priv, uint32_t len)
{
  FAR struct vhost_ping_hdr_s *hdr;
  uint32_t offered;
  uint16_t head;
  sem_t sem;

  hdr = vhost_ping_get_tx_buffer(priv, &offered, &head, true);
  if (hdr == NULL)
    {
      return -ENOMEM;
    }

  /* Whatever room the driver offered is the upper bound here */

  len = MIN(len, offered);

  nxsem_init(&sem, 0, 0);

  hdr->cmd    = VIRTIO_PING_CMD_REQ;
  hdr->cookie = (uintptr_t)&sem;
  memset(hdr->data, 0, len - VHOST_PING_HDRSIZE + 1);

  vhost_ping_add_buffer(priv, VHOST_PING_VQ_TX, head, len);
  nxsem_wait_uninterruptible(&sem);
  nxsem_destroy(&sem);

  return len;
}

/****************************************************************************
 * Name: vhost_ping_run
 ****************************************************************************/

static int vhost_ping_run(FAR struct vhost_ping_priv_s *priv,
                          FAR const struct vhost_ping_s *ping)
{
  clock_t min = CLOCK_MAX;
  clock_t max = 0;
  uint64_t total = 0;
  uint32_t len;
  int sent = 0;
  int i;

  /* Lower bound only, the upper one is whatever the driver offered and is
   * applied by vhost_ping_once.
   */

  len = MAX(ping->len, VHOST_PING_HDRSIZE);

  for (i = 0; i < ping->times; i++)
    {
      clock_t tm = perf_gettime();

      sent = vhost_ping_once(priv, len);
      if (sent < 0)
        {
          return sent;
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
         ping->times, sent);

  vhost_ping_logout("avg", total / ping->times);
  vhost_ping_logout("min", min);
  vhost_ping_logout("max", max);

  /* Bytes on the wire per round trip, reply header included */

  vhost_ping_logout_rate(sent + VHOST_PING_HDRSIZE, total / ping->times);

  return OK;
}

/****************************************************************************
 * Name: vhost_ping_probe
 ****************************************************************************/

static int vhost_ping_probe(FAR struct vhost_device *hdev)
{
  FAR struct vhost_ping_priv_s *priv;
  FAR const char *vqnames[VHOST_PING_VQ_NUM];
  vq_callback callback[VHOST_PING_VQ_NUM];
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vhosterr("No enough memory\n");
      return -ENOMEM;
    }

  spin_lock_init(&priv->lock);

  /* Before the queues exist: vhost_ping_txdone() posts to it and may run as
   * soon as a callback is armed.
   */

  nxsem_init(&priv->txbufs, 0, 0);
  priv->hdev = hdev;
  hdev->priv = priv;

  /* The name is how a user picks one device out of several */

  vhost_read_config_member(hdev, struct vhost_ping_config_s,
                           name, &priv->name);
  priv->name[VIRTIO_PING_NAMELEN - 1] = '\0';
  if (priv->name[0] == '\0')
    {
      vhosterr("no name in config space\n");
      ret = -EINVAL;
      goto err_with_priv;
    }

  /* The names must match the driver's, the roles are mirrored: this side
   * receives on what the driver calls TX.
   */

  vqnames[VHOST_PING_VQ_RX]  = "virtio_ping_tx";
  vqnames[VHOST_PING_VQ_TX]  = "virtio_ping_rx";
  callback[VHOST_PING_VQ_RX] = vhost_ping_rxready;
  callback[VHOST_PING_VQ_TX] = vhost_ping_txdone;
  ret = vhost_create_virtqueues(hdev, 0, VHOST_PING_VQ_NUM, vqnames,
                                callback, NULL);
  if (ret < 0)
    {
      vhosterr("vhost_create_virtqueues failed, ret=%d\n", ret);
      goto err_with_priv;
    }

  virtqueue_enable_cb(hdev->vrings_info[VHOST_PING_VQ_RX].vq);

  return OK;

err_with_priv:
  nxsem_destroy(&priv->txbufs);
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: vhost_ping_remove
 *
 * Description:
 *   Reached through vhost_unregister_device(), which holds the vhost bus
 *   lock.  vhost_foreach_device() holds that same lock for the whole walk,
 *   so a remove racing against a running ping blocks until it finishes.
 *
 ****************************************************************************/

static void vhost_ping_remove(FAR struct vhost_device *hdev)
{
  FAR struct vhost_ping_priv_s *priv = hdev->priv;

  virtqueue_disable_cb(hdev->vrings_info[VHOST_PING_VQ_RX].vq);
  virtqueue_disable_cb(hdev->vrings_info[VHOST_PING_VQ_TX].vq);
  vhost_delete_virtqueues(hdev);
  nxsem_destroy(&priv->txbufs);
  kmm_free(priv);
}

/****************************************************************************
 * Name: vhost_ping_dump
 ****************************************************************************/

static void vhost_ping_dump(FAR struct vhost_device *hdev)
{
  FAR struct vhost_ping_priv_s *priv = hdev->priv;

  _emerg("    ping: name=%s\n", priv->name);
}

/****************************************************************************
 * Name: vhost_ping_handler
 *
 * Description:
 *   Called by vhost_foreach_device() once per ping device on the bus.
 *
 ****************************************************************************/

static int vhost_ping_handler(FAR struct vhost_device *hdev, FAR void *arg)
{
  FAR struct vhost_ping_info_s *info = arg;
  FAR struct vhost_ping_priv_s *priv = hdev->priv;

  if (!info->all &&
      strncmp(priv->name, info->ping->name, VIRTIO_PING_NAMELEN) != 0)
    {
      return OK;
    }

  info->count++;
  return vhost_ping_run(priv, info->ping);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_ping
 *
 * Description:
 *   Run a ping measurement against the device named by ping->name, or every
 *   ping device when that name is "all", since /dev/vhost is a single global
 *   node with no per device context.  The device list walked is the bus's
 *   own, through vhost_foreach_device(), so this driver keeps no list.
 *
 ****************************************************************************/

int vhost_ping(FAR const struct vhost_ping_s *ping)
{
  struct vhost_ping_info_s info;
  int ret;

  if (ping == NULL || ping->times <= 0 || ping->len < 0 ||
      ping->name[0] == '\0')
    {
      return -EINVAL;
    }

  info.ping  = ping;
  info.all   = strncmp(ping->name, "all", VIRTIO_PING_NAMELEN) == 0;
  info.count = 0;

  ret = vhost_foreach_device(VIRTIO_ID_PING, vhost_ping_handler, &info);
  if (ret >= 0 && info.count == 0)
    {
      ret = -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: vhost_register_ping_driver
 ****************************************************************************/

int vhost_register_ping_driver(void)
{
  return vhost_register_driver(&g_vhost_ping_driver);
}
