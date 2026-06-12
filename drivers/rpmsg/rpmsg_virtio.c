/****************************************************************************
 * drivers/rpmsg/rpmsg_virtio.c
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

#include <nuttx/config.h>

#include <nuttx/debug.h>
#include <stdio.h>
#include <stdbool.h>
#include <syslog.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/power/pm.h>
#include <nuttx/rpmsg/rpmsg_virtio.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/virtio/virtio-config.h>
#include <nuttx/wdog.h>
#include <openamp/rpmsg_virtio.h>

#include "rpmsg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSG_VIRTIO_TIMEOUT_MS      20

/* The tx buffer acquisition gives up after the buffer has been
 * unavailable for RPMSG_VIRTIO_TIMEOUTTX_COUNT consecutive timeout slices,
 * i.e. ~15s (RPMSG_TICK_COUNT usec / per-slice wait time).
 */

#define RPMSG_VIRTIO_TIMEOUTTX_COUNT \
  (RPMSG_TICK_COUNT / (RPMSG_VIRTIO_TIMEOUT_MS * USEC_PER_MSEC))

#define RPMSG_VIRTIO_FEATURES        (1 << VIRTIO_RPMSG_F_NS | \
                                      1 << VIRTIO_RPMSG_F_ACK | \
                                      1 << VIRTIO_RPMSG_F_BUFSZ | \
                                      1 << VIRTIO_RPMSG_F_CPUNAME | \
                                      1 << VIRTIO_RPMSG_F_BUFADDR | \
                                      1 << VIRTIO_RPMSG_F_PRIORITY)

#ifdef CONFIG_OPENAMP_CACHE
#  define RPMSG_VIRTIO_INVALIDATE(x) metal_cache_invalidate(&x, sizeof(x))
#else
#  define RPMSG_VIRTIO_INVALIDATE(x)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsg_virtio_priv_s
{
  struct rpmsg_s                  rpmsg;
  struct rpmsg_virtio_device      rvdev;
  struct rpmsg_virtio_shm_pool    pool[2];
  FAR struct virtio_device       *vdev;
  sem_t                           semrx;
  sem_t                           semtx;
  vq_notify                       notifytx;
  uint16_t                        headrx;
  uint16_t                        headtx;
  int                             sleepertx;
  int                             timeouttx;  /* Consecutive tx wait slices */
  struct work_s                   startwork;
#ifdef CONFIG_RPMSG_VIRTIO_PM
  spinlock_t                      lock;
  struct pm_wakelock_s            wakelock;
  struct wdog_s                   wdog;
#endif
  bool                            init;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpmsg_virtio_wait(FAR struct rpmsg_s *rpmsg, clock_t delay);
static int rpmsg_virtio_post(FAR struct rpmsg_s *rpmsg);
static void rpmsg_virtio_dump(FAR struct rpmsg_s *rpmsg, bool verbose);
static FAR void *rpmsg_virtio_alloc_buf(FAR struct rpmsg_s *rpmsg,
                                        size_t size, size_t align);
static void rpmsg_virtio_free_buf(FAR struct rpmsg_s *rpmsg,
                                  FAR void *addr);

static void rpmsg_virtio_rx_callback(FAR struct virtqueue *vq);
static void rpmsg_virtio_tx_callback(FAR struct virtqueue *vq);
static void rpmsg_virtio_tx_notify(FAR struct virtqueue *vq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_virtio_buffer_nused
 ****************************************************************************/

static uint16_t
rpmsg_virtio_buffer_nused(FAR struct rpmsg_virtio_device *rvdev, bool rx)
{
  FAR struct virtqueue *vq = rx ? rvdev->rvq : rvdev->svq;
  uint16_t nused;

  if (VIRTIO_ROLE_IS_DRIVER(rvdev->vdev))
    {
      RPMSG_VIRTIO_INVALIDATE(vq->vq_ring.used->idx);
    }
  else if (VIRTIO_ROLE_IS_DEVICE(rvdev->vdev))
    {
      RPMSG_VIRTIO_INVALIDATE(vq->vq_ring.avail->idx);
    }

  nused = vq->vq_ring.avail->idx - vq->vq_ring.used->idx;
  if (VIRTIO_ROLE_IS_DRIVER(rvdev->vdev) == rx)
    {
      nused = vq->vq_nentries - nused;
    }

  return nused;
}

/****************************************************************************
 * Name: rpmsg_virtio_pm_callback
 ****************************************************************************/

#ifdef CONFIG_RPMSG_VIRTIO_PM_AUTORELAX
static void rpmsg_virtio_pm_callback(wdparm_t arg)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    (FAR struct rpmsg_virtio_priv_s *)arg;
  irqstate_t flags;

  flags = spin_lock_irqsave_nopreempt(&priv->lock);

  if (rpmsg_virtio_buffer_nused(&priv->rvdev, false))
    {
      wd_start(&priv->wdog, MSEC2TICK(RPMSG_VIRTIO_TIMEOUT_MS),
               rpmsg_virtio_pm_callback, (wdparm_t)priv);
    }
  else
    {
      pm_wakelock_relax(&priv->wakelock);
    }

  spin_unlock_irqrestore_nopreempt(&priv->lock, flags);
}
#endif

#ifdef CONFIG_RPMSG_VIRTIO_PM

/****************************************************************************
 * Name: rpmsg_virtio_pm_action
 ****************************************************************************/

static inline void
rpmsg_virtio_pm_action(FAR struct rpmsg_virtio_priv_s *priv, bool stay)
{
  irqstate_t flags;
  int count;

  flags = spin_lock_irqsave_nopreempt(&priv->lock);

  count = pm_wakelock_staycount(&priv->wakelock);
  if (stay && count == 0)
    {
      pm_wakelock_stay(&priv->wakelock);
#ifdef CONFIG_RPMSG_VIRTIO_PM_AUTORELAX
      wd_start(&priv->wdog, MSEC2TICK(RPMSG_VIRTIO_TIMEOUT_MS),
               rpmsg_virtio_pm_callback, (wdparm_t)priv);
#endif
    }

#ifndef CONFIG_RPMSG_VIRTIO_PM_AUTORELAX
  /* When enabled the CONFIG_RPMSG_VIRTIO_PM_AUTORELAX, use a timer to check
   * the buffers periodically and relax the pm wakelock and do not use this
   * logic.
   */

  if (!stay && count > 0 &&
      rpmsg_virtio_buffer_nused(&priv->rvdev, false) == 0u)
    {
      pm_wakelock_relax(&priv->wakelock);
    }
#endif

  spin_unlock_irqrestore_nopreempt(&priv->lock, flags);
}

#else
#  define rpmsg_virtio_pm_action(priv, stay)
#endif

/****************************************************************************
 * Name: rpmsg_virtio_process_rx_buffer
 *
 * Note: this function must be called with the rdev->lock held
 ****************************************************************************/

static inline_function int
rpmsg_virtio_process_rx_buffer(FAR struct rpmsg_device *rdev,
                               FAR struct rpmsg_endpoint *ept,
                               FAR struct rpmsg_hdr *hdr)
{
  size_t len = hdr->len + sizeof(*hdr);
  int status;

  if (ept->dest_addr == RPMSG_ADDR_ANY)
    {
      /* First message received from the remote side,
       * update channel destination address
       */

      ept->dest_addr = hdr->src;
    }

  rpmsg_ept_incref(ept);
  metal_mutex_release(&rdev->lock);

  rpmsg_trace(ept->name, false, hdr, len,
              "[Virtio] rx ept->cb start ept:%p name:%s cb:%p "
              "rdev:%p hdr:%p len:%zu\n",
              ept, ept->name, ept->cb, rdev, hdr, len);

  status = ept->cb(ept, RPMSG_LOCATE_DATA(hdr), hdr->len, hdr->src,
                   ept->priv);

  rpmsg_trace(ept->name, false, NULL, 0,
              "[Virtio] rx ept->cb end ept:%p name:%s cb:%p "
              "rdev:%p hdr:%p len:%zu\n",
              ept, ept->name, ept->cb, rdev, hdr, len);

  RPMSG_ASSERT(status >= 0 || status == RPMSG_SUCCESS_BUFFER_RELEASED,
               "unexpected callback status\r\n");

  metal_mutex_acquire(&rdev->lock);
  rpmsg_ept_decref(ept);
  return status;
}

/****************************************************************************
 * Name: rpmsg_virtio_rx_worker
 ****************************************************************************/

static void rpmsg_virtio_rx_worker(FAR struct rpmsg_s *rpmsg, FAR void *arg)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    (FAR struct rpmsg_virtio_priv_s *)rpmsg;
  FAR struct virtio_device *vdev = priv->vdev;
  FAR struct rpmsg_virtio_device *rvdev = vdev->priv;
  FAR struct rpmsg_device *rdev = &rvdev->rdev;
  FAR struct rpmsg_hdr *hdr = arg;
  FAR struct rpmsg_endpoint *ept;
  int status = RPMSG_SUCCESS;

  metal_mutex_acquire(&rdev->lock);
  ept = rpmsg_get_ept_from_addr(rdev, hdr->dst);
  if (ept)
    {
      status = rpmsg_virtio_process_rx_buffer(rdev, ept, hdr);
    }

  if (status != RPMSG_SUCCESS_BUFFER_RELEASED &&
      rpmsg_virtio_buf_held_dec_test(hdr))
    {
      rpmsg_virtio_release_rx_buffer_nolock(rvdev, hdr);
      virtqueue_kick(rvdev->rvq);
    }

  metal_mutex_release(&rdev->lock);
}

/****************************************************************************
 * Name: rpmsg_virtio_wait
 ****************************************************************************/

static int rpmsg_virtio_wait(FAR struct rpmsg_s *rpmsg, clock_t delay)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    (FAR struct rpmsg_virtio_priv_s *)rpmsg;

  if (delay > MSEC2TICK(RPMSG_VIRTIO_TIMEOUT_MS))
    {
      delay = MSEC2TICK(RPMSG_VIRTIO_TIMEOUT_MS);
    }

  return nxsem_tickwait(&priv->semtx, delay);
}

/****************************************************************************
 * Name: rpmsg_virtio_post
 ****************************************************************************/

static int rpmsg_virtio_post(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    (FAR struct rpmsg_virtio_priv_s *)rpmsg;
  int semcount = 0;

  nxsem_get_value(&priv->semtx, &semcount);
  while (semcount++ < 1)
    {
      nxsem_post(&priv->semtx);
    }

  return OK;
}

/****************************************************************************
 * Name: rpmsg_virtio_dump_buffer
 ****************************************************************************/

static void rpmsg_virtio_dump_buffer(FAR struct rpmsg_virtio_device *rvdev,
                                     bool rx)
{
  FAR struct virtqueue *vq = rx ? rvdev->rvq : rvdev->svq;
  FAR void *addr;
  uint16_t unretrieved = 0;
  uint16_t desc_idx;
  uint16_t num;
  uint16_t i;

  num = rpmsg_virtio_buffer_nused(rvdev, rx);

  if (VIRTIO_ROLE_IS_DRIVER(rvdev->vdev))
    {
      RPMSG_VIRTIO_INVALIDATE(vq->vq_ring.used->idx);
      unretrieved = vq->vq_ring.used->idx - vq->vq_used_cons_idx;
    }
  else if (VIRTIO_ROLE_IS_DEVICE(rvdev->vdev))
    {
      RPMSG_VIRTIO_INVALIDATE(vq->vq_ring.avail->idx);
      unretrieved = vq->vq_ring.avail->idx - vq->vq_available_idx;
    }

  if (rx)
    {
      syslog(LOG_EMERG, "    RX buffer total %u\n", vq->vq_nentries);
      syslog(LOG_EMERG, "      unretrieved %u\n", unretrieved);
      syslog(LOG_EMERG, "      retrieved %u\n", vq->vq_queued_cnt);
      syslog(LOG_EMERG, "      pending %u:\n", num);
    }
  else
    {
      syslog(LOG_EMERG, "    TX buffer total %u\n", vq->vq_nentries);
      syslog(LOG_EMERG, "      unretrieved %u\n", unretrieved);
      syslog(LOG_EMERG, "      retrieved %u\n", vq->vq_free_cnt);
      syslog(LOG_EMERG, "      sent %u:\n", num);
    }

  for (i = 0u; i < num; i++)
    {
      if (VIRTIO_ROLE_IS_DRIVER(rvdev->vdev) ^ rx)
        {
          RPMSG_VIRTIO_INVALIDATE(vq->vq_ring.used->idx);
          desc_idx = (vq->vq_ring.used->idx + i) & (vq->vq_nentries - 1u);
          RPMSG_VIRTIO_INVALIDATE(vq->vq_ring.avail->ring[desc_idx]);
          desc_idx = vq->vq_ring.avail->ring[desc_idx];
        }
      else
        {
          RPMSG_VIRTIO_INVALIDATE(vq->vq_ring.avail->idx);
          desc_idx = (vq->vq_ring.avail->idx + i) & (vq->vq_nentries - 1u);
          RPMSG_VIRTIO_INVALIDATE(vq->vq_ring.used->ring[desc_idx].id);
          desc_idx = vq->vq_ring.used->ring[desc_idx].id;
        }

      /* Validate desc_idx that was just read from the ring contents.
       * The ring is shared with the peer core and may contain stale or
       * corrupted indices (for example when the peer panicked in the
       * middle of an in-flight ring update).  Indexing vq_ring.desc[]
       * out of bounds would land on the avail/used regions whose
       * alignment is only 2 bytes; reading the 64-bit desc.addr field
       * there trips UBSan's alignment check inside this panic-dump
       * path.  Skip such invalid entries instead.
       */

      if (desc_idx >= vq->vq_nentries)
        {
          syslog(LOG_EMERG, "    %s buffer[%u] invalid desc_idx %u >= %u\n",
                 rx ? "RX" : "TX", i, desc_idx, vq->vq_nentries);
          continue;
        }

      addr = metal_io_phys_to_virt(vq->shm_io,
                                   vq->vq_ring.desc[desc_idx].addr);
      if (addr)
        {
          FAR struct rpmsg_hdr *hdr = addr;
          FAR struct rpmsg_endpoint *ept;

          ept = rpmsg_get_ept_from_addr(&rvdev->rdev,
                                        rx ? hdr->dst : hdr->src);
          if (ept)
            {
              syslog(LOG_EMERG, "        %s buffer %p hold by %s\n",
                     rx ? "RX" : "TX", hdr, ept->name);
            }
        }
    }
}

/****************************************************************************
 * Name: rpmsg_virtio_dump
 ****************************************************************************/

static void rpmsg_virtio_dump(FAR struct rpmsg_s *rpmsg, bool verbose)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    (FAR struct rpmsg_virtio_priv_s *)rpmsg;
  FAR struct rpmsg_virtio_device *rvdev = &priv->rvdev;
  FAR struct rpmsg_device *rdev = &rvdev->rdev;

  syslog(LOG_EMERG, "Local: %s Remote: %s Headrx %u Headtx %u\n",
         priv->rpmsg.local_cpuname, priv->rpmsg.cpuname, priv->headrx,
         priv->headtx);

  if (rvdev->vdev && rdev->ns_unbind_cb)
    {
      metal_mutex_acquire(&rdev->lock);
      if (verbose)
        {
          syslog(LOG_EMERG,
                 "Dump rpmsg info between cpu (master: %s)%s <==> %s:\n",
                 VIRTIO_ROLE_IS_DRIVER(rvdev->vdev) ? "yes" : "no",
                 priv->rpmsg.local_cpuname, priv->rpmsg.cpuname);

          rpmsg_dump_epts(rdev);
        }

      syslog(LOG_EMERG, "rpmsg vq RX:\n");
      virtqueue_dump(rvdev->rvq);
      syslog(LOG_EMERG, "rpmsg vq TX:\n");
      virtqueue_dump(rvdev->svq);

      if (verbose)
        {
          syslog(LOG_EMERG, "  rpmsg buffer list:\n");
          rpmsg_virtio_dump_buffer(rvdev, true);
          rpmsg_virtio_dump_buffer(rvdev, false);
        }

      metal_mutex_release(&rdev->lock);
    }
}

/****************************************************************************
 * Name: rpmsg_virtio_alloc_buf
 ****************************************************************************/

static FAR void *rpmsg_virtio_alloc_buf(FAR struct rpmsg_s *rpmsg,
                                        size_t size, size_t align)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    (FAR struct rpmsg_virtio_priv_s *)rpmsg;
  FAR void *buf = NULL;

  if (VIRTIO_ROLE_IS_DRIVER(priv->vdev))
    {
      buf = virtio_malloc_buf(priv->vdev, size, align);
    }
  else if (VIRTIO_ROLE_IS_DEVICE(priv->vdev))
    {
      buf = rpmsg_heap_alloc(&rpmsg->heap, size, align);
    }

  return buf;
}

/****************************************************************************
 * Name: rpmsg_virtio_free_buf
 ****************************************************************************/

static void rpmsg_virtio_free_buf(FAR struct rpmsg_s *rpmsg,
                                  FAR void *addr)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    (FAR struct rpmsg_virtio_priv_s *)rpmsg;

  if (VIRTIO_ROLE_IS_DRIVER(priv->vdev))
    {
      virtio_free_buf(priv->vdev, addr);
    }
  else if (VIRTIO_ROLE_IS_DEVICE(priv->vdev))
    {
      rpmsg_heap_free(&rpmsg->heap, addr);
    }
}

/****************************************************************************
 * Name: rpmsg_virtio_rx_update
 ****************************************************************************/

static void rpmsg_virtio_rx_update(FAR struct rpmsg_virtio_priv_s *priv,
                                   FAR struct virtqueue *rvq)
{
  FAR struct rpmsg_virtio_device *rvdev = &priv->rvdev;

  if (VIRTIO_ROLE_IS_DRIVER(rvdev->vdev))
    {
      RPMSG_VIRTIO_INVALIDATE(rvq->vq_ring.used->idx);
      priv->headrx = rvq->vq_ring.used->idx;
    }
  else if (VIRTIO_ROLE_IS_DEVICE(rvdev->vdev))
    {
      RPMSG_VIRTIO_INVALIDATE(rvq->vq_ring.avail->idx);
      priv->headrx = rvq->vq_ring.avail->idx;
    }
}

static void rpmsg_virtio_update_tx(FAR struct rpmsg_virtio_priv_s *priv)
{
  FAR struct rpmsg_virtio_device *rvdev = &priv->rvdev;
  FAR struct virtqueue *svq = rvdev->svq;

  if (VIRTIO_ROLE_IS_DRIVER(rvdev->vdev))
    {
      RPMSG_VIRTIO_INVALIDATE(svq->vq_ring.avail->idx);
      priv->headtx = svq->vq_ring.avail->idx;
    }
  else if (VIRTIO_ROLE_IS_DEVICE(rvdev->vdev))
    {
      RPMSG_VIRTIO_INVALIDATE(svq->vq_ring.used->idx);
      priv->headtx = svq->vq_ring.used->idx;
    }
}

/****************************************************************************
 * Name: rpmsg_virtio_rx_dispatch
 ****************************************************************************/

static void rpmsg_virtio_rx_dispatch(FAR struct rpmsg_virtio_priv_s *priv)
{
  FAR struct rpmsg_virtio_device *rvdev = &priv->rvdev;
  FAR struct rpmsg_device *rdev = &rvdev->rdev;
  FAR struct rpmsg_endpoint *ept;
  FAR struct rpmsg_hdr *hdr;
  bool release = false;
  bool last;
  uint32_t len;
  uint16_t idx;
  int status;

  metal_mutex_acquire(&rdev->lock);
  do
    {
      virtqueue_disable_cb(rvdev->rvq);
      last = false;
      while (!last)
        {
          /* Process the received data from remote node */

          hdr = rpmsg_virtio_get_rx_buffer(rvdev, &len, &idx, &last);
          if (hdr == NULL)
            {
              break;
            }

          hdr->reserved = idx;

          /* Drop a message whose length does not fit its buffer */

          ept = len >= sizeof(*hdr) && hdr->len <= len - sizeof(*hdr) ?
                rpmsg_get_ept_from_addr(rdev, hdr->dst) : NULL;
          RPMSG_BUF_HELD_INC(hdr);

          if (ept)
            {
              uint8_t priority = ept->priority;

              if (ept == &rdev->ns_ept)
                {
                  FAR struct rpmsg_endpoint *ept_;
                  FAR struct rpmsg_ns_msg *ns_msg;

                  ns_msg =
                    (FAR struct rpmsg_ns_msg *)RPMSG_LOCATE_DATA(hdr);
                  ept_ = rpmsg_get_endpoint(rdev, ns_msg->name,
                                            RPMSG_ADDR_ANY, hdr->dst);

                  /* Only inherit non-RT priority from the matched endpoint,
                   * ns_ept should not use RT priority to avoid overflow in
                   * priority calculation.
                   */

                  if (ept_ && ept_->priority != RPMSG_PRIO_RT)
                    {
                      priority = ept_->priority;
                    }
                }

              if (priority == RPMSG_PRIO_RT && ept != &rdev->ns_ept &&
                  ept->dest_addr != RPMSG_ADDR_ANY)
                {
                  status = rpmsg_virtio_process_rx_buffer(rdev, ept,
                                                          hdr);
                }
              else
                {
                  rpmsg_queue_rx_work(&priv->rpmsg, priority,
                                      rpmsg_virtio_rx_worker, hdr);
                  status = RPMSG_SUCCESS_BUFFER_RELEASED;
                }
            }
          else
            {
              status = RPMSG_SUCCESS;
            }

          if (status != RPMSG_SUCCESS_BUFFER_RELEASED &&
              rpmsg_virtio_buf_held_dec_test(hdr))
            {
              rpmsg_virtio_release_rx_buffer_nolock(rvdev, hdr);
              release = true;
            }
        }
    }
  while (virtqueue_enable_cb(rvdev->rvq));

  if (release)
    {
      /* Tell peer we return some rx buffer */

      virtqueue_kick(rvdev->rvq);
    }

  metal_mutex_release(&rdev->lock);
}

/****************************************************************************
 * Name: rpmsg_virtio_rx_callback
 ****************************************************************************/

static void rpmsg_virtio_rx_callback(FAR struct virtqueue *vq)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    container_of(vq->vq_dev->priv, struct rpmsg_virtio_priv_s, rvdev);

  if (priv->init)
    {
      /* Start response interrupt when rpmsg virtio initialized */

      rpmsg_virtio_rx_update(priv, vq);
      rpmsg_virtio_rx_dispatch(priv);
    }
}

/****************************************************************************
 * Name: rpmsg_virtio_tx_callback
 ****************************************************************************/

static void rpmsg_virtio_tx_callback(FAR struct virtqueue *vq)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    container_of(vq->vq_dev->priv, struct rpmsg_virtio_priv_s, rvdev);

  rpmsg_virtio_post(&priv->rpmsg);

  /* rpmsg_virtio_tx_callback() called normally means the tx buffer has been
   * returned by peer, so call rpmsg_virtio_pm_action(false) to enter to
   * low power mode when all the tx buffers have been returned.
   */

  rpmsg_virtio_pm_action(priv, false);
}

/****************************************************************************
 * Name: rpmsg_virtio_tx_notify
 ****************************************************************************/

static void rpmsg_virtio_tx_notify(FAR struct virtqueue *vq)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    container_of(vq->vq_dev->priv, struct rpmsg_virtio_priv_s, rvdev);

  /* rpmsg_virtio_tx_notify() called normally means send the buffer to peer,
   * so call rpmsg_virtio_pm_action(true) to hold the pm wakelock to avoid to
   * enter to low power mode until all the buffers are returned by peer.
   */

  rpmsg_virtio_update_tx(priv);
  rpmsg_virtio_pm_action(priv, true);
  priv->notifytx(vq);
}

/****************************************************************************
 * Name: rpmsg_virtio_notify_wait
 ****************************************************************************/

static int rpmsg_virtio_notify_wait(FAR struct rpmsg_device *rdev,
                                    uint32_t id)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    container_of(rdev, struct rpmsg_virtio_priv_s, rvdev);
  FAR struct virtqueue *vq = priv->rvdev.svq;
  int ret = 0;

  metal_mutex_acquire(&rdev->lock);
  if (priv->sleepertx++ == 0)
    {
      /* We are the first sleeping context, enable tx-complete interrupts */

      virtqueue_enable_cb(vq);
    }

  if ((VIRTIO_ROLE_IS_DRIVER(priv->vdev) && virtqueue_nused(vq) == 0) ||
      (VIRTIO_ROLE_IS_DEVICE(priv->vdev) && virtqueue_navail(vq) == 0))
    {
      metal_mutex_release(&rdev->lock);
      ret = rpmsg_tickwait(&priv->rvdev.rdev.ns_ept, &priv->semtx,
                           MSEC2TICK(RPMSG_VIRTIO_TIMEOUT_MS));
      metal_mutex_acquire(&rdev->lock);
    }

  /* Disable "tx-complete" interrupts after waking up if we're the last
   * sleeper. This avoids redundant interrupt overhead when buffers are
   * available.
   */

  if (--priv->sleepertx == 0)
    {
      /* We are the last sleeping context, disable tx-complete interrupts */

      virtqueue_disable_cb(vq);
    }

  /* Track how long the tx buffer stays unavailable. A non-timeout wakeup
   * means the peer returned a buffer (progress), so reset the counter.
   * A timeout is normally swallowed (return 0) so the OpenAMP wait loop
   * retries and pending recursive work can complete. Only after ~15s of
   * consecutive timeouts do we surface -ETIMEDOUT, which makes the
   * OpenAMP layer break out and assert on the lost tx buffer.
   */

  if (ret != -ETIMEDOUT)
    {
      priv->timeouttx = 0;
    }
  else if (++priv->timeouttx < RPMSG_VIRTIO_TIMEOUTTX_COUNT)
    {
      ret = 0;
    }
  else
    {
      /* Surface the timeout to the OpenAMP wait loop and re-arm the
       * counter so a later send attempt gets a full budget again.
       */

      priv->timeouttx = 0;
    }

  metal_mutex_release(&rdev->lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsg_virtio_start_worker
 ****************************************************************************/

static void rpmsg_virtio_start_worker(FAR void *arg)
{
  static const struct rpmsg_ops_s g_rpmsg_virtio_ops =
  {
    rpmsg_virtio_wait,
    rpmsg_virtio_post,
    NULL,
    NULL,
    rpmsg_virtio_dump,
    NULL,
    rpmsg_virtio_alloc_buf,
    rpmsg_virtio_free_buf,
  };

  FAR struct rpmsg_s *rpmsg = (FAR struct rpmsg_s *)arg;
  FAR struct rpmsg_virtio_priv_s *priv =
    (FAR struct rpmsg_virtio_priv_s *)rpmsg;
  FAR struct virtio_device *vdev = priv->vdev;
  struct rpmsg_virtio_config config =
  {
    RPMSG_BUFFER_SIZE,
    RPMSG_BUFFER_SIZE,
    false,
    rpmsg_virtio_rx_callback,
    rpmsg_virtio_tx_callback,
  };

  char name[64];
  uint16_t nrx = 0;
  int ret;

  if (virtio_has_feature(vdev, VIRTIO_RPMSG_F_BUFSZ))
    {
      virtio_read_config_member(vdev, struct fw_rsc_config, h2r_buf_size,
                                &config.h2r_buf_size)
      virtio_read_config_member(vdev, struct fw_rsc_config, r2h_buf_size,
                                &config.r2h_buf_size);
    }

  if (VIRTIO_ROLE_IS_DRIVER(vdev))
    {
      atomic_or(&rpmsg->signals, RPMSG_SIGNAL_ALLOC_LOCAL);
      if (virtio_has_feature(vdev, VIRTIO_RPMSG_F_BUFADDR))
        {
          FAR void *shmbuf_va0;
          FAR void *shmbuf_va1;
          uint64_t shmbuf_pa0 = 0ull;
          uint64_t shmbuf_pa1 = 0ull;

          /* In OpenAMP, priv->pool[0] is the RX share memory pool,
           * should use r2h_buf_addr and r2h_buf_size
           */

          virtio_read_config_member(vdev, struct fw_rsc_config, r2h_buf_addr,
                                    &shmbuf_pa0);
          shmbuf_va0 = up_addrenv_pa_to_va((uintptr_t)shmbuf_pa0);
          rpmsg_virtio_init_shm_pool(&priv->pool[0], shmbuf_va0,
                  config.r2h_buf_size * vdev->vrings_info[0].info.num_descs);

          /* In OpenAMP, priv->pool[1] is the TX share memory pool,
           * should use h2r_buf_addr and h2r_buf_size
           */

          virtio_read_config_member(vdev, struct fw_rsc_config, h2r_buf_addr,
                                    &shmbuf_pa1);
          shmbuf_va1 = up_addrenv_pa_to_va((uintptr_t)shmbuf_pa1);
          rpmsg_virtio_init_shm_pool(&priv->pool[1], shmbuf_va1,
                  config.h2r_buf_size * vdev->vrings_info[1].info.num_descs);

          config.split_shpool = true;
        }

      nrx = vdev->vrings_info[0].info.num_descs;
    }
  else if (VIRTIO_ROLE_IS_DEVICE(vdev))
    {
      nrx = vdev->vrings_info[1].info.num_descs;
    }

  /* Register the rpmsg to rpmsg framework */

  snprintf(name, sizeof(name), "/dev/rpmsg/%s", priv->rpmsg.cpuname);
  ret = rpmsg_register(name, &priv->rpmsg, &g_rpmsg_virtio_ops, nrx);
  if (ret >= 0)
    {
      ret = rpmsg_init_vdev_with_config(&priv->rvdev, vdev, rpmsg_ns_bind,
                                        metal_io_get_region(),
                                        priv->pool, &config);
      if (ret >= 0)
        {
          priv->notifytx = priv->rvdev.svq->notify;
          priv->rvdev.svq->notify = rpmsg_virtio_tx_notify;
          priv->rvdev.notify_wait_cb = rpmsg_virtio_notify_wait;
          priv->rvdev.rdev.ns_unbind_cb = rpmsg_ns_unbind;

          /* Wake up the rx thread to process message */

          rpmsg_virtio_rx_update(priv, priv->rvdev.rvq);
          rpmsg_virtio_rx_dispatch(priv);
          priv->init = true;

          /* Broadcast device_created to all registers */

          rpmsg_device_created(&priv->rpmsg);
        }
      else
        {
          rpmsgerr("rpmsg_init_vdev failed, ret=%d\n", ret);
          rpmsg_unregister(name, &priv->rpmsg);
        }
    }
  else
    {
      rpmsgerr("rpmsg register failed, ret=%d\n", ret);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_virtio_probe_cpuname
 ****************************************************************************/

int rpmsg_virtio_probe_cpuname(FAR struct virtio_device *vdev,
                               FAR const char *cpuname)
{
  FAR struct rpmsg_virtio_priv_s *priv;
  uint8_t priority;
#ifdef CONFIG_RPMSG_VIRTIO_PM
  char name[64];
#endif
  int ret = -ENOMEM;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv != NULL)
    {
      priv->vdev = vdev;
      nxsem_init(&priv->semrx, 0, 0);
      nxsem_init(&priv->semtx, 0, 0);

      if (VIRTIO_ROLE_IS_DRIVER(vdev))
        {
          virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
          virtio_negotiate_features(vdev, RPMSG_VIRTIO_FEATURES, NULL);
          virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);
        }
      else if (VIRTIO_ROLE_IS_DEVICE(vdev))
        {
          uint64_t features;

          virtio_get_features(vdev, &features);
        }

      /* Read the virtio rpmsg config to get the local/remote cpu name  */

      if (virtio_has_feature(vdev, VIRTIO_RPMSG_F_CPUNAME))
        {
          if (VIRTIO_ROLE_IS_DRIVER(vdev))
            {
              virtio_read_config_bytes(vdev, offsetof(struct fw_rsc_config,
                                                      host_cpuname),
                                      priv->rpmsg.local_cpuname,
                                      VIRTIO_RPMSG_CPUNAME_SIZE);
              virtio_read_config_bytes(vdev, offsetof(struct fw_rsc_config,
                                                      remote_cpuname),
                                      priv->rpmsg.cpuname,
                                      VIRTIO_RPMSG_CPUNAME_SIZE);
            }
          else if (VIRTIO_ROLE_IS_DEVICE(vdev))
            {
              virtio_read_config_bytes(vdev, offsetof(struct fw_rsc_config,
                                                      host_cpuname),
                                      priv->rpmsg.cpuname,
                                      VIRTIO_RPMSG_CPUNAME_SIZE);
              virtio_read_config_bytes(vdev, offsetof(struct fw_rsc_config,
                                                      remote_cpuname),
                                      priv->rpmsg.local_cpuname,
                                      VIRTIO_RPMSG_CPUNAME_SIZE);
            }
        }
      else
        {
          DEBUGASSERT(cpuname != NULL);
          strlcpy(priv->rpmsg.cpuname, cpuname, VIRTIO_RPMSG_CPUNAME_SIZE);
        }

      if (virtio_has_feature(vdev, VIRTIO_RPMSG_F_PRIORITY))
        {
          virtio_read_config(vdev, offsetof(struct fw_rsc_config, priority),
                             &priority, sizeof(priority));
        }
      else
        {
          priority = CONFIG_RPMSG_WQUEUE_PRIORITY;
        }

      ret = rpmsg_init_wqueues(&priv->rpmsg, priority);
      if (ret >= 0)
        {
#ifdef CONFIG_RPMSG_VIRTIO_PM
          spin_lock_init(&priv->lock);
          snprintf(name, sizeof(name), "rpmsg-virtio-%s",
                   priv->rpmsg.cpuname);
          pm_wakelock_init(&priv->wakelock, name, PM_IDLE_DOMAIN, PM_IDLE);
#endif

          rpmsg_queue_work(&priv->rpmsg, RPMSG_PRIO_DEFAULT,
                           &priv->startwork, rpmsg_virtio_start_worker,
                           &priv->rpmsg);
        }
      else
        {
          rpmsgerr("rpmsg_init_wqueues failed, ret=%d\n", ret);
          nxsem_destroy(&priv->semtx);
          nxsem_destroy(&priv->semrx);
          kmm_free(priv);
        }
    }
  else
    {
      rpmsgerr("No enough memory\n");
    }

  return ret;
}

/****************************************************************************
 * Name: rpmsg_virtio_probe
 ****************************************************************************/

int rpmsg_virtio_probe(FAR struct virtio_device *vdev)
{
  return rpmsg_virtio_probe_cpuname(vdev, NULL);
}

/****************************************************************************
 * Name: rpmsg_virtio_remove
 ****************************************************************************/

void rpmsg_virtio_remove(FAR struct virtio_device *vdev)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    container_of(vdev->priv, struct rpmsg_virtio_priv_s, rvdev);
  char name[64];

#ifdef CONFIG_RPMSG_VIRTIO_PM
  pm_wakelock_uninit(&priv->wakelock);
#endif

  /* Unregister the rpmsg */

  snprintf(name, sizeof(name), "/dev/rpmsg/%s", priv->rpmsg.cpuname);
  rpmsg_unregister(name, &priv->rpmsg);

  /* Disable tx buffer return callback */

  virtqueue_disable_cb(priv->rvdev.svq);

  /* Deinit the rpmsg wqueues */

  rpmsg_deinit_wqueues(&priv->rpmsg);

  /* Destroy all the rpmsg services */

  rpmsg_device_destroy(&priv->rpmsg);

  /* Reset the rpmsg virtio device for driver */

  if (VIRTIO_ROLE_IS_DRIVER(vdev))
    {
      virtio_reset_device(vdev);
    }

  /* Deinit the rpmsg virtio device */

  rpmsg_deinit_vdev(&priv->rvdev);

  /* Destroy the rpmsg virtio semaphores */

  nxsem_destroy(&priv->semtx);
  nxsem_destroy(&priv->semrx);

  /* Free the private data */

  kmm_free(priv);
}
