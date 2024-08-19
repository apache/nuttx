/****************************************************************************
 * drivers/rpmsg/rpmsg_virtio_lite.c
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

#include <nuttx/config.h>

#include <debug.h>
#include <stdio.h>
#include <sys/param.h>

#include <metal/cache.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/nuttx.h>
#include <nuttx/power/pm.h>
#include <nuttx/semaphore.h>
#include <nuttx/rpmsg/rpmsg_virtio_lite.h>
#include <rpmsg/rpmsg_internal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSG_VIRTIO_LITE_TIMEOUT_MS 20
#define RPMSG_VIRTIO_LITE_NOTIFYID   0

#ifdef CONFIG_OPENAMP_CACHE
#  define RPMSG_VIRTIO_INVALIDATE(x) metal_cache_invalidate(&x, sizeof(x))
#else
#  define RPMSG_VIRTIO_INVALIDATE(x)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsg_virtio_lite_priv_s
{
  struct rpmsg_s                     rpmsg;
  struct rpmsg_virtio_device         rvdev;
  FAR struct rpmsg_virtio_lite_s     *dev;
  FAR struct rpmsg_virtio_lite_rsc_s *rsc;
  struct virtio_device               vdev;
  struct rpmsg_virtio_shm_pool       pool[2];
  struct virtio_vring_info           rvrings[2];
  sem_t                              semtx;
  sem_t                              semrx;
  pid_t                              tid;
  uint16_t                           headrx;
#ifdef CONFIG_RPMSG_VIRTIO_LITE_PM
  struct pm_wakelock_s               wakelock;
  struct wdog_s                      wdog;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpmsg_virtio_lite_wait(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem);
static int rpmsg_virtio_lite_post(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem);
static void rpmsg_virtio_lite_panic(FAR struct rpmsg_s *rpmsg);
static void rpmsg_virtio_lite_dump(FAR struct rpmsg_s *rpmsg);
static FAR const char *
rpmsg_virtio_lite_get_local_cpuname(FAR struct rpmsg_s *rpmsg);
static FAR const char *
rpmsg_virtio_lite_get_cpuname(FAR struct rpmsg_s *rpmsg);

static int
rpmsg_virtio_lite_create_virtqueues_(FAR struct virtio_device *vdev,
                                     unsigned int flags,
                                     unsigned int nvqs,
                                     FAR const char *names[],
                                     vq_callback callbacks[],
                                     FAR void *callback_args[]);
static uint8_t rpmsg_virtio_lite_get_status_(FAR struct virtio_device *dev);
static void rpmsg_virtio_lite_set_status_(FAR struct virtio_device *dev,
                                          uint8_t status);
static uint64_t
rpmsg_virtio_lite_get_features_(FAR struct virtio_device *dev);
static void rpmsg_virtio_lite_set_features(FAR struct virtio_device *dev,
                                           uint64_t feature);
static void rpmsg_virtio_lite_notify(FAR struct virtqueue *vq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rpmsg_ops_s g_rpmsg_virtio_lite_ops =
{
  .wait               = rpmsg_virtio_lite_wait,
  .post               = rpmsg_virtio_lite_post,
  .panic              = rpmsg_virtio_lite_panic,
  .dump               = rpmsg_virtio_lite_dump,
  .get_local_cpuname  = rpmsg_virtio_lite_get_local_cpuname,
  .get_cpuname        = rpmsg_virtio_lite_get_cpuname,
};

static const struct virtio_dispatch g_rpmsg_virtio_lite_dispatch =
{
  .create_virtqueues = rpmsg_virtio_lite_create_virtqueues_,
  .get_status        = rpmsg_virtio_lite_get_status_,
  .set_status        = rpmsg_virtio_lite_set_status_,
  .get_features      = rpmsg_virtio_lite_get_features_,
  .set_features      = rpmsg_virtio_lite_set_features,
  .notify            = rpmsg_virtio_lite_notify,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_OPENAMP_DEBUG) || \
    defined(CONFIG_RPMSG_VIRTIO_LITE_PM_AUTORELAX)
static int
rpmsg_virtio_lite_buffer_nused(FAR struct rpmsg_virtio_device *rvdev,
                               bool rx)
{
  FAR struct virtqueue *vq = rx ? rvdev->rvq : rvdev->svq;
  uint16_t nused = vq->vq_ring.avail->idx - vq->vq_ring.used->idx;

  if ((rpmsg_virtio_get_role(rvdev) == RPMSG_HOST) ^ rx)
    {
      return nused;
    }
  else
    {
      return vq->vq_nentries - nused;
    }
}
#endif

static void
rpmsg_virtio_lite_wakeup_tx(FAR struct rpmsg_virtio_lite_priv_s *priv)
{
  int semcount;

  nxsem_get_value(&priv->semtx, &semcount);
  while (semcount++ < 1)
    {
      nxsem_post(&priv->semtx);
    }
}

#ifdef CONFIG_RPMSG_VIRTIO_LITE_PM

#ifdef CONFIG_RPMSG_VIRTIO_LITE_PM_AUTORELAX
static void rpmsg_virtio_lite_pm_callback(wdparm_t arg)
{
  FAR struct rpmsg_virtio_lite_priv_s *priv =
    (FAR struct rpmsg_virtio_lite_priv_s *)arg;

  if (rpmsg_virtio_lite_buffer_nused(&priv->rvdev, false))
    {
      rpmsg_virtio_lite_wakeup_tx(priv);

      wd_start(&priv->wdog, MSEC2TICK(RPMSG_VIRTIO_LITE_TIMEOUT_MS),
               rpmsg_virtio_lite_pm_callback, (wdparm_t)priv);
    }
  else
    {
      pm_wakelock_relax(&priv->wakelock);
    }
}
#endif

static inline void
rpmsg_virtio_lite_pm_action(FAR struct rpmsg_virtio_lite_priv_s *priv,
                            bool stay)
{
  irqstate_t flags;
  int count;

  flags = enter_critical_section();

  count = pm_wakelock_staycount(&priv->wakelock);
  if (stay && count == 0)
    {
      pm_wakelock_stay(&priv->wakelock);
#ifdef CONFIG_RPMSG_VIRTIO_LITE_PM_AUTORELAX
      wd_start(&priv->wdog, MSEC2TICK(RPMSG_VIRTIO_LITE_TIMEOUT_MS),
               rpmsg_virtio_lite_pm_callback, (wdparm_t)priv);
#endif
    }

#ifndef CONFIG_RPMSG_VIRTIO_LITE_PM_AUTORELAX
  if (!stay && count > 0 &&
      rpmsg_virtio_lite_buffer_nused(&priv->rvdev, false) == 0)
    {
      pm_wakelock_relax(&priv->wakelock);
    }
#endif

  leave_critical_section(flags);
}

static inline bool
rpmsg_virtio_lite_available_rx(FAR struct rpmsg_virtio_lite_priv_s *priv)
{
  FAR struct rpmsg_virtio_device *rvdev = &priv->rvdev;
  FAR struct virtqueue *rvq = rvdev->rvq;

  if (rpmsg_virtio_get_role(rvdev) == RPMSG_HOST)
    {
      return priv->headrx != rvq->vq_used_cons_idx;
    }
  else
    {
      return priv->headrx != rvq->vq_available_idx;
    }
}

#else
#  define rpmsg_virtio_lite_pm_action(priv, stay)
#  define rpmsg_virtio_lite_available_rx(priv) true
#endif

static inline void
rpmsg_virtio_lite_update_rx(FAR struct rpmsg_virtio_lite_priv_s *priv)
{
  FAR struct rpmsg_virtio_device *rvdev = &priv->rvdev;
  FAR struct virtqueue *rvq = rvdev->rvq;

  if (rpmsg_virtio_get_role(rvdev) == RPMSG_HOST)
    {
      RPMSG_VIRTIO_INVALIDATE(rvq->vq_ring.used->idx);
      priv->headrx = rvq->vq_ring.used->idx;
    }
  else
    {
      RPMSG_VIRTIO_INVALIDATE(rvq->vq_ring.avail->idx);
      priv->headrx = rvq->vq_ring.avail->idx;
    }
}

static FAR struct rpmsg_virtio_lite_priv_s *
rpmsg_virtio_lite_get_priv(FAR struct virtio_device *vdev)
{
  FAR struct rpmsg_virtio_device *rvdev = vdev->priv;

  return metal_container_of(rvdev, struct rpmsg_virtio_lite_priv_s, rvdev);
}

static int
rpmsg_virtio_lite_create_virtqueues_(FAR struct virtio_device *vdev,
                                     unsigned int flags,
                                     unsigned int nvqs,
                                     FAR const char *names[],
                                     vq_callback callbacks[],
                                     FAR void *callback_args[])
{
  int ret;
  int i;

  if (nvqs > vdev->vrings_num)
    {
      return ERROR_VQUEUE_INVLD_PARAM;
    }

  /* Initialize virtqueue for each vring */

  for (i = 0; i < nvqs; i++)
    {
      FAR struct virtio_vring_info *vinfo = &vdev->vrings_info[i];
      FAR struct vring_alloc_info *valloc = &vinfo->info;
#ifndef CONFIG_OPENAMP_VIRTIO_DEVICE_ONLY
      if (vdev->role == VIRTIO_DEV_DRIVER)
        {
          size_t offset;

          offset = metal_io_virt_to_offset(vinfo->io, valloc->vaddr);
          metal_io_block_set(vinfo->io, offset, 0,
                             vring_size(valloc->num_descs, valloc->align));
        }
#endif

      ret = virtqueue_create(vdev, i, names[i], valloc,
                             callbacks[i], vdev->func->notify,
                             vinfo->vq);
      if (ret < 0)
        {
          return ret;
        }
    }

  return 0;
}

static uint8_t rpmsg_virtio_lite_get_status_(FAR struct virtio_device *vdev)
{
  FAR struct rpmsg_virtio_lite_priv_s *priv =
    rpmsg_virtio_lite_get_priv(vdev);

  return priv->rsc->rpmsg_vdev.status;
}

static void rpmsg_virtio_lite_set_status_(FAR struct virtio_device *vdev,
                                          uint8_t status)
{
  FAR struct rpmsg_virtio_lite_priv_s *priv =
    rpmsg_virtio_lite_get_priv(vdev);

  priv->rsc->rpmsg_vdev.status = status;
}

static uint64_t
rpmsg_virtio_lite_get_features_(FAR struct virtio_device *vdev)
{
  FAR struct rpmsg_virtio_lite_priv_s *priv =
    rpmsg_virtio_lite_get_priv(vdev);

  return priv->rsc->rpmsg_vdev.dfeatures;
}

static void rpmsg_virtio_lite_set_features(FAR struct virtio_device *vdev,
                                           uint64_t features)
{
  FAR struct rpmsg_virtio_lite_priv_s *priv =
    rpmsg_virtio_lite_get_priv(vdev);

  priv->rsc->rpmsg_vdev.gfeatures = features;
}

static void rpmsg_virtio_lite_notify(FAR struct virtqueue *vq)
{
  FAR struct virtio_device *vdev = vq->vq_dev;
  FAR struct rpmsg_virtio_lite_priv_s *priv =
    rpmsg_virtio_lite_get_priv(vdev);
  FAR struct virtqueue *svq = priv->rvdev.svq;

  if (vdev->vrings_info->notifyid ==
      vdev->vrings_info[svq->vq_queue_index].notifyid)
    {
      rpmsg_virtio_lite_pm_action(priv, true);
    }

  RPMSG_VIRTIO_LITE_NOTIFY(priv->dev, vdev->vrings_info->notifyid);
}

static bool
rpmsg_virtio_lite_is_recursive(FAR struct rpmsg_virtio_lite_priv_s *priv)
{
  return nxsched_gettid() == priv->tid;
}

static int rpmsg_virtio_lite_wait(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem)
{
  FAR struct rpmsg_virtio_lite_priv_s *priv =
    (FAR struct rpmsg_virtio_lite_priv_s *)rpmsg;
  int ret;

  if (!rpmsg_virtio_lite_is_recursive(priv))
    {
      return nxsem_wait_uninterruptible(sem);
    }

  while (1)
    {
      ret = nxsem_trywait(sem);
      if (ret >= 0)
        {
          break;
        }

      nxsem_wait(&priv->semtx);
      virtqueue_notification(priv->rvdev.rvq);
    }

  return ret;
}

static int rpmsg_virtio_lite_post(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem)
{
  FAR struct rpmsg_virtio_lite_priv_s *priv =
    (FAR struct rpmsg_virtio_lite_priv_s *)rpmsg;
  int semcount;
  int ret;

  nxsem_get_value(sem, &semcount);
  ret = nxsem_post(sem);

  if (priv && semcount >= 0)
    {
      rpmsg_virtio_lite_wakeup_tx(priv);
    }

  return ret;
}

static void rpmsg_virtio_lite_panic(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_virtio_lite_priv_s *priv =
    (FAR struct rpmsg_virtio_lite_priv_s *)rpmsg;
  FAR struct rpmsg_virtio_lite_cmd_s *cmd =
    RPMSG_VIRTIO_LITE_RSC2CMD(priv->rsc);

  if (RPMSG_VIRTIO_LITE_IS_MASTER(priv->dev))
    {
      cmd->cmd_master = RPMSG_VIRTIO_LITE_CMD(
        RPMSG_VIRTIO_LITE_CMD_PANIC, 0);
    }
  else
    {
      cmd->cmd_slave = RPMSG_VIRTIO_LITE_CMD(
        RPMSG_VIRTIO_LITE_CMD_PANIC, 0);
    }

  rpmsg_virtio_lite_notify(priv->vdev.vrings_info->vq);
}

#ifdef CONFIG_OPENAMP_DEBUG
static void
rpmsg_virtio_lite_dump_buffer(FAR struct rpmsg_virtio_device *rvdev, bool rx)
{
  FAR struct virtqueue *vq = rx ? rvdev->rvq : rvdev->svq;
  int num;
  int i;

  num = rpmsg_virtio_lite_buffer_nused(rvdev, rx);
  metal_log(METAL_LOG_EMERGENCY,
            "    %s buffer, total %d, pending %d\n",
            rx ? "RX" : "TX", vq->vq_nentries, num);

  for (i = 0; i < num; i++)
    {
      FAR void *addr;
      int desc_idx;

      if ((rpmsg_virtio_get_role(rvdev) == RPMSG_HOST) ^ rx)
        {
          desc_idx = (vq->vq_ring.used->idx + i) & (vq->vq_nentries - 1);
          desc_idx = vq->vq_ring.avail->ring[desc_idx];
        }
      else
        {
          desc_idx = (vq->vq_ring.avail->idx + i) & (vq->vq_nentries - 1);
          desc_idx = vq->vq_ring.used->ring[desc_idx].id;
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
              metal_log(METAL_LOG_EMERGENCY,
                        "      %s buffer %p hold by %s\n",
                        rx ? "RX" : "TX", hdr, ept->name);
            }
        }
    }
}

static void rpmsg_virtio_lite_dump(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_virtio_lite_priv_s *priv =
      (FAR struct rpmsg_virtio_lite_priv_s *)rpmsg;
  FAR struct rpmsg_virtio_device *rvdev = &priv->rvdev;
  FAR struct rpmsg_device *rdev = rpmsg->rdev;
  FAR struct rpmsg_endpoint *ept;
  FAR struct metal_list *node;
  bool needlock = true;

  metal_log(METAL_LOG_EMERGENCY, "Remote: %s headrx %d\n",
            RPMSG_VIRTIO_LITE_GET_CPUNAME(priv->dev), priv->headrx);

  if (!rvdev->vdev)
    {
      return;
    }

  if (up_interrupt_context() || sched_idletask() ||
      nxmutex_is_hold(&rdev->lock))
    {
      needlock = false;
    }

  if (needlock)
    {
      metal_mutex_acquire(&rdev->lock);
    }

  metal_log(METAL_LOG_EMERGENCY,
            "Dump rpmsg info between cpu (master: %s)%s <==> %s:\n",
            rpmsg_virtio_get_role(rvdev) == RPMSG_HOST ? "yes" : "no",
            CONFIG_RPMSG_LOCAL_CPUNAME, rpmsg_get_cpuname(rdev));

  metal_log(METAL_LOG_EMERGENCY, "rpmsg vq RX:\n");
  virtqueue_dump(rvdev->rvq);
  metal_log(METAL_LOG_EMERGENCY, "rpmsg vq TX:\n");
  virtqueue_dump(rvdev->svq);

  metal_log(METAL_LOG_EMERGENCY, "  rpmsg ept list:\n");

  metal_list_for_each(&rdev->endpoints, node)
    {
      ept = metal_container_of(node, struct rpmsg_endpoint, node);
      metal_log(METAL_LOG_EMERGENCY, "    ept %s\n", ept->name);
    }

  metal_log(METAL_LOG_EMERGENCY, "  rpmsg buffer list:\n");

  rpmsg_virtio_lite_dump_buffer(rvdev, true);
  rpmsg_virtio_lite_dump_buffer(rvdev, false);

  if (needlock)
    {
      metal_mutex_release(&rdev->lock);
    }
}
#else
static void rpmsg_virtio_lite_dump(FAR struct rpmsg_s *rpmsg)
{
  /* Nothing */
}
#endif

static FAR const char *
rpmsg_virtio_lite_get_local_cpuname(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_virtio_lite_priv_s *priv =
    (FAR struct rpmsg_virtio_lite_priv_s *)rpmsg;

  return RPMSG_VIRTIO_LITE_GET_LOCAL_CPUNAME(priv->dev);
}

static FAR const char *
rpmsg_virtio_lite_get_cpuname(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_virtio_lite_priv_s *priv =
    (FAR struct rpmsg_virtio_lite_priv_s *)rpmsg;

  return RPMSG_VIRTIO_LITE_GET_CPUNAME(priv->dev);
}

static void
rpmsg_virtio_lite_wakeup_rx(FAR struct rpmsg_virtio_lite_priv_s *priv)
{
  int semcount;

  nxsem_get_value(&priv->semrx, &semcount);
  if (semcount < 1)
    {
      nxsem_post(&priv->semrx);
    }
}

static void rpmsg_virtio_lite_command(struct rpmsg_virtio_lite_priv_s *priv)
{
  FAR struct rpmsg_virtio_lite_cmd_s *rpmsg_virtio_cmd =
    RPMSG_VIRTIO_LITE_RSC2CMD(priv->rsc);
  uint32_t cmd;

  if (RPMSG_VIRTIO_LITE_IS_MASTER(priv->dev))
    {
      cmd = rpmsg_virtio_cmd->cmd_slave;
      rpmsg_virtio_cmd->cmd_slave = 0;
    }
  else
    {
      cmd = rpmsg_virtio_cmd->cmd_master;
      rpmsg_virtio_cmd->cmd_master = 0;
    }

  switch (RPMSG_VIRTIO_LITE_GET_CMD(cmd))
    {
      case RPMSG_VIRTIO_LITE_CMD_PANIC:
        PANIC();
        break;

      default:
        break;
    }
}

static int rpmsg_virtio_lite_callback(FAR void *arg, uint32_t vqid)
{
  FAR struct rpmsg_virtio_lite_priv_s *priv = arg;
  FAR struct rpmsg_virtio_device *rvdev = &priv->rvdev;
  FAR struct virtio_device *vdev = rvdev->vdev;
  FAR struct virtqueue *rvq = rvdev->rvq;
  FAR struct virtqueue *svq = rvdev->svq;

  rpmsg_virtio_lite_command(priv);

  if (vqid == RPMSG_VIRTIO_LITE_NOTIFY_ALL ||
      vqid == vdev->vrings_info[rvq->vq_queue_index].notifyid)
    {
      rpmsg_virtio_lite_update_rx(priv);
      rpmsg_virtio_lite_wakeup_rx(priv);
    }

  if (vqid == RPMSG_VIRTIO_LITE_NOTIFY_ALL ||
      vqid == vdev->vrings_info[svq->vq_queue_index].notifyid)
    {
      rpmsg_virtio_lite_wakeup_tx(priv);
      rpmsg_virtio_lite_pm_action(priv, false);
    }

  return OK;
}

static int rpmsg_virtio_lite_notify_wait(FAR struct rpmsg_device *rdev,
                                         uint32_t id)
{
  FAR struct rpmsg_virtio_lite_priv_s *priv =
    metal_container_of(rdev, struct rpmsg_virtio_lite_priv_s, rvdev.rdev);

  if (!rpmsg_virtio_lite_is_recursive(priv))
    {
      return -EAGAIN;
    }

  /* Wait to wakeup */

  nxsem_tickwait(&priv->semtx, MSEC2TICK(RPMSG_VIRTIO_LITE_TIMEOUT_MS));
  virtqueue_notification(priv->rvdev.rvq);

  return 0;
}

static int rpmsg_virtio_lite_start(FAR struct rpmsg_virtio_lite_priv_s *priv)
{
  FAR struct virtio_vring_info *rvrings = priv->rvrings;
  FAR struct virtio_device *vdev = &priv->vdev;
  FAR struct rpmsg_virtio_lite_rsc_s *rsc;
  struct rpmsg_virtio_config config;
  FAR void *shbuf0;
  FAR void *shbuf1;
  uint32_t align0;
  uint32_t align1;
  uint32_t tbsz;
  uint32_t v0sz;
  uint32_t v1sz;
  uint32_t shbufsz0;
  uint32_t shbufsz1;
  int ret;

  rsc = RPMSG_VIRTIO_LITE_GET_RESOURCE(priv->dev);
  if (!rsc)
    {
      return -EINVAL;
    }

  priv->rsc = rsc;

  vdev->notifyid = RPMSG_VIRTIO_LITE_NOTIFYID;
  vdev->vrings_num = rsc->rpmsg_vdev.num_of_vrings;
  vdev->role = RPMSG_VIRTIO_LITE_IS_MASTER(priv->dev) ?
               RPMSG_HOST : RPMSG_REMOTE;
  vdev->func = &g_rpmsg_virtio_lite_dispatch;

  align0 = rsc->rpmsg_vring0.align;
  align1 = rsc->rpmsg_vring1.align;
  tbsz = ALIGN_UP(sizeof(*rsc), MAX(align0, align1));
  v0sz = ALIGN_UP(vring_size(rsc->rpmsg_vring0.num, align0), align0);
  v1sz = ALIGN_UP(vring_size(rsc->rpmsg_vring1.num, align1), align1);

  shbuf0   = (FAR char *)rsc + tbsz + v0sz + v1sz;
  shbufsz0 = rsc->config.r2h_buf_size * rsc->rpmsg_vring0.num;
  shbuf1   = shbuf0 + shbufsz0;
  shbufsz1 = rsc->config.h2r_buf_size * rsc->rpmsg_vring1.num;

  rvrings[0].io = metal_io_get_region();
  rvrings[0].info.vaddr = (FAR char *)rsc + tbsz;
  rvrings[0].info.num_descs = rsc->rpmsg_vring0.num;
  rvrings[0].info.align = rsc->rpmsg_vring0.align;
  rvrings[0].vq = virtqueue_allocate(rsc->rpmsg_vring0.num);
  if (rvrings[0].vq == NULL)
    {
      return -ENOMEM;
    }

  rvrings[1].io = metal_io_get_region();
  rvrings[1].info.vaddr = (FAR char *)rsc + tbsz + v0sz;
  rvrings[1].info.num_descs = rsc->rpmsg_vring1.num;
  rvrings[1].info.align = rsc->rpmsg_vring1.align;
  rvrings[1].vq = virtqueue_allocate(rsc->rpmsg_vring1.num);
  if (rvrings[1].vq == NULL)
    {
      ret = -ENOMEM;
      goto err_vq0;
    }

  vdev->vrings_info = &rvrings[0];

  rpmsg_virtio_init_shm_pool(&priv->pool[0], shbuf0, shbufsz0);
  rpmsg_virtio_init_shm_pool(&priv->pool[1], shbuf1, shbufsz1);

  config.h2r_buf_size = rsc->config.h2r_buf_size;
  config.r2h_buf_size = rsc->config.r2h_buf_size;
  config.split_shpool = true;

  ret = rpmsg_init_vdev_with_config(&priv->rvdev, vdev, rpmsg_ns_bind,
                                    metal_io_get_region(),
                                    priv->pool, &config);
  if (ret != 0)
    {
      rpmsgerr("rpmsg_init_vdev failed %d\n", ret);
      ret = -ENOMEM;
      goto err_vq1;
    }

  priv->rvdev.rdev.ns_unbind_cb = rpmsg_ns_unbind;
  priv->rvdev.notify_wait_cb = rpmsg_virtio_lite_notify_wait;

  RPMSG_VIRTIO_LITE_REGISTER_CALLBACK(priv->dev, rpmsg_virtio_lite_callback,
                                      priv);

  rpmsg_virtio_lite_update_rx(priv);
  rpmsg_virtio_lite_wakeup_rx(priv);

  /* Broadcast device_created to all registers */

  rpmsg_device_created(&priv->rpmsg);

  return 0;

err_vq1:
  virtqueue_free(rvrings[1].vq);
err_vq0:
  virtqueue_free(rvrings[0].vq);
  return ret;
}

static int rpmsg_virtio_lite_thread(int argc, FAR char *argv[])
{
  FAR struct rpmsg_virtio_lite_priv_s *priv =
    (FAR struct rpmsg_virtio_lite_priv_s *)
    ((uintptr_t)strtoul(argv[2], NULL, 16));
  int ret;

  priv->tid = nxsched_gettid();

  ret = rpmsg_virtio_lite_start(priv);
  if (ret < 0)
    {
      rpmsgerr("rpmsg virtio thread start failed %d\n", ret);
      return ret;
    }

  while (1)
    {
      nxsem_wait_uninterruptible(&priv->semrx);
      if (rpmsg_virtio_lite_available_rx(priv))
        {
          virtqueue_notification(priv->rvdev.rvq);
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rpmsg_virtio_lite_initialize(FAR struct rpmsg_virtio_lite_s *dev)
{
  FAR struct rpmsg_virtio_lite_priv_s *priv;
  FAR char *argv[3];
  char arg1[32];
  char name[32];
  int ret;

  priv = kmm_zalloc(sizeof(struct rpmsg_virtio_lite_priv_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->dev = dev;
  nxsem_init(&priv->semrx, 0, 0);
  nxsem_init(&priv->semtx, 0, 0);

  snprintf(name, sizeof(name), "/dev/rpmsg/%s",
           RPMSG_VIRTIO_LITE_GET_CPUNAME(dev));
  ret = rpmsg_register(name, &priv->rpmsg, &g_rpmsg_virtio_lite_ops);
  if (ret < 0)
    {
      goto err_driver;
    }

  snprintf(arg1, sizeof(arg1), "%p", priv);
  argv[0] = (FAR char *)RPMSG_VIRTIO_LITE_GET_CPUNAME(dev);
  argv[1] = arg1;
  argv[2] = NULL;

  ret = kthread_create("rpmsg_virtio", CONFIG_RPMSG_VIRTIO_LITE_PRIORITY,
                       CONFIG_RPMSG_VIRTIO_LITE_STACKSIZE,
                       rpmsg_virtio_lite_thread, argv);
  if (ret < 0)
    {
      goto err_thread;
    }

  return OK;

err_thread:
  rpmsg_unregister(name, &priv->rpmsg);

err_driver:
  nxsem_destroy(&priv->semrx);
  nxsem_destroy(&priv->semtx);
  kmm_free(priv);

  return ret;
}
