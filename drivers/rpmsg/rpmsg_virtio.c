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

#include <debug.h>
#include <stdio.h>
#include <stdbool.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/rpmsg/rpmsg_virtio.h>
#include <nuttx/semaphore.h>
#include <nuttx/virtio/virtio-config.h>
#include <metal/utilities.h>
#include <openamp/rpmsg_virtio.h>
#include <rpmsg/rpmsg_internal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSG_VIRTIO_FEATURES        (1 << VIRTIO_RPMSG_F_NS | \
                                      1 << VIRTIO_RPMSG_F_ACK | \
                                      1 << VIRTIO_RPMSG_F_BUFSZ | \
                                      1 << VIRTIO_RPMSG_F_CPUNAME)

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
  struct rpmsg_s               rpmsg;
  struct rpmsg_virtio_device   rvdev;
  struct rpmsg_virtio_shm_pool pool[2];
  struct virtio_device         *vdev;
  sem_t                        semrx;
  sem_t                        semtx;
  pid_t                        tid;
  vq_callback                  cbrx;
  vq_callback                  cbtx;
  char                         local_cpuname[VIRTIO_RPMSG_CPUNAME_SIZE];
  char                         cpuname[VIRTIO_RPMSG_CPUNAME_SIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpmsg_virtio_wait(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem);
static int rpmsg_virtio_post(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem);
static void rpmsg_virtio_dump(FAR struct rpmsg_s *rpmsg);
static FAR const char *
rpmsg_virtio_get_local_cpuname(FAR struct rpmsg_s *rpmsg);
static FAR const char *rpmsg_virtio_get_cpuname(FAR struct rpmsg_s *rpmsg);

static void rpmsg_virtio_rx_callback(FAR struct virtqueue *vq);
static void rpmsg_virtio_tx_callback(FAR struct virtqueue *vq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rpmsg_ops_s g_rpmsg_virtio_ops =
{
  rpmsg_virtio_wait,
  rpmsg_virtio_post,
  NULL,
  NULL,
  rpmsg_virtio_dump,
  rpmsg_virtio_get_local_cpuname,
  rpmsg_virtio_get_cpuname,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_virtio_is_recursive
 ****************************************************************************/

static bool rpmsg_virtio_is_recursive(FAR struct rpmsg_virtio_priv_s *priv)
{
  return nxsched_gettid() == priv->tid;
}

/****************************************************************************
 * Name: rpmsg_virtio_wakeup_rx
 ****************************************************************************/

static void rpmsg_virtio_wakeup_rx(FAR struct rpmsg_virtio_priv_s *priv)
{
  int semcount;

  nxsem_get_value(&priv->semrx, &semcount);
  while (semcount++ < 1)
    {
      nxsem_post(&priv->semrx);
    }
}

/****************************************************************************
 * Name: rpmsg_virtio_wakeup_tx
 ****************************************************************************/

static void rpmsg_virtio_wakeup_tx(FAR struct rpmsg_virtio_priv_s *priv)
{
  int semcount;

  nxsem_get_value(&priv->semtx, &semcount);
  while (semcount++ < 1)
    {
      nxsem_post(&priv->semtx);
    }
}

/****************************************************************************
 * Name: rpmsg_virtio_wait
 ****************************************************************************/

static int rpmsg_virtio_wait(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    (FAR struct rpmsg_virtio_priv_s *)rpmsg;
  int ret;

  if (!rpmsg_virtio_is_recursive(priv))
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
      priv->cbrx(priv->rvdev.rvq);
    }

  return ret;
}

/****************************************************************************
 * Name: rpmsg_virtio_post
 ****************************************************************************/

static int rpmsg_virtio_post(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    (FAR struct rpmsg_virtio_priv_s *)rpmsg;
  int semcount;
  int ret;

  nxsem_get_value(sem, &semcount);
  ret = nxsem_post(sem);

  if (priv && semcount >= 0)
    {
      rpmsg_virtio_wakeup_tx(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: rpmsg_virtio_buffer_nused
 ****************************************************************************/

static int rpmsg_virtio_buffer_nused(FAR struct rpmsg_virtio_device *rvdev,
                                     bool rx)
{
  FAR struct virtqueue *vq = rx ? rvdev->rvq : rvdev->svq;
  bool is_host = rpmsg_virtio_get_role(rvdev) == RPMSG_HOST;
  uint16_t nused;

  if (is_host)
    {
      RPMSG_VIRTIO_INVALIDATE(vq->vq_ring.used->idx);
    }
  else
    {
      RPMSG_VIRTIO_INVALIDATE(vq->vq_ring.avail->idx);
    }

  nused = vq->vq_ring.avail->idx - vq->vq_ring.used->idx;
  if (is_host ^ rx)
    {
      return nused;
    }
  else
    {
      return vq->vq_nentries - nused;
    }
}

/****************************************************************************
 * Name: rpmsg_virtio_dump_buffer
 ****************************************************************************/

static void rpmsg_virtio_dump_buffer(FAR struct rpmsg_virtio_device *rvdev,
                                     bool rx)
{
  FAR struct virtqueue *vq = rx ? rvdev->rvq : rvdev->svq;
  FAR void *addr;
  int desc_idx;
  int num;
  int i;

  num = rpmsg_virtio_buffer_nused(rvdev, rx);
  metal_log(METAL_LOG_EMERGENCY,
            "    %s buffer, total %d, pending %d\n",
            rx ? "RX" : "TX", vq->vq_nentries, num);

  for (i = 0; i < num; i++)
    {
      if ((rpmsg_virtio_get_role(rvdev) == RPMSG_HOST) ^ rx)
        {
          RPMSG_VIRTIO_INVALIDATE(vq->vq_ring.used->idx);
          desc_idx = (vq->vq_ring.used->idx + i) & (vq->vq_nentries - 1);
          RPMSG_VIRTIO_INVALIDATE(vq->vq_ring.avail->ring[desc_idx]);
          desc_idx = vq->vq_ring.avail->ring[desc_idx];
        }
      else
        {
          RPMSG_VIRTIO_INVALIDATE(vq->vq_ring.avail->idx);
          desc_idx = (vq->vq_ring.avail->idx + i) & (vq->vq_nentries - 1);
          RPMSG_VIRTIO_INVALIDATE(vq->vq_ring.used->ring[desc_idx].id);
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

/****************************************************************************
 * Name: rpmsg_virtio_dump
 ****************************************************************************/

static void rpmsg_virtio_dump(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    (FAR struct rpmsg_virtio_priv_s *)rpmsg;
  FAR struct rpmsg_virtio_device *rvdev = &priv->rvdev;
  FAR struct rpmsg_device *rdev = rpmsg->rdev;
  FAR struct rpmsg_endpoint *ept;
  FAR struct metal_list *node;
  bool needunlock = false;

  if (!rvdev->vdev)
    {
      return;
    }

  if (!up_interrupt_context() && !sched_idletask() &&
      !metal_mutex_is_acquired(&rdev->lock))
    {
      metal_mutex_acquire(&rdev->lock);
      needunlock = true;
    }

  metal_log(METAL_LOG_EMERGENCY,
            "Dump rpmsg info between cpu (master: %s)%s <==> %s:\n",
            rpmsg_virtio_get_role(rvdev) == RPMSG_HOST ? "yes" : "no",
            priv->local_cpuname, priv->cpuname);

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

  rpmsg_virtio_dump_buffer(rvdev, true);
  rpmsg_virtio_dump_buffer(rvdev, false);

  if (needunlock)
    {
      metal_mutex_release(&rdev->lock);
    }
}

/****************************************************************************
 * Name: rpmsg_virtio_get_local_cpuname
 ****************************************************************************/

static FAR const char *
rpmsg_virtio_get_local_cpuname(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    (FAR struct rpmsg_virtio_priv_s *)rpmsg;

  return priv->local_cpuname;
}

/****************************************************************************
 * Name: rpmsg_virtio_get_cpuname
 ****************************************************************************/

static FAR const char *rpmsg_virtio_get_cpuname(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    (FAR struct rpmsg_virtio_priv_s *)rpmsg;

  return priv->cpuname;
}

/****************************************************************************
 * Name: rpmsg_virtio_rx_callback
 ****************************************************************************/

static void rpmsg_virtio_rx_callback(FAR struct virtqueue *vq)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    metal_container_of(vq->vq_dev->priv, struct rpmsg_virtio_priv_s, rvdev);

  rpmsg_virtio_wakeup_rx(priv);
}

/****************************************************************************
 * Name: rpmsg_virtio_tx_callback
 ****************************************************************************/

static void rpmsg_virtio_tx_callback(FAR struct virtqueue *vq)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    metal_container_of(vq->vq_dev->priv, struct rpmsg_virtio_priv_s, rvdev);

  rpmsg_virtio_wakeup_tx(priv);
}

/****************************************************************************
 * Name: rpmsg_virtio_notify_wait
 ****************************************************************************/

static int rpmsg_virtio_notify_wait(FAR struct rpmsg_device *rdev,
                                    uint32_t id)
{
  FAR struct rpmsg_virtio_priv_s *priv = (FAR struct rpmsg_virtio_priv_s *)
    metal_container_of(rdev, struct rpmsg_s, rdev);

  if (!rpmsg_virtio_is_recursive(priv))
    {
      return -EAGAIN;
    }

  /* Wait to wakeup */

  virtqueue_enable_cb(priv->rvdev.svq);
  nxsem_tickwait(&priv->semtx, MSEC2TICK(20));
  virtqueue_disable_cb(priv->rvdev.svq);
  priv->cbrx(priv->rvdev.rvq);
  return 0;
}

/****************************************************************************
 * Name: rpmsg_virtio_start
 ****************************************************************************/

static int rpmsg_virtio_start(FAR struct rpmsg_virtio_priv_s *priv)
{
  FAR struct virtio_device *vdev = priv->vdev;
  int ret;

  ret = rpmsg_init_vdev(&priv->rvdev, vdev, rpmsg_ns_bind,
                        metal_io_get_region(), priv->pool);
  if (ret < 0)
    {
      rpmsgerr("rpmsg_init_vdev failed, ret=%d\n", ret);
      return ret;
    }

  priv->cbrx = priv->rvdev.rvq->callback;
  priv->cbtx = priv->rvdev.svq->callback;
  priv->rvdev.rvq->callback = rpmsg_virtio_rx_callback;
  priv->rvdev.svq->callback = rpmsg_virtio_tx_callback;
  priv->rvdev.notify_wait_cb = rpmsg_virtio_notify_wait;
  priv->rvdev.rdev.ns_unbind_cb = rpmsg_ns_unbind;

  /* Wake up the rx thread to process message */

  rpmsg_virtio_wakeup_rx(priv);

  /* Broadcast device_created to all registers */

  rpmsg_device_created(&priv->rpmsg);

  return ret;
}

/****************************************************************************
 * Name: rpmsg_virtio_thread
 ****************************************************************************/

static int rpmsg_virtio_thread(int argc, FAR char *argv[])
{
  FAR struct rpmsg_virtio_priv_s *priv = (FAR struct rpmsg_virtio_priv_s *)
    ((uintptr_t)strtoul(argv[2], NULL, 16));
  int ret;

  priv->tid = nxsched_gettid();
  ret = rpmsg_virtio_start(priv);
  if (ret < 0)
    {
      rpmsgerr("virtio rpmsg start failed, ret=%d\n", ret);
      return ret;
    }

  while (1)
    {
      nxsem_wait_uninterruptible(&priv->semrx);
      priv->cbrx(priv->rvdev.rvq);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_virtio_probe
 ****************************************************************************/

int rpmsg_virtio_probe(FAR struct virtio_device *vdev)
{
  FAR struct rpmsg_virtio_priv_s *priv;
  FAR char *argv[3];
  uint64_t features;
  char name[32];
  char arg1[32];
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      rpmsgerr("No enough memory\n");
      return -ENOMEM;
    }

  priv->vdev = vdev;
  nxsem_init(&priv->semrx, 0, 0);
  nxsem_init(&priv->semtx, 0, 0);

  if (vdev->role == VIRTIO_DEV_DRIVER)
    {
      virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
      virtio_negotiate_features(vdev, RPMSG_VIRTIO_FEATURES, NULL);
      virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);
    }
  else
    {
      virtio_get_features(vdev, &features);
    }

  /* Read the virtio rpmsg config to get the local/remote cpu name  */

  DEBUGASSERT(virtio_has_feature(vdev, VIRTIO_RPMSG_F_CPUNAME));
  if (vdev->role == VIRTIO_DEV_DRIVER)
    {
      virtio_read_config(vdev, offsetof(struct fw_rsc_config, host_cpuname),
                         priv->local_cpuname, sizeof(priv->local_cpuname));
      virtio_read_config(vdev,
                         offsetof(struct fw_rsc_config, remote_cpuname),
                         priv->cpuname, sizeof(priv->cpuname));
    }
  else
    {
      virtio_read_config(vdev, offsetof(struct fw_rsc_config, host_cpuname),
                         priv->cpuname, sizeof(priv->cpuname));
      virtio_read_config(vdev,
                         offsetof(struct fw_rsc_config, remote_cpuname),
                         priv->local_cpuname, sizeof(priv->local_cpuname));
    }

  /* Register the rpmsg to rpmsg framework */

  snprintf(name, sizeof(name), "/dev/rpmsg/%s", priv->cpuname);
  ret = rpmsg_register(name, &priv->rpmsg, &g_rpmsg_virtio_ops);
  if (ret < 0)
    {
      rpmsgerr("rpmsg register failed, ret=%d\n", ret);
      goto err;
    }

  snprintf(arg1, sizeof(arg1), "%p", priv);
  argv[0] = priv->cpuname;
  argv[1] = arg1;
  argv[2] = NULL;
  ret = kthread_create("rpmsg-virtio", CONFIG_RPMSG_VIRTIO_PRIORITY,
                       CONFIG_RPMSG_VIRTIO_STACKSIZE,
                       rpmsg_virtio_thread, argv);
  if (ret < 0)
    {
      rpmsgerr("kthread_create failed, ret=%d\n", ret);
      goto err_kthread;
    }

  priv->tid = ret;
  return ret;

err_kthread:
  rpmsg_unregister(name, &priv->rpmsg);
err:
  nxsem_destroy(&priv->semtx);
  nxsem_destroy(&priv->semrx);
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: rpmsg_virtio_remove
 ****************************************************************************/

void rpmsg_virtio_remove(FAR struct virtio_device *vdev)
{
  FAR struct rpmsg_virtio_priv_s *priv =
    metal_container_of(vdev->priv, struct rpmsg_virtio_priv_s, rvdev);
  char name[32];

  /* Unregister the rpmsg */

  snprintf(name, sizeof(name), "/dev/rpmsg/%s", priv->cpuname);
  rpmsg_unregister(name, &priv->rpmsg);

  /* Disable tx buffer return callback */

  virtqueue_disable_cb(priv->rvdev.svq);

  /* Destroy all the rpmsg services */

  rpmsg_device_destory(&priv->rpmsg);

  /* Reset the rpmsg virtio device for driver */

  if (vdev->role == VIRTIO_DEV_DRIVER)
    {
      virtio_reset_device(vdev);
    }

  /* Deinit the rpmsg virtio device */

  rpmsg_deinit_vdev(&priv->rvdev);

  /* Delete the kthread */

  kthread_delete(priv->tid);

  /* Free the private data */

  kmm_free(priv);
}
