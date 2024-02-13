/****************************************************************************
 * drivers/rptun/rptun.c
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

#include <inttypes.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/param.h>
#include <fcntl.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/power/pm.h>
#include <nuttx/wqueue.h>
#include <metal/utilities.h>
#include <openamp/remoteproc_loader.h>
#include <openamp/remoteproc_virtio.h>

#include "rptun.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ALIGN_UP
#  define ALIGN_UP(s, a)            (((s) + (a) - 1) & ~((a) - 1))
#endif

#define RPTUNIOC_NONE               0

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rptun_priv_s
{
  struct rpmsg_s               rpmsg;
  struct rpmsg_virtio_device   rvdev;
  FAR struct rptun_dev_s       *dev;
  struct remoteproc            rproc;
  struct rpmsg_virtio_shm_pool pool[2];
  sem_t                        semtx;
  sem_t                        semrx;
  pid_t                        tid;
#ifdef CONFIG_RPTUN_PM
  bool                         stay;
#endif
};

struct rptun_store_s
{
  struct file file;
  FAR char   *buf;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR struct remoteproc *rptun_init(FAR struct remoteproc *rproc,
                                        FAR const struct remoteproc_ops *ops,
                                         FAR void *arg);
static void rptun_remove(FAR struct remoteproc *rproc);
static int rptun_config(struct remoteproc *rproc, void *data);
static int rptun_start(FAR struct remoteproc *rproc);
static int rptun_stop(FAR struct remoteproc *rproc);
static int rptun_notify(FAR struct remoteproc *rproc, uint32_t id);
static FAR struct remoteproc_mem *
rptun_get_mem(FAR struct remoteproc *rproc,
              FAR const char *name,
              metal_phys_addr_t pa,
              metal_phys_addr_t da,
              FAR void *va, size_t size,
              FAR struct remoteproc_mem *buf);
static int rptun_notify_wait(FAR struct remoteproc *rproc, uint32_t id);

static int rptun_dev_start(FAR struct remoteproc *rproc);
static int rptun_dev_stop(FAR struct remoteproc *rproc, bool stop_ns);

#ifdef CONFIG_RPTUN_LOADER
static int rptun_store_open(FAR void *store_, FAR const char *path,
                            FAR const void **img_data);
static void rptun_store_close(FAR void *store_);
static int rptun_store_load(FAR void *store_, size_t offset,
                            size_t size, FAR const void **data,
                            metal_phys_addr_t pa,
                            FAR struct metal_io_region *io,
                            char is_blocking);
#endif

static metal_phys_addr_t rptun_pa_to_da(FAR struct rptun_dev_s *dev,
                                        metal_phys_addr_t pa);
static metal_phys_addr_t rptun_da_to_pa(FAR struct rptun_dev_s *dev,
                                        metal_phys_addr_t da);

static int rptun_wait(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem);
static int rptun_post(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem);
static int rptun_ioctl(FAR struct rpmsg_s *rpmsg, int cmd,
                       unsigned long arg);
static FAR const char *rptun_get_cpuname(FAR struct rpmsg_s *rpmsg);
static int rptun_get_tx_buffer_size(FAR struct rpmsg_s *rpmsg);
static int rptun_get_rx_buffer_size(FAR struct rpmsg_s *rpmsg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct remoteproc_ops g_rptun_ops =
{
  .init        = rptun_init,
  .remove      = rptun_remove,
  .config      = rptun_config,
  .start       = rptun_start,
  .stop        = rptun_stop,
  .notify      = rptun_notify,
  .get_mem     = rptun_get_mem,
  .notify_wait = rptun_notify_wait,
};

#ifdef CONFIG_RPTUN_LOADER
static const struct image_store_ops g_rptun_store_ops =
{
  .open     = rptun_store_open,
  .close    = rptun_store_close,
  .load     = rptun_store_load,
  .features = SUPPORT_SEEK,
};
#endif

static const struct rpmsg_ops_s g_rptun_rpmsg_ops =
{
  rptun_wait,
  rptun_post,
  rptun_ioctl,
  rptun_get_cpuname,
  rptun_get_tx_buffer_size,
  rptun_get_rx_buffer_size,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_RPTUN_PM
static inline void rptun_pm_action(FAR struct rptun_priv_s *priv,
                                   bool stay)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (stay && !priv->stay)
    {
      pm_stay(PM_IDLE_DOMAIN, PM_IDLE);
      priv->stay = true;
    }

  if (!stay && priv->stay && !rptun_buffer_nused(&priv->rvdev, false))
    {
      pm_relax(PM_IDLE_DOMAIN, PM_IDLE);
      priv->stay = false;
    }

  leave_critical_section(flags);
}

#else
#  define rptun_pm_action(priv, stay)
#endif

static void rptun_start_worker(FAR void *arg)
{
  FAR struct rptun_priv_s *priv = arg;

  if (priv->rproc.state == RPROC_OFFLINE)
    {
      rptun_dev_start(&priv->rproc);
    }
}

static void rptun_worker(FAR void *arg)
{
  FAR struct rptun_priv_s *priv = arg;

  remoteproc_get_notification(&priv->rproc, RPTUN_NOTIFY_ALL);
}

static int rptun_thread(int argc, FAR char *argv[])
{
  FAR struct rptun_priv_s *priv;

  priv = (FAR struct rptun_priv_s *)((uintptr_t)strtoul(argv[2], NULL, 16));
  priv->tid = nxsched_gettid();

  if (RPTUN_IS_AUTOSTART(priv->dev))
    {
      rptun_start_worker(priv);
    }

  while (1)
    {
      nxsem_wait_uninterruptible(&priv->semrx);
      rptun_worker(priv);
    }

  return 0;
}

static void rptun_wakeup_rx(FAR struct rptun_priv_s *priv)
{
  int semcount;

  nxsem_get_value(&priv->semrx, &semcount);
  if (semcount < 1)
    {
      nxsem_post(&priv->semrx);
    }
}

static bool rptun_is_recursive(FAR struct rptun_priv_s *priv)
{
  return nxsched_gettid() == priv->tid;
}

static void rptun_wakeup_tx(FAR struct rptun_priv_s *priv)
{
  int semcount;

  nxsem_get_value(&priv->semtx, &semcount);
  while (semcount++ < 1)
    {
      nxsem_post(&priv->semtx);
    }
}

static int rptun_callback(FAR void *arg, uint32_t vqid)
{
  FAR struct rptun_priv_s *priv = arg;
  FAR struct rpmsg_virtio_device *rvdev = &priv->rvdev;
  FAR struct virtio_device *vdev = rvdev->vdev;
  FAR struct virtqueue *svq = rvdev->svq;
  FAR struct virtqueue *rvq = rvdev->rvq;

  if (vqid == RPTUN_NOTIFY_ALL ||
      vqid == vdev->vrings_info[rvq->vq_queue_index].notifyid)
    {
      if (rptun_buffer_nused(&priv->rvdev, true))
        {
          rptun_wakeup_rx(priv);
        }
    }

  if (vqid == RPTUN_NOTIFY_ALL ||
      vqid == vdev->vrings_info[svq->vq_queue_index].notifyid)
    {
      rptun_wakeup_tx(priv);
      rptun_pm_action(priv, false);
    }

  return OK;
}

static FAR struct remoteproc *rptun_init(FAR struct remoteproc *rproc,
                                        FAR const struct remoteproc_ops *ops,
                                         FAR void *arg)
{
  rproc->ops = ops;
  rproc->priv = arg;

  return rproc;
}

static void rptun_remove(FAR struct remoteproc *rproc)
{
  rproc->priv = NULL;
}

static int rptun_config(struct remoteproc *rproc, void *data)
{
  struct rptun_priv_s *priv = rproc->priv;

  if (RPTUN_IS_MASTER(priv->dev))
    {
      return RPTUN_CONFIG(priv->dev, data);
    }

  return 0;
}

static int rptun_start(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;

  if (RPTUN_IS_MASTER(priv->dev))
    {
      return RPTUN_START(priv->dev);
    }

  return 0;
}

static int rptun_stop(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;

  if (RPTUN_IS_MASTER(priv->dev))
    {
      return RPTUN_STOP(priv->dev);
    }

  return 0;
}

static int rptun_notify(FAR struct remoteproc *rproc, uint32_t id)
{
  FAR struct rptun_priv_s *priv = rproc->priv;
  FAR struct rpmsg_virtio_device *rvdev = &priv->rvdev;
  FAR struct virtqueue *vq = rvdev->svq;

  if (rvdev->vdev && vq &&
      rvdev->vdev->vrings_info[vq->vq_queue_index].notifyid == id)
    {
      rptun_pm_action(priv, true);
    }

  RPTUN_NOTIFY(priv->dev, id);
  return 0;
}

static FAR struct remoteproc_mem *
rptun_get_mem(FAR struct remoteproc *rproc,
              FAR const char *name,
              metal_phys_addr_t pa,
              metal_phys_addr_t da,
              FAR void *va, size_t size,
              FAR struct remoteproc_mem *buf)
{
  FAR struct rptun_priv_s *priv = rproc->priv;

  metal_list_init(&buf->node);
  strlcpy(buf->name, name ? name : "", RPROC_MAX_NAME_LEN);
  buf->io = metal_io_get_region();
  buf->size = size;

  if (pa != METAL_BAD_PHYS)
    {
      buf->pa = pa;
      buf->da = rptun_pa_to_da(priv->dev, pa);
    }
  else if (da != METAL_BAD_PHYS)
    {
      buf->pa = rptun_da_to_pa(priv->dev, da);
      buf->da = da;
    }
  else
    {
      buf->pa = metal_io_virt_to_phys(buf->io, va);
      buf->da = rptun_pa_to_da(priv->dev, buf->pa);
    }

  if (buf->pa == METAL_BAD_PHYS || buf->da == METAL_BAD_PHYS)
    {
      return NULL;
    }

  return buf;
}

static int rptun_notify_wait(FAR struct remoteproc *rproc, uint32_t id)
{
  FAR struct rptun_priv_s *priv = rproc->priv;

  if (!rptun_is_recursive(priv))
    {
      return -EAGAIN;
    }

  /* Wait to wakeup */

  nxsem_wait(&priv->semtx);
  rptun_worker(priv);

  return 0;
}

static int rptun_wait(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem)
{
  FAR struct rptun_priv_s *priv = (FAR struct rptun_priv_s *)rpmsg;
  int ret;

  if (!rptun_is_recursive(priv))
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
      rptun_worker(priv);
    }

  return ret;
}

static int rptun_post(FAR struct rpmsg_s *rpmsg, FAR sem_t *sem)
{
  FAR struct rptun_priv_s *priv = (FAR struct rptun_priv_s *)rpmsg;
  int semcount;
  int ret;

  nxsem_get_value(sem, &semcount);
  ret = nxsem_post(sem);

  if (priv && semcount >= 0)
    {
      rptun_wakeup_tx(priv);
    }

  return ret;
}

static int rptun_ioctl(FAR struct rpmsg_s *rpmsg, int cmd, unsigned long arg)
{
  FAR struct rptun_priv_s *priv = (FAR struct rptun_priv_s *)rpmsg;
  int ret = OK;

  switch (cmd)
    {
      case RPMSGIOC_START:
        if (priv->rproc.state == RPROC_OFFLINE)
          {
            ret = rptun_dev_start(&priv->rproc);
          }
        else
          {
            ret = rptun_dev_stop(&priv->rproc, false);
            if (ret == OK)
              {
                ret = rptun_dev_start(&priv->rproc);
              }
          }
        break;
      case RPMSGIOC_STOP:
        ret = rptun_dev_stop(&priv->rproc, true);
        break;
      case RPMSGIOC_RESET:
        RPTUN_RESET(priv->dev, arg);
        break;
      case RPMSGIOC_PANIC:
        RPTUN_PANIC(priv->dev);
        break;
      case RPMSGIOC_DUMP:
        rptun_dump(&priv->rvdev);
#ifdef CONFIG_RPTUN_PM
        metal_log(METAL_LOG_EMERGENCY, "rptun headrx %d\n", priv->headrx);
#endif
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static FAR const char *rptun_get_cpuname(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rptun_priv_s *priv = (FAR struct rptun_priv_s *)rpmsg;

  return RPTUN_GET_CPUNAME(priv->dev);
}

static int rptun_get_tx_buffer_size(FAR struct rpmsg_s *rpmsg)
{
  return rpmsg_virtio_get_buffer_size(rpmsg->rdev);
}

static int rptun_get_rx_buffer_size(FAR struct rpmsg_s *rpmsg)
{
  return rpmsg_virtio_get_rx_buffer_size(rpmsg->rdev);
}

static int rptun_dev_start(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;
  FAR struct virtio_device *vdev;
  FAR struct rptun_rsc_s *rsc;
  unsigned int role = RPMSG_REMOTE;
  int ret;

  ret = remoteproc_config(rproc, NULL);
  if (ret)
    {
      return ret;
    }

#ifdef CONFIG_RPTUN_LOADER
  if (RPTUN_GET_FIRMWARE(priv->dev))
    {
      struct rptun_store_s store =
      {
        0
      };

      ret = remoteproc_load(rproc, RPTUN_GET_FIRMWARE(priv->dev),
                            &store, &g_rptun_store_ops, NULL);
      if (ret)
        {
          return ret;
        }

      rsc = rproc->rsc_table;
    }
  else
#endif
    {
      rsc = RPTUN_GET_RESOURCE(priv->dev);
      if (!rsc)
        {
          return -EINVAL;
        }

      ret = remoteproc_set_rsc_table(rproc, (struct resource_table *)rsc,
                                     sizeof(struct rptun_rsc_s));
      if (ret)
        {
          return ret;
        }
    }

  /* Update resource table on MASTER side */

  if (RPTUN_IS_MASTER(priv->dev))
    {
      uint32_t tbsz;
      uint32_t v0sz;
      uint32_t v1sz;
      uint32_t shbufsz;
      metal_phys_addr_t da0;
      metal_phys_addr_t da1;
      uint32_t align0;
      uint32_t align1;
      FAR void *va0;
      FAR void *va1;
      FAR void *shbuf;
      FAR struct metal_io_region *io;
      metal_phys_addr_t pa0;
      metal_phys_addr_t pa1;

      align0 = rsc->rpmsg_vring0.align;
      align1 = rsc->rpmsg_vring1.align;

      v0sz = ALIGN_UP(vring_size(rsc->rpmsg_vring0.num, align0), align0);
      v1sz = ALIGN_UP(vring_size(rsc->rpmsg_vring1.num, align1), align1);

      if (rsc->rpmsg_vring0.da == 0 || rsc->rpmsg_vring1.da == 0)
        {
          tbsz = ALIGN_UP(sizeof(struct rptun_rsc_s), MAX(align0, align1));

          va0 = (FAR char *)rsc + tbsz;
          va1 = (FAR char *)rsc + tbsz + v0sz;

          io  = metal_io_get_region();
          pa0 = metal_io_virt_to_phys(io, va0);
          pa1 = metal_io_virt_to_phys(io, va1);

          da0 = da1 = METAL_BAD_PHYS;

          remoteproc_mmap(rproc, &pa0, &da0, v0sz, 0, NULL);
          remoteproc_mmap(rproc, &pa1, &da1, v1sz, 0, NULL);

          rsc->rpmsg_vring0.da = da0;
          rsc->rpmsg_vring1.da = da1;

          shbuf   = (FAR char *)rsc + tbsz + v0sz + v1sz;
          shbufsz = rsc->config.r2h_buf_size * rsc->rpmsg_vring0.num +
                    rsc->config.h2r_buf_size * rsc->rpmsg_vring1.num;

          rpmsg_virtio_init_shm_pool(priv->pool, shbuf, shbufsz);
        }
      else
        {
          da0 = rsc->rpmsg_vring0.da;
          shbuf = (FAR char *)remoteproc_mmap(rproc, NULL, &da0,
                                              v0sz, 0, NULL) + v0sz;
          shbufsz = rsc->config.r2h_buf_size * rsc->rpmsg_vring0.num;
          rpmsg_virtio_init_shm_pool(&priv->pool[0], shbuf, shbufsz);

          da1 = rsc->rpmsg_vring1.da;
          shbuf = (FAR char *)remoteproc_mmap(rproc, NULL, &da1,
                                              v1sz, 0, NULL) + v1sz;
          shbufsz = rsc->config.h2r_buf_size * rsc->rpmsg_vring1.num;
          rpmsg_virtio_init_shm_pool(&priv->pool[1], shbuf, shbufsz);
        }

      role = RPMSG_HOST;
    }

  /* Remote proc create */

  vdev = remoteproc_create_virtio(rproc, 0, role, NULL);
  if (!vdev)
    {
      return -ENOMEM;
    }

  if (priv->pool[1].base)
    {
      struct rpmsg_virtio_config config =
        {
          RPMSG_BUFFER_SIZE,
          RPMSG_BUFFER_SIZE,
          true,
        };

      ret = rpmsg_init_vdev_with_config(&priv->rvdev, vdev, rpmsg_ns_bind,
                                        metal_io_get_region(),
                                        priv->pool,
                                        &config);
    }
  else
    {
      ret = rpmsg_init_vdev(&priv->rvdev, vdev, rpmsg_ns_bind,
                            metal_io_get_region(), priv->pool);
    }

  if (ret)
    {
      remoteproc_remove_virtio(rproc, vdev);
      return ret;
    }

  priv->rvdev.rdev.ns_unbind_cb = rpmsg_ns_unbind;

  /* Remote proc start */

  ret = remoteproc_start(rproc);
  if (ret)
    {
      rpmsg_deinit_vdev(&priv->rvdev);
      remoteproc_remove_virtio(rproc, vdev);
      remoteproc_shutdown(rproc);
      return ret;
    }

  /* Register callback to mbox for receiving remote message */

  RPTUN_REGISTER_CALLBACK(priv->dev, rptun_callback, priv);
  rptun_wakeup_rx(priv);

  /* Broadcast device_created to all registers */

  rpmsg_device_created(&priv->rpmsg);

  /* Open tx buffer return callback */

  virtqueue_enable_cb(priv->rvdev.svq);

  return 0;
}

static int rptun_dev_stop(FAR struct remoteproc *rproc, bool stop_ns)
{
  FAR struct rptun_priv_s *priv = rproc->priv;
  FAR struct rpmsg_device *rdev = &priv->rvdev.rdev;

  if (priv->rproc.state == RPROC_OFFLINE)
    {
      return OK;
    }
  else if (priv->rproc.state == RPROC_CONFIGURED ||
           priv->rproc.state == RPROC_READY)
    {
      return -EBUSY;
    }

  rdev->support_ns = stop_ns;

  /* Unregister callback from mbox */

  RPTUN_UNREGISTER_CALLBACK(priv->dev);

  rpmsg_device_destory(&priv->rpmsg);

  /* Remote proc remove */

  rpmsg_deinit_vdev(&priv->rvdev);
  remoteproc_remove_virtio(rproc, priv->rvdev.vdev);

  /* Remote proc stop and shutdown */

  remoteproc_shutdown(rproc);

  return OK;
}

#ifdef CONFIG_RPTUN_LOADER
static int rptun_store_open(FAR void *store_,
                            FAR const char *path,
                            FAR const void **img_data)
{
  FAR struct rptun_store_s *store = store_;
  int len = 0x100;
  int ret;

  ret = file_open(&store->file, path, O_RDONLY | O_CLOEXEC);
  if (ret < 0)
    {
      return ret;
    }

  store->buf = kmm_malloc(len);
  if (!store->buf)
    {
      file_close(&store->file);
      return -ENOMEM;
    }

  *img_data = store->buf;

  return file_read(&store->file, store->buf, len);
}

static void rptun_store_close(FAR void *store_)
{
  FAR struct rptun_store_s *store = store_;

  kmm_free(store->buf);
  file_close(&store->file);
}

static int rptun_store_load(FAR void *store_, size_t offset,
                            size_t size, FAR const void **data,
                            metal_phys_addr_t pa,
                            FAR struct metal_io_region *io,
                            char is_blocking)
{
  FAR struct rptun_store_s *store = store_;
  FAR char *tmp;

  if (pa == METAL_BAD_PHYS)
    {
      tmp = kmm_realloc(store->buf, size);
      if (!tmp)
        {
          return -ENOMEM;
        }

      store->buf = tmp;
      *data = tmp;
    }
  else
    {
      tmp = metal_io_phys_to_virt(io, pa);
      if (!tmp)
        {
          return -EINVAL;
        }
    }

  file_seek(&store->file, offset, SEEK_SET);
  return file_read(&store->file, tmp, size);
}
#endif

static metal_phys_addr_t rptun_pa_to_da(FAR struct rptun_dev_s *dev,
                                        metal_phys_addr_t pa)
{
  FAR const struct rptun_addrenv_s *addrenv;
  uint32_t i;

  addrenv = RPTUN_GET_ADDRENV(dev);
  if (!addrenv)
    {
      return pa;
    }

  for (i = 0; addrenv[i].size; i++)
    {
      if (pa - addrenv[i].pa < addrenv[i].size)
        {
          return addrenv[i].da + (pa - addrenv[i].pa);
        }
    }

  return pa;
}

static metal_phys_addr_t rptun_da_to_pa(FAR struct rptun_dev_s *dev,
                                        metal_phys_addr_t da)
{
  FAR const struct rptun_addrenv_s *addrenv;
  uint32_t i;

  addrenv = RPTUN_GET_ADDRENV(dev);
  if (!addrenv)
    {
      return da;
    }

  for (i = 0; addrenv[i].size; i++)
    {
      if (da - addrenv[i].da < addrenv[i].size)
        {
          return addrenv[i].pa + (da - addrenv[i].da);
        }
    }

  return da;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rptun_initialize(FAR struct rptun_dev_s *dev)
{
  struct metal_init_params params = METAL_INIT_DEFAULTS;
  FAR struct rptun_priv_s *priv;
  static bool onceinit = false;
  FAR char *argv[3];
  char arg1[19];
  char name[32];
  int ret;

  if (!onceinit)
    {
      ret = metal_init(&params);
      if (ret < 0)
        {
          return ret;
        }

      onceinit = true;
    }

  priv = kmm_zalloc(sizeof(struct rptun_priv_s));
  if (priv == NULL)
    {
      ret = -ENOMEM;
      goto err_mem;
    }

  priv->dev = dev;

  remoteproc_init(&priv->rproc, &g_rptun_ops, priv);

  snprintf(name, sizeof(name), "/dev/rptun/%s", RPTUN_GET_CPUNAME(dev));
  ret = rpmsg_register(name, &priv->rpmsg, &g_rptun_rpmsg_ops);
  if (ret < 0)
    {
      goto err_driver;
    }

  nxsem_init(&priv->semtx, 0, 0);
  nxsem_init(&priv->semrx, 0, 0);
  snprintf(arg1, sizeof(arg1), "0x%" PRIxPTR, (uintptr_t)priv);
  argv[0] = (void *)RPTUN_GET_CPUNAME(dev);
  argv[1] = arg1;
  argv[2] = NULL;

  ret = kthread_create("rptun", CONFIG_RPTUN_PRIORITY,
                       CONFIG_RPTUN_STACKSIZE, rptun_thread, argv);
  if (ret < 0)
    {
      goto err_thread;
    }

  /* Add priv to list */

  return OK;

err_thread:
  nxsem_destroy(&priv->semtx);
  nxsem_destroy(&priv->semrx);
  rpmsg_unregister(name, &priv->rpmsg);

err_driver:
  kmm_free(priv);

err_mem:
  metal_finish();
  return ret;
}

int rptun_boot(FAR const char *cpuname)
{
  return rpmsg_ioctl(cpuname, RPMSGIOC_START, 0);
}

int rptun_poweroff(FAR const char *cpuname)
{
  return rpmsg_ioctl(cpuname, RPMSGIOC_STOP, 0);
}

int rptun_reset(FAR const char *cpuname, int value)
{
  return rpmsg_ioctl(cpuname, RPMSGIOC_RESET, value);
}

int rptun_panic(FAR const char *cpuname)
{
  return rpmsg_ioctl(cpuname, RPMSGIOC_PANIC, 0);
}

int rptun_buffer_nused(FAR struct rpmsg_virtio_device *rvdev, bool rx)
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

void rptun_dump_all(void)
{
  rpmsg_ioctl(NULL, RPMSGIOC_DUMP, 0);
}
