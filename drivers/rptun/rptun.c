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
#include <fcntl.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/rptun/rptun.h>
#include <metal/utilities.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MAX
#  define MAX(a,b)              ((a) > (b) ? (a) : (b))
#endif

#ifndef ALIGN_UP
#  define ALIGN_UP(s, a)        (((s) + (a) - 1) & ~((a) - 1))
#endif

#define RPTUNIOC_NONE           0

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rptun_priv_s
{
  FAR struct rptun_dev_s       *dev;
  struct remoteproc            rproc;
  struct rpmsg_virtio_device   vdev;
  struct rpmsg_virtio_shm_pool shm_pool;
  struct metal_list            bind;
  struct metal_list            node;
  sem_t                        sem;
  unsigned long                cmd;
};

struct rptun_bind_s
{
  char              name[RPMSG_NAME_SIZE];
  uint32_t          dest;
  struct metal_list node;
};

struct rptun_cb_s
{
  FAR void          *priv;
  rpmsg_dev_cb_t    device_created;
  rpmsg_dev_cb_t    device_destroy;
  rpmsg_bind_cb_t   ns_bind;
  struct metal_list node;
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
                                         FAR struct remoteproc_ops *ops,
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

static void rptun_ns_bind(FAR struct rpmsg_device *rdev,
                          FAR const char *name, uint32_t dest);

static int rptun_dev_start(FAR struct remoteproc *rproc);
static int rptun_dev_stop(FAR struct remoteproc *rproc);
static int rptun_dev_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

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

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct remoteproc_ops g_rptun_ops =
{
  .init    = rptun_init,
  .remove  = rptun_remove,
  .config  = rptun_config,
  .start   = rptun_start,
  .stop    = rptun_stop,
  .notify  = rptun_notify,
  .get_mem = rptun_get_mem,
};

static const struct file_operations g_rptun_devops =
{
  .ioctl = rptun_dev_ioctl,
};

#ifdef CONFIG_RPTUN_LOADER
static struct image_store_ops g_rptun_storeops =
{
  .open     = rptun_store_open,
  .close    = rptun_store_close,
  .load     = rptun_store_load,
  .features = SUPPORT_SEEK,
};
#endif

static sem_t g_rptun_sem = SEM_INITIALIZER(1);

static METAL_DECLARE_LIST(g_rptun_cb);
static METAL_DECLARE_LIST(g_rptun_priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rptun_thread(int argc, FAR char *argv[])
{
  FAR struct rptun_priv_s *priv;

  priv = (FAR struct rptun_priv_s *)((uintptr_t)strtoul(argv[2], NULL, 0));
  remoteproc_init(&priv->rproc, &g_rptun_ops, priv);

  while (1)
    {
      nxsem_wait_uninterruptible(&priv->sem);
      switch (priv->cmd)
        {
          case RPTUNIOC_START:
            if (priv->rproc.state == RPROC_OFFLINE)
              {
                rptun_dev_start(&priv->rproc);
              }
            break;

          case RPTUNIOC_STOP:
            if (priv->rproc.state != RPROC_OFFLINE)
              {
                rptun_dev_stop(&priv->rproc);
              }
            break;
        }

        priv->cmd = RPTUNIOC_NONE;
        remoteproc_get_notification(&priv->rproc, RPTUN_NOTIFY_ALL);
    }

  return 0;
}

static void rptun_wakeup(FAR struct rptun_priv_s *priv)
{
  int semcount;

  nxsem_get_value(&priv->sem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(&priv->sem);
    }
}

static int rptun_callback(FAR void *arg, uint32_t vqid)
{
  rptun_wakeup(arg);
  return OK;
}

static FAR struct remoteproc *rptun_init(FAR struct remoteproc *rproc,
                                         FAR struct remoteproc_ops *ops,
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

  RPTUN_NOTIFY(priv->dev, RPTUN_NOTIFY_ALL);

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
  strcpy(buf->name, name ? name : "");
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

static void *rptun_get_priv_by_rdev(FAR struct rpmsg_device *rdev)
{
  struct rpmsg_virtio_device *rvdev;
  struct virtio_device *vdev;
  struct remoteproc_virtio *rpvdev;
  struct remoteproc *rproc;

  rvdev = metal_container_of(rdev, struct rpmsg_virtio_device, rdev);
  vdev  = rvdev->vdev;
  if (!vdev)
    {
      return NULL;
    }

  rpvdev = metal_container_of(vdev, struct remoteproc_virtio, vdev);
  rproc  = rpvdev->priv;
  if (!rproc)
    {
      return NULL;
    }

  return rproc->priv;
}

static void rptun_ns_bind(FAR struct rpmsg_device *rdev,
                          FAR const char *name, uint32_t dest)
{
  FAR struct rptun_priv_s *priv = rptun_get_priv_by_rdev(rdev);
  FAR struct rptun_bind_s *bind;

  bind = kmm_malloc(sizeof(struct rptun_bind_s));
  if (bind)
    {
      FAR struct metal_list *node;
      FAR struct rptun_cb_s *cb;

      bind->dest = dest;
      strncpy(bind->name, name, RPMSG_NAME_SIZE);

      nxsem_wait(&g_rptun_sem);

      metal_list_add_tail(&priv->bind, &bind->node);

      metal_list_for_each(&g_rptun_cb, node)
        {
          cb = metal_container_of(node, struct rptun_cb_s, node);
          if (cb->ns_bind)
            {
              cb->ns_bind(rdev, cb->priv, name, dest);
            }
        }

      nxsem_post(&g_rptun_sem);
    }
}

static void rptun_ns_unbind(FAR struct rpmsg_device *rdev,
                            FAR const char *name, uint32_t dest)
{
  FAR struct rptun_priv_s *priv = rptun_get_priv_by_rdev(rdev);
  FAR struct metal_list *node;

  nxsem_wait(&g_rptun_sem);

  metal_list_for_each(&priv->bind, node)
    {
      struct rptun_bind_s *bind;

      bind = metal_container_of(node, struct rptun_bind_s, node);

      if (bind->dest == dest && !strncmp(bind->name, name, RPMSG_NAME_SIZE))
        {
          metal_list_del(node);
          kmm_free(bind);
          break;
        }
    }

  nxsem_post(&g_rptun_sem);
}

static int rptun_dev_start(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;
  FAR struct virtio_device *vdev;
  FAR struct rptun_rsc_s *rsc;
  FAR struct metal_list *node;
  FAR struct rptun_cb_s *cb;
  unsigned int role = RPMSG_REMOTE;
  int ret;

  ret = remoteproc_config(&priv->rproc, NULL);
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
                            &store, &g_rptun_storeops, NULL);
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

      align0 = B2C(rsc->rpmsg_vring0.align);
      align1 = B2C(rsc->rpmsg_vring1.align);

      tbsz = ALIGN_UP(sizeof(struct rptun_rsc_s), MAX(align0, align1));
      v0sz = ALIGN_UP(vring_size(rsc->rpmsg_vring0.num, align0), align0);
      v1sz = ALIGN_UP(vring_size(rsc->rpmsg_vring1.num, align1), align1);

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
      shbufsz = rsc->config.txbuf_size * rsc->rpmsg_vring0.num +
                rsc->config.rxbuf_size * rsc->rpmsg_vring1.num;

      rpmsg_virtio_init_shm_pool(&priv->shm_pool, shbuf, shbufsz);

      role = RPMSG_MASTER;
    }

  /* Remote proc create */

  vdev = remoteproc_create_virtio(rproc, 0, role, NULL);
  if (!vdev)
    {
      return -ENOMEM;
    }

  ret = rpmsg_init_vdev(&priv->vdev, vdev, rptun_ns_bind,
                        metal_io_get_region(), &priv->shm_pool);
  if (ret)
    {
      remoteproc_remove_virtio(rproc, vdev);
      return ret;
    }

  priv->vdev.rdev.ns_unbind_cb = rptun_ns_unbind;

  /* Remote proc start */

  ret = remoteproc_start(rproc);
  if (ret)
    {
      remoteproc_remove_virtio(rproc, vdev);
      return ret;
    }

  nxsem_wait(&g_rptun_sem);

  /* Add priv to list */

  metal_list_add_tail(&g_rptun_priv, &priv->node);

  /* Broadcast device_created to all registers */

  metal_list_for_each(&g_rptun_cb, node)
    {
      cb = metal_container_of(node, struct rptun_cb_s, node);
      if (cb->device_created)
        {
          cb->device_created(&priv->vdev.rdev, cb->priv);
        }
    }

  nxsem_post(&g_rptun_sem);

  /* Register callback to mbox for receiving remote message */

  RPTUN_REGISTER_CALLBACK(priv->dev, rptun_callback, priv);

  return 0;
}

static int rptun_dev_stop(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;
  FAR struct metal_list *node;
  FAR struct rptun_cb_s *cb;

  /* Unregister callback from mbox */

  RPTUN_UNREGISTER_CALLBACK(priv->dev);

  nxsem_wait(&g_rptun_sem);

  /* Remove priv from list */

  metal_list_del(&priv->node);

  /* Broadcast device_destroy to all registers */

  metal_list_for_each(&g_rptun_cb, node)
    {
      cb = metal_container_of(node, struct rptun_cb_s, node);
      if (cb->device_destroy)
        {
          cb->device_destroy(&priv->vdev.rdev, cb->priv);
        }
    }

  nxsem_post(&g_rptun_sem);

  /* Remote proc stop and shutdown */

  remoteproc_shutdown(rproc);

  /* Remote proc remove */

  remoteproc_remove_virtio(rproc, priv->vdev.vdev);
  rpmsg_deinit_vdev(&priv->vdev);

  return 0;
}

static int rptun_dev_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rptun_priv_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      case RPTUNIOC_START:
      case RPTUNIOC_STOP:
        priv->cmd = cmd;
        rptun_wakeup(priv);
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

#ifdef CONFIG_RPTUN_LOADER
static int rptun_store_open(FAR void *store_,
                            FAR const char *path,
                            FAR const void **img_data)
{
  FAR struct rptun_store_s *store = store_;
  int len = 0x100;
  int ret;

  ret = file_open(&store->file, path, O_RDONLY);
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

FAR const char *rpmsg_get_cpuname(FAR struct rpmsg_device *rdev)
{
  FAR struct rptun_priv_s *priv = rptun_get_priv_by_rdev(rdev);

  return RPTUN_GET_CPUNAME(priv->dev);
}

int rpmsg_register_callback(FAR void *priv_,
                            rpmsg_dev_cb_t device_created,
                            rpmsg_dev_cb_t device_destroy,
                            rpmsg_bind_cb_t ns_bind)
{
  FAR struct metal_list *node;
  FAR struct metal_list *bnode;
  FAR struct rptun_cb_s *cb;

  cb = kmm_zalloc(sizeof(struct rptun_cb_s));
  if (!cb)
    {
      return -ENOMEM;
    }

  cb->priv           = priv_;
  cb->device_created = device_created;
  cb->device_destroy = device_destroy;
  cb->ns_bind        = ns_bind;

  nxsem_wait(&g_rptun_sem);

  metal_list_add_tail(&g_rptun_cb, &cb->node);

  metal_list_for_each(&g_rptun_priv, node)
    {
      struct rptun_priv_s *priv;

      priv = metal_container_of(node, struct rptun_priv_s, node);
      if (device_created)
        {
          device_created(&priv->vdev.rdev, priv_);
        }

      if (ns_bind)
        {
          metal_list_for_each(&priv->bind, bnode)
            {
              struct rptun_bind_s *bind;

              bind = metal_container_of(bnode, struct rptun_bind_s, node);
              ns_bind(&priv->vdev.rdev, priv_, bind->name, bind->dest);
            }
        }
    }

  nxsem_post(&g_rptun_sem);

  return 0;
}

void rpmsg_unregister_callback(FAR void *priv_,
                               rpmsg_dev_cb_t device_created,
                               rpmsg_dev_cb_t device_destroy,
                               rpmsg_bind_cb_t ns_bind)
{
  FAR struct metal_list *node;
  FAR struct metal_list *pnode;

  nxsem_wait(&g_rptun_sem);

  metal_list_for_each(&g_rptun_cb, node)
    {
      struct rptun_cb_s *cb = NULL;

      cb = metal_container_of(node, struct rptun_cb_s, node);
      if (cb->priv == priv_ &&
          cb->device_created == device_created &&
          cb->device_destroy == device_destroy &&
          cb->ns_bind == ns_bind)
        {
          if (device_destroy)
            {
              metal_list_for_each(&g_rptun_priv, pnode)
                {
                  struct rptun_priv_s *priv;

                  priv = metal_container_of(pnode,
                                            struct rptun_priv_s, node);
                  device_destroy(&priv->vdev.rdev, priv_);
                }
            }

          metal_list_del(&cb->node);
          kmm_free(cb);

          break;
        }
    }

  nxsem_post(&g_rptun_sem);
}

int rptun_initialize(FAR struct rptun_dev_s *dev)
{
  struct metal_init_params params = METAL_INIT_DEFAULTS;
  FAR struct rptun_priv_s *priv;
  FAR char *argv[3];
  char arg1[16];
  char name[32];
  int ret;

  ret = metal_init(&params);
  if (ret < 0)
    {
      return ret;
    }

  priv = kmm_zalloc(sizeof(struct rptun_priv_s));
  if (priv == NULL)
    {
      ret = -ENOMEM;
      goto err_mem;
    }

  priv->dev = dev;
  if (RPTUN_IS_AUTOSTART(dev))
    {
      priv->cmd = RPTUNIOC_START;
    }

  metal_list_init(&priv->bind);
  nxsem_init(&priv->sem, 0, RPTUN_IS_AUTOSTART(dev) ? 1 : 0);
  nxsem_set_protocol(&priv->sem, SEM_PRIO_NONE);

  snprintf(name, 32, "/dev/rptun/%s", RPTUN_GET_CPUNAME(dev));
  ret = register_driver(name, &g_rptun_devops, 0666, priv);
  if (ret < 0)
    {
      goto err_driver;
    }

  snprintf(arg1, 16, "0x%" PRIxPTR, (uintptr_t)priv);
  argv[0] = (void *)RPTUN_GET_CPUNAME(dev);
  argv[1] = arg1;
  argv[2] = NULL;

  ret = kthread_create("rptun",
                       CONFIG_RPTUN_PRIORITY,
                       CONFIG_RPTUN_STACKSIZE,
                       rptun_thread,
                       argv);
  if (ret < 0)
    {
      goto err_thread;
    }

  return OK;

err_thread:
  unregister_driver(name);

err_driver:
  nxsem_destroy(&priv->sem);
  kmm_free(priv);

err_mem:
  metal_finish();
  return ret;
}

int rptun_boot(FAR const char *cpuname)
{
  struct file file;
  char name[32];
  int ret;

  if (!cpuname)
    {
      return -EINVAL;
    }

  snprintf(name, 32, "/dev/rptun/%s", cpuname);
  ret = file_open(&file, name, 0, 0);
  if (ret)
    {
      return ret;
    }

  ret = file_ioctl(&file, RPTUNIOC_START, 0);
  file_close(&file);

  return ret;
}
