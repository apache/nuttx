/****************************************************************************
 * drivers/rptun/rptun.c
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
#include <fcntl.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/boardctl.h>
#include <sys/param.h>
#include <sys/wait.h>

#include <metal/utilities.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mm/mm.h>
#include <nuttx/nuttx.h>
#include <nuttx/rpmsg/rpmsg_virtio.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/vhost/vhost.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/panic_notifier.h>
#include <openamp/remoteproc_loader.h>
#include <openamp/remoteproc_virtio.h>
#include <openamp/rsc_table_parser.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPTUN_RETRY_PERIOD_US  1000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rptun_carveout_s
{
  FAR struct mm_heap_s       *heap;
  FAR void                   *base;
  size_t                      size;
};

struct rptun_priv_s
{
  FAR struct rptun_dev_s      *dev;
  struct remoteproc           rproc;
  struct metal_list           node;
  struct notifier_block       nbpanic;
  bool                        rpanic;
  bool                        stop;
  pid_t                       pid;
};

struct rptun_store_s
{
  struct file file;
  FAR char   *buf;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR struct remoteproc *
rptun_init(FAR struct remoteproc *rproc,
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

static int rptun_dev_start(FAR struct rptun_priv_s *priv);
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

static FAR void *rptun_alloc_buf(FAR struct virtio_device *vdev,
                                 size_t size, size_t align);
static void rptun_free_buf(FAR struct virtio_device *vdev, FAR void *buf);

static void rptun_send_command(FAR struct rptun_priv_s *priv,
                               uint32_t cmd, bool wait);
static uint32_t rptun_recv_command(FAR struct rptun_priv_s *priv, bool ack);

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
};

static const struct file_operations g_rptun_fops =
{
  NULL,             /* open */
  NULL,             /* close */
  NULL,             /* read */
  NULL,             /* write */
  NULL,             /* seek */
  rptun_dev_ioctl,  /* ioctl */
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

static const struct virtio_memory_ops g_rptun_mmops =
{
  .alloc = rptun_alloc_buf,
  .free  = rptun_free_buf,
};

static struct metal_list g_rptun_priv = METAL_INIT_LIST(g_rptun_priv);
static metal_mutex_t g_rptun_lock = METAL_MUTEX_INIT(g_rptun_lock);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR struct remoteproc *
rptun_init(FAR struct remoteproc *rproc,
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

/****************************************************************************
 * Name: rptun_alloc_buf
 ****************************************************************************/

static FAR void *rptun_alloc_buf(FAR struct virtio_device *vdev,
                                 size_t size, size_t align)
{
  FAR struct rptun_carveout_s *carveout =
    (FAR struct rptun_carveout_s *)vdev->mm_priv;

  return mm_memalign(carveout->heap, align, size);
}

/****************************************************************************
 * Name: rptun_free_buf
 ****************************************************************************/

static void rptun_free_buf(FAR struct virtio_device *vdev, FAR void *buf)
{
  FAR struct rptun_carveout_s *carveout =
    (FAR struct rptun_carveout_s *)vdev->mm_priv;

  mm_free(carveout->heap, buf);
}

/****************************************************************************
 * Name: rptun_init_carveout
 ****************************************************************************/

static int rptun_init_carveout(FAR struct rptun_priv_s *priv,
                               FAR struct virtio_device *vdev,
                               FAR const char *shmname,
                               FAR void *shmbase, size_t shmlen)
{
  FAR struct rptun_carveout_s *carveout;
  struct mm_heap_config_s config;

  if (vdev->role == VIRTIO_DEV_DEVICE)
    {
      return OK;
    }

  carveout = kmm_zalloc(sizeof(*carveout));
  if (carveout == NULL)
    {
      return -ENOMEM;
    }

  memset(&config, 0, sizeof(config));
  config.heap    = KRN_HEAP;
  config.name    = shmname;
  config.start   = shmbase;
  config.size    = shmlen;
  config.nokasan = true;

  carveout->base = shmbase;
  carveout->size = shmlen;
  carveout->heap = mm_initialize_heap(&config);
  if (carveout->heap == NULL)
    {
      rptunerr("ERROR: Failed to initialize heap\n");
      kmm_free(carveout);
      return -ENOMEM;
    }

  vdev->mmops = &g_rptun_mmops;
  vdev->mm_priv = carveout;

  rptuninfo("caveouts=%p heap=%p name=%s base=%p size=%zu\n",
            carveout, carveout->heap, shmname, shmbase, shmlen);
  return OK;
}

/****************************************************************************
 * Name: rptun_uninit_carveout
 ****************************************************************************/

static void rptun_uninit_carveout(FAR struct virtio_device *vdev)
{
  FAR struct rptun_carveout_s *carveout = vdev->mm_priv;

  if (carveout == NULL || vdev->role == VIRTIO_DEV_DEVICE)
    {
      return;
    }

  mm_uninitialize(carveout->heap);
  kmm_free(carveout);
}

/****************************************************************************
 * Name: rptun_get_carveout_memory
 ****************************************************************************/

static FAR void *
rptun_get_carveout_memory(FAR struct rptun_priv_s *priv,
                          FAR struct fw_rsc_carveout *carveout,
                          FAR size_t *size)
{
  metal_phys_addr_t da = carveout->da;

  *size = carveout->len;
  return remoteproc_mmap(&priv->rproc, NULL, &da, carveout->len, 0, NULL);
}

static void rptun_update_vring_da(FAR struct remoteproc *rproc,
                                  FAR struct fw_rsc_vdev *vdev_rsc,
                                  unsigned int role, FAR char **shmbase,
                                  size_t *shmlen)
{
  uint8_t i;

  if (role != VIRTIO_DEV_DRIVER)
    {
      return;
    }

  /* Calculate the da of all vrings and assign back to the resource table */

  for (i = 0; i < vdev_rsc->num_of_vrings; i++)
    {
      FAR struct fw_rsc_vdev_vring *vring = &vdev_rsc->vring[i];
      metal_phys_addr_t vring_da = METAL_BAD_PHYS;
      metal_phys_addr_t vring_pa;
      uint32_t vring_sz;

      vring_sz = ALIGN_UP(vring_size(vring->num, vring->align),
                          vring->align);
      vring_pa = metal_io_virt_to_phys(metal_io_get_region(), *shmbase);
      remoteproc_mmap(rproc, &vring_pa, &vring_da, vring_sz, 0, NULL);

      rptuninfo("vr[%u] shm=%p len=%zu da=0x%lx pa=0x%lx sz=%" PRIu32 "\n",
                i, *shmbase, *shmlen, vring_da, vring_pa, vring_sz);

      if (vring->da == 0 || vring->da == FW_RSC_U32_ADDR_ANY)
        {
          vring->da = vring_da;
          *shmbase += vring_sz;
          *shmlen  -= vring_sz;
        }
    }
}

/****************************************************************************
 * Name: rptun_create_device
 ****************************************************************************/

static int rptun_create_device(FAR struct rptun_priv_s *priv,
                               FAR struct virtio_device **vdev_, int index)
{
  FAR struct remoteproc *rproc = &priv->rproc;
  FAR struct fw_rsc_carveout *carveout_rsc = NULL;
  FAR struct fw_rsc_vdev *vdev_rsc;
  FAR struct remoteproc_virtio *rvdev;
  FAR struct virtio_device *vdev;
  FAR struct metal_list *node;
  FAR char *rsc = rproc->rsc_table;
  FAR char *shmbase;
  unsigned int role;
  size_t shmlen;
  size_t off;
  int ret;

  off = find_rsc(rsc, RSC_VDEV, index);
  if (off == 0)
    {
      return index ? -ENODEV : -EINVAL;
    }

  vdev_rsc = (FAR struct fw_rsc_vdev *)(rsc + off);

  /* Check that this virtio device/driver is not created before */

  metal_mutex_acquire(&rproc->lock);
  metal_list_for_each(&rproc->vdevs, node)
    {
      rvdev = metal_container_of(node, struct remoteproc_virtio, node);
      if (rvdev->vdev_rsc == vdev_rsc)
        {
          metal_mutex_release(&rproc->lock);
          return -EEXIST;
        }
    }

  metal_mutex_release(&rproc->lock);

  /* Get virtio device role from virtio device resource table */

  role = RPTUN_IS_MASTER(priv->dev) ^
         (vdev_rsc->reserved[0] == VIRTIO_DEV_DRIVER);

  if (role == VIRTIO_DEV_DEVICE &&
      !(vdev_rsc->status & VIRTIO_CONFIG_STATUS_DRIVER_OK))
    {
      return -EAGAIN;
    }

  /* If provided the carveout, init the vring->da (driver side) and init
   * a share memory heap based on the carveout defined memory region.
   * Note: do not return error because the carveout is optional for the
   * virtio device side.
   */

  off = find_rsc(rsc, RSC_CARVEOUT, index);
  if (off != 0)
    {
      carveout_rsc = (FAR struct fw_rsc_carveout *)(rsc + off);

      /* Get share memory from carveout resource table */

      shmbase = rptun_get_carveout_memory(priv, carveout_rsc, &shmlen);
      DEBUGASSERT(shmbase != NULL);

      /* Update the vring->da address for driver if needed  */

      rptun_update_vring_da(rproc, vdev_rsc, role, &shmbase, &shmlen);
    }

  vdev = remoteproc_create_virtio(rproc, index, role, NULL);
  if (vdev == NULL)
    {
      return -ENOMEM;
    }

  ret = rproc_virtio_set_shm_io(vdev, metal_io_get_region());
  if (ret < 0)
    {
      goto err;
    }

  if (carveout_rsc != NULL)
    {
      ret = rptun_init_carveout(priv, vdev,
                                (FAR const char *)carveout_rsc->name,
                                shmbase, shmlen);
      if (ret < 0)
        {
          goto err;
        }
    }

  *vdev_ = vdev;
  return OK;

err:
  remoteproc_remove_virtio(rproc, vdev);
  return ret;
}

/****************************************************************************
 * Name: rptun_remove_device
 ****************************************************************************/

static void rptun_remove_device(FAR struct rptun_priv_s *priv,
                                FAR struct virtio_device *vdev)
{
  rptun_uninit_carveout(vdev);
  remoteproc_remove_virtio(&priv->rproc, vdev);
}

/****************************************************************************
 * Name: rptun_register_device
 ****************************************************************************/

static int rptun_register_device(FAR struct virtio_device *vdev)
{
  int ret = -ENODEV;

#ifdef CONFIG_DRIVERS_VIRTIO
  if (vdev->role == VIRTIO_DEV_DRIVER)
    {
      ret = virtio_register_device(vdev);
      if (ret < 0)
        {
          rptunerr("virtio_register_device failed, ret=%d\n", ret);
          return ret;
        }
    }
  else
#endif
#ifdef CONFIG_DRIVERS_VHOST
  if (vdev->role == VIRTIO_DEV_DEVICE)
    {
      ret = vhost_register_device(vdev);
      if (ret < 0)
        {
          rptunerr("vhost_register_device failed, ret=%d\n", ret);
          return ret;
        }
    }
  else
#endif
  if (vdev->id.device == VIRTIO_ID_RPMSG)
    {
      ret = rpmsg_virtio_probe(vdev);
      if (ret < 0)
        {
          rptunerr("rpmsg_virtio_probe failed, ret=%d\n", ret);
        }
    }
  else
    {
      rptunerr("virtio device id = %"PRIu32" not supported\n",
               vdev->id.device);
    }

  return ret;
}

/****************************************************************************
 * Name: rptun_unregister_device
 ****************************************************************************/

static void rptun_unregister_device(FAR struct virtio_device *vdev)
{
#ifdef CONFIG_DRIVERS_VIRTIO
  if (vdev->role == VIRTIO_DEV_DRIVER)
    {
      virtio_unregister_device(vdev);
    }
  else
#endif
#ifdef CONFIG_DRIVERS_VHOST
  if (vdev->role == VIRTIO_DEV_DEVICE)
    {
      vhost_unregister_device(vdev);
    }
  else
#endif
  if (vdev->id.device == VIRTIO_ID_RPMSG)
    {
      rpmsg_virtio_remove(vdev);
    }
}

/****************************************************************************
 * Name: rptun_remove_devices
 ****************************************************************************/

static void rptun_remove_devices(FAR struct rptun_priv_s *priv)
{
  FAR struct remoteproc *rproc = &priv->rproc;
  FAR struct remoteproc_virtio *rvdev;
  FAR struct metal_list *node;
  FAR struct metal_list *temp;

  metal_mutex_acquire(&rproc->lock);
  metal_list_for_each_safe(&rproc->vdevs, temp, node)
    {
      rvdev = metal_container_of(node, struct remoteproc_virtio, node);
      rptun_unregister_device(&rvdev->vdev);
      rptun_remove_device(priv, &rvdev->vdev);
    }

  metal_mutex_release(&rproc->lock);
}

/****************************************************************************
 * Name: rptun_create_devices
 ****************************************************************************/

static int rptun_create_devices(FAR struct rptun_priv_s *priv)
{
  FAR struct virtio_device *vdev;
  bool remain = false;
  int ret;
  int i;

  for (i = 0; ; i++)
    {
      ret = rptun_create_device(priv, &vdev, i);
      if (ret == -ENODEV)
        {
          return remain ? -EAGAIN : OK;
        }
      else if (ret == -EEXIST)
        {
          continue;
        }
      else if (ret == -EAGAIN)
        {
          remain = true;
          continue;
        }
      else if (ret < 0)
        {
          rptunerr("rptun_create_device failed, ret=%d i=%d\n", ret, i);
          goto err;
        }

      ret = rptun_register_device(vdev);
      if (ret < 0)
        {
          rptunerr("rptun_register_device failed, ret=%d i=%d\n", ret, i);
          break;
        }
    }

  rptun_remove_device(priv, vdev);
err:
  rptun_remove_devices(priv);
  return ret;
}

static void rptun_send_command(FAR struct rptun_priv_s *priv,
                               uint32_t cmd, bool wait)
{
  FAR struct rptun_cmd_s *rptun_cmd = RPTUN_RSC2CMD(priv->rproc.rsc_table);

  if (RPTUN_IS_MASTER(priv->dev))
    {
      rptun_cmd->cmd_master = cmd;
    }
  else
    {
      rptun_cmd->cmd_slave = cmd;
    }

  rptun_notify(&priv->rproc, RPTUN_NOTIFY_ALL);

  if (wait)
    {
      uint32_t timeout = CONFIG_RPTUN_CMD_TIMEOUT_MS;

      while (timeout-- > 0)
        {
          uint32_t ack = rptun_recv_command(priv, false);

          if (RPTUN_GET_CMD(ack) == RPTUN_CMD_ACK)
            {
              break;
            }

          up_mdelay(1);
        }
    }
}

static uint32_t rptun_recv_command(FAR struct rptun_priv_s *priv, bool ack)
{
  FAR struct rptun_cmd_s *rptun_cmd = RPTUN_RSC2CMD(priv->rproc.rsc_table);
  uint32_t cmd;

  if (RPTUN_IS_MASTER(priv->dev))
    {
      if (rptun_cmd->cmd_slave == RPTUN_CMD_DONE)
        {
          return RPTUN_CMD_DONE;
        }

      cmd = rptun_cmd->cmd_slave;
      rptun_cmd->cmd_slave = RPTUN_CMD_DONE;
    }
  else
    {
      if (rptun_cmd->cmd_master == RPTUN_CMD_DONE)
        {
          return RPTUN_CMD_DONE;
        }

      cmd = rptun_cmd->cmd_master;
      rptun_cmd->cmd_master = RPTUN_CMD_DONE;
    }

  if (ack)
    {
      rptun_send_command(priv, RPTUN_CMD(RPTUN_CMD_ACK, 0), false);
    }

  return cmd;
}

static void rptun_check_command(FAR struct rptun_priv_s *priv)
{
  uint32_t cmd = rptun_recv_command(priv, true);

  switch (RPTUN_GET_CMD(cmd))
    {
      case RPTUN_CMD_RESET:
#ifdef CONFIG_BOARDCTL_RESET
        boardctl(BOARDIOC_RESET, RPTUN_GET_CMD_VAL(cmd));
#endif
        break;

      case RPTUN_CMD_PANIC:
        priv->rpanic = true;
        PANIC();
        break;

      default:
        break;
    }
}

static int rptun_callback(FAR void *arg, uint32_t vqid)
{
  FAR struct rptun_priv_s *priv = arg;

  rptun_check_command(priv);
  return remoteproc_get_notification(&priv->rproc, vqid);
}

static int rptun_do_start(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;
  FAR struct resource_table *rsc;
  int ret;

  ret = remoteproc_config(rproc, NULL);
  if (ret < 0)
    {
      rptunerr("remoteproc config failed, ret=%d\n", ret);
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
      if (ret < 0)
        {
          rptunerr("remoteproc load failed, ret=%d\n", ret);
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
          rptunerr("RPTUN_GET_RESOURCE failed\n");
          return -EINVAL;
        }

      ret = remoteproc_set_rsc_table(rproc, (struct resource_table *)rsc,
                                     sizeof(struct rptun_rsc_s));
      if (ret < 0)
        {
          rptunerr("remoteproc set rsc_table failed, ret=%d\n", ret);
          return ret;
        }
    }

  /* Remote proc start */

  ret = remoteproc_start(rproc);
  if (ret < 0)
    {
      remoteproc_shutdown(rproc);
      rptunerr("remoteproc_start failed, ret=%d\n", ret);
      return ret;
    }

  /* Register callback to mbox for receiving remote message */

  RPTUN_REGISTER_CALLBACK(priv->dev, rptun_callback, priv);

  return ret;
}

static int rptun_start_thread(int argc, FAR char *argv[])
{
  FAR struct rptun_priv_s *priv =
  (FAR struct rptun_priv_s *)((uintptr_t)strtoul(argv[2], NULL, 16));
  int ret;

  ret = rptun_do_start(&priv->rproc);
  if (ret < 0)
    {
      return ret;
    }

  while (!priv->stop)
    {
      ret = rptun_create_devices(priv);
      if (ret != -EAGAIN)
        {
          break;
        }

      nxsig_usleep(RPTUN_RETRY_PERIOD_US);
    }

  return ret;
}

static int rptun_dev_start(FAR struct rptun_priv_s *priv)
{
  FAR struct rptun_dev_s *dev = priv->dev;
  FAR char *argv[3];
  char arg1[32];

  /* Create a thread to register the virtio and vhost devices */

  snprintf(arg1, sizeof(arg1), "%p", priv);
  argv[0] = (FAR char *)RPTUN_GET_CPUNAME(dev);
  argv[1] = arg1;
  argv[2] = NULL;

  if (dev->stack != NULL && dev->stack_size != 0)
    {
      priv->pid = kthread_create_with_stack("rptun",
                                            CONFIG_RPTUN_PRIORITY,
                                            dev->stack,
                                            dev->stack_size,
                                            rptun_start_thread, argv);
    }
  else
    {
      priv->pid = kthread_create("rptun",
                                 CONFIG_RPTUN_PRIORITY,
                                 CONFIG_RPTUN_STACKSIZE,
                                 rptun_start_thread,
                                 argv);
    }

  return priv->pid;
}

static int rptun_dev_stop(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;

  if (priv->rproc.state == RPROC_OFFLINE)
    {
      return OK;
    }
  else if (priv->rproc.state == RPROC_CONFIGURED ||
           priv->rproc.state == RPROC_READY)
    {
      return -EBUSY;
    }

  if (priv->pid >= 0)
    {
      priv->stop = true;
      nxsig_kill(priv->pid, SIGKILL);
      nxsched_waitpid(priv->pid, NULL, WEXITED);
      priv->stop = false;
      priv->pid = -EINVAL;
    }

  RPTUN_UNREGISTER_CALLBACK(priv->dev);
  rptun_remove_devices(priv);
  remoteproc_shutdown(rproc);

  return OK;
}

static void rptun_dev_reset(FAR struct rptun_priv_s *priv, uint16_t val)
{
  if (priv->dev->ops->reset)
    {
      priv->dev->ops->reset(priv->dev, val);
    }
  else
    {
      rptun_send_command(priv, RPTUN_CMD(RPTUN_CMD_RESET, val), true);
    }
}

static void rptun_dev_panic(FAR struct rptun_priv_s *priv)
{
  if (priv->rpanic)
    {
      return;
    }

  metal_log(METAL_LOG_EMERGENCY, "Panic remote cpu %s:\n",
            RPTUN_GET_CPUNAME(priv->dev));

  if (priv->dev->ops->panic)
    {
      priv->dev->ops->panic(priv->dev);
    }
  else
    {
      rptun_send_command(priv, RPTUN_CMD(RPTUN_CMD_PANIC, 0), true);
    }

  priv->rpanic = true;
}

static int rptun_do_ioctl(FAR struct rptun_priv_s *priv, int cmd,
                          unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      case RPTUNIOC_START:
        if (priv->rproc.state == RPROC_OFFLINE)
          {
            ret = rptun_dev_start(priv);
          }
        else
          {
            ret = rptun_dev_stop(&priv->rproc);
            if (ret == OK)
              {
                ret = rptun_dev_start(priv);
              }
          }
        break;
      case RPTUNIOC_STOP:
        ret = rptun_dev_stop(&priv->rproc);
        break;
      case RPTUNIOC_RESET:
        rptun_dev_reset(priv, arg);
        break;
      case RPTUNIOC_PANIC:
        rptun_dev_panic(priv);
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static int rptun_dev_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  return rptun_do_ioctl(inode->i_private, cmd, arg);
}

static int rptun_ioctl_foreach(FAR const char *cpuname, int cmd,
                               unsigned long value)
{
  FAR struct metal_list *node;
  int ret = OK;

  if (!up_interrupt_context())
    {
      metal_mutex_acquire(&g_rptun_lock);
    }

  metal_list_for_each(&g_rptun_priv, node)
    {
      FAR struct rptun_priv_s *priv;

      priv = metal_container_of(node, struct rptun_priv_s, node);

      if (!cpuname || !strcmp(RPTUN_GET_CPUNAME(priv->dev), cpuname))
        {
          ret = rptun_do_ioctl(priv, cmd, value);
          if (ret < 0)
            {
              break;
            }
        }
    }

  if (!up_interrupt_context())
    {
      metal_mutex_release(&g_rptun_lock);
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

  ret = file_read(&store->file, store->buf, len);
  if (ret < 0)
    {
      kmm_free(store->buf);
      file_close(&store->file);
    }

  return ret;
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
  ssize_t ret;

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
  ret = file_read(&store->file, tmp, size);
  if (ret > 0)
    {
      metal_cache_flush(tmp, ret);
    }

  return ret;
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

static int rptun_panic_notifier(FAR struct notifier_block *block,
                                unsigned long action, void *data)
{
  FAR struct rptun_priv_s *priv =
    container_of(block, struct rptun_priv_s, nbpanic);

  if (action == PANIC_KERNEL_FINAL)
    {
      /* PANIC all the remote core */

      rptun_dev_panic(priv);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rptun_initialize(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_priv_s *priv;
  char name[32];
  int ret;

  priv = kmm_zalloc(sizeof(struct rptun_priv_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->dev = dev;
  priv->pid = -EINVAL;
  remoteproc_init(&priv->rproc, &g_rptun_ops, priv);

  snprintf(name, sizeof(name), "/dev/rptun/%s", RPTUN_GET_CPUNAME(dev));
  ret = register_driver(name, &g_rptun_fops, 0222, priv);
  if (ret < 0)
    {
      rptunerr("rptun register driver failed %d\n", ret);
      goto err_driver;
    }

  if (RPTUN_IS_AUTOSTART(priv->dev))
    {
      ret = rptun_dev_start(priv);
      if (ret < 0)
        {
          rptunerr("rptun start failed %d\n", ret);
          goto err_start;
        }
    }

  priv->nbpanic.notifier_call = rptun_panic_notifier;
  panic_notifier_chain_register(&priv->nbpanic);

  metal_mutex_acquire(&g_rptun_lock);
  metal_list_add_tail(&g_rptun_priv, &priv->node);
  metal_mutex_release(&g_rptun_lock);
  return OK;

err_start:
  unregister_driver(name);
err_driver:
  kmm_free(priv);
  return ret;
}

int rptun_boot(FAR const char *cpuname)
{
  return rptun_ioctl_foreach(cpuname, RPTUNIOC_START, 0);
}

int rptun_poweroff(FAR const char *cpuname)
{
  return rptun_ioctl_foreach(cpuname, RPTUNIOC_STOP, 0);
}

int rptun_reset(FAR const char *cpuname, int value)
{
  return rptun_ioctl_foreach(cpuname, RPTUNIOC_RESET, value);
}
