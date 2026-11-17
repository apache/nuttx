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

#include <assert.h>
#include <ctype.h>
#include <nuttx/debug.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/boardctl.h>
#include <sys/param.h>
#include <sys/wait.h>

#include <nuttx/clock.h>
#include <nuttx/coredump.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mm/mm.h>
#include <nuttx/nuttx.h>
#include <nuttx/percpu.h>
#include <nuttx/queue.h>
#include <nuttx/rpmsg/rpmsg_virtio.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/vhost/vhost.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/panic_notifier.h>
#include <nuttx/reboot_notifier.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <openamp/remoteproc_loader.h>
#include <openamp/remoteproc_virtio.h>
#include <openamp/rsc_table_parser.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPTUN_RETRY_PERIOD_US    1000

#define RPTUN_STATUS_CHECK(s, v) (((s) & (1u << (v))) != 0)
#define RPTUN_STATUS_SET(s, v)   ((s) |= (1u << (v)))
#define RPTUN_REASON_GET(s)      (ffs(s) - 1u)

#define RPTUN_RSC2STATUS(r)      \
  ((FAR struct rptun_status_s *)&((FAR struct resource_table *)(r))->reserved[0])

#define rptunvbs(format, ...)    syslog(LOG_ALERT, format, ##__VA_ARGS__)

#define RPTUN_STATUS_PAUSE       BOARDIOC_SOFTRESETCAUSE_MAX
#define RPTUN_STATUS_STOP        (BOARDIOC_SOFTRESETCAUSE_MAX + 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rptun_carveout_s
{
  FAR struct mm_heap_s       *heap;
  FAR void                   *start;
  char                        name[RPROC_MAX_NAME_LEN];
};

struct rptun_priv_s
{
  FAR struct rptun_dev_s      *dev;
  struct remoteproc           rproc;
  dq_entry_t                  entry;
  uint32_t                    lstatus;
  bool                        stop;
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool                        unlinked;
  int                         crefs;
#endif
  pid_t                       pid;
  FAR const void             *trace_va;
  struct work_s               work;
  FAR struct mm_heap_s       *heap;
  char                        poolname[RPROC_MAX_NAME_LEN];
};

struct rptun_store_s
{
  struct file file;
  FAR char   *buf;
};

begin_packed_struct struct rptun_status_s
{
  uint32_t master;
  uint32_t slave;
} end_packed_struct;

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
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rptun_dev_open(FAR struct file *filep);
static int rptun_dev_close(FAR struct file *filep);
static int rptun_dev_unlink(FAR struct inode *inode);
#endif

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
static size_t rptun_free_largest(FAR struct virtio_device *vdev);
static FAR void *rptun_map_buf(FAR struct virtio_device *vdev,
                               FAR void *buf, size_t len,
                               enum virtio_dma_map dir);
static void rptun_unmap_buf(FAR struct virtio_device *vdev,
                            FAR void *buf, size_t len,
                            enum virtio_dma_map dir, FAR void *mapped);
static int rptun_get_shm_region(FAR struct virtio_device *vdev, uint8_t id,
                                FAR struct virtio_shm_region *region);

static void rptun_set_status_notify(FAR struct rptun_priv_s *priv,
                                    unsigned long reason);

#ifndef CONFIG_RPTUN_AUTO_RESET_DISABLE
static int rptun_notifier(FAR struct notifier_block *block,
                          unsigned long action, void *data);
#endif

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
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .open   = rptun_dev_open,
  .close  = rptun_dev_close,
#endif
  .ioctl  = rptun_dev_ioctl,
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .unlink = rptun_dev_unlink,
#endif
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
  .alloc          = rptun_alloc_buf,
  .free           = rptun_free_buf,
  .free_largest   = rptun_free_largest,
  .map            = rptun_map_buf,
  .unmap          = rptun_unmap_buf,
  .get_shm_region = rptun_get_shm_region,
};

static DEFINE_PER_CPU_BSS_BMP(dq_queue_t, g_rptun_priv);
#define g_rptun_priv this_cpu_var_bmp(g_rptun_priv)
static DEFINE_PER_CPU_BMP(rmutex_t, g_rptun_lock) = NXRMUTEX_INITIALIZER;
#define g_rptun_lock this_cpu_var_bmp(g_rptun_lock)

#ifdef CONFIG_RPTUN_AUTO_RESET_IN_PANIC_NOTIFIER
static DEFINE_PER_CPU_BMP(struct notifier_block, g_rptun_panic_nb) =
{
  .notifier_call = rptun_notifier,
  .priority = INT_MIN, /* Panic notifier should be called at the last */
};
#  define g_rptun_panic_nb this_cpu_var_bmp(g_rptun_panic_nb)
#endif

#ifndef CONFIG_RPTUN_AUTO_RESET_DISABLE
static DEFINE_PER_CPU_BMP(struct notifier_block, g_rptun_reboot_nb) =
{
  .notifier_call = rptun_notifier,
  .priority = INT_MIN, /* Reboot notifier should be called at the last */
};
#  define g_rptun_reboot_nb this_cpu_var_bmp(g_rptun_reboot_nb)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR struct remoteproc *
rptun_init(FAR struct remoteproc *rproc,
           FAR const struct remoteproc_ops *ops,
           FAR void *arg)
{
  rproc->lock.is_mutex = true;
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
  int ret = OK;

  if (RPTUN_IS_MASTER(priv->dev))
    {
      ret = RPTUN_CONFIG(priv->dev, data);
    }

  return ret;
}

static int rptun_start(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;
  int ret = OK;

  if (RPTUN_IS_MASTER(priv->dev))
    {
      ret = RPTUN_START(priv->dev);
    }

  return ret;
}

static int rptun_stop(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;
  int ret = OK;

  if (RPTUN_IS_MASTER(priv->dev))
    {
      ret = -ENOTSUP;
      if (priv->dev->ops->stop != NULL)
        {
          ret = RPTUN_STOP(priv->dev);
        }

      /* The transport cannot stop the peer, ask the peer to stop itself */

      if (ret == -ENOTSUP)
        {
          rptun_set_status_notify(priv, RPTUN_STATUS_STOP);
          ret = OK;
        }
    }

  return ret;
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
      buf = NULL;
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

  DEBUGASSERT(vdev->role == VIRTIO_DEV_DRIVER);

  return mm_memalign(carveout->heap, align, size);
}

/****************************************************************************
 * Name: rptun_free_buf
 ****************************************************************************/

static void rptun_free_buf(FAR struct virtio_device *vdev, FAR void *buf)
{
  FAR struct rptun_carveout_s *carveout =
    (FAR struct rptun_carveout_s *)vdev->mm_priv;

  DEBUGASSERT(vdev->role == VIRTIO_DEV_DRIVER);

  mm_free(carveout->heap, buf);
}

/****************************************************************************
 * Name: rptun_map_align
 *
 * Description:
 *   Return the alignment used for bounce buffers. Use the dcache line size
 *   so a bounce buffer never shares a cache line with a neighbour (avoids
 *   clobbering adjacent data on invalidate). Fall back to MM_ALIGN when the
 *   arch has no dcache, so the value is never zero.
 *
 ****************************************************************************/

static inline_function size_t rptun_map_align(void)
{
  size_t linesize = up_get_dcache_linesize();

  return linesize > MM_ALIGN ? linesize : MM_ALIGN;
}

/****************************************************************************
 * Name: rptun_free_largest
 *
 * Description:
 *   Return the largest buffer a single rptun_map_buf call can satisfy.
 *   mm_heapfree_largest gives the largest mm_malloc allocation; subtract the
 *   extra space mm_memalign consumes for alignment so a map of the returned
 *   size is guaranteed to succeed.
 *
 ****************************************************************************/

static size_t rptun_free_largest(FAR struct virtio_device *vdev)
{
  FAR struct rptun_carveout_s *carveout =
    (FAR struct rptun_carveout_s *)vdev->mm_priv;
  size_t align = rptun_map_align();
  size_t largest;

  DEBUGASSERT(vdev->role == VIRTIO_DEV_DRIVER);

  largest = mm_heapfree_largest(carveout->heap);

  /* mm_memalign over-allocates by up to 2 * align to find an aligned point,
   * and map rounds len up to align. Reserve 3 * align to stay safe.
   */

  size_t overhead = 3 * align;

  return largest > overhead ? largest - overhead : 0;
}

/****************************************************************************
 * Name: rptun_map_buf
 *
 * Description:
 *   Map a buffer so the remote side can access it. The remote can only
 *   touch the carveout shared memory, so a buffer outside the carveout is
 *   bounced through a buffer allocated from the carveout heap. Returns NULL
 *   when the carveout heap cannot satisfy the allocation; the caller may
 *   query rptun_free_largest and retry with a smaller length.
 *
 ****************************************************************************/

static FAR void *rptun_map_buf(FAR struct virtio_device *vdev,
                               FAR void *buf, size_t len,
                               enum virtio_dma_map dir)
{
  FAR struct rptun_carveout_s *carveout =
    (FAR struct rptun_carveout_s *)vdev->mm_priv;
  size_t align = rptun_map_align();
  FAR void *bounce;
  size_t maplen;

  DEBUGASSERT(vdev->role == VIRTIO_DEV_DRIVER);

  /* Already inside the carveout heap: the remote can access it directly. */

  if (mm_heapmember(carveout->heap, buf))
    {
      return buf;
    }

  maplen = ALIGN_UP(len, align);
  bounce = mm_memalign(carveout->heap, align, maplen);
  if (bounce == NULL)
    {
      /* Out of shared memory: the caller decides whether to query
       * rptun_free_largest and retry with a smaller length.
       */

      return NULL;
    }

  if (dir != VIRTIO_DMA_FROM_DEVICE)
    {
      memcpy(bounce, buf, len);
    }

  /* Flush for every direction: a dirty line left by an earlier user of
   * this carveout block must not be written back over the peer's data.
   */

  RPTUN_FLUSH(bounce, maplen);
  return bounce;
}

/****************************************************************************
 * Name: rptun_unmap_buf
 *
 * Description:
 *   Release a buffer mapped by rptun_map_buf, copying the device-written
 *   data back to the original buffer when the direction requires it. Pass
 *   the same direction that was given to rptun_map_buf.
 *
 ****************************************************************************/

static void rptun_unmap_buf(FAR struct virtio_device *vdev,
                            FAR void *buf, size_t len,
                            enum virtio_dma_map dir, FAR void *mapped)
{
  FAR struct rptun_carveout_s *carveout =
    (FAR struct rptun_carveout_s *)vdev->mm_priv;

  DEBUGASSERT(vdev->role == VIRTIO_DEV_DRIVER);

  /* Not bounced (buffer was already inside the carveout): nothing to do. */

  if (mapped == buf)
    {
      return;
    }

  if (dir != VIRTIO_DMA_TO_DEVICE)
    {
      /* Device wrote data: drop stale cache lines, then copy it back. */

      RPTUN_INVALIDATE(mapped, ALIGN_UP(len, rptun_map_align()));
      memcpy(buf, mapped, len);
    }

  mm_free(carveout->heap, mapped);
}

/****************************************************************************
 * Name: rptun_find_carveout
 *
 * Description:
 *   Find the carveout with the given name. Returns NULL when the table has
 *   none, which is legal because a carveout is optional.
 *
 ****************************************************************************/

static FAR struct fw_rsc_carveout *
rptun_find_carveout(FAR struct resource_table *rsc, FAR const char *name)
{
  FAR struct fw_rsc_carveout *carv = NULL;
  uint32_t i;

  for (i = 0; i < rsc->num; i++)
    {
      FAR struct fw_rsc_hdr *hdr =
        (FAR struct fw_rsc_hdr *)((FAR char *)rsc + rsc->offset[i]);
      FAR struct fw_rsc_carveout *tmp;

      if (hdr->type != RSC_CARVEOUT)
        {
          continue;
        }

      /* The name in the table need not be terminated, so bound the
       * comparison by the size of the field.
       */

      tmp = (FAR struct fw_rsc_carveout *)hdr;
      if (strncmp((FAR const char *)tmp->name, name,
                  RPROC_MAX_NAME_LEN) == 0)
        {
          carv = tmp;
          break;
        }
    }

  return carv;
}

/****************************************************************************
 * Name: rptun_is_shm_carveout
 *
 * Description:
 *   Tell whether a carveout is a shared memory region of vdev index, named
 *   "vdev<index>buffer<id>". The bare "vdev<index>buffer" is the heap.
 *
 ****************************************************************************/

static bool rptun_is_shm_carveout(FAR const struct fw_rsc_carveout *carv,
                                  unsigned int index)
{
  FAR const char *name = (FAR const char *)carv->name;
  char prefix[RPROC_MAX_NAME_LEN];
  int len;

  len = snprintf(prefix, sizeof(prefix), "vdev%ubuffer", index);
  return carv->type == RSC_CARVEOUT &&
         strncmp(name, prefix, len) == 0 && isdigit(name[len]);
}

/****************************************************************************
 * Name: rptun_alloc_shm_regions
 *
 * Description:
 *   Place the shared memory regions of vdev index the table left at
 *   FW_RSC_U32_ADDR_ANY: cut them from heap and write the da back, as
 *   rptun_alloc_vrings() does, so the peer finds them once DRIVER_OK is set.
 *
 ****************************************************************************/

static int rptun_alloc_shm_regions(FAR struct rptun_priv_s *priv,
                                   unsigned int index,
                                   FAR struct mm_heap_s *heap)
{
  FAR struct resource_table *rsc = priv->rproc.rsc_table;
  uint32_t i;

  for (i = 0; i < rsc->num; i++)
    {
      FAR struct fw_rsc_carveout *carv =
        (FAR struct fw_rsc_carveout *)((FAR char *)rsc + rsc->offset[i]);
      metal_phys_addr_t pa;
      FAR void *va;

      if ((carv->da != 0u && carv->da != FW_RSC_U32_ADDR_ANY) ||
          !rptun_is_shm_carveout(carv, index))
        {
          continue;
        }

      va = mm_memalign(heap, rptun_map_align(), carv->len);
      if (va == NULL)
        {
          rptunerr("ERROR: %s alloc failed, len=%" PRIu32 "\n",
                   (FAR const char *)carv->name, carv->len);
          return -ENOMEM;
        }

      pa = metal_io_virt_to_phys(metal_io_get_region(), va);
      carv->da = rptun_pa_to_da(priv->dev, pa);
    }

  return OK;
}

/****************************************************************************
 * Name: rptun_free_shm_regions
 *
 * Description:
 *   Release the regions rptun_alloc_shm_regions() placed and hand their da
 *   back to FW_RSC_U32_ADDR_ANY. A region with a static da lies outside
 *   heap and is left untouched.
 *
 ****************************************************************************/

static void rptun_free_shm_regions(FAR struct rptun_priv_s *priv,
                                   unsigned int index,
                                   FAR struct mm_heap_s *heap)
{
  FAR struct resource_table *rsc = priv->rproc.rsc_table;
  uint32_t i;

  for (i = 0; i < rsc->num; i++)
    {
      FAR struct fw_rsc_carveout *carv =
        (FAR struct fw_rsc_carveout *)((FAR char *)rsc + rsc->offset[i]);
      metal_phys_addr_t pa;
      FAR void *va;

      if (carv->da == 0u || carv->da == FW_RSC_U32_ADDR_ANY ||
          !rptun_is_shm_carveout(carv, index))
        {
          continue;
        }

      pa = rptun_da_to_pa(priv->dev, carv->da);
      va = metal_io_phys_to_virt(metal_io_get_region(), pa);
      if (va != NULL && mm_heapmember(heap, va))
        {
          mm_free(heap, va);
          carv->da = FW_RSC_U32_ADDR_ANY;
        }
    }
}

/****************************************************************************
 * Name: rptun_get_shm_region
 *
 * Description:
 *   Shared memory region id of vdev N is the carveout "vdevNbufferid".
 *   Both sides access it in place, so each maps the da through its own
 *   address environment.
 *
 ****************************************************************************/

static int rptun_get_shm_region(FAR struct virtio_device *vdev, uint8_t id,
                                FAR struct virtio_shm_region *region)
{
  FAR struct remoteproc_virtio *rpvdev =
    metal_container_of(vdev, struct remoteproc_virtio, vdev);
  FAR struct remoteproc *rproc = rpvdev->priv;
  FAR struct fw_rsc_carveout *rsc;
  char name[RPROC_MAX_NAME_LEN];
  metal_phys_addr_t pa = METAL_BAD_PHYS;
  metal_phys_addr_t da;

  snprintf(name, sizeof(name), "vdev%ubuffer%u", rpvdev->index, id);
  rsc = rptun_find_carveout(rproc->rsc_table, name);
  if (rsc == NULL || rsc->da == 0u || rsc->da == FW_RSC_U32_ADDR_ANY)
    {
      return -ENOENT;
    }

  da = rsc->da;
  region->va = remoteproc_mmap(rproc, &pa, &da, rsc->len, 0, NULL);
  if (region->va == NULL)
    {
      rptunerr("ERROR: %s mmap failed, da=0x%lx\n", name, da);
      return -ENOMEM;
    }

  region->addr = pa;
  region->len  = rsc->len;
  return OK;
}

/****************************************************************************
 * Name: rptun_create_heap
 *
 * Description:
 *   Build a heap named after a carveout.  Without a va the carveout is one
 *   the table placed, so map what its device address points at and take the
 *   heap context from the kernel heap.  With a va the caller cut the region
 *   out of the pool, and the context sits at the head of the region itself,
 *   so the heap pointer is the region base and one free returns all of it.
 *
 *   The heap keeps a pointer to its name, so the name is copied into the
 *   caller's buffer rather than pointing into the table the peer can
 *   rewrite.
 *
 ****************************************************************************/

static FAR struct mm_heap_s *
rptun_create_heap(FAR struct rptun_priv_s *priv, FAR char *name,
                  FAR const uint8_t *rscname, metal_phys_addr_t da,
                  FAR void *va, size_t len)
{
  FAR struct mm_heap_s *heap = NULL;
  struct mm_heap_config_s config;

  strlcpy(name, (FAR const char *)rscname, RPROC_MAX_NAME_LEN);
  if (va == NULL)
    {
      /* Create heap from carveout resource, need map the da to va */

      va = remoteproc_mmap(&priv->rproc, NULL, &da, len, 0, NULL);
      if (va == NULL)
        {
          rptunerr("ERROR: %s mmap failed, da=0x%lx len=%zu\n",
                    name, da, len);
        }
    }

  if (va != NULL)
    {
      memset(&config, 0, sizeof(config));
      config.name      = name;
      config.start     = va;
      config.size      = len;
      config.nokasan   = true;
      config.allocheap = true;

      heap = mm_initialize_heap(&config);
      if (heap != NULL)
        {
          rptuninfo("INFO: heap=%p name=%s va=%p size=%zu\n",
                    heap, name, va, len);
        }
      else
        {
          rptunerr("ERROR: failed to initialize heap %s\n", name);
        }
    }

  return heap;
}

/****************************************************************************
 * Name: rptun_create_carveout
 *
 * Description:
 *   Hand a virtio device the heap it allocates from.  A carveout named after
 *   the device index is private to it, and is either placed by the table or
 *   cut out of the pool here; a device the table names no carveout for gets
 *   the pool itself.
 *
 *   Only a driver side device gets this far, so the memory is this side's to
 *   manage.  Where the table leaves every device on the vhost side, none of
 *   this runs and the memory stays the peer's.
 *
 ****************************************************************************/

static FAR struct rptun_carveout_s *
rptun_create_carveout(FAR struct rptun_priv_s *priv, unsigned int index)
{
  FAR struct rptun_carveout_s *carv;
  FAR struct fw_rsc_carveout *rsc;
  char name[RPROC_MAX_NAME_LEN];

  carv = kmm_zalloc(sizeof(struct rptun_carveout_s));
  if (carv != NULL)
    {
      /* Try create the virtio driver private heap */

      snprintf(name, sizeof(name), "vdev%ubuffer", index);
      rsc = rptun_find_carveout(priv->rproc.rsc_table, name);
      if (rsc == NULL)
        {
          carv->heap = priv->heap;
        }
      else if (rsc->da != 0u && rsc->da != FW_RSC_U32_ADDR_ANY)
        {
          carv->heap = rptun_create_heap(priv, carv->name, rsc->name,
                                         rsc->da, NULL, rsc->len);
        }
      else if (priv->heap != NULL)
        {
          /* Resource not specific the memory address, so try cut a piece
           * from the shared heap.
           */

          FAR void *va = mm_memalign(priv->heap, MM_ALIGN, rsc->len);

          if (va != NULL)
            {
              carv->start = va;
              carv->heap = rptun_create_heap(priv, carv->name, rsc->name,
                                             rsc->da, va, rsc->len);
              if (carv->heap == NULL)
                {
                  mm_free(priv->heap, va);
                }
            }
          else
            {
              rptunerr("ERROR: no room in the pool for %"PRIu32" bytes\n",
                       rsc->len);
            }
        }

      if (carv->heap == NULL)
        {
          rptunerr("ERROR: failed to create heap for %s\n", name);
          kmm_free(carv);
          carv = NULL;
        }
    }

  return carv;
}

/****************************************************************************
 * Name: rptun_free_carveout
 *
 * Description:
 *   Drop a heap a device is done with.  One cut from the pool goes back to
 *   it, so a later start cuts the region afresh.  The pool itself outlives
 *   the devices that shared it.
 *
 ****************************************************************************/

static void rptun_free_carveout(FAR struct rptun_priv_s *priv,
                                FAR struct rptun_carveout_s *carv)
{
  if (carv->heap != priv->heap)
    {
      mm_uninitialize(carv->heap);

      if (priv->heap != NULL && mm_heapmember(priv->heap, carv->start))
        {
          mm_free(priv->heap, carv->start);
        }
    }

  kmm_free(carv);
}

/****************************************************************************
 * Name: rptun_free_vrings
 *
 * Description:
 *   Release the vrings taken from the heap and hand their da fields back to
 *   FW_RSC_U32_ADDR_ANY, so a later start places them again instead of
 *   reusing an address from a destroyed heap. A vring the table placed
 *   statically lies outside the heap and is left untouched.
 *
 ****************************************************************************/

static void rptun_free_vrings(FAR struct rptun_priv_s *priv,
                              FAR struct fw_rsc_vdev *vdev_rsc,
                              FAR struct mm_heap_s *heap)
{
  uint8_t i;

  for (i = 0; i < vdev_rsc->num_of_vrings; i++)
    {
      FAR struct fw_rsc_vdev_vring *vring_rsc = &vdev_rsc->vring[i];
      metal_phys_addr_t pa;
      FAR void *va;

      if (vring_rsc->da == 0u || vring_rsc->da == FW_RSC_U32_ADDR_ANY)
        {
          continue;
        }

      pa = rptun_da_to_pa(priv->dev, vring_rsc->da);
      va = metal_io_phys_to_virt(metal_io_get_region(), pa);
      if (va != NULL && mm_heapmember(heap, va))
        {
          mm_free(heap, va);
          vring_rsc->da = FW_RSC_U32_ADDR_ANY;
        }
    }
}

/****************************************************************************
 * Name: rptun_alloc_vrings
 *
 * Description:
 *   Place the vrings of a driver side vdev in its heap. They are plain
 *   allocations, so a heap shared by several devices needs no cross device
 *   reservation pass.
 *
 ****************************************************************************/

static int rptun_alloc_vrings(FAR struct rptun_priv_s *priv,
                              FAR struct fw_rsc_vdev *vdev_rsc,
                              FAR struct mm_heap_s *heap)
{
  int ret = OK;
  uint8_t i;

  for (i = 0; i < vdev_rsc->num_of_vrings; i++)
    {
      FAR struct fw_rsc_vdev_vring *vring_rsc = &vdev_rsc->vring[i];
      metal_phys_addr_t pa;
      FAR void *va;
      size_t sz;

      /* A static da was placed by the table author on purpose. */

      if (vring_rsc->da != 0u && vring_rsc->da != FW_RSC_U32_ADDR_ANY)
        {
          continue;
        }

      sz = ALIGN_UP(vring_size(vring_rsc->num, vring_rsc->align),
                    vring_rsc->align);
      va = mm_memalign(heap, vring_rsc->align, sz);
      if (va == NULL)
        {
          rptunerr("ERROR: vr[%u] alloc failed, sz=%zu\n", i, sz);
          ret = -ENOMEM;
          break;
        }

      pa = metal_io_virt_to_phys(metal_io_get_region(), va);
      vring_rsc->da = rptun_pa_to_da(priv->dev, pa);

      rptuninfo("INFO: vr[%u] va=%p da=0x%" PRIx32 " sz=%zu\n", i, va,
                vring_rsc->da, sz);
    }

  return ret;
}

/****************************************************************************
 * Name: rptun_check_vdev
 *
 * Description:
 *   Tell whether the virtio device can be created now: -EEXIST when this
 *   rproc already has it, -EAGAIN when the vhost side would come up before
 *   the peer driver is ready.
 *
 ****************************************************************************/

static int rptun_check_vdev(FAR struct rptun_priv_s *priv,
                            FAR struct fw_rsc_vdev *vdev_rsc,
                            unsigned int role)
{
  FAR struct remoteproc *rproc = &priv->rproc;
  FAR struct remoteproc_virtio *rvdev;
  FAR struct metal_list *node;
  int ret = OK;

  metal_mutex_acquire(&rproc->lock);
  metal_list_for_each(&rproc->vdevs, node)
    {
      rvdev = container_of(node, struct remoteproc_virtio, node);
      if (rvdev->vdev_rsc == vdev_rsc)
        {
          ret = -EEXIST;
          break;
        }
    }

  metal_mutex_release(&rproc->lock);

  if (ret >= 0 && role == VIRTIO_DEV_DEVICE &&
      !(vdev_rsc->status & VIRTIO_CONFIG_STATUS_DRIVER_OK))
    {
      ret = -EAGAIN;
    }

  return ret;
}

/****************************************************************************
 * Name: rptun_create_device
 ****************************************************************************/

static int rptun_create_device(FAR struct rptun_priv_s *priv,
                               FAR struct virtio_device **vdev_,
                               unsigned int index)
{
  FAR struct remoteproc *rproc = &priv->rproc;
  FAR struct rptun_carveout_s *carv = NULL;
  FAR char *rsc = rproc->rsc_table;
  FAR struct fw_rsc_vdev *vdev_rsc;
  FAR struct virtio_device *vdev;
  unsigned int role;
  size_t off;
  int ret;

  off = find_rsc(rsc, RSC_VDEV, index);
  if (off == 0u)
    {
      ret = index ? -ENODEV : -EINVAL;
    }
  else
    {
      vdev_rsc = (FAR struct fw_rsc_vdev *)(rsc + off);
      role = RPTUN_IS_MASTER(priv->dev) ^
             (vdev_rsc->reserved[0] == VIRTIO_DEV_DRIVER);

      ret = rptun_check_vdev(priv, vdev_rsc, role);

      /* Only the driver side manages the share memory as a heap. Without
       * one there is nowhere to put the vrings, so fail rather than bring
       * the device up unable to allocate.
       */

      if (ret >= 0 && role == VIRTIO_DEV_DRIVER)
        {
          carv = rptun_create_carveout(priv, index);
          if (carv != NULL)
            {
              ret = rptun_alloc_vrings(priv, vdev_rsc, carv->heap);
              if (ret >= 0)
                {
                  ret = rptun_alloc_shm_regions(priv, index, carv->heap);
                }
            }
          else
            {
              ret = -ENOMEM;
            }
        }

      if (ret >= 0)
        {
          vdev = remoteproc_create_virtio(rproc, index, role, 0);
          if (vdev != NULL)
            {
              ret = rproc_virtio_set_shm_io(vdev, metal_io_get_region());
              if (ret < 0)
                {
                  remoteproc_remove_virtio(rproc, vdev);
                }
              else
                {
                  /* Bind the heap to virtio device, the device side only
                   * looks up shared memory regions and never allocates or
                   * maps buffers.
                   */

                  vdev->mmops = &g_rptun_mmops;
                  if (carv != NULL)
                    {
                      vdev->mm_priv = carv;
                    }

                  *vdev_ = vdev;
                }
            }
          else
            {
              ret = -ENOMEM;
            }
        }

      if (ret < 0 && carv != NULL)
        {
          rptun_free_shm_regions(priv, index, carv->heap);
          rptun_free_vrings(priv, vdev_rsc, carv->heap);
          rptun_free_carveout(priv, carv);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rptun_remove_device
 ****************************************************************************/

static void rptun_remove_device(FAR struct rptun_priv_s *priv,
                                FAR struct virtio_device *vdev)
{
  FAR struct remoteproc_virtio *rvdev =
    container_of(vdev, struct remoteproc_virtio, vdev);
  FAR struct rptun_carveout_s *carveout = vdev->mm_priv;

  if (carveout != NULL)
    {
      rptun_free_shm_regions(priv, rvdev->index, carveout->heap);
      rptun_free_vrings(priv, rvdev->vdev_rsc, carveout->heap);
      rptun_free_carveout(priv, carveout);
      vdev->mm_priv = NULL;
    }

  vdev->mmops = NULL;
  remoteproc_remove_virtio(&priv->rproc, vdev);
}

/****************************************************************************
 * Name: rptun_register_device
 ****************************************************************************/

static int rptun_register_device(FAR struct rptun_priv_s *priv,
                                 FAR struct virtio_device *vdev)
{
  int ret = -ENODEV;

  if (vdev->id.device == VIRTIO_ID_RPMSG)
    {
      ret = rpmsg_virtio_probe_cpuname(vdev, RPTUN_GET_CPUNAME(priv->dev));
      if (ret < 0)
        {
          rptunerr("rpmsg_virtio_probe failed, ret=%d\n", ret);
        }
    }
  else
#ifdef CONFIG_DRIVERS_VIRTIO
  if (vdev->role == VIRTIO_DEV_DRIVER)
    {
      ret = virtio_register_device(vdev);
      if (ret < 0)
        {
          rptunerr("virtio_register_device failed, ret=%d\n", ret);
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
        }
    }
  else
#endif
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
  if (vdev->id.device == VIRTIO_ID_RPMSG)
    {
      rpmsg_virtio_remove(vdev);
    }
  else
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
    {
      rptunerr("virtio device id = %"PRIu32" not supported\n",
               vdev->id.device);
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
      rvdev = container_of(node, struct remoteproc_virtio, node);
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
  FAR struct virtio_device *vdev = NULL;
  bool remain = false;
  unsigned int i;
  int ret;

  for (i = 0u; ; i++)
    {
      ret = rptun_create_device(priv, &vdev, i);
      if (ret == -ENODEV)
        {
          ret = remain ? -EAGAIN : OK;
          break;
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
          rptun_remove_devices(priv);
          break;
        }

      ret = rptun_register_device(priv, vdev);
      if (ret < 0)
        {
          rptunerr("rptun_register_device failed, ret=%d i=%d\n", ret, i);
          rptun_remove_device(priv, vdev);
          rptun_remove_devices(priv);
          break;
        }
    }

  return ret;
}

static inline_function uint32_t
rptun_get_peer_status(FAR struct rptun_priv_s *priv)
{
  FAR struct rptun_status_s *status =
    RPTUN_RSC2STATUS(priv->rproc.rsc_table);

  RPTUN_INVALIDATE(status, sizeof(*status));

  return RPTUN_IS_MASTER(priv->dev) ?
         status->slave : status->master;
}

static inline_function void rptun_set_status(FAR struct rptun_priv_s *priv,
                                             unsigned long reason)
{
  FAR struct rptun_status_s *status;

  if (priv->rproc.rsc_table == NULL)
    {
      return;
    }

  status = RPTUN_RSC2STATUS(priv->rproc.rsc_table);

  if (RPTUN_IS_MASTER(priv->dev))
    {
      RPTUN_STATUS_SET(status->master, reason);
    }
  else
    {
      RPTUN_STATUS_SET(status->slave, reason);
    }

  RPTUN_FLUSH(status, sizeof(*status));
}

static void rptun_set_status_notify(FAR struct rptun_priv_s *priv,
                                    unsigned long reason)
{
  rptun_set_status(priv, reason);
  rptun_notify(&priv->rproc, RPTUN_NOTIFY_ALL);
}

#if defined(CONFIG_BOARDCTL_RESET) || defined(CONFIG_BOARDCTL_POWEROFF)
static void rptun_boardctl_work(FAR void *arg)
{
  if ((unsigned long)arg == BOARDIOC_SOFTRESETCAUSE_POWEROFF)
    {
      boardctl(BOARDIOC_POWEROFF, 0u);
    }
  else
    {
      boardctl(BOARDIOC_RESET, (unsigned long)arg);
    }
}
#endif

static void rptun_stop_work(FAR void *arg)
{
  FAR struct rptun_priv_s *priv = arg;

  rptun_dev_stop(&priv->rproc);
}

static void rptun_check_peer_status(FAR struct rptun_priv_s *priv)
{
  uint32_t status;

  if (priv->rproc.rsc_table == NULL)
    {
      return;
    }

  status = rptun_get_peer_status(priv);
  status = status & ~priv->lstatus;
  priv->lstatus |= status;

  if (RPTUN_STATUS_CHECK(status, RPTUN_STATUS_PAUSE))
    {
      int timeout = CONFIG_RPTUN_STATUS_TIMEOUT_MS;

      /* Wait for the crashing core to finish printing its log */

      while (timeout-- > 0 &&
             (!RPTUN_STATUS_CHECK(status, BOARDIOC_SOFTRESETCAUSE_PANIC)))
        {
          status = rptun_get_peer_status(priv);
          up_udelay(1000);
        }

      if (!RPTUN_STATUS_CHECK(status, BOARDIOC_SOFTRESETCAUSE_PANIC))
        {
          _emerg("FATAL: Timeout waiting for remote core panic: %s\n",
                 RPTUN_GET_CPUNAME(priv->dev));
          return;
        }
    }

  if (RPTUN_STATUS_CHECK(status, BOARDIOC_SOFTRESETCAUSE_PANIC))
    {
      char msg[64];

#ifdef CONFIG_RPTUN_REMOTE_PANIC_SILENT
      /* Keep this passive panic out of the crashing core's log */

      nx_setlogmask(LOG_NONE);
#endif

      snprintf(msg, sizeof(msg), "Panic by remote core: %s",
               RPTUN_GET_CPUNAME(priv->dev));
      PANIC_WITH_MSG(msg);
    }
  else if (RPTUN_STATUS_CHECK(status, RPTUN_STATUS_STOP))
    {
      /* Remote requested channel stop, stop local side.
       * Clear rsc_table to prevent re-triggering from shared
       * interrupt line (function returns early if rsc_table is NULL).
       */

      priv->rproc.rsc_table = NULL;
      _emerg("Stop by remote core: %s\n", RPTUN_GET_CPUNAME(priv->dev));
      work_queue(HPWORK, &priv->work, rptun_stop_work, priv, 0);
    }
  else if(status != 0u)
    {
      _emerg("FATAL: Reset by remote core: %s reason: %u\n",
             RPTUN_GET_CPUNAME(priv->dev), RPTUN_REASON_GET(status));
      if (RPTUN_STATUS_CHECK(status, BOARDIOC_SOFTRESETCAUSE_POWEROFF))
        {
#ifdef CONFIG_BOARDCTL_POWEROFF
          work_queue(HPWORK, &priv->work, rptun_boardctl_work,
            (FAR void *)(uintptr_t)BOARDIOC_SOFTRESETCAUSE_POWEROFF, 0);
#else
          rptun_set_status_notify(priv,
            (unsigned long)BOARDIOC_SOFTRESETCAUSE_POWEROFF);
#endif
        }
      else
        {
#ifdef CONFIG_BOARDCTL_RESET
          work_queue(HPWORK, &priv->work, rptun_boardctl_work,
                     (FAR void *)(uintptr_t)RPTUN_REASON_GET(status), 0);
#else
          rptun_set_status_notify(priv,
            (unsigned long)RPTUN_REASON_GET(status));
#endif
        }
    }
}

static int rptun_callback(FAR void *arg, uint32_t vqid)
{
  FAR struct rptun_priv_s *priv = arg;

  rptun_check_peer_status(priv);
  return remoteproc_get_notification(&priv->rproc, vqid);
}

/****************************************************************************
 * Name: rptun_update_coredump_regions
 *
 * Description:
 *   Scan RSC_CARVEOUT entries with name prefix "coredump" and register
 *   or unregister them from the system coredump list.
 *
 ****************************************************************************/

#ifdef CONFIG_COREDUMP
static void rptun_update_coredump_regions(FAR struct rptun_priv_s *priv,
                                          bool add)
{
  FAR char *rsc = priv->rproc.rsc_table;
  FAR struct fw_rsc_carveout *carv;
  size_t off;
  uint32_t i;

  if (!RPTUN_IS_MASTER(priv->dev) || rsc == NULL)
    {
      return;
    }

  for (i = 0; ; i++)
    {
      off = find_rsc(rsc, RSC_CARVEOUT, i);
      if (off == 0u)
        {
          break;
        }

      carv = (FAR struct fw_rsc_carveout *)(rsc + off);
      if (strncmp((FAR const char *)carv->name, "coredump", 8) == 0)
        {
          metal_phys_addr_t da = carv->da;
          FAR void *va;

          va = remoteproc_mmap(&priv->rproc, NULL, &da,
                               carv->len, 0, NULL);
          if (va != NULL)
            {
              if (add)
                {
                  coredump_add_memory_region(va, carv->len, carv->flags);
                }
              else
                {
                  coredump_remove_memory_region(va, carv->len);
                }
            }
        }
    }
}
#endif

static int rptun_do_start(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;
  FAR struct resource_table *rsc;
  int ret;

  ret = remoteproc_config(rproc, NULL);
  if (ret >= 0)
    {
#ifdef CONFIG_RPTUN_LOADER
      if (RPTUN_GET_FIRMWARE(priv->dev))
        {
          struct rptun_store_s store =
          {
            0
          };

          ret = remoteproc_load(rproc, RPTUN_GET_FIRMWARE(priv->dev),
                                &store, &g_rptun_store_ops, NULL);
          if (ret >= 0)
            {
              rsc = rproc->rsc_table;
            }
          else
            {
              rptunerr("remoteproc load failed, ret=%d\n", ret);
            }
        }
      else
#endif
        {
          rsc = RPTUN_GET_RESOURCE(priv->dev);
          if (rsc != NULL)
            {
              ret = remoteproc_set_rsc_table(rproc,
                                             (struct resource_table *)rsc,
                                             sizeof(struct rptun_rsc_s));
              if (ret < 0)
                {
                  rptunerr("remoteproc set rsc_table failed, ret=%d\n", ret);
                }
            }
          else
            {
              rptunerr("RPTUN_GET_RESOURCE failed\n");
              ret = -EINVAL;
            }
        }

      /* Remote proc start */

      if (ret < 0)
        {
          /* remoteproc_config() left it READY, which only
           * remoteproc_start() leaves; go back so that rptun_dev_stop()
           * and a later start see an idle core.
           */

          metal_mutex_acquire(&rproc->lock);
          rproc->state = RPROC_OFFLINE;
          metal_mutex_release(&rproc->lock);
        }
      else
        {
          ret = remoteproc_start(rproc);
          if (ret >= 0)
            {
              size_t off = find_rsc((FAR char *)rsc, RSC_TRACE, 0);

              if (off != 0u)
                {
                  FAR struct fw_rsc_trace *trace =
                    (FAR struct fw_rsc_trace *)((FAR char *)rsc + off);
                  metal_phys_addr_t da = trace->da;

                  priv->trace_va = remoteproc_mmap(rproc, NULL, &da,
                                                   trace->len, 0, NULL);
                }

#ifdef CONFIG_COREDUMP
              /* Register dump regions after remote core is started,
               * at which point the memory layout is determined.
               */

              rptun_update_coredump_regions(priv, true);
#endif

              /* Register callback to mbox for receiving remote message */

              RPTUN_REGISTER_CALLBACK(priv->dev, rptun_callback, priv);
            }
          else
            {
              remoteproc_shutdown(rproc);
              rptunerr("remoteproc_start failed, ret=%d\n", ret);
            }
        }
    }
  else
    {
      rptunerr("remoteproc config failed, ret=%d\n", ret);
    }

  return ret;
}

static int rptun_start_thread(int argc, FAR char *argv[])
{
  FAR struct rptun_priv_s *priv =
  (FAR struct rptun_priv_s *)((uintptr_t)strtoul(argv[argc - 1], NULL, 16));
  int ret;

  ret = rptun_do_start(&priv->rproc);
  if (ret >= 0)
    {
      FAR struct fw_rsc_carveout *rsc;
      FAR const char *name;

      /* Build the pool the devices without a region of their own allocate
       * from.  Each side gets its own carveout, because both read the same
       * table: two heaps over one region would hand out the same memory
       * twice.  The master takes "vdevbuffer" and the slave "hdevbuffer".
       */

      if (priv->heap == NULL)
        {
          name = RPTUN_IS_MASTER(priv->dev) ? "vdevbuffer" : "hdevbuffer";
          rsc = rptun_find_carveout(priv->rproc.rsc_table, name);
          if (rsc != NULL)
            {
              priv->heap = rptun_create_heap(priv, priv->poolname,
                                             rsc->name, rsc->da, NULL,
                                             rsc->len);
            }
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
    }

  return ret;
}

static int rptun_dev_start(FAR struct rptun_priv_s *priv)
{
  FAR struct rptun_dev_s *dev = priv->dev;
  FAR char *argv[3];
  char arg1[32];

  snprintf(arg1, sizeof(arg1), "%p", priv);
  argv[0] = (FAR char *)RPTUN_GET_CPUNAME(dev);
  argv[1] = arg1;
  argv[2] = NULL;

#ifdef CONFIG_RPTUN_START_SYNC
  /* Leave priv->pid unset: rptun_dev_stop() kills that pid, and here it
   * would be the caller.
   */

  return rptun_start_thread(2, argv);
#else
  /* Create a thread to register the virtio and vhost devices */

  if (dev->stack != NULL && dev->stack_size != 0u)
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
#endif
}

static int rptun_dev_stop(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;
  int ret = OK;

  if (priv->rproc.state == RPROC_CONFIGURED ||
      priv->rproc.state == RPROC_READY)
    {
      ret = -EBUSY;
    }
  else if (priv->rproc.state != RPROC_OFFLINE)
    {
      if (priv->pid >= 0)
        {
          priv->stop = true;
          nxsig_kill(priv->pid, SIGKILL);
#ifdef CONFIG_SCHED_WAITPID
          nxsched_waitpid(priv->pid, NULL, WEXITED);
#endif
          priv->stop = false;
          priv->pid = -EINVAL;
        }

      RPTUN_UNREGISTER_CALLBACK(priv->dev);
      rptun_remove_devices(priv);

      if (priv->heap != NULL)
        {
          mm_uninitialize(priv->heap);
          priv->heap = NULL;
        }

#ifdef CONFIG_COREDUMP
      rptun_update_coredump_regions(priv, false);
#endif
      remoteproc_shutdown(rproc);
    }

  return ret;
}

static int rptun_dev_reset(FAR struct rptun_priv_s *priv, unsigned long val)
{
  int timeout = CONFIG_RPTUN_STATUS_TIMEOUT_MS;
  int ret = -ENOTSUP;

  if (priv->rproc.rsc_table == NULL)
    {
      return -EINVAL;
    }

  if (priv->dev->ops->reset)
    {
      _emerg("Rptun driver reset remote %s reason: %lu\n",
             RPTUN_GET_CPUNAME(priv->dev), val);
      ret = priv->dev->ops->reset(priv->dev, val);
      _emerg("Rptun driver reset remote %s ret %d\n",
             RPTUN_GET_CPUNAME(priv->dev), ret);
    }

  if (ret == -ENOTSUP)
    {
      if (RPTUN_STATUS_CHECK(priv->lstatus, val))
        {
          rptun_set_status(priv, val);
          return OK;
        }

      RPTUN_STATUS_SET(priv->lstatus, val);
      rptun_set_status_notify(priv, val);

      if (val == RPTUN_STATUS_PAUSE)
        {
          return OK;
        }

      ret = -ETIMEDOUT;
      while (timeout-- > 0)
        {
          if (RPTUN_STATUS_CHECK(rptun_get_peer_status(priv), val))
            {
              ret = OK;
              break;
            }

          if (timeout % 1000 == 0)
            {
              _emerg("Waiting %s reset, timeout left %d ms\n",
                     RPTUN_GET_CPUNAME(priv->dev), timeout);
            }

          if (up_interrupt_context())
            {
              up_udelay(1000);
            }
          else
            {
              nxsig_usleep(1000);
            }
        }
    }

  return ret;
}

static void rptun_dev_dump(FAR struct rptun_priv_s *priv,
                           unsigned long arg)
{
  UNUSED(arg);

#ifdef CONFIG_RPTUN_RSC_DUMP
  if (arg & RPTUN_DUMP_RSC)
    {
      rptun_dump_resource(priv->rproc.rsc_table);
    }
#endif

#if defined(CONFIG_SYSLOG_RPMSG_SERVER) && defined(CONFIG_RPTUN_TRACE_DUMP)
  if ((arg & RPTUN_DUMP_SYSLOG) && priv->trace_va != NULL)
    {
      _emerg("Dump %s's trace\n", RPTUN_GET_CPUNAME(priv->dev));
      syslog_rpmsg_dump(priv->trace_va);
    }
#endif
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
        ret = rptun_dev_reset(priv, arg);
        break;
      case RPTUNIOC_DUMP:
        rptun_dev_dump(priv, arg);
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rptun_dev_open(FAR struct file *filep)
{
  FAR struct rptun_priv_s *priv = filep->f_inode->i_private;

  nxrmutex_lock(&g_rptun_lock);
  priv->crefs++;
  nxrmutex_unlock(&g_rptun_lock);
  return OK;
}

static int rptun_dev_close(FAR struct file *filep)
{
  FAR struct rptun_priv_s *priv = filep->f_inode->i_private;
  bool release;

  nxrmutex_lock(&g_rptun_lock);
  release = --priv->crefs == 0 && priv->unlinked;
  nxrmutex_unlock(&g_rptun_lock);

  if (release)
    {
      kmm_free(priv);
    }

  return OK;
}

static int rptun_dev_unlink(FAR struct inode *inode)
{
  FAR struct rptun_priv_s *priv = inode->i_private;
  bool release;
  int ret;

  ret = rptun_dev_stop(&priv->rproc);
  if (ret < 0)
    {
      return ret;
    }

  /* Not queued when unlinked by a failed rptun_initialize() */

  nxrmutex_lock(&g_rptun_lock);
  if (dq_inqueue(&priv->entry, &g_rptun_priv))
    {
      dq_rem(&priv->entry, &g_rptun_priv);
    }

  nxrmutex_unlock(&g_rptun_lock);

  remoteproc_remove(&priv->rproc);
  work_cancel_sync(HPWORK, &priv->work);

  /* An open file keeps the inode, and so priv, until its last close */

  nxrmutex_lock(&g_rptun_lock);
  priv->unlinked = true;
  release = priv->crefs == 0;
  nxrmutex_unlock(&g_rptun_lock);

  if (release)
    {
      kmm_free(priv);
    }

  return OK;
}
#endif

static int rptun_dev_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct rptun_priv_s *priv = filep->f_inode->i_private;

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      return -ENODEV;
    }
#endif

  return rptun_do_ioctl(priv, cmd, arg);
}

static int rptun_ioctl_foreach(FAR const char *cpuname, int cmd,
                               unsigned long value)
{
  FAR struct rptun_priv_s *priv;
  int ret = OK;

  if (!up_interrupt_context())
    {
      nxrmutex_lock(&g_rptun_lock);
    }

  dq_for_every_entry(&g_rptun_priv, priv, struct rptun_priv_s, entry)
    {
      if (!cpuname || !strcmp(RPTUN_GET_CPUNAME(priv->dev), cpuname))
        {
          int ret1 = rptun_do_ioctl(priv, cmd, value);

          if (ret1 < 0)
            {
              ret = ret1;
            }
        }
    }

  if (!up_interrupt_context())
    {
      nxrmutex_unlock(&g_rptun_lock);
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
  metal_phys_addr_t da = pa;
  uint32_t i;

  addrenv = RPTUN_GET_ADDRENV(dev);
  if (addrenv != NULL)
    {
      for (i = 0; addrenv[i].size; i++)
        {
          if (pa - addrenv[i].pa < addrenv[i].size)
            {
              da = addrenv[i].da + (pa - addrenv[i].pa);
              break;
            }
        }
    }

  return da;
}

static metal_phys_addr_t rptun_da_to_pa(FAR struct rptun_dev_s *dev,
                                        metal_phys_addr_t da)
{
  FAR const struct rptun_addrenv_s *addrenv;
  metal_phys_addr_t pa = da;
  uint32_t i;

  addrenv = RPTUN_GET_ADDRENV(dev);
  if (addrenv != NULL)
    {
      for (i = 0; addrenv[i].size; i++)
        {
          if (da - addrenv[i].da < addrenv[i].size)
            {
              pa = addrenv[i].pa + (da - addrenv[i].da);
              break;
            }
        }
    }

  return pa;
}

#ifndef CONFIG_RPTUN_AUTO_RESET_DISABLE
static int rptun_notifier(FAR struct notifier_block *block,
                          unsigned long action, void *data)
{
  unsigned long val;

  if (block == &g_rptun_reboot_nb)
    {
      if (action == SYS_POWER_OFF)
        {
          val = BOARDIOC_SOFTRESETCAUSE_POWEROFF;
        }
      else if (action == SYS_HALT)
        {
          val = BOARDIOC_SOFTRESETCAUSE_PANIC;
        }
      else
        {
          val = (unsigned long)data;
        }

      rptun_ioctl_foreach(NULL, RPTUNIOC_RESET, val);

      if (action == SYS_HALT)
        {
          rptun_ioctl_foreach(NULL, RPTUNIOC_DUMP, RPTUN_DUMP_SYSLOG);
        }
    }
  else if (action == PANIC_KERNEL)
    {
      rptun_ioctl_foreach(NULL, RPTUNIOC_RESET,
                          (unsigned long)RPTUN_STATUS_PAUSE);
    }
#ifdef CONFIG_RPTUN_AUTO_RESET_IN_PANIC_NOTIFIER
  else if (action == PANIC_KERNEL_FINAL)
    {
      rptun_ioctl_foreach(NULL, RPTUNIOC_RESET,
                          (unsigned long)BOARDIOC_SOFTRESETCAUSE_PANIC);
    }
#endif

  return 0;
}
#endif

static void rptun_dump_vdev_rsc(FAR void *rsc, uint32_t index)
{
  FAR struct fw_rsc_vdev *vdev = rsc;
  uint8_t i;

  rptunvbs("[vdev] Rsc %" PRIu32 " %p\n", index, rsc);
  rptunvbs("[vdev] VirtIO %s\n", virtio_dev_name(vdev->id));
  rptunvbs("[vdev] id %" PRIu32 " notifyid %" PRIu32
           " dfeatures 0x%08" PRIx32 " gfeatures 0x%08" PRIx32 "\n",
           vdev->id, vdev->notifyid, vdev->dfeatures, vdev->gfeatures);
  rptunvbs("[vdev] config_len %" PRIu32 " status %u num_of_vrings %u"
           " reserved[0] %u reserved[1] %u\n",
           vdev->config_len, vdev->status, vdev->num_of_vrings,
           vdev->reserved[0], vdev->reserved[1]);

  for (i = 0; i < vdev->num_of_vrings; i++)
    {
      FAR struct fw_rsc_vdev_vring *vr = &vdev->vring[i];

      rptunvbs("[vdev] [vring%u] da 0x%08" PRIx32 " align %" PRIu32
               " num %" PRIu32 " notifyid %" PRIu32
               " reserved %" PRIx32 "\n",
               i, vr->da, vr->align, vr->num, vr->notifyid, vr->reserved);
    }

  if (vdev->config_len != 0)
    {
      lib_dumpbuffer("[vdev] [config]",
                     (FAR uint8_t *)&vdev->vring[vdev->num_of_vrings],
                     vdev->config_len);
    }
}

static void rptun_dump_carveout_rsc(FAR void *rsc, uint32_t index)
{
  FAR struct fw_rsc_carveout *car = rsc;

  rptunvbs("[carveout] Rsc %" PRIu32 " %p\n", index, rsc);
  rptunvbs("[carveout] Carveout %s\n", car->name);
  rptunvbs("[carveout] da 0x%08" PRIx32 " pa 0x%08" PRIx32 " len %" PRIu32
           " flags 0x%" PRIx32 " reserved %" PRIx32 "\n",
           car->da, car->pa, car->len, car->flags, car->reserved);
}

static void rptun_dump_trace_rsc(FAR void *rsc, uint32_t index)
{
  FAR struct fw_rsc_trace *trc = rsc;

  rptunvbs("[trace] Rsc %" PRIu32 " %p\n", index, rsc);
  rptunvbs("[trace] Trace %s\n", trc->name);
  rptunvbs("[trace] da 0x%08" PRIx32 " len %" PRIu32
           " reserved %" PRIu32 "\n",
           trc->da, trc->len, trc->reserved);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rptun_initialize(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_priv_s *priv;
  char name[32];
  int ret = -ENOMEM;

  priv = kmm_zalloc(sizeof(struct rptun_priv_s));
  if (priv != NULL)
    {
#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
      /* Secure and Non-secure worlds share the same CPU. If the
       * secure side handles PAUSE, the CPU stalls in secure world
       * blocking the non-secure side as well.
       */

      RPTUN_STATUS_SET(priv->lstatus, RPTUN_STATUS_PAUSE);
#endif
      priv->dev = dev;
      priv->pid = -EINVAL;
      remoteproc_init(&priv->rproc, &g_rptun_ops, priv);

      snprintf(name, sizeof(name), "/dev/rptun/%s", RPTUN_GET_CPUNAME(dev));
      ret = register_driver(name, &g_rptun_fops, 0200, priv);
      if (ret >= 0)
        {
          if (RPTUN_IS_AUTOSTART(priv->dev))
            {
              ret = rptun_dev_start(priv);
              if (ret < 0)
                {
                  unregister_driver(name);
#ifdef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
                  kmm_free(priv);
#endif
                  rptunerr("rptun start failed %d\n", ret);
                }
            }

          if (ret >= 0)
            {
#ifdef CONFIG_RPTUN_AUTO_RESET_IN_PANIC_NOTIFIER
              panic_notifier_chain_register(&g_rptun_panic_nb);
#endif
#ifndef CONFIG_RPTUN_AUTO_RESET_DISABLE
              register_reboot_notifier(&g_rptun_reboot_nb);
#endif

              nxrmutex_lock(&g_rptun_lock);
              dq_addlast(&priv->entry, &g_rptun_priv);
              nxrmutex_unlock(&g_rptun_lock);
            }
        }
      else
        {
          kmm_free(priv);
          rptunerr("rptun register driver failed %d\n", ret);
        }
    }

  return ret;
}

void rptun_dump_resource(FAR const struct resource_table *rsc)
{
  FAR struct fw_rsc_hdr *hdr;
  uint32_t i;

  if (rsc == NULL)
    {
      rptunerr("Resource table is NULL\n");
      return;
    }

  rptunvbs("Dump resource table: %p num: %" PRIu32 "\n", rsc, rsc->num);
  for (i = 0; i < rsc->num; i++)
    {
      hdr = (FAR struct fw_rsc_hdr *)((FAR char *)rsc + rsc->offset[i]);
      switch (hdr->type)
        {
          case RSC_VDEV:
            rptun_dump_vdev_rsc(hdr, i);
            break;
          case RSC_CARVEOUT:
            rptun_dump_carveout_rsc(hdr, i);
            break;
          case RSC_TRACE:
            rptun_dump_trace_rsc(hdr, i);
            break;
          default:
            rptunerr("Not support rsc type: %" PRIu32 " %" PRIu32 "\n",
                     i, hdr->type);
            break;
        }
    }
}

int rptun_uninitialize(FAR const char *cpuname)
{
  char name[32];

  snprintf(name, sizeof(name), "/dev/rptun/%s", cpuname);
  return unregister_driver(name);
}

int rptun_boot(FAR const char *cpuname)
{
  return rptun_ioctl_foreach(cpuname, RPTUNIOC_START, 0);
}

int rptun_poweroff(FAR const char *cpuname)
{
  return rptun_ioctl_foreach(cpuname, RPTUNIOC_STOP, 0);
}

int rptun_reset(FAR const char *cpuname, unsigned long value)
{
  return rptun_ioctl_foreach(cpuname, RPTUNIOC_RESET, value);
}
