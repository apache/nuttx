/****************************************************************************
 * drivers/virtio/virtio-mmio.c
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
#include <errno.h>
#include <stdint.h>
#include <sys/param.h>

#include <nuttx/arch.h>
#include <nuttx/mm/kasan.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/virtio/virtio-mmio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRITO_PAGE_SHIFT               12
#define VIRTIO_PAGE_SIZE                (1 << VIRITO_PAGE_SHIFT)
#define VIRTIO_VRING_ALIGN              VIRTIO_PAGE_SIZE

#define VIRTIO_MMIO_VERSION_1           1

/* Control registers */

/* Magic value ("virt" string) - Read Only */

#define VIRTIO_MMIO_MAGIC_VALUE         0x000
#define VIRTIO_MMIO_MAGIC_VALUE_STRING  ('v' | ('i' << 8) | ('r' << 16) | ('t' << 24))

/* Virtio device version - Read Only */

#define VIRTIO_MMIO_VERSION             0x004

/* Virtio device ID - Read Only */

#define VIRTIO_MMIO_DEVICE_ID           0x008

/* Virtio vendor ID - Read Only */

#define VIRTIO_MMIO_VENDOR_ID           0x00c

/* Bitmask of the features supported by the device (host)
 * (32 bits per set) - Read Only
 */

#define VIRTIO_MMIO_DEVICE_FEATURES     0x010

/* Device (host) features set selector - Write Only */

#define VIRTIO_MMIO_DEVICE_FEATURES_SEL 0x014

/* Bitmask of features activated by the driver (guest)
 * (32 bits per set) - Write Only
 */

#define VIRTIO_MMIO_DRIVER_FEATURES     0x020

/* Activated features set selector - Write Only */

#define VIRTIO_MMIO_DRIVER_FEATURES_SEL 0x024

/* [VERSION 1 REGISTER] Guest page size */

#define VIRTIO_MMIO_PAGE_SIZE           0X028

/* Queue selector - Write Only */

#define VIRTIO_MMIO_QUEUE_SEL           0x030

/* Maximum size of the currently selected queue - Read Only */

#define VIRTIO_MMIO_QUEUE_NUM_MAX       0x034

/* Queue size for the currently selected queue - Write Only */

#define VIRTIO_MMIO_QUEUE_NUM           0x038

/* [VERSION 1 REGISTER] Used Ring alignment in the virtual queue */

#define VIRTIO_MMIO_QUEUE_ALIGN         0x03c

/* [VERSION 1 REGISTER] Guest physical page number of the virtual queue
 * Writing to this register notifies the device about location
 */

#define VIRTIO_MMIO_QUEUE_PFN           0x040

/* Ready bit for the currently selected queue - Read Write */

#define VIRTIO_MMIO_QUEUE_READY         0x044

/* Queue notifier - Write Only */

#define VIRTIO_MMIO_QUEUE_NOTIFY        0x050

/* Interrupt status - Read Only */

#define VIRTIO_MMIO_INTERRUPT_STATUS    0x060

/* Interrupt acknowledge - Write Only */

#define VIRTIO_MMIO_INTERRUPT_ACK       0x064
#define VIRTIO_MMIO_INTERRUPT_VRING     (1 << 0)
#define VIRTIO_MMIO_INTERRUPT_CONFIG    (1 << 1)

/* Device status register - Read Write */

#define VIRTIO_MMIO_STATUS              0x070

/* Selected queue's Descriptor Table address, 64 bits in two halves */

#define VIRTIO_MMIO_QUEUE_DESC_LOW      0x080
#define VIRTIO_MMIO_QUEUE_DESC_HIGH     0x084

/* Selected queue's Available Ring address, 64 bits in two halves */

#define VIRTIO_MMIO_QUEUE_AVAIL_LOW     0x090
#define VIRTIO_MMIO_QUEUE_AVAIL_HIGH    0x094

/* Selected queue's Used Ring address, 64 bits in two halves */

#define VIRTIO_MMIO_QUEUE_USED_LOW      0x0a0
#define VIRTIO_MMIO_QUEUE_USED_HIGH     0x0a4

/* Shared memory region id */

#define VIRTIO_MMIO_SHM_SEL             0x0ac

/* Shared memory region length, 64 bits in two halves */

#define VIRTIO_MMIO_SHM_LEN_LOW         0x0b0
#define VIRTIO_MMIO_SHM_LEN_HIGH        0x0b4

/* Shared memory region base address, 64 bits in two halves */

#define VIRTIO_MMIO_SHM_BASE_LOW        0x0b8
#define VIRTIO_MMIO_SHM_BASE_HIGH       0x0bc

/* Configuration atomicity value */

#define VIRTIO_MMIO_CONFIG_GENERATION   0x0fc

/* The config space is defined by each driver as
 * the per-driver configuration space - Read Write
 */

#define VIRTIO_MMIO_CONFIG              0x100

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct virtio_mmio_device_s
{
  struct virtio_device   vdev;       /* Virtio device */
  struct metal_io_region shm_io;     /* Share memory io region, virtqueue
                                      * use this io.
                                      */
  struct metal_io_region cfg_io;     /* Config memory io region, used to
                                      * read/write mmio register
                                      */
  metal_phys_addr_t      shm_phy;    /* Share memory physical address */
  metal_phys_addr_t      cfg_phy;    /* Config memory physical address */
  int                    irq;        /* The mmio interrupt number */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

static uint32_t virtio_mmio_get_queue_len(FAR struct metal_io_region *io,
                                          int idx);
static int virtio_mmio_config_virtqueue(FAR struct metal_io_region *io,
                                        FAR struct virtqueue *vq);
static FAR void *virtio_mmio_alloc_buf(FAR struct virtio_device *vdev,
                                       size_t size, size_t align);
static void virtio_mmio_free_buf(FAR struct virtio_device *vdev,
                                 FAR void *buf);
static int virtio_mmio_init_device(FAR struct virtio_mmio_device_s *vmdev,
                                   FAR void *regs, int irq);

/* Virtio mmio dispatch functions */

static int
virtio_mmio_create_virtqueue(FAR struct virtio_mmio_device_s *vmdev,
                             unsigned int i, FAR const char *name,
                             vq_callback callback);
static int virtio_mmio_create_virtqueues(FAR struct virtio_device *vdev,
                                         unsigned int flags,
                                         unsigned int nvqs,
                                         FAR const char *names[],
                                         vq_callback callbacks[],
                                         FAR void *callback_args[]);
static void virtio_mmio_delete_virtqueues(FAR struct virtio_device *vdev);
static void virtio_mmio_set_status(FAR struct virtio_device *vdev,
                                   uint8_t status);
static uint8_t virtio_mmio_get_status(FAR struct virtio_device *vdev);
static void virtio_mmio_write_config(FAR struct virtio_device *vdev,
                                     uint32_t offset, void *dst,
                                     int length);
static void virtio_mmio_read_config(FAR struct virtio_device *vdev,
                                    uint32_t offset, FAR void *dst,
                                    int length);
static uint64_t virtio_mmio_get_features(FAR struct virtio_device *vdev);
static void virtio_mmio_set_features(FAR struct virtio_device *vdev,
                                     uint64_t features);
static uint64_t virtio_mmio_negotiate_features(struct virtio_device *vdev,
                                               uint64_t features);
static void virtio_mmio_reset_device(FAR struct virtio_device *vdev);
static void virtio_mmio_notify(FAR struct virtqueue *vq);

/* Interrupt */

static int virtio_mmio_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct virtio_dispatch g_virtio_mmio_dispatch =
{
  virtio_mmio_create_virtqueues,  /* create_virtqueues */
  virtio_mmio_delete_virtqueues,  /* delete_virtqueues */
  virtio_mmio_get_status,         /* get_status */
  virtio_mmio_set_status,         /* set_status */
  virtio_mmio_get_features,       /* get_features */
  virtio_mmio_set_features,       /* set_features */
  virtio_mmio_negotiate_features, /* negotiate_features */
  virtio_mmio_read_config,        /* read_config */
  virtio_mmio_write_config,       /* write_config */
  virtio_mmio_reset_device,       /* reset_device */
  virtio_mmio_notify,             /* notify */
};

static const struct virtio_memory_ops g_virtio_mmio_mmops =
{
  virtio_mmio_alloc_buf, /* Alloc */
  virtio_mmio_free_buf,  /* Free */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_mmio_get_queue_len
 ****************************************************************************/

static uint32_t virtio_mmio_get_queue_len(FAR struct metal_io_region *io,
                                          int idx)
{
  uint32_t len;

  /* Select the queue we're interested in */

  metal_io_write32(io, VIRTIO_MMIO_QUEUE_SEL, idx);
  len = metal_io_read32(io, VIRTIO_MMIO_QUEUE_NUM_MAX);

  if (CONFIG_DRIVERS_VIRTIO_MMIO_QUEUE_LEN != 0)
    {
      len = MIN(len, CONFIG_DRIVERS_VIRTIO_MMIO_QUEUE_LEN);
    }

  return len;
}

/****************************************************************************
 * Name: virtio_mmio_config_virtqueue
 ****************************************************************************/

static int virtio_mmio_config_virtqueue(FAR struct metal_io_region *io,
                                        FAR struct virtqueue *vq)
{
  uint32_t version = vq->vq_dev->id.version;
  uint64_t addr;

  /* Select the queue we're interested in */

  metal_io_write32(io, VIRTIO_MMIO_QUEUE_SEL, vq->vq_queue_index);

  /* Queue shouldn't already be set up. */

  if (metal_io_read32(io, version == VIRTIO_MMIO_VERSION_1 ?
                      VIRTIO_MMIO_QUEUE_PFN : VIRTIO_MMIO_QUEUE_READY))
    {
      vrterr("Virtio queue not ready\n");
      return -ENOENT;
    }

  /* Activate the queue */

  if (version == VIRTIO_MMIO_VERSION_1)
    {
      uint64_t pfn = (uintptr_t)vq->vq_ring.desc >> VIRITO_PAGE_SHIFT;

      vrtinfo("Legacy, desc=%p, pfn=0x%" PRIx64 ", align=%d\n",
              vq->vq_ring.desc, pfn, VIRTIO_PAGE_SIZE);

      /* virtio-mmio v1 uses a 32bit QUEUE PFN. If we have something
       * that doesn't fit in 32bit, fail the setup rather than
       * pretending to be successful.
       */

      if (pfn >> 32)
        {
          vrterr("Legacy virtio-mmio used RAM shoud not above 0x%llxGB\n",
                 0x1ull << (2 + VIRITO_PAGE_SHIFT));
        }

      metal_io_write32(io, VIRTIO_MMIO_QUEUE_NUM, vq->vq_nentries);
      metal_io_write32(io, VIRTIO_MMIO_QUEUE_ALIGN, VIRTIO_PAGE_SIZE);
      metal_io_write32(io, VIRTIO_MMIO_QUEUE_PFN, pfn);
    }
  else
    {
      metal_io_write32(io, VIRTIO_MMIO_QUEUE_NUM, vq->vq_nentries);

      addr = (uint64_t)((uintptr_t)
                        kasan_reset_tag((FAR void *)vq->vq_ring.desc));
      metal_io_write32(io, VIRTIO_MMIO_QUEUE_DESC_LOW, addr);
      metal_io_write32(io, VIRTIO_MMIO_QUEUE_DESC_HIGH, addr >> 32);

      addr = (uint64_t)((uintptr_t)
                        kasan_reset_tag((FAR void *)vq->vq_ring.avail));
      metal_io_write32(io, VIRTIO_MMIO_QUEUE_AVAIL_LOW, addr);
      metal_io_write32(io, VIRTIO_MMIO_QUEUE_AVAIL_HIGH, addr >> 32);

      addr = (uint64_t)((uintptr_t)
                        kasan_reset_tag((FAR void *)vq->vq_ring.used));
      metal_io_write32(io, VIRTIO_MMIO_QUEUE_USED_LOW, addr);
      metal_io_write32(io, VIRTIO_MMIO_QUEUE_USED_HIGH, addr >> 32);

      metal_io_write32(io, VIRTIO_MMIO_QUEUE_READY, 1);
    }

  return OK;
}

/****************************************************************************
 * Name: virtio_mmio_create_virtqueue
 ****************************************************************************/

static int
virtio_mmio_create_virtqueue(FAR struct virtio_mmio_device_s *vmdev,
                             unsigned int i, FAR const char *name,
                             vq_callback callback)
{
  FAR struct virtio_device *vdev = &vmdev->vdev;
  FAR struct virtio_vring_info *vrinfo;
  FAR struct vring_alloc_info *vralloc;
  FAR struct virtqueue *vq;
  int vringsize;
  int ret;

  /* Alloc virtqueue and init the vring info and vring alloc info */

  vrinfo  = &vdev->vrings_info[i];
  vralloc = &vrinfo->info;
  vralloc->num_descs = virtio_mmio_get_queue_len(&vmdev->cfg_io, i);
  vq = virtqueue_allocate(vralloc->num_descs);
  if (vq == NULL)
    {
      vrterr("virtqueue_allocate failed\n");
      return -ENOMEM;
    }

  /* Init the vring info and vring alloc info */

  vrinfo->vq       = vq;
  vrinfo->io       = &vmdev->shm_io;
  vrinfo->notifyid = i;
  vralloc->align   = VIRTIO_VRING_ALIGN;
  vringsize        = vring_size(vralloc->num_descs, VIRTIO_VRING_ALIGN);
  vralloc->vaddr   = virtio_zalloc_buf(vdev, vringsize, VIRTIO_VRING_ALIGN);
  if (vralloc->vaddr == NULL)
    {
      vrterr("vring alloc failed\n");
      return -ENOMEM;
    }

  /* Initialize the virtio queue */

  ret = virtqueue_create(vdev, i, name, vralloc, callback,
                         vdev->func->notify, vq);
  if (ret < 0)
    {
      vrterr("virtqueue create error, ret=%d\n", ret);
      return ret;
    }

  virtqueue_set_shmem_io(vq, &vmdev->shm_io);

  /* Set the mmio virtqueue register */

  ret = virtio_mmio_config_virtqueue(&vmdev->cfg_io, vq);
  if (ret < 0)
    {
      vrterr("virtio_mmio_config_virtqueue failed, ret=%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_mmio_create_virtqueues
 ****************************************************************************/

static int virtio_mmio_create_virtqueues(FAR struct virtio_device *vdev,
                                         unsigned int flags,
                                         unsigned int nvqs,
                                         FAR const char *names[],
                                         vq_callback callbacks[],
                                         FAR void *callback_args[])
{
  FAR struct virtio_mmio_device_s *vmdev =
    (FAR struct virtio_mmio_device_s *)vdev;
  unsigned int i;
  int ret = OK;

  /* Alloc vring info */

  vdev->vrings_num = nvqs;
  vdev->vrings_info = kmm_zalloc(sizeof(struct virtio_vring_info) * nvqs);
  if (vdev->vrings_info == NULL)
    {
      vrterr("alloc vrings info failed\n");
      return -ENOMEM;
    }

  /* Alloc and init the virtqueue */

  for (i = 0; i < nvqs; i++)
    {
      ret = virtio_mmio_create_virtqueue(vmdev, i, names[i], callbacks[i]);
      if (ret < 0)
        {
          goto err;
        }
    }

  /* Finally, enable the interrupt */

  up_enable_irq(vmdev->irq);
  return ret;

err:
  virtio_mmio_delete_virtqueues(vdev);
  return ret;
}

/****************************************************************************
 * Name: virtio_mmio_delete_virtqueues
 ****************************************************************************/

static void virtio_mmio_delete_virtqueues(FAR struct virtio_device *vdev)
{
  FAR struct virtio_mmio_device_s *vmdev =
    (FAR struct virtio_mmio_device_s *)vdev;
  FAR struct virtio_vring_info *vrinfo;
  unsigned int i;

  /* Disable interrupt first */

  up_disable_irq(vmdev->irq);

  /* Free the memory */

  if (vdev->vrings_info != NULL)
    {
      for (i = 0; i < vdev->vrings_num; i++)
        {
          metal_io_write32(&vmdev->cfg_io, VIRTIO_MMIO_QUEUE_SEL, i);
          if (vdev->id.version == VIRTIO_MMIO_VERSION_1)
            {
              metal_io_write32(&vmdev->cfg_io, VIRTIO_MMIO_QUEUE_PFN, 0);
            }
          else
            {
              /* Virtio 1.2: To stop using the queue the driver MUST write
               * zero (0x0) to this QueueReady and MUST read the value back
               * to ensure synchronization.
               */

              metal_io_write32(&vmdev->cfg_io, VIRTIO_MMIO_QUEUE_READY, 0);
              if (metal_io_read32(&vmdev->cfg_io, VIRTIO_MMIO_QUEUE_READY))
                {
                  vrtwarn("queue ready set zero failed\n");
                }
            }

          /* Free the vring buffer and virtqueue */

          vrinfo = &vdev->vrings_info[i];
          if (vrinfo->info.vaddr != NULL)
            {
              virtio_free_buf(vdev, vrinfo->info.vaddr);
            }

          if (vrinfo->vq != NULL)
            {
              virtqueue_free(vrinfo->vq);
            }
        }

      kmm_free(vdev->vrings_info);
    }
}

/****************************************************************************
 * Name: virtio_mmio_set_status
 ****************************************************************************/

static void virtio_mmio_set_status(FAR struct virtio_device *vdev,
                                   uint8_t status)
{
  FAR struct virtio_mmio_device_s *vmdev =
    (FAR struct virtio_mmio_device_s *)vdev;
  metal_io_write32(&vmdev->cfg_io, VIRTIO_MMIO_STATUS, status);
}

/****************************************************************************
 * Name: virtio_mmio_get_status
 ****************************************************************************/

static uint8_t virtio_mmio_get_status(FAR struct virtio_device *vdev)
{
  FAR struct virtio_mmio_device_s *vmdev =
    (FAR struct virtio_mmio_device_s *)vdev;
  return metal_io_read32(&vmdev->cfg_io, VIRTIO_MMIO_STATUS);
}

/****************************************************************************
 * Name: virtio_mmio_write_config
 ****************************************************************************/

static void virtio_mmio_write_config(FAR struct virtio_device *vdev,
                                     uint32_t offset, void *src, int length)
{
  FAR struct virtio_mmio_device_s *vmdev =
    (FAR struct virtio_mmio_device_s *)vdev;
  uint32_t write_offset = VIRTIO_MMIO_CONFIG + offset;
  uint32_t u32data;
  uint16_t u16data;
  uint8_t u8data;

  if (vdev->id.version == VIRTIO_MMIO_VERSION_1)
    {
      goto byte_write;
    }

  switch (length)
    {
      case 1:
        memcpy(&u8data, src, sizeof(u8data));
        metal_io_write8(&vmdev->cfg_io, write_offset, u8data);
        break;
      case 2:
        memcpy(&u16data, src, sizeof(u16data));
        metal_io_write16(&vmdev->cfg_io, write_offset, u16data);
        break;
      case 4:
        memcpy(&u32data, src, sizeof(u32data));
        metal_io_write32(&vmdev->cfg_io, write_offset, u32data);
        break;
      case 8:
        memcpy(&u32data, src, sizeof(u32data));
        metal_io_write32(&vmdev->cfg_io, write_offset, u32data);
        memcpy(&u32data, src + sizeof(u32data), sizeof(u32data));
        metal_io_write32(&vmdev->cfg_io, write_offset + sizeof(u32data),
                         u32data);
        break;
      default:
byte_write:
        {
          FAR char *s = src;
          int i;
          for (i = 0; i < length; i++)
            {
              metal_io_write8(&vmdev->cfg_io, write_offset + i, s[i]);
            }
        }
    }
}

/****************************************************************************
 * Name: virtio_mmio_read_config
 ****************************************************************************/

static void virtio_mmio_read_config(FAR struct virtio_device *vdev,
                                    uint32_t offset, FAR void *dst,
                                    int length)
{
  FAR struct virtio_mmio_device_s *vmdev =
    (FAR struct virtio_mmio_device_s *)vdev;
  uint32_t read_offset = VIRTIO_MMIO_CONFIG + offset;
  uint32_t u32data;
  uint16_t u16data;
  uint8_t u8data;

  if (vdev->id.version == VIRTIO_MMIO_VERSION_1)
    {
      goto byte_read;
    }

  switch (length)
    {
      case 1:
        u8data = metal_io_read8(&vmdev->cfg_io, read_offset);
        memcpy(dst, &u8data, sizeof(u8data));
        break;
      case 2:
        u16data = metal_io_read16(&vmdev->cfg_io, read_offset);
        memcpy(dst, &u16data, sizeof(u16data));
        break;
      case 4:
        u32data = metal_io_read32(&vmdev->cfg_io, read_offset);
        memcpy(dst, &u32data, sizeof(u32data));
        break;
      case 8:
        u32data = metal_io_read32(&vmdev->cfg_io, read_offset);
        memcpy(dst, &u32data, sizeof(u32data));
        u32data = metal_io_read32(&vmdev->cfg_io,
                                  read_offset + sizeof(u32data));
        memcpy(dst + sizeof(u32data), &u32data, sizeof(u32data));
        break;
      default:
byte_read:
        {
          FAR char *d = dst;
          int i;
          for (i = 0; i < length; i++)
            {
              d[i] = metal_io_read8(&vmdev->cfg_io, read_offset + i);
            }
        }
    }
}

/****************************************************************************
 * Name: virtio_mmio_get_features
 ****************************************************************************/

static uint64_t virtio_mmio_get_features(FAR struct virtio_device *vdev)
{
  FAR struct virtio_mmio_device_s *vmdev =
    (FAR struct virtio_mmio_device_s *)vdev;
  uint32_t feature_lo;
  uint32_t feature_hi;

  metal_io_write32(&vmdev->cfg_io, VIRTIO_MMIO_DRIVER_FEATURES_SEL, 0);
  feature_lo = metal_io_read32(&vmdev->cfg_io, VIRTIO_MMIO_DEVICE_FEATURES);
  metal_io_write32(&vmdev->cfg_io, VIRTIO_MMIO_DRIVER_FEATURES_SEL, 1);
  feature_hi = metal_io_read32(&vmdev->cfg_io, VIRTIO_MMIO_DEVICE_FEATURES);
  return ((uint64_t)feature_hi << 32) | (uint64_t)feature_lo;
}

/****************************************************************************
 * Name: virtio_mmio_set_features
 ****************************************************************************/

static void virtio_mmio_set_features(FAR struct virtio_device *vdev,
                                     uint64_t features)
{
  FAR struct virtio_mmio_device_s *vmdev =
    (FAR struct virtio_mmio_device_s *)vdev;

  metal_io_write32(&vmdev->cfg_io, VIRTIO_MMIO_DRIVER_FEATURES_SEL, 0);
  metal_io_write32(&vmdev->cfg_io, VIRTIO_MMIO_DRIVER_FEATURES, features);
  metal_io_write32(&vmdev->cfg_io, VIRTIO_MMIO_DRIVER_FEATURES_SEL, 1);
  metal_io_write32(&vmdev->cfg_io, VIRTIO_MMIO_DRIVER_FEATURES,
                   features >> 32);
  vdev->features = features;
}

/****************************************************************************
 * Name: virtio_mmio_negotiate_features
 ****************************************************************************/

static uint64_t virtio_mmio_negotiate_features(struct virtio_device *vdev,
                                               uint64_t features)
{
  features = features & virtio_mmio_get_features(vdev);
  virtio_mmio_set_features(vdev, features);
  return features;
}

/****************************************************************************
 * Name: virtio_mmio_reset_device
 ****************************************************************************/

static void virtio_mmio_reset_device(FAR struct virtio_device *vdev)
{
  virtio_mmio_set_status(vdev, VIRTIO_CONFIG_STATUS_RESET);
}

/****************************************************************************
 * Name: virtio_mmio_notify
 ****************************************************************************/

static void virtio_mmio_notify(FAR struct virtqueue *vq)
{
  FAR struct virtio_mmio_device_s *vmdev =
    (FAR struct virtio_mmio_device_s *)vq->vq_dev;

  /* VIRTIO_F_NOTIFICATION_DATA is not supported for now */

  metal_io_write32(&vmdev->cfg_io, VIRTIO_MMIO_QUEUE_NOTIFY,
                   vq->vq_queue_index);
}

/****************************************************************************
 * Name: virtio_mmio_interrupt
 ****************************************************************************/

static int virtio_mmio_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct virtio_mmio_device_s *vmdev = arg;
  FAR struct virtio_vring_info *vrings_info = vmdev->vdev.vrings_info;
  FAR struct virtqueue *vq;
  unsigned int i;
  uint32_t isr;

  isr = metal_io_read32(&vmdev->cfg_io, VIRTIO_MMIO_INTERRUPT_STATUS);
  metal_io_write32(&vmdev->cfg_io, VIRTIO_MMIO_INTERRUPT_ACK, isr);
  if (isr & VIRTIO_MMIO_INTERRUPT_VRING)
    {
      for (i = 0; i < vmdev->vdev.vrings_num; i++)
        {
          vq = vrings_info[i].vq;
          if (vq->vq_used_cons_idx != vq->vq_ring.used->idx &&
              vq->callback != NULL)
            {
              vq->callback(vq);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: virtio_mmio_alloc_buf
 ****************************************************************************/

static FAR void *virtio_mmio_alloc_buf(FAR struct virtio_device *vdev,
                                       size_t size, size_t align)
{
  return kmm_memalign(align, size);
}

/****************************************************************************
 * Name: virtio_mmio_free_buf
 ****************************************************************************/

static void virtio_mmio_free_buf(FAR struct virtio_device *vdev,
                                 FAR void *buf)
{
  kmm_free(buf);
}

/****************************************************************************
 * Name: virtio_mmio_init_device
 ****************************************************************************/

static int virtio_mmio_init_device(FAR struct virtio_mmio_device_s *vmdev,
                                   FAR void *regs, int irq)
{
  FAR struct virtio_device *vdev = &vmdev->vdev;
  uint32_t magic;

  /* Save the irq */

  vmdev->irq = irq;

  /* Share memory io is used for the virtio buffer operations
   * Config memory is used for the mmio register operations
   */

  vmdev->shm_phy = (metal_phys_addr_t)0;
  vmdev->cfg_phy = (metal_phys_addr_t)regs;
  metal_io_init(&vmdev->shm_io, NULL, &vmdev->shm_phy,
                SIZE_MAX, UINT_MAX, 0, metal_io_get_ops());
  metal_io_init(&vmdev->cfg_io, regs, &vmdev->cfg_phy,
                SIZE_MAX, UINT_MAX, 0, metal_io_get_ops());

  /* Init the virtio device */

  vdev->role = VIRTIO_DEV_DRIVER;
  vdev->func = &g_virtio_mmio_dispatch;
  vdev->mmops = &g_virtio_mmio_mmops;

  magic = metal_io_read32(&vmdev->cfg_io, VIRTIO_MMIO_MAGIC_VALUE);
  if (magic != VIRTIO_MMIO_MAGIC_VALUE_STRING)
    {
      vrterr("Bad magic value %" PRIu32 "\n", magic);
      return -EINVAL;
    }

  vdev->id.version = metal_io_read32(&vmdev->cfg_io, VIRTIO_MMIO_VERSION);
  if (vdev->id.version < 1 || vdev->id.version > 2)
    {
      vrterr("Version %"PRIu32" not supported!\n", vdev->id.version);
      return -ENODEV;
    }

  vdev->id.device = metal_io_read32(&vmdev->cfg_io, VIRTIO_MMIO_DEVICE_ID);
  if (vdev->id.device == 0)
    {
      vrtinfo("Device Id 0\n");
      return -ENODEV;
    }

  vdev->id.vendor = metal_io_read32(&vmdev->cfg_io, VIRTIO_MMIO_VENDOR_ID);

  /* Legacy mmio version, set the page size */

  if (vdev->id.version == VIRTIO_MMIO_VERSION_1)
    {
      metal_io_write32(&vmdev->cfg_io, VIRTIO_MMIO_PAGE_SIZE,
                       VIRTIO_PAGE_SIZE);
    }

  vrtinfo("VIRTIO version: %"PRIu32" device: %"PRIu32" vendor: %"PRIx32"\n",
          vdev->id.version, vdev->id.device, vdev->id.vendor);

  /* Reset the virtio device and set ACK */

  virtio_mmio_set_status(vdev, VIRTIO_CONFIG_STATUS_RESET);
  virtio_mmio_set_status(vdev, VIRTIO_CONFIG_STATUS_ACK);
  return OK;
}

/****************************************************************************
 * Name: virtio_register_mmio_device_
 *
 * Description:
 *   Register secure or non-secure virtio mmio device to the virtio bus
 *
 ****************************************************************************/

static int virtio_register_mmio_device_(FAR void *regs, int irq, bool secure)
{
  FAR struct virtio_mmio_device_s *vmdev;
  int ret;

  DEBUGASSERT(regs != NULL);

  vmdev = kmm_zalloc(sizeof(*vmdev));
  if (vmdev == NULL)
    {
      vrterr("No enough memory\n");
      return -ENOMEM;
    }

  ret = virtio_mmio_init_device(vmdev, regs, irq);
  if (ret == -ENODEV)
    {
      vrtinfo("No virtio mmio device in regs=%p\n", regs);
      goto err;
    }
  else if (ret < 0)
    {
      vrterr("virtio_mmio_device_init failed, ret=%d\n", ret);
      goto err;
    }

#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
  if (secure)
    {
      up_secure_irq(irq, true);
    }
#else
  UNUSED(secure);
#endif

  /* Attach the intterupt before register the device driver */

  ret = irq_attach(irq, virtio_mmio_interrupt, vmdev);
  if (ret < 0)
    {
      vrterr("irq_attach failed, ret=%d\n", ret);
      goto err;
    }

  /* Register the virtio device */

  ret = virtio_register_device(&vmdev->vdev);
  if (ret < 0)
    {
      vrterr("virt_device_register failed, ret=%d\n", ret);
      irq_detach(irq);
      goto err;
    }

  return ret;

err:
  kmm_free(vmdev);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_register_mmio_device
 *
 * Description:
 *   Register virtio mmio device to the virtio bus
 *
 ****************************************************************************/

int virtio_register_mmio_device(FAR void *regs, int irq)
{
  return virtio_register_mmio_device_(regs, irq, false);
}

/****************************************************************************
 * Name: virtio_register_mmio_device_secure
 *
 * Description:
 *   Register secure virtio mmio device to the virtio bus
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
int virtio_register_mmio_device_secure(FAR void *regs, int irq)
{
  return virtio_register_mmio_device_(regs, irq, true);
}
#endif
