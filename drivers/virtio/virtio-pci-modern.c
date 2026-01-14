/****************************************************************************
 * drivers/virtio/virtio-pci-modern.c
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

#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <sys/param.h>

#include "virtio-pci.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Common configuration */

#define VIRTIO_PCI_CAP_COMMON_CFG        1

/* Notifications */

#define VIRTIO_PCI_CAP_NOTIFY_CFG        2

/* ISR Status */

#define VIRTIO_PCI_CAP_ISR_CFG           3

/* Device specific configuration */

#define VIRTIO_PCI_CAP_DEVICE_CFG        4

/* PCI configuration access */

#define VIRTIO_PCI_CAP_PCI_CFG           5

/* Shared memory region */

#define VIRTIO_PCI_CAP_SHARED_MEMORY_CFG 8

/* Vendor-specific data */

#define VIRTIO_PCI_CAP_VENDOR_CFG        9

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct virtio_pci_cap_s
{
  uint8_t  cap_vndr;   /* Generic PCI field: PCI_CAP_ID_VNDR */
  uint8_t  cap_next;   /* Generic PCI field: next ptr. */
  uint8_t  cap_len;    /* Generic PCI field: capability length */
  uint8_t  cfg_type;   /* Identifies the structure. */
  uint8_t  bar;        /* Where to find it. */
  uint8_t  id;         /* Multiple capabilities of the same type */
  uint8_t  padding[2]; /* Pad to full dword. */
  uint32_t offset;     /* Offset within bar. */
  uint32_t length;     /* Length of the structure, in bytes. */
} end_packed_struct;

begin_packed_struct struct virtio_pci_cap64_s
{
  struct virtio_pci_cap_s cap;
  uint32_t                offset_hi;
  uint32_t                length_hi;
} end_packed_struct;

/* VIRTIO_PCI_CAP_COMMON_CFG */

begin_packed_struct struct virtio_pci_common_cfg_s
{
  /* About the whole device. */

  uint32_t device_feature_select; /* read-write */
  uint32_t device_feature;        /* read-only for driver */
  uint32_t driver_feature_select; /* read-write */
  uint32_t driver_feature;        /* read-write */
  uint16_t config_msix_vector;    /* read-write */
  uint16_t num_queues;            /* read-only for driver */
  uint8_t  device_status;         /* read-write */
  uint8_t  config_generation;     /* read-only for driver */

  /* About a specific virtqueue. */

  uint16_t queue_select;          /* read-write */
  uint16_t queue_size;            /* read-write */
  uint16_t queue_msix_vector;     /* read-write */
  uint16_t queue_enable;          /* read-write */
  uint16_t queue_notify_off;      /* read-only for driver */
  uint64_t queue_desc;            /* read-write */
  uint64_t queue_avail;           /* read-write */
  uint64_t queue_used;            /* read-write */
  uint16_t queue_notify_data;     /* read-only for driver */
  uint16_t queue_reset;           /* read-write */
} end_packed_struct;

/* VIRTIO_PCI_CAP_NOTIFY_CFG */

begin_packed_struct struct virtio_pci_notify_cap_s
{
  struct virtio_pci_cap_s cap;

  /* Multiplier for queue_notify_off. */

  uint32_t                notify_off_multiplier;
} end_packed_struct;

/* VIRTIO_PCI_CAP_VENDOR_CFG */

begin_packed_struct struct virtio_pci_vndr_data_s
{
  uint8_t  cap_vndr;   /* Generic PCI field: PCI_CAP_ID_VNDR */
  uint8_t  cap_next;   /* Generic PCI field: next ptr. */
  uint8_t  cao_len;    /* Generic PCI field: capability length */
  uint8_t  cfg_type;   /* Identifies the structure. */
  uint16_t vendor_id;  /* Identifies the vendor-specific format. */

  /* For Vendor Definition
   * Pads structure to a multiple of 4 bytes
   * Reads must not have side effects
   */
} end_packed_struct;

/* VIRTIO_PCI_CAP_PCI_CFG */

begin_packed_struct struct virtio_pci_cfg_cap_s
{
  struct virtio_pci_cap_s cap;
  uint8_t                 pci_cfg_data[4]; /* Data for BAR access. */
} end_packed_struct;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

static uint16_t
virtio_pci_modern_get_queue_len(FAR struct virtio_pci_device_s *vpdev,
                                int idx);
static int
virtio_pci_modern_config_vector(FAR struct virtio_pci_device_s *vpdev,
                                bool enable);
static int
virtio_pci_modern_create_virtqueue(FAR struct virtio_pci_device_s *vpdev,
                                   FAR struct virtqueue *vq);
static void
virtio_pci_modern_delete_virtqueue(FAR struct virtio_device *vdev,
                                   int index);
static void virtio_pci_modern_set_status(FAR struct virtio_device *vdev,
                                         uint8_t status);
static uint8_t virtio_pci_modern_get_status(FAR struct virtio_device *vdev);
static void virtio_pci_modern_write_config(FAR struct virtio_device *vdev,
                                           uint32_t offset, FAR void *dst,
                                           int length);
static void virtio_pci_modern_read_config(FAR struct virtio_device *vdev,
                                          uint32_t offset, FAR void *dst,
                                          int length);
static uint64_t
virtio_pci_modern_get_features(FAR struct virtio_device *vdev);
static void virtio_pci_modern_set_features(FAR struct virtio_device *vdev,
                                           uint64_t features);
static void virtio_pci_modern_notify(FAR struct virtqueue *vq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct virtio_dispatch g_virtio_pci_dispatch =
{
  virtio_pci_create_virtqueues,          /* create_virtqueues */
  virtio_pci_delete_virtqueues,          /* delete_virtqueues */
  virtio_pci_modern_get_status,          /* get_status */
  virtio_pci_modern_set_status,          /* set_status */
  virtio_pci_modern_get_features,        /* get_features */
  virtio_pci_modern_set_features,        /* set_features */
  virtio_pci_negotiate_features,         /* negotiate_features */
  virtio_pci_modern_read_config,         /* read_config */
  virtio_pci_modern_write_config,        /* write_config */
  virtio_pci_reset_device,               /* reset_device */
  virtio_pci_modern_notify,              /* notify */
};

static const struct virtio_pci_ops_s g_virtio_pci_modern_ops =
{
  virtio_pci_modern_get_queue_len,      /* get_queue_len */
  virtio_pci_modern_config_vector,      /* config_vector */
  virtio_pci_modern_create_virtqueue,   /* create_virtqueue */
  virtio_pci_modern_delete_virtqueue,   /* delete_virtqueue */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_pci_find_capability
 ****************************************************************************/

static uint8_t virtio_pci_find_capability(FAR struct pci_device_s *dev,
                                          uint8_t cfg_type,
                                          unsigned int flags)
{
  uint8_t type;
  uint8_t bar;
  uint8_t pos;

  for (pos = pci_find_capability(dev, PCI_CAP_ID_VNDR); pos > 0;
       pos = pci_find_next_capability(dev, pos, PCI_CAP_ID_VNDR))
    {
      pci_read_config_byte(dev,
                           pos + offsetof(struct virtio_pci_cap_s, cfg_type),
                           &type);
      pci_read_config_byte(dev,
                           pos + offsetof(struct virtio_pci_cap_s, bar),
                           &bar);

      /* VirtIO Spec v1.2: The driver MUST ignore any vendor-specific
       * capability structure which has a reserved cfg_type value.
       */

      if (bar >= PCI_STD_NUM_BARS)
        {
          continue;
        }

      if (type == cfg_type && pci_resource_len(dev, bar) &&
          (pci_resource_flags(dev, bar) & flags))
        {
          return pos;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: virtio_pci_map_capability
 ****************************************************************************/

static FAR void *
virtio_pci_map_capability(FAR struct virtio_pci_device_s *vpdev, uint8_t pos,
                          size_t *len)
{
  FAR struct pci_device_s *dev = vpdev->dev;
  FAR void *ptr;
  uint32_t offset;
  uint32_t length;
  uint8_t bar;

  pci_read_config_byte(dev, pos + offsetof(struct virtio_pci_cap_s, bar),
                       &bar);
  pci_read_config_dword(dev, pos + offsetof(struct virtio_pci_cap_s, offset),
                        &offset);
  pci_read_config_dword(dev, pos + offsetof(struct virtio_pci_cap_s, length),
                        &length);

  if (bar >= PCI_STD_NUM_BARS)
    {
      vrterr("Bar error bard=%u\n", bar);
      return NULL;
    }

  ptr = pci_map_bar_region(dev, bar, offset, length);
  if (ptr == NULL)
    {
      vrterr("Unable to map virtio on bar %u\n", bar);
      return NULL;
    }

  if (len)
    {
      *len = length;
    }

  return ptr;
}

/****************************************************************************
 * Name: virtio_pci_modern_get_queue_len
 ****************************************************************************/

static uint16_t
virtio_pci_modern_get_queue_len(FAR struct virtio_pci_device_s *vpdev,
                                int idx)
{
  FAR struct virtio_pci_common_cfg_s *cfg = vpdev->common;
  uint16_t size;

  pci_write_io_word(vpdev->dev, (uintptr_t)&cfg->queue_select, idx);
  pci_read_io_word(vpdev->dev, (uintptr_t)&cfg->queue_size, &size);
  return size;
}

/****************************************************************************
 * Name: virtio_pci_modern_create_virtqueue
 ****************************************************************************/

static int
virtio_pci_modern_create_virtqueue(FAR struct virtio_pci_device_s *vpdev,
                                   FAR struct virtqueue *vq)
{
  FAR struct virtio_pci_common_cfg_s *cfg = vpdev->common;
  FAR struct virtio_vring_info *vrinfo;
#if CONFIG_DRIVERS_VIRTIO_PCI_POLLING_PERIOD <= 0
  uint16_t msix_vector;
#endif
  uint16_t off;

  /* Activate the queue */

  pci_write_io_word(vpdev->dev, (uintptr_t)&cfg->queue_select,
                    vq->vq_queue_index);
  pci_write_io_word(vpdev->dev, (uintptr_t)&cfg->queue_size,
                    vq->vq_ring.num);

  pci_write_io_qword(vpdev->dev, (uintptr_t)&cfg->queue_desc,
                     up_addrenv_va_to_pa(vq->vq_ring.desc));
  pci_write_io_qword(vpdev->dev, (uintptr_t)&cfg->queue_avail,
                     up_addrenv_va_to_pa(vq->vq_ring.avail));
  pci_write_io_qword(vpdev->dev, (uintptr_t)&cfg->queue_used,
                     up_addrenv_va_to_pa(vq->vq_ring.used));

#if CONFIG_DRIVERS_VIRTIO_PCI_POLLING_PERIOD <= 0
  pci_write_io_word(vpdev->dev, (uintptr_t)&cfg->queue_msix_vector,
                    VIRTIO_PCI_INT_VQ);
  pci_read_io_word(vpdev->dev, (uintptr_t)&cfg->queue_msix_vector,
                   &msix_vector);
  if (msix_vector == VIRTIO_PCI_MSI_NO_VECTOR)
    {
      vrterr("Msix_vector is no vector\n");
      return -EBUSY;
    }
#endif

  /* Enable vq */

  pci_write_io_word(vpdev->dev, (uintptr_t)&cfg->queue_enable, 1);

  pci_write_io_word(vpdev->dev, (uintptr_t)&cfg->queue_select,
                    vq->vq_queue_index);
  pci_read_io_word(vpdev->dev, (uintptr_t)&cfg->queue_notify_off, &off);

  if (off * vpdev->notify_off_multiplier + 2 > vpdev->notify_len)
    {
      return -EINVAL;
    }

  vrinfo = &vpdev->vdev.vrings_info[vq->vq_queue_index];
  vrinfo->notifyid = off * vpdev->notify_off_multiplier;

  return OK;
}

/****************************************************************************
 * Name: virtio_pci_modern_config_vector
 ****************************************************************************/

static int
virtio_pci_modern_config_vector(FAR struct virtio_pci_device_s *vpdev,
                                bool enable)
{
  FAR struct virtio_pci_common_cfg_s *cfg = vpdev->common;
  uint16_t vector = enable ? 0 : VIRTIO_PCI_MSI_NO_VECTOR;
  uint16_t rvector;

  pci_write_io_word(vpdev->dev, (uintptr_t)&cfg->config_msix_vector,
                    vector);
  pci_read_io_word(vpdev->dev, (uintptr_t)&cfg->config_msix_vector,
                   &rvector);
  if (rvector != vector)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: virtio_pci_modern_delete_virtqueue
 ****************************************************************************/

void virtio_pci_modern_delete_virtqueue(FAR struct virtio_device *vdev,
                                        int index)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  FAR struct virtio_pci_common_cfg_s *cfg = vpdev->common;

  pci_write_io_word(vpdev->dev, (uintptr_t)&cfg->queue_select, index);

#if CONFIG_DRIVERS_VIRTIO_PCI_POLLING_PERIOD <= 0
  pci_write_io_word(vpdev->dev, (uintptr_t)&cfg->queue_msix_vector,
                    VIRTIO_PCI_MSI_NO_VECTOR);
#endif
}

/****************************************************************************
 * Name: virtio_pci_modern_set_status
 ****************************************************************************/

static void virtio_pci_modern_set_status(FAR struct virtio_device *vdev,
                                         uint8_t status)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  FAR struct virtio_pci_common_cfg_s *cfg = vpdev->common;

  pci_write_io_byte(vpdev->dev, (uintptr_t)&cfg->device_status, status);
}

/****************************************************************************
 * Name: virtio_pci_modern_get_status
 ****************************************************************************/

static uint8_t virtio_pci_modern_get_status(FAR struct virtio_device *vdev)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  FAR struct virtio_pci_common_cfg_s *cfg = vpdev->common;
  uint8_t status;

  pci_read_io_byte(vpdev->dev, (uintptr_t)&cfg->device_status, &status);
  return status;
}

/****************************************************************************
 * Name: virtio_pci_modern_write_config
 ****************************************************************************/

static void virtio_pci_modern_write_config(FAR struct virtio_device *vdev,
                                           uint32_t offset, FAR void *src,
                                           int length)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  uint64_t u64data;
  uint32_t u32data;
  uint16_t u16data;
  uint8_t u8data;

  if (offset + length > vpdev->device_len)
    {
      return;
    }

  switch (length)
    {
      case 1:
        memcpy(&u8data, src, sizeof(u8data));
        pci_write_io_byte(vpdev->dev, (uintptr_t)(vpdev->device + offset),
                          u8data);
        break;
      case 2:
        memcpy(&u16data, src, sizeof(u16data));
        pci_write_io_word(vpdev->dev, (uintptr_t)(vpdev->device + offset),
                          u16data);
        break;
      case 4:
        memcpy(&u32data, src, sizeof(u32data));
        pci_write_io_dword(vpdev->dev, (uintptr_t)(vpdev->device + offset),
                           u32data);
        break;
      case 8:
        memcpy(&u64data, src, sizeof(u64data));
        pci_write_io_qword(vpdev->dev, (uintptr_t)(vpdev->device + offset),
                           u64data);
        break;
      default:
        {
          FAR char *s = src;
          int i;
          for (i = 0; i < length; i++)
            {
              pci_write_io_byte(vpdev->dev,
                                (uintptr_t)(vpdev->device + offset + i),
                                s[i]);
            }
        }
    }
}

/****************************************************************************
 * Name: virtio_pci_modern_read_config
 ****************************************************************************/

static void virtio_pci_modern_read_config(FAR struct virtio_device *vdev,
                                          uint32_t offset, FAR void *dst,
                                          int length)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  uint64_t u64data;
  uint32_t u32data;
  uint16_t u16data;
  uint8_t u8data;

  if (offset + length > vpdev->device_len)
    {
      return;
    }

  switch (length)
    {
      case 1:
        pci_read_io_byte(vpdev->dev, (uintptr_t)(vpdev->device + offset),
                         &u8data);
        memcpy(dst, &u8data, sizeof(u8data));
        break;
      case 2:
        pci_read_io_word(vpdev->dev, (uintptr_t)(vpdev->device + offset),
                         &u16data);
        memcpy(dst, &u16data, sizeof(u16data));
        break;
      case 4:
        pci_read_io_dword(vpdev->dev, (uintptr_t)(vpdev->device + offset),
                          &u32data);
        memcpy(dst, &u32data, sizeof(u32data));
        break;
      case 8:
        pci_read_io_qword(vpdev->dev, (uintptr_t)(vpdev->device + offset),
                          &u64data);
        memcpy(dst, &u64data, sizeof(u64data));
        break;
      default:
        {
          FAR char *d = dst;
          int i;
          for (i = 0; i < length; i++)
            {
              pci_read_io_byte(vpdev->dev,
                               (uintptr_t)(vpdev->device + offset + i),
                               (uint8_t *)&d[i]);
            }
        }
    }
}

/****************************************************************************
 * Name: virtio_pci_modern_get_features
 ****************************************************************************/

static uint64_t
virtio_pci_modern_get_features(FAR struct virtio_device *vdev)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  FAR struct virtio_pci_common_cfg_s *cfg = vpdev->common;
  uint32_t feature_lo;
  uint32_t feature_hi;

  pci_write_io_dword(vpdev->dev, (uintptr_t)&cfg->device_feature_select, 0);
  pci_read_io_dword(vpdev->dev, (uintptr_t)&cfg->device_feature,
                    &feature_lo);
  pci_write_io_dword(vpdev->dev, (uintptr_t)&cfg->device_feature_select, 1);
  pci_read_io_dword(vpdev->dev, (uintptr_t)&cfg->device_feature,
                    &feature_hi);
  return ((uint64_t)feature_hi << 32) | (uint64_t)feature_lo;
}

/****************************************************************************
 * Name: virtio_pci_modern_set_features
 ****************************************************************************/

static void virtio_pci_modern_set_features(FAR struct virtio_device *vdev,
                                           uint64_t features)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  FAR struct virtio_pci_common_cfg_s *cfg = vpdev->common;

  pci_write_io_dword(vpdev->dev, (uintptr_t)&cfg->driver_feature_select, 0);
  pci_write_io_dword(vpdev->dev, (uintptr_t)&cfg->driver_feature, features);
  pci_write_io_dword(vpdev->dev, (uintptr_t)&cfg->driver_feature_select, 1);
  pci_write_io_dword(vpdev->dev, (uintptr_t)&cfg->driver_feature,
                     features >> 32);
  vdev->features = features;
}

/****************************************************************************
 * Name: virtio_pci_modern_notify
 ****************************************************************************/

static void virtio_pci_modern_notify(FAR struct virtqueue *vq)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vq->vq_dev;
  FAR struct virtio_vring_info *vrinfo;

  vrinfo = &vpdev->vdev.vrings_info[vq->vq_queue_index];

  pci_write_io_word(vpdev->dev,
                    (uintptr_t)(vpdev->notify + vrinfo->notifyid),
                    vq->vq_queue_index);
}

/****************************************************************************
 * Name: virtio_pci_probe
 ****************************************************************************/

static int virtio_pci_init_device(FAR struct virtio_pci_device_s *vpdev)
{
  FAR struct virtio_device *vdev = &vpdev->vdev;
  FAR struct pci_device_s *dev = vpdev->dev;
  uint8_t common;
  uint8_t notify;
  uint8_t device;

  if (dev->device < 0x1040)
    {
      /* Transitional devices: use the PCI subsystem device id as
       * virtio device id, same as legacy driver always did.
       */

      vdev->id.device = dev->subsystem_device;
    }
  else
    {
      /* Modern devices: simply use PCI device id, but start from 0x1040. */

      vdev->id.device = dev->device - 0x1040;
    }

  vdev->id.vendor = dev->subsystem_vendor;

  /* Find pci capabilities */

  common = virtio_pci_find_capability(dev, VIRTIO_PCI_CAP_COMMON_CFG,
                                      PCI_RESOURCE_MEM);
  if (common == 0)
    {
      vrtinfo("Leaving for legacy driver\n");
      return -ENODEV;
    }

  notify = virtio_pci_find_capability(dev, VIRTIO_PCI_CAP_NOTIFY_CFG,
                                      PCI_RESOURCE_MEM);
  if (notify == 0)
    {
      vrterr("Missing capabilities %d %d\n", common, notify);
      return -EINVAL;
    }

  device = virtio_pci_find_capability(dev, VIRTIO_PCI_CAP_DEVICE_CFG,
                                      PCI_RESOURCE_MEM);

  /* Map the BAR */

  vpdev->common = virtio_pci_map_capability(vpdev, common,
                                            &vpdev->common_len);
  if (vpdev->common == NULL)
    {
      return -EINVAL;
    }

  pci_read_config_dword(dev, notify +
                        offsetof(struct virtio_pci_notify_cap_s,
                                 notify_off_multiplier),
                        &vpdev->notify_off_multiplier);

  vpdev->notify = virtio_pci_map_capability(vpdev, notify,
                                            &vpdev->notify_len);
  if (vpdev->notify == NULL)
    {
      return -EINVAL;
    }

  if (device != 0)
    {
      vpdev->device = virtio_pci_map_capability(vpdev, device,
                                                &vpdev->device_len);
      if (vpdev->device == NULL)
        {
          return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: virtio_pci_modern_probe
 ****************************************************************************/

int virtio_pci_modern_probe(FAR struct pci_device_s *dev)
{
  FAR struct virtio_pci_device_s *vpdev = dev->priv;
  FAR struct virtio_device *vdev = &vpdev->vdev;
  int ret;

  /* We only own devices >= 0x1000 and <= 0x107f: leave the rest. */

  if (dev->device < 0x1000 || dev->device > 0x107f)
    {
      return -ENODEV;
    }

  vpdev->ops = &g_virtio_pci_modern_ops;
  vdev->func = &g_virtio_pci_dispatch;
  vdev->role = VIRTIO_DEV_DRIVER;

  ret = virtio_pci_init_device(vpdev);
  if (ret < 0)
    {
      if (ret != -ENODEV)
        {
          pcierr("Virtio pci modern device init failed, ret=%d\n", ret);
        }
    }

  return ret;
}
