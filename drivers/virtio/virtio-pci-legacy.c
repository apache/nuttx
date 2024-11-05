/****************************************************************************
 * drivers/virtio/virtio-pci-legacy.c
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

#define VIRTIO_PCI_LEGACY_IO_BAR            0

/* A 32-bit r/o bitmask of the features supported by the host */

#define VIRTIO_PCI_HOST_FEATURES            0

/* A 32-bit r/w bitmask of features activated by the guest */

#define VIRTIO_PCI_GUEST_FEATURES           4

/* A 32-bit r/w PFN for the currently selected queue */

#define VIRTIO_PCI_QUEUE_PFN                8

/* A 16-bit r/o queue size for the currently selected queue */

#define VIRTIO_PCI_QUEUE_NUM                12

/* A 16-bit r/w queue selector */

#define VIRTIO_PCI_QUEUE_SEL                14

/* A 16-bit r/w queue notifier */

#define VIRTIO_PCI_QUEUE_NOTIFY             16

/* An 8-bit device status register.  */

#define VIRTIO_PCI_STATUS                   18

/* An 8-bit r/o interrupt status register.  Reading the value will return the
 * current contents of the ISR and will also clear it.  This is effectively
 * a read-and-acknowledge.
 */

#define VIRTIO_PCI_ISR                      19

/* MSI-X registers: only enabled if MSI-X is enabled. */

#define VIRTIO_MSI_CONFIG_VECTOR            20

/* A 16-bit vector for selected queue notifications. */

#define VIRTIO_MSI_QUEUE_VECTOR             22

/* The remaining space is defined by each driver as the per-driver
 * configuration space
 */

#define VIRTIO_PCI_CONFIG_OFF(msix_enabled) ((msix_enabled) ? 24 : 20)

/* Virtio ABI version, this must match exactly */

#define VIRTIO_PCI_ABI_VERSION              0

/* How many bits to shift physical queue address written to QUEUE_PFN.
 * 12 is historical, and due to x86 page size.
 */

#define VIRTIO_PCI_QUEUE_ADDR_SHIFT         12

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

static uint16_t
virtio_pci_legacy_get_queue_len(FAR struct virtio_pci_device_s *vpdev,
                                int idx);
static int
virtio_pci_legacy_create_virtqueue(FAR struct virtio_pci_device_s *vpdev,
                                   FAR struct virtqueue *vq);
static int
virtio_pci_legacy_config_vector(FAR struct virtio_pci_device_s *vpdev,
                                bool enable);

static void
virtio_pci_legacy_delete_virtqueue(FAR struct virtio_device *vdev,
                                   int index);
static void virtio_pci_legacy_set_status(FAR struct virtio_device *vdev,
                                         uint8_t status);
static uint8_t virtio_pci_legacy_get_status(FAR struct virtio_device *vdev);
static void virtio_pci_legacy_write_config(FAR struct virtio_device *vdev,
                                           uint32_t offset, FAR void *dst,
                                           int length);
static void virtio_pci_legacy_read_config(FAR struct virtio_device *vdev,
                                          uint32_t offset, FAR void *dst,
                                          int length);
static uint64_t
virtio_pci_legacy_get_features(FAR struct virtio_device *vdev);
static void virtio_pci_legacy_set_features(FAR struct virtio_device *vdev,
                                           uint64_t features);
static void virtio_pci_legacy_notify(FAR struct virtqueue *vq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct virtio_dispatch g_virtio_pci_dispatch =
{
  virtio_pci_create_virtqueues,          /* create_virtqueues */
  virtio_pci_delete_virtqueues,          /* delete_virtqueues */
  virtio_pci_legacy_get_status,          /* get_status */
  virtio_pci_legacy_set_status,          /* set_status */
  virtio_pci_legacy_get_features,        /* get_features */
  virtio_pci_legacy_set_features,        /* set_features */
  virtio_pci_negotiate_features,         /* negotiate_features */
  virtio_pci_legacy_read_config,         /* read_config */
  virtio_pci_legacy_write_config,        /* write_config */
  virtio_pci_reset_device,               /* reset_device */
  virtio_pci_legacy_notify,              /* notify */
};

static const struct virtio_pci_ops_s g_virtio_pci_legacy_ops =
{
  virtio_pci_legacy_get_queue_len,       /* get_queue_len */
  virtio_pci_legacy_config_vector,       /* config_vector */
  virtio_pci_legacy_create_virtqueue,    /* create_virtqueue */
  virtio_pci_legacy_delete_virtqueue,    /* delete_virtqueue */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_pci_legacy_get_queue_len
 ****************************************************************************/

static uint16_t
virtio_pci_legacy_get_queue_len(FAR struct virtio_pci_device_s *vpdev,
                                int idx)
{
  uint16_t num;

  pci_write_io_word(vpdev->dev,
                    (uintptr_t)(vpdev->ioaddr + VIRTIO_PCI_QUEUE_SEL), idx);
  pci_read_io_word(vpdev->dev,
                   (uintptr_t)(vpdev->ioaddr + VIRTIO_PCI_QUEUE_NUM), &num);
  if (num == 0)
    {
      pcierr("Queue is not available num=%d\n", num);
    }

  return num;
}

/****************************************************************************
 * Name: virtio_pci_legacy_create_virtqueue
 ****************************************************************************/

static int
virtio_pci_legacy_create_virtqueue(FAR struct virtio_pci_device_s *vpdev,
                                   FAR struct virtqueue *vq)
{
#if CONFIG_DRIVERS_VIRTIO_PCI_POLLING_PERIOD <= 0
  uint16_t msix_vector;
#endif

  /* Set the pci virtqueue register, active vq, enable vq */

  pci_write_io_word(vpdev->dev,
                    (uintptr_t)(vpdev->ioaddr + VIRTIO_PCI_QUEUE_SEL),
                    vq->vq_queue_index);

  /* activate the queue */

  pci_write_io_dword(vpdev->dev,
                     (uintptr_t)(vpdev->ioaddr + VIRTIO_PCI_QUEUE_PFN),
                     up_addrenv_va_to_pa(vq->vq_ring.desc) >>
                     VIRTIO_PCI_QUEUE_ADDR_SHIFT);

#if CONFIG_DRIVERS_VIRTIO_PCI_POLLING_PERIOD <= 0
  pci_write_io_word(vpdev->dev,
                    (uintptr_t)(vpdev->ioaddr + VIRTIO_MSI_QUEUE_VECTOR),
                    VIRTIO_PCI_INT_VQ);
  pci_read_io_word(vpdev->dev,
                   (uintptr_t)(vpdev->ioaddr + VIRTIO_MSI_QUEUE_VECTOR),
                   &msix_vector);
  if (msix_vector == VIRTIO_PCI_MSI_NO_VECTOR)
    {
      pci_write_io_dword(vpdev->dev,
                         (uintptr_t)(vpdev->ioaddr + VIRTIO_PCI_QUEUE_PFN),
                         0);
      vrterr("Msix vector is 0\n");
      return -EBUSY;
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: virtio_pci_legacy_config_vector
 ****************************************************************************/

static int
virtio_pci_legacy_config_vector(FAR struct virtio_pci_device_s *vpdev,
                                bool enable)
{
  uint16_t vector = enable ? 0 : VIRTIO_PCI_MSI_NO_VECTOR;
  uint16_t rvector;

  pci_write_io_word(vpdev->dev,
                    (uintptr_t)(vpdev->ioaddr + VIRTIO_MSI_CONFIG_VECTOR),
                    vector);
  pci_read_io_word(vpdev->dev,
                   (uintptr_t)(vpdev->ioaddr + VIRTIO_MSI_CONFIG_VECTOR),
                   &rvector);

  if (rvector != vector)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: virtio_pci_legacy_delete_virtqueue
 ****************************************************************************/

void virtio_pci_legacy_delete_virtqueue(FAR struct virtio_device *vdev,
                                        int index)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
#if CONFIG_DRIVERS_VIRTIO_PCI_POLLING_PERIOD <= 0
  uint8_t isr;
#endif

  pci_write_io_word(vpdev->dev,
                    (uintptr_t)(vpdev->ioaddr + VIRTIO_PCI_QUEUE_SEL),
                    index);

#if CONFIG_DRIVERS_VIRTIO_PCI_POLLING_PERIOD <= 0
  pci_write_io_word(vpdev->dev,
                    (uintptr_t)(vpdev->ioaddr + VIRTIO_MSI_QUEUE_VECTOR),
                    VIRTIO_PCI_MSI_NO_VECTOR);

  /* Flush the write out to device */

  pci_read_io_byte(vpdev->dev,
                   (uintptr_t)(vpdev->ioaddr + VIRTIO_PCI_ISR), &isr);
#endif

  /* Select and deactivate the queue */

  pci_write_io_dword(vpdev->dev,
                     (uintptr_t)(vpdev->ioaddr + VIRTIO_PCI_QUEUE_PFN), 0);
}

/****************************************************************************
 * Name: virtio_pci_legacy_set_status
 ****************************************************************************/

static void virtio_pci_legacy_set_status(FAR struct virtio_device *vdev,
                                         uint8_t status)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;

  pci_write_io_byte(vpdev->dev,
                    (uintptr_t)(vpdev->ioaddr + VIRTIO_PCI_STATUS), status);
}

/****************************************************************************
 * Name: virtio_pci_legacy_get_status
 ****************************************************************************/

static uint8_t virtio_pci_legacy_get_status(FAR struct virtio_device *vdev)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  uint8_t status;

  pci_read_io_byte(vpdev->dev,
                   (uintptr_t)(vpdev->ioaddr + VIRTIO_PCI_STATUS), &status);
  return status;
}

/****************************************************************************
 * Name: virtio_pci_legacy_write_config
 ****************************************************************************/

static void virtio_pci_legacy_write_config(FAR struct virtio_device *vdev,
                                           uint32_t offset, FAR void *src,
                                           int length)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
#if CONFIG_DRIVERS_VIRTIO_PCI_POLLING_PERIOD <= 0
  FAR char *config = vpdev->ioaddr + VIRTIO_PCI_CONFIG_OFF(true) + offset;
#else
  FAR char *config = vpdev->ioaddr + VIRTIO_PCI_CONFIG_OFF(false) + offset;
#endif
  FAR uint8_t *s = src;
  int i;

  for (i = 0; i < length; i++)
    {
      pci_write_io_byte(vpdev->dev, (uintptr_t)(config + i), s[i]);
    }
}

/****************************************************************************
 * Name: virtio_pci_legacy_read_config
 ****************************************************************************/

static void virtio_pci_legacy_read_config(FAR struct virtio_device *vdev,
                                          uint32_t offset, FAR void *dst,
                                          int length)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
#if CONFIG_DRIVERS_VIRTIO_PCI_POLLING_PERIOD <= 0
  FAR char *config = vpdev->ioaddr + VIRTIO_PCI_CONFIG_OFF(true) + offset;
#else
  FAR char *config = vpdev->ioaddr + VIRTIO_PCI_CONFIG_OFF(false) + offset;
#endif
  FAR uint8_t *d = dst;
  int i;

  for (i = 0; i < length; i++)
    {
      pci_read_io_byte(vpdev->dev, (uintptr_t)(config + i), &d[i]);
    }
}

/****************************************************************************
 * Name: virtio_pci_legacy_get_features
 ****************************************************************************/

static uint64_t
virtio_pci_legacy_get_features(FAR struct virtio_device *vdev)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  uint32_t feature;

  pci_read_io_dword(vpdev->dev,
                    (uintptr_t)(vpdev->ioaddr + VIRTIO_PCI_HOST_FEATURES),
                    &feature);
  return feature;
}

/****************************************************************************
 * Name: virtio_pci_legacy_set_features
 ****************************************************************************/

static void virtio_pci_legacy_set_features(FAR struct virtio_device *vdev,
                                           uint64_t features)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;

  if ((features >> 32) != 0)
    {
      features = (uint64_t)((uint32_t)features);
      vrtwarn("Virtio pci legacy not support feature bits larger than 32\n");
    }

  pci_write_io_dword(vpdev->dev,
                     (uintptr_t)(vpdev->ioaddr + VIRTIO_PCI_GUEST_FEATURES),
                     features);
  vdev->features = features;
}

/****************************************************************************
 * Name: virtio_pci_legacy_notify
 ****************************************************************************/

static void virtio_pci_legacy_notify(FAR struct virtqueue *vq)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vq->vq_dev;

  pci_write_io_word(vpdev->dev,
                    (uintptr_t)(vpdev->ioaddr + VIRTIO_PCI_QUEUE_NOTIFY),
                    vq->vq_queue_index);
}

/****************************************************************************
 * Name: virtio_pci_legacy_init_device
 ****************************************************************************/

static int
virtio_pci_legacy_init_device(FAR struct virtio_pci_device_s *vpdev)
{
  FAR struct virtio_device *vdev = &vpdev->vdev;
  FAR struct pci_device_s *dev = vpdev->dev;

  if (dev->revision != VIRTIO_PCI_ABI_VERSION)
    {
      pcierr("Virtio_pci: expected ABI version %d, got %u\n",
              VIRTIO_PCI_ABI_VERSION, dev->revision);
      return -ENODEV;
    }

  vpdev->ioaddr = pci_map_bar(dev, VIRTIO_PCI_LEGACY_IO_BAR);
  if (vpdev->ioaddr == NULL)
    {
      vrterr("Unable to map virtio on bar\n");
      return -EINVAL;
    }

  vdev->id.vendor = dev->subsystem_vendor;
  vdev->id.device = dev->subsystem_device;

  return OK;
}

/****************************************************************************
 * Name: virtio_pci_legacy_probe
 ****************************************************************************/

int virtio_pci_legacy_probe(FAR struct pci_device_s *dev)
{
  FAR struct virtio_pci_device_s *vpdev = dev->priv;
  FAR struct virtio_device *vdev = &vpdev->vdev;
  int ret;

  /* We only own devices >= 0x1000 and <= 0x107f: leave the rest. */

  if (dev->device < 0x1000 || dev->device > 0x103f)
    {
      return -ENODEV;
    }

  vpdev->ops = &g_virtio_pci_legacy_ops;
  vdev->func = &g_virtio_pci_dispatch;
  vdev->role = VIRTIO_DEV_DRIVER;

  ret = virtio_pci_legacy_init_device(vpdev);
  if (ret < 0)
    {
      pcierr("Virtio pci legacy device init failed, ret=%d\n", ret);
    }

  return ret;
}
