/****************************************************************************
 * drivers/virtio/virtio-pci.h
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

#ifndef __DRIVERS_VIRTIO_VIRTIO_PCI_H
#define __DRIVERS_VIRTIO_VIRTIO_PCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_DRIVERS_VIRTIO_PCI

#include <nuttx/arch.h>
#include <nuttx/pci/pci.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/wdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_PCI_VRING_ALIGN           (1 << 12)

#define VIRTIO_PCI_MSI_NO_VECTOR         0xffff

#define VIRTIO_PCI_INT_CFG               0
#define VIRTIO_PCI_INT_VQ                1
#define VIRTIO_PCI_INT_NUM               2

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct virtio_pci_device_s;
struct virtio_pci_ops_s
{
  CODE uint16_t (*get_queue_len)(FAR struct virtio_pci_device_s *vpdev,
                                 int idx);
  CODE int (*config_vector)(FAR struct virtio_pci_device_s *vpdev,
                            bool enable);
  CODE int (*create_virtqueue)(FAR struct virtio_pci_device_s *vpdev,
                               FAR struct virtqueue *vq);
  CODE void (*delete_virtqueue)(FAR struct virtio_device *vdev, int index);
};

struct virtio_pci_device_s
{
  struct virtio_device               vdev;    /* Virtio device */
  FAR struct pci_device_s           *dev;     /* PCI device */
  FAR const struct virtio_pci_ops_s *ops;
  metal_phys_addr_t                  shm_phy;
  struct metal_io_region             shm_io;  /* Share memory io region,
                                               * virtqueue use this io.
                                               */
#if CONFIG_DRIVERS_VIRTIO_PCI_POLLING_PERIOD <= 0
  int                                irq[VIRTIO_PCI_INT_NUM];
#else
  struct wdog_s                      wdog;
#endif

  /* for modern */

  FAR void                          *common;
  size_t                             common_len;
  FAR void                          *notify;
  size_t                             notify_len;
  uint32_t                           notify_off_multiplier;
  FAR void                          *device;
  size_t                             device_len;

  /* for legacy */

  FAR void                          *ioaddr;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void virtio_pci_reset_device(FAR struct virtio_device *vdev);
uint64_t
virtio_pci_negotiate_features(FAR struct virtio_device *vdev,
                              uint64_t features);
int virtio_pci_create_virtqueues(FAR struct virtio_device *vdev,
                                 unsigned int flags,
                                 unsigned int nvqs,
                                 FAR const char *names[],
                                 vq_callback callbacks[],
                                 FAR void *callback_args[]);
void virtio_pci_delete_virtqueues(FAR struct virtio_device *vdev);

int virtio_pci_modern_probe(FAR struct pci_device_s *dev);
int virtio_pci_legacy_probe(FAR struct pci_device_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_VIRTIO_PCI */

#endif /* __DRIVERS_VIRTIO_VIRTIO_PCI_H */
