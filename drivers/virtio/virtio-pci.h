/****************************************************************************
 * drivers/virtio/virtio-pci.h
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

#include <nuttx/pci/pci.h>
#include <nuttx/virtio/virtio.h>

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

struct virtio_pci_device_s
{
  struct virtio_device               vdev;    /* Virtio deivce */
  FAR struct pci_device_s           *dev;     /* PCI device */
  metal_phys_addr_t                  shm_phy;
  struct metal_io_region             shm_io;  /* Share memory io region,
                                               * virtqueue use this io.
                                               */
  int                                irq[VIRTIO_PCI_INT_NUM];

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

int virtio_pci_legacy_probe(FAR struct pci_device_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_VIRTIO_PCI */

#endif /* __DRIVERS_VIRTIO_VIRTIO_PCI_H */
