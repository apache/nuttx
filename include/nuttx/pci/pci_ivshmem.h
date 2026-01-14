/****************************************************************************
 * include/nuttx/pci/pci_ivshmem.h
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

#ifndef __INCLUDE_NUTTX_PCI_PCI_IVSHMEM_H
#define __INCLUDE_NUTTX_PCI_PCI_IVSHMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_PCI_IVSHMEM

#include <nuttx/irq.h>
#include <nuttx/list.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ivshmem_device_s;
struct ivshmem_driver_s
{
  int              id;
  CODE int         (*probe)(FAR struct ivshmem_device_s *dev);
  CODE void        (*remove)(FAR struct ivshmem_device_s *dev);
  struct list_node node;
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ivshmem_get_driver
 *
 * Description:
 *   Get the ivshmem device corresponding driver.
 *
 ****************************************************************************/

FAR struct ivshmem_driver_s *
ivshmem_get_driver(FAR struct ivshmem_device_s *dev);

/****************************************************************************
 * Name: ivshmem_get_shmem
 *
 * Description:
 *   Get the ivshmem device share memory address and size.
 *
 ****************************************************************************/

FAR void *ivshmem_get_shmem(FAR struct ivshmem_device_s *dev,
                            FAR size_t *size);

/****************************************************************************
 * Name: ivshmem_attach_irq
 *
 * Description:
 *   Attach the interrupt handler to the ivshmem interrupt
 *
 ****************************************************************************/

int ivshmem_attach_irq(FAR struct ivshmem_device_s *dev, xcpt_t isr,
                       FAR void *arg);

/****************************************************************************
 * Name: ivshmem_detach_irq
 *
 * Description:
 *   Detach the interrupt handler to the ivshmem interrupt
 *
 ****************************************************************************/

int ivshmem_detach_irq(FAR struct ivshmem_device_s *dev);

/****************************************************************************
 * Name: ivshmem_control_irq
 *
 * Description:
 *   Enable/Disable the ivshmem interrupt
 *
 ****************************************************************************/

int ivshmem_control_irq(FAR struct ivshmem_device_s *dev, bool on);

/****************************************************************************
 * Name: ivshmem_support_irq
 *
 * Description:
 *   judge if support ivshmem interrupt
 *
 ****************************************************************************/

bool ivshmem_support_irq(FAR struct ivshmem_device_s *dev);

/****************************************************************************
 * Name: ivshmem_kick_peer
 *
 * Description:
 *   Send interrupt to peer
 *
 ****************************************************************************/

int ivshmem_kick_peer(FAR struct ivshmem_device_s *dev);

/****************************************************************************
 * Name: ivshmem_register_driver
 *
 * Description:
 *   Register ivshmem driver to ivshmem bus
 *
 ****************************************************************************/

int ivshmem_register_driver(FAR struct ivshmem_driver_s *drv);

/****************************************************************************
 * Name: ivshmem_unregister_driver
 *
 * Description:
 *   Un-register ivshmem driver from ivshmem bus
 *
 ****************************************************************************/

int ivshmem_unregister_driver(FAR struct ivshmem_driver_s *drv);

/****************************************************************************
 * Name: pci_ivshemem_register
 *
 * Description:
 *   Register the pci ivshmem driver to the pci bus.
 *
 ****************************************************************************/

int pci_ivshemem_register(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_PCI_IVSHMEM */
#endif /* __INCLUDE_NUTTX_PCI_PCI_IVSHMEM_H */
