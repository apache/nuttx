/****************************************************************************
 * include/nuttx/ivshmem.h
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

#ifndef __INCLUDE_NUTTX_IVSHMEM_H
#define __INCLUDE_NUTTX_IVSHMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_IVSHMEM

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/queue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register layout from QEMU docs/specs/ivshmem-spec.rst */

#define IVSHMEM_REG_INT_MASK    0
#define IVSHMEM_REG_INT_STATUS  4
#define IVSHMEM_REG_IVPOSITION  8
#define IVSHMEM_REG_DOORBELL    12

#define IVSHMEM_INVALID_VMID    UINT32_MAX

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ivshmem_device_s;

/* Notifies the peer, or NULL for a device that cannot */

typedef CODE int (*ivshmem_kick_peer_t)
                 (FAR struct ivshmem_device_s *dev, uint32_t peer_id);

struct ivshmem_driver_s
{
  int              id;
  CODE int         (*probe)(FAR struct ivshmem_device_s *dev);
  CODE void        (*remove)(FAR struct ivshmem_device_s *dev);
  dq_entry_t       node;
};

/* Filled in by a backend and embedded in its own device structure. A
 * negative id asks ivshmem_register_device() to assign the next one.
 */

struct ivshmem_device_s
{
  ivshmem_kick_peer_t             kick_peer;

  FAR struct ivshmem_driver_s    *drv;
  dq_entry_t                      node;

  int                             id;

  FAR void                       *shmem;
  size_t                          shmem_size;

  FAR void                       *reg;
  int                             irq;
  uint32_t                        vmid;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
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
 * Name: ivshmem_register_device
 *
 * Description:
 *   Called by a backend to add a device to the ivshmem bus. Assigns the
 *   device id if it is negative and probes the matching driver, if one
 *   is registered. kick_peer notifies the peer, or is NULL when the
 *   device has none.
 *
 ****************************************************************************/

int ivshmem_register_device(FAR struct ivshmem_device_s *dev,
                            ivshmem_kick_peer_t kick_peer);

/****************************************************************************
 * Name: ivshmem_unregister_device
 *
 * Description:
 *   Called by a backend to remove a device from the ivshmem bus.
 *
 ****************************************************************************/

int ivshmem_unregister_device(FAR struct ivshmem_device_s *dev);

/****************************************************************************
 * Name: ivshmem_register_drivers
 *
 * Description:
 *   Register the enabled ivshmem drivers (uio, ramdisk, rptun) on
 *   the ivshmem bus. Called once from drivers_initialize().
 *
 ****************************************************************************/

void ivshmem_register_drivers(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_IVSHMEM */
#endif /* __INCLUDE_NUTTX_IVSHMEM_H */
