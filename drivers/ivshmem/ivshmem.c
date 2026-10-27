/****************************************************************************
 * drivers/ivshmem/ivshmem.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/percpu.h>
#include <nuttx/ivshmem.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ivshmem_bus_s
{
  mutex_t          lock; /* Lock for the queue */
  dq_queue_t       dev;  /* Wait match ivshmem device queue */
  dq_queue_t       drv;  /* Ivshmem driver queue */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static DEFINE_PER_CPU_BSS_BMP(int, g_ivshmem_next_id);
#define g_ivshmem_next_id this_cpu_var_bmp(g_ivshmem_next_id)

static DEFINE_PER_CPU_BMP(struct ivshmem_bus_s, g_ivshmem_bus) =
{
  NXMUTEX_INITIALIZER,
};
#define g_ivshmem_bus this_cpu_var_bmp(g_ivshmem_bus)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ivshmem_register_device
 ****************************************************************************/

int ivshmem_register_device(FAR struct ivshmem_device_s *dev,
                            ivshmem_kick_peer_t kick_peer)
{
  FAR struct ivshmem_driver_s *drv;
  int ret;

  DEBUGASSERT(dev != NULL);

  ret = nxmutex_lock(&g_ivshmem_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  dev->kick_peer = kick_peer;

  if (dev->id < 0)
    {
      dev->id = g_ivshmem_next_id++;
    }

  /* Add the device to the ivshmem_bus device queue */

  dq_addlast(&dev->node, &g_ivshmem_bus.dev);

  /* Match the driver has registered in the ivshmem_bus */

  dq_for_every_entry(&g_ivshmem_bus.drv, drv, struct ivshmem_driver_s, node)
    {
      if (drv->id == dev->id)
        {
          /* If found the driver in the driver queue, call driver probe,
           * if probe success, assign item->driver to indicate the device
           * matched.
           */

          dev->drv = drv;
          if (drv->probe(dev) < 0)
            {
              dev->drv = NULL;
            }

          break;
        }
    }

  nxmutex_unlock(&g_ivshmem_bus.lock);
  return ret;
}

/****************************************************************************
 * Name: ivshmem_unregister_device
 ****************************************************************************/

int ivshmem_unregister_device(FAR struct ivshmem_device_s *dev)
{
  int ret;

  ret = nxmutex_lock(&g_ivshmem_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Remove the device from the device queue */

  dq_rem(&dev->node, &g_ivshmem_bus.dev);

  nxmutex_unlock(&g_ivshmem_bus.lock);

  /* Call driver remove and mark tmp->driver NULL to indicate
   * the device unmatched
   */

  if (dev->drv)
    {
      dev->drv->remove(dev);
    }

  return ret;
}

/****************************************************************************
 * Name: ivshmem_get_driver
 *
 * Description:
 *   Get the ivshmem device corresponding driver.
 *
 ****************************************************************************/

FAR struct ivshmem_driver_s *
ivshmem_get_driver(FAR struct ivshmem_device_s *dev)
{
  return dev->drv;
}

/****************************************************************************
 * Name: ivshmem_get_shmem
 *
 * Description:
 *   Get the ivshmem device share memory address and size.
 *
 ****************************************************************************/

FAR void *ivshmem_get_shmem(FAR struct ivshmem_device_s *dev,
                            FAR size_t *size)
{
  *size = dev->shmem_size;
  return dev->shmem;
}

/****************************************************************************
 * Name: ivshmem_attach_irq
 *
 * Description:
 *   Attach/detach the interrupt handler to the ivshmem interrupt
 *
 ****************************************************************************/

int ivshmem_attach_irq(FAR struct ivshmem_device_s *dev, xcpt_t isr,
                       FAR void *arg)
{
  if (dev->vmid == IVSHMEM_INVALID_VMID)
    {
      return -ENOTSUP;
    }

  return irq_attach(dev->irq, isr, arg);
}

/****************************************************************************
 * Name: ivshmem_detach_irq
 *
 * Description:
 *   Detach the interrupt handler to the ivshmem interrupt
 *
 ****************************************************************************/

int ivshmem_detach_irq(FAR struct ivshmem_device_s *dev)
{
  if (dev->vmid == IVSHMEM_INVALID_VMID)
    {
      return -ENOTSUP;
    }

  return irq_detach(dev->irq);
}

/****************************************************************************
 * Name: ivshmem_control_irq
 *
 * Description:
 *   Enable/Disable the ivshmem interrupt
 *
 ****************************************************************************/

int ivshmem_control_irq(FAR struct ivshmem_device_s *dev, bool on)
{
  if (dev->vmid == IVSHMEM_INVALID_VMID)
    {
      return -ENOTSUP;
    }

  if (on)
    {
      up_enable_irq(dev->irq);
    }
  else
    {
      up_disable_irq(dev->irq);
    }

  return OK;
}

/****************************************************************************
 * Name: ivshmem_support_irq
 *
 * Description:
 *   Judge if support ivshmem interrupt
 *
 ****************************************************************************/

bool ivshmem_support_irq(FAR struct ivshmem_device_s *dev)
{
  return dev->vmid != IVSHMEM_INVALID_VMID;
}

/****************************************************************************
 * Name: ivshmem_kick_peer
 *
 * Description:
 *   Send interrupt to peer
 *
 ****************************************************************************/

int ivshmem_kick_peer(FAR struct ivshmem_device_s *dev)
{
  uint32_t peer_id;

  if (dev->vmid == IVSHMEM_INVALID_VMID)
    {
      return -ENOTSUP;
    }

  if (dev->vmid == 0)
    {
      peer_id = 1;
    }
  else
    {
      peer_id = 0;
    }

  DEBUGASSERT(dev->kick_peer != NULL);

  return dev->kick_peer(dev, peer_id);
}

/****************************************************************************
 * Name: ivshmem_register_driver
 ****************************************************************************/

int ivshmem_register_driver(FAR struct ivshmem_driver_s *drv)
{
  FAR struct ivshmem_device_s *dev;
  int ret;

  DEBUGASSERT(drv != NULL && drv->probe != NULL &&
              drv->remove != NULL);

  ret = nxmutex_lock(&g_ivshmem_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Add the driver to the ivshmem_bus driver queue */

  dq_addlast(&drv->node, &g_ivshmem_bus.drv);

  /* Match all the devices has registered in the ivshmem_bus */

  dq_for_every_entry(&g_ivshmem_bus.dev, dev, struct ivshmem_device_s, node)
    {
      if (drv->id == dev->id)
        {
          /* If found the device in the device queue, call driver probe,
           * if probe success, assign item->driver to indicate the device
           * matched.
           */

          dev->drv = drv;
          if (drv->probe(dev) < 0)
            {
              dev->drv = NULL;
            }
        }
    }

  nxmutex_unlock(&g_ivshmem_bus.lock);
  return ret;
}

/****************************************************************************
 * Name: ivshmem_unregister_driver
 ****************************************************************************/

int ivshmem_unregister_driver(FAR struct ivshmem_driver_s *drv)
{
  FAR struct ivshmem_device_s *dev;
  int ret;

  DEBUGASSERT(drv != NULL);

  ret = nxmutex_lock(&g_ivshmem_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find all the devices matched with driver in device queue */

  dq_for_every_entry(&g_ivshmem_bus.dev, dev, struct ivshmem_device_s, node)
    {
      if (dev->drv == drv)
        {
          /* 1. Call driver remove function;
           * 2. Mark dev->drv NULL to indicate the device unmatched;
           */

          drv->remove(dev);
          dev->drv = NULL;
        }
    }

  /* Remove the driver from the driver queue */

  dq_rem(&drv->node, &g_ivshmem_bus.drv);

  nxmutex_unlock(&g_ivshmem_bus.lock);
  return ret;
}
