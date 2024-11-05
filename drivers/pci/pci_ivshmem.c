/****************************************************************************
 * drivers/pci/pci_ivshmem.c
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

#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/pci/pci.h>
#include <nuttx/pci/pci_ivshmem.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IVSHMEM_REG_BAR         0
#define IVSHMEM_MSIX_BAR        1
#define IVSHMEM_SHMEM_BAR       2

#define IVSHMEM_REG_INT_MASK    0
#define IVSHMEM_REG_INT_STATUS  4
#define IVSHMEM_REG_IVPOSITION  8
#define IVSHMEM_REG_DOORBELL    12

#define IVSHMEM_INVALID_VMID    UINT32_MAX

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ivshmem_bus_s
{
  mutex_t          lock; /* Lock for the list */
  struct list_node dev;  /* Wait match ivshmem device list */
  struct list_node drv;  /* Ivshmem driver list */
};

struct ivshmem_device_s
{
  FAR struct pci_device_s     *dev;

  FAR struct ivshmem_driver_s *drv;
  struct list_node             node;

  int                          id;

  FAR void                    *shmem;
  size_t                       shmem_size;

  FAR void                    *reg;
  int                          irq;
  uint32_t                     vmid;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ivshmem_probe(FAR struct pci_device_s *dev);
static void ivshmem_remove(FAR struct pci_device_s *dev);

static int ivshmem_register_device(FAR struct ivshmem_device_s *dev);
static int ivshmem_unregister_device(FAR struct ivshmem_device_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_ivshmem_next_id = 0;

static struct ivshmem_bus_s g_ivshmem_bus =
{
  NXMUTEX_INITIALIZER,
  LIST_INITIAL_VALUE(g_ivshmem_bus.dev),
  LIST_INITIAL_VALUE(g_ivshmem_bus.drv),
};

static const struct pci_device_id_s g_ivshmem_id_table[] =
{
  { PCI_DEVICE(0x1af4, 0x1110) },
  { 0, }
};

static struct pci_driver_s g_ivshmem_drv =
{
  g_ivshmem_id_table, /* PCI id table */
  ivshmem_probe,      /* Probe function */
  ivshmem_remove      /* Remove function */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ivshmem_register_device
 ****************************************************************************/

static int ivshmem_register_device(FAR struct ivshmem_device_s *dev)
{
  FAR struct ivshmem_driver_s *drv;
  int ret;

  ret = nxmutex_lock(&g_ivshmem_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Add the device to the ivshmem_bus device list */

  list_add_tail(&g_ivshmem_bus.dev, &dev->node);

  /* Match the driver has registered in the ivshmem_bus */

  list_for_every_entry(&g_ivshmem_bus.drv, drv, struct ivshmem_driver_s,
                       node)
    {
      if (drv->id == dev->id)
        {
          /* If found the driver in the driver list, call driver probe,
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

static int ivshmem_unregister_device(FAR struct ivshmem_device_s *dev)
{
  int ret;

  ret = nxmutex_lock(&g_ivshmem_bus.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Remove the device from the device list */

  list_delete(&dev->node);

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
 * Name: ivshmem_probe
 ****************************************************************************/

static int ivshmem_probe(FAR struct pci_device_s *dev)
{
  FAR struct ivshmem_device_s *ivdev;
  int ret;

  ivdev = kmm_zalloc(sizeof(*ivdev));
  if (ivdev == NULL)
    {
      return -ENOMEM;
    }

  dev->priv  = ivdev;
  ivdev->dev = dev;
  ivdev->id  = g_ivshmem_next_id;

  ret = pci_enable_device(dev);
  if (ret < 0)
    {
      pciinfo("Enable device failed, ret=%d\n", ret);
      goto err;
    }

  pci_set_master(dev);

  ivdev->reg = pci_map_bar(dev, IVSHMEM_REG_BAR);
  if (ivdev->reg == NULL)
    {
      pcierr("Device Not support Register Bar\n");
      ret = -ENOTTY;
      goto err_master;
    }

  ivdev->shmem = pci_map_bar(dev, IVSHMEM_SHMEM_BAR);
  if (ivdev->shmem == NULL)
    {
      pcierr("Device Not support Share Memory Bar\n");
      ret = -ENOTTY;
      goto err_master;
    }

  ivdev->shmem_size = pci_resource_len(dev, IVSHMEM_SHMEM_BAR);

  pciinfo("shmem addr=%p size=%zu reg=%p\n",
          ivdev->shmem, ivdev->shmem_size, ivdev->reg);

  if (pci_resource_flags(dev, IVSHMEM_MSIX_BAR) != 0)
    {
      pci_read_mmio_dword(dev,
                          (FAR char *)ivdev->reg + IVSHMEM_REG_IVPOSITION,
                          &ivdev->vmid);

      if (ivdev->vmid != 0 && ivdev->vmid != 1)
        {
          ret = -EINVAL;
          pcierr("Vmid must be 0 or 1\n");
          goto err_master;
        }

      ret = pci_alloc_irq(dev, &ivdev->irq, 1);
      if (ret != 1)
        {
          pcierr("Failed to allocate irq %d\n", ret);
          goto err_master;
        }

      ret = pci_connect_irq(dev, &ivdev->irq, 1);
      if (ret < 0)
        {
          pcierr("Failed to connect irq %d\n", ret);
          goto err_msi_alloc;
        }

      pciinfo("vmid=%" PRIu32 " irq=%d\n", ivdev->vmid, ivdev->irq);
    }
  else
    {
      ivdev->vmid = IVSHMEM_INVALID_VMID;
    }

  ret = ivshmem_register_device(ivdev);
  if (ret < 0)
    {
      goto err_master;
    }

  g_ivshmem_next_id++;
  return ret;

err_msi_alloc:
  pci_release_irq(dev, &ivdev->irq, 1);
err_master:
  pci_clear_master(dev);
  pci_disable_device(dev);
err:
  kmm_free(ivdev);
  return ret;
}

/****************************************************************************
 * Name: ivshmem_remove
 ****************************************************************************/

static void ivshmem_remove(FAR struct pci_device_s *dev)
{
  FAR struct ivshmem_device_s *ivdev = dev->priv;

  ivshmem_unregister_device(ivdev);
  if (ivdev->vmid != IVSHMEM_INVALID_VMID)
    {
      pci_release_irq(dev, &ivdev->irq, 1);
    }

  pci_clear_master(ivdev->dev);
  pci_disable_device(ivdev->dev);
  kmm_free(ivdev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

  /* We only use one vector, so vector is 0, peer_id must be 0 or 1
   * bit 0..15: vector
   * bit 16..31: peer ID
   */

  pci_write_mmio_dword(dev->dev, dev->reg + IVSHMEM_REG_DOORBELL,
                       peer_id << 16);
  return 0;
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

  /* Add the driver to the ivshmem_bus driver list */

  list_add_tail(&g_ivshmem_bus.drv, &drv->node);

  /* Match all the devices has registered in the ivshmem_bus */

  list_for_every_entry(&g_ivshmem_bus.dev, dev,
                       struct ivshmem_device_s, node)
    {
      if (drv->id == dev->id)
        {
          /* If found the device in the device list, call driver probe,
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

  /* Find all the devices matched with driver in device list */

  list_for_every_entry(&g_ivshmem_bus.dev, dev,
                       struct ivshmem_device_s, node)
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

  /* Remove the driver from the driver list */

  list_delete(&drv->node);

  nxmutex_unlock(&g_ivshmem_bus.lock);
  return ret;
}

/****************************************************************************
 * Name: pci_ivshmem_register
 ****************************************************************************/

int pci_ivshmem_register(void)
{
  return pci_register_driver(&g_ivshmem_drv);
}
