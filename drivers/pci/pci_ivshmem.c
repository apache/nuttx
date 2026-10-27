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
#include <nuttx/debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/pci/pci.h>
#include <nuttx/pci/pci_ivshmem.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IVSHMEM_REG_BAR         0
#define IVSHMEM_MSIX_BAR        1
#define IVSHMEM_SHMEM_BAR       2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pci_ivshmem_s
{
  struct ivshmem_device_s      ivdev;
  FAR struct pci_device_s     *dev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pci_ivshmem_probe(FAR struct pci_device_s *dev);
static void pci_ivshmem_remove(FAR struct pci_device_s *dev);
static int pci_ivshmem_kick_peer(FAR struct ivshmem_device_s *ivdev,
                                 uint32_t peer_id);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pci_device_id_s g_pci_ivshmem_id_table[] =
{
  { PCI_DEVICE(0x1af4, 0x1110) },
  { 0, }
};

static struct pci_driver_s g_pci_ivshmem_drv =
{
  g_pci_ivshmem_id_table, /* PCI id table */
  pci_ivshmem_probe,      /* Probe function */
  pci_ivshmem_remove      /* Remove function */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_ivshmem_kick_peer
 ****************************************************************************/

static int pci_ivshmem_kick_peer(FAR struct ivshmem_device_s *ivdev,
                                 uint32_t peer_id)
{
  FAR struct pci_ivshmem_s *priv =
    container_of(ivdev, struct pci_ivshmem_s, ivdev);

  /* We only use one vector, so vector is 0, peer_id must be 0 or 1
   * bit 0..15: vector
   * bit 16..31: peer ID
   */

  pci_write_mmio_dword(priv->dev,
                       (FAR char *)ivdev->reg + IVSHMEM_REG_DOORBELL,
                       peer_id << 16);
  return 0;
}

/****************************************************************************
 * Name: pci_ivshmem_probe
 ****************************************************************************/

static int pci_ivshmem_probe(FAR struct pci_device_s *dev)
{
  FAR struct pci_ivshmem_s *priv;
  FAR struct ivshmem_device_s *ivdev;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  dev->priv  = priv;
  priv->dev  = dev;
  ivdev      = &priv->ivdev;
  ivdev->id  = -1;

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
          pciwarn("Vmid=%"PRIu32" not 0/1, use polling mode\n", ivdev->vmid);
          ivdev->vmid = IVSHMEM_INVALID_VMID;
          goto out;
        }

      ret = pci_alloc_irq(dev, &ivdev->irq, 1);
      if (ret != 1)
        {
          pciwarn("Failed to allocate irq %d, use polling mode\n", ret);
          ivdev->vmid = IVSHMEM_INVALID_VMID;
          goto out;
        }

      ret = pci_connect_irq(dev, &ivdev->irq, 1);
      if (ret < 0)
        {
          pciwarn("Failed to connect irq %d, use polling mode\n", ret);
          pci_release_irq(dev, &ivdev->irq, 1);
          ivdev->vmid = IVSHMEM_INVALID_VMID;
          goto out;
        }

      pciinfo("vmid=%" PRIu32 " irq=%d\n", ivdev->vmid, ivdev->irq);
    }
  else
    {
      ivdev->vmid = IVSHMEM_INVALID_VMID;
    }

out:
  ret = ivshmem_register_device(ivdev, pci_ivshmem_kick_peer);
  if (ret < 0)
    {
      goto err_master;
    }

#ifdef CONFIG_PCI_RTT_IVSHMEM
  if (CONFIG_PCI_RTT_IVSHMEM_IDX == ivdev->id)
    {
      g_segger_offset = (uintptr_t)ivdev->shmem - (uintptr_t)&_SEGGER_RTT;
    }
#endif

  return ret;

err_master:
  pci_clear_master(dev);
  pci_disable_device(dev);
err:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: pci_ivshmem_remove
 ****************************************************************************/

static void pci_ivshmem_remove(FAR struct pci_device_s *dev)
{
  FAR struct pci_ivshmem_s *priv = dev->priv;
  FAR struct ivshmem_device_s *ivdev = &priv->ivdev;

  ivshmem_unregister_device(ivdev);
  if (ivdev->vmid != IVSHMEM_INVALID_VMID)
    {
      pci_release_irq(dev, &ivdev->irq, 1);
    }

  pci_clear_master(priv->dev);
  pci_disable_device(priv->dev);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_ivshmem_register
 ****************************************************************************/

int pci_ivshmem_register(void)
{
  return pci_register_driver(&g_pci_ivshmem_drv);
}
