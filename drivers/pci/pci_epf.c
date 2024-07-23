/****************************************************************************
 * drivers/pci/pci_epf.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/lib/math32.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pci/pci_epc.h>
#include <nuttx/pci/pci_epf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ALIGN_UP
#  define ALIGN_UP(s, a)            (((s) + (a) - 1) & ~((a) - 1))
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct list_node g_pci_epf_device_list =
                        LIST_INITIAL_VALUE(g_pci_epf_device_list);
static struct list_node g_pci_epf_driver_list =
                        LIST_INITIAL_VALUE(g_pci_epf_driver_list);
static mutex_t g_pci_epf_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_epf_match_id
 ****************************************************************************/

static FAR const struct pci_epf_device_id_s *
pci_epf_match_id(FAR const struct pci_epf_device_s *dev,
                 FAR const struct pci_epf_device_id_s *id)
{
  while (id->name[0])
    {
      if (strcmp(dev->name, id->name) == 0)
        {
          return id;
        }

      id++;
    }

  return NULL;
}

/****************************************************************************
 * Name: pci_epf_match_device
 ****************************************************************************/

static FAR const struct pci_epf_device_id_s *
pci_epf_match_device(FAR struct pci_epf_device_s *dev,
                     FAR struct pci_epf_driver_s *drv)
{
  if (drv->id_table)
    {
      return pci_epf_match_id(dev, drv->id_table);
    }
  else
    {
      return NULL;
    }
}

/****************************************************************************
 * Name: pci_epf_unbind
 *
 * Description:
 *   Notify the function driver that the binding between the EPF device and
 * EPC device has been lost.
 *
 *   Invoke to notify the function driver that the binding between the EPF
 * device and EPC device has been lost.
 *
 * Input Parameters:
 *   epf - The EPF device which has lost the binding with the EPC device
 *
 * Returned Value:
 *   None
 ****************************************************************************/

static void pci_epf_unbind(FAR struct pci_epf_device_s *epf)
{
  if (epf->driver == NULL || epf->driver->ops->unbind == NULL)
    {
      pcierr("Invalid epf parameter, unbind failed!\n");
      return;
    }

  nxmutex_lock(&epf->lock);
  if (epf->is_bound)
    {
      epf->driver->ops->unbind(epf);
      epf->is_bound = false;
    }

  nxmutex_unlock(&epf->lock);
}

/****************************************************************************
 * Name: pci_epf_bind
 *
 * Description:
 *   Notify the function driver that the EPF device has been bound to a EPC
 * device.
 *
 *   Invoke to notify the function driver that it has been bound to a EPC
 * device.
 *
 * Input Parameters:
 *   epf - The EPF device which has been bound to the EPC device
 *
 * Returned Value:
 *   Return 0 if success, negative if failed
 ****************************************************************************/

static int pci_epf_bind(FAR struct pci_epf_device_s *epf)
{
  int ret;

  if (epf->driver == NULL || epf->driver->ops->bind == NULL)
    {
      pcierr("Invalid epf parameter, bind failed!\n");
      return -EINVAL;
    }

  nxmutex_lock(&epf->lock);
  ret = epf->driver->ops->bind(epf);
  if (ret >= 0)
    {
      epf->is_bound = true;
    }

  nxmutex_unlock(&epf->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_epf_free_space
 *
 * Description:
 *   Free the allocated PCI EPF register space
 *
 *   Invoke to free the allocated PCI EPF register space.
 *
 * Input Parameters:
 *   epf    - The EPF device from whom to free the memory
 *   barno  - The BAR number corresponding to the register space
 *   addr   - The virtual address of the PCI EPF register space
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void pci_epf_free_space(FAR struct pci_epf_device_s *epf,
                        int barno, FAR void *addr)
{
  FAR struct pci_epf_bar_s *bar;

  if (addr == NULL || barno >= PCI_STD_NUM_BARS)
    {
      return;
    }

  bar = epf->bar;

  kmm_free(addr);

  bar[barno].phys_addr = 0;
  bar[barno].addr = NULL;
  bar[barno].size = 0;
  bar[barno].barno = 0;
  bar[barno].flags = 0;
}

/****************************************************************************
 * Name: pci_epf_alloc_space
 *
 * Description:
 *   This function is used to allocate memory for the PCI EPF register
 * space.
 *
 *   Invoke to allocate memory for the PCI EPF register space.
 *
 * Input Parameters:
 *   epf     - The EPF device to whom allocate the memory
 *   barno   - The BAR number corresponding to the allocated register space
 *   size    - The size of the memory that has to be allocated
 *   align   - Alignment size for the allocation region
 *
 * Returned Value:
 *  Return space address malloced if success, otherwise NULL
 ****************************************************************************/

FAR void *pci_epf_alloc_space(FAR struct pci_epf_device_s *epf, int barno,
                              size_t size, size_t align)
{
  FAR struct pci_epf_bar_s *bar;
  FAR void *space;
  uintptr_t phys_addr;

  if (epf == NULL || barno >= PCI_STD_NUM_BARS)
    {
      return NULL;
    }

  if (size < 128)
    {
      size = 128;
    }

  if (align)
    {
      size = ALIGN_UP(size, align);
    }
  else
    {
      size = roundup_pow_of_two(size);
    }

  bar = epf->bar;

  space = kmm_zalloc(size);
  if (space == NULL)
    {
      pcierr("Failed to allocate mem space\n");
      return NULL;
    }

  phys_addr = up_addrenv_va_to_pa(space);

  bar[barno].phys_addr = phys_addr;
  bar[barno].addr = space;
  bar[barno].size = size;
  bar[barno].barno = barno;
  bar[barno].flags |= (size > UINT32_MAX) ?
                       PCI_BASE_ADDRESS_MEM_TYPE_64 :
                       PCI_BASE_ADDRESS_MEM_TYPE_32;

  return space;
}

/****************************************************************************
 * Name: pci_epf_device_register
 *
 * Description:
 *   This function is used to create a new PCI EPF device.
 *
 *   Invoke to create a new PCI EPF device by providing the name of the
 * function device.
 *
 * Input Parameters:
 *   epf - The name of the PCI EPF device be used to bind the EPF device to
 * a EPF driver
 *
 * Returned Value:
 *    Return >= 0 if success, < 0 if failed
 ****************************************************************************/

int pci_epf_device_register(FAR struct pci_epf_device_s *epf)
{
  FAR struct pci_epf_driver_s *drv;
  FAR struct pci_epc_ctrl_s *epc;
  int ret;

  DEBUGASSERT(epf != NULL || epf->name != NULL);

  ret = nxmutex_lock(&g_pci_epf_lock);
  if (ret < 0)
    {
      return ret;
    }

  list_add_tail(&g_pci_epf_device_list, &epf->node);

  list_for_every_entry(&g_pci_epf_driver_list, drv,
                       struct pci_epf_driver_s, node)
    {
      epf->id = pci_epf_match_device(epf, drv);
      if (epf->id == NULL)
        {
          continue;
        }

      /* Find the specify EPC by epc_name in epf structure */

      epc = pci_get_epc(epf->epc_name);
      if (epc == NULL)
        {
          ret = -ENODEV;
          break;
        }

      ret = drv->probe(epf);
      if (ret < 0)
        {
          continue;
        }

      epf->driver = drv;

      /* Added the epc to epf */

      pci_epc_add_epf(epc, epf);

      /* Notify the function driver */

      pci_epf_bind(epf);
      break;
    }

  nxmutex_unlock(&g_pci_epf_lock);
  return ret;
}

/****************************************************************************
 * Name: pci_epf_device_unregister
 *
 * Description:
 *   This function is used to unregister a PCI EPF device.
 *
 *   Invoke to unregister the PCI EPF device.
 *
 * Input Parameters:
 *   epf - The PCI EPF driver that has to be unregistered device
 *
 * Returned Value:
 *    Return >= 0 if success, < 0 if failed
 ****************************************************************************/

int pci_epf_device_unregister(FAR struct pci_epf_device_s *epf)
{
  FAR struct pci_epc_ctrl_s *epc;
  int ret;

  ret = nxmutex_lock(&g_pci_epf_lock);
  if (ret < 0)
    {
      return ret;
    }

  pci_epf_unbind(epf);
  epc = pci_get_epc(epf->epc_name);
  if (epc)
    {
      pci_epc_remove_epf(epc, epf);
    }

  if (epf->driver && epf->driver->remove)
    {
      epf->driver->remove(epf);
    }

  epf->driver = NULL;
  list_delete(&epf->node);

  nxmutex_unlock(&g_pci_epf_lock);
  return ret;
}

/****************************************************************************
 * Name: pci_epf_register_driver
 *
 * Description:
 *   This function is used to register a new PCI EPF driver.
 *
 * Input Parameters:
 *   drv - EPF driver
 *
 * Returned Value:
 *   Return >= 0 if success, < 0 if failed
 ****************************************************************************/

int pci_epf_register_driver(FAR struct pci_epf_driver_s *drv)
{
  FAR struct pci_epf_device_s *epf;
  FAR struct pci_epc_ctrl_s *epc;
  int ret;

  if (drv == NULL || drv->ops == NULL ||
      drv->ops->bind == NULL || drv->id_table == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_pci_epf_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Add the driver to the pci epf driver list */

  list_add_tail(&g_pci_epf_driver_list, &drv->node);

  list_for_every_entry(&g_pci_epf_device_list, epf, struct pci_epf_device_s,
                       node)
    {
      if (epf->driver != NULL)
        {
          continue;
        }

      /* 1. Match epf device and epf driver.
       * 2. Get the epc by name.
       * 3. Add epc into epf.
       * 4. Notify the epf driver be bond.
       */

      epf->id = pci_epf_match_device(epf, drv);
      if (epf->id == NULL)
        {
          continue;
        }

      epc = pci_get_epc(epf->epc_name);
      if (epc == NULL)
        {
          ret = -ENODEV;
          break;
        }

      ret = drv->probe(epf);
      if (ret < 0)
        {
          continue;
        }

      epf->driver = drv;

      /* Added the epc to epf */

      pci_epc_add_epf(epc, epf);

      /* Notify the function driver */

      pci_epf_bind(epf);
    }

  nxmutex_unlock(&g_pci_epf_lock);
  return ret;
}

/****************************************************************************
 * Name: pci_epf_unregister_driver
 *
 * Description:
 *   This function is used to unregister the PCI EPF driver.
 *
 *   Invoke to unregister the PCI EPF driver.
 *
 * Input Parameters:
 *   drv - The PCI EPF driver that has to be unregistered
 *
 * Returned Value:
 *    Return >= 0 if success, < 0 if failed
 ****************************************************************************/

int pci_epf_unregister_driver(FAR struct pci_epf_driver_s *drv)
{
  FAR struct pci_epf_device_s *epf;
  int ret;

  DEBUGASSERT(drv != NULL || drv->remove != NULL);

  ret = nxmutex_lock(&g_pci_epf_lock);
  if (ret < 0)
    {
      return ret;
    }

  list_for_every_entry(&g_pci_epf_device_list, epf, struct pci_epf_device_s,
                       node)
    {
      if (epf->driver == drv)
        {
          pci_epf_unbind(epf);
          drv->remove(epf);
          epf->driver = NULL;
        }
    }

  list_delete(&drv->node);

  nxmutex_unlock(&g_pci_epf_lock);
  return ret;
}
