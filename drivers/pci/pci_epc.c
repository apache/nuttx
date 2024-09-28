/****************************************************************************
 * drivers/pci/pci_epc.c
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

#include <assert.h>
#include <errno.h>
#include <string.h>
#include <debug.h>

#include <nuttx/bits.h>
#include <nuttx/kmalloc.h>
#include <nuttx/lib/math32.h>
#include <nuttx/pci/pci_epc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_pci_epc_lock = NXMUTEX_INITIALIZER;
static struct list_node g_pci_epc_device_list =
                        LIST_INITIAL_VALUE(g_pci_epc_device_list);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_get_epc
 *
 * Description:
 *   This function is used to get a PCI endpoint controller.
 *
 *   Invoke to get struct pci_epc_ctrl_s * corresponding to the device name
 * of the endpoint controller.
 *
 * Input Parameters:
 *   epc_name - Device name of the endpoint controller
 *
 * Returned Value:
 *   Return epc created if success, NULL if failed
 ****************************************************************************/

FAR struct pci_epc_ctrl_s *pci_get_epc(FAR const char *epc_name)
{
  FAR struct pci_epc_ctrl_s *res = NULL;
  FAR struct pci_epc_ctrl_s *epc;
  int ret;

  DEBUGASSERT(epc_name != NULL);

  ret = nxmutex_lock(&g_pci_epc_lock);
  if (ret < 0)
    {
      return NULL;
    }

  list_for_every_entry(&g_pci_epc_device_list, epc, struct pci_epc_ctrl_s,
                       node)
    {
      if (strcmp(epc_name, epc->name) == 0)
        {
          res = epc;
          break;
        }
    }

  nxmutex_unlock(&g_pci_epc_lock);
  return res;
}

/****************************************************************************
 * Name: pci_epc_get_next_free_bar
 *
 * Description:
 *   Helper to get unreserved BAR starting from bar.
 *
 *   Invoke to get the next unreserved BAR starting from barno that can be
 * used for endpoint function. For any incorrect value in bar_reserved return
 * '0'.
 *
 * Input Parameters:
 *   epc_features - pci_epc_features_s structure that holds the reserved bar
 * bitmap
 *   bar          - The starting BAR number from where unreserved BAR should
 * be searched
 *
 * Returned Value:
 *    Return the member if success, negative if failed
 ****************************************************************************/

int pci_epc_get_next_free_bar(
  FAR const struct pci_epc_features_s *epc_features, int barno)
{
  unsigned long free_bar;

  if (epc_features == NULL)
    {
      return -EINVAL;
    }

  /* If 'bar - 1' is a 64-bit BAR, move to the next BAR */

  if ((epc_features->bar_fixed_64bit << 1) & (1 << barno))
    {
      barno++;
    }

  /* Find if the reserved BAR is also a 64-bit BAR */

  free_bar = epc_features->bar_reserved & epc_features->bar_fixed_64bit;

  /* Set the adjacent bit if the reserved BAR is also a 64-bit BAR */

  free_bar <<= 1;
  free_bar |= epc_features->bar_reserved;

  free_bar = find_next_zero_bit(&free_bar, PCI_STD_NUM_BARS, barno);
  if (free_bar >= PCI_STD_NUM_BARS)
    {
      return -ENOENT;
    }

  return free_bar;
}

/****************************************************************************
 * Name: pci_epc_get_features
 *
 * Description:
 *   This function is used to get the features supported by EPC.
 *
 *   Invoke to get the features provided by the EPC which may be
 * specific to an endpoint function. Returns pci_epc_features_s on success
 * and NULL for any failures.
 *
 * Input Parameters:
 *   epc     - The features supported by *this* EPC device will be returned
 *   funcno  - The features supported by the EPC device specific to the
 * endpoint function with funcno will be returned
 *
 * Returned Value:
 *    Epc features if success, NULL if failed
 ****************************************************************************/

FAR const struct pci_epc_features_s *
pci_epc_get_features(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno)
{
  FAR const struct pci_epc_features_s *epc_features;

  if (epc == NULL || epc->ops->get_features == NULL ||
      funcno >= epc->max_functions)
    {
      return NULL;
    }

  nxmutex_lock(&epc->lock);
  epc_features = epc->ops->get_features(epc, funcno);
  nxmutex_unlock(&epc->lock);

  return epc_features;
}

/****************************************************************************
 * Name: pci_epc_stop
 *
 * Description:
 *   This function is used to stop the PCI link.
 *
 *   Invoke to stop the PCI link.
 *
 * Input Parameters:
 *   epc - The link of the EPC device that has to be stopped
 *
 * Returned Value:
 *    None
 ****************************************************************************/

void pci_epc_stop(FAR struct pci_epc_ctrl_s *epc)
{
  if (epc == NULL || !epc->ops->stop)
    {
      return;
    }

  nxmutex_lock(&epc->lock);
  epc->ops->stop(epc);
  nxmutex_unlock(&epc->lock);
}

/****************************************************************************
 * Name: pci_epc_start
 *
 * Description:
 *   This function is used to start the PCI link.
 *
 *   Invoke to start the PCI link.
 *
 * Input Parameters:
 *   epc - The link of *this* EPC device has to be started
 *
 * Returned Value:
 *    Return 0 if success, negative if failed
 ****************************************************************************/

int pci_epc_start(FAR struct pci_epc_ctrl_s *epc)
{
  int ret;

  if (epc == NULL || !epc->ops->start)
    {
      return -EINVAL;
    }

  nxmutex_lock(&epc->lock);
  ret = epc->ops->start(epc);
  nxmutex_unlock(&epc->lock);

  return ret;
}

/****************************************************************************
 * Name: pci_epc_raise_irq
 *
 * Description:
 *   This function is used to interrupt the host system.
 *
 *   Invoke to raise an legacy, MSI or MSI-X interrupt.
 *
 * Input Parameters:
 *   epc           - The EPC device which has to interrupt the host
 *   funcno        - The physical endpoint function number in the EPC device
 *   type          - Specify the type of interrupt; legacy, MSI or MSI-X
 *   interrupt_num - The MSI or MSI-X interrupt number with range (1-N)
 *
 * Returned Value:
 *    Return 0 if success, negative if failed
 ****************************************************************************/

int pci_epc_raise_irq(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                      enum pci_epc_irq_type_e type, uint16_t interrupt_num)
{
  int ret;

  if (epc == NULL || epc->ops->raise_irq == NULL ||
      funcno >= epc->max_functions)
    {
      return -EINVAL;
    }

  nxmutex_lock(&epc->lock);
  ret = epc->ops->raise_irq(epc, funcno, type, interrupt_num);
  nxmutex_unlock(&epc->lock);

  return ret;
}

/****************************************************************************
 * Name: pci_epc_map_msi_irq
 *
 * Description:
 *   Map physical address to MSI address and return MSI data.
 *
 *   Invoke to map physical address to MSI address and return MSI data. The
 * physical address should be an address in the outbound region. This is
 * required to implement doorbell functionality of NTB wherein EPC on either
 * side of the interface (primary and secondary) can directly write to the
 * physical address (in outbound region) of the other interface to ring
 * doorbell.
 *
 * Input Parameters:
 *   epc             - The EPC device which has to interrupt the host
 *   funcno          - The physical endpoint function number in the EPC
 * device
 *   phys_addr       - The physical address of the outbound region
 *   interrupt_num   - The MSI or MSI-X interrupt number with range (1-N)
 *   entry_size      - Size of Outbound address region for each interrupt
 *   msi_data        - The data that should be written in order to raise MSI
 * interrupt with interrupt number as 'interrupt num'
 *   msi_addr_offset - Offset of MSI address from the aligned outbound
 * address to which the MSI address is mapped
 *
 * Returned Value:
 *    Return 0 if success, negative if failed
 ****************************************************************************/

int pci_epc_map_msi_irq(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                        uintptr_t phys_addr, uint8_t interrupt_num,
                        uint32_t entry_size, FAR uint32_t *msi_data,
                        FAR uint32_t *msi_addr_offset)
{
  int ret;

  if (epc == NULL && epc->ops->map_msi_irq == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&epc->lock);
  ret = epc->ops->map_msi_irq(epc, funcno, phys_addr, interrupt_num,
                              entry_size, msi_data, msi_addr_offset);
  nxmutex_unlock(&epc->lock);

  return ret;
}

/****************************************************************************
 * Name: pci_epc_get_msi
 *
 * Description:
 *   Get the number of MSI interrupt numbers allocated.
 *
 *   Message Control Register for MSI:bit6-bit4:Multiple Message Enable
 *   Software writes to this field to indicate the number of allocate vectors
 *   Equal to or less than the number of requested vectors. The number of
 *   allocated vectors is aligned to a power of two. If a Function requests
 *   four vectors (indicated by a Multiple Message Capable encoding of
 *   010b), system software can allocate either four, two, or one vector
 *   by writing a 010b, 001b, or 000b to this field, respectively. When
 *   MSI Enable is Set, a Function will be allocated at least 1 vector
 *
 *   Invoke to get the number of MSI interrupts allocated by the RC.
 *
 * Input Parameters:
 *   epc     - The EPC device to which MSI interrupts was requested
 *   funcno  - The physical endpoint function number in the EPC device
 *
 * Returned Value:
 *    Return interrupt number if success, negative if failed
 ****************************************************************************/

int pci_epc_get_msi(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno)
{
  int interrupt;

  if (epc == NULL || funcno >= epc->max_functions ||
      epc->ops->get_msi == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&epc->lock);
  interrupt = epc->ops->get_msi(epc, funcno);
  nxmutex_unlock(&epc->lock);

  if (interrupt >= 0)
    {
      interrupt = 1 << interrupt;
    }

  return interrupt;
}

/****************************************************************************
 * Name: pci_epc_set_msi
 *
 * Description:
 *   Set the number of MSI interrupt numbers required.
 *
 *   Message Control Register for MSI:bit3-bit1:System software reads this
 *   field to determine the number of requested vectors. The number of
 *   requested vectors must be aligned to a power of two (if a Function
 *   requires three vectors, it requests four by initializing this field
 *   to 010b).
 *
 *   Invoke to set the required number of MSI interrupts.
 *
 * Input Parameters:
 *   epc        - The EPC device on which MSI has to be configured
 *   funcno     - The physical endpoint function number in the EPC device
 *   interrupts - Number of MSI interrupts required by the EPF
 *
 * Returned Value:
 *    Return 0 if success, negative if failed
 ****************************************************************************/

int pci_epc_set_msi(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                    uint8_t interrupts)
{
  int ret;

  if (epc == NULL || funcno >= epc->max_functions ||
      interrupts < 1 || interrupts > 32 || epc->ops->set_msi == NULL)
    {
      return -EINVAL;
    }

  interrupts = order_base_2(interrupts);

  nxmutex_lock(&epc->lock);
  ret = epc->ops->set_msi(epc, funcno, interrupts);
  nxmutex_unlock(&epc->lock);

  return ret;
}

/****************************************************************************
 * Name: pci_epc_get_msix
 *
 * Description:
 *   Get the number of MSI-X interrupt numbers allocated.
 *
 *   Message Control Register for MSI-X:bit10-bit0:Table Size
 *   System software reads this field to determine the MSI-X Table Size N,
 *   which is encoded as N-1. For example, a returned value of 000 0000 0011b
 *   indicates a table size of 4.
 *
 *   Invoke to get the number of MSI-X interrupts allocated by the RC.
 *
 * Input Parameters:
 *   epc     - The EPC device to which MSI-X interrupts was requested
 *   funcno  - The physical endpoint function number in the EPC device
 *
 * Returned Value:
 *    Return interrupt + 1 number if success, negative if failed
 ****************************************************************************/

int pci_epc_get_msix(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno)
{
  int interrupt;

  if (epc == NULL || funcno >= epc->max_functions ||
      epc->ops->get_msix == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&epc->lock);
  interrupt = epc->ops->get_msix(epc, funcno);
  nxmutex_unlock(&epc->lock);

  if (interrupt >= 0)
    {
      interrupt += 1;
    }

  return interrupt;
}

/****************************************************************************
 * Name: pci_epc_set_msix
 *
 * Description:
 *   Set the number of MSI-X interrupt numbers required.
 *
 *   Message Control Register for MSI-X:bit10-bit0:Table Size
 *   System software reads this field to determine the MSI-X Table Size N,
 *   which is encoded as N-1. For example, a returned value of 000 0000 0011b
 *   indicates a table size of 4.
 *
 *   Invoke to set the required number of MSI-X interrupts.
 *
 * Input Parameters:
 *   epc        - The EPC device on which MSI-X has to be configured
 *   funcno     - The physical endpoint function number in the EPC device
 *   interrupts - Number of MSI-X interrupts required by the EPF
 *   barno      - BAR where the MSI-X table resides
 *   offset     - Offset pointing to the start of MSI-X table
 *
 * Returned Value:
 *    Return 0 if success, negative if failed
 ****************************************************************************/

int pci_epc_set_msix(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                     uint16_t interrupts, int barno, uint32_t offset)
{
  int ret;

  if (epc == NULL || funcno >= epc->max_functions || interrupts < 1 ||
      interrupts > 2048 || epc->ops->set_msix == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&epc->lock);
  ret = epc->ops->set_msix(epc, funcno, interrupts - 1, barno, offset);
  nxmutex_unlock(&epc->lock);

  return ret;
}

/****************************************************************************
 * Name: pci_epc_unmap_addr
 *
 * Description:
 *   Unmap CPU address from PCI address.
 *
 *   Invoke to unmap the CPU address from PCI address.
 *
 * Input Parameters:
 *   epc       - The EPC device on which address is allocated
 *   funcno    - The physical endpoint function number in the EPC device
 *   phys_addr - Physical address of the local systeme
 *
 * Returned Value:
 *    None
 ****************************************************************************/

void pci_epc_unmap_addr(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                        uintptr_t phys_addr)
{
  if (epc == NULL || funcno >= epc->max_functions ||
      epc->ops->unmap_addr == NULL)
    {
      return;
    }

  nxmutex_lock(&epc->lock);
  epc->ops->unmap_addr(epc, funcno, phys_addr);
  nxmutex_unlock(&epc->lock);
}

/****************************************************************************
 * Name: pci_epc_map_addr
 *
 * Description:
 *   Map CPU address to PCI address.
 *
 *   Invoke to map CPU address with PCI address.
 *
 * Input Parameters:
 *   epc       - The EPC device on which address is allocated
 *   funcno    - The physical endpoint function number in the EPC device
 *   phys_addr - Physical address of the local system
 *   pci_addr  - PCI address to which the physical address should be mapped
 *   size      - The size of the allocation
 *
 * Returned Value:
 *    Return 0 if success, negative if failed
 ****************************************************************************/

int pci_epc_map_addr(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                     uintptr_t phys_addr, uint64_t pci_addr, size_t size)
{
  int ret;

  if (epc == NULL || funcno >= epc->max_functions ||
      epc->ops->map_addr == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&epc->lock);
  ret = epc->ops->map_addr(epc, funcno, phys_addr, pci_addr, size);
  nxmutex_unlock(&epc->lock);

  return ret;
}

/****************************************************************************
 * Name: pci_epc_clear_bar
 *
 * Description:
 *   Reset the BAR.
 *
 *   Invoke to reset the BAR of the endpoint device.
 *
 * Input Parameters:
 *   epc     - The EPC device for which the BAR has to be cleared
 *   funcno  - The physical endpoint function number in the EPC device
 *   bar     - The struct bar that contains the BAR information
 *
 * Returned Value:
 *    None
 ****************************************************************************/

void pci_epc_clear_bar(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                       FAR struct pci_epf_bar_s *bar)
{
  if (epc == NULL || funcno >= epc->max_functions ||
      epc->ops->clear_bar == NULL ||
      (bar->barno == PCI_STD_NUM_BARS - 1 &&
       (bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64)))
    {
      return;
    }

  nxmutex_lock(&epc->lock);
  epc->ops->clear_bar(epc, funcno, bar);
  nxmutex_unlock(&epc->lock);
}

/****************************************************************************
 * Name: pci_epc_set_bar
 *
 * Description:
 *   Configure BAR in order for host to assign PCI addr space.
 *
 *   Invoke to configure the BAR of the endpoint device.
 *
 * Input Parameters:
 *   epc     - The EPC device on which BAR has to be configured
 *   funcno  - The physical endpoint function number in the EPC device
 *   bar     - The struct bar that contains the BAR information
 *
 * Returned Value:
 *    Return 0 if success, negative if failed
 ****************************************************************************/

int pci_epc_set_bar(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                    FAR struct pci_epf_bar_s *bar)
{
  int flags = bar->flags;
  int ret;

  if (epc == NULL || funcno >= epc->max_functions ||
      (bar->barno == PCI_STD_NUM_BARS - 1 &&
       flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ||
      (bar->size > UINT32_MAX &&
       !(flags & PCI_BASE_ADDRESS_MEM_TYPE_64)) ||
      epc->ops->set_bar == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&epc->lock);
  ret = epc->ops->set_bar(epc, funcno, bar);
  nxmutex_unlock(&epc->lock);

  return ret;
}

/****************************************************************************
 * Name: pci_epc_write_header
 *
 * Description:
 *   Write standard configuration header.
 *
 *   Invoke to write the configuration header to the endpoint controller.
 * Every endpoint controller will have a dedicated location to which the
 * standard configuration header would be written. The callback function
 * should write the header fields to this dedicated location.
 *
 * Input Parameters:
 *   epc     - The EPC device to which the configuration header should be
 * written
 *   funcno  - The physical endpoint function number in the EPC device
 *   header  - Standard configuration header fields
 *
 * Returned Value:
 *    Return 0 if success, negative if failed
 ****************************************************************************/

int pci_epc_write_header(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                         FAR struct pci_epf_header_s *header)
{
  int ret;

  if (epc == NULL || epc->ops->write_header == NULL ||
      funcno >= epc->max_functions)
    {
      return -EINVAL;
    }

  nxmutex_lock(&epc->lock);
  ret = epc->ops->write_header(epc, funcno, header);
  nxmutex_unlock(&epc->lock);

  return ret;
}

/****************************************************************************
 * Name: pci_epc_add_epf
 *
 * Description:
 *   This function is used to bind PCI endpoint function to an endpoint
 * controller.
 *
 *   A PCI endpoint device can have one or more functions In the case of
 * PCIe,the specification allows up to 8 PCIe endpoint functions Invoke
 * pci_epc_add_epf() to add a PCI endpoint function to an endpoint
 * controller.
 *
 * Input Parameters:
 *   epc - The EPC device to which the endpoint function should be added
 *   epf - The endpoint function to be added
 *
 * Returned Value:
 *   Return 0 if success, negative if failed
 ****************************************************************************/

int pci_epc_add_epf(FAR struct pci_epc_ctrl_s *epc,
                    FAR struct pci_epf_device_s *epf)
{
  uint32_t funcno;
  int ret = 0;

  DEBUGASSERT(epc != NULL && epf != NULL);

  if (epf->epc)
    {
      return -EBUSY;
    }

  nxmutex_lock(&epc->lock);

  funcno = find_first_zero_bit(&epc->funcno_map, epc->max_functions);
  if (funcno >= epc->max_functions)
    {
      pcierr("Exceeding max supported Function Number\n");
      ret = -ENOENT;
      goto out;
    }

  set_bit(funcno, &epc->funcno_map);
  epf->funcno = funcno;
  epf->epc = epc;

  list_add_tail(&epc->epf, &epf->epc_node);

out:
  nxmutex_unlock(&epc->lock);
  return ret;
}

/****************************************************************************
 * Name: pci_epc_remove_epf
 *
 * Description:
 *   This function is used to remove PCI endpoint function from endpoint
 * controller.
 *
 *   Invoke to remove PCI endpoint function from the endpoint controller.
 *
 * Input Parameters:
 *   epc - The EPC device from which the endpoint function should be removed
 *   epf - The endpoint function to be removed
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void pci_epc_remove_epf(FAR struct pci_epc_ctrl_s *epc,
                        FAR struct pci_epf_device_s *epf)
{
  if (epc == NULL || epf == NULL)
    {
      return;
    }

  nxmutex_lock(&epc->lock);
  clear_bit(epf->funcno, &epc->funcno_map);
  list_delete(&epf->node);
  epf->epc = NULL;
  nxmutex_unlock(&epc->lock);
}

/****************************************************************************
 * Name: pci_epc_linkup
 *
 * Description:
 *   Notify the EPF device that EPC device has established a connection with
 * the Root Complex.
 *
 *   Invoke to Notify the EPF device that the EPC device has established a
 * connection with the Root Complex.
 *
 * Input Parameters:
 *   epc - The EPC device which has established link with the host
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void pci_epc_linkup(FAR struct pci_epc_ctrl_s *epc)
{
  FAR struct pci_epf_device_s *epf;

  if (epc == NULL)
    {
      return;
    }

  nxmutex_lock(&epc->lock);
  list_for_every_entry(&epc->epf, epf, struct pci_epf_device_s, epc_node)
    {
      nxmutex_lock(&epf->lock);
      if (epf->event_ops && epf->event_ops->link_up)
        {
          epf->event_ops->link_up(epf);
        }

      nxmutex_unlock(&epf->lock);
    }

  nxmutex_unlock(&epc->lock);
}

/****************************************************************************
 * Name: pci_epc_linkdown
 *
 * Description:
 *   Notify the EPF device that EPC device has dropped the connection with
 * the Root Complex.
 *
 *   Invoke to Notify the EPF device that the EPC device has dropped the
 * connection with the Root Complex.
 *
 * Input Parameters:
 *   epc - The EPC device which has dropped the link with the host
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void pci_epc_linkdown(FAR struct pci_epc_ctrl_s *epc)
{
  struct pci_epf_device_s *epf;

  if (epc == NULL)
    {
      return;
    }

  nxmutex_lock(&epc->lock);
  list_for_every_entry(&epc->epf, epf, struct pci_epf_device_s, epc_node)
    {
      nxmutex_lock(&epf->lock);
      if (epf->event_ops && epf->event_ops->link_down)
        {
          epf->event_ops->link_down(epf);
        }

      nxmutex_unlock(&epf->lock);
    }

  nxmutex_unlock(&epc->lock);
}

/****************************************************************************
 * Name: pci_epc_init_notify
 *
 * Description:
 *   Notify the EPF device that EPC device's core initialization is
 * completed.
 *
 *   Invoke to Notify the EPF device that the EPC device's initialization
 * is completed.
 *
 * Input Parameters:
 *   epc - The EPC device whose core initialization is completed
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void pci_epc_init_notify(FAR struct pci_epc_ctrl_s *epc)
{
  FAR struct pci_epf_device_s *epf;

  if (epc == NULL)
    {
      return;
    }

  nxmutex_lock(&epc->lock);
  list_for_every_entry(&epc->epf, epf, struct pci_epf_device_s, epc_node)
    {
      nxmutex_lock(&epf->lock);
      if (epf->event_ops && epf->event_ops->core_init)
        {
          epf->event_ops->core_init(epf);
        }

      nxmutex_unlock(&epf->lock);
    }

  nxmutex_unlock(&epc->lock);
}

/****************************************************************************
 * Name: pci_epc_bme_notify
 *
 * Description:
 *   Notify the EPF device that the EPC device has received the BME event
 * from the Root complex.
 *
 *   Invoke to Notify the EPF device that the EPC device has received the Bus
 * Master Enable (BME) event from the Root complex.
 *
 * Input Parameters:
 *   epc - The EPC device that received the BME event
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void pci_epc_bme_notify(FAR struct pci_epc_ctrl_s *epc)
{
  FAR struct pci_epf_device_s *epf;

  if (epc == NULL)
    {
      return;
    }

  nxmutex_lock(&epc->lock);
  list_for_every_entry(&epc->epf, epf, struct pci_epf_device_s, epc_node)
    {
      nxmutex_lock(&epf->lock);
      if (epf->event_ops && epf->event_ops->bme)
        {
          epf->event_ops->bme(epf);
        }

      nxmutex_unlock(&epf->lock);
    }

  nxmutex_unlock(&epc->lock);
}

/****************************************************************************
 * Name: pci_epc_create
 *
 * Description:
 *   This function is used to destroy the EPC device.
 *
 *   Invoke to create a new EPC device and add it to pci_epc class.
 *
 * Input Parameters:
 *   name        - EPC name strings
 *   priv        - The epc priv data
 *   ops         - Function pointers for performing EPC operations
 *
 * Returned Value:
 *   Return struct pci_epc_ctrl_s * if success, NULL if failed.
 ****************************************************************************/

FAR struct pci_epc_ctrl_s *
pci_epc_create(FAR const char *name, FAR void *priv,
               FAR const struct pci_epc_ops_s *ops)
{
  FAR struct pci_epc_ctrl_s *epc;
  size_t len;

  if (name == NULL || ops == NULL)
    {
      return NULL;
    }

  len = strlen(name) + 1;
  epc = kmm_zalloc(sizeof(*epc) + len);
  if (epc == NULL)
    {
      return NULL;
    }

  epc->priv = priv;
  memcpy(epc->name, name, len);
  nxmutex_init(&epc->lock);
  list_initialize(&epc->epf);
  epc->ops = ops;

  nxmutex_lock(&g_pci_epc_lock);
  list_add_tail(&g_pci_epc_device_list, &epc->node);
  nxmutex_unlock(&g_pci_epc_lock);

  return epc;
}

/****************************************************************************
 * Name: pci_epc_destroy
 *
 * Description:
 *   This function is used to create a new endpoint controller (EPC) device.
 *
 *   Invoke to destroy the PCI EPC device.
 *
 * Input Parameters:
 *   epc - The EPC device that has to be destroyed
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void pci_epc_destroy(FAR struct pci_epc_ctrl_s *epc)
{
  if (epc == NULL)
    {
      return;
    }

  nxmutex_lock(&g_pci_epc_lock);
  list_delete(&epc->node);
  nxmutex_unlock(&g_pci_epc_lock);

  nxmutex_destroy(&epc->lock);
  kmm_free(epc);
}
