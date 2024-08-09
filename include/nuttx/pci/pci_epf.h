/****************************************************************************
 * include/nuttx/pci/pci_epf.h
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

#ifndef __INCLUDE_NUTTX_PCI_PCI_EPF_H
#define __INCLUDE_NUTTX_PCI_PCI_EPF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/compiler.h>
#include <nuttx/list.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/pci/pci_regs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* struct pci_epf_header_s - Represents standard configuration header
 *
 * vendorid: Identifies device manufacturer
 * deviceid: Identifies a particular device
 * revid: Specifies a device-specific revision identifier
 * progif_code: Identifies a specific register-level programming interface
 * subclass_code: Identifies more specifically the function of the device
 * baseclass_code: Broadly classifies the type of function the device
 * performs
 * cache_line_size: Specifies the system cacheline size in units of DWORDs
 * interrupt_pin: Interrupt pin the device (or device function) uses
 * subsys_vendor_id: Vendor of the add-in card or subsystem
 * subsys_id: Id specific to vendor
 */

struct pci_epf_header_s
{
  uint16_t vendorid;
  uint16_t deviceid;
  uint8_t revid;
  uint8_t progif_code;
  uint8_t subclass_code;
  uint8_t baseclass_code;
  uint8_t cache_line_size;
  uint8_t interrupt_pin;
  uint16_t subsys_vendor_id;
  uint16_t subsys_id;
};

/* struct pci_epf_bar_s - Represents the BAR of EPF device
 *
 * phys_addr: Physical address value that should be mapped to the BAR
 * addr: Virtual address pointer corresponding to the phys_addr
 * size: The size of the address space present in BAR
 * barno: BAR number
 * flags: Flags that are set for the BAR
 */

struct pci_epf_bar_s
{
  uintptr_t phys_addr;
  FAR void *addr;
  size_t size;
  int barno;
  int flags;
};

struct pci_epf_device_id_s
{
  char name[32];
  unsigned long driver_data;
};

/* struct pci_epf_device_s - Represents the PCI EPF device
 *
 * name: The name of the PCI EPF device
 * header: Represents standard configuration header
 * bar: Represents the BAR of EPF device
 * msi_interrupts: Number of MSI interrupts required by this function
 * msix_interrupts: Number of MSI-X interrupts required by this function
 * funcno: Unique (physical) function number within this endpoint device
 * epc: The EPC device to which this EPF device is bound
 * driver: The EPF driver to which this EPF device is bound
 * id: Pointer to the EPF device ID
 * node: To add pci_epf as a list of PCI endpoint functions to pci_epc_ctrl_s
 * lock: mutex to protect pci_epf_ops_s
 * is_bound: Indicates if bind notification to function driver has been
 *   invoked
 * event_ops: Callbacks for capturing the EPC events
 * priv: The private data
 */

struct pci_epf_device_s
{
  FAR const char *name;
  FAR const char *epc_name;
  FAR struct pci_epf_header_s *header;
  struct pci_epf_bar_s bar[PCI_STD_NUM_BARS];
  uint8_t msi_interrupts;
  uint16_t msix_interrupts;
  uint8_t funcno;

  FAR struct pci_epc_ctrl_s *epc;
  FAR struct pci_epf_driver_s *driver;
  FAR const struct pci_epf_device_id_s *id;
  struct list_node node;
  struct list_node epc_node;

  /* Mutex to protect against concurrent access of pci_epf_ops_s */

  mutex_t lock;
  bool is_bound;
  FAR const struct pci_epc_event_ops_s *event_ops;
  FAR void *priv;
};

/* struct pci_epf_ops_s - Set of function pointers for performing EPF
 *   operations
 *
 * bind: Ops to perform when a EPC device has been bound to EPF device
 * unbind: Ops to perform when a binding has been lost between a EPC device
 *   and EPF device
 */

struct pci_epf_ops_s
{
  CODE int (*bind)(FAR struct pci_epf_device_s *epf);
  CODE void (*unbind)(FAR struct pci_epf_device_s *epf);
};

/* struct pci_epc_event_ops_s - Callbacks for capturing the EPC events
 *
 * core_init: Callback for the EPC initialization complete event
 * link_up: Callback for the EPC link up event
 * link_down: Callback for the EPC link down event
 * bme: Callback for the EPC BME (Bus Master Enable) event
 */

struct pci_epc_event_ops_s
{
  CODE int (*core_init)(FAR struct pci_epf_device_s *epf);
  CODE int (*link_up)(FAR struct pci_epf_device_s *epf);
  CODE int (*link_down)(FAR struct pci_epf_device_s *epf);
  CODE int (*bme)(FAR struct pci_epf_device_s *epf);
};

/* struct pci_epf_driver_s - Represents the PCI EPF driver
 *
 * probe: Ops to perform when a new EPF device has been bound to the EPF
 *    driver
 * remove: Ops to perform when the binding between the EPF device and EPF
 *    driver is broken
 * node: EPF driver list node
 * ops: Set of function pointers for performing EPF operations
 * id_table: Identifies EPF devices for probing
 */

struct pci_epf_driver_s
{
  CODE int (*probe)(FAR struct pci_epf_device_s *epf);
  CODE void (*remove)(FAR struct pci_epf_device_s *epf);

  struct list_node node;
  FAR const struct pci_epf_ops_s *ops;
  FAR const struct pci_epf_device_id_s *id_table;
};

/* struct pci_epf_msix_tbl_s - Represents the MSIX table entry structure
 *
 * msg_addr: Writes to this address will trigger MSIX interrupt in host
 * msg_data: Data that should be written to msg_addr to trigger MSIX
 *   interrupt
 * vector_ctrl: Identifies if the function is prohibited from sending a
 *   message using this MSIX table entry
 */

struct pci_epf_msix_tbl_s
{
  uint64_t msg_addr;
  uint32_t msg_data;
  uint32_t vector_ctrl;
};

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

int pci_epf_device_register(FAR struct pci_epf_device_s *epf);

/****************************************************************************
 * Name: pci_epf_device_unregister
 *
 * Description:
 *   This function is used to unregister a PCI EPF device.
 *
 *   Invoke to unregister the PCI EPF device.
 *
 * Input Parameters:
 *   epf - PCI EPF device
 *
 * Returned Value:
 *    Return >= 0 if success, < 0 if failed
 ****************************************************************************/

int pci_epf_device_unregister(FAR struct pci_epf_device_s *epf);

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
 *    Return >= 0 if success, < 0 if failed
 ****************************************************************************/

int pci_epf_register_driver(FAR struct pci_epf_driver_s *drv);

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

int pci_epf_unregister_driver(FAR struct pci_epf_driver_s *drv);

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
 *   Return space address malloced if success, otherwise NULL
 ****************************************************************************/

FAR void *pci_epf_alloc_space(FAR struct pci_epf_device_s *epf, int barno,
                              size_t size, size_t align);

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
                        int barno, FAR void *addr);

#endif /* __INCLUDE_NUTTX_PCI_PCI_EPF_H */
