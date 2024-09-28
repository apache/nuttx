/****************************************************************************
 * include/nuttx/pci/pci_epc.h
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

#ifndef __INCLUDE_NUTTX_PCI_PCI_EPC_H
#define __INCLUDE_NUTTX_PCI_PCI_EPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/pci/pci_epf.h>
#include <nuttx/pci/pci_regs.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum pci_epc_irq_type_e
{
  PCI_EPC_IRQ_UNKNOWN,
  PCI_EPC_IRQ_LEGACY,
  PCI_EPC_IRQ_MSI,
  PCI_EPC_IRQ_MSIX,
};

/* struct pci_epc_features_s - Features supported by a EPC device per
 * function
 *
 * linkup_notifier: Indicate if the EPC device can notify EPF driver on link
 *   up
 * core_init_notifier: Indicate cores that can notify about their
 *   vailability for initialization
 * msi_capable: Indicate if the endpoint function has MSI capability
 * msix_capable: Indicate if the endpoint function has MSI-X capability
 * bar_reserved: Bitmap to indicate reserved BAR unavailable to function
 *   driver
 * bar_fixed_64bit: Bitmap to indicate fixed 64bit BARs
 * bar_fixed_size: Array specifying the size supported by each BAR
 * align: Alignment size required for BAR buffer allocation
 */

struct pci_epc_features_s
{
  unsigned int linkup_notifier : 1;
  unsigned int core_init_notifier : 1;
  unsigned int msi_capable : 1;
  unsigned int msix_capable : 1;
  uint8_t bar_reserved;
  uint8_t bar_fixed_64bit;
  uint64_t bar_fixed_size[PCI_STD_NUM_BARS];
  size_t align;
};

/* struct pci_epc_ops_s - Set of function pointers for performing EPC
 * operations
 *
 * write_header: Ops to populate configuration space header
 * set_bar: Ops to configure the BAR
 * clear_bar: Ops to reset the BAR
 * map_addr: Ops to map CPU address to PCI address
 * unmap_addr: Ops to unmap CPU address and PCI address
 * set_msi: Ops to set the requested number of MSI interrupts in the MSI
 *   capability register
 * get_msi: Ops to get the number of MSI interrupts allocated by the RC from
 *   the MSI capability register
 * set_msix: Ops to set the requested number of MSI-X interrupts in the
 *   MSI-X capability register
 * get_msix: Ops to get the number of MSI-X interrupts allocated by the RC
 *   from the MSI-X capability register
 * raise_irq: Ops to raise a legacy, MSI or MSI-X interrupt
 * map_msi_irq: Ops to map physical address to MSI address and return MSI
 *   data
 * start: Ops to start the PCI link
 * stop: Ops to stop the PCI link
 * get_features: Ops to get the features supported by the EPC
 * dma_xfer: Ops to transfer data between mapped Host memory and device
 *   memory with "System DMA"
 */

struct pci_epc_ops_s
{
  CODE int (*write_header)(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                           FAR struct pci_epf_header_s *hdr);
  CODE int (*set_bar)(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                      FAR struct pci_epf_bar_s *bar);
  CODE void (*clear_bar)(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                         FAR struct pci_epf_bar_s *bar);
  CODE int (*map_addr)(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                       uintptr_t addr, uint64_t pci_addr, size_t size);
  CODE void (*unmap_addr)(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                          uintptr_t addr);
  CODE int (*set_msi)(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                      uint8_t interrupts);
  CODE int (*get_msi)(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno);
  CODE int (*set_msix)(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                       uint16_t interrupts, int barno, uint32_t offset);
  CODE int (*get_msix)(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno);
  CODE int (*raise_irq)(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                        enum pci_epc_irq_type_e type,
                        uint16_t interrupt_num);
  CODE int (*map_msi_irq)(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                          uintptr_t phys_addr, uint8_t interrupt_num,
                          uint32_t entry_size, FAR uint32_t *msi_data,
                          FAR uint32_t *msi_addr_offset);
  CODE int (*start)(FAR struct pci_epc_ctrl_s *epc);
  CODE void (*stop)(FAR struct pci_epc_ctrl_s *epc);
  CODE FAR const struct pci_epc_features_s *
  (*get_features)(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno);
  CODE int (*dma_xfer)(FAR struct pci_epc_ctrl_s *epc, uintptr_t mapped_addr,
                       uintptr_t local_addr, uint32_t size, bool fromdev);
};

/* struct pci_epc_mem_window_s - Address window of the endpoint controller
 *
 * virt_base: Virtual base address of the PCI address window
 * phys_base: Physical base address of the PCI address window
 * size: The size of the PCI address window
 * page_size: Size of each page
 */

struct pci_epc_mem_window_s
{
  FAR void *virt_base;
  uintptr_t phys_base;
  size_t size;
  size_t page_size;
};

/* struct pci_epc_mem_s - Address space of the endpoint controller
 *
 * virt_base: Virtual base address of the PCI address window
 * phys_base: Physical base address of the PCI address window
 * size: The size of the PCI address window
 * page_size: Size of each page
 * bitmap: Bitmap to manage the PCI address space
 * pages: Number of bits representing the address region
 * lock: Mutex to protect bitmap
 */

struct pci_epc_mem_s
{
  FAR void *virt_base;
  uintptr_t phys_base;
  size_t size;
  size_t page_size;

  FAR unsigned long *bitmap;
  int pages;

  /* Mutex to protect against concurrent access for memory allocation */

  mutex_t lock;
};

/* struct pci_epc_ctrl_s - Represents the PCI EPC device
 *
 * epf: List of endpoint functions present in this EPC device
 * ops: Function pointers for performing endpoint operations
 * mem: Array of address space of the endpoint controlle
 * num_windows: Number of mem supported by device
 * max_functions: Max number of functions that can be configured in this EPC
 * node: The node of epc list
 * lock: Mutex to protect pci_epc ops
 * funcno_map: Bitmap to manage physical function number
 * priv: The private data
 * name: The current epc structure name that used to bind the epf
 */

struct pci_epc_ctrl_s
{
  struct list_node epf;
  FAR const struct pci_epc_ops_s *ops;
  FAR struct pci_epc_mem_s *mem;
  unsigned int num_windows;
  uint8_t max_functions;
  struct list_node node;

  /* Mutex to protect against concurrent access of EP controller */

  mutex_t lock;
  unsigned long funcno_map;
  FAR void *priv;
  char name[0];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pci_get_epc
 *
 * Description:
 *   This function is used to get the PCI endpoint controller.
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

FAR struct pci_epc_ctrl_s *pci_get_epc(FAR const char *epc_name);

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
 *   barno        - The starting BAR number from where unreserved BAR should
 * be searched
 *
 * Returned Value:
 *    Return the member of if success, negative if failed
 ****************************************************************************/

int pci_epc_get_next_free_bar(
  FAR const struct pci_epc_features_s *epc_features, int barno);

/****************************************************************************
 * Name: pci_epc_get_first_free_bar
 *
 * Description:
 *   Helper to get first unreserved BAR.
 *
 *   Invoke to get the first unreserved BAR that can be used by the endpoint
 * function. For any incorrect value in bar_reserved return '0'.
 *
 * Input Parameters:
 *   epc_features - pci_epc_features_s structure that holds the reserved bar
 * bitmap
 *
 * Returned Value:
 *    Return the member if success, negative if failed
 ****************************************************************************/

#define pci_epc_get_first_free_bar(f) pci_epc_get_next_free_bar(f, 0)

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
pci_epc_get_features(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno);

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

void pci_epc_stop(FAR struct pci_epc_ctrl_s *epc);

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

int pci_epc_start(FAR struct pci_epc_ctrl_s *epc);

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
                      enum pci_epc_irq_type_e type, uint16_t interrupt_num);

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
                        FAR uint32_t *msi_addr_offset);

/****************************************************************************
 * Name: pci_epc_get_msi
 *
 * Description:
 *   Get the number of MSI interrupt numbers allocated.
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

int pci_epc_get_msi(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno);

/****************************************************************************
 * Name: pci_epc_set_msi
 *
 * Description:
 *   Set the number of MSI interrupt numbers required.
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
                    uint8_t interrupts);

/****************************************************************************
 * Name: pci_epc_get_msix
 *
 * Description:
 *   Get the number of MSI-X interrupt numbers allocated.
 *
 *   Invoke to get the number of MSI-X interrupts allocated by the RC.
 *
 * Input Parameters:
 *   epc     - The EPC device to which MSI-X interrupts was requested
 *   funcno  - The physical endpoint function number in the EPC device
 *
 * Returned Value:
 *    Return 0 if success, negative if failed
 ****************************************************************************/

int pci_epc_get_msix(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno);

/****************************************************************************
 * Name: pci_epc_set_msix
 *
 * Description:
 *   Set the number of MSI-X interrupt numbers required.
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
 *    Return interrupt + 1 number if success, negative if failed
 ****************************************************************************/

int pci_epc_set_msix(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                     uint16_t interrupts, int barno, uint32_t offset);

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
                        uintptr_t phys_addr);

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
                     uintptr_t phys_addr, uint64_t pci_addr, size_t size);

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
                       FAR struct pci_epf_bar_s *bar);

/****************************************************************************
 * Name: pci_epc_set_bar
 *
 * Description:
 *   Configure BAR in order for host to assign PCI addr space.
 *
 *   Invoke to configure the BAR of the endpoint device.
 *
 * Input Parameters:
 *   epc    - The EPC device on which BAR has to be configured
 *   funcno - The physical endpoint function number in the EPC device
 *   bar    - The struct pci_epf_bar_s that contains the BAR information
 *
 * Returned Value:
 *    Return 0 if success, negative if failed
 ****************************************************************************/

int pci_epc_set_bar(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                    FAR struct pci_epf_bar_s *bar);

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
                         FAR struct pci_epf_header_s *header);

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
                    FAR struct pci_epf_device_s *epf);

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
                        FAR struct pci_epf_device_s *epf);

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

void pci_epc_linkup(FAR struct pci_epc_ctrl_s *epc);

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

void pci_epc_linkdown(FAR struct pci_epc_ctrl_s *epc);

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

void pci_epc_init_notify(FAR struct pci_epc_ctrl_s *epc);

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

void pci_epc_bme_notify(FAR struct pci_epc_ctrl_s *epc);

/****************************************************************************
 * Name: pci_epc_create
 *
 * Description:
 *   This function is used to destroy the EPC device.
 *
 *   Invoke to create a new EPC device and add it to pci_epc class.
 *
 * Input Parameters:
 *   name - EPC name strings
 *   priv - The epc priv data
 *   ops  - Function pointers for performing EPC operations
 *
 * Returned Value:
 *   Return struct pci_epc_ctrl_s * if success, NULL if failed.
 ****************************************************************************/

FAR struct pci_epc_ctrl_s *
pci_epc_create(FAR const char *name, FAR void *priv,
               FAR const struct pci_epc_ops_s *ops);

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

void pci_epc_destroy(FAR struct pci_epc_ctrl_s *epc);

/****************************************************************************
 * Name: pci_epc_mem_multi_init
 *
 * Description:
 *   This function is used to initialize the pci_epc_mem_s structure.
 *
 *   Invoke to initialize the pci_epc_mem_s structure used by the
 * endpoint functions to allocate mapped PCI address.
 *
 * Input Parameters:
 *   epc         - The EPC device that invoked pci_epc_mem_init
 *   windows     - Pointer to windows supported by the device
 *   num_windows - Number of windows device supports
 *
 * Returned Value:
 *   0 if success, negative if failed
 ****************************************************************************/

int pci_epc_mem_multi_init(FAR struct pci_epc_ctrl_s *epc,
                           FAR const struct pci_epc_mem_window_s *windows,
                           unsigned int num_windows);

/****************************************************************************
 * Name: pci_epc_mem_init
 *
 * Description:
 *   This function is used to initialize the PCI endpoint controller memory
 * space.
 *
 * Input Parameters:
 *   epc       - PCI EPC device
 *   virt      - The virtual addr
 *   phys      - The physical base address of the PCI address window
 *   size      - The PCI window size
 *   page_size - Size of each window page
 *
 * Returned Value:
 *   0 if success, negative if failed
 ****************************************************************************/

int pci_epc_mem_init(FAR struct pci_epc_ctrl_s *epc, FAR void *virt,
                     uintptr_t phys, size_t size, size_t page_size);

/****************************************************************************
 * Name: pci_epc_mem_exit
 *
 * Description:
 *   This function is used to cleanup the pci_epc_mem_s structure.
 *
 * Invoke to cleanup the pci_epc_mem_s structure allocated in
 * pci_epc_mem_init().
 *
 * Input Parameters:
 *   epc  - The EPC device that invoked pci_epc_mem_exit
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void pci_epc_mem_exit(FAR struct pci_epc_ctrl_s *epc);

/****************************************************************************
 * Name: pci_epc_mem_alloc_addr
 *
 * Description:
 *   Allocate memory address from EPC addr space
 *
 *   Invoke to allocate memory address from the EPC address space. This
 * is usually done to map the remote RC address into the local system.
 *
 * Input Parameters:
 *   epc   - The EPC device on which memory has to be allocated
 *   phys  - The Physical addr
 *   size  - The size of the address space that has to be allocated
 *
 * Returned Value:
 *   The memory address alloced if success, NULL if failed
 ****************************************************************************/

FAR void *pci_epc_mem_alloc_addr(FAR struct pci_epc_ctrl_s *epc,
                                 FAR uintptr_t *phys, size_t size);

/****************************************************************************
 * Name: pci_epc_mem_free_addr
 *
 * Description:
 *   Free the allocated memory address.
 *
 *   Invoke to free the memory allocated using pci_epc_mem_alloc_addr.
 *
 * Input Parameters:
 *   epc       - The EPC device on which memory was allocated
 *   phys_addr - The allocated physical address
 *   size      - The size of the allocated address space
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void pci_epc_mem_free_addr(FAR struct pci_epc_ctrl_s *epc,
                           uintptr_t phys_addr, size_t size);

#endif /* __INCLUDE_NUTTX_PCI_PCI_EPC_H */
