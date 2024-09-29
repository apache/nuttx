/****************************************************************************
 * include/nuttx/pci/pci.h
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

#ifndef __INCLUDE_NUTTX_PCI_PCI_H
#define __INCLUDE_NUTTX_PCI_PCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/list.h>
#include <nuttx/pci/pci_ids.h>
#include <nuttx/pci/pci_regs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PCI_RESOURCE_IO       0x00000001
#define PCI_RESOURCE_MEM      0x00000002
#define PCI_RESOURCE_MEM_64   0x00000004
#define PCI_RESOURCE_PREFETCH 0x00000008 /* No side effects */

/* The PCI interface treats multi-function devices as independent
 * devices.  The slot/function address of each device is encoded
 * in a single byte as follows:
 *
 *  7:3 = slot
 *  2:0 = function
 */

#define PCI_DEVFN(slot, func) ((((slot) & 0x1f) << 3) | ((func) & 0x07))
#define PCI_SLOT(devfn)       (((devfn) >> 3) & 0x1f)
#define PCI_FUNC(devfn)       ((devfn) & 0x07)

#define PCI_ANY_ID (uint16_t)(~0)

/* PCI_DEFINE_DEVICE_TABLE - macro used to describe a pci device table
 * table: device table name
 *
 * This macro is used to create a struct pci_device_id_s array (a device
 * table)in a generic manner.
 */

#define PCI_DEFINE_DEVICE_TABLE(table) \
  static const struct pci_device_id_s table[]

/* PCI_DEVICE - macro used to describe a specific pci device
 * vend: the 16 bit PCI Vendor ID
 * dev: the 16 bit PCI Device ID
 *
 * This macro is used to create a struct pci_device_id_s that matches a
 * specific device.  The subvendor and subdevice fields will be set to
 * PCI_ANY_ID.
 */

#define PCI_DEVICE(vend, dev) \
  .vendor = (vend), .device = (dev), \
  .subvendor = PCI_ANY_ID, .subdevice = PCI_ANY_ID

/* PCI_DEVICE_SUB - macro used to describe a specific PCI device with
 * subsystem
 *
 * vend: the 16 bit PCI Vendor ID
 * dev: the 16 bit PCI Device ID
 * subvend: the 16 bit PCI Subvendor ID
 * subdev: the 16 bit PCI Subdevice ID
 *
 * This macro is used to create a struct pci_device_id_s that matches a
 * specific device with subsystem information.
 */

#define PCI_DEVICE_SUB(vend, dev, subvend, subdev) \
  .vendor = (vend), .device = (dev), \
  .subvendor = (subvend), .subdevice = (subdev)

/* PCI_DEVICE_CLASS - macro used to describe a specific pci device class
 * dev_class: the class, subclass, prog-if triple for this device
 * dev_class_mask: the class mask for this device
 *
 * This macro is used to create a struct pci_device_id_s that matches a
 * specific PCI class.  The vendor, device, subvendor, and subdevice
 * fields will be set to PCI_ANY_ID.
 */

#define PCI_DEVICE_CLASS(dev_class, dev_class_mask) \
  .class = (dev_class), .class_mask = (dev_class_mask), \
  .vendor = PCI_ANY_ID, .device = PCI_ANY_ID, \
  .subvendor = PCI_ANY_ID, .subdevice = PCI_ANY_ID

/* PCI_VDEVICE - macro used to describe a specific pci device in short form
 * vend: the vendor name
 * dev: the 16 bit PCI Device ID
 *
 * This macro is used to create a struct pci_device_id_s that matches a
 * specific PCI device.  The subvendor, and subdevice fields will be set
 * to PCI_ANY_ID. The macro allows the next field to follow as the device
 * private data.
 */

#define PCI_VDEVICE(vend, dev) \
  .vendor = PCI_VENDOR_ID_##vend, .device = (dev), \
  .subvendor = PCI_ANY_ID, .subdevice = PCI_ANY_ID

/* These helpers provide future and backwards compatibility
 * for accessing popular PCI BAR info
 */

#define pci_resource_size(res) \
  (((res)->start == 0 && (res)->end == 0) ? 0 : \
   ((res)->end - (res)->start + 1))

#define pci_resource_start(dev, bar) ((dev)->resource[(bar)].start)
#define pci_resource_end(dev, bar)   ((dev)->resource[(bar)].end)
#define pci_resource_flags(dev, bar) ((dev)->resource[(bar)].flags)
#define pci_resource_len(dev, bar)   pci_resource_size(&((dev)->resource[(bar)]))

#define pci_read_config_byte(dev, where, val) \
  pci_bus_read_config_byte((dev)->bus, (dev)->devfn, where, val)

#define pci_read_config_word(dev, where, val) \
  pci_bus_read_config_word((dev)->bus, (dev)->devfn, where, val)

#define pci_read_config_dword(dev, where, val) \
  pci_bus_read_config_dword((dev)->bus, (dev)->devfn, where, val)

#define pci_write_config_byte(dev, where, val) \
  pci_bus_write_config_byte((dev)->bus, (dev)->devfn, where, val)

#define pci_write_config_word(dev, where, val) \
  pci_bus_write_config_word((dev)->bus, (dev)->devfn, where, val)

#define pci_write_config_dword(dev, where, val) \
  pci_bus_write_config_dword((dev)->bus, (dev)->devfn, where, val)

#define pci_read_io_byte(dev, addr, val) \
  pci_bus_read_io_byte((dev)->bus, addr, val)

#define pci_read_io_word(dev, addr, val) \
  pci_bus_read_io_word((dev)->bus, addr, val)

#define pci_read_io_dword(dev, addr, val) \
  pci_bus_read_io_dword((dev)->bus, addr, val)

#define pci_read_io_qword(dev, addr, val) \
  do \
    { \
      uint32_t valhi; \
      uint32_t vallo; \
      pci_bus_read_io_dword((dev)->bus, addr, &vallo); \
      pci_bus_read_io_dword((dev)->bus, (uintptr_t)(addr) + sizeof(uint32_t), &valhi); \
      *(val) = ((uint64_t)valhi << 32) | (uint64_t)vallo; \
    } \
  while (0)

#define pci_write_io_byte(dev, addr, val) \
  pci_bus_write_io_byte((dev)->bus, addr, val)

#define pci_write_io_word(dev, addr, val) \
  pci_bus_write_io_word((dev)->bus, addr, val)

#define pci_write_io_dword(dev, addr, val) \
  pci_bus_write_io_dword((dev)->bus, addr, val)

#define pci_write_io_qword(dev, addr, val) \
  do \
    { \
      pci_bus_write_io_dword((dev)->bus, addr, (uint32_t)(val)); \
      pci_bus_write_io_dword((dev)->bus, (uintptr_t)(addr) + sizeof(uint32_t), (val) >> 32); \
    } \
  while (0)

#define pci_write_mmio_byte(dev, addr, val)  \
  (*((FAR volatile uint8_t *)(addr))) = val

#define pci_write_mmio_word(dev, addr, val)  \
  (*((FAR volatile uint16_t *)(addr))) = val

#define pci_write_mmio_dword(dev, addr, val)  \
  (*((FAR volatile uint32_t *)(addr))) = val

#define pci_read_mmio_byte(dev, addr, val)    \
  (*val) = *((FAR volatile uint8_t *)(addr))

#define pci_read_mmio_word(dev, addr, val)    \
  (*val) = *((FAR volatile uint16_t *)(addr))

#define pci_read_mmio_dword(dev, addr, val)   \
  (*val) = *((FAR volatile uint32_t *)(addr))

#define pci_map_region(dev, start, size) pci_bus_map_region((dev)->bus, start, size)

#define pci_is_bridge(dev) ((dev)->hdr_type == PCI_HEADER_TYPE_BRIDGE || \
                            (dev)->hdr_type == PCI_HEADER_TYPE_CARDBUS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct pci_resource_s
{
  uintptr_t start;
  uintptr_t end;
  unsigned int flags;
};

/* For PCI devices, the region numbers are assigned this way: */

enum
{
  /* #0-5: Standard PCI resources */

  PCI_STD_RESOURCES,
  PCI_STD_RESOURCE_END = 5,

  /* #6: Expansion ROM resource */

  PCI_ROM_RESOURCE,

  /* Total resources associated with a PCI device */

  PCI_NUM_RESOURCES,
};

/* The pci_device_s structure is used to describe PCI devices. */

struct pci_device_id_s
{
  uint16_t vendor;    /* Vendor id */
  uint16_t device;    /* Device id */
  uint16_t subvendor; /* Sub vendor id */
  uint16_t subdevice; /* Sub device id */
  uint32_t class;     /* (Class, subclass, prog-if) triplet */
  uint32_t class_mask;
  uintptr_t driver_data;
};

struct pci_device_s
{
  struct list_node node;
  struct list_node bus_list;         /* Node in per-bus list */
  FAR struct pci_bus_s *bus;         /* Bus this device is on */
  FAR struct pci_bus_s *subordinate; /* Bus this device bridges to */

  uint32_t devfn;  /* Encoded device & function index */
  uint16_t vendor; /* Vendor id */
  uint16_t device; /* Device id */
  uint16_t subsystem_vendor;
  uint16_t subsystem_device;
  uint32_t class;   /* 3 bytes: (base,sub,prog-if) */
  uint8_t revision; /* PCI revision, low byte of class word */
  uint8_t hdr_type; /* PCI header type (`multi' flag masked out) */

  /* I/O and memory regions + expansion ROMs */

  struct pci_resource_s resource[PCI_NUM_RESOURCES];

  FAR const struct pci_device_id_s *id;
  FAR struct pci_driver_s *drv;
  FAR void *priv; /* Used by pci driver */
};

struct pci_bus_s
{
  FAR struct pci_controller_s *ctrl; /* Associated host controller */
  FAR struct pci_bus_s *parent_bus;  /* Parent bus */

  struct list_node node;     /* Node in list of buses */
  struct list_node children; /* List of child buses */
  struct list_node devices;  /* List of devices on this bus */

  uint8_t number; /* Bus number */
};

/* Low-level architecture-dependent routines */

struct pci_ops_s
{
  CODE int (*read)(FAR struct pci_bus_s *bus, unsigned int devfn, int where,
                   int size, FAR uint32_t *val);
  CODE int (*write)(FAR struct pci_bus_s *bus, unsigned int devfn, int where,
                    int size, uint32_t val);

  /* Return memory address for pci resource */

  CODE uintptr_t (*map)(FAR struct pci_bus_s *bus, uintptr_t start,
                        uintptr_t end);
  CODE int (*read_io)(FAR struct pci_bus_s *bus, uintptr_t addr,
                      int size, FAR uint32_t *val);
  CODE int (*write_io)(FAR struct pci_bus_s *bus, uintptr_t addr,
                       int size, uint32_t val);

  /* Get interrupt number associated with a given INTx line */

  CODE int (*get_irq)(FAR struct pci_bus_s *bus, uint32_t devfn,
                      uint8_t line, uint8_t pin);

  /* Allocate interrupt for MSI/MSI-X */

  CODE int (*alloc_irq)(FAR struct pci_bus_s *bus, FAR int *irq, int num);

  CODE void (*release_irq)(FAR struct pci_bus_s *bus, FAR int *irq, int num);

  /* Connect interrupt for MSI/MSI-X */

  CODE int (*connect_irq)(FAR struct pci_bus_s *bus, FAR int *irq,
                          int num, FAR uintptr_t *mar, FAR uint32_t *mdr);
};

/* Each pci channel is a top-level PCI bus seem by CPU.  A machine with
 * multiple PCI channels may have multiple PCI host controllers or a
 * single controller supporting multiple channels.
 */

struct pci_controller_s
{
  struct pci_resource_s io;
  struct pci_resource_s mem;
  struct pci_resource_s mem_pref;
  FAR const struct pci_ops_s *ops;

  FAR struct pci_bus_s *bus;
  struct list_node node;
  uint8_t busno;
};

struct pci_driver_s
{
  FAR const struct pci_device_id_s *id_table;

  /* New device inserted */

  CODE int (*probe)(FAR struct pci_device_s *dev);

  /* Device removed (NULL if not a hot-plug capable driver) */

  CODE void (*remove)(FAR struct pci_device_s *dev);

  struct list_node node;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pci_bus_read_config
 *
 * Description:
 *  read pci device config space
 *
 * Input Parameters:
 *   bus   - The PCI device to belong to
 *   devfn - The PCI device number and function number
 *   where - The register address
 *   size  - The length data
 *   val   - The data buf
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_bus_read_config(FAR struct pci_bus_s *bus,
                        unsigned int devfn, int where,
                        int size, FAR uint32_t *val);

/****************************************************************************
 * Name: pci_bus_read_config_xxx
 *
 * Description:
 *  Read pci device config space
 *
 * Input Parameters:
 *   bus   - The PCI device to belong to
 *   devfn - The PCI device number and function number
 *   where - The register address
 *   val   - The data buf
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_bus_read_config_byte(FAR struct pci_bus_s *bus,
                             unsigned int devfn, int where,
                             FAR uint8_t *val);
int pci_bus_read_config_word(FAR struct pci_bus_s *bus,
                              unsigned int devfn, int where,
                              FAR uint16_t *val);
int pci_bus_read_config_dword(FAR struct pci_bus_s *bus,
                              unsigned int devfn, int where,
                              FAR uint32_t *val);

/****************************************************************************
 * Name: pci_bus_write_config
 *
 * Description:
 *  Write pci device config space
 *
 * Input Parameters:
 *   bus   - The PCI device to belong to
 *   devfn - The PCI device number and function number
 *   where - The register address
 *   size  - The length data
 *   val   - The data
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_bus_write_config(FAR struct pci_bus_s *bus,
                         unsigned int devfn, int where,
                         int size, uint32_t val);

/****************************************************************************
 * Name: pci_bus_write_config_xxx
 *
 * Description:
 *  Write pci device config space
 *
 * Input Parameters:
 *   bus   - The PCI device to belong to
 *   devfn - The PCI device number and function number
 *   where - The register address
 *   val   - The data
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_bus_write_config_byte(FAR struct pci_bus_s *bus,
                              unsigned int devfn, int where,
                              uint8_t val);
int pci_bus_write_config_word(FAR struct pci_bus_s *bus,
                              unsigned int devfn, int where,
                              uint16_t val);
int pci_bus_write_config_dword(FAR struct pci_bus_s *bus,
                               unsigned int devfn, int where,
                               uint32_t val);

/****************************************************************************
 * Name: pci_bus_read_io
 *
 * Description:
 *  Read pci device io space
 *
 * Input Parameters:
 *   bus   - The PCI device belong to
 *   addr  - The address to read
 *   size  - The data length
 *   val   - The data buffer
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_bus_read_io(FAR struct pci_bus_s *bus, uintptr_t addr,
                    int size, FAR uint32_t *val);

/****************************************************************************
 * Name: pci_bus_read_io_xxx
 *
 * Description:
 *  Read pci device io space
 *
 * Input Parameters:
 *   bus   - The PCI device belong to
 *   where - The address to read
 *   val   - The data buffer
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_bus_read_io_byte(FAR struct pci_bus_s *bus, uintptr_t where,
                         FAR uint8_t *val);
int pci_bus_read_io_word(FAR struct pci_bus_s *bus, uintptr_t where,
                          FAR uint16_t *val);
int pci_bus_read_io_dword(FAR struct pci_bus_s *bus, uintptr_t where,
                          FAR uint32_t *val);

/****************************************************************************
 * Name: pci_bus_write_io
 *
 * Description:
 *  Write pci device io space
 *
 * Input Parameters:
 *   bus   - The PCI device belong to
 *   addr  - The address to write
 *   size  - The data length
 *   val   - The data
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_bus_write_io(FAR struct pci_bus_s *bus, uintptr_t addr,
                     int size, uint32_t val);

/****************************************************************************
 * Name: pci_bus_write_io_xxx
 *
 * Description:
 *  Write pci device io space
 *
 * Input Parameters:
 *   bus   - The PCI device belong to
 *   where - The address to write
 *   val   - The data
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_bus_write_io_byte(FAR struct pci_bus_s *bus, uintptr_t where,
                          uint8_t value);
int pci_bus_write_io_word(FAR struct pci_bus_s *bus, uintptr_t where,
                          uint16_t value);
int pci_bus_write_io_dword(FAR struct pci_bus_s *bus, uintptr_t where,
                          uint32_t value);

/****************************************************************************
 * Name: pci_set_master
 *
 * Description:
 *   Enables bus-mastering for device
 *
 * Input Parameters:
 *   dev - The PCI device to enable
 *
 ****************************************************************************/

void pci_set_master(FAR struct pci_device_s *dev);

/****************************************************************************
 * Name: pci_clear_master
 *
 * Description:
 *   Disables bus-mastering for device
 *
 * Input Parameters:
 *   dev - The PCI device to disable
 *
 ****************************************************************************/

void pci_clear_master(FAR struct pci_device_s *dev);

/****************************************************************************
 * Name: pci_enable_device
 *
 * Description:
 *   Initialize device before it's used by a driver by setting command
 *   register.
 *
 * Input Parameters:
 *   dev - PCI device to be initialized
 *pci_bus_ops_s
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_enable_device(FAR struct pci_device_s *dev);

/****************************************************************************
 * Name: pci_disable_device
 *
 * Description:
 *   Disable pci device before it's unused by a driver by setting command
 *   register.
 *
 * Input Parameters:
 *   dev - PCI device to be Disable
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_disable_device(FAR struct pci_device_s *dev);

/****************************************************************************
 * Name: pci_select_bars
 *
 * Description:
 *   Make BAR mask from the type of resource
 *   This helper routine makes bar mask from the type of resource.
 *
 * Input Parameters:
 *   dev   - The PCI device for which BAR mask is made
 *   flags - Resource type mask to be selected
 *
 ****************************************************************************/

int pci_select_bars(FAR struct pci_device_s *dev, unsigned int flags);

/****************************************************************************
 * Name: pci_bus_map_region
 *
 * Description:
 *   Create a virtual mapping for a address.
 *
 *   Using this function you will get an virtual address.
 *   These functions hide the details if this is a MMIO or PIO address
 *   space and will just do what you expect from them in the correct way.
 *
 * Input Parameters:
 *   bus   - PCI bus
 *   start - The address base
 *   size  - The length of the address
 *
 * Returned Value:
 *  Virtual address or zero if failed
 ****************************************************************************/

FAR void *
pci_bus_map_region(FAR struct pci_bus_s *bus, uintptr_t start, size_t size);

/****************************************************************************
 * Name: pci_map_bar_region
 *
 * Description:
 *   Create a virtual mapping for a PCI BAR REGION.
 *
 *   Using this function you will get an address to your device BAR region.
 *   These functions hide the details if this is a MMIO or PIO address
 *   space and will just do what you expect from them in the correct way.
 *
 * Input Parameters:
 *   dev - PCI device that owns the BAR
 *   bar - BAR number
 *   offset - BAR region offset
 *   length - BAR region length
 *
 * Returned Value:
 *  IO address or zero if failed
 ****************************************************************************/

FAR void *pci_map_bar_region(FAR struct pci_device_s *dev, int bar,
                             uintptr_t offset, size_t length);

/****************************************************************************
 * Name: pci_map_bar
 *
 * Description:
 *   Create a virtual mapping for a PCI BAR.
 *
 *   Using this function you will get an address to your device BAR.
 *   These functions hide the details if this is a MMIO or PIO address
 *   space and will just do what you expect from them in the correct way.
 *
 * Input Parameters:
 *   dev - PCI device that owns the BAR
 *   bar - BAR number
 *
 * Returned Value:
 *  IO address or zero if failed
 *
 ****************************************************************************/

FAR void *pci_map_bar(FAR struct pci_device_s *dev, int bar);

/****************************************************************************
 * Name: pci_find_capability
 *
 * Description:
 *   Query for devices' capabilities
 *
 *   Tell if a device supports a given PCI capability.
 *
 * Input Parameters:
 *   dev - PCI device to query
 *   cap - Capability code
 *
 * Returned Value:
 *   Returns the address of the requested capability structure within the
 *   device's PCI configuration space or 0 in case the device does not
 *   support it.
 *
 ****************************************************************************/

uint8_t pci_find_capability(FAR struct pci_device_s *dev, int cap);

/****************************************************************************
 * Name: pci_find_next_capability
 *
 * Description:
 *   To find the next capability.
 *
 * Input Parameters:
 *   dev - The PCI device to find capbilities
 *   pos - List node
 *   cap - Value of capabilities
 *
 * Returned Value:
 *   Return the capability data
 *
 ****************************************************************************/

uint8_t pci_find_next_capability(FAR struct pci_device_s *dev, uint8_t pos,
                                 int cap);

/****************************************************************************
 * Name: pci_stat_line
 *
 * Description:
 *  Determine if the interrupt line is active for a given device
 *
 * Input Parameters:
 *   dev - device
 *
 * Return value:
 *   True if interrupt is active
 *
 ****************************************************************************/

bool pci_stat_line(FAR struct pci_device_s *dev);

/****************************************************************************
 * Name: pci_get_irq
 *
 * Description:
 *  Get interrupt number associated with a device PCI interrupt line
 *
 * Input Parameters:
 *   dev - PCI device
 *
 * Return value:
 *   Return interrupt number associated with a given INTx.
 *
 ****************************************************************************/

int pci_get_irq(FAR struct pci_device_s *dev);

/****************************************************************************
 * Name: pci_alloc_irq
 *
 * Description:
 *   Allocate MSI or MSI-X vectors
 *
 * Input Parameters:
 *   dev - PCI device
 *   irq - allocated vectors
 *   num - number of vectors
 *
 * Return value:
 *   Return the number of allocated vectors on succes or negative errno
 *   on failure.
 *
 ****************************************************************************/

int pci_alloc_irq(FAR struct pci_device_s *dev, FAR int *irq, int num);

/****************************************************************************
 * Name: pci_release_irq
 *
 * Description:
 *   Release MSI or MSI-X vectors
 *
 * Input Parameters:
 *   dev - PCI device
 *   irq - allocated vectors
 *   num - number of vectors
 *
 * Return value:
 *   None
 *
 ****************************************************************************/

void pci_release_irq(FAR struct pci_device_s *dev, FAR int *irq, int num);

/****************************************************************************
 * Name: pci_connect_irq
 *
 * Description:
 *   Connect MSI or MSI-X if available.
 *
 * Input Parameters:
 *   dev - PCI device
 *   irq - allocated vectors
 *   num - number of vectors
 *
 * Return value:
 *   Return -ENOSETUP if MSI/MSI-X not available. Return OK on success.
 *
 ****************************************************************************/

int pci_connect_irq(FAR struct pci_device_s *dev, FAR int *irq, int num);

/****************************************************************************
 * Name: pci_register_driver
 *
 * Description:
 *   To register a PCI driver
 *
 * Input Parameters:
 *   drv - PCI driver
 *
 * Returned Value:
 *   Failed if return a negative value, otherwise success
 *
 ****************************************************************************/

int pci_register_driver(FAR struct pci_driver_s *drv);

/****************************************************************************
 * Name: pci_unregister_driver
 *
 * Description:
 *   To unregister a PCI driver
 *
 * Input Parameters:
 *   drv - PCI driver
 *
 * Returned Value:
 *   Failed if return a negative value, otherwise success
 *
 ****************************************************************************/

int pci_unregister_driver(FAR struct pci_driver_s *drv);

/****************************************************************************
 * Name: pci_register_device
 *
 * Description:
 *   To register a PCI device
 *
 * Input Parameters:
 *   dev - PCI device
 *
 * Returned Value:
 *   Failed if return a negative value, otherwise success
 *
 ****************************************************************************/

int pci_register_device(FAR struct pci_device_s *dev);

/****************************************************************************
 * Name: pci_unregister_device
 *
 * Description:
 *   To unregister a PCI device
 *
 * Input Parameters:
 *   dev - PCI device
 *
 * Returned Value:
 *   Failed if return a negative value, otherwise success
 *
 ****************************************************************************/

int pci_unregister_device(FAR struct pci_device_s *dev);

/****************************************************************************
 * Name: pci_register_controller
 *
 * Description:
 *   Start pci bridge enumeration process, and register pci device.
 *
 * Input Parameters:
 *   ctrl - PCI ctrl to register
 *
 ****************************************************************************/

int pci_register_controller(FAR struct pci_controller_s *ctrl);

/****************************************************************************
 * Name: pci_bus_find_capability
 *
 * Description:
 *   Query for devices' capabilities
 *
 *   Tell if a device supports a given PCI capability.
 *
 * Input Parameters:
 *   bus    - PCI device bus belong to
 *   devfn  - PCI device number and function number
 *   cap    - Capability code
 *
 * Returned Value:
 *   Returns the address of the requested capability structure within the
 *   device's PCI configuration space or 0 in case the device does not
 *   support it.
 *
 ****************************************************************************/

uint8_t pci_bus_find_capability(FAR struct pci_bus_s *bus,
                                unsigned int devfn, int cap);

/****************************************************************************
 * Name: pci_bus_read_config_byte
 *
 * Description:
 *   Read data from specify position with byte size
 *
 * Input Parameters:
 *   bus   - PCI bus
 *   devfn - BDF
 *   where - Pos ID
 *   val   - Value
 *
 * Return value
 *   Return 0 if success, otherwise Error values
 *
 ****************************************************************************/

int pci_bus_read_config_byte(FAR struct pci_bus_s *bus, unsigned int devfn,
                             int where, FAR uint8_t *val);

/****************************************************************************
 * Name: pci_bus_read_config_word
 *
 * Description:
 *   Read data from specify position with word size
 *
 * Input Parameters:
 *   bus   - PCI bus
 *   devfn - BDF
 *   where - Pos ID
 *   val   - Value
 *
 * Return value
 *   Return 0 if success, otherwise Error values
 *
 ****************************************************************************/

int pci_bus_read_config_word(FAR struct pci_bus_s *bus, unsigned int devfn,
                             int where, FAR uint16_t *val);

/****************************************************************************
 * Name: pci_bus_read_config_dword
 *
 * Description:
 *   Read data from specify position with dword size
 *
 * Input Parameters:
 *   bus   - PCI bus
 *   devfn - BDF
 *   where - Pos ID
 *   val   - Value
 *
 * Return value
 *   Return 0 if success, otherwise Error values
 *
 ****************************************************************************/

int pci_bus_read_config_dword(FAR struct pci_bus_s *bus, unsigned int devfn,
                              int where, FAR uint32_t *val);

/****************************************************************************
 * Name: pci_bus_write_config_byte
 *
 * Description:
 *   Write data to specify reg with byte size
 *
 * Input Parameters:
 *   bus   - PCI bus
 *   devfn - BDF
 *   where - Pos ID
 *   val   - Value
 *
 * Return value
 *   Return 0 if success, otherwise Error values
 *
 ****************************************************************************/

int pci_bus_write_config_byte(FAR struct pci_bus_s *bus, unsigned int devfn,
                              int where, uint8_t val);

/****************************************************************************
 * Name: pci_bus_write_config_word
 *
 * Description:
 *   Write data to specify reg with word size
 *
 * Input Parameters:
 *   bus   - PCI bus
 *   devfn - BDF
 *   where - Pos ID
 *   val   - Value
 *
 * Return value
 *   Return 0 if success, otherwise Error values
 *
 ****************************************************************************/

int pci_bus_write_config_word(FAR struct pci_bus_s *bus, unsigned int devfn,
                              int where, uint16_t val);

/****************************************************************************
 * Name: pci_bus_write_config_dword
 *
 * Description:
 *   Write data to specify reg with dword size
 *
 * Input Parameters:
 *   bus   - PCI bus
 *   devfn - BDF
 *   where - Pos ID
 *   val   - Value
 *
 * Return value
 *   Return 0 if success, otherwise Error values
 *
 ****************************************************************************/

int pci_bus_write_config_dword(FAR struct pci_bus_s *bus, unsigned int devfn,
                               int where, uint32_t val);

/****************************************************************************
 * Name: pci_find_device_from_bus
 *
 * Description:
 *   To find a PCI device from the bus
 *
 * Input Parameters:
 *   bus   - pci bus
 *   busno - bus number
 *   devfn - device number and function number
 *
 * Returned Value:
 *   Failed if return NULL, otherwise return pci devices
 *
 ****************************************************************************/

FAR struct pci_device_s *
pci_find_device_from_bus(FAR struct pci_bus_s *bus, uint8_t busno,
                         unsigned int devfn);

/****************************************************************************
 * Name: pci_register_drivers
 *
 * Description:
 *   Register all the pci drivers to pci bus
 *
 ****************************************************************************/

int pci_register_drivers(void);

#endif /* __INCLUDE_NUTTX_PCI_PCI_H */
