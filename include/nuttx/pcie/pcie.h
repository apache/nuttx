/****************************************************************************
 * include/nuttx/pcie/pcie.h
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

#ifndef __INCLUDE_NUTTX_PCIE_PCIE_H
#define __INCLUDE_NUTTX_PCIE_PCIE_H

#ifdef CONFIG_PCIE

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PCI_CFG_VENDOR_ID       0x000
#define PCI_CFG_DEVICE_ID       0x002
#define PCI_CFG_COMMAND         0x004
# define PCI_CMD_IO             (1 << 0)
# define PCI_CMD_MEM            (1 << 1)
# define PCI_CMD_MASTER         (1 << 2)
# define PCI_CMD_INTX_OFF       (1 << 10)
#define PCI_CFG_STATUS          0x006
# define PCI_STS_INT            (1 << 3)
# define PCI_STS_CAPS           (1 << 4)
#define PCI_CFG_REVERSION       0x008
#define PCI_CFG_BAR             0x010
# define PCI_BAR_IO             0x1
# define PCI_BAR_1M             0x2
# define PCI_BAR_64BIT          0x4
#define PCI_CFG_CAP_PTR         0x034

#define PCI_ID_ANY              0xffff
#define PCI_DEV_CLASS_OTHER     0xff

#define PCI_CAP_PM              0x01

#define PCI_CAP_MSI             0x05
# define PCI_MSI_MCR            0x02
# define PCI_MSI_MCR_SIZE       2
# define PCI_MSI_MCR_EN         (1 << 0)
# define PCI_MSI_MCR_64         (1 << 7)
# define PCI_MSI_MAR            0x04
# define PCI_MSI_MAR_SIZE       4
# define PCI_MSI_MDR            0x08
# define PCI_MSI_MDR_SIZE       2
# define PCI_MSI_MAR64_HI       0x08
# define PCI_MSI_MAR64_HI_SIZE  4
# define PCI_MSI_MDR64          0x0c
# define PCI_MSI_MDR64_SIZE     2
# define PCI_MSI_APIC_ID_OFFSET 0xc

#define PCI_CAP_MSIX            0x11
# define PCI_MSIX_MCR           0x02
# define PCI_MSIX_MCR_SIZE      2
# define PCI_MSIX_MCR_EN        (1 << 15)
# define PCI_MSIX_MCR_FMASK     0x4000
# define PCI_MSIX_MCR_TBL_MASK  0x03ff
# define PCI_MSIX_TBL           0x04
# define PCI_MSIX_TBL_SIZE      4
# define PCI_MSIX_PBA           0x08
# define PCI_MSIX_PBA_SIZE      4
# define PCI_MSIX_BIR_MASK      0x07
# define PCI_MSIX_TBL_ENTRY_SIZE 0x10
# define PCI_MSIX_TBL_LO_ADDR   0x0
# define PCI_MSIX_TBL_HI_ADDR   0x4
# define PCI_MSIX_TBL_MSG_DATA  0x8
# define PCI_MSIX_TBL_VEC_CTL   0xc
# define PCI_MSIX_APIC_ID_OFFSET 0xc

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The PCIE driver interface */

struct pcie_bus_s;
struct pcie_dev_type_s;
struct pcie_dev_s;

/* Bus related operations */

struct pcie_bus_ops_s
{
    CODE int (*pcie_enumerate)(FAR struct pcie_bus_s *bus,
                               FAR struct pcie_dev_type_s **types);

    CODE int (*pci_cfg_write)(FAR struct pcie_dev_s *dev, uintptr_t addr,
                              FAR const void *buffer, unsigned int size);

    CODE int (*pci_cfg_read)(FAR struct pcie_dev_s *dev, uintptr_t addr,
                             FAR void *buffer, unsigned int size);

    CODE int (*pci_map_bar)(FAR struct pcie_dev_s *dev, uint32_t addr,
                            unsigned long length);

    CODE int (*pci_map_bar64)(FAR struct pcie_dev_s *dev, uint64_t addr,
                            unsigned long length);

    CODE int (*pci_msi_register)(FAR struct pcie_dev_s *dev,
                                 uint16_t vector);

    CODE int (*pci_msix_register)(FAR struct pcie_dev_s *dev,
                                  uint32_t vector, uint32_t index);
};

/* PCIE bus private data. */

struct pcie_bus_s
{
  FAR const struct pcie_bus_ops_s *ops; /* operations */
};

/* PCIE device type, defines by vendor ID and device ID */

struct pcie_dev_type_s
{
  uint16_t      vendor;            /* Device vendor ID */
  uint16_t      device;            /* Device ID */
  uint32_t      class_rev;         /* Device reversion */
  const char    *name;             /* Human readable name */

  /* Call back function when a device is probed */

  CODE int (*probe)(FAR struct pcie_bus_s *bus,
                    FAR struct pcie_dev_type_s *type, uint16_t bdf);
};

/* PCIE device private data. */

struct pcie_dev_s
{
    FAR struct pcie_bus_s       *bus;
    FAR struct pcie_dev_type_s  *type;
    uint16_t                    bdf;
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
 * Name: pcie_initialize
 *
 * Description:
 *  Initialize the PCI-E bus and enumerate the devices with give devices
 *  type array
 *
 * Input Parameters:
 *   bus    - An PCIE bus
 *   types  - A array of PCIE device types
 *   num    - Number of device types
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int pcie_initialize(FAR struct pcie_bus_s *bus);

/****************************************************************************
 * Name: pci_enable_device
 *
 * Description:
 *  Enable device with MMIO
 *
 * Input Parameters:
 *   dev - device
 *
 * Return value:
 *   -EINVAL: error
 *   OK: OK
 *
 ****************************************************************************/

int pci_enable_device(FAR struct pcie_dev_s *dev);

/****************************************************************************
 * Name: pci_find_cap
 *
 * Description:
 *  Search through the PCI-e device capability list to find given capability.
 *
 * Input Parameters:
 *   dev - Device
 *   cap - Bitmask of capability
 *
 * Returned Value:
 *   -1: Capability not supported
 *   other: the offset in PCI configuration space to the capability structure
 *
 ****************************************************************************/

int pci_find_cap(FAR struct pcie_dev_s *dev, uint16_t cap);

/****************************************************************************
 * Name: pci_map_bar
 *
 * Description:
 *  Map address in a 32 bits bar in the flat memory address space
 *
 * Input Parameters:
 *   dev    - Device private data
 *   bar    - Bar number
 *   length - Map length, multiple of PAGE_SIZE
 *   ret    - Bar Contentif not NULL
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int pci_map_bar(FAR struct pcie_dev_s *dev, uint32_t bar,
                unsigned long length, uint32_t *ret);

/****************************************************************************
 * Name: pci_map_bar64
 *
 * Description:
 *  Map address in a 64 bits bar in the flat memory address space
 *
 * Input Parameters:
 *   dev    - Device private data
 *   bar    - Bar number
 *   length - Map length, multiple of PAGE_SIZE
 *   ret    - Bar Content if not NULL
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int pci_map_bar64(FAR struct pcie_dev_s *dev, uint32_t bar,
                  unsigned long length, uint64_t *ret);

/****************************************************************************
 * Name: pci_get_bar
 *
 * Description:
 *  Get a 32 bits bar
 *
 * Input Parameters:
 *   dev    - Device private data
 *   bar    - Bar number
 *   ret    - Bar Content
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int pci_get_bar(FAR struct pcie_dev_s *dev, uint32_t bar,
                uint32_t *ret);

/****************************************************************************
 * Name: pci_get_bar64
 *
 * Description:
 *  Get a 64 bits bar
 *
 * Input Parameters:
 *   dev    - Device private data
 *   bar    - Bar number
 *   ret    - Bar Content
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int pci_get_bar64(FAR struct pcie_dev_s *dev, uint32_t bar,
                  uint64_t *ret);

/****************************************************************************
 * Name: pci_set_bar
 *
 * Description:
 *  Set a 32 bits bar
 *
 * Input Parameters:
 *   dev    - Device private data
 *   bar    - Bar number
 *   val    - Bar Content
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int pci_set_bar(FAR struct pcie_dev_s *dev, uint32_t bar,
                uint32_t val);

/****************************************************************************
 * Name: pci_set_bar64
 *
 * Description:
 *  Set a 64 bits bar
 *
 * Input Parameters:
 *   dev    - Device private data
 *   bar    - Bar number
 *   val    - Bar Content
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int pci_set_bar64(FAR struct pcie_dev_s *dev, uint32_t bar,
                  uint64_t val);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif
#endif /* __INCLUDE_NUTTX_I2C_I2C_MASTER_H */
