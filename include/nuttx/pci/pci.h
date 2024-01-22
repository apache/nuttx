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

#ifdef CONFIG_PCI

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

/* PCI config common registers */

#define	PCI_CONFIG_VENDOR           0x00
#define	PCI_CONFIG_DEVICE           0x02
#define	PCI_CONFIG_COMMAND          0x04
#define	PCI_CONFIG_REV_ID           0x08
#define	PCI_CONFIG_PROG_IF          0x09
#define	PCI_CONFIG_SUBCLASS         0x0A
#define	PCI_CONFIG_CLASS            0x0B
#define	PCI_CONFIG_CACHE_LINE_SIZE  0x0C
#define	PCI_CONFIG_LATENCY_TIMER    0x0D
#define	PCI_CONFIG_HEADER_TYPE      0x0E
#define	PCI_CONFIG_BIST             0x0F

/* PCI config header types */

#define PCI_HEADER_NORMAL      0x00
#define PCI_HEADER_BRIDGE      0x01
#define PCI_HEADER_CARDBUS     0x02
#define PCI_HEADER_TYPE_MASK   0x3F
#define PCI_HEADER_MASK_MULTI  0x80

/* PCI config registers type 0 (Normal devices) */

#define PCI_HEADER_NORM_BAR0      0x10
#define PCI_HEADER_NORM_BAR1      0x14
#define PCI_HEADER_NORM_BAR2      0x18
#define PCI_HEADER_NORM_BAR3      0x1C
#define PCI_HEADER_NORM_BAR4      0x20
#define PCI_HEADER_NORM_BAR5      0x24
#define PCI_HEADER_NORM_CB_CIS    0x28
#define PCI_HEADER_NORM_SUB_VID   0x2C
#define PCI_HEADER_NORM_SUB_ID    0x2E
#define PCI_HEADER_NORM_EXP_ROM   0x30
#define PCI_HEADER_NORM_CAP       0x34
#define PCI_HEADER_NORM_INT_LINE  0x3C
#define PCI_HEADER_NORM_INT_PIN   0x3D
#define PCI_HEADER_NORM_MIN_GRANT 0x3E
#define PCI_HEADER_NORM_MAX_LAT   0x3E

/* PCI config registers type 1 (PCI-PCI bridge) */

#define PCI_CONFIG_SEC_BUS 0x19

/* PCI config registers type 2 (CardBus) */

/* PCI Base Class Codes */

#define PCI_CLASS_BASE_UNCLASSIFIED      0x00
#define PCI_CLASS_BASE_MASS_STORAGE_CTRL 0x01
#define PCI_CLASS_BASE_NETWORK_CTRL      0x02
#define PCI_CLASS_BASE_DISPLAY_CTRL      0x03
#define PCI_CLASS_BASE_MULTIMEDIA_CTRL   0x04
#define PCI_CLASS_BASE_MEM_CTRL          0x05
#define PCI_CLASS_BASE_BRG_DEV           0x06
#define PCI_CLASS_BASE_SMPL_COM_CTRL     0x07
#define PCI_CLASS_BASE_BSP               0x08
#define PCI_CLASS_BASE_INPUT_DEV_CTRL    0x09
#define PCI_CLASS_BASE_DOCK_STN          0x0A
#define PCI_CLASS_BASE_PROCESSOR         0x0B
#define PCI_CLASS_BASE_SBC               0x0C
#define PCI_CLASS_BASE_WIRELESS_CTRL     0x0D
#define PCI_CLASS_BASE_INTL_CTRL         0x0E
#define PCI_CLASS_BASE_SAT_COM_CTRL      0x0F
#define PCI_CLASS_BASE_ENCRYPT_CTRL      0x10
#define PCI_CLASS_BASE_SPC               0x11
#define PCI_CLASS_BASE_PROC_ACCEL        0x12
#define PCI_CLASS_BASE_NON_ES_INST       0x13

/* Reserved 0x14-0x3F */

#define PCI_CLASS_BASE_CO_PROC           0x40

/* Reserved 0x41-0xFE */

#define PCI_CLASS_BASE_UNASSIGNED        0xFF

/* PCI Sub Class Codes (most missing) */

/* Bridge Class */

#define PCI_CLASS_SUB_HOST_BRG           0x00
#define PCI_CLASS_SUB_ISA_BRG            0x01
#define PCI_CLASS_SUB_EISA_BRG           0x02
#define PCI_CLASS_SUB_MCA_BRG            0x03
#define PCI_CLASS_SUB_PCI_BRG            0x04
#define PCI_CLASS_SUB_PCMCIA_BRG         0x05
#define PCI_CLASS_SUB_NUBUS_BRG          0x06
#define PCI_CLASS_SUB_CARDBUS_BRG        0x07
#define PCI_CLASS_SUB_RACEWAY_BRG        0x08
#define PCI_CLASS_SUB_PCI_TRNSP_BRG      0x09
#define PCI_CLASS_SUB_INFINI_BRG         0x0A
#define PCI_CLASS_SUB_NUBUS_BRG          0x80

#define PCI_ID_ANY                       0xffff

/* PCI Command Register Bitmasks */

#define PCI_CMD_IO_SPACE    0x0001
#define PCI_CMD_MEM_SPACE   0x0002
#define PCI_CMD_BUS_MSTR    0x0004
#define PCI_CMD_SPECIAL_CYC 0x0008
#define PCI_CMD_MEM_INV     0x0030
#define PCI_CMD_VGA_PLT     0x0040
#define PCI_CMD_PAR_ERR     0x0080
#define PCI_CMD_SERR        0x0100
#define PCI_CMD_FST_B2B     0x0200
#define PCI_CMD_INT         0x0400

/* PCI BAR Bitmasks */

#define PCI_BAR_LAYOUT_MASK     0x00000001
#define PCI_BAR_TYPE_MASK       0x00000006
#define PCI_BAR_MEM_PF_MASK     0x00000008
#define PCI_BAR_MEM_BASE_MASK   0xfffffff0
#define PCI_BAR_IO_BASE_MASK    0xfffffffc

/* PCI BAR OFFSETS */

#define PCI_BAR_LAYOUT_OFFSET   0
#define PCI_BAR_TYPE_OFFSET     1
#define PCI_BAR_MEM_PF_OFFSET   3
#define PCI_BAR_MEM_BASE_OFFSET 4
#define PCI_BAR_IO_BASE_OFFSET  2

/* PCI BAR */

#define PCI_BAR_CNT         6
#define PCI_BAR_INVALID     0
#define PCI_BAR_LAYOUT_MEM  0
#define PCI_BAR_LAYOUT_IO   1
#define PCI_BAR_TYPE_32     0x00
#define PCI_BAR_TYPE_16     0x01  /* This mode is not used */
#define PCI_BAR_TYPE_64     0x02

/* PCI CAP */

#define PCI_CAP_ID_PM       0x01  /* Power Management */
#define PCI_CAP_ID_AGP      0x02  /* Accelerated Graphics */
#define PCI_CAP_ID_VPD      0x03  /* Vital Product Data */
#define PCI_CAP_ID_SLOT     0x04  /* Slot ID */
#define PCI_CAP_ID_MSI      0x05  /* MSI */
#define PCI_CAP_ID_CHP      0x06  /* CompactPCI Hot-Swap */
#define PCI_CAP_ID_PCIX     0x07  /* PCI-X */
#define PCI_CAP_ID_HT       0x08  /* HyperTransport */
#define PCI_CAP_ID_VNDR     0x09  /* Vendor */
#define PCI_CAP_ID_DBG      0x0A  /* Debug */
#define PCI_CAP_ID_CCRC     0x0B  /* CompactPCI Central Resource Control */
#define PCI_CAP_ID_HOT      0x0C  /* Hot-Plug Controller */
#define PCI_CAP_ID_BRG_VID  0x0D  /* Bridge Vendor/Device ID */
#define PCI_CAP_ID_AGP_BRG  0x0E  /* AGP PCI-PCI Bridge */
#define PCI_CAP_ID_SEC_DEV  0x0F  /* Secure Device */
#define PCI_CAP_ID_PCIE     0x10  /* PCIe */
#define PCI_CAP_ID_MSIX     0x11  /* MSI-X */
#define PCI_CAP_ID_SATA     0x12  /* SATA */
#define PCI_CAP_ID_ADVF     0x13  /* Advanced Features */

#define PCI_CAP_ID_END PCI_CAP_ID_ADVF

/* Resource types used by PCI devices */

#define PCI_SYS_RES_IOPORT 0x00
#define PCI_SYS_RES_MEM    0x01

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The PCI driver interface */

struct pci_bus_s;
struct pci_dev_type_s;
struct pci_dev_s;

/* Bus related operations */

struct pci_bus_ops_s
{
  /* Write 8, 16, 32, 64 bits data to PCI-E configuration space of device
   * specified by dev.
   */

  CODE void (*pci_cfg_write)(FAR struct pci_dev_s *dev, int reg,
                             uint32_t val, int width);

  /* Read 8, 16, 32, 64 bits data to PCI-E configuration space of device
   * specified by dev.
   */

  CODE uint32_t (*pci_cfg_read)(FAR struct pci_dev_s *dev, int reg,
                                int width);

  /* Map address in a 32 bits bar in the memory address space */

  CODE int (*pci_map_bar)(uint64_t addr, uint64_t len);

  /* Read from IO port */

  CODE uint32_t (*pci_io_read)(FAR const volatile void *addr, int width);

  /* Write to IO port */

  CODE void (*pci_io_write)(FAR const volatile void *addr, uint32_t val,
                            int width);
};

/* PCI bus private data. */

struct pci_bus_s
{
  FAR const struct pci_bus_ops_s *ops; /* operations */
};

/* PCI device type, defines by vendor ID and device ID */

struct pci_dev_type_s
{
  uint16_t      vendor;            /* Device vendor ID */
  uint16_t      device;            /* Device ID */
  uint32_t      class_rev;         /* Device reversion */
  const char    *name;             /* Human readable name */

  /* Call back function when a device is probed */

  CODE int (*probe)(FAR struct pci_bus_s *bus,
                    FAR const struct pci_dev_type_s *type, uint16_t bdf);
};

/* PCI device private data. */

struct pci_dev_s
{
  FAR struct pci_bus_s            *bus;
  FAR const struct pci_dev_type_s *type;
  uint32_t                         bdf;
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
 * Name: pci_initialize
 *
 * Description:
 *  Initialize the PCI bus and enumerate the devices with give devices
 *  type array
 *
 * Input Parameters:
 *   bus    - An PCI bus
 *   types  - A array of PCI device types
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int pci_initialize(FAR struct pci_bus_s *bus);

/****************************************************************************
 * Name: pci_enable_io
 *
 * Description:
 *  Enable MMIO or IOPORT
 *
 * Input Parameters:
 *   dev   - device
 *   space - which resource is being enabled
 *           PCI_SYS_RES_IOPORT for io port address decoding or
 *           PCI_RES_MEM for memory
 *
 * Return value:
 *   -EINVAL: error
 *   OK: OK
 *
 ****************************************************************************/

int pci_enable_io(FAR struct pci_dev_s *dev, int res);

/****************************************************************************
 * Name: pci_disable_io
 *
 * Description:
 *  Enable MMIO or IOPORT
 *
 * Input Parameters:
 *   dev   - device
 *   space - which resource is being disabled
 *           PCI_SYS_RES_IOPORT for io port address decoding or
 *           PCI_SYS_RES_MEM for memory
 *
 * Return value:
 *   -EINVAL: error
 *   OK: OK
 *
 ****************************************************************************/

int pci_disable_io(FAR struct pci_dev_s *dev, int res);

/****************************************************************************
 * Name: pci_enable_bus_master
 *
 * Description:
 *  Enable bus mastering for device so it can perform PCI accesses
 *
 * Input Parameters:
 *   dev   - device
 *
 * Return value:
 *   -EINVAL: error
 *   OK: OK
 *
 ****************************************************************************/

int pci_enable_bus_master(FAR struct pci_dev_s *dev);

/****************************************************************************
 * Name: pci_disable_bus_master
 *
 * Description:
 *  Disable bus mastering for device
 *
 * Input Parameters:
 *   dev   - device
 *
 * Return value:
 *   -EINVAL: error
 *   OK: OK
 *
 ****************************************************************************/

int pci_disable_bus_master(FAR struct pci_dev_s *dev);

/****************************************************************************
 * Name: pci_bar_valid
 *
 * Description:
 *  Determine in if the address in the BAR is valid
 *
 * Input Parameters:
 *   dev   - device
 *   bar_id - bar number
 *
 * Return value:
 *   -EINVAL: error
 *   OK: OK
 *
 ****************************************************************************/

int pci_bar_valid(FAR struct pci_dev_s *dev, uint8_t bar_id);

/****************************************************************************
 * Name: pci_bar_is_64
 *
 * Description:
 *  Determine in if the bar address is 64 bit.  If it is the address includes
 *  the address in the next bar location.
 *
 * Input Parameters:
 *   dev   - device
 *   bar_id - bar number
 *
 * Return value:
 *   true: 64bit address
 *
 ****************************************************************************/

bool pci_bar_is_64(FAR struct pci_dev_s *dev, uint8_t bar_id);

/****************************************************************************
 * Name: pci_bar_size
 *
 * Description:
 *  Determine the size of the address space required by the BAR
 *
 * Input Parameters:
 *   dev   - device
 *   bar_id - bar number
 *
 * Return value:
 *   Size of address space
 *
 ****************************************************************************/

uint64_t pci_bar_size(FAR struct pci_dev_s *dev, uint8_t bar_id);

/****************************************************************************
 * Name: pci_bar_addr
 *
 * Description:
 *  Determine the size of the address space required by the BAR
 *
 * Input Parameters:
 *   dev   - device
 *   bar_id - bar number
 *
 * Return value:
 *   full bar address
 *
 ****************************************************************************/

uint64_t pci_bar_addr(FAR struct pci_dev_s *dev, uint8_t bar_id);

/****************************************************************************
 * Name: pci_dev_dump
 *
 * Description:
 *  Dump the configuration information for the device
 *
 * Input Parameters:
 *   dev   - device
 *
 ****************************************************************************/

void pci_dev_dump(FAR struct pci_dev_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif
#endif /* __INCLUDE_NUTTX_PCI_PCI_H */
