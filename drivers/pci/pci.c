/****************************************************************************
 * nuttx/drivers/pci/pci.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/pci/pci.h>
#include <nuttx/virt/qemu_pci.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PCI_BDF(bus, slot, func) (((uint32_t)bus << 8) | \
                                  ((uint32_t)slot << 3) | \
                                  func)

/****************************************************************************
 * Private Functions Definitions
 ****************************************************************************/

static void pci_probe_device(FAR struct pci_bus_s *root_bus,
                             uint8_t bus_idx, uint8_t slot_idx, uint8_t func,
                             FAR struct pci_dev_type_s **types);

static uint8_t pci_check_pci_bridge(FAR struct pci_bus_s *root_bus,
                                    uint8_t bus_idx, uint8_t slot_idx,
                                    uint8_t dev_func);

static void pci_scan_device(FAR struct pci_bus_s *root_bus,
                            uint8_t bus_idx, uint8_t slot_idx,
                            FAR struct pci_dev_type_s **types);

static void pci_scan_bus(FAR struct pci_bus_s *root_bus,
                         uint8_t bus_idx,
                         FAR struct pci_dev_type_s **types);

static void pci_set_cmd_bit(FAR struct pci_dev_s *dev, uint16_t bitmask);

static void pci_clear_cmd_bit(FAR struct pci_dev_s *dev, uint16_t bitmask);

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct pci_dev_type_s *pci_device_types[] =
{
#ifdef CONFIG_VIRT_QEMU_PCI_TEST
  &pci_type_qemu_pci_test,
#endif /* CONFIG_VIRT_QEMU_PCI_TEST */
  NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_probe
 *
 * Description:
 *  Checks if the specified device is supported and if so calls probe on it
 *
 * Input Parameters:
 *   root_bus - The root bus device that lets us address the whole tree
 *   bus      - Bus ID
 *   slot     - Device Slot
 *   func     - Device Function
 *   types    - List of pointers to devices types recognized, NULL terminated
 *
 ****************************************************************************/

static void pci_probe_device(FAR struct pci_bus_s *root_bus,
                             uint8_t bus_idx, uint8_t slot_idx, uint8_t func,
                             FAR struct pci_dev_type_s **types)
{
  struct pci_dev_s tmp_dev;
  uint16_t vid;
  uint16_t id;
  uint32_t class_rev;

  tmp_dev.bus = root_bus;
  tmp_dev.bdf = PCI_BDF(bus_idx, slot_idx, func);

  vid = root_bus->ops->pci_cfg_read(&tmp_dev, PCI_CONFIG_VENDOR, 2);
  id = root_bus->ops->pci_cfg_read(&tmp_dev, PCI_CONFIG_DEVICE, 2);

  /* This is reading rev prog_if subclass and class */

  class_rev = root_bus->ops->pci_cfg_read(&tmp_dev, PCI_CONFIG_REV_ID, 4);

  pciinfo("[%02x:%02x.%x] Found %04x:%04x, class/revision %08x\n",
          bus_idx, slot_idx, func, vid, id, class_rev);

  for (int i = 0; types[i] != NULL; i++)
    {
      if (types[i]->vendor == PCI_ID_ANY ||
          types[i]->vendor == vid)
        {
          if (types[i]->device == PCI_ID_ANY ||
              types[i]->device == id)
            {
              if (types[i]->class_rev == PCI_ID_ANY ||
                  types[i]->class_rev == class_rev)
                {
                  if (types[i]->probe)
                    {
                      pciinfo("[%02x:%02x.%x] Probing\n",
                        bus_idx, slot_idx, func);
                      types[i]->probe(root_bus, types[i], tmp_dev.bdf);
                    }
                  else
                    {
                      pcierr("[%02x:%02x.%x] Error: Invalid \
                              device probe function\n",
                              bus_idx, slot_idx, func);
                    }
                  break;
                }
            }
        }
    }
}

/****************************************************************************
 * Name: pci_check_pci_bridge
 *
 * Description:
 *  Checks if the specified device is PCI bridge and return the sub-bridge
 *  idx if found.  Otherwise return 0.
 *
 * Input Parameters:
 *   root_bus - The root bus device that lets us address the whole tree
 *   bus      - Bus ID
 *   slot     - Device Slot
 *   func     - Device Function
 *
 ****************************************************************************/

static uint8_t pci_check_pci_bridge(FAR struct pci_bus_s *root_bus,
                                    uint8_t bus_idx, uint8_t slot_idx,
                                    uint8_t dev_func)
{
  struct pci_dev_s tmp_dev;
  uint8_t base_class;
  uint8_t sub_class;
  uint8_t secondary_bus;

  tmp_dev.bus = root_bus;
  tmp_dev.bdf = PCI_BDF(bus_idx, slot_idx, dev_func);

  /* Check if this is a PCI-PCI bridge device */

  base_class = root_bus->ops->pci_cfg_read(&tmp_dev, PCI_CONFIG_CLASS, 1);
  sub_class = root_bus->ops->pci_cfg_read(&tmp_dev, PCI_CONFIG_SUBCLASS, 1);

  if ((base_class == PCI_CLASS_BASE_BRG_DEV) && \
     (sub_class == PCI_CLASS_SUB_PCI_BRG))
    {
      /* This is a bridge device we need to determin the bus idx and
       * enumerate it just like we do the root.
       */

      pciinfo("[%02x:%02x.%x] Found Bridge\n",
        bus_idx, slot_idx, dev_func);

      secondary_bus = root_bus->ops->pci_cfg_read(
        &tmp_dev, PCI_CONFIG_SEC_BUS, 1);
      return secondary_bus;
    }

  return 0;
}

/****************************************************************************
 * Name: pci_scan_device
 *
 * Description:
 *  Checks if the specified device is a bus and iterates over it or
 *  if it is a real device initializes it if recognized.
 *
 * Input Parameters:
 *   root_bus - The root bus device that lets us address the whole tree
 *   bus      - Bus ID
 *   slot     - Device Slot
 *   types    - List of pointers to devices types recognized, NULL terminated
 *
 ****************************************************************************/

static void pci_scan_device(FAR struct pci_bus_s *root_bus,
                            uint8_t bus_idx, uint8_t slot_idx,
                            FAR struct pci_dev_type_s **types)
{
  struct pci_dev_s tmp_dev;
  uint8_t dev_func = 0;
  uint16_t vid;
  uint8_t sec_bus;
  uint8_t multi_function;

  tmp_dev.bus = root_bus;
  tmp_dev.bdf = PCI_BDF(bus_idx, slot_idx, dev_func);
  vid = root_bus->ops->pci_cfg_read(&tmp_dev, PCI_CONFIG_VENDOR, 2);
  if (vid == 0xffff)
      return;

  /* Check if this is a PCI-PCI bridge device */

  sec_bus = pci_check_pci_bridge(root_bus, bus_idx, slot_idx, dev_func);
  if (sec_bus)
    pci_scan_bus(root_bus, sec_bus, types);

  multi_function = root_bus->ops->pci_cfg_read(
    &tmp_dev, PCI_CONFIG_HEADER_TYPE, 1) & PCI_HEADER_MASK_MULTI;
  if (multi_function)
    {
      /* This is a multi-function device that we need to iterate over */

      for (dev_func = 1; dev_func < 8; dev_func++)
        {
          vid = root_bus->ops->pci_cfg_read(&tmp_dev, PCI_CONFIG_VENDOR, 2);
          if (vid != 0xffff)
            {
              sec_bus = pci_check_pci_bridge(
                root_bus, bus_idx, slot_idx, dev_func);
              if (sec_bus)
                {
                  pci_scan_bus(root_bus, sec_bus, types);
                  continue;
                }

              /* Actually enumerate device */

              pci_probe_device(root_bus, bus_idx, slot_idx, dev_func, types);
            }
        }
    }
  else
    {
      pci_probe_device(root_bus, bus_idx, slot_idx, dev_func, types);
    }
}

/****************************************************************************
 * Name: pci_scan_bus
 *
 * Description:
 *  Iterates over all slots on bus looking for devices and buses to
 *  enumerate.
 *
 * Input Parameters:
 *   root_bus - The root bus device that lets us address the whole tree
 *   bus      - Bus ID
 *   types    - List of pointers to devices types recognized, NULL terminated
 *
 ****************************************************************************/

static void pci_scan_bus(FAR struct pci_bus_s *root_bus,
                         uint8_t bus_idx,
                         FAR struct pci_dev_type_s **types)
{
  uint8_t slot_idx;

  for (slot_idx = 0; slot_idx < 32; slot_idx++)
    {
      pci_scan_device(root_bus, bus_idx, slot_idx, types);
    }

  return;
}

/****************************************************************************
 * Name: pci_set_cmd_bit
 *
 * Description:
 *  This sets an individual bit in the command register for a device.
 *
 * Input Parameters:
 *   dev - device
 *   bit - Bit to set
 *
 ****************************************************************************/

static void pci_set_cmd_bit(FAR struct pci_dev_s *dev, uint16_t bitmask)
{
  uint16_t	cmd;

  cmd = dev->bus->ops->pci_cfg_read(dev, PCI_CONFIG_COMMAND, 2);
  dev->bus->ops->pci_cfg_write(dev, PCI_CONFIG_COMMAND,
    (cmd | bitmask), 2);
}

/****************************************************************************
 * Name: pci_clear_cmd_bit
 *
 * Description:
 *  This clears an individual bit in the command register for a device.
 *
 * Input Parameters:
 *   dev - device
 *   bit - Bit to set
 *
 ****************************************************************************/

static void pci_clear_cmd_bit(FAR struct pci_dev_s *dev, uint16_t bitmask)
{
  uint16_t	cmd;

  cmd = dev->bus->ops->pci_cfg_read(dev, PCI_CONFIG_COMMAND, 2);
  dev->bus->ops->pci_cfg_write(dev, PCI_CONFIG_COMMAND,
    (cmd & ~bitmask), 2);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_enumerate
 *
 * Description:
 *  Scan the PCI bus and enumerate the devices.
 *  Initialize any recognized devices, given in types.
 *
 * Input Parameters:
 *   bus    - PCI-E bus structure
 *   types  - List of pointers to devices types recognized, NULL terminated
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int pci_enumerate(FAR struct pci_bus_s *bus,
                  FAR struct pci_dev_type_s **types)
{
  if (!bus)
      return -EINVAL;
  if (!types)
      return -EINVAL;

  pci_scan_bus(bus, 0, types);
  return OK;
}

/****************************************************************************
 * Name: pci_initialize
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

int pci_initialize(FAR struct pci_bus_s *bus)
{
  return pci_enumerate(bus, pci_device_types);
}

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
 *           PCI_SYS_RES_MEM for memory
 *
 * Return value:
 *   -EINVAL: error
 *   OK: OK
 *
 ****************************************************************************/

int pci_enable_io(FAR struct pci_dev_s *dev, int res)
{
  switch (res)
    {
    case PCI_SYS_RES_IOPORT:
      pci_set_cmd_bit(dev, PCI_CMD_IO_SPACE);
      return OK;
    case PCI_SYS_RES_MEM:
      pci_set_cmd_bit(dev, PCI_CMD_MEM_SPACE);
      return OK;
    }

  return -EINVAL;
}

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

int pci_disable_io(FAR struct pci_dev_s *dev, int res)
{
  switch (res)
    {
    case PCI_SYS_RES_IOPORT:
      pci_clear_cmd_bit(dev, PCI_CMD_IO_SPACE);
      return OK;
    case PCI_SYS_RES_MEM:
      pci_clear_cmd_bit(dev, PCI_CMD_MEM_SPACE);
      return OK;
    }

  return -EINVAL;
}

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

int pci_enable_bus_master(FAR struct pci_dev_s *dev)
{
  pci_set_cmd_bit(dev, PCI_CMD_BUS_MSTR);
  return OK;
}

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

int pci_disable_bus_master(FAR struct pci_dev_s *dev)
{
  pci_clear_cmd_bit(dev, PCI_CMD_BUS_MSTR);
  return OK;
}
