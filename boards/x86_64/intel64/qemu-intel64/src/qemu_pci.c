/****************************************************************************
 * boards/x86_64/intel64/qemu-intel64/src/qemu_pci.c
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
#include <debug.h>

#include <nuttx/pci/pci.h>

#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PCI_CFG_ADDR       0xcf8
#define PCI_DATA_ADDR      0xcfc
#define PCI_CFG_EN         (1 << 31)

/****************************************************************************
 * Private Functions Definitions
 ****************************************************************************/

static void qemu_pci_cfg_write(FAR struct pci_dev_s *dev, int reg,
                              uint32_t val, int width);

static uint32_t qemu_pci_cfg_read(FAR struct pci_dev_s *dev, int reg,
                                  int width);

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct pci_bus_ops_s qemu_pci_bus_ops =
{
    .pci_cfg_write     =   qemu_pci_cfg_write,
    .pci_cfg_read      =   qemu_pci_cfg_read,
};

struct pci_bus_s qemu_pci_bus =
{
    .ops = &qemu_pci_bus_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_pci_cfg_write
 *
 * Description:
 *  Write 8, 16, 32, 64 bits data to PCI-E configuration space of device
 *  specified by dev
 *
 * Input Parameters:
 *   bdf    - Device private data
 *   reg - A pointer to the read-only buffer of data to be written
 *   size   - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static void qemu_pci_cfg_write(FAR struct pci_dev_s *dev, int reg,
                               uint32_t val, int width)
{
  uint8_t offset_mask = (4 - width);
  outl(PCI_CFG_EN | (dev->bdf << 8) | reg, PCI_CFG_ADDR);
  switch (width)
    {
      case 1:
        outb(val, PCI_DATA_ADDR + (reg & offset_mask));
        return;
      case 2:
        outw(val, PCI_DATA_ADDR + (reg & offset_mask));
        return;
      case 4:
        outl(val, PCI_DATA_ADDR);
        return;
      default:
        pcierr("Invalid cfg write width %d\n", width);
    }
}

/****************************************************************************
 * Name: qemu_pci_cfg_read
 *
 * Description:
 *  Read 8, 16, 32, 64 bits data from PCI-E configuration space of device
 *  specified by dev
 *
 * Input Parameters:
 *   dev    - Device private data
 *   buffer - A pointer to a buffer to receive the data from the device
 *   size   - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static uint32_t qemu_pci_cfg_read(FAR struct pci_dev_s *dev, int reg,
                                  int width)
{
  uint32_t ret;
  uint8_t offset_mask = 4 - width;
  outl(PCI_CFG_EN | (dev->bdf << 8) | reg, PCI_CFG_ADDR);

  switch (width)
    {
      case 1:
        ret = inb(PCI_DATA_ADDR + (reg & offset_mask));
        return ret;

      case 2:
        ret = inw(PCI_DATA_ADDR + (reg & offset_mask));
        return ret;
      case 4:
        ret = inl(PCI_DATA_ADDR);
        return ret;
      default:
        pcierr("Invalid cfg read width %d\n", width);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_pci_init
 *
 * Description:
 *  Initialize the PCI-E bus *
 *
 ****************************************************************************/

void qemu_pci_init(void)
{
  pciinfo("Initializing PCI Bus\n");
  pci_initialize(&qemu_pci_bus);
}
