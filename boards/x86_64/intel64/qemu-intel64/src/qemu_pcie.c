/****************************************************************************
 * boards/x86_64/intel64/qemu-intel64/src/qemu_pcie.c
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

/* The MSI and MSI-X vector setup function are taken from Jailhouse inmate
 * library
 *
 * Jailhouse, a Linux-based partitioning hypervisor
 *
 * Copyright (c) Siemens AG, 2014
 *
 * Authors:
 *  Jan Kiszka <jan.kiszka@siemens.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 * Alternatively, you can use or redistribute this file under the following
 * BSD license:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>

#include <nuttx/pcie/pcie.h>

#include "qemu_pcie_readwrite.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QEMU_PCIE_MAX_BDF 0x10000

/****************************************************************************
 * Private Functions Definitions
 ****************************************************************************/

static int qemu_pci_enumerate(FAR struct pcie_bus_s *bus,
                               FAR struct pcie_dev_type_s **types);

static int qemu_pci_cfg_write(FAR struct pcie_dev_s *dev, uintptr_t addr,
                              FAR const void *buffer, unsigned int size);

static int qemu_pci_cfg_read(FAR struct pcie_dev_s *dev, uintptr_t addr,
                             FAR void *buffer, unsigned int size);

static int qemu_pci_map_bar(FAR struct pcie_dev_s *dev, uint32_t addr,
                            unsigned long length);

static int qemu_pci_map_bar64(FAR struct pcie_dev_s *dev, uint64_t addr,
                              unsigned long length);

static int qemu_pci_msix_register(FAR struct pcie_dev_s *dev,
                                  uint32_t vector, uint32_t index);

static int qemu_pci_msi_register(FAR struct pcie_dev_s *dev,
                                 uint16_t vector);

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct pcie_bus_ops_s qemu_pcie_bus_ops =
{
    .pcie_enumerate    =   qemu_pci_enumerate,
    .pci_cfg_write     =   qemu_pci_cfg_write,
    .pci_cfg_read      =   qemu_pci_cfg_read,
    .pci_map_bar       =   qemu_pci_map_bar,
    .pci_map_bar64     =   qemu_pci_map_bar64,
    .pci_msix_register = qemu_pci_msix_register,
    .pci_msi_register  = qemu_pci_msi_register,
};

struct pcie_bus_s qemu_pcie_bus =
{
    .ops = &qemu_pcie_bus_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_pci_enumerate
 *
 * Description:
 *  Scan the PCI bus and enumerate the devices.
 *  Initialize any recognized devices, given in types.
 *
 * Input Parameters:
 *   bus    - PCI-E bus structure
 *   type   - List of pointers to devices types recognized, NULL terminated
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static int qemu_pci_enumerate(FAR struct pcie_bus_s *bus,
                               FAR struct pcie_dev_type_s **types)
{
  unsigned int bdf;
  uint16_t vid;
  uint16_t id;
  uint16_t rev;

  if (!bus)
      return -EINVAL;
  if (!types)
      return -EINVAL;

  for (bdf = 0; bdf < QEMU_PCIE_MAX_BDF; bdf++)
    {
      __qemu_pci_cfg_read(bdf, PCI_CFG_VENDOR_ID, &vid, 2);
      __qemu_pci_cfg_read(bdf, PCI_CFG_DEVICE_ID, &id, 2);
      __qemu_pci_cfg_read(bdf, PCI_CFG_REVERSION, &rev, 2);

      if (vid == PCI_ID_ANY)
        continue;

      pciinfo("[%02x:%02x.%x] Found %04x:%04x, class/reversion %08x\n",
              bdf >> 8, (bdf >> 3) & 0x1f, bdf & 0x3,
              vid, id, rev);

      for (int i = 0; types[i] != NULL; i++)
        {
          if (types[i]->vendor == PCI_ID_ANY ||
              types[i]->vendor == vid)
            {
              if (types[i]->device == PCI_ID_ANY ||
                  types[i]->device == id)
                {
                  if (types[i]->class_rev == PCI_ID_ANY ||
                      types[i]->class_rev == rev)
                    {
                      if (types[i]->probe)
                        {
                          pciinfo("[%02x:%02x.%x] %s\n",
                                  bdf >> 8, (bdf >> 3) & 0x1f, bdf & 0x3,
                                  types[i]->name);
                          types[i]->probe(bus, types[i], bdf);
                        }
                      else
                        {
                          pcierr("[%02x:%02x.%x] Error: Invalid \
                                  device probe function\n",
                                  bdf >> 8, (bdf >> 3) & 0x1f, bdf & 0x3);
                        }
                      break;
                    }
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: qemu_pci_cfg_write
 *
 * Description:
 *  Write 8, 16, 32, 64 bits data to PCI-E configuration space of device
 *  specified by dev
 *
 * Input Parameters:
 *   bdf    - Device private data
 *   buffer - A pointer to the read-only buffer of data to be written
 *   size   - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static int qemu_pci_cfg_write(FAR struct pcie_dev_s *dev, uintptr_t addr,
                              FAR const void *buffer, unsigned int size)
{
  switch (size)
    {
      case 1:
      case 2:
      case 4:
        return __qemu_pci_cfg_write(dev->bdf, addr, buffer, size);
      case 8:
        return __qemu_pci_cfg_write(dev->bdf, addr, buffer, size);
      default:
        return -EINVAL;
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

static int qemu_pci_cfg_read(FAR struct pcie_dev_s *dev, uintptr_t addr,
                             FAR void *buffer, unsigned int size)
{
  switch (size)
    {
      case 1:
      case 2:
      case 4:
        return __qemu_pci_cfg_read(dev->bdf, addr, buffer, size);
      case 8:
        return __qemu_pci_cfg_read64(dev->bdf, addr, buffer, size);
      default:
        return -EINVAL;
    }
}

/****************************************************************************
 * Name: qemu_pci_map_bar
 *
 * Description:
 *  Map address in a 32 bits bar in the memory address space
 *
 * Input Parameters:
 *   dev    - Device private data
 *   bar    - Bar number
 *   length - Map length, multiple of PAGE_SIZE
 *   ret    - Bar Content
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static int qemu_pci_map_bar(FAR struct pcie_dev_s *dev, uint32_t addr,
                            unsigned long length)
{
  up_map_region((void *)((uintptr_t)addr), length,
      X86_PAGE_WR | X86_PAGE_PRESENT | X86_PAGE_NOCACHE | X86_PAGE_GLOBAL);

  return OK;
}

/****************************************************************************
 * Name: qemu_pci_map_bar64
 *
 * Description:
 *  Map address in a 64 bits bar in the memory address space
 *
 * Input Parameters:
 *   dev    - Device private data
 *   bar    - Bar number
 *   length - Map length, multiple of PAGE_SIZE
 *   ret    - Bar Content
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static int qemu_pci_map_bar64(FAR struct pcie_dev_s *dev, uint64_t addr,
                              unsigned long length)
{
  up_map_region((void *)((uintptr_t)addr), length,
      X86_PAGE_WR | X86_PAGE_PRESENT | X86_PAGE_NOCACHE | X86_PAGE_GLOBAL);

  return OK;
}

/****************************************************************************
 * Name: qemu_pci_msix_register
 *
 * Description:
 *  Map a device MSI-X vector to a platform IRQ vector
 *
 * Input Parameters:
 *   dev - Device
 *   vector - IRQ number of the platform
 *   index  - Device MSI-X vector number
 *
 * Returned Value:
 *   <0: Mapping failed
 *    0: Mapping succeed
 *
 ****************************************************************************/

static int qemu_pci_msix_register(FAR struct pcie_dev_s *dev,
                                  uint32_t vector, uint32_t index)
{
  unsigned int bar;
  uint16_t message_control;
  uint32_t table_bar_ind;
  uint32_t lo_table_addr;
  uint32_t hi_table_addr;
  uint64_t msix_table_addr = 0;

  int cap = pci_find_cap(dev, PCI_CAP_MSIX);
  if (cap < 0)
      return -EINVAL;

  __qemu_pci_cfg_read(dev->bdf, cap + 2, &message_control, 2);

  /* bounds check */

  if (index > (message_control & 0x3ff))
      return -EINVAL;

  __qemu_pci_cfg_read(dev->bdf, cap + 4, &table_bar_ind, 4);

  bar = (table_bar_ind & 7) * 4 + PCI_CFG_BAR;

  __qemu_pci_cfg_read(dev->bdf, bar, &lo_table_addr, 4);

  if ((lo_table_addr & 6) == PCI_BAR_64BIT)
    {
      __qemu_pci_cfg_read(dev->bdf, bar + 4, &hi_table_addr, 4);
      msix_table_addr = (uint64_t)hi_table_addr << 32;
    }

  msix_table_addr |= lo_table_addr & ~0xf;
  msix_table_addr += table_bar_ind & ~0x7;

  /* enable and mask */

  message_control |= (MSIX_CTRL_ENABLE | MSIX_CTRL_FMASK);
  __qemu_pci_cfg_write(dev->bdf, cap + 2, &message_control, 2);

  msix_table_addr += 16 * index;
  mmio_write32((uint32_t *)(msix_table_addr),
               0xfee00000 | up_apic_cpu_id() << 12);
  mmio_write32((uint32_t *)(msix_table_addr + 4), 0);
  mmio_write32((uint32_t *)(msix_table_addr + 8), vector);
  mmio_write32((uint32_t *)(msix_table_addr + 12), 0);

  /* enable and unmask */

  message_control &= ~MSIX_CTRL_FMASK;

  __qemu_pci_cfg_write(dev->bdf, cap + 2, &message_control, 2);

  return 0;
}

/****************************************************************************
 * Name: qemu_pci_msi_register
 *
 * Description:
 *  Map device MSI vectors to a platform IRQ vector
 *
 * Input Parameters:
 *   dev - Device
 *   vector - IRQ number of the platform
 *
 * Returned Value:
 *   <0: Mapping failed
 *    0: Mapping succeed
 *
 ****************************************************************************/

static int qemu_pci_msi_register(FAR struct pcie_dev_s *dev, uint16_t vector)
{
  uint16_t ctl;
  uint16_t data;

  int cap = pci_find_cap(dev, PCI_CAP_MSI);
  if (cap < 0)
      return -1;

  uint32_t dest = 0xfee00000 | (up_apic_cpu_id() << 12);
  __qemu_pci_cfg_write(dev->bdf, cap + 4, &dest, 4);

  __qemu_pci_cfg_read(dev->bdf, cap + 2, &ctl, 2);
  if (ctl & (1 << 7))
    {
      uint32_t tmp = 0;
      __qemu_pci_cfg_write(dev->bdf, cap + 8, &tmp, 4);
      data = cap + 0x0c;
    }
  else
    {
      data = cap + 0x08;
    }

  __qemu_pci_cfg_write(dev->bdf, data, &vector, 2);

  __qemu_pci_cfg_write(dev->bdf, cap + 2, &vector, 2);

  uint16_t en = 0x0001;
  __qemu_pci_cfg_write(dev->bdf, cap + 2, &en, 2);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_pcie_init
 *
 * Description:
 *  Initialize the PCI-E bus *
 *
 ****************************************************************************/

void qemu_pcie_init(void)
{
  pcie_initialize(&qemu_pcie_bus);
}
