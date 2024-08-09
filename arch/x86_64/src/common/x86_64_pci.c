/****************************************************************************
 * arch/x86_64/src/common/x86_64_pci.c
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

#include "x86_64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PCI_CFG_ADDR      0xcf8
#define PCI_DATA_ADDR     0xcfc
#define PCI_CFG_EN        (1 << 31)

/****************************************************************************
 * Private Functions Definitions
 ****************************************************************************/

static int x86_64_pci_write(struct pci_bus_s *bus, unsigned int devfn,
                            int where, int size, uint32_t val);
static int x86_64_pci_read(struct pci_bus_s *bus, unsigned int devfn,
                           int where, int size, uint32_t *val);
static uintptr_t x86_64_pci_map(struct pci_bus_s *bus, uintptr_t start,
                                uintptr_t end);
static int x86_64_pci_read_io(struct pci_bus_s *bus, uintptr_t addr,
                              int size, uint32_t *val);
static int x86_64_pci_write_io(struct pci_bus_s *bus, uintptr_t addr,
                               int size, uint32_t val);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pci_ops_s g_x86_64_pci_ops =
{
  .write    = x86_64_pci_write,
  .read     = x86_64_pci_read,
  .map      = x86_64_pci_map,
  .read_io  = x86_64_pci_read_io,
  .write_io = x86_64_pci_write_io,
};

static struct pci_controller_s g_x86_64_pci =
{
  .ops = &g_x86_64_pci_ops
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_pci_write
 *
 * Description:
 *  Write 8, 16, 32, 64 bits data to PCI configuration space of device
 *  specified by dev
 *
 * Input Parameters:
 *   bus   - Bus that PCI device resides
 *   devfn - The device and function bit field of bdf
 *   where - Offset in the specify PCI device cfg address space
 *   size  - The number of bytes to send from the buffer
 *   val   - The value to write
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static int x86_64_pci_write(struct pci_bus_s *bus, unsigned int devfn,
                            int where, int size, uint32_t val)
{
  uint8_t offset_mask = (4 - size);

  outl(PCI_CFG_EN | (bus->number << 16) | (devfn << 8) | where,
       PCI_CFG_ADDR);

  switch (size)
    {
      case 1:
        outb(val, PCI_DATA_ADDR + (where & offset_mask));
        break;
      case 2:
        outw(val, PCI_DATA_ADDR + (where & offset_mask));
        break;
      case 4:
        outl(val, PCI_DATA_ADDR);
        break;
      default:
        pcierr("Invalid cfg write size %d\n", size);
        return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: x86_64_pci_read
 *
 * Description:
 *  Read 8, 16, 32, 64 bits data from PCI configuration space of device
 *  specified by dev
 *
 * Input Parameters:
 *   bus    - Bus that PCI device resides
 *   devfn  - The device and function bit field of bdf
 *   where  - Offset in the specify PCI device cfg address space
 *   size   - The requested number of bytes to be read
 *   val    - A pointer to a buffer to receive the data from the device
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static int x86_64_pci_read(struct pci_bus_s *bus, unsigned int devfn,
                           int where, int size, uint32_t *val)
{
  uint8_t offset_mask = 4 - size;

  outl(PCI_CFG_EN | (bus->number << 16) | (devfn << 8) | where,
       PCI_CFG_ADDR);

  switch (size)
    {
      case 1:
        *val = inb(PCI_DATA_ADDR + (where & offset_mask));
        break;
      case 2:
        *val = inw(PCI_DATA_ADDR + (where & offset_mask));
        break;
      case 4:
        *val = inl(PCI_DATA_ADDR);
        break;
      default:
        *val = 0;
        pcierr("Invalid cfg read size %d\n", size);
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: x86_64_pci_read_io
 *
 * Description:
 *  Read 8, 16, 32, 64 bits data from PCI io address space of x86 64 device
 *
 * Input Parameters:
 *   bus    - Bus that PCI device resides
 *   addr   - The address to received data
 *   size   - The requested number of bytes to be read
 *   val    - A pointer to a buffer to receive the data from the device
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static int x86_64_pci_read_io(struct pci_bus_s *bus, uintptr_t addr,
                              int size, uint32_t *val)
{
  uint16_t portaddr = (uint16_t)addr;

  switch (size)
  {
    case 1:
      *val = (uint32_t)inb(portaddr);
      break;
    case 2:
      *val = (uint32_t)inw(portaddr);
      break;
    case 4:
      *val = (uint32_t)inl(portaddr);
      break;
    default:
      *val = 0;
      pcierr("Invalid read size %d\n", size);
      DEBUGPANIC();
      return -EINVAL;
  }

  return OK;
}

/****************************************************************************
 * Name: x86_64_pci_write_io
 *
 * Description:
 *  Write 8, 16, 32, 64 bits data to PCI io address space of x86 64 device
 *
 * Input Parameters:
 *   bus    - Bus that PCI device resides
 *   addr   - The address to write data
 *   size   - The requested number of bytes to be write
 *   val    - The value to write
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static int x86_64_pci_write_io(struct pci_bus_s *bus, uintptr_t addr,
                               int size, uint32_t val)
{
  uint16_t portaddr = (uint16_t)addr;

  switch (size)
  {
    case 1:
      outb((uint8_t)val, portaddr);
      break;
    case 2:
      outw((uint16_t)val, portaddr);
      break;
    case 4:
      outl((uint32_t)val, portaddr);
      break;
    default:
      pcierr("Invalid write size %d\n", size);
      DEBUGPANIC();
      return -EINVAL;
  }

  return OK;
}

/****************************************************************************
 * Name: x86_64_pci_map
 *
 * Description:
 *  Map a memory region
 *
 * Input Parameters:
 *   bus   - Bus that PCI device resides
 *   start - The start address to map
 *   end   - The end address to map
 *
 * Returned Value:
 *   >0: success, 0: A positive value errno
 *
 ****************************************************************************/

static uintptr_t x86_64_pci_map(struct pci_bus_s *bus, uintptr_t start,
                                uintptr_t end)
{
  int ret;

  ret = up_map_region((void *)start, end - start + 1, X86_PAGE_WR |
                 X86_PAGE_PRESENT | X86_PAGE_NOCACHE | X86_PAGE_GLOBAL);

  return ret < 0 ? 0 : start;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_pci_init
 *
 * Description:
 *  Initialize the PCI bus
 *
 ****************************************************************************/

void x86_64_pci_init(void)
{
  pci_register_controller(&g_x86_64_pci);
}
