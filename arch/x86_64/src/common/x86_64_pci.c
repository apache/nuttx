/****************************************************************************
 * arch/x86_64/src/common/x86_64_pci.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/arch.h>
#include <nuttx/pci/pci.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PCI_CFG_ADDR         0xcf8
#define PCI_DATA_ADDR        0xcfc
#define PCI_CFG_EN           (1 << 31)

#define X86_64_IO_ADDR_LIMIT 0xffff

#define readb(addr)          ((addr) > X86_64_IO_ADDR_LIMIT ? \
                              *((volatile uint8_t *)(addr)) : inb(addr))
#define readw(addr)          ((addr) > X86_64_IO_ADDR_LIMIT ? \
                              *((volatile uint16_t *)(addr)) : inw(addr))
#define readl(addr)          ((addr) > X86_64_IO_ADDR_LIMIT ? \
                              *((volatile uint32_t *)(addr)) : inl(addr))

#define writeb(addr, val)    ((addr) > X86_64_IO_ADDR_LIMIT ? \
                              *((volatile uint8_t *)(addr)) = (val) : outb(val, addr))
#define writew(addr, val)    ((addr) > X86_64_IO_ADDR_LIMIT ? \
                              *((volatile uint16_t *)(addr)) = (val) : outw(val, addr))
#define writel(addr, val)    ((addr) > X86_64_IO_ADDR_LIMIT ? \
                              *((volatile uint32_t *)(addr)) = (val) : outl(val, addr))

/****************************************************************************
 * Private Functions Definitions
 ****************************************************************************/

static int x86_64_pci_write(struct pci_bus_s *bus, uint32_t devfn,
                            int where, int size, uint32_t val);
static int x86_64_pci_read(struct pci_bus_s *bus, uint32_t devfn,
                           int where, int size, uint32_t *val);
static uintptr_t x86_64_pci_map(struct pci_bus_s *bus, uintptr_t start,
                                uintptr_t end);
static int x86_64_pci_read_io(struct pci_bus_s *bus, uintptr_t addr,
                              int size, uint32_t *val);
static int x86_64_pci_write_io(struct pci_bus_s *bus, uintptr_t addr,
                               int size, uint32_t val);

static int x86_64_pci_get_irq(struct pci_bus_s *bus, uint32_t devfn,
                              uint8_t line, uint8_t pin);

static int x86_64_pci_alloc_irq(struct pci_bus_s *bus, uint32_t devfn,
                                int *irq, int num);
static void x86_64_pci_release_irq(struct pci_bus_s *bus,
                                   int *irq, int num);
static int x86_64_pci_connect_irq(struct pci_bus_s *bus,
                                  int *irq, int num,
                                  uintptr_t *mar, uint32_t *mdr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pci_ops_s g_x86_64_pci_ops =
{
  .write       = x86_64_pci_write,
  .read        = x86_64_pci_read,
  .map         = x86_64_pci_map,
  .read_io     = x86_64_pci_read_io,
  .write_io    = x86_64_pci_write_io,
  .get_irq     = x86_64_pci_get_irq,
  .alloc_irq   = x86_64_pci_alloc_irq,
  .release_irq = x86_64_pci_release_irq,
  .connect_irq = x86_64_pci_connect_irq,
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

static int x86_64_pci_write(struct pci_bus_s *bus, uint32_t devfn,
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

static int x86_64_pci_read(struct pci_bus_s *bus, uint32_t devfn,
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
  switch (size)
  {
    case 1:
      *val = readb(addr);
      break;
    case 2:
      *val = readw(addr);
      break;
    case 4:
      *val = readl(addr);
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
  switch (size)
  {
    case 1:
      writeb(addr, val);
      break;
    case 2:
      writew(addr, val);
      break;
    case 4:
      writel(addr, val);
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
 * Name: x86_64_pci_get_irq
 *
 * Description:
 *  Get interrupt number associated with a given INTx line.
 *
 * Input Parameters:
 *   bus   - Bus that PCI device resides
 *   devfn - The pci device and function number
 *   line  - Activated PCI legacy interrupt line
 *   pin   - Intx pin number
 *
 * Returned Value:
 *   Return interrupt number associated with a given INTx
 *
 ****************************************************************************/

static int x86_64_pci_get_irq(struct pci_bus_s *bus, uint32_t devfn,
                              uint8_t line, uint8_t pin)
{
  UNUSED(bus);

  return up_get_legacy_irq(devfn, line, pin);
}

/****************************************************************************
 * Name: x86_64_pci_alloc_irq
 *
 * Description:
 *  Allocate interrupts for MSI/MSI-X vector.
 *
 * Input Parameters:
 *   bus - Bus that PCI device resides
 *   irq - allocated vectors array
 *   num - number of vectors to allocate
 *   devfn - The pci device and function number
 *
 * Returned Value:
 *   >0: success, return number of allocated vectors,
 *   <0: A negative value errno
 *
 ****************************************************************************/

static int x86_64_pci_alloc_irq(struct pci_bus_s *bus, uint32_t devfn,
                                int *irq, int num)
{
  return up_alloc_irq_msi(bus->ctrl->busno, devfn, irq, num);
}

/****************************************************************************
 * Name: x86_64_pci_release_irq
 *
 * Description:
 *  Allocate interrupts for MSI/MSI-X vector.
 *
 * Input Parameters:
 *   bus - Bus that PCI device resides
 *   irq - vectors array to release
 *   num - number of vectors in array
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void x86_64_pci_release_irq(struct pci_bus_s *bus, int *irq, int num)
{
  up_release_irq_msi(irq, num);
}

/****************************************************************************
 * Name: x86_64_pci_connect_irq
 *
 * Description:
 *  Connect interrupt for MSI/MSI-X.
 *
 * Input Parameters:
 *   bus - Bus that PCI device resides
 *   irq - vectors array
 *   num - number of vectors in array
 *   mar - returned value for Message Address Register
 *   mdr - returned value for Message Data Register
 *
 * Returned Value:
 *   >0: success, 0: A positive value errno
 *
 ****************************************************************************/

static int x86_64_pci_connect_irq(struct pci_bus_s *bus, int *irq, int num,
                                  uintptr_t *mar, uint32_t *mdr)
{
  UNUSED(bus);

  return up_connect_irq(irq, num, mar, mdr);
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
