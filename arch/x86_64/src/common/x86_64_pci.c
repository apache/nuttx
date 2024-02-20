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

#define PCI_CFG_ADDR       0xcf8
#define PCI_DATA_ADDR      0xcfc
#define PCI_CFG_EN         (1 << 31)

/****************************************************************************
 * Private Functions Definitions
 ****************************************************************************/

static void x86_64_pci_cfg_write(struct pci_dev_s *dev, int reg,
                               uint32_t val, int width);
static uint32_t x86_64_pci_cfg_read(struct pci_dev_s *dev, int reg,
                                  int width);
static int x86_64_pci_map_bar(uint64_t addr, uint64_t len);
static uint32_t x86_64_pci_io_read(const volatile void *addr, int width);
static void x86_64_pci_io_write(const volatile void *addr, uint32_t val,
                              int width);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pci_bus_ops_s g_x86_64_pci_bus_ops =
{
  .pci_cfg_write = x86_64_pci_cfg_write,
  .pci_cfg_read  = x86_64_pci_cfg_read,
  .pci_map_bar   = x86_64_pci_map_bar,
  .pci_io_read   = x86_64_pci_io_read,
  .pci_io_write  = x86_64_pci_io_write,
};

static struct pci_bus_s g_x86_64_pci_bus =
{
  .ops = &g_x86_64_pci_bus_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_pci_cfg_write
 *
 * Description:
 *  Write 8, 16, 32, 64 bits data to PCI-E configuration space of device
 *  specified by dev
 *
 * Input Parameters:
 *   bdf  - Device private data
 *   reg  - A pointer to the read-only buffer of data to be written
 *   size - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static void x86_64_pci_cfg_write(struct pci_dev_s *dev, int reg,
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
 * Name: x86_64_pci_cfg_read
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

static uint32_t x86_64_pci_cfg_read(struct pci_dev_s *dev, int reg,
                                  int width)
{
  uint32_t ret;
  uint8_t  offset_mask = 4 - width;

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

static uint32_t x86_64_pci_io_read(const volatile void *addr, int width)
{
  uint16_t portaddr = (uint16_t)(intptr_t)addr;

  switch (width)
  {
    case 1:
      return (uint32_t)inb(portaddr);
    case 2:
      return (uint32_t)inw(portaddr);
    case 4:
      return (uint32_t)inl(portaddr);
    default:
      pcierr("Invalid read width %d\n", width);
      DEBUGPANIC();
  }

  return 0;
}

static void x86_64_pci_io_write(const volatile void *addr, uint32_t val,
                              int width)
{
  uint16_t portaddr = (uint16_t)(intptr_t)addr;

  switch (width)
  {
    case 1:
      outb((uint8_t)val, portaddr);
      return;
    case 2:
      outw((uint16_t)val, portaddr);
      return;
    case 4:
      outl((uint32_t)val, portaddr);
      return;
    default:
      pcierr("Invalid write width %d\n", width);
      DEBUGPANIC();
  }
}

static int x86_64_pci_map_bar(uint64_t addr, uint64_t len)
{
  up_map_region((void *)(uintptr_t)addr, len,
      X86_PAGE_WR | X86_PAGE_PRESENT | X86_PAGE_NOCACHE | X86_PAGE_GLOBAL);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_pci_init
 *
 * Description:
 *  Initialize the PCI-E bus
 *
 ****************************************************************************/

void x86_64_pci_init(void)
{
  pciinfo("Initializing PCI Bus\n");
  pci_initialize(&g_x86_64_pci_bus);
}
