/*****************************************************************************
 * drivers/virt/qemu_edu.c
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>

#include <nuttx/pci/pci.h>
#include <nuttx/virt/qemu_pci.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* Registers defined for device.  Size 4 for < 0x80.  Size 8 for >= 0x80. */

#define EDU_REG_ID          0x00  /* Identification */
#define EDU_REG_LIVE        0x04  /* Liveness Check */
#define EDU_REG_FAC         0x08  /* Factorial Computation */
#define EDU_REG_STATUS      0x20  /* Status */
#define EDU_REG_INT_STATUS  0x24  /* Interupt Status */
#define EDU_REG_INT_RAISE   0x60  /* Raise an interrupt */
#define EDU_REG_INT_ACK     0x64  /* Acknowledge interrupt */
#define EDU_REG_DMA_SOURCE  0x80  /* Source address for DMA transfer */
#define EDU_REG_DMA_DEST    0x88  /* Destination address for DMA transfer */
#define EDU_REG_DMA_COUNT   0x90  /* Size of area to transfer with DMA */
#define EDU_REG_DMA_CMD     0x98  /* Control DMA tranfer */

#define EDU_CONTROL_BAR_ID      0
#define EDU_CONTROL_BAR_OFFSET  PCI_HEADER_NORM_BAR0

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/*****************************************************************************
 * Private Functions Definitions
 *****************************************************************************/

static void qemu_edu_test_poll(FAR struct pci_dev_s *dev,
                               uintptr_t base_addr);

/*****************************************************************************
 * Private Data
 *****************************************************************************/

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: qemu_edu_write_reg32
 *
 * Description:
 *   Provide a write interface for 32bit mapped registers
 *
 * Input Parameters:
 *   addr - Register address
 *   val  - Value to assign to register
 *
 *****************************************************************************/

static void qemu_edu_write_reg32(uintptr_t addr, uint32_t val)
{
  *(volatile uint32_t *)addr = val;
}

/*****************************************************************************
 * Name: qemu_edu_read_reg32
 *
 * Description:
 *   Provide a read interface for 32bit mapped registers
 *
 * Returned Value:
 *   Register value
 *
 *****************************************************************************/

static uint32_t qemu_edu_read_reg32(uintptr_t addr)
{
  return *(volatile uint32_t *)addr;
}

/*****************************************************************************
 * Name: qemu_edu_test_poll
 *
 * Description:
 *   Performs basic functional test of PCI device and MMIO using polling
 *   of mapped register interfaces.
 *
 * Input Parameters:
 *   bus       - An PCI device
 *   base_addr - Base address of device register space
 *
 *****************************************************************************/

static void qemu_edu_test_poll(FAR struct pci_dev_s *dev, uintptr_t base_addr)
{
  uint32_t test_value;
  uint32_t test_read;

  pciinfo("Identification: 0x%04xu\n",
          qemu_edu_read_reg32(base_addr + EDU_REG_ID));

  test_value = 0xdeadbeef;
  qemu_edu_write_reg32(base_addr + EDU_REG_LIVE, test_value);
  test_read = qemu_edu_read_reg32(base_addr + EDU_REG_LIVE);
  pciinfo("Live Check: Wrote: 0x%08x Read: 0x%08x Error Bits 0x%08x\n",
          test_value, test_read, test_read ^ ~test_value);

  test_value = 10;
  qemu_edu_write_reg32(base_addr + EDU_REG_FAC, test_value);
  qemu_edu_write_reg32(base_addr + EDU_REG_STATUS, 0);
  while (qemu_edu_read_reg32(base_addr + EDU_REG_STATUS) & 0x01)
    {
      pciinfo("Waiting to compute factorial...");
      usleep(10000);
    }

  test_read = qemu_edu_read_reg32(base_addr + EDU_REG_FAC);
  pciinfo("Computed factorial of %d as %d\n", test_value, test_read);
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: qemu_edu_probe
 *
 * Description:
 *   Initialize device
 *****************************************************************************/

int qemu_edu_probe(FAR struct pci_bus_s *bus,
                   FAR struct pci_dev_type_s *type, uint16_t bdf)
{
  uint32_t bar;
  uintptr_t bar_addr;
  struct pci_dev_s dev =
    {
      .bus = bus,
      .type = type,
      .bdf = bdf,
    };

  pci_enable_bus_master(&dev);
  pciinfo("Enabled bus mastering\n");
  pci_enable_io(&dev, PCI_SYS_RES_MEM);
  pciinfo("Enabled memory resources\n");

  if (pci_bar_valid(&dev, EDU_CONTROL_BAR_ID) != OK)
    {
      pcierr("Control BAR is not valid\n");
      DEBUGPANIC();
      return -EINVAL;
    }

  bar_addr = pci_bar_addr(&dev, EDU_CONTROL_BAR_ID);
  bar = bus->ops->pci_cfg_read(&dev, EDU_CONTROL_BAR_OFFSET, 4);
  if ((bar & PCI_BAR_LAYOUT_MASK) != PCI_BAR_LAYOUT_MEM)
    {
      pcierr("Control bar expected to be MMIO\n");
      DEBUGPANIC();
      return -EINVAL;
    }

  if (bus->ops->pci_map_bar(bar_addr,
      pci_bar_size(&dev, EDU_CONTROL_BAR_ID)) != OK)
    {
      pcierr("Failed to map address space\n");
      DEBUGPANIC();
      return -EINVAL;
    }

  pciinfo("Device Initialized\n");

  qemu_edu_test_poll(&dev, bar_addr);

  /* TODO: Configure MSI and run interrupt test functions */

  /* TODO: Configure DMA transfer run DMA test functions */

  return OK;
}

/*****************************************************************************
 * Public Data
 *****************************************************************************/

struct pci_dev_type_s pci_type_qemu_edu =
{
    .vendor = 0x1234,
    .device = 0x11e8,
    .class_rev = PCI_ID_ANY,
    .name = "Qemu PCI EDU device",
    .probe = qemu_edu_probe
};
