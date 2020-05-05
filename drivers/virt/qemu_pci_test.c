/*****************************************************************************
 * drivers/virt/qemu_pci_test.c
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

#include <nuttx/pcie/pcie.h>
#include <nuttx/virt/qemu_pci.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/*****************************************************************************
 * Private Types
 *****************************************************************************/

struct pci_test_dev_hdr_s
{
    volatile uint8_t test;       /* write-only, starts a given test number */
    volatile uint8_t width_type; /* read-only, type and width of access for a given test.
                                  * 1,2,4 for byte,word or long write.
                                  * any other value if test not supported on this BAR */
    volatile uint8_t pad0[2];
    volatile uint32_t offset;    /* read-only, offset in this BAR for a given test */
    volatile uint32_t data;      /* read-only, data to use for a given test */
    volatile uint32_t count;     /* for debugging. number of writes detected. */
    volatile uint8_t name[];     /* for debugging. 0-terminated ASCII string. */
};

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: qemu_pci_test_probe
 *
 * Description:
 *   Initialize device
 *****************************************************************************/

int qemu_pci_test_probe(FAR struct pcie_bus_s *bus,
                        FAR struct pcie_dev_type_s *type, uint16_t bdf)
{
  uint32_t bar[2];
  struct pcie_dev_s dev =
    {
      .bus = bus,
      .type = type,
      .bdf = bdf,
    };

  pci_enable_device(&dev, (PCI_CMD_MASTER | PCI_CMD_MEM));

  for (int ii = 0; ii < 2; ii++)
    {
      pci_get_bar(&dev, ii, bar + ii);

      if ((bar[ii] & PCI_BAR_IO) != PCI_BAR_IO)
        {
          pciinfo("Mapping BAR%d: %x\n", ii, bar[ii]);

          pci_map_bar(&dev, ii, 0x1000, NULL);

          struct pci_test_dev_hdr_s *ptr =
            (struct pci_test_dev_hdr_s *)(uintptr_t)bar[ii];

          int i = 0;
          while (1)
            {
              ptr->test = i;

              if (ptr->width_type != 1 &&
                  ptr->width_type != 2 &&
                  ptr->width_type != 4)
                break;

              pciinfo("Test[%d] Size:%d %s\n",
                  i, ptr->width_type,
                  ptr->name);

              i++;
            }
        }
    }

  return OK;
}

/*****************************************************************************
 * Public Data
 *****************************************************************************/

struct pcie_dev_type_s pcie_type_qemu_pci_test =
{
    .vendor = 0x1b36,
    .device = 0x0005,
    .class_rev = PCI_ID_ANY,
    .name = "Qemu PCI test device",
    .probe = qemu_pci_test_probe
};
