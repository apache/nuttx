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

#include <debug.h>
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

/*****************************************************************************
 * Private Functions Definitions
 *****************************************************************************/

static uint32_t mem_read(FAR const volatile void *addr, int width);

static void mem_write(FAR const volatile void *addr, uint32_t val, int width);

static int qemu_pci_test_probe(FAR struct pci_bus_s *bus,
                               FAR const struct pci_dev_type_s *type,
                               uint16_t bdf);

/*****************************************************************************
 * Public Data
 *****************************************************************************/

const struct pci_dev_type_s g_pci_type_qemu_pci_test =
{
  .vendor    = 0x1b36,
  .device    = 0x0005,
  .class_rev = PCI_ID_ANY,
  .name      = "Qemu PCI test device",
  .probe     = qemu_pci_test_probe
};

/*****************************************************************************
 * Private Types
 *****************************************************************************/

struct pci_test_dev_hdr_s
{
    uint8_t  test;      /* write-only, starts a given test number */
    uint8_t  width;     /* read-only, type and width of access for a test */
    uint8_t  pad0[2];
    uint32_t offset;    /* read-only, offset in this BAR for a given test */
    uint32_t data;      /* read-only, data to use for a given test */
    uint32_t count;     /* for debugging. number of writes detected. */
    uint8_t  name[];    /* for debugging. 0-terminated ASCII string. */
};

/* Structure the read and write helpers */

struct pci_test_dev_ops_s
{
    uint32_t (*read)(FAR const volatile void *addr, int width);
    void (*write)(FAR const volatile void *addr, uint32_t val, int width);
};

/*****************************************************************************
 * Private Data
 *****************************************************************************/

static struct pci_test_dev_ops_s g_mem_ops =
{
    .read = mem_read,
    .write = mem_write
};

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

static uint32_t mem_read(FAR const volatile void *addr, int unused)
{
  return *(volatile uint32_t *)addr;
}

static void mem_write(FAR const volatile void *addr, uint32_t val, int unused)
{
  *(volatile uint32_t *)addr = val;
}

static bool qemu_pci_test_bar(FAR struct pci_test_dev_ops_s *test_ops,
                             FAR struct pci_test_dev_hdr_s *test_hdr,
                             uint16_t test_num)
{
  const int write_limit = 8;
  uint32_t  count;
  uint32_t  data;
  uint32_t  offset;
  uint8_t   width;
  int       write_cnt;
  int       i;
  char      testname[32];

  pciinfo("WRITING Test# %d %p\n", test_num, &test_hdr->test);
  test_ops->write(&test_hdr->test, test_num, 1);

  /* Reading of the string is a little ugly to handle the case where
   * we must use the port access methods.  For memory map we would
   * be able to just read directly.
   */

  testname[sizeof(testname) - 1] = 0;
  for (i = 0; i < sizeof(testname); i++)
    {
      testname[i] = (char)test_ops->read((void *)&test_hdr->name + i, 1);
      if (testname[i] == 0)
        {
          break;
        }
    }

  pciinfo("Running test: %s\n", testname);

  count = test_ops->read(&test_hdr->count, 4);
  pciinfo("COUNT: %04x\n", count);
  if (count != 0)
    {
      return false;
    }

  width = test_ops->read(&test_hdr->width, 1);
  pciinfo("Width: %d\n", width);

  if (width == 0 || width > 4)
    {
      return false;
    }

  data = test_ops->read(&test_hdr->data, 4);
  pciinfo("Data: %04x\n", data);

  offset = test_ops->read(&test_hdr->offset, 4);
  pciinfo("Offset: %04x\n", offset);

  for (write_cnt = 0; write_cnt < write_limit; write_cnt++)
    {
      pciinfo("Issuing WRITE to %p %x %d\n",
              (void *)test_hdr + offset,
              data, width);
      test_ops->write((void *)test_hdr + offset, data, width);
    }

  count = test_ops->read(&test_hdr->count, 4);
  pciinfo("COUNT: %04x\n", count);

  if (!count)
    {
      return true;
    }

  return (int)count == write_cnt;
}

/*****************************************************************************
 * Name: qemu_pci_test_probe
 *
 * Description:
 *   Initialize device
 *
 *****************************************************************************/

static int qemu_pci_test_probe(FAR struct pci_bus_s *bus,
                               FAR const struct pci_dev_type_s *type,
                               uint16_t bdf)
{
  struct pci_dev_s           dev;
  struct pci_test_dev_ops_s  io_ops;
  struct pci_test_dev_ops_s *test_ops;
  struct pci_test_dev_hdr_s *test_hdr;
  uint8_t                    bar_id;
  uint32_t                   bar;
  uint64_t                   bar_addr;
  uint16_t                   test_cnt;

  /* Get dev */

  dev.bus  = bus;
  dev.type = type;
  dev.bdf  = bdf;

  /* Get io ops */

  io_ops.read  = bus->ops->pci_io_read;
  io_ops.write = bus->ops->pci_io_write;

  pci_enable_bus_master(&dev);
  pciinfo("Enabled bus mastering\n");
  pci_enable_io(&dev, PCI_SYS_RES_MEM);
  pci_enable_io(&dev, PCI_SYS_RES_IOPORT);
  pciinfo("Enabled i/o port and memory resources\n");

  for (bar_id = 0; bar_id < PCI_BAR_CNT; bar_id++)
    {
      /* Need to query the BAR for IO vs MEM
       * Also handle if the bar is 64bit address
       */

      if (pci_bar_valid(&dev, bar_id) != OK)
        {
          continue;
        }

      bar = bus->ops->pci_cfg_read(&dev,
          PCI_HEADER_NORM_BAR0 + (bar_id * 4), 4);

      bar_addr = pci_bar_addr(&dev, bar_id);
      test_hdr = (struct pci_test_dev_hdr_s *)bar_addr;

      if ((bar & PCI_BAR_LAYOUT_MASK) == PCI_BAR_LAYOUT_MEM)
        {
          test_ops = &g_mem_ops;

          /* If the BAR is MMIO the it must be mapped */

          bus->ops->pci_map_bar(bar_addr, pci_bar_size(&dev, bar_id));
        }
      else
        {
          test_ops = &io_ops;
        }

      for (test_cnt = 0; test_cnt < 0xffff; test_cnt++)
        {
          if (!qemu_pci_test_bar(test_ops, test_hdr, test_cnt))
            {
              break;
            }

          pciinfo("Test Completed BAR [%d] TEST [%d]\n", bar_id, test_cnt);
        }

      if (pci_bar_is_64(&dev, bar_id))
        {
          bar_id++;
        }
    }

  return OK;
}
