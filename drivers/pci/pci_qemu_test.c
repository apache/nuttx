/****************************************************************************
 * drivers/pci/pci_qemu_test.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdint.h>

#include <nuttx/pci/pci.h>
#include <nuttx/pci/pci_qemu_test.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pci_qemu_test_hdr_s
{
  uint8_t  test;    /* Write-only, starts a given test number */
  uint8_t  width;   /* Read-only, type and width of access for a test */
  uint8_t  pad0[2];
  uint32_t offset;  /* Read-only, offset in this BAR for a given test */
  uint32_t data;    /* Read-only, data to use for a given test */
  uint32_t count;   /* For debugging. number of writes detected. */
  uint8_t  name[];  /* For debugging. 0-terminated ASCII string. */
};

/* Structure the read and write helpers */

struct pci_qemu_test_ops_s
{
  CODE uint32_t (*read)(FAR struct pci_bus_s *bus, FAR void *addr, int size);
  CODE int (*write)(FAR struct pci_bus_s *bus, FAR void *addr, uint32_t val,
                    int size);
};

/****************************************************************************
 * Private Functions Definitions
 ****************************************************************************/

static uint32_t pci_qemu_test_read_mem(FAR struct pci_bus_s *bus,
                                       FAR void *addr, int size);
static int pci_qemu_test_write_mem(FAR struct pci_bus_s *bus, FAR void *addr,
                                   uint32_t val, int size);
static uint32_t pci_qemu_test_read_io(FAR struct pci_bus_s *bus,
                                      FAR void *addr, int size);
static int pci_qemu_test_write_io(FAR struct pci_bus_s *bus, FAR void *addr,
                                  uint32_t val, int size);
static int pci_qemu_test_probe(FAR struct pci_device_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pci_qemu_test_ops_s g_pci_qemu_test_mem_ops =
{
  pci_qemu_test_read_mem, /* read */
  pci_qemu_test_write_mem /* write */
};

static const struct pci_qemu_test_ops_s g_pci_qemu_test_io_ops =
{
  pci_qemu_test_read_io, /* read */
  pci_qemu_test_write_io /* write */
};

static const struct pci_device_id_s g_pci_qemu_test_id_table[] =
{
  { PCI_DEVICE(0x1b36, 0x0005), },
  { }
};

static struct pci_driver_s g_pci_qemu_test_drv =
{
  .id_table = g_pci_qemu_test_id_table,
  .probe    = pci_qemu_test_probe,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_qemu_test_read_mem
 *
 * Description:
 *   This function is used to read mem register.
 *
 ****************************************************************************/

static uint32_t pci_qemu_test_read_mem(FAR struct pci_bus_s *bus,
                                       FAR void *addr, int size)
{
  if (size == 1)
    {
      return *(FAR volatile uint8_t *)addr;
    }
  else if (size == 2)
    {
      return *(FAR volatile uint16_t *)addr;
    }
  else if (size == 4)
    {
      return *(FAR volatile uint32_t *)addr;
    }

  DEBUGPANIC();
  return 0;
}

/****************************************************************************
 * Name: pci_qemu_test_write_mem
 *
 * Description:
 *   This function is used to write a value to mem register.
 *
 ****************************************************************************/

static int pci_qemu_test_write_mem(FAR struct pci_bus_s *bus, FAR void *addr,
                                   uint32_t val, int size)
{
  if (size == 1)
    {
      *(FAR volatile uint8_t *)addr = (uint8_t)val;
    }
  else if (size == 2)
    {
      *(FAR volatile uint16_t *)addr = (uint16_t)val;
    }
  else if (size == 4)
    {
      *(FAR volatile uint32_t *)addr = (uint32_t)val;
    }
  else
    {
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: pci_qemu_test_read_io
 ****************************************************************************/

static uint32_t pci_qemu_test_read_io(FAR struct pci_bus_s *bus,
                                      FAR void *addr, int size)
{
  uint32_t val;
  int ret;

  ret = bus->ctrl->ops->read_io(bus, (uintptr_t)addr, size, &val);
  if (ret < 0)
    {
      pcierr("Read io failed, ret=%d\n", ret);
      return 0;
    }

  return val;
}

/****************************************************************************
 * Name: pci_qemu_test_write_io
 ****************************************************************************/

static int pci_qemu_test_write_io(FAR struct pci_bus_s *bus, FAR void *addr,
                                  uint32_t val, int size)
{
  return bus->ctrl->ops->write_io(bus, (uintptr_t)addr, size, val);
}

/****************************************************************************
 * Name: pci_qemu_test_bar
 *
 * Description:
 *   The pci bar test demo in the qemu environment
 *
 ****************************************************************************/

static bool pci_qemu_test_bar(FAR struct pci_device_s *dev,
                              FAR const struct pci_qemu_test_ops_s *ops,
                              FAR struct pci_qemu_test_hdr_s *hdr,
                              uint8_t num)
{
  const uint32_t write_limit = 8;
  uint32_t write_cnt;
  uint32_t offset;
  uint32_t count;
  uint32_t data;
  uint8_t width;
  char name[32];
  int i;

  pciinfo("WRITING Test# %u %p\n", num, &hdr->test);
  ops->write(dev->bus, &hdr->test, num, sizeof(num));

  /* Reading of the string is a little ugly to handle the case where
   * we must use the port access methods.  For memory map we would
   * be able to just read directly.
   */

  name[sizeof(name) - 1] = 0;
  for (i = 0; i < sizeof(name); i++)
    {
      name[i] = (char)ops->read(dev->bus, (FAR char *)&hdr->name + i, 1);
      if (name[i] == 0)
        {
          break;
        }
    }

  pciinfo("Running test: %s\n", name);

  count = ops->read(dev->bus, &hdr->count, sizeof(count));
  pciinfo("Start Count: %04" PRIu32 "\n", count);

  if (count != 0)
    {
      return false;
    }

  width = ops->read(dev->bus, &hdr->width, sizeof(width));
  pciinfo("Width: %d\n", width);

  if (width == 0 || width > 4)
    {
      return false;
    }

  data = ops->read(dev->bus, &hdr->data, sizeof(data));
  offset = ops->read(dev->bus, &hdr->offset, sizeof(offset));
  pciinfo("Data: 0x%04" PRIx32 " Offset: 0x%04" PRIx32 "\n", data, offset);

  for (write_cnt = 0; write_cnt < write_limit; write_cnt++)
    {
      pciinfo("[%" PRIu32 "]Issuing WRITE to %p Data: 0x%04" PRIx32
              " Width: %u\n",
              write_cnt, (FAR char *)hdr + offset, data, width);
      ops->write(dev->bus, (FAR char *)hdr + offset, data, width);
    }

  count = ops->read(dev->bus, &hdr->count, sizeof(count));
  pciinfo("End Count: %04" PRIu32 "\n", count);

  if (count == 0)
    {
      return true;
    }

  return count == write_cnt;
}

/****************************************************************************
 * Name: pci_qemu_test_probe
 *
 * Description:
 *   Initialize device
 *
 ****************************************************************************/

static int pci_qemu_test_probe(FAR struct pci_device_s *dev)
{
  FAR const struct pci_qemu_test_ops_s *ops;
  FAR struct pci_qemu_test_hdr_s *hdr;
  unsigned long flags;
  uint8_t test_cnt;
  int bar;
  int ret;

  pciinfo("Enter pci test probe.\n");

  ret = pci_enable_device(dev);
  if (ret < 0)
    {
      pcierr("Enable device failed, ret=%d\n", ret);
      return ret;
    }

  pci_set_master(dev);

  for (bar = 0; bar < PCI_NUM_RESOURCES; bar++)
    {
      hdr = (FAR struct pci_qemu_test_hdr_s *)pci_map_bar(dev, bar);
      if (hdr == NULL)
        {
          continue;
        }

      flags = pci_resource_flags(dev, bar);
      if ((flags & PCI_RESOURCE_MEM) == PCI_RESOURCE_MEM)
        {
          ops = &g_pci_qemu_test_mem_ops;
        }
      else if ((flags & PCI_RESOURCE_IO) == PCI_RESOURCE_IO)
        {
          ops = &g_pci_qemu_test_io_ops;
        }
      else
        {
          PANIC();
        }

      for (test_cnt = 0; test_cnt < 0xff; test_cnt++)
        {
          if (!pci_qemu_test_bar(dev, ops, hdr, test_cnt))
            {
              break;
            }
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_register_qemu_test_driver
 *
 * Description:
 *   Register a pci driver
 *
 ****************************************************************************/

int pci_register_qemu_test_driver(void)
{
  return pci_register_driver(&g_pci_qemu_test_drv);
}
