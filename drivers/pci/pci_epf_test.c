/****************************************************************************
 * drivers/pci/pci_epf_test.c
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

#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/bits.h>
#include <nuttx/compiler.h>
#include <nuttx/crc32.h>
#include <nuttx/lib/math32.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pci/pci_epf.h>
#include <nuttx/pci/pci_epc.h>
#include <nuttx/pci/pci_ids.h>
#include <nuttx/pci/pci.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PCI_EPF_TEST_FUNCTIONS                 2

#define PCI_EPF_TEST_IRQ_TYPE_LEGACY           0
#define PCI_EPF_TEST_IRQ_TYPE_MSI              1
#define PCI_EPF_TEST_IRQ_TYPE_MSIX             2

#define PCI_EPF_TEST_COMMAND_RAISE_LEGACY_IRQ  BIT(0)
#define PCI_EPF_TEST_COMMAND_RAISE_MSI_IRQ     BIT(1)
#define PCI_EPF_TEST_COMMAND_RAISE_MSIX_IRQ    BIT(2)
#define PCI_EPF_TEST_COMMAND_READ              BIT(3)
#define PCI_EPF_TEST_COMMAND_WRITE             BIT(4)
#define PCI_EPF_TEST_COMMAND_COPY              BIT(5)

#define PCI_EPF_TEST_STATUS_READ_SUCCESS       BIT(0)
#define PCI_EPF_TEST_STATUS_READ_FAIL          BIT(1)
#define PCI_EPF_TEST_STATUS_WRITE_SUCCESS      BIT(2)
#define PCI_EPF_TEST_STATUS_WRITE_FAIL         BIT(3)
#define PCI_EPF_TEST_STATUS_COPY_SUCCESS       BIT(4)
#define PCI_EPF_TEST_STATUS_COPY_FAIL          BIT(5)
#define PCI_EPF_TEST_STATUS_IRQ_RAISED         BIT(6)
#define PCI_EPF_TEST_STATUS_SRC_ADDR_INVALID   BIT(7)
#define PCI_EPF_TEST_STATUS_DST_ADDR_INVALID   BIT(8)

#define PCI_EPF_TEST_WORK_PERIOD               MSEC2TICK(10)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct begin_packed_struct pci_epf_test_reg_s
{
  uint32_t magic;
  uint32_t command;
  uint32_t status;
  uint64_t src_addr;
  uint64_t dst_addr;
  uint32_t size;
  uint32_t checksum;
  uint32_t irq_type;
  uint32_t irq_number;
  uint32_t flags;
} end_packed_struct;

struct pci_epf_test_s
{
  FAR void *reg[PCI_STD_NUM_BARS];
  FAR struct pci_epf_device_s *epf;
  int test_reg_bar;
  size_t msix_table_offset;
  struct work_s work;
  struct pci_epf_header_s header;
  int bar_size[PCI_STD_NUM_BARS];
};

/****************************************************************************
 * Private Functions Definitions
 ****************************************************************************/

static void pci_epf_test_unbind(FAR struct pci_epf_device_s *epf);
static int pci_epf_test_bind(FAR struct pci_epf_device_s *epf);
static int pci_epf_test_probe(FAR struct pci_epf_device_s *epf);
static int pci_epf_test_core_init(FAR struct pci_epf_device_s *epf);
static int pci_epf_test_link_up(FAR struct pci_epf_device_s *epf);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pci_epf_ops_s g_pci_epf_test_ops =
{
  .unbind = pci_epf_test_unbind,
  .bind   = pci_epf_test_bind,
};

static const struct pci_epf_device_id_s g_pci_epf_test_id_table[] =
{
  {.name = "pci_epf_test_0", },
  {.name = "pci_epf_test_1", },
  {}
};

static FAR const char *g_pci_epf_test_name[] =
{
  "pci_epf_test_0",
  "pci_epf_test_1",
};

static struct pci_epf_driver_s g_pci_epf_test_driver =
{
  .id_table = g_pci_epf_test_id_table,
  .probe    = pci_epf_test_probe,
  .ops      = &g_pci_epf_test_ops,
};

static const struct pci_epc_event_ops_s g_pci_epf_test_event_ops =
{
  .core_init = pci_epf_test_core_init,
  .link_up   = pci_epf_test_link_up,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int pci_epf_test_copy(FAR struct pci_epf_test_s *test)
{
  FAR struct pci_epf_device_s *epf = test->epf;
  FAR struct pci_epc_ctrl_s *epc = epf->epc;
  FAR struct pci_epf_test_reg_s *reg;
  FAR void *vsrc_addr;
  FAR void *vdst_addr;
  uintptr_t src_addr;
  uintptr_t dst_addr;
  int ret;

  reg = test->reg[test->test_reg_bar];
  vsrc_addr = pci_epc_mem_alloc_addr(epc, &src_addr, reg->size);
  if (vsrc_addr == NULL)
    {
      pcierr("Failed to allocate source address\n");
      reg->status = PCI_EPF_TEST_STATUS_SRC_ADDR_INVALID;
      return -ENOMEM;
    }

  ret = pci_epc_map_addr(epc, epf->funcno, src_addr,
                         reg->src_addr, reg->size);
  if (ret < 0)
    {
      pcierr("Failed to map source address\n");
      reg->status = PCI_EPF_TEST_STATUS_SRC_ADDR_INVALID;
      goto err_src_addr;
    }

  vdst_addr = pci_epc_mem_alloc_addr(epc, &dst_addr, reg->size);
  if (vdst_addr == NULL)
    {
      pcierr("Failed to allocate destination address\n");
      reg->status = PCI_EPF_TEST_STATUS_DST_ADDR_INVALID;
      ret = -ENOMEM;
      goto err_src_map_addr;
    }

  ret = pci_epc_map_addr(epc, epf->funcno, dst_addr,
                         reg->dst_addr, reg->size);
  if (ret < 0)
    {
      pcierr("Failed to map destination address\n");
      reg->status = PCI_EPF_TEST_STATUS_DST_ADDR_INVALID;
      goto err_dst_addr;
    }

  memcpy(vdst_addr, vsrc_addr, reg->size);
  pci_epc_unmap_addr(epc, epf->funcno, dst_addr);

err_dst_addr:
  pci_epc_mem_free_addr(epc, dst_addr, reg->size);
err_src_map_addr:
  pci_epc_unmap_addr(epc, epf->funcno, src_addr);
err_src_addr:
  pci_epc_mem_free_addr(epc, src_addr, reg->size);
  return ret;
}

static int pci_epf_test_read(FAR struct pci_epf_test_s *test)
{
  FAR struct pci_epf_device_s *epf = test->epf;
  FAR struct pci_epc_ctrl_s *epc = epf->epc;
  FAR struct pci_epf_test_reg_s *reg;
  FAR void *vsrc_addr;
  uintptr_t src_addr;
  FAR void *buf;
  int ret;

  reg = test->reg[test->test_reg_bar];
  vsrc_addr = pci_epc_mem_alloc_addr(epc, &src_addr, reg->size);
  if (vsrc_addr == NULL)
    {
      pcierr("Failed to allocate address\n");
      reg->status = PCI_EPF_TEST_STATUS_SRC_ADDR_INVALID;
      return -ENOMEM;
    }

  ret = pci_epc_map_addr(epc, epf->funcno, src_addr,
                         reg->src_addr, reg->size);
  if (ret < 0)
    {
      pcierr("Failed to map address\n");
      reg->status = PCI_EPF_TEST_STATUS_SRC_ADDR_INVALID;
      goto err_addr;
    }

  buf = kmm_zalloc(reg->size);
  if (buf == NULL)
    {
      ret = -ENOMEM;
      goto err_map_addr;
    }

  memcpy(buf, vsrc_addr, reg->size);
  if (reg->checksum != crc32part(buf, reg->size, ~0))
    {
      ret = -EIO;
    }

  kmm_free(buf);

err_map_addr:
  pci_epc_unmap_addr(epc, epf->funcno, src_addr);
err_addr:
  pci_epc_mem_free_addr(epc, src_addr, reg->size);
  return ret;
}

static int pci_epf_test_write(FAR struct pci_epf_test_s *test)
{
  FAR struct pci_epf_device_s *epf = test->epf;
  FAR struct pci_epc_ctrl_s *epc = epf->epc;
  FAR struct pci_epf_test_reg_s *reg;
  FAR void *vdst_addr;
  uintptr_t dst_addr;
  FAR void *buf;
  int ret;

  reg = test->reg[test->test_reg_bar];
  vdst_addr = pci_epc_mem_alloc_addr(epc, &dst_addr, reg->size);
  if (vdst_addr == NULL)
    {
      pcierr("Failed to allocate address\n");
      reg->status = PCI_EPF_TEST_STATUS_DST_ADDR_INVALID;
      return -ENOMEM;
    }

  ret = pci_epc_map_addr(epc, epf->funcno, dst_addr,
                         reg->dst_addr, reg->size);
  if (ret < 0)
    {
      pcierr("Failed to map address\n");
      reg->status = PCI_EPF_TEST_STATUS_DST_ADDR_INVALID;
      goto err_addr;
    }

  buf = kmm_malloc(reg->size);
  if (buf == NULL)
    {
      ret = -ENOMEM;
      goto err_map_addr;
    }

  reg->checksum = crc32part(buf, reg->size, ~0);
  memcpy(vdst_addr, buf, reg->size);
  kmm_free(buf);

err_map_addr:
  pci_epc_unmap_addr(epc, epf->funcno, dst_addr);
err_addr:
  pci_epc_mem_free_addr(epc, dst_addr, reg->size);
  return ret;
}

static void
pci_epf_test_raise_irq(FAR struct pci_epf_test_s *test,
                       uint8_t irq_type, uint16_t irq)
{
  FAR struct pci_epf_device_s *epf = test->epf;
  FAR struct pci_epc_ctrl_s *epc = epf->epc;
  FAR struct pci_epf_test_reg_s *reg;

  reg = test->reg[test->test_reg_bar];
  reg->status |= PCI_EPF_TEST_STATUS_IRQ_RAISED;

  switch (irq_type)
    {
    case PCI_EPF_TEST_IRQ_TYPE_LEGACY:
      pci_epc_raise_irq(epc, epf->funcno, PCI_EPC_IRQ_LEGACY, 0);
      break;
    case PCI_EPF_TEST_IRQ_TYPE_MSI:
      pci_epc_raise_irq(epc, epf->funcno, PCI_EPC_IRQ_MSI, irq);
      break;
    case PCI_EPF_TEST_IRQ_TYPE_MSIX:
      pci_epc_raise_irq(epc, epf->funcno, PCI_EPC_IRQ_MSIX, irq);
      break;
    default:
      pcierr("Failed to raise IRQ, unknown type\n");
      break;
    }
}

static void pci_epf_test_cmd_handler(FAR void *arg)
{
  FAR struct pci_epf_test_s *test = arg;
  FAR struct pci_epf_device_s *epf = test->epf;
  FAR struct pci_epc_ctrl_s *epc = epf->epc;
  FAR struct pci_epf_test_reg_s *reg;
  uint32_t command;
  int count;
  int ret;

  reg = test->reg[test->test_reg_bar];
  command = reg->command;
  if (!command)
    {
      goto reset_handler;
    }

  reg->command = 0;
  reg->status = 0;

  if (reg->irq_type > PCI_EPF_TEST_IRQ_TYPE_MSIX)
    {
      pcierr("Failed to detect IRQ type\n");
      goto reset_handler;
    }

  if (command & PCI_EPF_TEST_COMMAND_RAISE_LEGACY_IRQ)
    {
      reg->status = PCI_EPF_TEST_STATUS_IRQ_RAISED;
      pci_epc_raise_irq(epc, epf->funcno,
                        PCI_EPC_IRQ_LEGACY, 0);
    }
  else if (command & PCI_EPF_TEST_COMMAND_WRITE)
    {
      ret = pci_epf_test_write(test);
      if (ret < 0)
        {
          reg->status |= PCI_EPF_TEST_STATUS_WRITE_FAIL;
        }
      else
        {
          reg->status |= PCI_EPF_TEST_STATUS_WRITE_SUCCESS;
        }

      pci_epf_test_raise_irq(test, reg->irq_type, reg->irq_number);
    }
  else if (command & PCI_EPF_TEST_COMMAND_READ)
    {
      ret = pci_epf_test_read(test);
      if (ret < 0)
        {
          reg->status |= PCI_EPF_TEST_STATUS_READ_FAIL;
        }
      else
        {
          reg->status |= PCI_EPF_TEST_STATUS_READ_SUCCESS;
        }

      pci_epf_test_raise_irq(test, reg->irq_type, reg->irq_number);
    }
  else if (command & PCI_EPF_TEST_COMMAND_COPY)
    {
      ret = pci_epf_test_copy(test);
      if (ret < 0)
        {
          reg->status |= PCI_EPF_TEST_STATUS_COPY_FAIL;
        }
      else
        {
          reg->status |= PCI_EPF_TEST_STATUS_COPY_SUCCESS;
        }

      pci_epf_test_raise_irq(test, reg->irq_type, reg->irq_number);
    }
  else if (command & PCI_EPF_TEST_COMMAND_RAISE_MSI_IRQ)
    {
      count = pci_epc_get_msi(epc, epf->funcno);
      if (reg->irq_number > count || count <= 0)
        {
          goto reset_handler;
        }

      reg->status = PCI_EPF_TEST_STATUS_IRQ_RAISED;
      pci_epc_raise_irq(epc, epf->funcno,
                        PCI_EPC_IRQ_MSI, reg->irq_number);
    }
  else if (command & PCI_EPF_TEST_COMMAND_RAISE_MSIX_IRQ)
    {
      count = pci_epc_get_msix(epc, epf->funcno);
      if (reg->irq_number > count || count <= 0)
        {
          goto reset_handler;
        }

      reg->status = PCI_EPF_TEST_STATUS_IRQ_RAISED;
      pci_epc_raise_irq(epc, epf->funcno,
                        PCI_EPC_IRQ_MSIX, reg->irq_number);
    }

reset_handler:
  work_queue(HPWORK, &test->work, pci_epf_test_cmd_handler,
             test, PCI_EPF_TEST_WORK_PERIOD);
}

static void pci_epf_test_unbind(FAR struct pci_epf_device_s *epf)
{
  FAR struct pci_epf_test_s *test = epf->priv;
  FAR struct pci_epc_ctrl_s *epc = epf->epc;
  int bar;

  work_cancel(HPWORK, &test->work);
  for (bar = 0; bar < PCI_STD_NUM_BARS; bar++)
    {
      if (test->reg[bar])
        {
          pci_epc_clear_bar(epc, epf->funcno, &epf->bar[bar]);
          pci_epf_free_space(epf, bar, test->reg[bar]);
        }
    }
}

static int pci_epf_test_set_bar(FAR struct pci_epf_device_s *epf)
{
  FAR const struct pci_epc_features_s *features;
  FAR struct pci_epf_test_s *test = epf->priv;
  FAR struct pci_epc_ctrl_s *epc = epf->epc;
  int bar;
  int add;
  int ret;

  features = pci_epc_get_features(epc, epf->funcno);
  for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add)
    {
      /* pci_epc_set_bar() sets PCI_BASE_ADDRESS_MEM_TYPE_64
       * if the specific implementation required a 64-bit BAR,
       * even if we only requested a 32-bit BAR.
       */

      add = (epf->bar[bar].flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;

      if (!!(features->bar_reserved & (1 << bar)))
        {
          continue;
        }

      ret = pci_epc_set_bar(epc, epf->funcno, &epf->bar[bar]);
      if (ret < 0)
        {
          pci_epf_free_space(epf, bar, test->reg[bar]);
          pcierr("Failed to set BAR%d\n", bar);
          if (bar == test->test_reg_bar)
            {
              return ret;
            }
        }
    }

  return 0;
}

static int pci_epf_test_core_init(FAR struct pci_epf_device_s *epf)
{
  FAR struct pci_epf_header_s *header = epf->header;
  FAR const struct pci_epc_features_s *features;
  FAR struct pci_epf_test_s *test = epf->priv;
  FAR struct pci_epc_ctrl_s *epc = epf->epc;
  bool msix_capable = false;
  bool msi_capable = false;
  int ret;

  features = pci_epc_get_features(epc, epf->funcno);
  if (features != NULL)
    {
      msix_capable = features->msix_capable;
      msi_capable = features->msi_capable;
    }

  ret = pci_epc_write_header(epc, epf->funcno, header);
  if (ret < 0)
    {
      pcierr("Configuration header write failed\n");
      return ret;
    }

  ret = pci_epf_test_set_bar(epf);
  if (ret < 0)
    {
      return ret;
    }

  if (msi_capable)
    {
      ret = pci_epc_set_msi(epc, epf->funcno,
                            epf->msi_interrupts);
      if (ret < 0)
        {
          pcierr("MSI configuration failed\n");
          return ret;
        }
    }

  if (msix_capable)
    {
      ret = pci_epc_set_msix(epc, epf->funcno,
                             epf->msix_interrupts,
                             test->test_reg_bar,
                             test->msix_table_offset);
      if (ret < 0)
        {
          pcierr("MSI-X configuration failed\n");
          return ret;
        }
    }

  return 0;
}

static int pci_epf_test_link_up(FAR struct pci_epf_device_s *epf)
{
  FAR struct pci_epf_test_s *test = epf->priv;

  return work_queue(HPWORK, &test->work, pci_epf_test_cmd_handler,
                    test, PCI_EPF_TEST_WORK_PERIOD);
}

static int
pci_epf_test_alloc_space(FAR struct pci_epf_device_s *epf)
{
  FAR const struct pci_epc_features_s *features;
  FAR struct pci_epf_test_s *test = epf->priv;
  FAR struct pci_epc_ctrl_s *epc = epf->epc;
  int test_reg_bar = test->test_reg_bar;
  size_t msix_table_size = 0;
  size_t test_reg_bar_size;
  size_t test_reg_size;
  bool bar_fixed_64bit;
  size_t pba_size = 0;
  bool msix_capable;
  int bar;
  int add;

  features = pci_epc_get_features(epc, epf->funcno);
  test_reg_bar_size = ALIGN_UP(sizeof(struct pci_epf_test_reg_s), 128);
  msix_capable = features->msix_capable;
  if (msix_capable)
    {
      msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
      test->msix_table_offset = test_reg_bar_size;

      /* Align to QWORD or 8 Bytes */

      pba_size = ALIGN_UP(div_round_up(epf->msix_interrupts, 8), 8);
    }

  test_reg_size = test_reg_bar_size + msix_table_size + pba_size;
  if (test_reg_size > test->bar_size[test_reg_bar])
    {
      return -ENOMEM;
    }

  for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add)
    {
      bar_fixed_64bit = !!(features->bar_fixed_64bit & (1 << bar));
      if (bar_fixed_64bit)
        {
          epf->bar[bar].flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
        }

      add = (epf->bar[bar].flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;
      if (!!(features->bar_reserved & (1 << bar)))
        {
          continue;
        }

      test->reg[bar] = pci_epf_alloc_space(epf, bar, test->bar_size[bar],
                                           features->align);
      if (test->reg[bar] == NULL)
        {
          pcierr("Failed to allocate space for BAR%d\n", bar);
          if (bar == test_reg_bar)
            {
              return -ENOMEM;
            }
        }
    }

  return 0;
}

static int pci_epf_test_bind(FAR struct pci_epf_device_s *epf)
{
  FAR const struct pci_epc_features_s *features;
  FAR struct pci_epf_test_s *test = epf->priv;
  FAR struct pci_epc_ctrl_s *epc = epf->epc;
  int ret;

  features = pci_epc_get_features(epc, epf->funcno);
  if (features == NULL)
    {
      pcierr("features not implemented\n");
      return -ENOTSUP;
    }

  test->test_reg_bar = pci_epc_get_first_free_bar(features);
  if (test->test_reg_bar < 0)
    {
      return -EINVAL;
    }

  ret = pci_epf_test_alloc_space(epf);
  if (ret < 0)
    {
      return ret;
    }

  if (!features->core_init_notifier)
    {
      ret = pci_epf_test_core_init(epf);
      if (ret < 0)
        {
          return ret;
        }
    }

  ret = pci_epc_start(epf->epc);
  if (ret < 0)
    {
      pcierr("epc control start error\n");
      return ret;
    }

  if (!features->linkup_notifier && !features->core_init_notifier)
    {
      work_queue(HPWORK, &test->work, pci_epf_test_cmd_handler,
                 test, PCI_EPF_TEST_WORK_PERIOD);
    }

  return 0;
}

static int pci_epf_test_probe(FAR struct pci_epf_device_s *epf)
{
  FAR struct pci_epf_test_s *test = kmm_zalloc(sizeof(*test));

  if (test == NULL)
    {
      return -ENOMEM;
    }

  test->header.vendorid = 0x104c;
  test->header.deviceid = 0xb500;
  test->header.baseclass_code = PCI_CLASS_OTHERS;
  test->header.interrupt_pin  = PCI_INTERRUPT_INTA + epf->funcno;
  test->bar_size[0] = 1024;
  test->bar_size[1] = 512;
  test->bar_size[2] = 1024;
  test->bar_size[3] = 16384;
  test->bar_size[4] = 131072;
  test->bar_size[5] = 1048576;
  test->epf = epf;

  epf->header = &test->header;
  epf->priv = test;
  epf->event_ops = &g_pci_epf_test_event_ops;

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_register_epf_test_device
 *
 * Description:
 *  Init a epf device test
 *
 ****************************************************************************/

int pci_register_epf_test_device(FAR const char *epc_name)
{
  FAR struct pci_epf_device_s *epf;
  int ret;
  int i;

  epf = kmm_zalloc(sizeof(*epf) * PCI_EPF_TEST_FUNCTIONS);
  if (NULL == epf)
    {
      pcierr("create epf error\n");
      return -ENOMEM;
    }

  for (i = 0; i < PCI_EPF_TEST_FUNCTIONS; i++)
    {
      epf[i].name = g_pci_epf_test_name[i];
      epf[i].epc_name = epc_name;
      epf[i].msi_interrupts = 1;
      epf[i].msix_interrupts = 32;
      epf[i].funcno = i;
      nxmutex_init(&epf[i].lock);
      ret = pci_epf_device_register(&epf[i]);
      if (ret < 0)
        {
          pcierr("func %d link error\n", i);
          goto err;
        }
    }

  return 0;

err:
  nxmutex_destroy(&epf[i].lock);
  while (i-- > 0)
    {
      pci_epf_device_unregister(&epf[i]);
      nxmutex_destroy(&epf[i].lock);
    }

  kmm_free(epf);
  return ret;
}

/****************************************************************************
 * Name: pci_register_epf_test_driver
 *
 * Description:
 *  Init a epf test driver
 *
 ****************************************************************************/

int pci_register_epf_test_driver(void)
{
  return pci_epf_register_driver(&g_pci_epf_test_driver);
}
