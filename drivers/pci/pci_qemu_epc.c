/****************************************************************************
 * drivers/pci/pci_qemu_epc.c
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

#include <debug.h>

#include <nuttx/bits.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pci/pci.h>
#include <nuttx/pci/pci_epc.h>
#include <nuttx/pci/pci_epf.h>

#include "pci_drivers.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The epc register description:
 * https://github.com/ShunsukeMie/qemu/blob/epf-bridge/v1/hw/misc/qemu-epc.c
 */

#define QEMU_EPC_BAR_CTRL               0
#define QEMU_EPC_BAR_PCI_CFG            1
#define QEMU_EPC_BAR_BAR_CFG            2
#define QEMU_EPC_BAR_WINDOW             3

#define QEMU_EPC_CTRL_OFF_START         0x00
#define QEMU_EPC_CTRL_OFF_WIN_START     0x08
#define QEMU_EPC_CTRL_OFF_WIN_SIZE      0x10
#define QEMU_EPC_CTRL_OFF_IRQ_TYPE      0x18
#define QEMU_EPC_CTRL_OFF_IRQ_NUM       0x1c
#define QEMU_EPC_CTRL_OFF_OB_MAP_MASK   0x20
#define QEMU_EPC_CTRL_OFF_OB_IDX        0x24
#define QEMU_EPC_CTRL_OFF_OB_MAP_PHYS   0x28
#define QEMU_EPC_CTRL_OFF_OB_MAP_PCI    0x30
#define QEMU_EPC_CTRL_OFF_OB_MAP_SIZE   0x38

#define QEMU_EPC_BAR_CFG_OFF_MASK        0x00
#define QEMU_EPC_BAR_CFG_OFF_NUMBER      0x01
#define QEMU_EPC_BAR_CFG_OFF_FLAGS       0x02
#define QEMU_EPC_BAR_CFG_OFF_RSV         0x04
#define QEMU_EPC_BAR_CFG_OFF_PHYS_ADDR   0x08
#define QEMU_EPC_BAR_CFG_OFF_SIZE        0x10
#define QEMU_EPC_BAR_CFG_SIZE            0x8000

#define QEMU_EPC_BAR_CFG_MSI             0x40
#define QEMU_EPC_BAR_CFG_MSIX            0x60

#define QEMU_EPC_CONFIG_SPACE_SIZE       0x1000
#define QEMU_EPC_MAX_FUNCTIONS           0x8
#define QEMU_EPC_ALIGN_PAGE_SIZE         0x1000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct qemu_epc_s
{
  FAR struct pci_epc_ctrl_s *epc;
  FAR struct pci_device_s *pdev;
  FAR void *ctl_base;
  FAR void *cfg_base;
  FAR void *bar_base;
  FAR void *msi_vaddr;
  uintptr_t msi_paddr;
  uint32_t msi_data;
  FAR void *msix_vaddr;
  FAR void *msix_vob;
  uintptr_t msix_pob;
  uint64_t ob_phys[32];
};

/****************************************************************************
 * Private Functions Definitions
 ****************************************************************************/

static int qemu_epc_probe(FAR struct pci_device_s *dev);
static int
qemu_epc_write_header(FAR struct pci_epc_ctrl_s *epc,
                      uint8_t funcno, FAR struct pci_epf_header_s *hdr);
static int qemu_epc_set_bar(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                            FAR struct pci_epf_bar_s *bar);
static void
qemu_epc_clear_bar(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                   FAR struct pci_epf_bar_s *bar);
static int
qemu_epc_map_addr(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                  uintptr_t addr, uint64_t pci_addr, size_t size);
static void
qemu_epc_unmap_addr(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                    uintptr_t addr);
static int
qemu_epc_raise_irq(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                   enum pci_epc_irq_type_e type, uint16_t interrupt_num);
static int qemu_epc_start(FAR struct pci_epc_ctrl_s *epc);
static const FAR struct pci_epc_features_s *
qemu_epc_get_features(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno);
static int qemu_epc_set_msi(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                            uint8_t interrupts);
static int qemu_epc_get_msi(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno);
static int qemu_epc_get_msix(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno);
static int
qemu_epc_set_msix(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                  uint16_t interrupts, int barno, uint32_t offset);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pci_device_id_s g_qemu_epc_id_table[] =
{
  { PCI_DEVICE(0x1b36, 0x0013), },
  { }
};

static struct pci_driver_s g_qemu_epc_drv =
{
  .id_table = g_qemu_epc_id_table,
  .probe    = qemu_epc_probe,
};

static const struct pci_epc_features_s g_qemu_epc_features =
{
  .linkup_notifier = false,
  .core_init_notifier = false,
  .msi_capable = true,
  .msix_capable = true,
};

static const struct pci_epc_ops_s g_qemu_epc_ops =
{
  .write_header = qemu_epc_write_header,
  .set_bar      = qemu_epc_set_bar,
  .clear_bar    = qemu_epc_clear_bar,
  .map_addr     = qemu_epc_map_addr,
  .unmap_addr   = qemu_epc_unmap_addr,
  .raise_irq    = qemu_epc_raise_irq,
  .start        = qemu_epc_start,
  .get_features = qemu_epc_get_features,
  .set_msi      = qemu_epc_set_msi,
  .get_msi      = qemu_epc_get_msi,
  .set_msix     = qemu_epc_set_msix,
  .get_msix     = qemu_epc_get_msix,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void qemu_epc_ctl_write8(FAR struct qemu_epc_s *qep,
                                unsigned offset, uint8_t value)
{
  pci_write_mmio_byte(qep->pdev, qep->ctl_base + offset, value);
}

static void qemu_epc_ctl_write32(FAR struct qemu_epc_s *qep,
                                 unsigned offset, uint32_t value)
{
  pci_write_mmio_dword(qep->pdev, qep->ctl_base + offset, value);
}

static void qemu_epc_ctl_write64(FAR struct qemu_epc_s *qep,
                                 unsigned offset, uint64_t value)
{
  pci_write_mmio_qword(qep->pdev, qep->ctl_base + offset, value);
}

static uint32_t
qemu_epc_ctl_read32(FAR struct qemu_epc_s *qep, unsigned offset)
{
  uint32_t val;

  pci_read_mmio_dword(qep->pdev, qep->ctl_base + offset, &val);
  return val;
}

static uint64_t
qemu_epc_ctl_read64(FAR struct qemu_epc_s *qep, unsigned offset)
{
  uint64_t val;

  pci_read_mmio_qword(qep->pdev, qep->ctl_base + offset, &val);
  return val;
}

static uint16_t qemu_epc_cfg_read16(FAR struct qemu_epc_s *qep,
                                    unsigned offset)
{
  uint16_t val;

  pci_read_mmio_word(qep->pdev, qep->cfg_base + offset, &val);
  return val;
}

static uint32_t qemu_epc_cfg_read32(FAR struct qemu_epc_s *qep,
                                    unsigned offset)
{
  uint32_t val;

  pci_read_mmio_dword(qep->pdev, qep->cfg_base + offset, &val);
  return val;
}

static uint64_t qemu_epc_cfg_read64(FAR struct qemu_epc_s *qep,
                                    unsigned offset)
{
  uint64_t val;

  pci_read_mmio_qword(qep->pdev, qep->cfg_base + offset, &val);
  return val;
}

static void qemu_epc_cfg_write8(FAR struct qemu_epc_s *qep,
                                unsigned offset, uint8_t value)
{
  pci_write_mmio_byte(qep->pdev, qep->cfg_base + offset, value);
}

static void qemu_epc_cfg_write16(FAR struct qemu_epc_s *qep,
                                 unsigned offset, uint16_t value)
{
  pci_write_mmio_word(qep->pdev, qep->cfg_base + offset, value);
}

static void qemu_epc_cfg_write32(FAR struct qemu_epc_s *qep,
                                 unsigned offset, uint32_t value)
{
  pci_write_mmio_dword(qep->pdev, qep->cfg_base + offset, value);
}

static uint8_t qemu_epc_bar_cfg_read8(FAR struct qemu_epc_s *qep,
                                      unsigned offset)
{
  uint8_t val;

  pci_read_mmio_byte(qep->pdev, qep->bar_base + offset, &val);
  return val;
}

static void qemu_epc_bar_cfg_write8(FAR struct qemu_epc_s *qep,
                                    unsigned offset, uint8_t value)
{
  pci_write_mmio_byte(qep->pdev, qep->bar_base + offset, value);
}

static void qemu_epc_bar_cfg_write64(FAR struct qemu_epc_s *qep,
                                     unsigned offset, uint64_t value)
{
  pci_write_mmio_qword(qep->pdev, qep->bar_base + offset, value);
}

static uint32_t pci_qep_func_base(FAR struct qemu_epc_s *qep,
                                  uint8_t funcno)
{
  return QEMU_EPC_CONFIG_SPACE_SIZE * funcno;
}

/****************************************************************************
 * Name: qemu_epc_write_header
 *
 * Description:
 *   This function is used to set standard pci header.
 *
 * Input Parameters:
 *   epc    - Device name of the endpoint controller
 *   funcno - The epc's function number
 *   hdr    - The standard pci header
 *
 * Returned Value:
 *   Return 0
 ****************************************************************************/

static int
qemu_epc_write_header(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                      FAR struct pci_epf_header_s *hdr)
{
  FAR struct qemu_epc_s *qep = epc->priv;
  uint32_t base;

  base = pci_qep_func_base(qep, funcno);
  qemu_epc_cfg_write16(qep, base + PCI_VENDOR_ID, hdr->vendorid);
  qemu_epc_cfg_write16(qep, base + PCI_DEVICE_ID, hdr->deviceid);
  qemu_epc_cfg_write8(qep, base + PCI_REVISION_ID, hdr->revid);
  qemu_epc_cfg_write8(qep, base + PCI_CLASS_PROG, hdr->progif_code);
  qemu_epc_cfg_write8(qep, base + PCI_HEADER_TYPE, 0x80);
  qemu_epc_cfg_write8(qep, base + PCI_CLASS_DEVICE, hdr->baseclass_code);
  qemu_epc_cfg_write8(qep, base + PCI_CLASS_DEVICE + 1, hdr->subclass_code);
  qemu_epc_cfg_write8(qep, base + PCI_CACHE_LINE_SIZE, hdr->cache_line_size);
  qemu_epc_cfg_write8(qep, base + PCI_SUBSYSTEM_VENDOR_ID,
                      hdr->subsys_vendor_id);
  qemu_epc_cfg_write8(qep, base + PCI_SUBSYSTEM_ID, hdr->subsys_id);
  qemu_epc_cfg_write8(qep, base + PCI_INTERRUPT_PIN, hdr->interrupt_pin);

  return 0;
}

/****************************************************************************
 * Name: qemu_epc_set_bar
 *
 * Description:
 *   This function is used to create inbound mapping.
 *
 * Input Parameters:
 *   epc    - Device name of the endpoint controller
 *   funcno - The epc's function number
 *   bar    - The bar is used mapping
 *
 * Returned Value:
 *   Return 0
 ****************************************************************************/

static int qemu_epc_set_bar(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                            FAR struct pci_epf_bar_s *bar)
{
  FAR struct qemu_epc_s *qep = epc->priv;
  uint32_t base;
  uint8_t mask;

  base = pci_qep_func_base(qep, funcno);
  qemu_epc_bar_cfg_write8(qep, base + QEMU_EPC_BAR_CFG_OFF_NUMBER,
                          bar->barno);
  qemu_epc_bar_cfg_write64(qep, base + QEMU_EPC_BAR_CFG_OFF_PHYS_ADDR,
                           bar->phys_addr);
  qemu_epc_bar_cfg_write64(qep, base + QEMU_EPC_BAR_CFG_OFF_SIZE, bar->size);
  qemu_epc_bar_cfg_write8(qep, base + QEMU_EPC_BAR_CFG_OFF_FLAGS,
                          bar->flags);
  mask = qemu_epc_bar_cfg_read8(qep, base + QEMU_EPC_BAR_CFG_OFF_MASK)
                                | BIT(bar->barno);
  qemu_epc_bar_cfg_write8(qep, base + QEMU_EPC_BAR_CFG_OFF_MASK, mask);

  return 0;
}

/****************************************************************************
 * Name: qemu_epc_clear_bar
 *
 * Description:
 *   This function is used to clear inbound mapping.
 *
 * Input Parameters:
 *   epc    - Device name of the endpoint controller
 *   funcno - The epc's function number
 *   bar    - The bar is used mapping
 ****************************************************************************/

static void
qemu_epc_clear_bar(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                   FAR struct pci_epf_bar_s *bar)
{
  FAR struct qemu_epc_s *qep = epc->priv;
  uint32_t base;
  uint8_t mask;

  base = pci_qep_func_base(qep, funcno);
  mask = qemu_epc_bar_cfg_read8(qep, base + QEMU_EPC_BAR_CFG_OFF_MASK)
                                & ~BIT(bar->barno);
  qemu_epc_bar_cfg_write8(qep, base + QEMU_EPC_BAR_CFG_OFF_MASK, mask);
}

/****************************************************************************
 * Name: qemu_epc_map_addr
 *
 * Description:
 *   This function is used to create a outbound mapping.
 *
 * Input Parameters:
 *   epc      - Device name of the endpoint controller
 *   funcno   - The epc's function number
 *   addr     - The outbound phy addr
 *   pci_addr - The pci addr
 *   size     - The addr's length
 *
 * Returned Value:
 *   0 if success, negative if failed
 ****************************************************************************/

static int qemu_epc_map_addr(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                             uintptr_t addr, uint64_t pci_addr, size_t size)
{
  FAR struct qemu_epc_s *qep = epc->priv;
  uint32_t idx = 0;
  uint32_t base;
  uint32_t mask;

  base = pci_qep_func_base(qep, funcno);
  mask = qemu_epc_ctl_read32(qep, base + QEMU_EPC_CTRL_OFF_OB_MAP_MASK);
  while (1)
    {
      if (!(mask & (1 << idx)))
        {
          break;
        }

      if (++idx >= 32)
        {
          return -ENOMEM;
        }
    }

  qemu_epc_ctl_write32(qep, base + QEMU_EPC_CTRL_OFF_OB_IDX, idx);
  qemu_epc_ctl_write64(qep, base + QEMU_EPC_CTRL_OFF_OB_MAP_PHYS, addr);
  qemu_epc_ctl_write64(qep, base + QEMU_EPC_CTRL_OFF_OB_MAP_PCI, pci_addr);
  qemu_epc_ctl_write64(qep, base + QEMU_EPC_CTRL_OFF_OB_MAP_SIZE, size);
  qemu_epc_ctl_write32(qep, base + QEMU_EPC_CTRL_OFF_OB_MAP_MASK,
                       mask | BIT(idx));
  qep->ob_phys[idx] = addr;

  return 0;
}

/****************************************************************************
 * Name: qemu_epc_unmap_addr
 *
 * Description:
 *   This function is used to umap a outbound mapping.
 *
 * Input Parameters:
 *   epc    - Device name of the endpoint controller
 *   funcno - The epc's function number
 *   addr   - The outbound phy addr
 ****************************************************************************/

static void
qemu_epc_unmap_addr(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                    uintptr_t addr)
{
  FAR struct qemu_epc_s *qep = epc->priv;
  uint32_t base;
  uint32_t mask;
  int i;

  base = pci_qep_func_base(qep, funcno);
  mask = qemu_epc_ctl_read32(qep, base + QEMU_EPC_CTRL_OFF_OB_MAP_MASK);
  for (i = 0; i < 32; i++)
    {
      if (qep->ob_phys[i] == addr)
        {
          mask &= ~BIT(i);
          qemu_epc_ctl_write32(qep, base + QEMU_EPC_CTRL_OFF_OB_MAP_MASK,
                               mask);
          break;
        }
    }
}

FAR void *
qemu_epc_get_bar_addr_from_funcno(FAR struct pci_epc_ctrl_s *epc,
                                  uint8_t funcno, uint8_t bar)
{
  FAR struct pci_epf_device_s *epf;

  list_for_every_entry(&epc->epf, epf, struct pci_epf_device_s, epc_node)
    {
      if (epf->funcno == funcno)
        {
          return epf->bar[bar].addr;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: qemu_epc_raise_irq
 *
 * Description:
 *   This function is used to raise a irq.
 *
 * Input Parameters:
 *   epc           - Device name of the endpoint controller
 *   funcno        - The epc's function number
 *   type          - The type of irq
 *   interrupt_num - The number of irq
 *
 * Returned Value:
 *   Return 0 if success, negative number if failed
 ****************************************************************************/

static int
qemu_epc_raise_irq(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                   enum pci_epc_irq_type_e type, uint16_t interrupt_num)
{
  FAR struct qemu_epc_s *qep = epc->priv;
  uint64_t pci_addr;
  uint32_t offset;
  uint32_t data;
  uint32_t base;

  base = pci_qep_func_base(qep, funcno);
  switch (type)
    {
      case PCI_EPC_IRQ_LEGACY:
        qemu_epc_ctl_write32(qep, base + QEMU_EPC_CTRL_OFF_IRQ_TYPE, type);
        qemu_epc_ctl_write32(qep, base + QEMU_EPC_CTRL_OFF_IRQ_NUM,
                             interrupt_num);
        return 0;
      case PCI_EPC_IRQ_MSI:
        if (qep->msi_vaddr == NULL)
          {
            uint16_t flags =
              qemu_epc_cfg_read16(qep, base + QEMU_EPC_BAR_CFG_MSI +
                                  PCI_MSI_FLAGS);
            if (flags & PCI_MSI_FLAGS_64BIT)
              {
                pci_addr =
                  qemu_epc_cfg_read64(qep,
                                      base + QEMU_EPC_BAR_CFG_MSI +
                                      PCI_MSI_ADDRESS_LO);
                qep->msi_data =
                  qemu_epc_cfg_read32(qep,
                                      base + QEMU_EPC_BAR_CFG_MSI +
                                      PCI_MSI_DATA_64);
              }
            else
              {
                pci_addr =
                  qemu_epc_cfg_read32(qep,
                                      base + QEMU_EPC_BAR_CFG_MSI +
                                      PCI_MSI_ADDRESS_LO);
                qep->msi_data =
                  qemu_epc_cfg_read32(qep,
                                      base + QEMU_EPC_BAR_CFG_MSI +
                                      PCI_MSI_DATA_32);
              }

            if (pci_addr == 0 || qep->msi_data == 0)
              {
                return -EINVAL;
              }

            qep->msi_vaddr = pci_epc_mem_alloc_addr(epc,
                                                    &qep->msi_paddr, 0x1000);
            qemu_epc_map_addr(epc, funcno,
                              qep->msi_paddr, pci_addr, 0x1000);
          }

        pci_write_mmio_qword(qep->pdev, qep->msi_vaddr, qep->msi_data);
        return 0;
      case PCI_EPC_IRQ_MSIX:
        if (--interrupt_num > qemu_epc_get_msix(epc, funcno))
          {
            return -EINVAL;
          }

        data = qemu_epc_cfg_read32(qep,
                                   base + QEMU_EPC_BAR_CFG_MSIX +
                                   PCI_MSIX_TABLE);
        offset = data & PCI_MSIX_TABLE_OFFSET;
        if (qep->msix_vaddr == NULL)
          {
            uint8_t bar = data & PCI_MSIX_TABLE_BIR;
            qep->msix_vaddr =
              qemu_epc_get_bar_addr_from_funcno(epc, funcno, bar);
            qep->msix_vob =
              pci_epc_mem_alloc_addr(epc, &qep->msix_pob, 0x1000);
            pci_read_mmio_qword(qep->pdev, qep->msix_vaddr + offset,
                                &pci_addr);
            qemu_epc_map_addr(epc, funcno, qep->msix_pob,
                              pci_addr, 0x1000);
          }

        pci_read_mmio_dword(qep->pdev,
                            qep->msix_vaddr + offset +
                            interrupt_num * PCI_MSIX_ENTRY_SIZE +
                            PCI_MSIX_ENTRY_DATA, &data);
        pci_write_mmio_qword(qep->pdev, qep->msix_vob, data);
        return 0;
      default:
        pcierr("Failed to raise IRQ, unknown type\n");
        return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: qemu_epc_start
 *
 * Description:
 *   This function is used to start the epc.
 *
 * Input Parameters:
 *   epc - Device name of the endpoint controller
 *
 * Returned Value:
 *   Return 0
 ****************************************************************************/

static int qemu_epc_start(FAR struct pci_epc_ctrl_s *epc)
{
  FAR struct qemu_epc_s *qep = epc->priv;

  qemu_epc_ctl_write8(qep, QEMU_EPC_CTRL_OFF_START, 1);
  return 0;
}

/****************************************************************************
 * Name: qemu_epc_get_features
 *
 * Description:
 *   This function is used to get the Epc supported features.
 *
 *   Invoke to get struct pci_epc_features_s* corresponding to the epc
 *
 * Input Parameters:
 *   epc    - Device name of the endpoint controller
 *   funcno - The epc's function number
 *
 * Returned Value:
 *   Return epc's features
 ****************************************************************************/

static const FAR struct pci_epc_features_s *
qemu_epc_get_features(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno)
{
  return &g_qemu_epc_features;
}

/****************************************************************************
 * Name: qemu_epc_get_msi
 *
 * Description:
 *   This function is used to get number of the supported msi.
 *
 * Input Parameters:
 *   epc    - Device name of the endpoint controller
 *   funcno - The epc's function number
 *
 * Returned Value:
 *   Return the number of interrupts
 ****************************************************************************/

static int qemu_epc_get_msi(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno)
{
  FAR struct qemu_epc_s *qep = epc->priv;
  uint16_t flags;
  uint32_t base;

  base = pci_qep_func_base(qep, funcno);
  flags = qemu_epc_cfg_read16(qep, base + QEMU_EPC_BAR_CFG_MSI +
                              PCI_MSI_FLAGS);
  if (!(flags & PCI_MSIX_FLAGS_ENABLE))
    {
      pcierr("msi is not enabled\n");
      return -EINVAL;
    }

  return (flags & PCI_MSI_FLAGS_QSIZE) >> PCI_MSI_FLAGS_QSIZE_SHIFT;
}

/****************************************************************************
 * Name: qemu_epc_set_msi
 *
 * Description:
 *   This function is used to set Epc msi interrupt number.
 *
 * Input Parameters:
 *   epc    - Device name of the endpoint controller
 *   funcno - The epc's function number
 *   interrupts - The interrupts number
 *
 * Returned Value:
 *   Return 0
 ****************************************************************************/

static int qemu_epc_set_msi(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                            uint8_t interrupts)
{
  FAR struct qemu_epc_s *qep = epc->priv;
  uint16_t flags;
  uint32_t base;

  base = pci_qep_func_base(qep, funcno);
  flags = qemu_epc_cfg_read16(qep, base + QEMU_EPC_BAR_CFG_MSI +
                              PCI_MSI_FLAGS);
  flags |= (interrupts << PCI_MSI_FLAGS_QMASK_SHIFT) & PCI_MSI_FLAGS_QMASK;
  qemu_epc_cfg_write16(qep, base + QEMU_EPC_BAR_CFG_MSI + PCI_MSI_FLAGS,
                       flags);
  return 0;
}

/****************************************************************************
 * Name: qemu_epc_get_msix
 *
 * Description:
 *   This function is used to get number of the supported msix.
 *
 * Input Parameters:
 *   epc    - Device name of the endpoint controller
 *   funcno - The epc's function number
 *
 * Returned Value:
 *   Return the number of interrupts
 ****************************************************************************/

static int qemu_epc_get_msix(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno)
{
  FAR struct qemu_epc_s *qep = epc->priv;
  uint16_t flags;
  uint32_t base;

  base = pci_qep_func_base(qep, funcno);
  flags = qemu_epc_cfg_read16(qep, base + QEMU_EPC_BAR_CFG_MSIX +
                              PCI_MSIX_FLAGS);
  if ((flags & PCI_MSIX_FLAGS_ENABLE) == 0)
    {
      return -EINVAL;
    }

  return flags & PCI_MSIX_FLAGS_QSIZE;
}

/****************************************************************************
 * Name: qemu_epc_set_msix
 *
 * Description:
 *   This function is used to set Epc msix interrupt number.
 *
 * Input Parameters:
 *   epc        - Device name of the endpoint controller
 *   funcno     - The epc's function number
 *   interrupts - The interrupts number
 *   barno      - BAR where the MSI-X table resides
 *   offset     - Offset pointing to the start of MSI-X table
 *
 * Returned Value:
 *   Return 0
 ****************************************************************************/

static int qemu_epc_set_msix(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                             uint16_t interrupts, int barno, uint32_t offset)
{
  FAR struct qemu_epc_s *qep = epc->priv;
  uint32_t data;
  uint16_t flags;
  uint32_t base;

  base = pci_qep_func_base(qep, funcno);
  flags = qemu_epc_cfg_read16(qep, base + QEMU_EPC_BAR_CFG_MSIX +
                              PCI_MSIX_FLAGS);
  flags &= ~PCI_MSIX_FLAGS_QSIZE;
  flags |= interrupts;
  qemu_epc_cfg_write16(qep, base + QEMU_EPC_BAR_CFG_MSIX + PCI_MSIX_FLAGS,
                       flags);
  data = offset | barno;
  qemu_epc_cfg_write32(qep, base + QEMU_EPC_BAR_CFG_MSIX + PCI_MSIX_TABLE,
                       data);
  data = (offset + (interrupts * PCI_MSIX_ENTRY_SIZE)) | barno;
  qemu_epc_cfg_write32(qep, base + QEMU_EPC_BAR_CFG_MSIX + PCI_MSIX_PBA,
                       data);

  return 0;
}

static int qemu_epc_probe(FAR struct pci_device_s *dev)
{
  FAR struct pci_epc_ctrl_s *epc;
  FAR struct qemu_epc_s *qep;
  FAR void *virt;
  uintptr_t phys;
  size_t size;
  int ret;

  qep = kmm_zalloc(sizeof(*qep));
  if (qep == NULL)
    {
      pcierr("Failed to allocate memory\n");
      return -ENOMEM;
    }

  qep->pdev = dev;
  epc = pci_epc_create("qemu_epc", qep, &g_qemu_epc_ops);
  if (epc == NULL)
    {
      pcierr("Failed to create epc device\n");
      ret = -ENOMEM;
      goto err_free_epc;
    }

  epc->max_functions = QEMU_EPC_MAX_FUNCTIONS;
  ret = pci_enable_device(dev);
  if (ret < 0)
    {
      pcierr("Cannot enable PCI device\n");
      goto err_free_epc;
    }

  qep->cfg_base = pci_map_bar(dev, QEMU_EPC_BAR_PCI_CFG);
  if (qep->cfg_base == NULL)
    {
      pcierr("Cannot map device registers\n");
      ret = -ENOMEM;
      goto err_disable_pdev;
    }

  qep->bar_base = pci_map_bar(dev, QEMU_EPC_BAR_BAR_CFG);
  if (qep->bar_base == NULL)
    {
      pcierr("Cannot map device register for bar\n");
      ret = -ENOMEM;
      goto err_disable_pdev;
    }

  qep->ctl_base = pci_map_bar(dev, QEMU_EPC_BAR_CTRL);
  if (qep->ctl_base == NULL)
    {
      pcierr("Cannot map ctrl register\n");
      ret = -ENOMEM;
      goto err_disable_pdev;
    }

  phys = qemu_epc_ctl_read64(qep, QEMU_EPC_CTRL_OFF_WIN_START);
  size = qemu_epc_ctl_read64(qep, QEMU_EPC_CTRL_OFF_WIN_SIZE);
  virt = pci_map_region(dev, phys, size);
  ret = pci_epc_mem_init(epc, virt, phys, size, QEMU_EPC_ALIGN_PAGE_SIZE);
  if (ret < 0)
    {
      pcierr("epc mem init error\n");
      goto err_disable_pdev;
    }

  pci_set_master(dev);
  return 0;

err_disable_pdev:
  pci_disable_device(dev);
err_free_epc:
  kmm_free(qep);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_register_qemu_epc_driver
 *
 * Description:
 *   Register a pci epc driver
 *
 ****************************************************************************/

int pci_register_qemu_epc_driver(void)
{
  return pci_register_driver(&g_qemu_epc_drv);
}
