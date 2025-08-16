/****************************************************************************
 * drivers/pci/pcie_dw_ep.c
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

#include <debug.h>

#include <nuttx/bits.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pci/pci.h>
#include <nuttx/pci/pci_regs.h>
#include <nuttx/pci/pci_epc.h>
#include <nuttx/pci/pci_epf.h>

#include "pcie_dw.h"
#include "pci_drivers.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void
__dw_pcie_ep_reset_bar(FAR struct dw_pcie_s *pci, uint8_t funcno,
                       int bar, int flags);
static void
__dw_pcie_ep_reset_bar(FAR struct dw_pcie_s *pci, uint8_t funcno,
                       int bar, int flags);
static uint8_t
__dw_pcie_ep_find_next_cap(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                           uint8_t cap_ptr, uint8_t cap);
static uint8_t
dw_pcie_ep_find_capability(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                           uint8_t cap);
static int
dw_pcie_ep_write_header(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                        FAR struct pci_epf_header_s *hdr);
static int
dw_pcie_ep_inbound_atu(FAR struct dw_pcie_ep_s *ep, uint8_t funcno, int type,
                       uintptr_t cpu_addr, uint8_t bar);
static int
dw_pcie_ep_outbound_atu(FAR struct dw_pcie_ep_s *ep,
                        struct dw_pcie_ob_atu_cfg_s *atu);
static void
dw_pcie_ep_clear_bar(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                     FAR struct pci_epf_bar_s *epf_bar);
static int
dw_pcie_ep_set_bar(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                   FAR struct pci_epf_bar_s *epf_bar);
static int
dw_pcie_find_index(FAR struct dw_pcie_ep_s *ep, uintptr_t addr,
                   FAR uint32_t *atu_index);
static void
dw_pcie_ep_unmap_addr(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                      uintptr_t addr);
static int
dw_pcie_ep_map_addr(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                    uintptr_t addr, uint64_t pci_addr, size_t size);
static int
dw_pcie_ep_get_msi(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno);
static int
dw_pcie_ep_set_msi(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                   uint8_t interrupts);
static int
dw_pcie_ep_get_msix(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno);
static int
dw_pcie_ep_set_msix(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                    uint16_t interrupts, int bir, uint32_t offset);
static int
dw_pcie_ep_raise_irq(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                     unsigned int type, uint16_t interrupt_num);
static void dw_pcie_ep_stop(FAR struct pci_epc_ctrl_s *epc);
static int dw_pcie_ep_start(FAR struct pci_epc_ctrl_s *epc);
static FAR const struct pci_epc_features_s *
dw_pcie_ep_get_features(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno);
static unsigned int
dw_pcie_ep_find_ext_capability(FAR struct dw_pcie_s *pci, int cap);
static void dw_pcie_ep_init_non_sticky_registers(FAR struct dw_pcie_s *pci);
static int dw_pcie_ep_init_registers(FAR struct dw_pcie_ep_s *ep);
FAR struct dw_pcie_ep_func_s *dw_pcie_ep_get_func_from_ep(
  FAR struct dw_pcie_ep_s *ep, uint8_t funcno);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pci_epc_ops_s g_epc_ops =
{
  .write_header   = dw_pcie_ep_write_header,
  .set_bar        = dw_pcie_ep_set_bar,
  .clear_bar      = dw_pcie_ep_clear_bar,
  .map_addr       = dw_pcie_ep_map_addr,
  .unmap_addr     = dw_pcie_ep_unmap_addr,
  .set_msi        = dw_pcie_ep_set_msi,
  .get_msi        = dw_pcie_ep_get_msi,
  .set_msix       = dw_pcie_ep_set_msix,
  .get_msix       = dw_pcie_ep_get_msix,
  .raise_irq      = dw_pcie_ep_raise_irq,
  .start          = dw_pcie_ep_start,
  .stop           = dw_pcie_ep_stop,
  .get_features   = dw_pcie_ep_get_features,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void __dw_pcie_ep_reset_bar(FAR struct dw_pcie_s *pci, uint8_t funcno,
           int bar, int flags)
{
  FAR struct dw_pcie_ep_s *ep = &pci->ep;
  uint32_t reg;

  reg = PCI_BASE_ADDRESS_0 + (4 * bar);
  dw_pcie_dbi_ro_wr_en(pci);
  dw_pcie_ep_writel_dbi2(ep, funcno, reg, 0x0);
  dw_pcie_ep_writel_dbi(ep, funcno, reg, 0x0);
  if (flags & PCI_BASE_ADDRESS_MEM_TYPE_64)
    {
      dw_pcie_ep_writel_dbi2(ep, funcno, reg + 4, 0x0);
      dw_pcie_ep_writel_dbi(ep, funcno, reg + 4, 0x0);
    }

  dw_pcie_dbi_ro_wr_dis(pci);
}

static uint8_t
__dw_pcie_ep_find_next_cap(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                           uint8_t cap_ptr, uint8_t cap)
{
  uint8_t next_cap_ptr;
  uint8_t cap_id;
  uint16_t reg;

  if (!cap_ptr)
    {
      return 0;
    }

  reg = dw_pcie_ep_readw_dbi(ep, funcno, cap_ptr);
  cap_id = (reg & 0x00ff);

  if (cap_id > PCI_CAP_ID_MAX)
    {
      return 0;
    }

  if (cap_id == cap)
    {
      return cap_ptr;
    }

  next_cap_ptr = (reg & 0xff00) >> 8;
  return __dw_pcie_ep_find_next_cap(ep, funcno, next_cap_ptr, cap);
}

static uint8_t
dw_pcie_ep_find_capability(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                           uint8_t cap)
{
  uint8_t next_cap_ptr;
  uint16_t reg;

  reg = dw_pcie_ep_readw_dbi(ep, funcno, PCI_CAPABILITY_LIST);
  next_cap_ptr = (reg & 0x00ff);

  return __dw_pcie_ep_find_next_cap(ep, funcno, next_cap_ptr, cap);
}

static int
dw_pcie_ep_write_header(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                        FAR struct pci_epf_header_s *hdr)
{
  FAR struct dw_pcie_ep_s *ep = epc->priv;
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);

  dw_pcie_dbi_ro_wr_en(pci);
  dw_pcie_ep_writew_dbi(ep, funcno, PCI_VENDOR_ID, hdr->vendorid);
  dw_pcie_ep_writew_dbi(ep, funcno, PCI_DEVICE_ID, hdr->deviceid);
  dw_pcie_ep_writeb_dbi(ep, funcno, PCI_REVISION_ID, hdr->revid);
  dw_pcie_ep_writeb_dbi(ep, funcno, PCI_CLASS_PROG, hdr->progif_code);
  dw_pcie_ep_writew_dbi(ep, funcno, PCI_CLASS_DEVICE,
            hdr->subclass_code | hdr->baseclass_code << 8);
  dw_pcie_ep_writeb_dbi(ep, funcno, PCI_CACHE_LINE_SIZE,
            hdr->cache_line_size);
  dw_pcie_ep_writew_dbi(ep, funcno, PCI_SUBSYSTEM_VENDOR_ID,
            hdr->subsys_vendor_id);
  dw_pcie_ep_writew_dbi(ep, funcno, PCI_SUBSYSTEM_ID, hdr->subsys_id);
  dw_pcie_ep_writeb_dbi(ep, funcno, PCI_INTERRUPT_PIN,
            hdr->interrupt_pin);
  dw_pcie_dbi_ro_wr_dis(pci);

  return 0;
}

static int
dw_pcie_ep_inbound_atu(FAR struct dw_pcie_ep_s *ep, uint8_t funcno, int type,
                       uintptr_t cpu_addr, uint8_t bar)
{
  int ret;
  uint32_t free_win;
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);

  if (!ep->bar_to_atu[bar])
    {
      free_win = find_first_zero_bit(ep->ib_window_map, pci->num_ib_windows);
    }
  else
    {
      free_win = ep->bar_to_atu[bar] - 1;
    }

  if (free_win >= pci->num_ib_windows)
    {
      pcierr("No free inbound window\n");
      return -EINVAL;
    }

  ret = dw_pcie_prog_ep_inbound_atu(pci, funcno, free_win, type,
            cpu_addr, bar);
  if (ret < 0)
    {
      pcierr("Failed to program IB window ret %d bar %d cpuaddr %lx\n",
             ret, bar, cpu_addr);
      return ret;
    }

  /* Always increment free_win before assignment, since value 0
   * is used to identify unallocated mapping.
   */

  ep->bar_to_atu[bar] = free_win + 1;
  set_bit(free_win, ep->ib_window_map);

  return 0;
}

static int
dw_pcie_ep_outbound_atu(FAR struct dw_pcie_ep_s *ep,
                        FAR struct dw_pcie_ob_atu_cfg_s *atu)
{
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);
  uint32_t free_win;
  int ret;

  free_win = find_first_zero_bit(ep->ob_window_map, pci->num_ob_windows);
  if (free_win >= pci->num_ob_windows)
    {
      pcierr("No free outbound window\n");
      return -EINVAL;
    }

  atu->index = free_win;
  ret = dw_pcie_prog_outbound_atu(pci, atu);
  if (ret)
    {
      return ret;
    }

  set_bit(free_win, ep->ob_window_map);
  ep->outbound_addr[free_win] = atu->cpu_addr;

  return 0;
}

static void
dw_pcie_ep_clear_bar(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                     FAR struct pci_epf_bar_s *epf_bar)
{
  FAR struct dw_pcie_ep_s *ep = epc->priv;
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);
  uint8_t bar = epf_bar->barno;
  uint32_t atu_index = ep->bar_to_atu[bar] - 1;

  if (!ep->bar_to_atu[bar])
    {
      return;
    }

  __dw_pcie_ep_reset_bar(pci, funcno, bar, epf_bar->flags);

  dw_pcie_disable_atu(pci, DW_PCIE_ATU_REGION_DIR_IB, atu_index);
  clear_bit(atu_index, ep->ib_window_map);
  ep->epf_bar[bar] = NULL;
  ep->bar_to_atu[bar] = 0;
}

static int
dw_pcie_ep_set_bar(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                   FAR struct pci_epf_bar_s *epf_bar)
{
  FAR struct dw_pcie_ep_s *ep = epc->priv;
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);
  uint8_t bar = epf_bar->barno;
  size_t size = epf_bar->size;
  int flags = epf_bar->flags;
  uint32_t reg;
  int ret;
  int type;

  /* DWC does not allow BAR pairs to overlap, e.g. you cannot combine BARs
   * 1 and 2 to form a 64-bit BAR.
   */

  if ((flags & PCI_BASE_ADDRESS_MEM_TYPE_64) && (bar & 1))
    {
      return -EINVAL;
    }

  reg = PCI_BASE_ADDRESS_0 + (4 * bar);

  if (!(flags & PCI_BASE_ADDRESS_SPACE))
    {
      type = DW_PCIE_ATU_TYPE_MEM;
    }
  else
    {
      type = DW_PCIE_ATU_TYPE_IO;
    }

  ret = dw_pcie_ep_inbound_atu(ep, funcno, type, epf_bar->phys_addr, bar);
  if (ret)
    {
      return ret;
    }

  if (ep->epf_bar[bar])
    {
      return 0;
    }

  dw_pcie_dbi_ro_wr_en(pci);

  dw_pcie_ep_writel_dbi2(ep, funcno, reg, lower_32_bits(size - 1));
  dw_pcie_ep_writel_dbi(ep, funcno, reg, flags);

  if (flags & PCI_BASE_ADDRESS_MEM_TYPE_64)
    {
      dw_pcie_ep_writel_dbi2(ep, funcno, reg + 4, upper_32_bits(size - 1));
      dw_pcie_ep_writel_dbi(ep, funcno, reg + 4, 0);
    }

  ep->epf_bar[bar] = epf_bar;
  dw_pcie_dbi_ro_wr_dis(pci);

  return 0;
}

static int
dw_pcie_find_index(FAR struct dw_pcie_ep_s *ep, uintptr_t addr,
                   FAR uint32_t *atu_index)
{
  uint32_t index;
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);

  for (index = 0; index < pci->num_ob_windows; index++)
    {
      if (ep->outbound_addr[index] != addr)
        {
          continue;
        }

      *atu_index = index;
      return 0;
    }

  return -EINVAL;
}

static void
dw_pcie_ep_unmap_addr(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                      uintptr_t addr)
{
  int ret;
  uint32_t atu_index;
  FAR struct dw_pcie_ep_s *ep = epc->priv;
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);

  ret = dw_pcie_find_index(ep, addr, &atu_index);
  if (ret < 0)
    {
      return;
    }

  dw_pcie_disable_atu(pci, DW_PCIE_ATU_REGION_DIR_OB, atu_index);
  clear_bit(atu_index, ep->ob_window_map);
}

static int
dw_pcie_ep_map_addr(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                    FAR uintptr_t addr, uint64_t pci_addr, size_t size)
{
  FAR struct dw_pcie_ep_s *ep = epc->priv;
  FAR struct dw_pcie_ob_atu_cfg_s atu =
    {
      0
    };

  int ret;

  atu.funcno = funcno;
  atu.type = DW_PCIE_ATU_TYPE_MEM;
  atu.cpu_addr = addr;
  atu.pci_addr = pci_addr;
  atu.size = size;
  ret = dw_pcie_ep_outbound_atu(ep, &atu);
  if (ret)
    {
      pcierr("Failed to enable address\n");
      return ret;
    }

  return 0;
}

static int
dw_pcie_ep_get_msi(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno)
{
  FAR struct dw_pcie_ep_s *ep = epc->priv;
  FAR struct dw_pcie_ep_func_s *ep_func;
  uint32_t val;
  uint32_t reg;

  ep_func = dw_pcie_ep_get_func_from_ep(ep, funcno);
  if (!ep_func || !ep_func->msi_cap)
    {
      return -EINVAL;
    }

  reg = ep_func->msi_cap + PCI_MSI_FLAGS;
  val = dw_pcie_ep_readw_dbi(ep, funcno, reg);
  if (!(val & PCI_MSI_FLAGS_ENABLE))
    {
      return -EINVAL;
    }

  val = FIELD_GET(PCI_MSI_FLAGS_QSIZE, val);

  return val;
}

static int
dw_pcie_ep_set_msi(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                   uint8_t interrupts)
{
  FAR struct dw_pcie_ep_s *ep = epc->priv;
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);
  FAR struct dw_pcie_ep_func_s *ep_func;
  uint32_t val;
  uint32_t reg;

  ep_func = dw_pcie_ep_get_func_from_ep(ep, funcno);
  if (!ep_func || !ep_func->msi_cap)
    {
      return -EINVAL;
    }

  reg = ep_func->msi_cap + PCI_MSI_FLAGS;
  val = dw_pcie_ep_readw_dbi(ep, funcno, reg);
  val &= ~PCI_MSI_FLAGS_QMASK;
  val |= FIELD_PREP(PCI_MSI_FLAGS_QMASK, interrupts);
  dw_pcie_dbi_ro_wr_en(pci);
  dw_pcie_ep_writew_dbi(ep, funcno, reg, val);
  dw_pcie_dbi_ro_wr_dis(pci);

  return 0;
}

static int
dw_pcie_ep_get_msix(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno)
{
  struct dw_pcie_ep_s *ep = epc->priv;
  struct dw_pcie_ep_func_s *ep_func;
  uint32_t val;
  uint32_t reg;

  ep_func = dw_pcie_ep_get_func_from_ep(ep, funcno);
  if (!ep_func || !ep_func->msix_cap)
    {
      return -EINVAL;
    }

  reg = ep_func->msix_cap + PCI_MSIX_FLAGS;
  val = dw_pcie_ep_readw_dbi(ep, funcno, reg);
  if (!(val & PCI_MSIX_FLAGS_ENABLE))
    {
      return -EINVAL;
    }

  val &= PCI_MSIX_FLAGS_QSIZE;

  return val;
}

static int
dw_pcie_ep_set_msix(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                    uint16_t interrupts, int bir, uint32_t offset)
{
  FAR struct dw_pcie_ep_s *ep = epc->priv;
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);
  FAR struct dw_pcie_ep_func_s *ep_func;
  uint32_t val;
  uint32_t reg;

  ep_func = dw_pcie_ep_get_func_from_ep(ep, funcno);
  if (!ep_func || !ep_func->msix_cap)
    {
      return -EINVAL;
    }

  dw_pcie_dbi_ro_wr_en(pci);

  reg = ep_func->msix_cap + PCI_MSIX_FLAGS;
  val = dw_pcie_ep_readw_dbi(ep, funcno, reg);
  val &= ~PCI_MSIX_FLAGS_QSIZE;
  val |= interrupts;
  dw_pcie_writew_dbi(pci, reg, val);

  reg = ep_func->msix_cap + PCI_MSIX_TABLE;
  val = offset | bir;
  dw_pcie_ep_writel_dbi(ep, funcno, reg, val);

  reg = ep_func->msix_cap + PCI_MSIX_PBA;
  val = (offset + (interrupts * PCI_MSIX_ENTRY_SIZE)) | bir;
  dw_pcie_ep_writel_dbi(ep, funcno, reg, val);

  dw_pcie_dbi_ro_wr_dis(pci);

  return 0;
}

static int
dw_pcie_ep_raise_irq(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno,
                     unsigned int type, uint16_t interrupt_num)
{
  FAR struct dw_pcie_ep_s *ep = epc->priv;

  if (!ep->ops->raise_irq)
    {
      return -EINVAL;
    }

  return ep->ops->raise_irq(ep, funcno, type, interrupt_num);
}

static void dw_pcie_ep_stop(FAR struct pci_epc_ctrl_s *epc)
{
  struct dw_pcie_ep_s *ep = epc->priv;
  struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);

  dw_pcie_stop_link(pci);
}

static int dw_pcie_ep_start(FAR struct pci_epc_ctrl_s *epc)
{
  FAR struct dw_pcie_ep_s *ep = epc->priv;
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);

  return dw_pcie_start_link(pci);
}

static FAR const struct pci_epc_features_s *
dw_pcie_ep_get_features(FAR struct pci_epc_ctrl_s *epc, uint8_t funcno)
{
  FAR struct dw_pcie_ep_s *ep = epc->priv;

  if (!ep->ops->get_features)
    {
      return NULL;
    }

  return ep->ops->get_features(ep);
}

static unsigned int
dw_pcie_ep_find_ext_capability(FAR struct dw_pcie_s *pci, int cap)
{
  uint32_t header;
  int pos = PCI_CFG_SPACE_SIZE;

  while (pos)
    {
      header = dw_pcie_readl_dbi(pci, pos);
      if (PCI_EXT_CAP_ID(header) == cap)
        {
          return pos;
        }

      pos = PCI_EXT_CAP_NEXT(header);
      if (!pos)
        {
          break;
        }
    }

  return 0;
}

static void dw_pcie_ep_init_non_sticky_registers(FAR struct dw_pcie_s *pci)
{
  unsigned int offset;
  unsigned int nbars;
  uint32_t reg;
  uint32_t i;

  offset = dw_pcie_ep_find_ext_capability(pci, PCI_EXT_CAP_ID_REBAR);

  dw_pcie_dbi_ro_wr_en(pci);

  if (offset)
    {
      reg = dw_pcie_readl_dbi(pci, offset + PCI_REBAR_CTRL);
      nbars = (reg & PCI_REBAR_CTRL_NBAR_MASK) >>
               PCI_REBAR_CTRL_NBAR_SHIFT;

      /* PCIe r6.0, sec 7.8.6.2 require us to support at least one
       * size in the range from 1 MB to 512 GB. Advertise support
       * for 1 MB BAR size only.
       */

      for (i = 0; i < nbars; i++, offset += PCI_REBAR_CTRL)
        {
          dw_pcie_writel_dbi(pci, offset + PCI_REBAR_CAP, 0x0);
        }
    }

  dw_pcie_setup(pci);
  dw_pcie_dbi_ro_wr_dis(pci);
}

static int dw_pcie_ep_init_registers(FAR struct dw_pcie_ep_s *ep)
{
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);
  FAR struct dw_pcie_ep_func_s *ep_func;
  FAR struct pci_epc_ctrl_s *epc = ep->epc;
  uint32_t ptm_cap_base;
  size_t bitmap_size;
  uint8_t hdr_type;
  uint8_t funcno;
  FAR void *addr;
  uint32_t reg;

  hdr_type = dw_pcie_readb_dbi(pci, PCI_HEADER_TYPE) &
                               PCI_HEADER_TYPE_MASK;
  pciinfo("hdr_type %x \n", hdr_type);

  if (hdr_type != PCI_HEADER_TYPE_NORMAL)
    {
      pcierr("PCIe controller is not set to EP mode (hdr_type:0x%x)!\n",
             hdr_type);
      return -EIO;
    }

  hdr_type = dw_pcie_readl_dbi(pci, 8);

  dw_pcie_version_detect(pci);

  dw_pcie_iatu_detect(pci);

  if (!ep->ib_window_map)
    {
      bitmap_size = BITS_TO_LONGS(pci->num_ib_windows) * sizeof(long);
      ep->ib_window_map = kmm_zalloc(bitmap_size);
      if (ep->ib_window_map == NULL)
        {
          return -ENOMEM;
        }
    }

  if (!ep->ob_window_map)
    {
      bitmap_size = BITS_TO_LONGS(pci->num_ob_windows) * sizeof(long);
      ep->ob_window_map = kmm_zalloc(bitmap_size);
      if (!ep->ob_window_map)
        {
          goto err_free_ib_map;
        }
    }

  if (!ep->outbound_addr)
    {
      addr = kmm_zalloc(pci->num_ob_windows * sizeof(uintptr_t));
      if (!addr)
        {
          goto err_free_ob_map;
        }

      ep->outbound_addr = addr;
    }

  for (funcno = 0; funcno < epc->max_functions; funcno++)
    {
      ep_func = dw_pcie_ep_get_func_from_ep(ep, funcno);
      if (ep_func)
        {
          continue;
        }

      ep_func = kmm_zalloc(sizeof(*ep_func));
      if (!ep_func)
        {
          goto err_free_ob_addr;
        }

      ep_func->funcno = funcno;
      ep_func->msi_cap
        = dw_pcie_ep_find_capability(ep, funcno, PCI_CAP_ID_MSI);
      ep_func->msix_cap
        = dw_pcie_ep_find_capability(ep, funcno, PCI_CAP_ID_MSIX);

      list_add_tail(&ep->func_list, &ep_func->list);
    }

  if (ep->ops->init)
    {
      ep->ops->init(ep);
    }

  ptm_cap_base = dw_pcie_ep_find_ext_capability(pci, PCI_EXT_CAP_ID_PTM);

  /* PTM responder capability can be disabled only after disabling
   * PTM root capability.
   */

  if (ptm_cap_base)
    {
      dw_pcie_dbi_ro_wr_en(pci);
      reg = dw_pcie_readl_dbi(pci, ptm_cap_base + PCI_PTM_CAP);
      reg &= ~PCI_PTM_CAP_ROOT;
      dw_pcie_writel_dbi(pci, ptm_cap_base + PCI_PTM_CAP, reg);

      reg = dw_pcie_readl_dbi(pci, ptm_cap_base + PCI_PTM_CAP);
      reg &= ~(PCI_PTM_CAP_RES | PCI_PTM_GRANULARITY_MASK);
      dw_pcie_writel_dbi(pci, ptm_cap_base + PCI_PTM_CAP, reg);
      dw_pcie_dbi_ro_wr_dis(pci);
    }

  dw_pcie_ep_init_non_sticky_registers(pci);

  return 0;

err_free_ob_addr:
  kmm_free(ep->outbound_addr);
err_free_ob_map:
  kmm_free(ep->ob_window_map);
err_free_ib_map:
  kmm_free(ep->ib_window_map);
  return -EINVAL;
}

/****************************************************************************
 * Name: dw_pcie_ep_get_func_from_ep
 *
 * Description:
 *   This function is used to Get the struct dw_pcie_ep_func_s corresponding
 *   to the endpoint function
 *
 * Input Parameters:
 *   ep               - DWC EP of the endpoint controller
 *   funcno           - Function number of the endpoint device
 *
 * Returned Value:
 *   Return struct dw_pcie_ep_func_s if success, NULL otherwise.
 *
 ****************************************************************************/

FAR struct dw_pcie_ep_func_s *dw_pcie_ep_get_func_from_ep(
  FAR struct dw_pcie_ep_s *ep, uint8_t funcno)
{
  FAR struct dw_pcie_ep_func_s *ep_func;

  list_for_every_entry(&ep->func_list, ep_func, struct dw_pcie_ep_func_s,
                       list)
    {
      if (ep_func->funcno == funcno)
        {
          return ep_func;
        }
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dw_pcie_ep_reset_bar
 *
 * Description:
 *   This function is used to Reset endpoint BARMSI-X to the host
 *
 * Input Parameters:
 *   ep               - DWC EP of the endpoint controller
 *   bar              - BAR number of the endpoint
 *
 * Returned Value:
 *   Return 0, errno otherwise.
 *
 ****************************************************************************/

void dw_pcie_ep_reset_bar(FAR struct dw_pcie_s *pci, int bar)
{
  uint8_t funcno;
  uint8_t funcs;

  funcs = pci->ep.epc->max_functions;

  for (funcno = 0; funcno < funcs; funcno++)
    {
      __dw_pcie_ep_reset_bar(pci, funcno, bar, 0);
    }
}

/****************************************************************************
 * Name: dw_pcie_ep_raise_intx_irq
 *
 * Description:
 *   This function is used to Raise MSI-X to the host
 *
 * Input Parameters:
 *   ep               - DWC EP of the endpoint controller
 *   funcno           - The epc's function number
 *
 * Returned Value:
 *   Return 0, errno otherwise.
 *
 ****************************************************************************/

int dw_pcie_ep_raise_intx_irq(FAR struct dw_pcie_ep_s *ep, uint8_t funcno)
{
  pcierr("EP cannot raise INTX IRQs\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: dw_pcie_ep_raise_msi_irq
 *
 * Description:
 *   This function is used to Raise MSI-X to the host
 *
 * Input Parameters:
 *   ep               - DWC EP of the endpoint controller
 *   funcno           - The epc's function number
 *   interrupt_num    - The number of irq
 *
 * Returned Value:
 *   Return 0, errno otherwise.
 *
 ****************************************************************************/

int dw_pcie_ep_raise_msi_irq(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                             uint8_t interrupt_num)
{
  FAR struct dw_pcie_ep_func_s *ep_func;
  FAR struct pci_epc_ctrl_s *epc = ep->epc;
  uint32_t aligned_offset;
  uint32_t msg_addr_lower;
  uint32_t msg_addr_upper;
  uint64_t msg_addr;
  uint16_t msg_ctrl;
  uint16_t msg_data;
  bool has_upper;
  uint32_t reg;
  int ret;

  ep_func = dw_pcie_ep_get_func_from_ep(ep, funcno);
  if (!ep_func || !ep_func->msi_cap)
    {
      return -EINVAL;
    }

  /* Raise MSI per the PCI Local Bus Specification Revision 3.0, 6.8.1. */

  reg = ep_func->msi_cap + PCI_MSI_FLAGS;
  msg_ctrl = dw_pcie_ep_readw_dbi(ep, funcno, reg);
  has_upper = !!(msg_ctrl & PCI_MSI_FLAGS_64BIT);
  reg = ep_func->msi_cap + PCI_MSI_ADDRESS_LO;
  msg_addr_lower = dw_pcie_ep_readl_dbi(ep, funcno, reg);
  if (has_upper)
    {
      reg = ep_func->msi_cap + PCI_MSI_ADDRESS_HI;
      msg_addr_upper = dw_pcie_ep_readl_dbi(ep, funcno, reg);
      reg = ep_func->msi_cap + PCI_MSI_DATA_64;
      msg_data = dw_pcie_ep_readw_dbi(ep, funcno, reg);
    }
  else
    {
      msg_addr_upper = 0;
      reg = ep_func->msi_cap + PCI_MSI_DATA_32;
      msg_data = dw_pcie_ep_readw_dbi(ep, funcno, reg);
    }

  msg_addr = ((uint64_t)msg_addr_upper) << 32 | msg_addr_lower;

  aligned_offset = msg_addr & (epc->mem->page_size - 1);
  msg_addr = ALIGN_DOWN(msg_addr, epc->mem->page_size);
  ret = dw_pcie_ep_map_addr(epc, funcno, ep->msi_mem_phys, msg_addr,
                            epc->mem->page_size);
  if (ret)
    {
      return ret;
    }

  writel(msg_data | (interrupt_num - 1), ep->msi_mem + aligned_offset);

  dw_pcie_ep_unmap_addr(epc, funcno, ep->msi_mem_phys);

  return 0;
}

/****************************************************************************
 * Name: dw_pcie_ep_raise_msix_irq_doorbell
 *
 * Description:
 *   This function is used to Raise MSI-X to the host
 *
 * Input Parameters:
 *   ep               - DWC EP of the endpoint controller
 *   funcno           - The epc's function number
 *   interrupt_num    - The number of irq
 *
 * Returned Value:
 *   Return 0, errno otherwise.
 *
 ****************************************************************************/

int dw_pcie_ep_raise_msix_irq_doorbell(FAR struct dw_pcie_ep_s *ep,
                                       uint8_t funcno,
                                       uint16_t interrupt_num)
{
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);
  FAR struct dw_pcie_ep_func_s *ep_func;
  uint32_t msg_data;

  ep_func = dw_pcie_ep_get_func_from_ep(ep, funcno);
  if (!ep_func || !ep_func->msix_cap)
    {
      return -EINVAL;
    }

  msg_data = (funcno << DW_PCIE_MSIX_DOORBELL_PF_SHIFT) |
             (interrupt_num - 1);

  dw_pcie_writel_dbi(pci, DW_PCIE_MSIX_DOORBELL, msg_data);

  return 0;
}

/****************************************************************************
 * Name: dw_pcie_ep_raise_msix_irq
 *
 * Description:
 *   This function is used to Raise MSI-X to the host
 *
 * Input Parameters:
 *   ep               - DWC EP of the endpoint controller
 *   funcno           - The epc's function number
 *   interrupt_num    - The number of irq
 *
 * Returned Value:
 *   Return 0, errno otherwise.
 *
 ****************************************************************************/

int dw_pcie_ep_raise_msix_irq(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                              uint16_t interrupt_num)
{
  FAR struct pci_epf_msix_tbl_s *msix_tbl;
  FAR struct dw_pcie_ep_func_s *ep_func;
  FAR struct pci_epc_ctrl_s *epc = ep->epc;
  uint32_t aligned_offset;
  uint32_t tbl_offset;
  uint32_t msg_data;
  uint32_t vec_ctrl;
  uint64_t msg_addr;
  uint32_t reg;
  uint8_t bir;
  int ret;

  ep_func = dw_pcie_ep_get_func_from_ep(ep, funcno);
  if (!ep_func || !ep_func->msix_cap)
    {
      return -EINVAL;
    }

  reg = ep_func->msix_cap + PCI_MSIX_TABLE;
  tbl_offset = dw_pcie_ep_readl_dbi(ep, funcno, reg);
  bir = FIELD_GET(PCI_MSIX_TABLE_BIR, tbl_offset);
  tbl_offset &= PCI_MSIX_TABLE_OFFSET;

  msix_tbl = ep->epf_bar[bir]->addr + tbl_offset;
  msg_addr = msix_tbl[(interrupt_num - 1)].msg_addr;
  msg_data = msix_tbl[(interrupt_num - 1)].msg_data;
  vec_ctrl = msix_tbl[(interrupt_num - 1)].vector_ctrl;

  if (vec_ctrl & PCI_MSIX_ENTRY_CTRL_MASKBIT)
    {
      pcierr("MSI-X entry ctrl set\n");
      return -EPERM;
    }

  aligned_offset = msg_addr & (epc->mem->page_size - 1);
  msg_addr = ALIGN_DOWN(msg_addr, epc->mem->page_size);
  ret = dw_pcie_ep_map_addr(epc, funcno, ep->msi_mem_phys, msg_addr,
                            epc->mem->page_size);
  if (ret)
    {
      return ret;
    }

  writel(msg_data, ep->msi_mem + aligned_offset);

  dw_pcie_ep_unmap_addr(epc, funcno, ep->msi_mem_phys);

  return 0;
}

/****************************************************************************
 * Name: dw_pcie_ep_deinit
 *
 * Description:
 *   This function is used to DEInitialize DWC EP specific registers
 *
 * Input Parameters:
 *   ep               - DWC EP of the endpoint controller
 *
 * Returned Value:
 *   Return 0, errno otherwise.
 *
 ****************************************************************************/

void dw_pcie_ep_deinit(FAR struct dw_pcie_ep_s *ep)
{
  FAR struct pci_epc_ctrl_s *epc = ep->epc;

  pci_epc_mem_free_addr(epc, ep->msi_mem_phys,
                        epc->mem->page_size);

  pci_epc_mem_exit(epc);
}

/****************************************************************************
 * Name: dw_pcie_ep_linkup
 *
 * Description:
 *   This function is used to start link
 *
 * Input Parameters:
 *   ep               - DWC EP of the endpoint controller
 *
 ****************************************************************************/

void dw_pcie_ep_linkup(FAR struct dw_pcie_ep_s *ep)
{
  FAR struct pci_epc_ctrl_s *epc = ep->epc;

  pci_epc_linkup(epc);
}

/****************************************************************************
 * Name: dw_pcie_ep_linkdown
 *
 * Description:
 *   This function is used to link down with rc.
 *
 * Input Parameters:
 *   ep               - DWC EP of the endpoint controller
 *
 ****************************************************************************/

void dw_pcie_ep_linkdown(FAR struct dw_pcie_ep_s *ep)
{
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);
  FAR struct pci_epc_ctrl_s *epc = ep->epc;

  /* Initialize the non-sticky DWC registers as they would've reset post
   * Link Down. This is specifically needed for drivers not supporting
   * PERST# as they have no way to reinitialize the registers before the
   * link comes back again.
   */

  dw_pcie_ep_init_non_sticky_registers(pci);

  pci_epc_linkdown(epc);
}

/****************************************************************************
 * Name: dw_pcie_ep_init
 *
 * Description:
 *   This function is used to init epc.
 *
 * Input Parameters:
 *   ep               - DWC EP of the endpoint controller
 *   epc_name         - Device name of the endpoint controller
 *
 * Returned Value:
 *   Return 0, errno otherwise.
 *
 ****************************************************************************/

int dw_pcie_ep_init(FAR struct dw_pcie_ep_s *ep, char *epc_name)
{
  int ret;
  FAR struct pci_epc_ctrl_s *epc;

  list_initialize(&ep->func_list);

  if (ep->ops->pre_init)
    {
      ep->ops->pre_init(ep);
    }

  epc = pci_epc_create(epc_name, ep, ep->dma_addr, ep->dma_len, &g_epc_ops);
  if (epc == NULL)
    {
      pcierr("Failed to create epc device\n");
      return -ENOMEM;
    }

  ep->epc = epc;

  epc->max_functions = 1;

  ret = pci_epc_mem_init(epc, (void *)ep->phys_base, ep->phys_base,
                         ep->addr_size, ep->page_size);
  if (ret < 0)
    {
      pcierr("Failed to initialize address space\n");
      return ret;
    }

  ep->msi_mem = pci_epc_mem_alloc_addr(epc, &ep->msi_mem_phys,
               SZ_4K);
  if (!ep->msi_mem)
    {
      ret = -ENOMEM;
      pcierr("Failed to reserve memory for MSI/MSI-X\n");
      goto err_exit_epc_mem;
    }

  ret = dw_pcie_ep_init_registers(ep);
  if (ret)
    {
      pcierr("Failed to initialize DWC endpoint registers\n");
      goto ep_deinit;
    }

  pci_epc_init_notify(ep->epc);

  return 0;

ep_deinit:
      dw_pcie_ep_deinit(ep);
err_exit_epc_mem:
  pci_epc_mem_exit(epc);

  return ret;
}
