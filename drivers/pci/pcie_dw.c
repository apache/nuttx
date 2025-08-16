/****************************************************************************
 * drivers/pci/pcie_dw.c
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

#include <nuttx/pci/pci_regs.h>
#include <sys/param.h>

#include "pcie_dw.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t
__dw_pcie_find_next_cap(FAR struct dw_pcie_s *pci, uint8_t cap_ptr,
                        uint8_t cap);
FAR static void *
dw_pcie_select_atu(FAR struct dw_pcie_s *pci, uint32_t dir,
                   uint32_t index);
static uint32_t
dw_pcie_readl_atu(FAR struct dw_pcie_s *pci, uint32_t dir, uint32_t index,
                  uint32_t reg);
static void
dw_pcie_writel_atu(FAR struct dw_pcie_s *pci, uint32_t dir, uint32_t index,
                   uint32_t reg, uint32_t val);
static uint32_t
dw_pcie_readl_atu_ob(FAR struct dw_pcie_s *pci, uint32_t index,
                     uint32_t reg);
static inline void
dw_pcie_writel_atu_ob(FAR struct dw_pcie_s *pci, uint32_t index,
                      uint32_t reg, uint32_t val);
static uint32_t dw_pcie_enable_ecrc(uint32_t val);
static uint32_t
dw_pcie_readl_atu_ib(FAR struct dw_pcie_s *pci, uint32_t index,
                     uint32_t reg);
static void
dw_pcie_writel_atu_ib(FAR struct dw_pcie_s *pci, uint32_t index,
                      uint32_t reg, uint32_t val);
static void dw_pcie_link_set_max_speed(FAR struct dw_pcie_s *pci);
static void
dw_pcie_link_set_max_link_width(FAR struct dw_pcie_s *pci,
                                uint32_t num_lanes);
static int dw_pcie_link_up(FAR struct dw_pcie_s *pci);
static uint8_t
dw_pcie_find_capability(FAR struct dw_pcie_s *pci, uint8_t cap);
static int dw_pcie_write(FAR void *addr, int size, uint32_t val);
static int dw_pcie_read(FAR void *addr, int size, FAR uint32_t *val);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t
__dw_pcie_find_next_cap(FAR struct dw_pcie_s *pci, uint8_t cap_ptr,
                        uint8_t cap)
{
  uint8_t next_cap_ptr;
  uint8_t cap_id;
  uint16_t reg;

  if (!cap_ptr)
    {
      return 0;
    }

  reg = dw_pcie_readw_dbi(pci, cap_ptr);
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
  return __dw_pcie_find_next_cap(pci, next_cap_ptr, cap);
}

FAR static void *
dw_pcie_select_atu(FAR struct dw_pcie_s *pci, uint32_t dir,
                   uint32_t index)
{
  if (dw_pcie_cap_is(pci, IATU_UNROLL))
    {
      return pci->atu_base + DW_PCIE_ATU_UNROLL_BASE(dir, index);
    }

  dw_pcie_writel_dbi(pci, DW_PCIE_ATU_VIEWPORT, dir | index);
  return pci->atu_base;
}

static uint32_t
dw_pcie_readl_atu(FAR struct dw_pcie_s *pci, uint32_t dir, uint32_t index,
                  uint32_t reg)
{
  FAR void *base;
  int ret;
  uint32_t val;

  base = dw_pcie_select_atu(pci, dir, index);

  if (pci->ops && pci->ops->read_dbi)
    {
      return pci->ops->read_dbi(pci, base, reg, 4);
    }

  ret = dw_pcie_read(base + reg, 4, &val);
  if (ret)
    {
      pcierr("Read ATU address failed\n");
    }

  return val;
}

static void
dw_pcie_writel_atu(FAR struct dw_pcie_s *pci, uint32_t dir, uint32_t index,
                   uint32_t reg, uint32_t val)
{
  void  *base;
  int ret;

  base = dw_pcie_select_atu(pci, dir, index);

  if (pci->ops && pci->ops->write_dbi)
    {
      pci->ops->write_dbi(pci, base, reg, 4, val);
      return;
    }

  ret = dw_pcie_write(base + reg, 4, val);
  if (ret)
    {
      pcierr("Write ATU address failed\n");
    }
}

static uint32_t
dw_pcie_readl_atu_ob(FAR struct dw_pcie_s *pci, uint32_t index, uint32_t reg)
{
  return dw_pcie_readl_atu(pci, DW_PCIE_ATU_REGION_DIR_OB, index, reg);
}

static void
dw_pcie_writel_atu_ob(FAR struct dw_pcie_s *pci, uint32_t index,
                      uint32_t reg, uint32_t val)
{
  dw_pcie_writel_atu(pci, DW_PCIE_ATU_REGION_DIR_OB, index, reg, val);
}

static uint32_t dw_pcie_enable_ecrc(uint32_t val)
{
  return val | DW_PCIE_ATU_TD;
}

static uint32_t
dw_pcie_readl_atu_ib(FAR struct dw_pcie_s *pci, uint32_t index, uint32_t reg)
{
  return dw_pcie_readl_atu(pci, DW_PCIE_ATU_REGION_DIR_IB, index, reg);
}

static void
dw_pcie_writel_atu_ib(FAR struct dw_pcie_s *pci, uint32_t index,
                      uint32_t reg, uint32_t val)
{
  dw_pcie_writel_atu(pci, DW_PCIE_ATU_REGION_DIR_IB, index, reg, val);
}

static void dw_pcie_link_set_max_speed(FAR struct dw_pcie_s *pci)
{
  uint32_t cap;
  uint32_t ctrl2;
  uint32_t link_speed;
  uint8_t offset = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);

  cap = dw_pcie_readl_dbi(pci, offset + PCI_EXP_LNKCAP);

  /* Even if the platform doesn't want to limit the maximum link speed,
   * just cache the hardware default value so that the vendor drivers can
   * use it to do any link specific configuration.
   */

  if (pci->max_link_speed < 1)
    {
      pci->max_link_speed = FIELD_GET(PCI_EXP_LNKCAP_SLS, cap);
      return;
    }

  ctrl2 = dw_pcie_readl_dbi(pci, offset + PCI_EXP_LNKCTL2);
  ctrl2 &= ~PCI_EXP_LNKCTL2_TLS;

  link_speed = pci->max_link_speed;
  if (link_speed > PCI_EXP_LNKCTL2_TLS_16_0GT)
    {
      link_speed = FIELD_GET(PCI_EXP_LNKCAP_SLS, cap);
      ctrl2 &= ~PCI_EXP_LNKCTL2_HASD;
    }

  dw_pcie_writel_dbi(pci, offset + PCI_EXP_LNKCTL2, ctrl2 | link_speed);

  cap &= ~((uint32_t)PCI_EXP_LNKCAP_SLS);
  dw_pcie_writel_dbi(pci, offset + PCI_EXP_LNKCAP, cap | link_speed);
}

static void
dw_pcie_link_set_max_link_width(FAR struct dw_pcie_s *pci,
                                uint32_t num_lanes)
{
  uint32_t lnkcap;
  uint32_t lwsc;
  uint32_t plc;
  uint8_t cap;

  if (!num_lanes)
    {
      return;
    }

  /* Set the number of lanes */

  plc = dw_pcie_readl_dbi(pci, DW_PCIE_PORT_LINK_CONTROL);
  plc &= ~DW_PCIE_PORT_LINK_FAST_LINK_MODE;
  plc &= ~DW_PCIE_PORT_LINK_MODE_MASK;

  /* Set link width speed control register */

  lwsc = dw_pcie_readl_dbi(pci, DW_PCIE_LINK_WIDTH_SPEED_CONTROL);
  lwsc &= ~DW_PCIE_PORT_LOGIC_LINK_WIDTH_MASK;
  switch (num_lanes)
    {
      case 1:
        plc |= DW_PCIE_PORT_LINK_MODE_1_LANES;
        lwsc |= DW_PCIE_PORT_LOGIC_LINK_WIDTH_1_LANES;
        break;
      case 2:
        plc |= DW_PCIE_PORT_LINK_MODE_2_LANES;
        lwsc |= DW_PCIE_PORT_LOGIC_LINK_WIDTH_2_LANES;
        break;
      case 4:
        plc |= DW_PCIE_PORT_LINK_MODE_4_LANES;
        lwsc |= DW_PCIE_PORT_LOGIC_LINK_WIDTH_4_LANES;
        break;
      case 8:
        plc |= DW_PCIE_PORT_LINK_MODE_8_LANES;
        lwsc |= DW_PCIE_PORT_LOGIC_LINK_WIDTH_8_LANES;
        break;
      default:
        pcierr("num-lanes %u: invalid value\n", num_lanes);
        return;
    }

  dw_pcie_writel_dbi(pci, DW_PCIE_PORT_LINK_CONTROL, plc);
  dw_pcie_writel_dbi(pci, DW_PCIE_LINK_WIDTH_SPEED_CONTROL, lwsc);

  cap = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
  lnkcap = dw_pcie_readl_dbi(pci, cap + PCI_EXP_LNKCAP);
  lnkcap &= ~PCI_EXP_LNKCAP_MLW;
  lnkcap |= FIELD_PREP(PCI_EXP_LNKCAP_MLW, num_lanes);
  dw_pcie_writel_dbi(pci, cap + PCI_EXP_LNKCAP, lnkcap);
}

static int dw_pcie_link_up(FAR struct dw_pcie_s *pci)
{
  uint32_t val;

  if (pci->ops && pci->ops->link_up)
    {
      return pci->ops->link_up(pci);
    }

  val = dw_pcie_readl_dbi(pci, DW_PCIE_PORT_DEBUG1);
  return ((val & DW_PCIE_PORT_DEBUG1_LINK_UP) &&
    (!(val & DW_PCIE_PORT_DEBUG1_LINK_IN_TRAINING)));
}

static uint8_t
dw_pcie_find_capability(FAR struct dw_pcie_s *pci, uint8_t cap)
{
  uint8_t next_cap_ptr;
  uint16_t reg;

  reg = dw_pcie_readw_dbi(pci, PCI_CAPABILITY_LIST);
  next_cap_ptr = (reg & 0x00ff);

  return __dw_pcie_find_next_cap(pci, next_cap_ptr, cap);
}

static int dw_pcie_read(FAR void *addr, int size, FAR uint32_t *val)
{
  if (!IS_ALIGNED((uintptr_t)addr, size))
    {
      *val = 0;
      return -EFAULT;
    }

  if (size == 4)
    {
      *val = readl(addr);
    }
  else if (size == 2)
    {
      *val = readw(addr);
    }
  else if (size == 1)
    {
    *val = readb(addr);
    }
  else
    {
      *val = 0;
      return -EFAULT;
    }

  return OK;
}

static int dw_pcie_write(FAR void *addr, int size, uint32_t val)
{
  if (!IS_ALIGNED((uintptr_t)addr, size))
    {
      return -EFAULT;
    }

  if (size == 4)
    {
      writel(val, addr);
    }
  else if (size == 2)
    {
      writew(val, addr);
    }
  else if (size == 1)
    {
      writeb(val, addr);
    }
  else
    {
      return -EFAULT;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void dw_pcie_version_detect(FAR struct dw_pcie_s *pci)
{
  uint32_t ver;

  /* The content of the CSR is zero on DWC PCIe older than v4.70a */

  ver = dw_pcie_readl_dbi(pci, DW_PCIE_VERSION_NUMBER);
  if (!ver)
    {
      return;
    }

  if (pci->version && pci->version != ver)
    {
      pcierr("Versions don't match (%08x != %08x)\n",
             pci->version, ver);
    }
  else
    {
      pci->version = ver;
    }

  ver = dw_pcie_readl_dbi(pci, DW_PCIE_VERSION_TYPE);

  if (pci->type && pci->type != ver)
    {
      pcierr("Types don't match (%08x != %08x)\n",
             pci->type, ver);
    }
  else
    {
      pci->type = ver;
    }
}

uint32_t
dw_pcie_read_dbi(FAR struct dw_pcie_s *pci, uint32_t reg, size_t size)
{
  int ret;
  uint32_t val;

  if (pci->ops && pci->ops->read_dbi)
    {
      return pci->ops->read_dbi(pci, pci->dbi_base, reg, size);
    }

  ret = dw_pcie_read(pci->dbi_base + reg, size, &val);
  if (ret)
    {
      pcierr("Read DBI address failed\n");
    }

  return val;
}

void
dw_pcie_write_dbi(FAR struct dw_pcie_s *pci, uint32_t reg, size_t size,
                  uint32_t val)
{
  int ret;

  if (pci->ops && pci->ops->write_dbi)
    {
      pci->ops->write_dbi(pci, pci->dbi_base, reg, size, val);
      return;
    }

  ret = dw_pcie_write(pci->dbi_base + reg, size, val);
  if (ret)
    {
      pcierr("Write DBI address failed\n");
    }
}

void
dw_pcie_write_dbi2(FAR struct dw_pcie_s *pci, uint32_t reg, size_t size,
                   uint32_t val)
{
  int ret;

  if (pci->ops && pci->ops->write_dbi2)
    {
      pci->ops->write_dbi2(pci, pci->dbi_base2, reg, size, val);
      return;
    }

  ret = dw_pcie_write(pci->dbi_base2 + reg, size, val);
  if (ret)
    {
      pcierr("write DBI address failed\n");
    }
}

void
dw_pcie_writel_dbi(FAR struct dw_pcie_s *pci, uint32_t reg, uint32_t val)
{
  dw_pcie_write_dbi(pci, reg, 0x4, val);
}

uint32_t dw_pcie_readl_dbi(FAR struct dw_pcie_s *pci, uint32_t reg)
{
  return dw_pcie_read_dbi(pci, reg, 0x4);
}

void
dw_pcie_writew_dbi(FAR struct dw_pcie_s *pci, uint32_t reg, uint16_t val)
{
  dw_pcie_write_dbi(pci, reg, 0x2, val);
}

uint16_t dw_pcie_readw_dbi(FAR struct dw_pcie_s *pci, uint32_t reg)
{
  return dw_pcie_read_dbi(pci, reg, 0x2);
}

void dw_pcie_writeb_dbi(FAR struct dw_pcie_s *pci, uint32_t reg, uint8_t val)
{
  dw_pcie_write_dbi(pci, reg, 0x1, val);
}

uint8_t dw_pcie_readb_dbi(FAR struct dw_pcie_s *pci, uint32_t reg)
{
  return dw_pcie_read_dbi(pci, reg, 0x1);
}

void
dw_pcie_writel_dbi2(FAR struct dw_pcie_s *pci, uint32_t reg, uint32_t val)
{
  dw_pcie_write_dbi2(pci, reg, 0x4, val);
}

uint32_t
dw_pcie_ep_get_dbi_offset(FAR struct dw_pcie_ep_s *ep, uint8_t funcno)
{
  unsigned int dbi_offset = 0;

  if (ep->ops->get_dbi_offset)
    dbi_offset = ep->ops->get_dbi_offset(ep, funcno);

  return dbi_offset;
}

uint32_t dw_pcie_ep_read_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                    uint32_t reg, size_t size)
{
  unsigned int offset = dw_pcie_ep_get_dbi_offset(ep, funcno);
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);

  return dw_pcie_read_dbi(pci, offset + reg, size);
}

void dw_pcie_ep_write_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                     uint32_t reg, size_t size, uint32_t val)
{
  unsigned int offset = dw_pcie_ep_get_dbi_offset(ep, funcno);
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);

  dw_pcie_write_dbi(pci, offset + reg, size, val);
}

void dw_pcie_ep_writel_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                      uint32_t reg, uint32_t val)
{
  dw_pcie_ep_write_dbi(ep, funcno, reg, 0x4, val);
}

uint32_t dw_pcie_ep_readl_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                     uint32_t reg)
{
  return dw_pcie_ep_read_dbi(ep, funcno, reg, 0x4);
}

void dw_pcie_ep_writew_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                      uint32_t reg, uint16_t val)
{
  dw_pcie_ep_write_dbi(ep, funcno, reg, 0x2, val);
}

uint16_t dw_pcie_ep_readw_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                     uint32_t reg)
{
  return dw_pcie_ep_read_dbi(ep, funcno, reg, 0x2);
}

void dw_pcie_ep_writeb_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                      uint32_t reg, uint8_t val)
{
  dw_pcie_ep_write_dbi(ep, funcno, reg, 0x1, val);
}

uint8_t dw_pcie_ep_readb_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                     uint32_t reg)
{
  return dw_pcie_ep_read_dbi(ep, funcno, reg, 0x1);
}

uint32_t
dw_pcie_ep_get_dbi2_offset(FAR struct dw_pcie_ep_s *ep, uint8_t funcno)
{
  unsigned int dbi2_offset = 0;

  if (ep->ops->get_dbi2_offset)
    {
      dbi2_offset = ep->ops->get_dbi2_offset(ep, funcno);
    }
  else if (ep->ops->get_dbi_offset)     /* for backward compatibility */
    {
      dbi2_offset = ep->ops->get_dbi_offset(ep, funcno);
    }

  return dbi2_offset;
}

void dw_pcie_ep_write_dbi2(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                      uint32_t reg, size_t size, uint32_t val)
{
  unsigned int offset = dw_pcie_ep_get_dbi2_offset(ep, funcno);
  FAR struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);

  dw_pcie_write_dbi2(pci, offset + reg, size, val);
}

void dw_pcie_ep_writel_dbi2(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                       uint32_t reg, uint32_t val)
{
  dw_pcie_ep_write_dbi2(ep, funcno, reg, 0x4, val);
}

void dw_pcie_dbi_ro_wr_en(FAR struct dw_pcie_s *pci)
{
  uint32_t reg;
  uint32_t val;

  reg = DW_PCIE_MISC_CONTROL_1_OFF;
  val = dw_pcie_readl_dbi(pci, reg);
  val |= DW_PCIE_DBI_RO_WR_EN;
  dw_pcie_writel_dbi(pci, reg, val);
}

void dw_pcie_dbi_ro_wr_dis(FAR struct dw_pcie_s *pci)
{
  uint32_t reg;
  uint32_t val;

  reg = DW_PCIE_MISC_CONTROL_1_OFF;
  val = dw_pcie_readl_dbi(pci, reg);
  val &= ~DW_PCIE_DBI_RO_WR_EN;
  dw_pcie_writel_dbi(pci, reg, val);
}

int dw_pcie_start_link(FAR struct dw_pcie_s *pci)
{
  if (pci->ops && pci->ops->start_link)
    {
      return pci->ops->start_link(pci);
    }

  return 0;
}

void dw_pcie_stop_link(FAR struct dw_pcie_s *pci)
{
  if (pci->ops && pci->ops->stop_link)
    {
      pci->ops->stop_link(pci);
    }
}

enum dw_pcie_ltssm dw_pcie_get_ltssm(FAR struct dw_pcie_s *pci)
{
  uint32_t val;

  if (pci->ops && pci->ops->get_ltssm)
    {
      return pci->ops->get_ltssm(pci);
    }

  val = dw_pcie_readl_dbi(pci, DW_PCIE_PORT_DEBUG0);

  return (enum dw_pcie_ltssm)FIELD_GET(
          DW_PCIE_PORT_LOGIC_LTSSM_STATE_MASK, val);
}

int dw_pcie_prog_outbound_atu(FAR struct dw_pcie_s *pci,
                              FAR const struct dw_pcie_ob_atu_cfg_s *atu)
{
  uint64_t cpu_addr = atu->cpu_addr;
  uint32_t retries;
  uint32_t val;
  uint64_t limit_addr;

  if (pci->ops && pci->ops->cpu_addr_fixup)
    {
      cpu_addr = pci->ops->cpu_addr_fixup(pci, cpu_addr);
    }

  limit_addr = cpu_addr + atu->size - 1;

  if ((limit_addr & ~pci->region_limit) != (cpu_addr & ~pci->region_limit) ||
      !IS_ALIGNED(cpu_addr, pci->region_align) ||
      !IS_ALIGNED(atu->pci_addr, pci->region_align) || !atu->size)
    {
      return -EINVAL;
    }

  dw_pcie_writel_atu_ob(pci, atu->index, DW_PCIE_ATU_LOWER_BASE,
            lower_32_bits(cpu_addr));
  dw_pcie_writel_atu_ob(pci, atu->index, DW_PCIE_ATU_UPPER_BASE,
            upper_32_bits(cpu_addr));

  dw_pcie_writel_atu_ob(pci, atu->index, DW_PCIE_ATU_LIMIT,
            lower_32_bits(limit_addr));
  if (dw_pcie_ver_is_ge(pci, 460A))
    {
      dw_pcie_writel_atu_ob(pci, atu->index, DW_PCIE_ATU_UPPER_LIMIT,
                upper_32_bits(limit_addr));
    }

  dw_pcie_writel_atu_ob(pci, atu->index, DW_PCIE_ATU_LOWER_TARGET,
            lower_32_bits(atu->pci_addr));
  dw_pcie_writel_atu_ob(pci, atu->index, DW_PCIE_ATU_UPPER_TARGET,
            upper_32_bits(atu->pci_addr));

  val = atu->type | atu->routing | DW_PCIE_ATU_FUNC_NUM(atu->funcno);
  if (upper_32_bits(limit_addr) > upper_32_bits(cpu_addr) &&
      dw_pcie_ver_is_ge(pci, 460A))
    {
      val |= DW_PCIE_ATU_INCREASE_REGION_SIZE;
    }

  if (dw_pcie_ver_is(pci, 490A))
    {
      val = dw_pcie_enable_ecrc(val);
    }

  dw_pcie_writel_atu_ob(pci, atu->index, DW_PCIE_ATU_REGION_CTRL1, val);

  val = DW_PCIE_ATU_ENABLE;
  if (atu->type == DW_PCIE_ATU_TYPE_MSG)
    {
      /* The data-less messages only for now */

      val |= DW_PCIE_ATU_INHIBIT_PAYLOAD | atu->code;
    }

  dw_pcie_writel_atu_ob(pci, atu->index, DW_PCIE_ATU_REGION_CTRL2, val);

  /* Make sure ATU enable takes effect before any subsequent config
   * and I/O accesses.
   */

  for (retries = 0; retries < DW_PCIE_LINK_WAIT_MAX_IATU_RETRIES; retries++)
    {
      val = dw_pcie_readl_atu_ob(pci, atu->index, DW_PCIE_ATU_REGION_CTRL2);
      if (val & DW_PCIE_ATU_ENABLE)
        {
          return 0;
        }

      up_mdelay(DW_PCIE_LINK_WAIT_IATU);
    }

  pcierr("Outbound iATU is not being enabled\n");

  return -ETIMEDOUT;
}

int dw_pcie_prog_inbound_atu(FAR struct dw_pcie_s *pci, int index, int type,
                             uint64_t cpu_addr, uint64_t pci_addr,
                             uint64_t size)
{
  uint64_t limit_addr = pci_addr + size - 1;
  uint32_t retries;
  uint32_t val;

  if ((limit_addr & ~pci->region_limit) != (pci_addr & ~pci->region_limit) ||
      !IS_ALIGNED(cpu_addr, pci->region_align) ||
      !IS_ALIGNED(pci_addr, pci->region_align) || !size)
    {
      return -EINVAL;
    }

  dw_pcie_writel_atu_ib(pci, index, DW_PCIE_ATU_LOWER_BASE,
            lower_32_bits(pci_addr));
  dw_pcie_writel_atu_ib(pci, index, DW_PCIE_ATU_UPPER_BASE,
            upper_32_bits(pci_addr));

  dw_pcie_writel_atu_ib(pci, index, DW_PCIE_ATU_LIMIT,
            lower_32_bits(limit_addr));
  if (dw_pcie_ver_is_ge(pci, 460A))
    {
      dw_pcie_writel_atu_ib(pci, index, DW_PCIE_ATU_UPPER_LIMIT,
                upper_32_bits(limit_addr));
    }

  dw_pcie_writel_atu_ib(pci, index, DW_PCIE_ATU_LOWER_TARGET,
            lower_32_bits(cpu_addr));
  dw_pcie_writel_atu_ib(pci, index, DW_PCIE_ATU_UPPER_TARGET,
            upper_32_bits(cpu_addr));

  val = type;
  if (upper_32_bits(limit_addr) > upper_32_bits(pci_addr) &&
      dw_pcie_ver_is_ge(pci, 460A))
    {
      val |= DW_PCIE_ATU_INCREASE_REGION_SIZE;
    }

  dw_pcie_writel_atu_ib(pci, index, DW_PCIE_ATU_REGION_CTRL1, val);
  dw_pcie_writel_atu_ib(pci, index, DW_PCIE_ATU_REGION_CTRL2,
                        DW_PCIE_ATU_ENABLE);

  /* Make sure ATU enable takes effect before any subsequent config
   * and I/O accesses.
   */

  for (retries = 0; retries < DW_PCIE_LINK_WAIT_MAX_IATU_RETRIES; retries++)
    {
      val = dw_pcie_readl_atu_ib(pci, index, DW_PCIE_ATU_REGION_CTRL2);
      if (val & DW_PCIE_ATU_ENABLE)
        {
          return 0;
        }

      up_mdelay(DW_PCIE_LINK_WAIT_IATU);
    }

  pcierr("Inbound iATU is not being enabled\n");

  return -ETIMEDOUT;
}

int
dw_pcie_prog_ep_inbound_atu(FAR struct dw_pcie_s *pci, uint8_t funcno,
                            int index, int type, uint64_t cpu_addr,
                            uint8_t bar)
{
  uint32_t retries;
  uint32_t val;

  if (!IS_ALIGNED(cpu_addr, pci->region_align))
    {
      return -EINVAL;
    }

  dw_pcie_writel_atu_ib(pci, index, DW_PCIE_ATU_LOWER_TARGET,
            lower_32_bits(cpu_addr));
  dw_pcie_writel_atu_ib(pci, index, DW_PCIE_ATU_UPPER_TARGET,
            upper_32_bits(cpu_addr));

  dw_pcie_writel_atu_ib(pci, index, DW_PCIE_ATU_REGION_CTRL1, type |
            DW_PCIE_ATU_FUNC_NUM(funcno));
  dw_pcie_writel_atu_ib(pci, index, DW_PCIE_ATU_REGION_CTRL2,
            DW_PCIE_ATU_ENABLE | DW_PCIE_ATU_FUNC_NUM_MATCH_EN |
            DW_PCIE_ATU_BAR_MODE_ENABLE | (bar << 8));

  /* Make sure ATU enable takes effect before any subsequent config
   * and I/O accesses.
   */

  for (retries = 0; retries < DW_PCIE_LINK_WAIT_MAX_IATU_RETRIES; retries++)
    {
      val = dw_pcie_readl_atu_ib(pci, index, DW_PCIE_ATU_REGION_CTRL2);
      if (val & DW_PCIE_ATU_ENABLE)
        {
          return 0;
        }

      up_udelay(DW_PCIE_LINK_WAIT_IATU * 1000);
    }

  pcierr("Inbound iATU is not being enabled\n");

  return -ETIMEDOUT;
}

void dw_pcie_disable_atu(FAR struct dw_pcie_s *pci, uint32_t dir, int index)
{
  dw_pcie_writel_atu(pci, dir, index, DW_PCIE_ATU_REGION_CTRL2, 0);
}

int dw_pcie_wait_for_link(FAR struct dw_pcie_s *pci)
{
  uint32_t offset;
  uint32_t val;
  int retries;

  /* Check if the link is up or not */

  for (retries = 0; retries < DW_PCIE_LINK_WAIT_MAX_RETRIES; retries++)
    {
      if (dw_pcie_link_up(pci))
        {
          break;
        }

      up_mdelay(DW_PCIE_LINK_WAIT_SLEEP_MS * 100);
    }

  if (retries >= DW_PCIE_LINK_WAIT_MAX_RETRIES)
    {
      pcierr("Phy link never came up\n");
      return -ETIMEDOUT;
    }

  offset = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
  val = dw_pcie_readw_dbi(pci, offset + PCI_EXP_LNKSTA);

  pcierr("PCIe Gen.%u x%u link up\n",
         FIELD_GET(PCI_EXP_LNKSTA_CLS, val),
         FIELD_GET(PCI_EXP_LNKSTA_NLW, val));

  return 0;
}

void dw_pcie_linkcap_update(FAR struct dw_pcie_s *pci, uint32_t new)
{
  uint32_t val;
  uint8_t offset = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);

  val = dw_pcie_readl_dbi(pci, offset + PCI_EXP_LNKCAP);
  dw_pcie_dbi_ro_wr_en(pci);
  val &= ~PCI_EXP_LNKCAP_SLS;
  val |= new;
  dw_pcie_writel_dbi(pci, offset + PCI_EXP_LNKCAP, val);
  dw_pcie_dbi_ro_wr_dis(pci);
}

int dw_pcie_get_link_cap(FAR struct dw_pcie_s *pci)
{
  uint8_t offset = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);

  return dw_pcie_readl_dbi(pci, offset + PCI_EXP_LNKCAP);
}

void dw_pcie_chang_speed(FAR struct dw_pcie_s *pci)
{
  uint32_t tmp;

  dw_pcie_dbi_ro_wr_en(pci);
  tmp = dw_pcie_readl_dbi(pci, DW_PCIE_LINK_WIDTH_SPEED_CONTROL);
  tmp |= DW_PCIE_PORT_LOGIC_SPEED_CHANGE;
  dw_pcie_writel_dbi(pci, DW_PCIE_LINK_WIDTH_SPEED_CONTROL, tmp);
  dw_pcie_dbi_ro_wr_dis(pci);
}

int dw_pcie_get_link_state(FAR struct dw_pcie_s *pci)
{
  uint8_t offset = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);

  return dw_pcie_readw_dbi(pci, offset + PCI_EXP_LNKSTA) &
         PCI_EXP_LNKSTA_CLS;
}

void dw_pcie_iatu_detect(FAR struct dw_pcie_s *pci)
{
  int max_region;
  uint64_t max;
  uint32_t val;
  uint32_t min;
  uint32_t dir;
  int ob;
  int ib;

  val = dw_pcie_readl_dbi(pci, DW_PCIE_ATU_VIEWPORT);
  if (val == 0xffffffff)
    {
      dw_pcie_cap_set(pci, IATU_UNROLL);

      max_region = MIN((int)pci->atu_size / 512, 256);
    }
  else
    {
      pci->atu_base = pci->dbi_base + DW_PCIE_ATU_VIEWPORT_BASE;
      pci->atu_size = DW_PCIE_ATU_VIEWPORT_SIZE;

      dw_pcie_writel_dbi(pci, DW_PCIE_ATU_VIEWPORT, 0xff);
      max_region = dw_pcie_readl_dbi(pci, DW_PCIE_ATU_VIEWPORT) + 1;
    }

  for (ob = 0; ob < max_region; ob++)
    {
      dw_pcie_writel_atu_ob(pci, ob, DW_PCIE_ATU_LOWER_TARGET, 0x11110000);
      val = dw_pcie_readl_atu_ob(pci, ob, DW_PCIE_ATU_LOWER_TARGET);
      if (val != 0x11110000)
        {
          break;
        }
    }

  for (ib = 0; ib < max_region; ib++)
    {
      dw_pcie_writel_atu_ib(pci, ib, DW_PCIE_ATU_LOWER_TARGET, 0x11110000);
      val = dw_pcie_readl_atu_ib(pci, ib, DW_PCIE_ATU_LOWER_TARGET);
      if (val != 0x11110000)
        {
          break;
        }
    }

  if (ob)
    {
      dir = DW_PCIE_ATU_REGION_DIR_OB;
    }
  else if (ib)
    {
      dir = DW_PCIE_ATU_REGION_DIR_IB;
    }
  else
    {
      pcierr("No iATU regions found\n");
      return;
    }

  dw_pcie_writel_atu(pci, dir, 0, DW_PCIE_ATU_LIMIT, 0x0);
  min = dw_pcie_readl_atu(pci, dir, 0, DW_PCIE_ATU_LIMIT);

  if (dw_pcie_ver_is_ge(pci, 460A))
    {
      dw_pcie_writel_atu(pci, dir, 0, DW_PCIE_ATU_UPPER_LIMIT, 0xffffffff);
      max = dw_pcie_readl_atu(pci, dir, 0, DW_PCIE_ATU_UPPER_LIMIT);
    }
  else
    {
      max = 0;
    }

  pci->num_ob_windows = ob;
  pci->num_ib_windows = ib;
  pci->region_align = 1 << fls(min);
  pci->region_limit = (max << 32) | (SZ_4G - 1);

  pcierr("iATU: unroll %s, %u ob, %u ib, align %uK, limit %luG\n",
         dw_pcie_cap_is(pci, IATU_UNROLL) ? "T" : "F",
         pci->num_ob_windows, pci->num_ib_windows,
         pci->region_align / SZ_1K, (pci->region_limit + 1) / SZ_1G);
  dw_pcie_dbi_ro_wr_en(pci);
  dw_pcie_dbi_ro_wr_dis(pci);
}

void dw_pcie_setup(FAR struct dw_pcie_s *pci)
{
  uint32_t val;

  dw_pcie_link_set_max_speed(pci);

  /* Configure Gen1 N_FTS */

  if (pci->n_fts[0])
    {
      val = dw_pcie_readl_dbi(pci, DW_PCIE_PORT_AFR);
      val &= ~(DW_PCIE_PORT_AFR_N_FTS_MASK | DW_PCIE_PORT_AFR_CC_N_FTS_MASK);
      val |= DW_PCIE_PORT_AFR_N_FTS(pci->n_fts[0]);
      val |= DW_PCIE_PORT_AFR_CC_N_FTS(pci->n_fts[0]);
      dw_pcie_writel_dbi(pci, DW_PCIE_PORT_AFR, val);
    }

  /* Configure Gen2+ N_FTS */

  if (pci->n_fts[1])
    {
      val = dw_pcie_readl_dbi(pci, DW_PCIE_LINK_WIDTH_SPEED_CONTROL);
      val &= ~DW_PCIE_PORT_LOGIC_N_FTS_MASK;
      val |= pci->n_fts[1];
      dw_pcie_writel_dbi(pci, DW_PCIE_LINK_WIDTH_SPEED_CONTROL, val);
    }

  if (dw_pcie_cap_is(pci, CDM_CHECK))
    {
      val = dw_pcie_readl_dbi(pci, DW_PCIE_PL_CHK_REG_CONTROL_STATUS);
      val |= DW_PCIE_PL_CHK_REG_CHK_REG_CONTINUOUS |
             DW_PCIE_PL_CHK_REG_CHK_REG_START;
      dw_pcie_writel_dbi(pci, DW_PCIE_PL_CHK_REG_CONTROL_STATUS, val);
    }

  val = dw_pcie_readl_dbi(pci, DW_PCIE_PORT_LINK_CONTROL);
  val &= ~DW_PCIE_PORT_LINK_FAST_LINK_MODE;
  val |= DW_PCIE_PORT_LINK_DLL_LINK_EN;
  dw_pcie_writel_dbi(pci, DW_PCIE_PORT_LINK_CONTROL, val);

  dw_pcie_link_set_max_link_width(pci, pci->num_lanes);
}
