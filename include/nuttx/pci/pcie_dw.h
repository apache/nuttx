/****************************************************************************
 * include/nuttx/pci/pcie_dw.h
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

#ifndef _PCIE_DESIGNWARE_H
#define _PCIE_DESIGNWARE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <debug.h>

#include <nuttx/bits.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pci/pci.h>
#include <nuttx/pci/pci_epc.h>
#include <nuttx/pci/pci_epf.h>
#include <nuttx/spinlock.h>
#include <nuttx/list.h>
#include <nuttx/nuttx.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SZ_1K                             0x00000400
#define SZ_4K                             0x00001000
#define SZ_1G                             0x40000000
#define SZ_2G                             0x80000000
#define SZ_4G                             0x100000000

#define upper_32_bits(n)                  ((uint32_t)(((n) >> 16) >> 16))
#define lower_32_bits(n)                  ((uint32_t)(n))
#define readb(a)                          (*(FAR volatile uint8_t *)(a))
#define writeb(v,a)                       (*(FAR volatile uint8_t *)(a) = (v))
#define readw(a)                          (*(FAR volatile uint16_t *)(a))
#define writew(v,a)                       (*(FAR volatile uint16_t *)(a) = (v))
#define readl(a)                          (*(FAR volatile uint32_t *)(a))
#define writel(v,a)                       (*(FAR volatile uint32_t *)(a) = (v))

#define MAX_MSI_IRQS                         256
#define MAX_MSI_IRQS_PER_CTRL                32
#define MAX_MSI_CTRLS                        (MAX_MSI_IRQS / MAX_MSI_IRQS_PER_CTRL)
#define MSI_REG_CTRL_BLOCK_SIZE              12
#define MSI_DEF_NUM_VECTORS                  32

#define to_dw_pcie_from_pp(port)             container_of((port), struct dw_pcie_s, pp)
#define to_dw_pcie_from_ep(endpoint)   \
    container_of((endpoint), struct dw_pcie_s, ep)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct dw_pcie_ep_s
{
  FAR struct pci_epc_ctrl_s       *epc;
  struct list_node                func_list;
  FAR const struct dw_pcie_ep_ops *ops;
  uint64_t                        phys_base;
  size_t                          addr_size;
  FAR void                        *dma_addr;
  size_t                          dma_len;
  size_t                          page_size;
  uint8_t                         bar_to_atu[PCI_STD_NUM_BARS];
  FAR uintptr_t                   *outbound_addr;
  FAR unsigned long               *ib_window_map;
  FAR unsigned long               *ob_window_map;
  FAR void                        *msi_mem;
  uint64_t                        msi_mem_phys;
  FAR struct pci_epf_bar_s        *epf_bar[PCI_STD_NUM_BARS];
};

struct dw_pcie_rp_s
{
  bool                              has_msi_ctrl:1;
  bool                              cfg0_io_shared:1;
  uint64_t                          cfg0_base;
  FAR void                          *va_cfg0_base;
  uint32_t                          cfg0_size;
  uintptr_t                         io_base;
  uint64_t                          io_bus_addr;
  uint32_t                          io_size;
  int                               irq;
  FAR const struct dw_pcie_host_ops *ops;
  int                               msi_irq[MAX_MSI_CTRLS];
  uint64_t                          msi_data;
  uint32_t                          num_vectors;
  uint32_t                          irq_mask[MAX_MSI_CTRLS];
  FAR struct pci_host_bridge        *bridge;
  spinlock_t                        lock;
  DECLARE_BITMAP(msi_irq_in_use, MAX_MSI_IRQS);
  bool                              use_atu_msg;
  int                               msg_atu_index;
  FAR struct resource               *msg_res;
};

struct dw_pcie_s
{
  FAR void                        *dbi_base;
  FAR uintptr_t                   dbi_phys_addr;
  FAR void                        *dbi_base2;
  FAR void                        *atu_base;
  uintptr_t                       atu_phys_addr;
  size_t                          atu_size;
  uint32_t                        num_ib_windows;
  uint32_t                        num_ob_windows;
  uint32_t                        region_align;
  uint64_t                        region_limit;
  struct dw_pcie_rp_s             pp;
  struct dw_pcie_ep_s             ep;
  FAR const struct dw_pcie_ops_s *ops;
  uint32_t                       version;
  uint32_t                       type;
  unsigned long                  caps;
  int                            num_lanes;
  int                            max_link_speed;
  uint8_t                        n_fts[2];
  FAR void                       *priv;
};

enum dw_pcie_ltssm
{
  /* Need to align with PCIE_PORT_DEBUG0 bits 0:5 */

  DW_PCIE_LTSSM_DETECT_QUIET = 0x0,
  DW_PCIE_LTSSM_DETECT_ACT = 0x1,
  DW_PCIE_LTSSM_L0 = 0x11,
  DW_PCIE_LTSSM_L2_IDLE = 0x15,

  DW_PCIE_LTSSM_UNKNOWN = 0xffffffff,
};

struct dw_pcie_ob_atu_cfg_s
{
  int      index;
  int      type;
  uint8_t  funcno;
  uint8_t  code;
  uint8_t  routing;
  uint64_t cpu_addr;
  uint64_t pci_addr;
  uint64_t size;
};

struct dw_pcie_host_ops
{
  CODE int (*init)(FAR struct dw_pcie_rp_s *pp);
  CODE void (*deinit)(FAR struct dw_pcie_rp_s *pp);
  CODE void (*post_init)(FAR struct dw_pcie_rp_s *pp);
  CODE int (*msi_init)(FAR struct dw_pcie_rp_s *pp);
  CODE void (*pme_turn_off)(FAR struct dw_pcie_rp_s *pp);
};

struct dw_pcie_ep_ops
{
  CODE void (*pre_init)(FAR struct dw_pcie_ep_s *ep);
  CODE void (*init)(FAR struct dw_pcie_ep_s *ep);
  CODE int (*raise_irq)(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                        unsigned int type, uint16_t interrupt_num);
  CODE const FAR struct pci_epc_features_s *(*get_features)
      (FAR struct dw_pcie_ep_s *ep);
  CODE uint32_t (*get_dbi_offset)(FAR struct dw_pcie_ep_s *ep,
                                  uint8_t funcno);
  CODE uint32_t (*get_dbi2_offset)(FAR struct dw_pcie_ep_s *ep,
                                   uint8_t funcno);
};

struct dw_pcie_ep_func_s
{
  struct list_node  list;
  uint8_t           funcno;
  uint8_t           msi_cap;  /* MSI capability offset */
  uint8_t           msix_cap; /* MSI-X capability offset */
};

struct dw_pcie_ops_s
{
  CODE uint64_t (*cpu_addr_fixup)(FAR struct dw_pcie_s *pcie,
                                  uint64_t cpu_addr);
  CODE uint32_t (*read_dbi)(FAR struct dw_pcie_s *pcie, FAR void *base,
                            uint32_t reg, size_t size);
  CODE void (*write_dbi)(FAR struct dw_pcie_s *pcie, FAR void *base,
                         uint32_t reg, size_t size, uint32_t val);
  CODE void (*write_dbi2)(FAR struct dw_pcie_s *pcie, FAR void *base,
                          uint32_t reg, size_t size, uint32_t val);
  CODE int (*link_up)(FAR struct dw_pcie_s *pcie);
  CODE enum dw_pcie_ltssm (*get_ltssm)(FAR struct dw_pcie_s *pcie);
  CODE int (*start_link)(FAR struct dw_pcie_s *pcie);
  CODE void (*stop_link)(FAR struct dw_pcie_s *pcie);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int dw_pcie_start_link(FAR struct dw_pcie_s *pci);
void dw_pcie_stop_link(FAR struct dw_pcie_s *pci);
enum dw_pcie_ltssm dw_pcie_get_ltssm(FAR struct dw_pcie_s *pci);
int dw_pcie_ep_init(FAR struct dw_pcie_ep_s *ep, FAR char *epc_name);
void dw_pcie_ep_deinit(FAR struct dw_pcie_ep_s *ep);
int dw_pcie_ep_raise_intx_irq(FAR struct dw_pcie_ep_s *ep, uint8_t funcno);
int dw_pcie_ep_raise_msi_irq(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                             uint8_t interrupt_num);
int dw_pcie_ep_raise_msix_irq(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                              uint16_t interrupt_num);
void dw_pcie_ep_reset_bar(FAR struct dw_pcie_s *pci, int bar);
void dw_pcie_linkcap_update(FAR struct dw_pcie_s *pci, uint32_t new);
int dw_pcie_get_link_cap(FAR struct dw_pcie_s *pci);
void dw_pcie_chang_speed(FAR struct dw_pcie_s *pci);
int dw_pcie_get_link_state(FAR struct dw_pcie_s *pci);
int dw_pcie_wait_for_link(FAR struct dw_pcie_s *pci);
int
dw_pcie_ep_raise_msix_irq_doorbell(FAR struct dw_pcie_ep_s *ep,
                                   uint8_t funcno, uint16_t interrupt_num);
#endif /* _PCIE_DESIGNWARE_H */
