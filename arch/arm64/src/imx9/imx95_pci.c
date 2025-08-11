/****************************************************************************
 * arch/arm64/src/imx9/imx95_pci.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/mm/mm.h>
#include <nuttx/pci/pcie_dw.h>
#include "imx95_pci.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMX95_PCIE_PHY_GEN_CTRL     0x0
#define IMX95_PCIE_REF_USE_PAD      BIT(17)

#define IMX95_PCIE_SS_RW_REG_0      0xf0
#define IMX95_PCIE_REF_CLKEN        BIT(23)
#define IMX95_PCIE_PHY_CR_PARA_SEL  BIT(9)
#define IMX95_PCIE_SS_RW_REG_1      0xf4
#define IMX95_PCIE_SYS_AUX_PWR_DET  BIT(31)

#define IMX95_PE0_GEN_CTRL_1        0x1050
#define IMX95_PCIE_DEVICE_TYPE      GENMASK(3, 0)

#define IMX95_PE0_GEN_CTRL_3        0x1058
#define IMX95_PCIE_LTSSM_EN         BIT(0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imx95_pcie_s
{
  struct dw_pcie_s    pci;
  FAR void            *iomuxc_gpr;
  bool                link_is_up;
  mutex_t             lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void
reg_update_bits(void *base, uint32_t reg, uint32_t mask, uint32_t val);
static int imx95_pcie_start_link(struct dw_pcie_s *pci);
static void imx95_pcie_stop_link(struct dw_pcie_s *pci);
static uint64_t
imx95_pcie_cpu_addr_fixup(struct dw_pcie_s *pcie, uint64_t cpu_addr);
static int imx95_pcie_host_init(struct dw_pcie_rp_s *pp);
static void imx95_pcie_host_exit(struct dw_pcie_rp_s *pp);
static int imx95_pcie_init_phy(struct imx95_pcie_s *imx95_pcie_s);
static void imx95_pcie_configure_type(struct imx95_pcie_s *imx95_pcie_s);
static void imx95_pcie_ltssm_enable(struct imx95_pcie_s *imx95_pcie_s);
static void imx95_pcie_ltssm_disable(struct imx95_pcie_s *imx95_pcie_s);
static void imx95_pcie_ep_init(struct dw_pcie_ep_s *ep);
static int
imx95_pcie_ep_raise_irq(struct dw_pcie_ep_s *ep, uint8_t funcno,
                      enum pci_epc_irq_type_e type, uint16_t interrupt_num);
static const struct pci_epc_features_s *
imx95_pcie_ep_get_features(struct dw_pcie_ep_s *ep);
static int imx95_add_pcie_ep(struct imx95_pcie_s *imx95_pcie_s);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pci_epc_features_s g_imx95_pcie_epc_features =
{
  .msi_capable = true,
  .align       = SZ_4K,
};

static const struct dw_pcie_ops_s g_dw_pcie_ops =
{
  .start_link     = imx95_pcie_start_link,
  .stop_link      = imx95_pcie_stop_link,
  .cpu_addr_fixup = imx95_pcie_cpu_addr_fixup,
};

static const struct dw_pcie_host_ops g_imx95_pcie_host_dw_pme_ops =
{
  .init   = imx95_pcie_host_init,
  .deinit = imx95_pcie_host_exit,
};

struct imx95_pcie_s g_imx95_pcie =
{
  .pci =
    {
      .dbi_base = (void *)PCIE1_DBI_BASE,
      .dbi_base2 = (void *)PCIE1_DBI2_BASE,
      .atu_base = (void *)PCIE1_ATU_BASE,
      .atu_size = PCIE1_ATU_SIZE,
      .ep.phys_base = PCIE1_OB_BASE,
      .ep.addr_size = PCIE1_OB_SIZE,
      .ep.dma_addr = (void *)PCIE1_DMA_BASE,
      .ep.dma_len = PCIE1_DMA_SIZE,
      .max_link_speed = PCIE1_LINK_SPEED,
      .num_lanes = PCIE1_LINK_LANES,
      .ops = &g_dw_pcie_ops,
      .pp.ops = &g_imx95_pcie_host_dw_pme_ops,
    },
  .iomuxc_gpr = (void *)PCIE1_APP_BASE,
};

static const struct dw_pcie_ep_ops g_pcie_ep_ops =
{
  .init         = imx95_pcie_ep_init,
  .raise_irq    = imx95_pcie_ep_raise_irq,
  .get_features = imx95_pcie_ep_get_features,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void
reg_update_bits(void *base, uint32_t reg, uint32_t mask, uint32_t val)
{
  uint32_t tmp;
  uint32_t orig;
  orig = readl(base + reg);
  tmp = orig & ~mask;
  tmp |= val & mask;
  writel(tmp, base + reg);
}

static int imx95_pcie_init_phy(struct imx95_pcie_s *imx95_pcie_s)
{
  reg_update_bits(imx95_pcie_s->iomuxc_gpr, IMX95_PCIE_SS_RW_REG_1,
      IMX95_PCIE_SYS_AUX_PWR_DET, IMX95_PCIE_PHY_CR_PARA_SEL);

  reg_update_bits(imx95_pcie_s->iomuxc_gpr,
      IMX95_PCIE_SS_RW_REG_0,
      IMX95_PCIE_PHY_CR_PARA_SEL,
      IMX95_PCIE_PHY_CR_PARA_SEL);

  reg_update_bits(imx95_pcie_s->iomuxc_gpr,
         IMX95_PCIE_PHY_GEN_CTRL,
         IMX95_PCIE_REF_USE_PAD, 0);
  reg_update_bits(imx95_pcie_s->iomuxc_gpr,
         IMX95_PCIE_SS_RW_REG_0,
         IMX95_PCIE_REF_CLKEN,
         IMX95_PCIE_REF_CLKEN);

  return 0;
}

static void imx95_pcie_configure_type(struct imx95_pcie_s *imx95_pcie_s)
{
  unsigned int val;

  val = PCI_EXP_TYPE_ENDPOINT << (ffs(IMX95_PCIE_DEVICE_TYPE) - 1);
  reg_update_bits(imx95_pcie_s->iomuxc_gpr, IMX95_PE0_GEN_CTRL_1,
                  IMX95_PCIE_DEVICE_TYPE, val);
}

static void imx95_pcie_ltssm_enable(struct imx95_pcie_s *imx95_pcie_s)
{
  reg_update_bits(imx95_pcie_s->iomuxc_gpr, IMX95_PE0_GEN_CTRL_3,
                  IMX95_PCIE_LTSSM_EN, IMX95_PCIE_LTSSM_EN);
}

static void imx95_pcie_ltssm_disable(struct imx95_pcie_s *imx95_pcie_s)
{
  reg_update_bits(imx95_pcie_s->iomuxc_gpr, IMX95_PE0_GEN_CTRL_3,
         IMX95_PCIE_LTSSM_EN, 0);
}

static int imx95_pcie_start_link(struct dw_pcie_s *pci)
{
  struct imx95_pcie_s *imx95_pcie_s = pci->priv;
  uint32_t link_cap;
  uint32_t tmp;
  int ret;

  /* Force Gen1 operation when starting the link.  In case the link is
   * started in Gen2 mode, there is a possibility the devices on the
   * bus will not be detected at all.  This happens with PCIe switches.
   */

  link_cap = dw_pcie_get_link_cap(pci);
  dw_pcie_linkcap_update(pci, PCI_EXP_LNKCAP_SLS_2_5GB);

  /* Start LTSSM. */

  imx95_pcie_ltssm_enable(imx95_pcie_s);

  ret = dw_pcie_wait_for_link(pci);
  if (ret)
    {
      goto err_reset_phy;
    }

  if (pci->max_link_speed > 1)
    {
      /* Allow faster modes after the link is up */

      dw_pcie_linkcap_update(pci, pci->max_link_speed);

      /* Start Directed Speed Change so the best possible
       * speed both link partners support can be negotiated.
       */

      dw_pcie_chang_speed(pci);

      /* Make sure link training is finished as well! */

      ret = dw_pcie_wait_for_link(pci);
      if (ret)
        {
          goto err_reset_phy;
        }
    }
  else
    {
      pciinfo("Link: Only Gen1 is enabled\n");
    }

  imx95_pcie_s->link_is_up = true;
  tmp = dw_pcie_get_link_state(pci);
  pciinfo("Link up, Gen%i\n", tmp);
  return 0;

err_reset_phy:
  imx95_pcie_s->link_is_up = false;
  pcierr("Pcie start err\n");
  dw_pcie_linkcap_update(pci, link_cap);

  return 0;
}

static void imx95_pcie_stop_link(struct dw_pcie_s *pci)
{
  /* Turn off PCIe LTSSM */

  imx95_pcie_ltssm_disable(pci->priv);
}

static int imx95_pcie_host_init(struct dw_pcie_rp_s *pp)
{
  struct dw_pcie_s *pci = to_dw_pcie_from_pp(pp);
  struct imx95_pcie_s *imx95_pcie_s = pci->priv;

  imx95_pcie_init_phy(imx95_pcie_s);
  imx95_pcie_configure_type(imx95_pcie_s);

  return 0;
}

static void imx95_pcie_host_exit(struct dw_pcie_rp_s *pp)
{
}

static uint64_t
imx95_pcie_cpu_addr_fixup(struct dw_pcie_s *pcie, uint64_t cpu_addr)
{
  return cpu_addr;
}

static void imx95_pcie_ep_init(struct dw_pcie_ep_s *ep)
{
  int bar;
  struct dw_pcie_s *pci = to_dw_pcie_from_ep(ep);

  for (bar = 0; bar <= 5; bar++)
    {
      dw_pcie_ep_reset_bar(pci, bar);
    }
}

static int
imx95_pcie_ep_raise_irq(struct dw_pcie_ep_s *ep, uint8_t funcno,
                      enum pci_epc_irq_type_e type, uint16_t interrupt_num)
{
  switch (type)
    {
      case PCI_EPC_IRQ_LEGACY:
        return dw_pcie_ep_raise_intx_irq(ep, funcno);
      case PCI_EPC_IRQ_MSI:
        return dw_pcie_ep_raise_msi_irq(ep, funcno, interrupt_num);
      case PCI_EPC_IRQ_MSIX:
        return dw_pcie_ep_raise_msix_irq(ep, funcno, interrupt_num);
      default:
        pcierr("UNKNOWN IRQ type\n");
        return -EINVAL;
    }

  return 0;
}

static const struct pci_epc_features_s *
imx95_pcie_ep_get_features(struct dw_pcie_ep_s *ep)
{
  return &g_imx95_pcie_epc_features;
}

static int imx95_add_pcie_ep(struct imx95_pcie_s *imx95_pcie)
{
  struct dw_pcie_s *pci = &imx95_pcie->pci;
  struct dw_pcie_rp_s *pp = &pci->pp;
  struct dw_pcie_ep_s *ep;
  int ret;

  imx95_pcie_host_init(pp);
  ep = &pci->ep;
  ep->ops = &g_pcie_ep_ops;
  ep->page_size = g_imx95_pcie_epc_features.align;

  ret = dw_pcie_ep_init(ep, "imx95_epc");
  if (ret)
    {
      pcierr("failed to initialize endpoint\n");
      return ret;
    }

  /* Start LTSSM. */

  imx95_pcie_ltssm_enable(imx95_pcie);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx95_pcie_init
 *
 * Description:
 *   Init imx95 pcie
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *  0 if success, a negated errno value if failed
 *
 ****************************************************************************/

int imx95_pcie_init(void)
{
  g_imx95_pcie.pci.priv = &g_imx95_pcie;
  nxmutex_init(&g_imx95_pcie.lock);
  return imx95_add_pcie_ep(&g_imx95_pcie);
}

/****************************************************************************
 * Name: imx95_pcie_uninit
 *
 * Description:
 *   Init imx95 pcie
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void imx95_pcie_uninit(void)
{
  imx95_pcie_stop_link(&g_imx95_pcie.pci);
}
