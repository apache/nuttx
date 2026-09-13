/****************************************************************************
 * arch/arm/src/imxrt/imxrt118x_trdc.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/bits.h>
#include <nuttx/clock.h>
#include <sys/param.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "imxrt118x_ele.h"
#include "imxrt118x_trdc.h"
#include <arch/board/imxrt118x_trdc_config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define mmio_read_32(c)                       getreg32(c)
#define mmio_write_32(c, v)                   putreg32(v, c)
#define mmio_clrbits_32(addr, clear)          modifyreg32(addr, clear, 0)
#define mmio_setbits_32(addr, set)            modifyreg32(addr, 0, set)
#define mmio_clrsetbits_32(addr, clear, set)  modifyreg32(addr, clear, set)

/* Packed ELE TRDC release IDs for the CM33. */

#define TRDC_AON            0x7401
#define TRDC_WAKEUP         0x7801
#define TRDC_MEDIA          0x8201

#define VERBOSE _none

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mbc_mem_dom
{
  uint32_t mem_glbcfg[4];
  uint32_t nse_blk_index;
  uint32_t nse_blk_set;
  uint32_t nse_blk_clr;
  uint32_t nsr_blk_clr_all;
  uint32_t memn_glbac[8];

  /* The upper only existed in the beginning of each MBC */

  uint32_t mem0_blk_cfg_w[64];
  uint32_t mem0_blk_nse_w[16];
  uint32_t mem1_blk_cfg_w[8];
  uint32_t mem1_blk_nse_w[2];
  uint32_t mem2_blk_cfg_w[8];
  uint32_t mem2_blk_nse_w[2];
  uint32_t mem3_blk_cfg_w[8];
  uint32_t mem3_blk_nse_w[2]; /* 0x1F0, 0x1F4 */
  uint32_t reserved[2];
};

struct mrc_rgn_dom
{
  uint32_t mrc_glbcfg[4];
  uint32_t nse_rgn_indirect;
  uint32_t nse_rgn_set;
  uint32_t nse_rgn_clr;
  uint32_t nse_rgn_clr_all;
  uint32_t memn_glbac[8];

  /* The upper only existed in the beginning of each MRC */

  uint32_t rgn_desc_words[16][2]; /* 16 regions at max, 2 words per region */
  uint32_t rgn_nse;
  uint32_t reserved2[15];
};

struct mda_inst
{
  uint32_t mda_w[8];
};

struct trdc_mgr
{
  uint32_t trdc_cr;
  uint32_t res0[59];
  uint32_t trdc_hwcfg0;
  uint32_t trdc_hwcfg1;
  uint32_t res1[450];
  struct mda_inst mda[8];
  uint32_t res2[15808];
};

struct trdc_mbc
{
  struct mbc_mem_dom mem_dom[DID_NUM];
};

struct trdc_mrc
{
  struct mrc_rgn_dom mrc_dom[DID_NUM];
};

struct trdc_mgr_info
{
  unsigned long trdc_base;
  uint8_t mbc_id;
  uint8_t mbc_mem_id;
  uint8_t blk_mgr;
  uint8_t blk_mc;
};

struct trdc_config_info
{
  unsigned long trdc_base;
  struct trdc_glbac_config *mbc_glbac;
  uint32_t num_mbc_glbac;
  struct trdc_mbc_config *mbc_cfg;
  uint32_t num_mbc_cfg;
  struct trdc_glbac_config *mrc_glbac;
  uint32_t num_mrc_glbac;
  struct trdc_mrc_config *mrc_cfg;
  uint32_t num_mrc_cfg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Protect each TRDC's own manager/controller register blocks. */

static const struct trdc_mgr_info g_trdc_mgr_blks[] =
{
  { IMXRT_TRDC1_BASE, 0, 0, 39, 40 }, /* TRDC1 (AON)    */
  { IMXRT_TRDC2_BASE, 0, 0, 70, 71 }, /* TRDC2 (WAKEUP) */
  { IMXRT_TRDC2_BASE, 1, 0, 1,  2  }, /* TRDC3 via TRDC2 MBC1 */
};

static const struct trdc_config_info g_trdc_cfg_info[] =
{
  {
    IMXRT_TRDC1_BASE,
    trdc_a_mbc_glbac, nitems(trdc_a_mbc_glbac),
    trdc_a_mbc, nitems(trdc_a_mbc),
    trdc_a_mrc_glbac, nitems(trdc_a_mrc_glbac),
    trdc_a_mrc, nitems(trdc_a_mrc)
  }, /* TRDC1 (AON) */
  {
    IMXRT_TRDC2_BASE,
    trdc_w_mbc_glbac, nitems(trdc_w_mbc_glbac),
    trdc_w_mbc, nitems(trdc_w_mbc),
    trdc_w_mrc_glbac, nitems(trdc_w_mrc_glbac),
    trdc_w_mrc, nitems(trdc_w_mrc)
  }, /* TRDC2 (WAKEUP / MEGA) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int trdc_mda_set_noncpu(unsigned long trdc_reg, uint32_t mda_inst,
     uint32_t mda_reg, bool did_bypass, uint8_t sa,
     uint8_t pa, uint8_t did, bool lock)
{
  struct trdc_mgr *trdc_base = (struct trdc_mgr *)trdc_reg;
  uint32_t *mda_w = &trdc_base->mda[mda_inst].mda_w[mda_reg];
  uint32_t val = mmio_read_32((uintptr_t)mda_w);

  if (!(val & BIT(29))) /* cpu */
    return -EINVAL;

  val = BIT(31) | ((sa & 0x3) << 6) | ((pa & 0x3) << 4) | (did & 0xf);
  if (did_bypass)
    val |= BIT(8);

  mmio_write_32((uintptr_t)mda_w, val);

  if (lock)
    mmio_write_32((uintptr_t)mda_w, val | BIT(30));

  return 0;
}

static int trdc_mda_set_cpu(unsigned long trdc_reg, uint32_t mda_inst,
     uint32_t mda_reg, uint8_t did_sel, uint8_t sa, uint8_t did, bool lock)
{
  struct trdc_mgr *trdc_base = (struct trdc_mgr *)trdc_reg;
  uint32_t *mda_w = &trdc_base->mda[mda_inst].mda_w[mda_reg];
  uint32_t val = mmio_read_32((uintptr_t)mda_w);

  if (val & BIT(29)) /* non-cpu */
    return -EINVAL;

  val = BIT(31) | ((sa & 0x3) << 14) | ((did_sel & 0x3) << 4) |
        (did & 0xf);

  mmio_write_32((uintptr_t)mda_w, val);

  if (lock)
    mmio_write_32((uintptr_t)mda_w, val | BIT(30));

  return 0;
}

static unsigned long trdc_get_mbc_base(unsigned long trdc_reg,
  uint32_t mbc_x)
{
  struct trdc_mgr *trdc_base = (struct trdc_mgr *)trdc_reg;
  uint32_t mbc_num = MBC_NUM(trdc_base->trdc_hwcfg0);

  if (mbc_x >= mbc_num)
    return 0;

  return trdc_reg + 0x10000 + 0x2000 * mbc_x;
}

static unsigned long trdc_get_mrc_base(unsigned long trdc_reg,
  uint32_t mrc_x)
{
  struct trdc_mgr *trdc_base = (struct trdc_mgr *)trdc_reg;
  uint32_t mbc_num = MBC_NUM(trdc_base->trdc_hwcfg0);
  uint32_t mrc_num = MRC_NUM(trdc_base->trdc_hwcfg0);

  if (mrc_x >= mrc_num)
    return 0;

  return trdc_reg + 0x10000 + 0x2000 * mbc_num + 0x1000 * mrc_x;
}

static uint32_t trdc_mbc_blk_num(unsigned long trdc_reg, uint32_t mbc_x,
  uint32_t mem_x)
{
  struct trdc_mbc *mbc_base =
    (struct trdc_mbc *)trdc_get_mbc_base(trdc_reg, mbc_x);
  struct mbc_mem_dom *mbc_dom;
  uint32_t glbcfg;

  if (mbc_base == 0)
    return 0;

  /* only first dom has the glbcfg */

  mbc_dom = &mbc_base->mem_dom[0];
  glbcfg = mmio_read_32((uintptr_t)&mbc_dom->mem_glbcfg[mem_x]);

  return MBC_BLK_NUM(glbcfg);
}

static int trdc_mbc_set_control(unsigned long trdc_reg, uint32_t mbc_x,
  uint32_t glbac_id, uint32_t glbac_val)
{
  struct trdc_mbc *mbc_base =
    (struct trdc_mbc *)trdc_get_mbc_base(trdc_reg, mbc_x);
  struct mbc_mem_dom *mbc_dom;
  uint32_t i;

  if (mbc_base == 0 || glbac_id >= 8)
    return -EINVAL;

  /* Skip glbac7 used for TRDC MGR protection */

  if (glbac_id == 7 && glbac_val != 0x6000)
    {
      for (i = 0; i < nitems(g_trdc_mgr_blks); i++)
        {
          if (trdc_reg == g_trdc_mgr_blks[i].trdc_base
            && mbc_x == g_trdc_mgr_blks[i].mbc_id)
            {
              return -EPERM;
            }
        }
    }

  /* Skip glbac6 used for fused module */

  if (glbac_id == 6 && glbac_val != 0)
    return -EPERM;

  /* only first dom has the glbac */

  mbc_dom = &mbc_base->mem_dom[0];

  mmio_write_32((uintptr_t)&mbc_dom->memn_glbac[glbac_id], glbac_val);

  return 0;
}

static int trdc_mbc_blk_config(unsigned long trdc_reg, uint32_t mbc_x,
  uint32_t dom_x, uint32_t mem_x, uint32_t blk_x,
  bool sec_access, uint32_t glbac_id)
{
  struct trdc_mbc *mbc_base =
    (struct trdc_mbc *)trdc_get_mbc_base(trdc_reg, mbc_x);
  struct mbc_mem_dom *mbc_dom;
  uint32_t *cfg_w;
  uint32_t index;
  uint32_t offset;
  uint32_t val;

  if (mbc_base == 0 || glbac_id >= 8)
    return -EINVAL;

  mbc_dom = &mbc_base->mem_dom[dom_x];

  switch (mem_x)
    {
      case 0:
        cfg_w = &mbc_dom->mem0_blk_cfg_w[blk_x / 8];
        break;
      case 1:
        cfg_w = &mbc_dom->mem1_blk_cfg_w[blk_x / 8];
        break;
      case 2:
        cfg_w = &mbc_dom->mem2_blk_cfg_w[blk_x / 8];
        break;
      case 3:
        cfg_w = &mbc_dom->mem3_blk_cfg_w[blk_x / 8];
        break;
      default:
        return -1;
    };

  index = blk_x % 8;
  offset = index * 4;

  val = mmio_read_32((uintptr_t)cfg_w);
  val &= ~(0xfu << offset);

  if (sec_access)
    {
      val |= ((0x0 | (glbac_id & 0x7)) << offset);
      mmio_write_32((uintptr_t)cfg_w, val);
    }
  else
    {
      val |= ((0x8 | (glbac_id & 0x7)) << offset); /* nse bit set */
      mmio_write_32((uintptr_t)cfg_w, val);
    }

  return 0;
}

static int trdc_mrc_set_control(unsigned long trdc_reg, uint32_t mrc_x,
  uint32_t glbac_id, uint32_t glbac_val)
{
  struct trdc_mrc *mrc_base =
    (struct trdc_mrc *)trdc_get_mrc_base(trdc_reg, mrc_x);
  struct mrc_rgn_dom *mrc_dom;

  if (mrc_base == 0 || glbac_id >= 8)
    return -EINVAL;

  /* only first dom has the glbac */

  mrc_dom = &mrc_base->mrc_dom[0];

  mmio_write_32((uintptr_t)&mrc_dom->memn_glbac[glbac_id], glbac_val);

  return 0;
}

static int trdc_mrc_rgn_config(unsigned long trdc_reg,
  uint32_t mrc_x, uint32_t dom_x, uint32_t rgn_id,
  uint32_t addr_start, uint32_t addr_size,
  bool sec_access, uint32_t glbac_id)
{
  struct trdc_mrc *mrc_base =
    (struct trdc_mrc *)trdc_get_mrc_base(trdc_reg, mrc_x);
  struct mrc_rgn_dom *mrc_dom;
  uint32_t *desc_w;
  uint32_t addr_end;

  if (mrc_base == 0 || glbac_id >= 8 || rgn_id >= 16)
    return -EINVAL;

  mrc_dom = &mrc_base->mrc_dom[dom_x];

  addr_end = addr_start + addr_size - 1;
  addr_start &= ~0x3fff;
  addr_end &= ~0x3fff;

  desc_w = &mrc_dom->rgn_desc_words[rgn_id][0];

  if (sec_access)
    {
      mmio_write_32((uintptr_t)desc_w, addr_start | (glbac_id & 0x7));
      mmio_write_32((uintptr_t)(desc_w + 1), addr_end | 0x1);
    }
  else
    {
      mmio_write_32((uintptr_t)desc_w, addr_start | (glbac_id & 0x7));
      mmio_write_32((uintptr_t)(desc_w + 1), (addr_end | 0x1 | 0x10));
    }
  return 0;
}

static bool trdc_mrc_enabled(unsigned long trdc_base)
{
  return (!!(mmio_read_32(trdc_base) & 0x8000));
}

static bool trdc_mbc_enabled(unsigned long trdc_base)
{
  return (!!(mmio_read_32(trdc_base) & 0x4000));
}

static bool is_trdc_mgr_slot(unsigned long trdc_base,
  uint8_t mbc_id, uint8_t mem_id, uint16_t blk_id)
{
  uint32_t i;

  for (i = 0; i < nitems(g_trdc_mgr_blks); i++)
    {
      if (g_trdc_mgr_blks[i].trdc_base == trdc_base)
        {
          if (mbc_id == g_trdc_mgr_blks[i].mbc_id
            && mem_id == g_trdc_mgr_blks[i].mbc_mem_id
            && (blk_id == g_trdc_mgr_blks[i].blk_mgr
            || blk_id == g_trdc_mgr_blks[i].blk_mc))
            return true;
        }
    }
  return false;
}

static void trdc_mgr_mbc_setup(const struct trdc_mgr_info *mgr)
{
  uint32_t i;

  if (trdc_mbc_enabled(mgr->trdc_base))
    {
      trdc_mbc_set_control(mgr->trdc_base, mgr->mbc_id, 7, 0x6000); /* ONLY secure privilege can access */
      for (i = 0; i < 16; i++)
        {
          trdc_mbc_blk_config(mgr->trdc_base, mgr->mbc_id,
            i, mgr->mbc_mem_id, mgr->blk_mgr, true, 7);
          trdc_mbc_blk_config(mgr->trdc_base, mgr->mbc_id,
            i, mgr->mbc_mem_id, mgr->blk_mc, true, 7);
        }

      /* Lock the TRDC manager policy. */

      trdc_mbc_set_control(mgr->trdc_base, mgr->mbc_id,
        7, GLBAC_LOCK_MASK | 0x6000);
    }
}

static void trdc_setup(const struct trdc_config_info *cfg)
{
  int i;
  int j;
  uint32_t num;
  bool is_mgr;

  if (trdc_mrc_enabled(cfg->trdc_base))
    {
      for (i = 0; i < cfg->num_mrc_glbac; i++)
        {
          trdc_mrc_set_control(cfg->trdc_base, cfg->mrc_glbac[i].mbc_mrc_id,
          cfg->mrc_glbac[i].glbac_id,
          cfg->mrc_glbac[i].glbac_val & GLBAC_SETTING_MASK);
        }

      for (i = 0; i < cfg->num_mrc_cfg; i++)
        {
          trdc_mrc_rgn_config(cfg->trdc_base,
            cfg->mrc_cfg[i].mrc_id, cfg->mrc_cfg[i].dom_id,
            cfg->mrc_cfg[i].region_id, cfg->mrc_cfg[i].region_start,
            cfg->mrc_cfg[i].region_size, cfg->mrc_cfg[i].secure,
            cfg->mrc_cfg[i].glbac_id);
        }
    }

  if (trdc_mbc_enabled(cfg->trdc_base))
    {
      for (i = 0; i < cfg->num_mbc_glbac; i++)
        {
          trdc_mbc_set_control(cfg->trdc_base, cfg->mbc_glbac[i].mbc_mrc_id,
            cfg->mbc_glbac[i].glbac_id,
            cfg->mbc_glbac[i].glbac_val & GLBAC_SETTING_MASK);
        }

      for (i = 0; i < cfg->num_mbc_cfg; i++)
        {
          if (cfg->mbc_cfg[i].blk_id == MBC_BLK_ALL)
            {
              num = trdc_mbc_blk_num(cfg->trdc_base, cfg->mbc_cfg[i].mbc_id,
                cfg->mbc_cfg[i].mem_id);

              for (j = 0; j < num; j++)
                {
                  /* Skip the protected TRDC blocks. */

                  is_mgr = is_trdc_mgr_slot(cfg->trdc_base,
                    cfg->mbc_cfg[i].mbc_id, cfg->mbc_cfg[i].mem_id, j);
                  if (is_mgr)
                    continue;

                  trdc_mbc_blk_config(cfg->trdc_base, cfg->mbc_cfg[i].mbc_id,
                    cfg->mbc_cfg[i].dom_id,
                  cfg->mbc_cfg[i].mem_id, j, cfg->mbc_cfg[i].secure,
                  cfg->mbc_cfg[i].glbac_id);
                }
            }
          else
            {
              trdc_mbc_blk_config(cfg->trdc_base, cfg->mbc_cfg[i].mbc_id,
                cfg->mbc_cfg[i].dom_id, cfg->mbc_cfg[i].mem_id,
                cfg->mbc_cfg[i].blk_id, cfg->mbc_cfg[i].secure,
                cfg->mbc_cfg[i].glbac_id);
            }
        }
    }
}

static void trdc_try_lockup(const struct trdc_config_info *cfg)
{
  uint32_t i;

  if (trdc_mrc_enabled(cfg->trdc_base))
    {
      for (i = 0; i < cfg->num_mrc_glbac; i++)
        {
          trdc_mrc_set_control(cfg->trdc_base,
            cfg->mrc_glbac[i].mbc_mrc_id,
            cfg->mrc_glbac[i].glbac_id,
            cfg->mrc_glbac[i].glbac_val
              & (GLBAC_SETTING_MASK | GLBAC_LOCK_MASK));
        }
    }

  if (trdc_mbc_enabled(cfg->trdc_base))
    {
      for (i = 0; i < cfg->num_mbc_glbac; i++)
        {
          trdc_mbc_set_control(cfg->trdc_base, cfg->mbc_glbac[i].mbc_mrc_id,
          cfg->mbc_glbac[i].glbac_id,
          cfg->mbc_glbac[i].glbac_val
            & (GLBAC_SETTING_MASK | GLBAC_LOCK_MASK));
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void imxrt118x_trdc_config(void)
{
  int i;

  /* Assign GPIO2..GPIO6 to non-secure access. */

  mmio_write_32(IMXRT_GPIO2_BASE + 0x10, 0xffffffff);
  mmio_write_32(IMXRT_GPIO2_BASE + 0x14, 0x3);
  mmio_write_32(IMXRT_GPIO2_BASE + 0x18, 0xffffffff);
  mmio_write_32(IMXRT_GPIO2_BASE + 0x1c, 0x3);

  mmio_write_32(IMXRT_GPIO3_BASE + 0x10, 0xffffffff);
  mmio_write_32(IMXRT_GPIO3_BASE + 0x14, 0x3);
  mmio_write_32(IMXRT_GPIO3_BASE + 0x18, 0xffffffff);
  mmio_write_32(IMXRT_GPIO3_BASE + 0x1c, 0x3);

  mmio_write_32(IMXRT_GPIO4_BASE + 0x10, 0xffffffff);
  mmio_write_32(IMXRT_GPIO4_BASE + 0x14, 0x3);
  mmio_write_32(IMXRT_GPIO4_BASE + 0x18, 0xffffffff);
  mmio_write_32(IMXRT_GPIO4_BASE + 0x1c, 0x3);

  mmio_write_32(IMXRT_GPIO5_BASE + 0x10, 0xffffffff);
  mmio_write_32(IMXRT_GPIO5_BASE + 0x14, 0x3);
  mmio_write_32(IMXRT_GPIO5_BASE + 0x18, 0xffffffff);
  mmio_write_32(IMXRT_GPIO5_BASE + 0x1c, 0x3);

  mmio_write_32(IMXRT_GPIO6_BASE + 0x10, 0xffffffff);
  mmio_write_32(IMXRT_GPIO6_BASE + 0x14, 0x3);
  mmio_write_32(IMXRT_GPIO6_BASE + 0x18, 0xffffffff);
  mmio_write_32(IMXRT_GPIO6_BASE + 0x1c, 0x3);

  /* Apply common DAC setup. */

  trdc_mda_set_cpu(IMXRT_TRDC1_BASE, 1, 0, TRDC_MDA_DID_FROM_INPUT,
    TRDC_MDA_FORCE_SECURE, DID_CM33, false);      /* MDAC_A1 CM33 */
  trdc_mda_set_noncpu(IMXRT_TRDC1_BASE, 2, 0, true,
    TRDC_MDA_FORCE_SECURE, TRDC_MDA_FORCE_PRIVILEGE, DID_EDMA3, false);

  trdc_mda_set_cpu(IMXRT_TRDC2_BASE, 0, 0, TRDC_MDA_DID_FROM_INPUT,
    TRDC_MDA_FORCE_SECURE, DID_CM7, false);       /* MDAC_W0 CM7 AHBP */
  trdc_mda_set_cpu(IMXRT_TRDC2_BASE, 1, 0, TRDC_MDA_DID_FROM_INPUT,
    TRDC_MDA_FORCE_SECURE, DID_CM7, false);       /* MDAC_W1 CM7 AXI */
  trdc_mda_set_noncpu(IMXRT_TRDC2_BASE, 2, 0, true,
    TRDC_MDA_FORCE_SECURE, TRDC_MDA_FORCE_PRIVILEGE, DID_DAP, false);
  trdc_mda_set_noncpu(IMXRT_TRDC2_BASE, 3, 0, true,
    TRDC_MDA_FORCE_SECURE, TRDC_MDA_FORCE_PRIVILEGE, DID_CORESIGHT, false);
  trdc_mda_set_noncpu(IMXRT_TRDC2_BASE, 4, 0, true,
    TRDC_MDA_FORCE_SECURE, TRDC_MDA_FORCE_PRIVILEGE, DID_EDMA4, false);
  trdc_mda_set_cpu(IMXRT_TRDC2_BASE, 5, 0, TRDC_MDA_DID_FROM_INPUT,
    TRDC_MDA_FORCE_SECURE, DID_NETC, false);      /* MDAC_W5 NETC */

  trdc_mda_set_noncpu(IMXRT_TRDC3_BASE, 0, 0, true,
    TRDC_MDA_FORCE_SECURE, TRDC_MDA_FORCE_PRIVILEGE, DID_USDHC1, false);
  trdc_mda_set_noncpu(IMXRT_TRDC3_BASE, 1, 0, true,
    TRDC_MDA_FORCE_SECURE, TRDC_MDA_FORCE_PRIVILEGE, DID_USDHC2, false);
  trdc_mda_set_noncpu(IMXRT_TRDC3_BASE, 3, 0, true,
    TRDC_MDA_FORCE_SECURE, TRDC_MDA_FORCE_PRIVILEGE, DID_USB, false);
  trdc_mda_set_noncpu(IMXRT_TRDC3_BASE, 4, 0, true,
    TRDC_MDA_FORCE_SECURE, TRDC_MDA_FORCE_PRIVILEGE, DID_FLEXSPI_FLR, false);

  /* Protect TRDC manager slots. */

  for (i = 0; i < nitems(g_trdc_mgr_blks); i++)
    {
      trdc_mgr_mbc_setup(&g_trdc_mgr_blks[i]);
    }

  /* Apply board TRDC settings. */

  for (i = 0; i < nitems(g_trdc_cfg_info); i++)
    {
      trdc_setup(&g_trdc_cfg_info[i]);
    }

  /* Lock board TRDC settings where requested. */

  for (i = 0; i < nitems(g_trdc_cfg_info); i++)
    {
      trdc_try_lockup(&g_trdc_cfg_info[i]);
    }
}

/****************************************************************************
 * Name: imxrt118x_trdc_init
 *
 * Description:
 *   Request ownership of the TRDCs.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success, a negated error value otherwise
 *
 ****************************************************************************/

int imxrt118x_trdc_init(void)
{
  int ret;

  ret = imxrt118x_ele_release_rdc(TRDC_AON);
  ret |= imxrt118x_ele_release_rdc(TRDC_MEDIA);
  ret |= imxrt118x_ele_release_rdc(TRDC_WAKEUP);

  if (ret != 0)
    {
      return ret;
    }

  return 0;
}
