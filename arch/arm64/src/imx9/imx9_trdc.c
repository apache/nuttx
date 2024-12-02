/****************************************************************************
 * arch/arm64/src/imx9/imx9_trdc.c
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
#include <nuttx/clock.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm64_internal.h"
#include "imx9_trdc.h"
#include <arch/board/imx9_trdc_config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define mmio_read_32(c)                       getreg32(c)
#define mmio_write_32(c, v)                   putreg32(v, c)
#define mmio_clrbits_32(addr, clear)          modifyreg32(addr, clear, 0)
#define mmio_setbits_32(addr, set)            modifyreg32(addr, 0, set)
#define mmio_clrsetbits_32(addr, clear, set)  modifyreg32(addr, clear, set)

/* Bits 8:15 rdc id, bits 0:7 core id */

#define TRDC_AON            0x7402
#define TRDC_WAKEUP         0x7802
#define TRDC_MEDIA          0x8202
#define TRDC_NIX            0x8602

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

struct trdc_fused_module_info
{
  unsigned long trdc_base;
  uint8_t fsb_index;
  uint8_t fuse_bit;
  uint8_t mbc_id;
  uint8_t mem_id;
  uint8_t blk_id;
  uint8_t blk_num;
};

struct trdc_fuse_data
{
  uint8_t fsb_index;
  uint32_t value;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct trdc_mgr_info g_trdc_mgr_blks[] =
{
  { 0x44270000, 0, 0, 39, 40 }, /* TRDC_A */
  { 0x42460000, 0, 0, 70, 71 }, /* TRDC_W */
  { 0x42460000, 1, 0, 1,  2  }, /* TRDC_M */
  { 0x49010000, 0, 1, 1,  2  }, /* TRDC_N */
};

static const struct trdc_config_info g_trdc_cfg_info[] =
{
  {
    0x44270000,
    trdc_a_mbc_glbac, nitems(trdc_a_mbc_glbac),
    trdc_a_mbc, nitems(trdc_a_mbc),
    trdc_a_mrc_glbac, nitems(trdc_a_mrc_glbac),
    trdc_a_mrc, nitems(trdc_a_mrc)
  }, /* TRDC_A */
  {
    0x42460000,
    trdc_w_mbc_glbac, nitems(trdc_w_mbc_glbac),
    trdc_w_mbc, nitems(trdc_w_mbc),
    trdc_w_mrc_glbac, nitems(trdc_w_mrc_glbac),
    trdc_w_mrc, nitems(trdc_w_mrc)
  }, /* TRDC_W */
  {
    0x49010000,
    trdc_n_mbc_glbac, nitems(trdc_n_mbc_glbac),
    trdc_n_mbc, nitems(trdc_n_mbc),
    trdc_n_mrc_glbac, nitems(trdc_n_mrc_glbac),
    trdc_n_mrc, nitems(trdc_n_mrc)
  }, /* TRDC_N */
};

static const struct trdc_config_info g_trdc_boot_cfg_info[] =
{
  {
    0x49010000,
    trdc_boot_mbc_glbac, nitems(trdc_boot_mbc_glbac),
    trdc_boot_ocram_mbc, nitems(trdc_boot_ocram_mbc),
    trdc_n_mrc_glbac, nitems(trdc_n_mrc_glbac),
    trdc_n_mrc, nitems(trdc_n_mrc)
  },
};

static const struct trdc_fused_module_info g_fuse_info[] =
{
  { 0x49010000, 19, 13, 1, 2, 16, 1 }, /* NPU, NICMIX, MBC1, MEM2, slot 16 */
  { 0x49010000, 19, 13, 1, 3, 16, 1 }, /* NPU, NICMIX, MBC1, MEM3, slot 16 */
  { 0x44270000, 19, 30, 0, 0, 58, 1 }, /* FLEXCAN1, AONMIX, MBC0, MEM0, slot 58 */
  { 0x42460000, 19, 31, 0, 0, 91, 1 }, /* FLEXCAN2, WAKEUPMIX, MBC0, MEM0, slot 91 */
  { 0x49010000, 20, 3, 0, 3, 16, 16 }, /* USB1, NICMIX, MBC0, MEM3, slot 16-31 */
  { 0x49010000, 20, 4, 0, 3, 32, 16 }, /* USB2, NICMIX, MBC0, MEM3, slot 32-47 */
  { 0x42460000, 20, 5, 1, 0, 9, 1   }, /* ENET1 (FEC), WAKEUPMIX, MBC1, MEM0, slot 9 */
  { 0x42460000, 20, 6, 1, 0, 10, 1  }, /* ENET2 (eQOS), WAKEUPMIX, MBC1, MEM0, slot 10 */
  { 0x49010000, 20, 10, 0, 2, 34, 1 }, /* PXP, NICMIX, MBC0, MEM2, slot 34 */
  { 0x49010000, 20, 17, 0, 2, 32, 1 }, /* MIPI CSI NICMIX, MBC0, MEM2, slot 32 */
  { 0x49010000, 20, 19, 0, 2, 33, 1 }, /* MIPI DSI NICMIX, MBC0, MEM2, slot 33 */
  { 0x44270000, 21, 7, 0, 0, 83, 1  }, /* ADC1 AONMIX, MBC0, MEM0, slot 83 */
};

static struct trdc_fuse_data g_fuse_data[] =
{
  { 19, 0 },
  { 20, 0 },
  { 21, 0 },
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_init_mu
 *
 * Description:
 *   This function disable interrupts from AHAB
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imx9_init_mu(void)
{
  putreg32(0, ELE_MU_TCR);
  putreg32(0, ELE_MU_RCR);
}

/****************************************************************************
 * Name: imx9_ele_sendmsg
 *
 * Description:
 *   This function communicates with the Advanced High Assurance Boot (AHAB)
 *   image that should reside in the particular address. This function
 *   sends a message to AHAB.
 *
 * Input Parameters:
 *   msg         -  Message to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imx9_ele_sendmsg(struct ele_msg *msg)
{
  /* Check that ele is ready to receive */

  while (!((1) & getreg32(ELE_MU_TSR)));

  /* write header to slog 0 */

  putreg32(msg->header.data, ELE_MU_TR(0));

  /* write data */

  for (int i = 1; i < msg->header.size; i++)
    {
      int tx_channel;

      tx_channel = i % ELE_TR_NUM ;
      while (!((1 << tx_channel) & getreg32(ELE_MU_TSR)));

      /* Write data */

      putreg32(msg->data[i - 1], ELE_MU_TR(i));
    }
}

/****************************************************************************
 * Name: imx9_ele_receivemsg
 *
 * Description:
 *   This function communicates with the Advanced High Assurance Boot (AHAB)
 *   image that should reside in the particular address. This function
 *   receives message from AHAB.
 *
 * Input Parameters:
 *   msg         -  receive message buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imx9_ele_receivemsg(struct ele_msg *msg)
{
  /* Check if data ready */

  while (!((1) & getreg32(ELE_MU_RSR)));

  /* Read Header from slot 0 */

  msg->header.data = getreg32(ELE_MU_RR(0));

  for (int i = 1; i < msg->header.size; i++)
    {
      /* Check if empty */

      int rx_channel = (i) % ELE_RR_NUM;
      while (!((1 << rx_channel) & getreg32(ELE_MU_RSR)));

      /* Read data */

      msg->data[i - 1] = getreg32(ELE_MU_RR(i));
    }
}

/****************************************************************************
 * Name: imx9_release_rdc
 *
 * Description:
 *   Trusted Resource Domain Controller AHAB interface.  This function
 *   communicates with the Advanced High Assurance Boot (AHAB) image that
 *   should reside in the particular address. This releases particular
 *   resources.
 *
 * Input Parameters:
 *   xrdc    -  RDC index
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int imx9_release_rdc(uint32_t rdc_id)
{
  static struct ele_msg msg;

  msg.header.version = AHAB_VERSION;
  msg.header.tag = AHAB_CMD_TAG;
  msg.header.size = 2;
  msg.header.command = ELE_RELEASE_RDC_REQ;
  msg.data[0] = rdc_id;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if ((msg.data[0] & 0xff) == ELE_OK)
    {
      return 0;
    }

  return -EIO;
}

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

  /**************************************************************************
   * MBC0-3 Global 0, 0x7777 secure pri/user read/write/execute,
   * S400 has already set it. So select MBC0_MEMN_GLBAC0
   **************************************************************************/

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

      /* lock it up for TRDC mgr */

      trdc_mbc_set_control(mgr->trdc_base, mgr->mbc_id,
        7, GLBAC_LOCK_MASK | 0x6000);
    }
}

static uint32_t ele_read_common_fuse(uint32_t fuse_id)
{
  static struct ele_msg msg;
  uint32_t value = 0;

  msg.header.version = AHAB_VERSION;
  msg.header.tag = AHAB_CMD_TAG;
  msg.header.size = 2;
  msg.header.command = ELE_READ_FUSE_REQ;
  msg.data[0] = fuse_id;

  imx9_ele_sendmsg(&msg);
  imx9_ele_receivemsg(&msg);

  if ((msg.data[0] & 0xff) == ELE_OK)
    {
      value = msg.data[1];
    }

  VERBOSE("resp %x; %x; %x\n", msg.header.data, msg.data[0], value);
  return value;
}

static void trdc_fuse_init(void)
{
  uint32_t val;
  int i;

  val = mmio_read_32(BLK_CTRL_NS_ANOMIX_BASE + 0x28);
  for (i = 0; i < nitems(g_fuse_data); i++)
    {
      if (val & BIT(0)) /* OSCCA enabled */
        {
          g_fuse_data[i].value =
            ele_read_common_fuse(g_fuse_data[i].fsb_index);
        }
      else
        {
          g_fuse_data[i].value = mmio_read_32(FSB_BASE + FSB_SHADOW_OFF
            + (g_fuse_data[i].fsb_index << 2));
        }
    }
}

static uint32_t trdc_fuse_read(uint8_t word_index)
{
  uint32_t i;

  for (i = 0; i < nitems(g_fuse_data); i++)
    {
      if (g_fuse_data[i].fsb_index == word_index)
        return g_fuse_data[i].value;
    }
  return 0;
}

static void trdc_mgr_fused_slot_setup
  (const struct trdc_fused_module_info *fused_slot)
{
  uint32_t i;
  uint32_t val;
  uint32_t did;

  if (trdc_mbc_enabled(fused_slot->trdc_base))
    {
      trdc_mbc_set_control(fused_slot->trdc_base, fused_slot->mbc_id, 6, 0x0); /* No access permission */

      val = trdc_fuse_read(fused_slot->fsb_index);

      /* If the module is fused, set GLBAC6 for no access permission */

      if (val & BIT(fused_slot->fuse_bit))
        {
        for (i = 0; i < fused_slot->blk_num; i++)
          {
            for (did = 0; did < DID_NUM; did++)
              trdc_mbc_blk_config(fused_slot->trdc_base,
                fused_slot->mbc_id, did,
                fused_slot->mem_id,
                fused_slot->blk_id + i,
                true, 6);
          }
        }
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
                  /* Skip mgr and mc */

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

void imx9_trdc_config(void)
{
  int i;

  /* Assign all the GPIO pins to non-secure world by default */

  mmio_write_32(IMX9_GPIO2_BASE + 0x10, 0xffffffff);
  mmio_write_32(IMX9_GPIO2_BASE + 0x14, 0x3);
  mmio_write_32(IMX9_GPIO2_BASE + 0x18, 0xffffffff);
  mmio_write_32(IMX9_GPIO2_BASE + 0x1c, 0x3);

  mmio_write_32(IMX9_GPIO3_BASE + 0x10, 0xffffffff);
  mmio_write_32(IMX9_GPIO3_BASE + 0x14, 0x3);
  mmio_write_32(IMX9_GPIO3_BASE + 0x18, 0xffffffff);
  mmio_write_32(IMX9_GPIO3_BASE + 0x1c, 0x3);

  mmio_write_32(IMX9_GPIO4_BASE + 0x10, 0xffffffff);
  mmio_write_32(IMX9_GPIO4_BASE + 0x14, 0x3);
  mmio_write_32(IMX9_GPIO4_BASE + 0x18, 0xffffffff);
  mmio_write_32(IMX9_GPIO4_BASE + 0x1c, 0x3);

  mmio_write_32(IMX9_GPIO1_BASE + 0x10, 0xffffffff);
  mmio_write_32(IMX9_GPIO1_BASE + 0x14, 0x3);
  mmio_write_32(IMX9_GPIO1_BASE + 0x18, 0xffffffff);
  mmio_write_32(IMX9_GPIO1_BASE + 0x1c, 0x3);

  trdc_fuse_init();

  /* Set MTR to DID1 */

  trdc_mda_set_noncpu(0x44270000, 4, 0, false, 0x2, 0x2, 0x1, false);

  /* Configure the access permission for TRDC MGR and MC slots */

  for (i = 0; i < nitems(g_trdc_mgr_blks); i++)
    {
      trdc_mgr_mbc_setup(&g_trdc_mgr_blks[i]);
    }

  /* Configure TRDC user settings from config table */

  for (i = 0; i < nitems(g_trdc_cfg_info); i++)
    {
      trdc_setup(&g_trdc_cfg_info[i]);
    }

  /* Configure the access permission for fused slots */

  for (i = 0; i < nitems(g_fuse_info); i++)
    {
      trdc_mgr_fused_slot_setup(&g_fuse_info[i]);
    }

  /* Try to lock up TRDC MBC/MRC according
   * to user settings from config table
   */

  for (i = 0; i < nitems(g_trdc_cfg_info); i++)
    {
      trdc_try_lockup(&g_trdc_cfg_info[i]);
    }
}

/****************************************************************************
 * Name: imx9_trdc_init
 *
 * Description:
 *   Trusted Resource Domain Controller initialization function. This gives
 *   accesses to various resources in early boot phase.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success, a negated error value otherwise
 *
 ****************************************************************************/

int imx9_trdc_init(void)
{
  int ret;
  imx9_init_mu();

  ret = imx9_release_rdc(TRDC_AON);
  ret |= imx9_release_rdc(TRDC_MEDIA);
  ret |= imx9_release_rdc(TRDC_WAKEUP);
  ret |= imx9_release_rdc(TRDC_NIX);

  if (ret != 0)
    {
      return ret;
    }

  trdc_setup(&g_trdc_boot_cfg_info[0]);

  return 0;
}
