/****************************************************************************
 * arch/arm64/src/imx9/ddr/hardware/imx9_ddr_training.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_DDR_HARDWARE_IMX9_DDR_TRAINING_H
#define __ARCH_ARM64_SRC_IMX9_DDR_HARDWARE_IMX9_DDR_TRAINING_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMX9_IMEM_OFFSET_ADDR 0x50000
#define IMX9_DMEM_OFFSET_ADDR 0x54000

#define IMX9_DDR_PHY_BASE          0x4E100000

#define IMX9_DDRC_CS0_BNDS        (IMX9_DDR_CTRL_BASE + 0x0)
#define IMX9_DDRC_CS1_BNDS        (IMX9_DDR_CTRL_BASE + 0x8)
#define IMX9_DDRC_CS0_CFG         (IMX9_DDR_CTRL_BASE + 0x80)
#define IMX9_DDRC_CS1_CFG         (IMX9_DDR_CTRL_BASE + 0x84)
#define IMX9_DDRC_TIMING_CFG_3    (IMX9_DDR_CTRL_BASE + 0x100)
#define IMX9_DDRC_TIMING_CFG_0    (IMX9_DDR_CTRL_BASE + 0x104)
#define IMX9_DDRC_TIMING_CFG_1    (IMX9_DDR_CTRL_BASE + 0x108)
#define IMX9_DDRC_TIMING_CFG_2    (IMX9_DDR_CTRL_BASE + 0x10c)
#define IMX9_DDRC_SDRAM_CFG       (IMX9_DDR_CTRL_BASE + 0x110)
#define IMX9_DDRC_SDRAM_CFG2      (IMX9_DDR_CTRL_BASE + 0x114)
#define IMX9_DDRC_SDRAM_MD_CNTL   (IMX9_DDR_CTRL_BASE + 0x120)
#define IMX9_DDRC_SDRAM_INTERVAL  (IMX9_DDR_CTRL_BASE + 0x124)
#define IMX9_DDRC_TIMING_CFG_4    (IMX9_DDR_CTRL_BASE + 0x160)
#define IMX9_DDRC_TIMING_CFG_7    (IMX9_DDR_CTRL_BASE + 0x16c)
#define IMX9_DDRC_ZQ_CNTL         (IMX9_DDR_CTRL_BASE + 0x170)
#define IMX9_DDRC_DEBUG_19        (IMX9_DDR_CTRL_BASE + 0xF48)
#define IMX9_DDRC_TIMING_CFG_8    (IMX9_DDR_CTRL_BASE + 0x250)
#define IMX9_DDRC_TIMING_CFG_9    (IMX9_DDR_CTRL_BASE + 0x254)
#define IMX9_DDRC_TIMING_CFG_10   (IMX9_DDR_CTRL_BASE + 0x258)
#define IMX9_DDRC_TIMING_CFG_11   (IMX9_DDR_CTRL_BASE + 0x25c)
#define IMX9_DDRC_SDRAM_CFG_3     (IMX9_DDR_CTRL_BASE + 0x260)
#define IMX9_DDRC_SDRAM_CFG_4     (IMX9_DDR_CTRL_BASE + 0x264)
#define IMX9_DDRC_SDRAM_MD_CNTL_2 (IMX9_DDR_CTRL_BASE + 0x270)
#define IMX9_DDRC_SDRAM_MPR4      (IMX9_DDR_CTRL_BASE + 0x28C)
#define IMX9_DDRC_SDRAM_MPR5      (IMX9_DDR_CTRL_BASE + 0x290)
#define IMX9_DDRC_TIMING_CFG_12   (IMX9_DDR_CTRL_BASE + 0x300)
#define IMX9_DDRC_TIMING_CFG_13   (IMX9_DDR_CTRL_BASE + 0x304)
#define IMX9_DDRC_TIMING_CFG_14   (IMX9_DDR_CTRL_BASE + 0x308)
#define IMX9_DDRC_TX_CFG_1        (IMX9_DDR_CTRL_BASE + 0x800)
#define IMX9_DDRC_UNKNOWN_1       (IMX9_DDR_CTRL_BASE + 0x804)
#define IMX9_DDRDSR_2             (IMX9_DDR_CTRL_BASE + 0xB24)
#define IMX9_DDRC_UNKNOWN_2       (IMX9_DDR_CTRL_BASE + 0xf04)
#define IMX9_DDRC_ERR_EN          (IMX9_DDR_CTRL_BASE + 0x1000)
#define IMX9_DDRC_ECC_REG_0       (IMX9_DDR_CTRL_BASE + 0x1240)
#define IMX9_DDRC_ECC_REG_1       (IMX9_DDR_CTRL_BASE + 0x1244)
#define IMX9_DDRC_ECC_REG_2       (IMX9_DDR_CTRL_BASE + 0x1248)
#define IMX9_DDRC_ECC_REG_3       (IMX9_DDR_CTRL_BASE + 0x124c)
#define IMX9_DDRC_ECC_REG_4       (IMX9_DDR_CTRL_BASE + 0x1250)
#define IMX9_DDRC_ECC_REG_5       (IMX9_DDR_CTRL_BASE + 0x1254)
#define IMX9_DDRC_ECC_REG_6       (IMX9_DDR_CTRL_BASE + 0x1258)
#define IMX9_DDRC_ECC_REG_7       (IMX9_DDR_CTRL_BASE + 0x125c)

#define IMX9_DDRC_MEM_EN          BIT(31)
#define IMX9_DDRC_CFG2_D_INIT     BIT(4)

#define IMX9_REG_SRC_DPHY_SW_CTRL              (IMX9_SRC_DPHY_SLICE_BASE + 0x20)
#define IMX9_REG_SRC_DPHY_SINGLE_RESET_SW_CTRL (IMX9_SRC_DPHY_SLICE_BASE + 0x24)

enum msg_response
{
  TRAIN_SUCCESS = 0x7,
  TRAIN_STREAM_START = 0x8,
  TRAIN_FAIL = 0xff,
};

/* user data type */

enum fw_type
{
  FW_1D_IMAGE,
  FW_2D_IMAGE,
};

struct dram_cfg_param
{
  unsigned int reg;
  unsigned int val;
};

struct dram_fsp_cfg
{
  struct dram_cfg_param ddrc_cfg[20];
  struct dram_cfg_param mr_cfg[10];
  unsigned int bypass;
};

struct dram_fsp_msg
{
  unsigned int drate;
  enum fw_type fw_type;
  struct dram_cfg_param *fsp_cfg;
  unsigned int fsp_cfg_num;
};

struct dram_timing_info
{
  /* umctl2 config */

  struct dram_cfg_param *ddrc_cfg;
  unsigned int ddrc_cfg_num;

  /* fsp config */

  struct dram_fsp_cfg *fsp_cfg;
  unsigned int fsp_cfg_num;

  /* ddrphy config */

  struct dram_cfg_param *ddrphy_cfg;
  unsigned int ddrphy_cfg_num;

  /* ddr fsp train info */

  struct dram_fsp_msg *fsp_msg;
  unsigned int fsp_msg_num;

  /* ddr phy trained CSR */

  struct dram_cfg_param *ddrphy_trained_csr;
  unsigned int ddrphy_trained_csr_num;

  /* ddr phy PIE */

  struct dram_cfg_param *ddrphy_pie;
  unsigned int ddrphy_pie_num;
};

#endif
