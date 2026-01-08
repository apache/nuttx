/****************************************************************************
 * boards/arm64/imx9/imx93-evk/src/imx9_ddr_config.c
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
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <chip.h>
#include <debug.h>
#include "arm64_internal.h"
#include "arm64_mmu.h"
#include "hardware/imx9_memorymap.h"

#include "ddr/hardware/imx9_ddr_training.h"
#include <arch/board/imx9_ddr_training.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* PHY Initialize Configuration */

static struct dram_fsp_msg ddr_dram_fsp_msg[] =
{
  {
    /* P0 3733 1D */

    .drate = 3733,
    .fw_type = FW_1D_IMAGE,
    .fsp_cfg = ddr_fsp0_cfg,
    .fsp_cfg_num = nitems(ddr_fsp0_cfg),
  },
  {
    /* P0 3733 2D */

    .drate = 3733,
    .fw_type = FW_2D_IMAGE,
    .fsp_cfg = ddr_fsp0_2d_cfg,
    .fsp_cfg_num = nitems(ddr_fsp0_2d_cfg),
  },
};

/* DDR timing config params */

struct dram_timing_info dram_timing_default_config =
{
  .ddrc_cfg = ddr_ddrc_cfg,
  .ddrc_cfg_num = nitems(ddr_ddrc_cfg),
  .ddrphy_cfg = ddr_ddrphy_cfg,
  .ddrphy_cfg_num = nitems(ddr_ddrphy_cfg),
  .fsp_msg = ddr_dram_fsp_msg,
  .fsp_msg_num = nitems(ddr_dram_fsp_msg),
  .ddrphy_pie = ddr_phy_pie,
  .ddrphy_pie_num = nitems(ddr_phy_pie),
  .fsp_cfg = ddr_dram_fsp_cfg,
  .fsp_cfg_num = nitems(ddr_dram_fsp_cfg),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct dram_timing_info *imx9_ddr_config(void)
{
  return &dram_timing_default_config;
}
