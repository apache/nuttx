/****************************************************************************
 * arch/arm64/src/zynq-mpsoc/hardware/zynq_pll.h
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

#ifndef __ARCH_ARM64_SRC_ZYNQ_MPSOC_HARDWARE_ZYNQ_PLL_H
#define __ARCH_ARM64_SRC_ZYNQ_MPSOC_HARDWARE_ZYNQ_PLL_H

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* NOTE: The PLL input (IN) clocks are not available in clock tree */

enum mpsoc_clk
{
  iopll, rpll,
  apll, dpll, vpll,
  iopll_to_fpd, rpll_to_fpd, apll_to_lpd, dpll_to_lpd, vpll_to_lpd,
  acpu, acpu_half,
  dbg_fpd, dbg_lpd, dbg_trace, dbg_tstmp,
  dp_video_ref, dp_audio_ref,
  dp_stc_ref, gdma_ref, dpdma_ref,
  ddr_ref, sata_ref, pcie_ref,
  gpu_ref, gpu_pp0_ref, gpu_pp1_ref,
  topsw_main, topsw_lsbus,
  gtgref0_ref,
  lpd_switch, lpd_lsbus,
  usb0_bus_ref, usb1_bus_ref, usb3_dual_ref, usb0, usb1,
  cpu_r5, cpu_r5_core,
  csu_spb, csu_pll, pcap,
  iou_switch,
  gem_tsu_ref, gem_tsu,
  gem0_ref, gem1_ref, gem2_ref, gem3_ref,
  gem0_rx, gem1_rx, gem2_rx, gem3_rx,
  qspi_ref,
  sdio0_ref, sdio1_ref,
  uart0_ref, uart1_ref,
  spi0_ref, spi1_ref,
  nand_ref,
  i2c0_ref, i2c1_ref, can0_ref, can1_ref, can0, can1,
  dll_ref,
  adma_ref,
  timestamp_ref,
  ams_ref,
  pl0, pl1, pl2, pl3,
  wdt,
  ttc0_ref, ttc1_ref, ttc2_ref, ttc3_ref,
  clk_max,
};

/****************************************************************************
 * Function: mpsoc_clk_rate_get
 *
 * Description:
 *  Get controller running frequency
 *
 * Input Parameters:
 *   clk - peripheral clock ID
 *
 * Returned Value:
 *   Running frequency.
 *
 ****************************************************************************/

uintptr_t mpsoc_clk_rate_get(enum mpsoc_clk clk);

/****************************************************************************
 * Function: mpsoc_clk_rate_set
 *
 * Description:
 *  Set running frequency
 *
 * Input Parameters:
 *   clk - peripheral clock ID
 *   rate - clock freq
 *
 * Returned Value:
 *   peripheral running frequency.
 *
 ****************************************************************************/

uintptr_t mpsoc_clk_rate_set(enum mpsoc_clk clk, uintptr_t rate);

#endif /* __ARCH_ARM54_SRC_ZYNQ_MPSOC_HARDWARE_ZYNQ_PLL_H */
