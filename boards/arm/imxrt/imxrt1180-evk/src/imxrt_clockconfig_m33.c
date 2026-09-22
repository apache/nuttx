/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt_clockconfig_m33.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "imxrt_clockconfig.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clock_configuration_s g_initial_clkconfig =
{
  .ccm =
  {
    .m7_clk_root            = CCM_CLOCK_ROOT_IGNORE,
    .m33_clk_root =
    {
      .action = CCM_CLOCK_ROOT_CONFIGURE,
      .div    = 3,
      .mux    = ARM_PLL_OUT,
    },
    .edgelock_clk_root =
    {
      .action = CCM_CLOCK_ROOT_CONFIGURE,
      .div    = 2,
      .mux    = OSC_RC_400M,
    },
    .bus_aon_clk_root =
    {
      .action = CCM_CLOCK_ROOT_CONFIGURE,
      .div    = 4,
      .mux    = SYS_PLL2_OUT,
    },
    .bus_wakeup_clk_root =
    {
      .action = CCM_CLOCK_ROOT_CONFIGURE,
      .div    = 4,
      .mux    = SYS_PLL2_OUT,
    },
    .wakeup_axi_clk_root =
    {
      .action = CCM_CLOCK_ROOT_CONFIGURE,
      .div    = 2,
      .mux    = SYS_PLL3_OUT,
    },
    .swo_trace_clk_root =
    {
      .action = CCM_CLOCK_ROOT_CONFIGURE,
      .div    = 3,
      .mux    = SYS_PLL3_DIV2,
    },
    .m33_systick_clk_root =
    {
      .action = CCM_CLOCK_ROOT_CONFIGURE,
      .div    = 240,
      .mux    = OSC_24M,
    },
    .m7_systick_clk_root    = CCM_CLOCK_ROOT_IGNORE,
    .flexio1_clk_root       = CCM_CLOCK_ROOT_DISABLE,
    .flexio2_clk_root       = CCM_CLOCK_ROOT_DISABLE,
    .lpit3_clk_root         = CCM_CLOCK_ROOT_DISABLE,
    .lptimer1_clk_root      = CCM_CLOCK_ROOT_DISABLE,
    .lptimer2_clk_root      = CCM_CLOCK_ROOT_DISABLE,
    .lptimer3_clk_root      = CCM_CLOCK_ROOT_DISABLE,
    .tpm2_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .tpm4_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .tpm5_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .tpm6_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .gpt1_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .gpt2_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .flexspi1_clk_root      = CCM_CLOCK_ROOT_IGNORE,
    .flexspi2_clk_root      = CCM_CLOCK_ROOT_DISABLE,
    .flexspi_slv_clk_root   = CCM_CLOCK_ROOT_DISABLE,
    .can1_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .can2_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .can3_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .lpuart0102_clk_root =
    {
      .action = CCM_CLOCK_ROOT_CONFIGURE,
      .div    = 10,
      .mux    = SYS_PLL3_DIV2,
    },
    .lpuart0304_clk_root    = CCM_CLOCK_ROOT_DISABLE,
    .lpuart0506_clk_root    = CCM_CLOCK_ROOT_DISABLE,
    .lpuart0708_clk_root    = CCM_CLOCK_ROOT_DISABLE,
    .lpuart0910_clk_root    = CCM_CLOCK_ROOT_DISABLE,
    .lpuart1112_clk_root    = CCM_CLOCK_ROOT_DISABLE,
#ifndef CONFIG_IMXRT1180_EVK_BOOTLOADER
    .lpi2c0102_clk_root =
    {
      .action = CCM_CLOCK_ROOT_CONFIGURE,
      .div    = 10,
      .mux    = SYS_PLL3_DIV2,
    },
    .lpi2c0304_clk_root =
    {
      .action = CCM_CLOCK_ROOT_CONFIGURE,
      .div    = 10,
      .mux    = SYS_PLL3_DIV2,
    },
    .lpi2c0506_clk_root =
    {
      .action = CCM_CLOCK_ROOT_CONFIGURE,
      .div    = 10,
      .mux    = SYS_PLL3_DIV2,
    },
    .lpspi0102_clk_root =
    {
      .action = CCM_CLOCK_ROOT_CONFIGURE,
      .div    = 8,
      .mux    = SYS_PLL2_OUT,
    },
    .lpspi0304_clk_root =
    {
      .action = CCM_CLOCK_ROOT_CONFIGURE,
      .div    = 8,
      .mux    = SYS_PLL2_OUT,
    },
    .lpspi0506_clk_root =
    {
      .action = CCM_CLOCK_ROOT_CONFIGURE,
      .div    = 8,
      .mux    = SYS_PLL2_OUT,
    },
#else
    .lpi2c0102_clk_root     = CCM_CLOCK_ROOT_DISABLE,
    .lpi2c0304_clk_root     = CCM_CLOCK_ROOT_DISABLE,
    .lpi2c0506_clk_root     = CCM_CLOCK_ROOT_DISABLE,
    .lpspi0102_clk_root     = CCM_CLOCK_ROOT_DISABLE,
    .lpspi0304_clk_root     = CCM_CLOCK_ROOT_DISABLE,
    .lpspi0506_clk_root     = CCM_CLOCK_ROOT_DISABLE,
#endif
    .i3c1_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .i3c2_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .usdhc1_clk_root        = CCM_CLOCK_ROOT_DISABLE,
    .usdhc2_clk_root        = CCM_CLOCK_ROOT_DISABLE,
    .semc_clk_root          = CCM_CLOCK_ROOT_IGNORE,
    .adc1_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .adc2_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .acmp_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .ecat_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .enet_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .tmr_1588_clk_root      = CCM_CLOCK_ROOT_DISABLE,
    .netc_clk_root          = CCM_CLOCK_ROOT_IGNORE,
    .mac0_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .mac1_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .mac2_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .mac3_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .mac4_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .serdes0_clk_root       = CCM_CLOCK_ROOT_DISABLE,
    .serdes1_clk_root       = CCM_CLOCK_ROOT_DISABLE,
    .serdes2_clk_root       = CCM_CLOCK_ROOT_DISABLE,
    .serdes0_1g_clk_root    = CCM_CLOCK_ROOT_DISABLE,
    .serdes1_1g_clk_root    = CCM_CLOCK_ROOT_DISABLE,
    .serdes2_1g_clk_root    = CCM_CLOCK_ROOT_DISABLE,
    .xcelbusx_clk_root      = CCM_CLOCK_ROOT_DISABLE,
    .xriocu4_clk_root       = CCM_CLOCK_ROOT_DISABLE,
    .mctrl_clk_root         = CCM_CLOCK_ROOT_DISABLE,
    .sai1_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .sai2_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .sai3_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .sai4_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .spdif_clk_root         = CCM_CLOCK_ROOT_DISABLE,
    .asrc_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .mic_clk_root           = CCM_CLOCK_ROOT_DISABLE,
    .cko1_clk_root          = CCM_CLOCK_ROOT_DISABLE,
    .cko2_clk_root          = CCM_CLOCK_ROOT_DISABLE,
  },
  .arm_pll =
  {
    .post_div = 0,
    .loop_div = 133,
  },
  .sys_pll1 =
  {
    .enable = 1,
    .div    = 41,
    .num    = 178956970,
    .denom  = 268435455,
  },
  .sys_pll2 =
  {
    .mfd       = 268435455,
    .ss_enable = 0,
    .pfd0      = 27,
    .pfd1      = 16,
    .pfd2      = 24,
    .pfd3      = 32,
  },
  .sys_pll3 =
  {
    .pfd0 = 22,
    .pfd1 = 33,
    .pfd2 = 22,
    .pfd3 = 18,
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
