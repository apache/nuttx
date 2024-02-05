/****************************************************************************
 * boards/arm/imxrt/imxrt1170-evk/src/imxrt_clockconfig.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "imxrt_clockconfig.h"

#include "imxrt1170-evk.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Each IMXRT117X board must provide the following initialized structure.
 * This is needed to establish the initial board clocking.
 */

const struct clock_configuration_s g_initial_clkconfig =
{
  .ccm               =
  {
    .m7_clk_root     =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = M7_CLK_ROOT_PLL_ARM_CLK,
    },
    .m4_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = M4_CLK_ROOT_SYS_PLL3_PFD3,
    },
    .bus_clk_root =
    {
      .enable        = 1,
      .div           = 2,
      .mux           = BUS_CLK_ROOT_SYS_PLL3_CLK,
    },
    .bus_lpsr_clk_root =
    {
      .enable        = 1,
      .div           = 3,
      .mux           = BUS_LPSR_CLK_ROOT_SYS_PLL3_CLK,
    },
    .semc_clk_root =
    {
      .enable        = 1,
      .div           = 3,
      .mux           = SEMC_CLK_ROOT_SYS_PLL2_PFD1,
    },
    .cssys_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = CSSYS_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .cstrace_clk_root =
    {
      .enable        = 1,
      .div           = 4,
      .mux           = CSTRACE_CLK_ROOT_SYS_PLL2_CLK,
    },
    .m4_systick_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = M4_SYSTICK_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .m7_systick_clk_root =
    {
      .enable        = 1,
      .div           = 240,
      .mux           = M7_SYSTICK_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .adc1_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = ADC1_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .adc2_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = ADC2_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .acmp_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = ACMP_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .flexio1_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = FLEXIO1_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .flexio2_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = FLEXIO2_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .gpt1_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = GPT1_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .gpt2_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = GPT2_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .gpt3_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = GPT3_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .gpt4_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = GPT4_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .gpt5_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = GPT5_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .gpt6_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = GPT6_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .flexspi1_clk_root =
    {
      .enable        = 1,
      .div           = 4,
      .mux           = FLEXSPI1_CLK_ROOT_SYS_PLL2_CLK,
    },
    .flexspi2_clk_root =
    {
      .enable        = 1,
      .div           = 132,
      .mux           = FLEXSPI1_CLK_ROOT_SYS_PLL2_CLK,
    },
    .can1_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = CAN1_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .can2_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = CAN2_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .can3_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = CAN3_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpuart1_clk_root =
    {
      .enable        = 1,
      .div           = 22,
      .mux           = LPUART1_CLK_ROOT_SYS_PLL2_CLK,
    },
    .lpuart2_clk_root =
    {
      .enable        = 1,
      .div           = 22,
      .mux           = LPUART2_CLK_ROOT_SYS_PLL2_CLK,
    },
    .lpuart3_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPUART3_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpuart4_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPUART4_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpuart5_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPUART5_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpuart6_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPUART6_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpuart7_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPUART7_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpuart8_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPUART8_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpuart9_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPUART9_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpuart10_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPUART10_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpuart11_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPUART11_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpuart12_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPUART12_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpi2c1_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPI2C1_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpi2c2_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPI2C2_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpi2c3_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPI2C3_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpi2c4_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPI2C4_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpi2c5_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPI2C5_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpi2c6_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPI2C6_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpspi1_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPSPI1_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpspi2_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPSPI2_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpspi3_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPSPI3_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpspi4_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPSPI4_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpspi5_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPSPI5_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lpspi6_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LPSPI6_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .emv1_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = EMV1_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .emv2_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = EMV2_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .enet1_clk_root =
    {
      .enable        = 1,
      .div           = 10,
      .mux           = ENET1_CLK_ROOT_SYS_PLL1_DIV2,
    },
    .enet2_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = ENET2_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .enet_qos_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = ENET_QOS_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .enet_25m_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = ENET_25M_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .enet_timer1_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = ENET_TIMER1_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .enet_timer2_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = ENET_TIMER2_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .enet_timer3_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = ENET_TIMER3_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .usdhc1_clk_root =
    {
      .enable        = 1,
      .div           = 2,
      .mux           = USDHC1_CLK_ROOT_SYS_PLL2_PFD2,
    },
    .usdhc2_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = USDHC2_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .asrc_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = ASRC_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .mqs_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = MQS_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .mic_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = MIC_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .spdif_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = SPDIF_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .sai1_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = SAI1_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .sai2_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = SAI2_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .sai3_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = SAI3_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .sai4_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = SAI4_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .gc355_clk_root =
    {
      .enable        = 1,
      .div           = 2,
      .mux           = GC355_CLK_ROOT_VIDEO_PLL_CLK,
    },
    .lcdif_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LCDIF_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .lcdifv2_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = LCDIFV2_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .mipi_ref_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = MIPI_REF_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .mipi_esc_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = MIPI_ESC_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .csi2_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = CSI2_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .csi2_esc_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = CSI2_ESC_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .csi2_ui_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = CSI2_UI_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .csi_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = CSI_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .cko1_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = CKO1_CLK_ROOT_OSC_RC_48M_DIV2,
    },
    .cko2_clk_root =
    {
      .enable        = 1,
      .div           = 1,
      .mux           = CKO2_CLK_ROOT_OSC_RC_48M_DIV2,
    },
  },
  .arm_pll =
  {
    /* ARM_PLL = Fin * ( loop_div / ( 2 * post_div ) ) */

    .post_div        = 0,   /* 0 = DIV by 2
                             * 1 = DIV by 4
                             * 2 = DIV by 8
                             * 3 = DIV by 1 */
    .loop_div        = 166, /* ARM_PLL = 996 Mhz */
  },
  .sys_pll1 =
  {
    .enable          = 1,
    .div             = 41,
    .num             = 178956970,
    .denom           = 268435455,
  },
  .sys_pll2 =
  {
    .mfd             = 268435455,
    .ss_enable       = 0,
    .pfd0            = 27, /* (528 * 18) / 27 = 352 MHz */
    .pfd1            = 16, /* (528 * 16) / 16 = 594 MHz */
    .pfd2            = 24, /* (528 * 24) / 27 = 396 MHz */
    .pfd3            = 32, /* (528 * 32) / 27 = 297 MHz */
  },
  .sys_pll3 =
  {
    .pfd0            = 13, /* (480 * 18) / 13 = 8640/13 = 664.62 MHz */
    .pfd1            = 17, /* (480 * 18) / 17 = 8640/17 = 508.24 MHz */
    .pfd2            = 32, /* (480 * 18) / 32 = 270 MHz */
    .pfd3            = 22, /* (480 * 18) / 22 = 8640/20 = 392.73 MHz */
  }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
