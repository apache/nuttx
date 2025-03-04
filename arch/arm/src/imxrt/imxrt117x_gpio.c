/****************************************************************************
 * arch/arm/src/imxrt/imxrt117x_gpio.c
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
 * Private Data
 ****************************************************************************/

static const uint8_t g_gpio1_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_EMC_B1_00_INDEX,          /* GPIO1 Pin 0 */
  IMXRT_PADMUX_GPIO_EMC_B1_01_INDEX,          /* GPIO1 Pin 1 */
  IMXRT_PADMUX_GPIO_EMC_B1_02_INDEX,          /* GPIO1 Pin 2 */
  IMXRT_PADMUX_GPIO_EMC_B1_03_INDEX,          /* GPIO1 Pin 3 */
  IMXRT_PADMUX_GPIO_EMC_B1_04_INDEX,          /* GPIO1 Pin 4 */
  IMXRT_PADMUX_GPIO_EMC_B1_05_INDEX,          /* GPIO1 Pin 5 */
  IMXRT_PADMUX_GPIO_EMC_B1_06_INDEX,          /* GPIO1 Pin 6 */
  IMXRT_PADMUX_GPIO_EMC_B1_07_INDEX,          /* GPIO1 Pin 7 */

  IMXRT_PADMUX_GPIO_EMC_B1_08_INDEX,          /* GPIO1 Pin 8 */
  IMXRT_PADMUX_GPIO_EMC_B1_09_INDEX,          /* GPIO1 Pin 9 */
  IMXRT_PADMUX_GPIO_EMC_B1_10_INDEX,          /* GPIO1 Pin 10 */
  IMXRT_PADMUX_GPIO_EMC_B1_11_INDEX,          /* GPIO1 Pin 11 */
  IMXRT_PADMUX_GPIO_EMC_B1_12_INDEX,          /* GPIO1 Pin 12 */
  IMXRT_PADMUX_GPIO_EMC_B1_13_INDEX,          /* GPIO1 Pin 13 */
  IMXRT_PADMUX_GPIO_EMC_B1_14_INDEX,          /* GPIO1 Pin 14 */
  IMXRT_PADMUX_GPIO_EMC_B1_15_INDEX,          /* GPIO1 Pin 15 */

  IMXRT_PADMUX_GPIO_EMC_B1_16_INDEX,          /* GPIO1 Pin 16 */
  IMXRT_PADMUX_GPIO_EMC_B1_17_INDEX,          /* GPIO1 Pin 17 */
  IMXRT_PADMUX_GPIO_EMC_B1_18_INDEX,          /* GPIO1 Pin 18 */
  IMXRT_PADMUX_GPIO_EMC_B1_19_INDEX,          /* GPIO1 Pin 19 */
  IMXRT_PADMUX_GPIO_EMC_B1_20_INDEX,          /* GPIO1 Pin 20 */
  IMXRT_PADMUX_GPIO_EMC_B1_21_INDEX,          /* GPIO1 Pin 21 */
  IMXRT_PADMUX_GPIO_EMC_B1_22_INDEX,          /* GPIO1 Pin 22 */
  IMXRT_PADMUX_GPIO_EMC_B1_23_INDEX,          /* GPIO1 Pin 23 */

  IMXRT_PADMUX_GPIO_EMC_B1_24_INDEX,          /* GPIO1 Pin 24 */
  IMXRT_PADMUX_GPIO_EMC_B1_25_INDEX,          /* GPIO1 Pin 25 */
  IMXRT_PADMUX_GPIO_EMC_B1_26_INDEX,          /* GPIO1 Pin 26 */
  IMXRT_PADMUX_GPIO_EMC_B1_27_INDEX,          /* GPIO1 Pin 27 */
  IMXRT_PADMUX_GPIO_EMC_B1_28_INDEX,          /* GPIO1 Pin 28 */
  IMXRT_PADMUX_GPIO_EMC_B1_29_INDEX,          /* GPIO1 Pin 29 */
  IMXRT_PADMUX_GPIO_EMC_B1_30_INDEX,          /* GPIO1 Pin 30 */
  IMXRT_PADMUX_GPIO_EMC_B1_31_INDEX           /* GPIO1 Pin 31 */
};

static const uint8_t g_gpio2_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_EMC_B1_32_INDEX,          /* GPIO2 Pin 0 */
  IMXRT_PADMUX_GPIO_EMC_B1_33_INDEX,          /* GPIO2 Pin 1 */
  IMXRT_PADMUX_GPIO_EMC_B1_34_INDEX,          /* GPIO2 Pin 2 */
  IMXRT_PADMUX_GPIO_EMC_B1_35_INDEX,          /* GPIO2 Pin 3 */
  IMXRT_PADMUX_GPIO_EMC_B1_36_INDEX,          /* GPIO2 Pin 4 */
  IMXRT_PADMUX_GPIO_EMC_B1_37_INDEX,          /* GPIO2 Pin 5 */
  IMXRT_PADMUX_GPIO_EMC_B1_38_INDEX,          /* GPIO2 Pin 6 */
  IMXRT_PADMUX_GPIO_EMC_B1_39_INDEX,          /* GPIO2 Pin 7 */

  IMXRT_PADMUX_GPIO_EMC_B1_40_INDEX,          /* GPIO2 Pin 8 */
  IMXRT_PADMUX_GPIO_EMC_B1_41_INDEX,          /* GPIO2 Pin 9 */
  IMXRT_PADMUX_GPIO_EMC_B2_00_INDEX,          /* GPIO2 Pin 10 */
  IMXRT_PADMUX_GPIO_EMC_B2_01_INDEX,          /* GPIO2 Pin 11 */
  IMXRT_PADMUX_GPIO_EMC_B2_02_INDEX,          /* GPIO2 Pin 12 */
  IMXRT_PADMUX_GPIO_EMC_B2_03_INDEX,          /* GPIO2 Pin 13 */
  IMXRT_PADMUX_GPIO_EMC_B2_04_INDEX,          /* GPIO2 Pin 14 */
  IMXRT_PADMUX_GPIO_EMC_B2_05_INDEX,          /* GPIO2 Pin 15 */

  IMXRT_PADMUX_GPIO_EMC_B2_06_INDEX,          /* GPIO2 Pin 16 */
  IMXRT_PADMUX_GPIO_EMC_B2_07_INDEX,          /* GPIO2 Pin 17 */
  IMXRT_PADMUX_GPIO_EMC_B2_08_INDEX,          /* GPIO2 Pin 18 */
  IMXRT_PADMUX_GPIO_EMC_B2_09_INDEX,          /* GPIO2 Pin 19 */
  IMXRT_PADMUX_GPIO_EMC_B2_10_INDEX,          /* GPIO2 Pin 20 */
  IMXRT_PADMUX_GPIO_EMC_B2_11_INDEX,          /* GPIO2 Pin 21 */
  IMXRT_PADMUX_GPIO_EMC_B2_12_INDEX,          /* GPIO2 Pin 22 */
  IMXRT_PADMUX_GPIO_EMC_B2_13_INDEX,          /* GPIO2 Pin 23 */

  IMXRT_PADMUX_GPIO_EMC_B2_14_INDEX,          /* GPIO2 Pin 24 */
  IMXRT_PADMUX_GPIO_EMC_B2_15_INDEX,          /* GPIO2 Pin 25 */
  IMXRT_PADMUX_GPIO_EMC_B2_16_INDEX,          /* GPIO2 Pin 26 */
  IMXRT_PADMUX_GPIO_EMC_B2_17_INDEX,          /* GPIO2 Pin 27 */
  IMXRT_PADMUX_GPIO_EMC_B2_18_INDEX,          /* GPIO2 Pin 28 */
  IMXRT_PADMUX_GPIO_EMC_B2_19_INDEX,          /* GPIO2 Pin 29 */
  IMXRT_PADMUX_GPIO_EMC_B2_20_INDEX,          /* GPIO2 Pin 30 */
  IMXRT_PADMUX_GPIO_AD_00_INDEX,              /* GPIO2 Pin 31 */
};

static const uint8_t g_gpio3_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_AD_01_INDEX,              /* GPIO3 Pin 0 */
  IMXRT_PADMUX_GPIO_AD_02_INDEX,              /* GPIO3 Pin 1 */
  IMXRT_PADMUX_GPIO_AD_03_INDEX,              /* GPIO3 Pin 2 */
  IMXRT_PADMUX_GPIO_AD_04_INDEX,              /* GPIO3 Pin 3 */
  IMXRT_PADMUX_GPIO_AD_05_INDEX,              /* GPIO3 Pin 4 */
  IMXRT_PADMUX_GPIO_AD_06_INDEX,              /* GPIO3 Pin 5 */
  IMXRT_PADMUX_GPIO_AD_07_INDEX,              /* GPIO3 Pin 6 */
  IMXRT_PADMUX_GPIO_AD_08_INDEX,              /* GPIO3 Pin 7 */

  IMXRT_PADMUX_GPIO_AD_09_INDEX,              /* GPIO3 Pin 8 */
  IMXRT_PADMUX_GPIO_AD_10_INDEX,              /* GPIO3 Pin 9 */
  IMXRT_PADMUX_GPIO_AD_11_INDEX,              /* GPIO3 Pin 10 */
  IMXRT_PADMUX_GPIO_AD_12_INDEX,              /* GPIO3 Pin 11 */
  IMXRT_PADMUX_GPIO_AD_13_INDEX,              /* GPIO3 Pin 12 */
  IMXRT_PADMUX_GPIO_AD_14_INDEX,              /* GPIO3 Pin 13 */
  IMXRT_PADMUX_GPIO_AD_15_INDEX,              /* GPIO3 Pin 14 */
  IMXRT_PADMUX_GPIO_AD_16_INDEX,              /* GPIO3 Pin 15 */

  IMXRT_PADMUX_GPIO_AD_17_INDEX,              /* GPIO3 Pin 16 */
  IMXRT_PADMUX_GPIO_AD_18_INDEX,              /* GPIO3 Pin 17 */
  IMXRT_PADMUX_GPIO_AD_19_INDEX,              /* GPIO3 Pin 18 */
  IMXRT_PADMUX_GPIO_AD_20_INDEX,              /* GPIO3 Pin 19 */
  IMXRT_PADMUX_GPIO_AD_21_INDEX,              /* GPIO3 Pin 20 */
  IMXRT_PADMUX_GPIO_AD_22_INDEX,              /* GPIO3 Pin 21 */
  IMXRT_PADMUX_GPIO_AD_23_INDEX,              /* GPIO3 Pin 22 */
  IMXRT_PADMUX_GPIO_AD_24_INDEX,              /* GPIO3 Pin 23 */

  IMXRT_PADMUX_GPIO_AD_25_INDEX,              /* GPIO3 Pin 24 */
  IMXRT_PADMUX_GPIO_AD_26_INDEX,              /* GPIO3 Pin 25 */
  IMXRT_PADMUX_GPIO_AD_27_INDEX,              /* GPIO3 Pin 26 */
  IMXRT_PADMUX_GPIO_AD_28_INDEX,              /* GPIO3 Pin 27 */
  IMXRT_PADMUX_GPIO_AD_29_INDEX,              /* GPIO3 Pin 28 */
  IMXRT_PADMUX_GPIO_AD_30_INDEX,              /* GPIO3 Pin 29 */
  IMXRT_PADMUX_GPIO_AD_31_INDEX,              /* GPIO3 Pin 30 */
  IMXRT_PADMUX_GPIO_AD_32_INDEX,              /* GPIO3 Pin 31 */
};

static const uint8_t g_gpio4_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_AD_33_INDEX,              /* GPIO4 Pin 0 */
  IMXRT_PADMUX_GPIO_AD_34_INDEX,              /* GPIO4 Pin 1 */
  IMXRT_PADMUX_GPIO_AD_35_INDEX,              /* GPIO4 Pin 2 */
  IMXRT_PADMUX_GPIO_SD_B1_00_INDEX,           /* GPIO4 Pin 3 */
  IMXRT_PADMUX_GPIO_SD_B1_01_INDEX,           /* GPIO4 Pin 4 */
  IMXRT_PADMUX_GPIO_SD_B1_02_INDEX,           /* GPIO4 Pin 5 */
  IMXRT_PADMUX_GPIO_SD_B1_03_INDEX,           /* GPIO4 Pin 6 */
  IMXRT_PADMUX_GPIO_SD_B1_04_INDEX,           /* GPIO4 Pin 7 */

  IMXRT_PADMUX_GPIO_SD_B1_05_INDEX,           /* GPIO4 Pin 8 */
  IMXRT_PADMUX_GPIO_SD_B2_00_INDEX,           /* GPIO4 Pin 9 */
  IMXRT_PADMUX_GPIO_SD_B2_01_INDEX,           /* GPIO4 Pin 10 */
  IMXRT_PADMUX_GPIO_SD_B2_02_INDEX,           /* GPIO4 Pin 11 */
  IMXRT_PADMUX_GPIO_SD_B2_03_INDEX,           /* GPIO4 Pin 12 */
  IMXRT_PADMUX_GPIO_SD_B2_04_INDEX,           /* GPIO4 Pin 13 */
  IMXRT_PADMUX_GPIO_SD_B2_05_INDEX,           /* GPIO4 Pin 14 */
  IMXRT_PADMUX_GPIO_SD_B2_06_INDEX,           /* GPIO4 Pin 15 */

  IMXRT_PADMUX_GPIO_SD_B2_07_INDEX,           /* GPIO4 Pin 16 */
  IMXRT_PADMUX_GPIO_SD_B2_08_INDEX,           /* GPIO4 Pin 17 */
  IMXRT_PADMUX_GPIO_SD_B2_09_INDEX,           /* GPIO4 Pin 18 */
  IMXRT_PADMUX_GPIO_SD_B2_10_INDEX,           /* GPIO4 Pin 19 */
  IMXRT_PADMUX_GPIO_SD_B2_11_INDEX,           /* GPIO4 Pin 20 */
  IMXRT_PADMUX_GPIO_DISP_B1_00_INDEX,         /* GPIO4 Pin 21 */
  IMXRT_PADMUX_GPIO_DISP_B1_01_INDEX,         /* GPIO4 Pin 22 */
  IMXRT_PADMUX_GPIO_DISP_B1_02_INDEX,         /* GPIO4 Pin 23 */

  IMXRT_PADMUX_GPIO_DISP_B1_03_INDEX,         /* GPIO4 Pin 24 */
  IMXRT_PADMUX_GPIO_DISP_B1_04_INDEX,         /* GPIO4 Pin 25 */
  IMXRT_PADMUX_GPIO_DISP_B1_05_INDEX,         /* GPIO4 Pin 26 */
  IMXRT_PADMUX_GPIO_DISP_B1_06_INDEX,         /* GPIO4 Pin 27 */
  IMXRT_PADMUX_GPIO_DISP_B1_07_INDEX,         /* GPIO4 Pin 28 */
  IMXRT_PADMUX_GPIO_DISP_B1_08_INDEX,         /* GPIO4 Pin 29 */
  IMXRT_PADMUX_GPIO_DISP_B1_09_INDEX,         /* GPIO4 Pin 30 */
  IMXRT_PADMUX_GPIO_DISP_B1_10_INDEX,         /* GPIO4 Pin 31 */
};

static const uint8_t g_gpio5_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_DISP_B1_11_INDEX,         /* GPIO5 Pin 0 */
  IMXRT_PADMUX_GPIO_DISP_B2_00_INDEX,         /* GPIO5 Pin 1 */
  IMXRT_PADMUX_GPIO_DISP_B2_01_INDEX,         /* GPIO5 Pin 2 */
  IMXRT_PADMUX_GPIO_DISP_B2_02_INDEX,         /* GPIO5 Pin 3 */
  IMXRT_PADMUX_GPIO_DISP_B2_03_INDEX,         /* GPIO5 Pin 4 */
  IMXRT_PADMUX_GPIO_DISP_B2_04_INDEX,         /* GPIO5 Pin 5 */
  IMXRT_PADMUX_GPIO_DISP_B2_05_INDEX,         /* GPIO5 Pin 6 */
  IMXRT_PADMUX_GPIO_DISP_B2_06_INDEX,         /* GPIO5 Pin 7 */

  IMXRT_PADMUX_GPIO_DISP_B2_07_INDEX,         /* GPIO5 Pin 8 */
  IMXRT_PADMUX_GPIO_DISP_B2_08_INDEX,         /* GPIO5 Pin 9 */
  IMXRT_PADMUX_GPIO_DISP_B2_09_INDEX,         /* GPIO5 Pin 10 */
  IMXRT_PADMUX_GPIO_DISP_B2_10_INDEX,         /* GPIO5 Pin 11 */
  IMXRT_PADMUX_GPIO_DISP_B2_11_INDEX,         /* GPIO5 Pin 12 */
  IMXRT_PADMUX_GPIO_DISP_B2_12_INDEX,         /* GPIO5 Pin 13 */
  IMXRT_PADMUX_GPIO_DISP_B2_13_INDEX,         /* GPIO5 Pin 14 */
  IMXRT_PADMUX_GPIO_DISP_B2_14_INDEX,         /* GPIO5 Pin 15 */

  IMXRT_PADMUX_GPIO_DISP_B2_15_INDEX,         /* GPIO5 Pin 16 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 17 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 18 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 19 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 20 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 21 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 22 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 23 */

  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 24 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 25 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 26 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 27 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 28 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 29 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 30 */
  IMXRT_PADMUX_INVALID                        /* GPIO5 Pin 31 */
};

static const uint8_t g_gpio6_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_LPSR_00_INDEX,            /* GPIO6 Pin 0 */
  IMXRT_PADMUX_GPIO_LPSR_01_INDEX,            /* GPIO6 Pin 1 */
  IMXRT_PADMUX_GPIO_LPSR_02_INDEX,            /* GPIO6 Pin 2 */
  IMXRT_PADMUX_GPIO_LPSR_03_INDEX,            /* GPIO6 Pin 3 */
  IMXRT_PADMUX_GPIO_LPSR_04_INDEX,            /* GPIO6 Pin 4 */
  IMXRT_PADMUX_GPIO_LPSR_05_INDEX,            /* GPIO6 Pin 5 */
  IMXRT_PADMUX_GPIO_LPSR_06_INDEX,            /* GPIO6 Pin 6 */
  IMXRT_PADMUX_GPIO_LPSR_07_INDEX,            /* GPIO6 Pin 7 */

  IMXRT_PADMUX_GPIO_LPSR_08_INDEX,            /* GPIO6 Pin 8 */
  IMXRT_PADMUX_GPIO_LPSR_09_INDEX,            /* GPIO6 Pin 9 */
  IMXRT_PADMUX_GPIO_LPSR_10_INDEX,            /* GPIO6 Pin 10 */
  IMXRT_PADMUX_GPIO_LPSR_11_INDEX,            /* GPIO6 Pin 11 */
  IMXRT_PADMUX_GPIO_LPSR_12_INDEX,            /* GPIO6 Pin 12 */
  IMXRT_PADMUX_GPIO_LPSR_13_INDEX,            /* GPIO6 Pin 13 */
  IMXRT_PADMUX_GPIO_LPSR_14_INDEX,            /* GPIO6 Pin 14 */
  IMXRT_PADMUX_GPIO_LPSR_15_INDEX,            /* GPIO6 Pin 15 */

  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 16 */
  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 17 */
  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 18 */
  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 19 */
  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 20 */
  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 21 */
  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 22 */
  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 23 */

  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 24 */
  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 25 */
  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 26 */
  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 27 */
  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 28 */
  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 29 */
  IMXRT_PADMUX_INVALID,                       /* GPIO6 Pin 30 */
  IMXRT_PADMUX_INVALID                        /* GPIO6 Pin 31 */
};

static const uint8_t g_gpio7_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_EMC_B1_00_INDEX,          /* GPIO7 Pin 0 */
  IMXRT_PADMUX_GPIO_EMC_B1_01_INDEX,          /* GPIO7 Pin 1 */
  IMXRT_PADMUX_GPIO_EMC_B1_02_INDEX,          /* GPIO7 Pin 2 */
  IMXRT_PADMUX_GPIO_EMC_B1_03_INDEX,          /* GPIO7 Pin 3 */
  IMXRT_PADMUX_GPIO_EMC_B1_04_INDEX,          /* GPIO7 Pin 4 */
  IMXRT_PADMUX_GPIO_EMC_B1_05_INDEX,          /* GPIO7 Pin 5 */
  IMXRT_PADMUX_GPIO_EMC_B1_06_INDEX,          /* GPIO7 Pin 6 */
  IMXRT_PADMUX_GPIO_EMC_B1_07_INDEX,          /* GPIO7 Pin 7 */

  IMXRT_PADMUX_GPIO_EMC_B1_08_INDEX,          /* GPIO7 Pin 8 */
  IMXRT_PADMUX_GPIO_EMC_B1_09_INDEX,          /* GPIO7 Pin 9 */
  IMXRT_PADMUX_GPIO_EMC_B1_10_INDEX,          /* GPIO7 Pin 10 */
  IMXRT_PADMUX_GPIO_EMC_B1_11_INDEX,          /* GPIO7 Pin 11 */
  IMXRT_PADMUX_GPIO_EMC_B1_12_INDEX,          /* GPIO7 Pin 12 */
  IMXRT_PADMUX_GPIO_EMC_B1_13_INDEX,          /* GPIO7 Pin 13 */
  IMXRT_PADMUX_GPIO_EMC_B1_14_INDEX,          /* GPIO7 Pin 14 */
  IMXRT_PADMUX_GPIO_EMC_B1_15_INDEX,          /* GPIO7 Pin 15 */

  IMXRT_PADMUX_GPIO_EMC_B1_16_INDEX,          /* GPIO7 Pin 16 */
  IMXRT_PADMUX_GPIO_EMC_B1_17_INDEX,          /* GPIO7 Pin 17 */
  IMXRT_PADMUX_GPIO_EMC_B1_18_INDEX,          /* GPIO7 Pin 18 */
  IMXRT_PADMUX_GPIO_EMC_B1_19_INDEX,          /* GPIO7 Pin 19 */
  IMXRT_PADMUX_GPIO_EMC_B1_20_INDEX,          /* GPIO7 Pin 20 */
  IMXRT_PADMUX_GPIO_EMC_B1_21_INDEX,          /* GPIO7 Pin 21 */
  IMXRT_PADMUX_GPIO_EMC_B1_22_INDEX,          /* GPIO7 Pin 22 */
  IMXRT_PADMUX_GPIO_EMC_B1_23_INDEX,          /* GPIO7 Pin 23 */

  IMXRT_PADMUX_GPIO_EMC_B1_24_INDEX,          /* GPIO7 Pin 24 */
  IMXRT_PADMUX_GPIO_EMC_B1_25_INDEX,          /* GPIO7 Pin 25 */
  IMXRT_PADMUX_GPIO_EMC_B1_26_INDEX,          /* GPIO7 Pin 26 */
  IMXRT_PADMUX_GPIO_EMC_B1_27_INDEX,          /* GPIO7 Pin 27 */
  IMXRT_PADMUX_GPIO_EMC_B1_28_INDEX,          /* GPIO7 Pin 28 */
  IMXRT_PADMUX_GPIO_EMC_B1_29_INDEX,          /* GPIO7 Pin 29 */
  IMXRT_PADMUX_GPIO_EMC_B1_30_INDEX,          /* GPIO7 Pin 30 */
  IMXRT_PADMUX_GPIO_EMC_B1_31_INDEX           /* GPIO7 Pin 31 */
};

static const uint8_t g_gpio8_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_EMC_B1_32_INDEX,          /* GPIO8 Pin 0 */
  IMXRT_PADMUX_GPIO_EMC_B1_33_INDEX,          /* GPIO8 Pin 1 */
  IMXRT_PADMUX_GPIO_EMC_B1_34_INDEX,          /* GPIO8 Pin 2 */
  IMXRT_PADMUX_GPIO_EMC_B1_35_INDEX,          /* GPIO8 Pin 3 */
  IMXRT_PADMUX_GPIO_EMC_B1_36_INDEX,          /* GPIO8 Pin 4 */
  IMXRT_PADMUX_GPIO_EMC_B1_37_INDEX,          /* GPIO8 Pin 5 */
  IMXRT_PADMUX_GPIO_EMC_B1_38_INDEX,          /* GPIO8 Pin 6 */
  IMXRT_PADMUX_GPIO_EMC_B1_39_INDEX,          /* GPIO8 Pin 7 */

  IMXRT_PADMUX_GPIO_EMC_B1_40_INDEX,          /* GPIO8 Pin 8 */
  IMXRT_PADMUX_GPIO_EMC_B1_41_INDEX,          /* GPIO8 Pin 9 */
  IMXRT_PADMUX_GPIO_EMC_B2_00_INDEX,          /* GPIO8 Pin 10 */
  IMXRT_PADMUX_GPIO_EMC_B2_01_INDEX,          /* GPIO8 Pin 11 */
  IMXRT_PADMUX_GPIO_EMC_B2_02_INDEX,          /* GPIO8 Pin 12 */
  IMXRT_PADMUX_GPIO_EMC_B2_03_INDEX,          /* GPIO8 Pin 13 */
  IMXRT_PADMUX_GPIO_EMC_B2_04_INDEX,          /* GPIO8 Pin 14 */
  IMXRT_PADMUX_GPIO_EMC_B2_05_INDEX,          /* GPIO8 Pin 15 */

  IMXRT_PADMUX_GPIO_EMC_B2_06_INDEX,          /* GPIO8 Pin 16 */
  IMXRT_PADMUX_GPIO_EMC_B2_07_INDEX,          /* GPIO8 Pin 17 */
  IMXRT_PADMUX_GPIO_EMC_B2_08_INDEX,          /* GPIO8 Pin 18 */
  IMXRT_PADMUX_GPIO_EMC_B2_09_INDEX,          /* GPIO8 Pin 19 */
  IMXRT_PADMUX_GPIO_EMC_B2_10_INDEX,          /* GPIO8 Pin 20 */
  IMXRT_PADMUX_GPIO_EMC_B2_11_INDEX,          /* GPIO8 Pin 21 */
  IMXRT_PADMUX_GPIO_EMC_B2_12_INDEX,          /* GPIO8 Pin 22 */
  IMXRT_PADMUX_GPIO_EMC_B2_13_INDEX,          /* GPIO8 Pin 23 */

  IMXRT_PADMUX_GPIO_EMC_B2_14_INDEX,          /* GPIO8 Pin 24 */
  IMXRT_PADMUX_GPIO_EMC_B2_15_INDEX,          /* GPIO8 Pin 25 */
  IMXRT_PADMUX_GPIO_EMC_B2_16_INDEX,          /* GPIO8 Pin 26 */
  IMXRT_PADMUX_GPIO_EMC_B2_17_INDEX,          /* GPIO8 Pin 27 */
  IMXRT_PADMUX_GPIO_EMC_B2_18_INDEX,          /* GPIO8 Pin 28 */
  IMXRT_PADMUX_GPIO_EMC_B2_19_INDEX,          /* GPIO8 Pin 29 */
  IMXRT_PADMUX_GPIO_EMC_B2_20_INDEX,          /* GPIO8 Pin 30 */
  IMXRT_PADMUX_GPIO_AD_00_INDEX               /* GPIO8 Pin 31 */
};

static const uint8_t g_gpio9_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_AD_01_INDEX,              /* GPIO9 Pin 0 */
  IMXRT_PADMUX_GPIO_AD_02_INDEX,              /* GPIO9 Pin 1 */
  IMXRT_PADMUX_GPIO_AD_03_INDEX,              /* GPIO9 Pin 2 */
  IMXRT_PADMUX_GPIO_AD_04_INDEX,              /* GPIO9 Pin 3 */
  IMXRT_PADMUX_GPIO_AD_05_INDEX,              /* GPIO9 Pin 4 */
  IMXRT_PADMUX_GPIO_AD_06_INDEX,              /* GPIO9 Pin 5 */
  IMXRT_PADMUX_GPIO_AD_07_INDEX,              /* GPIO9 Pin 6 */
  IMXRT_PADMUX_GPIO_AD_08_INDEX,              /* GPIO9 Pin 7 */

  IMXRT_PADMUX_GPIO_AD_09_INDEX,              /* GPIO9 Pin 8 */
  IMXRT_PADMUX_GPIO_AD_10_INDEX,              /* GPIO9 Pin 9 */
  IMXRT_PADMUX_GPIO_AD_11_INDEX,              /* GPIO9 Pin 10 */
  IMXRT_PADMUX_GPIO_AD_12_INDEX,              /* GPIO9 Pin 11 */
  IMXRT_PADMUX_GPIO_AD_13_INDEX,              /* GPIO9 Pin 12 */
  IMXRT_PADMUX_GPIO_AD_14_INDEX,              /* GPIO9 Pin 13 */
  IMXRT_PADMUX_GPIO_AD_15_INDEX,              /* GPIO9 Pin 14 */
  IMXRT_PADMUX_GPIO_AD_16_INDEX,              /* GPIO9 Pin 15 */

  IMXRT_PADMUX_GPIO_AD_17_INDEX,              /* GPIO9 Pin 16 */
  IMXRT_PADMUX_GPIO_AD_18_INDEX,              /* GPIO9 Pin 17 */
  IMXRT_PADMUX_GPIO_AD_19_INDEX,              /* GPIO9 Pin 18 */
  IMXRT_PADMUX_GPIO_AD_20_INDEX,              /* GPIO9 Pin 19 */
  IMXRT_PADMUX_GPIO_AD_21_INDEX,              /* GPIO9 Pin 20 */
  IMXRT_PADMUX_GPIO_AD_22_INDEX,              /* GPIO9 Pin 21 */
  IMXRT_PADMUX_GPIO_AD_23_INDEX,              /* GPIO9 Pin 22 */
  IMXRT_PADMUX_GPIO_AD_24_INDEX,              /* GPIO9 Pin 23 */

  IMXRT_PADMUX_GPIO_AD_25_INDEX,              /* GPIO9 Pin 24 */
  IMXRT_PADMUX_GPIO_AD_26_INDEX,              /* GPIO9 Pin 25 */
  IMXRT_PADMUX_GPIO_AD_27_INDEX,              /* GPIO9 Pin 26 */
  IMXRT_PADMUX_GPIO_AD_28_INDEX,              /* GPIO9 Pin 27 */
  IMXRT_PADMUX_GPIO_AD_29_INDEX,              /* GPIO9 Pin 28 */
  IMXRT_PADMUX_GPIO_AD_30_INDEX,              /* GPIO9 Pin 29 */
  IMXRT_PADMUX_GPIO_AD_31_INDEX,              /* GPIO9 Pin 30 */
  IMXRT_PADMUX_GPIO_AD_32_INDEX               /* GPIO9 Pin 31 */
};

static const uint8_t g_gpio10_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_AD_33_INDEX,              /* GPIO10 Pin 0 */
  IMXRT_PADMUX_GPIO_AD_34_INDEX,              /* GPIO10 Pin 1 */
  IMXRT_PADMUX_GPIO_AD_35_INDEX,              /* GPIO10 Pin 2 */
  IMXRT_PADMUX_GPIO_SD_B1_00_INDEX,           /* GPIO10 Pin 3 */
  IMXRT_PADMUX_GPIO_SD_B1_01_INDEX,           /* GPIO10 Pin 4 */
  IMXRT_PADMUX_GPIO_SD_B1_02_INDEX,           /* GPIO10 Pin 5 */
  IMXRT_PADMUX_GPIO_SD_B1_03_INDEX,           /* GPIO10 Pin 6 */
  IMXRT_PADMUX_GPIO_SD_B1_04_INDEX,           /* GPIO10 Pin 7 */

  IMXRT_PADMUX_GPIO_SD_B1_05_INDEX,           /* GPIO10 Pin 8 */
  IMXRT_PADMUX_GPIO_SD_B2_00_INDEX,           /* GPIO10 Pin 9 */
  IMXRT_PADMUX_GPIO_SD_B2_01_INDEX,           /* GPIO10 Pin 10 */
  IMXRT_PADMUX_GPIO_SD_B2_02_INDEX,           /* GPIO10 Pin 11 */
  IMXRT_PADMUX_GPIO_SD_B2_03_INDEX,           /* GPIO10 Pin 12 */
  IMXRT_PADMUX_GPIO_SD_B2_04_INDEX,           /* GPIO10 Pin 13 */
  IMXRT_PADMUX_GPIO_SD_B2_05_INDEX,           /* GPIO10 Pin 14 */
  IMXRT_PADMUX_GPIO_SD_B2_06_INDEX,           /* GPIO10 Pin 15 */

  IMXRT_PADMUX_GPIO_SD_B2_07_INDEX,           /* GPIO10 Pin 16 */
  IMXRT_PADMUX_GPIO_SD_B2_08_INDEX,           /* GPIO10 Pin 17 */
  IMXRT_PADMUX_GPIO_SD_B2_09_INDEX,           /* GPIO10 Pin 18 */
  IMXRT_PADMUX_GPIO_SD_B2_10_INDEX,           /* GPIO10 Pin 19 */
  IMXRT_PADMUX_GPIO_SD_B2_11_INDEX,           /* GPIO10 Pin 20 */
  IMXRT_PADMUX_GPIO_DISP_B1_00_INDEX,         /* GPIO10 Pin 21 */
  IMXRT_PADMUX_GPIO_DISP_B1_01_INDEX,         /* GPIO10 Pin 22 */
  IMXRT_PADMUX_GPIO_DISP_B1_02_INDEX,         /* GPIO10 Pin 23 */

  IMXRT_PADMUX_GPIO_DISP_B1_03_INDEX,         /* GPIO10 Pin 24 */
  IMXRT_PADMUX_GPIO_DISP_B1_04_INDEX,         /* GPIO10 Pin 25 */
  IMXRT_PADMUX_GPIO_DISP_B1_05_INDEX,         /* GPIO10 Pin 26 */
  IMXRT_PADMUX_GPIO_DISP_B1_06_INDEX,         /* GPIO10 Pin 27 */
  IMXRT_PADMUX_GPIO_DISP_B1_07_INDEX,         /* GPIO10 Pin 28 */
  IMXRT_PADMUX_GPIO_DISP_B1_08_INDEX,         /* GPIO10 Pin 29 */
  IMXRT_PADMUX_GPIO_DISP_B1_09_INDEX,         /* GPIO10 Pin 30 */
  IMXRT_PADMUX_GPIO_DISP_B1_10_INDEX          /* GPIO10 Pin 31 */
};

static const uint8_t g_gpio11_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_DISP_B1_11_INDEX,         /* GPIO11 Pin 0 */
  IMXRT_PADMUX_GPIO_DISP_B2_00_INDEX,         /* GPIO11 Pin 1 */
  IMXRT_PADMUX_GPIO_DISP_B2_01_INDEX,         /* GPIO11 Pin 2 */
  IMXRT_PADMUX_GPIO_DISP_B2_02_INDEX,         /* GPIO11 Pin 3 */
  IMXRT_PADMUX_GPIO_DISP_B2_03_INDEX,         /* GPIO11 Pin 4 */
  IMXRT_PADMUX_GPIO_DISP_B2_04_INDEX,         /* GPIO11 Pin 5 */
  IMXRT_PADMUX_GPIO_DISP_B2_05_INDEX,         /* GPIO11 Pin 6 */
  IMXRT_PADMUX_GPIO_DISP_B2_06_INDEX,         /* GPIO11 Pin 7 */

  IMXRT_PADMUX_GPIO_DISP_B2_07_INDEX,         /* GPIO11 Pin 8 */
  IMXRT_PADMUX_GPIO_DISP_B2_08_INDEX,         /* GPIO11 Pin 9 */
  IMXRT_PADMUX_GPIO_DISP_B2_09_INDEX,         /* GPIO11 Pin 10 */
  IMXRT_PADMUX_GPIO_DISP_B2_10_INDEX,         /* GPIO11 Pin 11 */
  IMXRT_PADMUX_GPIO_DISP_B2_11_INDEX,         /* GPIO11 Pin 12 */
  IMXRT_PADMUX_GPIO_DISP_B2_12_INDEX,         /* GPIO11 Pin 13 */
  IMXRT_PADMUX_GPIO_DISP_B2_13_INDEX,         /* GPIO11 Pin 14 */
  IMXRT_PADMUX_GPIO_DISP_B2_14_INDEX,         /* GPIO11 Pin 15 */

  IMXRT_PADMUX_GPIO_DISP_B2_15_INDEX,         /* GPIO11 Pin 16 */
  IMXRT_PADMUX_INVALID,                       /* GPIO11 Pin 17 */
  IMXRT_PADMUX_INVALID,                       /* GPIO11 Pin 18 */
  IMXRT_PADMUX_INVALID,                       /* GPIO11 Pin 19 */
  IMXRT_PADMUX_INVALID,                       /* GPIO11 Pin 20 */
  IMXRT_PADMUX_INVALID,                       /* GPIO11 Pin 21 */
  IMXRT_PADMUX_INVALID,                       /* GPIO11 Pin 22 */
  IMXRT_PADMUX_INVALID,                       /* GPIO11 Pin 23 */

  IMXRT_PADMUX_INVALID,                       /* GPIO11 Pin 24 */
  IMXRT_PADMUX_INVALID,                       /* GPIO11 Pin 25 */
  IMXRT_PADMUX_INVALID,                       /* GPIO11 Pin 26 */
  IMXRT_PADMUX_INVALID,                       /* GPIO11 Pin 27 */
  IMXRT_PADMUX_INVALID,                       /* GPIO11 Pin 28 */
  IMXRT_PADMUX_INVALID,                       /* GPIO11 Pin 29 */
  IMXRT_PADMUX_INVALID,                       /* GPIO11 Pin 30 */
  IMXRT_PADMUX_INVALID                        /* GPIO11 Pin 31 */
};

static const uint8_t g_gpio12_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_LPSR_00_INDEX,            /* GPIO12 Pin 0 */
  IMXRT_PADMUX_GPIO_LPSR_01_INDEX,            /* GPIO12 Pin 1 */
  IMXRT_PADMUX_GPIO_LPSR_02_INDEX,            /* GPIO12 Pin 2 */
  IMXRT_PADMUX_GPIO_LPSR_03_INDEX,            /* GPIO12 Pin 3 */
  IMXRT_PADMUX_GPIO_LPSR_04_INDEX,            /* GPIO12 Pin 4 */
  IMXRT_PADMUX_GPIO_LPSR_05_INDEX,            /* GPIO12 Pin 5 */
  IMXRT_PADMUX_GPIO_LPSR_06_INDEX,            /* GPIO12 Pin 6 */
  IMXRT_PADMUX_GPIO_LPSR_07_INDEX,            /* GPIO12 Pin 7 */

  IMXRT_PADMUX_GPIO_LPSR_08_INDEX,            /* GPIO12 Pin 8 */
  IMXRT_PADMUX_GPIO_LPSR_09_INDEX,            /* GPIO12 Pin 9 */
  IMXRT_PADMUX_GPIO_LPSR_10_INDEX,            /* GPIO12 Pin 10 */
  IMXRT_PADMUX_GPIO_LPSR_11_INDEX,            /* GPIO12 Pin 11 */
  IMXRT_PADMUX_GPIO_LPSR_12_INDEX,            /* GPIO12 Pin 12 */
  IMXRT_PADMUX_GPIO_LPSR_13_INDEX,            /* GPIO12 Pin 13 */
  IMXRT_PADMUX_GPIO_LPSR_14_INDEX,            /* GPIO12 Pin 14 */
  IMXRT_PADMUX_GPIO_LPSR_15_INDEX,            /* GPIO12 Pin 15 */

  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 16 */
  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 17 */
  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 18 */
  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 19 */
  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 20 */
  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 21 */
  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 22 */
  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 23 */

  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 24 */
  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 25 */
  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 26 */
  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 27 */
  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 28 */
  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 29 */
  IMXRT_PADMUX_INVALID,                       /* GPIO12 Pin 30 */
  IMXRT_PADMUX_INVALID                        /* GPIO12 Pin 31 */
};

static const uint8_t g_gpio13_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_WAKEUP_INDEX,                  /* GPIO13 Pin 0 */
  IMXRT_PADMUX_PMIC_ON_REQ_INDEX,             /* GPIO13 Pin 1 */
  IMXRT_PADMUX_PMIC_STBY_REQ_INDEX,           /* GPIO13 Pin 2 */
  IMXRT_PADMUX_GPIO_SNVS_00_INDEX,            /* GPIO13 Pin 3 */
  IMXRT_PADMUX_GPIO_SNVS_01_INDEX,            /* GPIO13 Pin 4 */
  IMXRT_PADMUX_GPIO_SNVS_02_INDEX,            /* GPIO13 Pin 5 */
  IMXRT_PADMUX_GPIO_SNVS_03_INDEX,            /* GPIO13 Pin 6 */
  IMXRT_PADMUX_GPIO_SNVS_04_INDEX,            /* GPIO13 Pin 7 */

  IMXRT_PADMUX_GPIO_SNVS_05_INDEX,            /* GPIO13 Pin 8 */
  IMXRT_PADMUX_GPIO_SNVS_06_INDEX,            /* GPIO13 Pin 9 */
  IMXRT_PADMUX_GPIO_SNVS_07_INDEX,            /* GPIO13 Pin 10 */
  IMXRT_PADMUX_GPIO_SNVS_08_INDEX,            /* GPIO13 Pin 11 */
  IMXRT_PADMUX_GPIO_SNVS_09_INDEX,            /* GPIO13 Pin 12 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 13 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 14 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 15 */

  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 16 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 17 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 18 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 19 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 20 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 21 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 22 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 23 */

  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 24 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 25 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 26 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 27 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 28 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 29 */
  IMXRT_PADMUX_INVALID,                       /* GPIO13 Pin 30 */
  IMXRT_PADMUX_INVALID                        /* GPIO13 Pin 31 */
};

static const uint8_t * const g_gpio_padmux[IMXRT_GPIO_NPORTS + 1] =
{
  g_gpio1_padmux,                             /* GPIO1 */
  g_gpio2_padmux,                             /* GPIO2 */
  g_gpio3_padmux,                             /* GPIO3 */
  g_gpio4_padmux,                             /* GPIO4 */
  g_gpio5_padmux,                             /* GPIO5 */
  g_gpio6_padmux,                             /* GPIO6 */
  g_gpio7_padmux,                             /* GPIO7 */
  g_gpio8_padmux,                             /* GPIO8 */
  g_gpio9_padmux,                             /* GPIO9 */
  g_gpio10_padmux,                            /* GPIO10 */
  g_gpio11_padmux,                            /* GPIO11 */
  g_gpio12_padmux,                            /* GPIO12 */
  g_gpio13_padmux,                            /* GPIO13 */
  NULL                                        /* End of list */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/