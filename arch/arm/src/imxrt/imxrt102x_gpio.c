/****************************************************************************
 * arch/arm/src/imxrt/imxrt102x_gpio.c
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
  IMXRT_PADMUX_GPIO_AD_B0_00_INDEX,           /* GPIO1 Pin 0 */
  IMXRT_PADMUX_GPIO_AD_B0_01_INDEX,           /* GPIO1 Pin 1 */
  IMXRT_PADMUX_GPIO_AD_B0_02_INDEX,           /* GPIO1 Pin 2 */
  IMXRT_PADMUX_GPIO_AD_B0_03_INDEX,           /* GPIO1 Pin 3 */
  IMXRT_PADMUX_GPIO_AD_B0_04_INDEX,           /* GPIO1 Pin 4 */
  IMXRT_PADMUX_GPIO_AD_B0_05_INDEX,           /* GPIO1 Pin 5 */
  IMXRT_PADMUX_GPIO_AD_B0_06_INDEX,           /* GPIO1 Pin 6 */
  IMXRT_PADMUX_GPIO_AD_B0_07_INDEX,           /* GPIO1 Pin 7 */

  IMXRT_PADMUX_GPIO_AD_B0_08_INDEX,           /* GPIO1 Pin 8 */
  IMXRT_PADMUX_GPIO_AD_B0_09_INDEX,           /* GPIO1 Pin 9 */
  IMXRT_PADMUX_GPIO_AD_B0_10_INDEX,           /* GPIO1 Pin 10 */
  IMXRT_PADMUX_GPIO_AD_B0_11_INDEX,           /* GPIO1 Pin 11 */
  IMXRT_PADMUX_GPIO_AD_B0_12_INDEX,           /* GPIO1 Pin 12 */
  IMXRT_PADMUX_GPIO_AD_B0_13_INDEX,           /* GPIO1 Pin 13 */
  IMXRT_PADMUX_GPIO_AD_B0_14_INDEX,           /* GPIO1 Pin 14 */
  IMXRT_PADMUX_GPIO_AD_B0_15_INDEX,           /* GPIO1 Pin 15 */

  IMXRT_PADMUX_GPIO_AD_B1_00_INDEX,           /* GPIO1 Pin 16 */
  IMXRT_PADMUX_GPIO_AD_B1_01_INDEX,           /* GPIO1 Pin 17 */
  IMXRT_PADMUX_GPIO_AD_B1_02_INDEX,           /* GPIO1 Pin 18 */
  IMXRT_PADMUX_GPIO_AD_B1_03_INDEX,           /* GPIO1 Pin 19 */
  IMXRT_PADMUX_GPIO_AD_B1_04_INDEX,           /* GPIO1 Pin 20 */
  IMXRT_PADMUX_GPIO_AD_B1_05_INDEX,           /* GPIO1 Pin 21 */
  IMXRT_PADMUX_GPIO_AD_B1_06_INDEX,           /* GPIO1 Pin 22 */
  IMXRT_PADMUX_GPIO_AD_B1_07_INDEX,           /* GPIO1 Pin 23 */

  IMXRT_PADMUX_GPIO_AD_B1_08_INDEX,           /* GPIO1 Pin 24 */
  IMXRT_PADMUX_GPIO_AD_B1_09_INDEX,           /* GPIO1 Pin 25 */
  IMXRT_PADMUX_GPIO_AD_B1_10_INDEX,           /* GPIO1 Pin 26 */
  IMXRT_PADMUX_GPIO_AD_B1_11_INDEX,           /* GPIO1 Pin 27 */
  IMXRT_PADMUX_GPIO_AD_B1_12_INDEX,           /* GPIO1 Pin 28 */
  IMXRT_PADMUX_GPIO_AD_B1_13_INDEX,           /* GPIO1 Pin 29 */
  IMXRT_PADMUX_GPIO_AD_B1_14_INDEX,           /* GPIO1 Pin 30 */
  IMXRT_PADMUX_GPIO_AD_B1_15_INDEX            /* GPIO1 Pin 31 */
};

static const uint8_t g_gpio2_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_EMC_00_INDEX,             /* GPIO2 Pin 0 */
  IMXRT_PADMUX_GPIO_EMC_01_INDEX,             /* GPIO2 Pin 1 */
  IMXRT_PADMUX_GPIO_EMC_02_INDEX,             /* GPIO2 Pin 2 */
  IMXRT_PADMUX_GPIO_EMC_03_INDEX,             /* GPIO2 Pin 3 */
  IMXRT_PADMUX_GPIO_EMC_04_INDEX,             /* GPIO2 Pin 4 */
  IMXRT_PADMUX_GPIO_EMC_05_INDEX,             /* GPIO2 Pin 5 */
  IMXRT_PADMUX_GPIO_EMC_06_INDEX,             /* GPIO2 Pin 6 */
  IMXRT_PADMUX_GPIO_EMC_07_INDEX,             /* GPIO2 Pin 7 */

  IMXRT_PADMUX_GPIO_EMC_08_INDEX,             /* GPIO2 Pin 8 */
  IMXRT_PADMUX_GPIO_EMC_09_INDEX,             /* GPIO2 Pin 9 */
  IMXRT_PADMUX_GPIO_EMC_10_INDEX,             /* GPIO2 Pin 10 */
  IMXRT_PADMUX_GPIO_EMC_11_INDEX,             /* GPIO2 Pin 11 */
  IMXRT_PADMUX_GPIO_EMC_12_INDEX,             /* GPIO2 Pin 12 */
  IMXRT_PADMUX_GPIO_EMC_13_INDEX,             /* GPIO2 Pin 13 */
  IMXRT_PADMUX_GPIO_EMC_14_INDEX,             /* GPIO2 Pin 14 */
  IMXRT_PADMUX_GPIO_EMC_15_INDEX,             /* GPIO2 Pin 15 */

  IMXRT_PADMUX_GPIO_EMC_16_INDEX,             /* GPIO2 Pin 16 */
  IMXRT_PADMUX_GPIO_EMC_17_INDEX,             /* GPIO2 Pin 17 */
  IMXRT_PADMUX_GPIO_EMC_18_INDEX,             /* GPIO2 Pin 18 */
  IMXRT_PADMUX_GPIO_EMC_19_INDEX,             /* GPIO2 Pin 19 */
  IMXRT_PADMUX_GPIO_EMC_20_INDEX,             /* GPIO2 Pin 20 */
  IMXRT_PADMUX_GPIO_EMC_21_INDEX,             /* GPIO2 Pin 21 */
  IMXRT_PADMUX_GPIO_EMC_22_INDEX,             /* GPIO2 Pin 22 */
  IMXRT_PADMUX_GPIO_EMC_23_INDEX,             /* GPIO2 Pin 23 */

  IMXRT_PADMUX_GPIO_EMC_24_INDEX,             /* GPIO2 Pin 24 */
  IMXRT_PADMUX_GPIO_EMC_25_INDEX,             /* GPIO2 Pin 25 */
  IMXRT_PADMUX_GPIO_EMC_26_INDEX,             /* GPIO2 Pin 26 */
  IMXRT_PADMUX_GPIO_EMC_27_INDEX,             /* GPIO2 Pin 27 */
  IMXRT_PADMUX_GPIO_EMC_28_INDEX,             /* GPIO2 Pin 28 */
  IMXRT_PADMUX_GPIO_EMC_29_INDEX,             /* GPIO2 Pin 29 */
  IMXRT_PADMUX_GPIO_EMC_30_INDEX,             /* GPIO2 Pin 30 */
  IMXRT_PADMUX_GPIO_EMC_31_INDEX              /* GPIO2 Pin 31 */
};

static const uint8_t g_gpio3_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_EMC_32_INDEX,             /* GPIO3 Pin 0 */
  IMXRT_PADMUX_GPIO_EMC_33_INDEX,             /* GPIO3 Pin 1 */
  IMXRT_PADMUX_GPIO_EMC_34_INDEX,             /* GPIO3 Pin 2 */
  IMXRT_PADMUX_GPIO_EMC_35_INDEX,             /* GPIO3 Pin 3 */
  IMXRT_PADMUX_GPIO_EMC_36_INDEX,             /* GPIO3 Pin 4 */
  IMXRT_PADMUX_GPIO_EMC_37_INDEX,             /* GPIO3 Pin 5 */
  IMXRT_PADMUX_GPIO_EMC_38_INDEX,             /* GPIO3 Pin 6 */
  IMXRT_PADMUX_GPIO_EMC_39_INDEX,             /* GPIO3 Pin 7 */

  IMXRT_PADMUX_GPIO_EMC_40_INDEX,             /* GPIO3 Pin 8 */
  IMXRT_PADMUX_GPIO_EMC_41_INDEX,             /* GPIO3 Pin 9 */
  IMXRT_PADMUX_INVALID,                       /* GPIO3 Pin 10 */
  IMXRT_PADMUX_INVALID,                       /* GPIO3 Pin 11 */
  IMXRT_PADMUX_INVALID,                       /* GPIO3 Pin 12 */
  IMXRT_PADMUX_GPIO_SD_B0_00_INDEX,           /* GPIO3 Pin 13 */
  IMXRT_PADMUX_GPIO_SD_B0_01_INDEX,           /* GPIO3 Pin 14 */
  IMXRT_PADMUX_GPIO_SD_B0_02_INDEX,           /* GPIO3 Pin 15 */

  IMXRT_PADMUX_GPIO_SD_B0_03_INDEX,           /* GPIO3 Pin 16 */
  IMXRT_PADMUX_GPIO_SD_B0_04_INDEX,           /* GPIO3 Pin 17 */
  IMXRT_PADMUX_GPIO_SD_B0_05_INDEX,           /* GPIO3 Pin 18 */
  IMXRT_PADMUX_GPIO_SD_B0_06_INDEX,           /* GPIO3 Pin 19 */
  IMXRT_PADMUX_GPIO_SD_B1_00_INDEX,           /* GPIO3 Pin 20 */
  IMXRT_PADMUX_GPIO_SD_B1_01_INDEX,           /* GPIO3 Pin 21 */
  IMXRT_PADMUX_GPIO_SD_B1_02_INDEX,           /* GPIO3 Pin 22 */
  IMXRT_PADMUX_GPIO_SD_B1_03_INDEX,           /* GPIO3 Pin 23 */

  IMXRT_PADMUX_GPIO_SD_B1_04_INDEX,           /* GPIO3 Pin 24 */
  IMXRT_PADMUX_GPIO_SD_B1_05_INDEX,           /* GPIO3 Pin 25 */
  IMXRT_PADMUX_GPIO_SD_B1_06_INDEX,           /* GPIO3 Pin 26 */
  IMXRT_PADMUX_GPIO_SD_B1_07_INDEX,           /* GPIO3 Pin 27 */
  IMXRT_PADMUX_GPIO_SD_B1_08_INDEX,           /* GPIO3 Pin 28 */
  IMXRT_PADMUX_GPIO_SD_B1_09_INDEX,           /* GPIO3 Pin 29 */
  IMXRT_PADMUX_GPIO_SD_B1_10_INDEX,           /* GPIO3 Pin 30 */
  IMXRT_PADMUX_GPIO_SD_B1_11_INDEX,           /* GPIO3 Pin 31 */
};

static const uint8_t g_gpio5_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_WAKEUP_INDEX,                  /* GPIO5 Pin 0 */
  IMXRT_PADMUX_PMIC_ON_REQ_INDEX,             /* GPIO5 Pin 1 */
  IMXRT_PADMUX_PMIC_STBY_REQ_INDEX,           /* GPIO5 Pin 2 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 3 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 4 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 5 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 6 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 7 */

  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 8 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 9 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 10 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 11 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 12 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 13 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 14 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 15 */

  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 16 */
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

static const uint8_t * const g_gpio_padmux[IMXRT_GPIO_NPORTS + 1] =
{
  g_gpio1_padmux,                             /* GPIO1 */
  g_gpio2_padmux,                             /* GPIO2 */
  g_gpio3_padmux,                             /* GPIO3 */
  NULL,                                       /* GPIO4 doesn't exist on 102x */
  g_gpio5_padmux,                             /* GPIO5 */
  NULL                                        /* End of list */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
