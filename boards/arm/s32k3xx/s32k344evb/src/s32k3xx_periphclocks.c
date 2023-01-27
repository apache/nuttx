/****************************************************************************
 * boards/arm/s32k3xx/s32k344evb/src/s32k3xx_periphclocks.c
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

#include "s32k3xx_clocknames.h"
#include "s32k3xx_periphclocks.h"

#include "s32k344evb.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Each S32K3XX board must provide the following initialized structure.
 * This is needed to establish the initial peripheral clocking.
 */

const struct peripheral_clock_config_s g_peripheral_clockconfig0[] =
{
  {
    .clkname = LPI2C0_CLK,
#ifdef CONFIG_S32K3XX_LPI2C0
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPI2C1_CLK,
#ifdef CONFIG_S32K3XX_LPI2C1
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPSPI0_CLK,
#ifdef CONFIG_S32K3XX_LPSPI0
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPSPI1_CLK,
#ifdef CONFIG_S32K3XX_LPSPI1
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPSPI2_CLK,
#ifdef CONFIG_S32K3XX_LPSPI2
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPSPI3_CLK,
#ifdef CONFIG_S32K3XX_LPSPI3
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPSPI4_CLK,
#ifdef CONFIG_S32K3XX_LPSPI4
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPSPI5_CLK,
#ifdef CONFIG_S32K3XX_LPSPI5
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART0_CLK,
#ifdef CONFIG_S32K3XX_LPUART0
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART1_CLK,
#ifdef CONFIG_S32K3XX_LPUART1
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART2_CLK,
#ifdef CONFIG_S32K3XX_LPUART2
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART3_CLK,
#ifdef CONFIG_S32K3XX_LPUART3
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART4_CLK,
#ifdef CONFIG_S32K3XX_LPUART4
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART5_CLK,
#ifdef CONFIG_S32K3XX_LPUART5
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART6_CLK,
#ifdef CONFIG_S32K3XX_LPUART6
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART7_CLK,
#ifdef CONFIG_S32K3XX_LPUART7
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART8_CLK,
#ifdef CONFIG_S32K3XX_LPUART8
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART9_CLK,
#ifdef CONFIG_S32K3XX_LPUART9
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART10_CLK,
#ifdef CONFIG_S32K3XX_LPUART10
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART11_CLK,
#ifdef CONFIG_S32K3XX_LPUART11
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART12_CLK,
#ifdef CONFIG_S32K3XX_LPUART12
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART13_CLK,
#ifdef CONFIG_S32K3XX_LPUART13
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART14_CLK,
#ifdef CONFIG_S32K3XX_LPUART14
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPUART15_CLK,
#ifdef CONFIG_S32K3XX_LPUART15
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = WKPU_CLK,
#ifdef CONFIG_S32K3XX_WKPUINTS
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
};

unsigned int const num_of_peripheral_clocks_0 =
    sizeof(g_peripheral_clockconfig0) /
    sizeof(g_peripheral_clockconfig0[0]);

/****************************************************************************
 * Public Functions
 ****************************************************************************/
