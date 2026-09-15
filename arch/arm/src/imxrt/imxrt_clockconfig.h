/****************************************************************************
 * arch/arm/src/imxrt/imxrt_clockconfig.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* IMXRT117X Clock pheripheral is different from IMXRT10XX
 * hence we use a VER2 driver for clockconfig
 */

#ifdef CONFIG_ARCH_FAMILY_IMXRT118x

#include <stdbool.h>
#include <stdint.h>

enum imxrt_clock_source_e
{
  IMXRT_CLK_OSC_RC24M = 0,
  IMXRT_CLK_OSC_RC400M,
  IMXRT_CLK_OSC24M,
  IMXRT_CLK_ARM_PLL,
  IMXRT_CLK_SYS_PLL1,
  IMXRT_CLK_SYS_PLL1_DIV2,
  IMXRT_CLK_SYS_PLL1_DIV5,
  IMXRT_CLK_SYS_PLL2,
  IMXRT_CLK_SYS_PLL2_PFD0,
  IMXRT_CLK_SYS_PLL2_PFD1,
  IMXRT_CLK_SYS_PLL2_PFD2,
  IMXRT_CLK_SYS_PLL2_PFD3,
  IMXRT_CLK_SYS_PLL3,
  IMXRT_CLK_SYS_PLL3_DIV2,
  IMXRT_CLK_SYS_PLL3_PFD0,
  IMXRT_CLK_SYS_PLL3_PFD1,
  IMXRT_CLK_SYS_PLL3_PFD2,
  IMXRT_CLK_SYS_PLL3_PFD3,
  IMXRT_CLK_AUDIO_PLL,
  IMXRT_CLK_SOURCE_COUNT
};

enum imxrt_pll_e
{
  IMXRT_PLL_ARM = 0,
  IMXRT_PLL_SYS1,
  IMXRT_PLL_SYS2,
  IMXRT_PLL_SYS3,
  IMXRT_PLL_AUDIO
};

#define IMXRT_CLOCK_STATUS_ELE_READY       (1u << 0)
#define IMXRT_CLOCK_STATUS_TRDC_AON         (1u << 1)
#define IMXRT_CLOCK_STATUS_TRDC_MEGA        (1u << 2)
#define IMXRT_CLOCK_STATUS_TRDC_WAKEUP      (1u << 3)
#define IMXRT_CLOCK_STATUS_PLL_LDO          (1u << 4)
#define IMXRT_CLOCK_STATUS_SYS_PLL3         (1u << 5)
#define IMXRT_CLOCK_STATUS_ROOTS_CONFIGURED (1u << 6)
#define IMXRT_CLOCK_STATUS_SYS_PLL1         (1u << 7)
#define IMXRT_CLOCK_STATUS_NETC             (1u << 8)

extern volatile uint32_t g_imxrt118x_clock_status;
extern volatile int32_t g_imxrt118x_clock_error;

void imxrt_clockconfig(void);
int imxrt_clockroot_configure(unsigned int root, unsigned int mux,
                              unsigned int divider, bool enable);
int imxrt_clockroot_frequency(unsigned int root, uint32_t *frequency);
uint32_t imxrt_clocksource_frequency(enum imxrt_clock_source_e source);
uint32_t imxrt_pll_frequency(enum imxrt_pll_e pll);
void imxrt_clockgate_configure(unsigned int gate, bool enable);
#ifdef CONFIG_IMXRT_NETC
int imxrt_netc_clocks_configure(void);
#endif
#elif defined(CONFIG_IMXRT_CLOCKCONFIG_VER2)
#include "imxrt_clockconfig_ver2.h"
#else
#include "imxrt_clockconfig_ver1.h"
#endif

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_CLOCKCONFIG_H */
