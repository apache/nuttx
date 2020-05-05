/****************************************************************************
 * arch/risc-v/src/k210/k210_clockconfig.c
 *
 * Derives from software originally provided by Canaan Inc
 *
 *   Copyright 2018 Canaan Inc
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "riscv_arch.h"
#include "k210_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OSC_FREQ 26000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_cpu_clock = 416000000;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k210_get_cpuclk
 ****************************************************************************/

uint32_t k210_get_cpuclk(void)
{
  return g_cpu_clock;
}

/****************************************************************************
 * Name: k210_get_pll0clk
 ****************************************************************************/

#ifndef CONFIG_K210_WITH_QEMU
uint32_t k210_get_pll0clk(void)
{
  uint32_t pll0;
  uint32_t nr;
  uint32_t nf;
  uint32_t od;

  pll0 = getreg32(K210_SYSCTL_PLL0);
  nr   = PLL_CLK_R(pll0)  + 1;
  nf   = PLL_CLK_F(pll0)  + 1;
  od   = PLL_CLK_OD(pll0) + 1;

  return OSC_FREQ / nr * nf / od;
}
#endif

/****************************************************************************
 * Name: k210_clockconfig
 ****************************************************************************/

void k210_clockconfig(void)
{
#ifndef CONFIG_K210_WITH_QEMU
  uint32_t clksel0;

  /* Obtain clock selector for ACLK */

  clksel0 = getreg32(K210_SYSCTL_CLKSEL0);

  if (1 == CLKSEL0_ACLK_SEL(clksel0))
    {
      /* PLL0 selected */

      g_cpu_clock = k210_get_pll0clk() / 2;
    }
  else
    {
      /* OSC selected */

      g_cpu_clock = OSC_FREQ;
    }
#endif
}
