/****************************************************************************
 * arch/risc-v/src/c906/c906_clockconfig.c
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

#include "riscv_internal.h"
#include "c906_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OSC_FREQ 20000000UL

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint64_t g_cpu_clock = OSC_FREQ;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: c906_get_cpuclk
 ****************************************************************************/

uint64_t c906_get_cpuclk(void)
{
  return g_cpu_clock;
}

/****************************************************************************
 * Name: c906_get_pll0clk
 ****************************************************************************/

#ifndef CONFIG_C906_WITH_QEMU
uint64_t c906_get_pll0clk(void)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: c906_clockconfig
 ****************************************************************************/

void c906_clockconfig(void)
{
#ifndef CONFIG_C906_WITH_QEMU

#endif
}
