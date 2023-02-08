/****************************************************************************
 * arch/risc-v/src/hpm6750/hpm6750_clockconfig.c
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
#include "chip.h"
#include "hpm6750.h"
#include "hpm6750_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EXT_OSC 24000000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm6750_get_osc_freq
 ****************************************************************************/

uint32_t hpm6750_get_osc_freq(void)
{
  return EXT_OSC;
}

/****************************************************************************
 * Name: hpm6750_sysctl_config_clock
 ****************************************************************************/

static void hpm6750_sysctl_config_clock(clock_node_t node_index,
                                  clock_source_t source, uint32_t divide_by)
{
  uint32_t *clock_ptr = (uint32_t *)HPM6750_SYSCTL_CLOCK_NODE;
  uint32_t node = (uint32_t)node_index;
  uint32_t tmp;

  tmp = clock_ptr[node];
  tmp &= ~(SYSCTL_CLOCK_MUX_MASK | SYSCTL_CLOCK_DIV_MASK);
  tmp |= SYSCTL_CLOCK_MUX_SET(source);
  tmp |= SYSCTL_CLOCK_DIV_SET(divide_by - 1);
  clock_ptr[node] = tmp;

  while (clock_ptr[node] & SYSCTL_CLOCK_LOC_BUSY_MASK)
    ;
}

/****************************************************************************
 * Name: hpm6750_clockconfig
 ****************************************************************************/

void hpm6750_clockconfig(void)
{
  putreg32(0xffffffffu, HPM6750_SYSCTL_GROUP0_0);
  putreg32(0xffffffffu, HPM6750_SYSCTL_GROUP0_1);
  putreg32(0xffffffffu, HPM6750_SYSCTL_GROUP0_2);

  hpm6750_sysctl_config_clock(clock_node_cpu0, clock_source_pll0_clk0, 1);
  hpm6750_sysctl_config_clock(clock_node_mchtmr0, clock_source_osc0_clk0, 1);
  hpm6750_sysctl_config_clock(clock_node_uart0, clock_source_osc0_clk0, 1);
}
