/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/nxxx_clock.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_CCM_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_CCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/nxxx_memorymap.h"

#if defined(CONFIG_ARCH_CHIP_N236)
#  include "hardware/n236/n236_clock.h"
#else
#  error Unrecognized NXXx architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Peripheral clocks are controlled by SYSCON0.sel and SYSCON0.div */

#define PERIPH_CLOCK(_mux, _div) \
  (struct clock_regs_s)          \
  {                              \
    .mux = (_mux),               \
    .div = (_div),               \
  }

/* Clock gates are in SYSCON0.ahb_clk_ctrlX, one bit per clock source */

#define CLOCK_GATE(_reg, _bit) \
  (struct clock_gate_reg_s)    \
  {                            \
    .reg = (_reg),             \
    .bit = (_bit),             \
  }

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct clock_gate_reg_s
{
  uint32_t reg;
  uint32_t bit;
};

struct clock_regs_s
{
  uint32_t mux;
  uint32_t div;
};

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_CCM_H_ */
