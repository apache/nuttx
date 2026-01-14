/****************************************************************************
 * arch/risc-v/src/hpm6000/hpm_clockconfig.h
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

#ifndef __ARCH_RISCV_SRC_HPM6000_HPM_CLOCKCONFIG_H
#define __ARCH_RISCV_SRC_HPM6000_HPM_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hpm_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_DIV 1
#define AXI_SUB_DIV 3
#define AHB_SUB_DIV 3
#define PLL1_DIV 1
#define PLL1_FREQ 576000000
#define AUD_DIV 46

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @brief Clock nodes
 */

typedef enum
{
  clock_node_mchtmr0 = 0,
  clock_node_femc = 1,
  clock_node_xpi0 = 2,
  clock_node_xpi1 = 3,
  clock_node_gptmr0 = 4,
  clock_node_gptmr1 = 5,
  clock_node_gptmr2 = 6,
  clock_node_gptmr3 = 7,
  clock_node_uart0 = 8,
  clock_node_uart1 = 9,
  clock_node_uart2 = 10,
  clock_node_uart3 = 11,
  clock_node_uart4 = 12,
  clock_node_uart5 = 13,
  clock_node_uart6 = 14,
  clock_node_uart7 = 15,
  clock_node_i2c0 = 16,
  clock_node_i2c1 = 17,
  clock_node_i2c2 = 18,
  clock_node_i2c3 = 19,
  clock_node_spi0 = 20,
  clock_node_spi1 = 21,
  clock_node_spi2 = 22,
  clock_node_spi3 = 23,
  clock_node_can0 = 24,
  clock_node_can1 = 25,
  clock_node_ptpc = 26,
  clock_node_ana0 = 27,
  clock_node_ana1 = 28,
  clock_node_ana2 = 29,
  clock_node_ana3 = 30,
  clock_node_aud0 = 31,
  clock_node_aud1 = 32,
  clock_node_eth0 = 33,
  clock_node_ptp0 = 34,
  clock_node_ref0 = 35,
  clock_node_ref1 = 36,
  clock_node_ntmr0 = 37,
  clock_node_sdxc0 = 38,

  clock_node_adc_i2s_start,
  clock_node_adc0 = clock_node_adc_i2s_start,
  clock_node_adc1,
  clock_node_adc2,

  clock_node_i2s0,
  clock_node_i2s1,

  clock_node_end,

  clock_node_core_start = 0xfc,
  clock_node_cpu0 = clock_node_core_start,
  clock_node_axi,
  clock_node_ahb,
} clock_node_t;

/**
 * @brief General clock sources
 */

typedef enum
{
  clock_source_osc0_clk0 = 0,
  clock_source_pll0_clk0 = 1,
  clock_source_pll0_clk1 = 2,
  clock_source_pll0_clk2 = 3,
  clock_source_pll1_clk0 = 4,
  clock_source_pll1_clk1 = 5,
  clock_source_pll2_clk0 = 6,
  clock_source_pll2_clk1 = 7,
  clock_source_general_source_end,
} clock_source_t;

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN uint32_t hpm_get_osc_freq(void);
EXTERN void hpm_clockconfig(void);
void hpm_uart_clockconfig(void);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_HPM6000_HPM_CLOCKCONFIG_H */
