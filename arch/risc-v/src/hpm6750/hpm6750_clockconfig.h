/****************************************************************************
 * arch/risc-v/src/hpm6750/hpm6750_clockconfig.h
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

#ifndef __ARCH_RISCV_SRC_HPM6750_HPM6750_CLOCKCONFIG_H
#define __ARCH_RISCV_SRC_HPM6750_HPM6750_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hpm6750_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @brief Clock nodes
 */

typedef enum
{
  clock_node_cpu0 = 0,
  clock_node_mchtmr0 = 1,
  clock_node_cpu1 = 2,
  clock_node_mchtmr1 = 3,
  clock_node_axi0 = 4,
  clock_node_axi1 = 5,
  clock_node_axi2 = 6,
  clock_node_ahb0 = 7,
  clock_node_femc = 8,
  clock_node_xpi0 = 9,
  clock_node_xpi1 = 10,
  clock_node_gptmr0 = 11,
  clock_node_gptmr1 = 12,
  clock_node_gptmr2 = 13,
  clock_node_gptmr3 = 14,
  clock_node_gptmr4 = 15,
  clock_node_gptmr5 = 16,
  clock_node_gptmr6 = 17,
  clock_node_gptmr7 = 18,
  clock_node_uart0 = 19,
  clock_node_uart1 = 20,
  clock_node_uart2 = 21,
  clock_node_uart3 = 22,
  clock_node_uart4 = 23,
  clock_node_uart5 = 24,
  clock_node_uart6 = 25,
  clock_node_uart7 = 26,
  clock_node_uart8 = 27,
  clock_node_uart9 = 28,
  clock_node_uarta = 29,
  clock_node_uartb = 30,
  clock_node_uartc = 31,
  clock_node_uartd = 32,
  clock_node_uarte = 33,
  clock_node_uartf = 34,
  clock_node_i2c0 = 35,
  clock_node_i2c1 = 36,
  clock_node_i2c2 = 37,
  clock_node_i2c3 = 38,
  clock_node_spi0 = 39,
  clock_node_spi1 = 40,
  clock_node_spi2 = 41,
  clock_node_spi3 = 42,
  clock_node_can0 = 43,
  clock_node_can1 = 44,
  clock_node_can2 = 45,
  clock_node_can3 = 46,
  clock_node_ptpc = 47,
  clock_node_ana0 = 48,
  clock_node_ana1 = 49,
  clock_node_ana2 = 50,
  clock_node_aud0 = 51,
  clock_node_aud1 = 52,
  clock_node_aud2 = 53,
  clock_node_dis0 = 54,
  clock_node_cam0 = 55,
  clock_node_cam1 = 56,
  clock_node_eth0 = 57,
  clock_node_eth1 = 58,
  clock_node_ptp0 = 59,
  clock_node_ptp1 = 60,
  clock_node_ref0 = 61,
  clock_node_ref1 = 62,
  clock_node_ntmr0 = 63,
  clock_node_ntmr1 = 64,
  clock_node_sdxc0 = 65,
  clock_node_sdxc1 = 66,
  clock_node_adc_i2s_start,
  clock_node_adc0 = clock_node_adc_i2s_start,
  clock_node_adc1,
  clock_node_adc2,
  clock_node_adc3,
  clock_node_i2s0,
  clock_node_i2s1,
  clock_node_i2s2,
  clock_node_i2s3,
  clock_node_end,
} clock_node_t;

/**
 * @brief General clock sources
 */

typedef enum
{
  clock_source_osc0_clk0 = 0,
  clock_source_pll0_clk0 = 1,
  clock_source_pll1_clk0 = 2,
  clock_source_pll1_clk1 = 3,
  clock_source_pll2_clk0 = 4,
  clock_source_pll2_clk1 = 5,
  clock_source_pll3_clk0 = 6,
  clock_source_pll4_clk0 = 7,
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

EXTERN uint32_t hpm6750_get_osc_freq(void);
EXTERN void hpm6750_clockconfig(void);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_HPM6750_HPM6750_CLOCKCONFIG_H */
