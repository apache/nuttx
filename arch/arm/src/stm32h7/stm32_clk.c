/****************************************************************************
 * arch/arm/src/stm32h7/stm32_rcc.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/param.h>
#include <arch/board/board.h>
#include <assert.h>

#include "arm_internal.h"
#include "stm32_clk.h"
#include "hardware/stm32h7x3xx_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helper macro to define a gate configuration (for nested use) */

#define STM32_GATE(_name, _reg, _bit, _flags)                                \
  { .gate_name = _name,                                                      \
    .gate = {                                                                \
      .reg = _reg,                                                           \
      .bit_idx = _bit,                                                       \
      .flags = _flags                                                        \
    }                                                                        \
  }

/* Helper macro to define a divider configuration (for nested use) */

#define STM32_DIVIDER(_name, _reg, _shift, _width, _flags)                   \
  { .div_name = _name,                                                       \
    .div = {                                                                 \
      .reg = _reg,                                                           \
      .shift = _shift,                                                       \
      .width = _width,                                                       \
      .flags = _flags                                                        \
    }                                                                        \
  }

/* Helper macro to define a mux configuration (for nested use) */

#define STM32_MUX(_name, _reg, _shift, _width, _flags)                       \
  { .mux_name = _name,                                                       \
    .mux = {                                                                 \
      .reg = _reg,                                                           \
      .shift = _shift,                                                       \
      .width = _width,                                                       \
      .flags = _flags                                                        \
    }                                                                        \
  }

/* Helper macro to define a fixed rate configuration (for nested use) */

#define STM32_FIXED_RATE(_name, _rate, _flags)                               \
  {                                                                          \
    .fixed_rate_name = _name,                                                \
    .fixed_rate = {                                                          \
      .fixed_rate = _rate,                                                   \
      .flags = _flags,                                                       \
    }                                                                        \
  }

/* Helper macro to define a multiplier configuration (for nested use) */

#define STM32_MULTIPLIER(_name, _reg, _shift, _width, _flags)                \
  {                                                                          \
    .mult_name = _name,                                                      \
    .mult = {                                                                \
      .reg = _reg,                                                           \
      .shift = _shift,                                                       \
      .width = _width,                                                       \
      .flags = _flags                                                        \
    }                                                                        \
  }

/* Macro to define N-gated fixed rate oscillators */

#define STM32_GATED_FIXED_RATE_N(_fixed_rate, _flags, _num_gates, ...)        \
{                                                                            \
  .num_gates = _num_gates,                                                   \
  .base_flags = _flags,                                                      \
  .fixed_rate = _fixed_rate,                                                 \
  .gates = (const struct stm32_gate_s []) { __VA_ARGS__ },                   \
}

/* Macro to define N-gated mux with variadic gates */

#define STM32_GATED_MUX_N(_parents, _parents_n, _mux, _flags, _num_gates, ...)\
{                                                                            \
  .parent_names = _parents,                                                  \
  .parent_names_count = _parents_n,                                          \
  .num_gates = _num_gates,                                                   \
  .base_flags = _flags,                                                      \
  .mux = _mux,                                                               \
  .gates = (const struct stm32_gate_s []) { __VA_ARGS__ }                    \
}

/* Macro to define PLL configuration */

#define STM32_PLL(_parent, _flags, _divn, _divp, _divq, _divr,               \
                  _gatep, _gateq, _gater)                                    \
{                                                                            \
  .parent_name = _parent,                                                    \
  .base_flags = _flags,                                                      \
  .divn = _divn,                                                             \
  .divp = _divp,                                                             \
  .divq = _divq,                                                             \
  .divr = _divr,                                                             \
  .gatep = _gatep,                                                           \
  .gateq = _gateq,                                                           \
  .gater = _gater,                                                           \
}

/* Macro to define divided mux with multiple dividers */

#define STM32_DIVIDED_MUX_N(_parents, _parents_n, _mux, _flags, _num_divs, ...)\
{                                                                            \
  .parent_names = _parents,                                                  \
  .parent_names_count = _parents_n,                                          \
  .num_divs = _num_divs,                                                     \
  .base_flags = _flags,                                                      \
  .mux = _mux,                                                               \
  .divs = (const struct stm32_divider_s []) { __VA_ARGS__ }                  \
}

/* Macro to define system clock */

#define STM32_SYSCLK(_sysclk_parents, _sysclk_parents_n, _sysclk_mux,        \
                     _traceclk_parents, _traceclk_parents_n, _traceclk_mux,  \
                     _flags, _cdcpre, _hpre, _cdppre, _cdppre1, _cdppre2,    \
                     _sdrppre)                                               \
{                                                                            \
  .sysclk_parent_names = _sysclk_parents,                                    \
  .sysclk_parent_names_count = _sysclk_parents_n,                            \
  .traceclk_parent_names = _traceclk_parents,                                \
  .traceclk_parent_names_count = _traceclk_parents_n,                        \
  .base_flags = _flags,                                                      \
  .sysclk_mux = _sysclk_mux,                                                 \
  .traceclk_mux = _traceclk_mux,                                             \
  .cdcpre = _cdcpre,                                                         \
  .hpre = _hpre,                                                             \
  .cdppre = _cdppre,                                                         \
  .cdppre1 = _cdppre1,                                                       \
  .cdppre2 = _cdppre2,                                                       \
  .sdrppre = _sdrppre,                                                       \
}

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_gate_s
{
  const char *gate_name;
  struct clk_gate_s gate;
};

struct stm32_fixed_rate_s
{
  const char *fixed_rate_name;
  struct clk_fixed_rate_s fixed_rate;
};

struct stm32_multiplier_s
{
  const char *mult_name;
  struct clk_multiplier_s mult;
};

struct stm32_divider_s
{
  const char *div_name;
  struct clk_divider_s div;
};

struct stm32_mux_s
{
  const char *mux_name;
  struct clk_mux_s mux;
};

/* Oscillators configuration */

struct stm32_gated_fixed_rate_s
{
  const uint8_t num_gates;
  const uint8_t base_flags;

  const struct stm32_gate_s *gates;
  struct stm32_fixed_rate_s fixed_rate;
};

/* PLL configuration */

struct stm32_pll_s
{
  const char *parent_name;

  const uint8_t base_flags;

  struct stm32_multiplier_s divn;
  struct stm32_divider_s divp;
  struct stm32_divider_s divq;
  struct stm32_divider_s divr;
  struct stm32_gate_s gatep;
  struct stm32_gate_s gateq;
  struct stm32_gate_s gater;
};

struct stm32_sysclk_s
{
  const char * const *sysclk_parent_names;
  const int sysclk_parent_names_count;
  const char * const *traceclk_parent_names;
  const int traceclk_parent_names_count;

  const uint8_t base_flags;

  struct stm32_mux_s sysclk_mux;
  struct stm32_mux_s traceclk_mux;
  struct stm32_divider_s cdcpre;
  struct stm32_divider_s hpre;
  struct stm32_divider_s cdppre;
  struct stm32_divider_s cdppre1;
  struct stm32_divider_s cdppre2;
  struct stm32_divider_s sdrppre;
};

struct stm32_gated_mux_s
{
  const char * const *parent_names;
  const int parent_names_count;

  const char *mux_name;

  const uint8_t num_gates;
  const uint8_t base_flags;

  const struct stm32_gate_s *gates;
  struct stm32_mux_s mux;
};

/* Divided mux with multiple dividers configuration */

struct stm32_divided_mux_n_s
{
  const char * const *parent_names;
  const int parent_names_count;

  const uint8_t num_divs;
  const uint8_t base_flags;

  const struct stm32_divider_s *divs;
  struct stm32_mux_s mux;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* STM32H7 gated oscillators */

static const struct stm32_gated_fixed_rate_s stm32h7_gated_oscs[] =
{
  STM32_GATED_FIXED_RATE_N(
    STM32_FIXED_RATE("csi", STM32_CSI_FREQUENCY, 0),
    CLK_IS_CRITICAL | CLK_NAME_IS_STATIC,
    2,
    STM32_GATE("csi_ck", STM32_RCC_CR, 7, 0),
    STM32_GATE("csi_ker_ck", STM32_RCC_CR, 9, 0)
  ),
  STM32_GATED_FIXED_RATE_N(
    STM32_FIXED_RATE("hse", STM32_HSE_FREQUENCY, 0),
    CLK_IS_CRITICAL | CLK_NAME_IS_STATIC,
    1,
    STM32_GATE("hse_ck", STM32_RCC_CR, 16, 0)
  ),
  STM32_GATED_FIXED_RATE_N(
    STM32_FIXED_RATE("hsi48", STM32_HSI48_FREQUENCY, 0),
    CLK_IS_CRITICAL | CLK_NAME_IS_STATIC,
    1,
    STM32_GATE("hsi48_ck", STM32_RCC_CR, 12, 0)
  ),
  STM32_GATED_FIXED_RATE_N(
    STM32_FIXED_RATE("lsi", STM32_LSI_FREQUENCY, 0),
    CLK_IS_CRITICAL | CLK_NAME_IS_STATIC,
    1,
    STM32_GATE("lsi_ck", STM32_RCC_CSR, 0, 0)
  ),
  STM32_GATED_FIXED_RATE_N(
    STM32_FIXED_RATE("hsi", STM32_HSI_FREQUENCY, 0),
    CLK_IS_CRITICAL | CLK_NAME_IS_STATIC,
    0
  ),
  STM32_GATED_FIXED_RATE_N(
    STM32_FIXED_RATE("lse", STM32_LSE_FREQUENCY, 0),
    CLK_IS_CRITICAL | CLK_NAME_IS_STATIC,
    1,
    STM32_GATE("lse_ck", STM32_RCC_BDCR, 0, 0)
  ),
};

/* STM32H7 PLL clock selection parent names */

static const char * const pll_cksel_parents[] =
{
  "hsi_ck", "csi_ck", "hse_ck", "off"
};

/* STM32H7 PLL clock selection */

static const struct stm32_divided_mux_n_s stm32h7_pll_cksel =
  STM32_DIVIDED_MUX_N(
    pll_cksel_parents,
    nitems(pll_cksel_parents),
    STM32_MUX(
      "pllsrc",
      STM32_RCC_PLLCKSELR,
      RCC_PLLCKSELR_PLLSRC_SHIFT,
      2,
      CLK_MUX_SET_RATE_NO_REPARENT
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
    3,
    STM32_DIVIDER(
      "divm1",
      STM32_RCC_PLLCKSELR,
      RCC_PLLCKSELR_DIVM1_SHIFT,
      6,
      CLK_DIVIDER_ONE_BASED
    ),
    STM32_DIVIDER(
      "divm2",
      STM32_RCC_PLLCKSELR,
      RCC_PLLCKSELR_DIVM2_SHIFT,
      6,
      CLK_DIVIDER_ONE_BASED
    ),
    STM32_DIVIDER(
      "divm3",
      STM32_RCC_PLLCKSELR,
      RCC_PLLCKSELR_DIVM3_SHIFT,
      6,
      CLK_DIVIDER_ONE_BASED
    )
  );

/* STM32H7 PLLs */

static const struct stm32_pll_s stm32h7_plls[] =
{
  STM32_PLL(
    "divm1",
    CLK_NAME_IS_STATIC  | CLK_PARENT_NAME_IS_STATIC | 
    CLK_SET_RATE_PARENT | CLK_GET_RATE_NOCACHE,
    STM32_MULTIPLIER(
      "divn1",
      STM32_RCC_PLL1DIVR,
      RCC_PLLDIVR_N_SHIFT,
      9,
      CLK_MULT_ONE_BASED | CLK_MULT_ROUND_CLOSEST
    ),
    STM32_DIVIDER(
      "divp1",
      STM32_RCC_PLL1DIVR,
      RCC_PLLDIVR_P_SHIFT,
      7,
      CLK_DIVIDER_DIV_NEED_EVEN | CLK_DIVIDER_ROUND_CLOSEST
    ),
    STM32_DIVIDER(
      "divq1",
      STM32_RCC_PLL1DIVR,
      RCC_PLLDIVR_Q_SHIFT,
      7,
      CLK_DIVIDER_ROUND_CLOSEST
    ),
    STM32_DIVIDER(
      "divr1",
      STM32_RCC_PLL1DIVR,
      RCC_PLLDIVR_R_SHIFT,
      7,
      CLK_DIVIDER_ROUND_CLOSEST
    ),
    STM32_GATE(
      "pll1_p_ck",
      STM32_RCC_PLLCFGR,
      RCC_PLLCFGR_DIVP1EN_SHIFT,
      0
    ),
    STM32_GATE(
      "pll1_q_ck",
      STM32_RCC_PLLCFGR,
      RCC_PLLCFGR_DIVQ1EN_SHIFT,
      0
    ),
    STM32_GATE(
      "pll1_r_ck",
      STM32_RCC_PLLCFGR,
      RCC_PLLCFGR_DIVR1EN_SHIFT,
      0
    )
  ),
  STM32_PLL(
    "divm2",
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT | CLK_GET_RATE_NOCACHE,
    STM32_MULTIPLIER(
      "divn2",
      STM32_RCC_PLL2DIVR,
      RCC_PLLDIVR_N_SHIFT,
      9,
      CLK_MULT_ONE_BASED | CLK_MULT_ROUND_CLOSEST
    ),
    STM32_DIVIDER(
      "divp2",
      STM32_RCC_PLL2DIVR,
      RCC_PLLDIVR_P_SHIFT,
      7,
      CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ROUND_CLOSEST
    ),
    STM32_DIVIDER(
      "divq2",
      STM32_RCC_PLL2DIVR,
      RCC_PLLDIVR_Q_SHIFT,
      7,
      CLK_DIVIDER_ROUND_CLOSEST
    ),
    STM32_DIVIDER(
      "divr2",
      STM32_RCC_PLL2DIVR,
      RCC_PLLDIVR_R_SHIFT,
      7,
      CLK_DIVIDER_ROUND_CLOSEST
    ),
    STM32_GATE(
      "pll2_p_ck",
      STM32_RCC_PLLCFGR,
      RCC_PLLCFGR_DIVP2EN_SHIFT,
      0
    ),
    STM32_GATE(
      "pll2_q_ck",
      STM32_RCC_PLLCFGR,
      RCC_PLLCFGR_DIVQ2EN_SHIFT,
      0
    ),
    STM32_GATE(
      "pll2_r_ck",
      STM32_RCC_PLLCFGR,
      RCC_PLLCFGR_DIVR2EN_SHIFT,
      0
    )
  ),
  STM32_PLL(
    "divm3",
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT | CLK_GET_RATE_NOCACHE,
    STM32_MULTIPLIER(
      "divn3",
      STM32_RCC_PLL3DIVR,
      RCC_PLLDIVR_N_SHIFT,
      9,
      CLK_MULT_ONE_BASED | CLK_MULT_ROUND_CLOSEST
    ),
    STM32_DIVIDER(
      "divp3",
      STM32_RCC_PLL3DIVR,
      RCC_PLLDIVR_P_SHIFT,
      7,
      CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ROUND_CLOSEST
    ),
    STM32_DIVIDER(
      "divq3",
      STM32_RCC_PLL3DIVR,
      RCC_PLLDIVR_Q_SHIFT,
      7,
      CLK_DIVIDER_ROUND_CLOSEST
    ),
    STM32_DIVIDER(
      "divr3",
      STM32_RCC_PLL3DIVR,
      RCC_PLLDIVR_R_SHIFT,
      7,
      CLK_DIVIDER_ROUND_CLOSEST
    ),
    STM32_GATE(
      "pll3_p_ck",
      STM32_RCC_PLLCFGR,
      RCC_PLLCFGR_DIVP3EN_SHIFT,
      0
    ),
    STM32_GATE(
      "pll3_q_ck",
      STM32_RCC_PLLCFGR,
      RCC_PLLCFGR_DIVQ3EN_SHIFT,
      0
    ),
    STM32_GATE(
      "pll3_r_ck",
      STM32_RCC_PLLCFGR,
      RCC_PLLCFGR_DIVR3EN_SHIFT,
      0
    )
  ),
};

/* STM32H7 system clock parent names */

static const char * const sysclk_parents[] =
{
  "hsi_ck", "csi_ck", "hse_ck", "pll1_p_ck"
};

static const char * const traceclk_parents[] =
{
  "hsi_ck", "csi_ck", "hse_ck", "pll1_r_ck"
};

/* STM32H7 system clock */

static const struct stm32_sysclk_s stm32h7_sysclk =
  STM32_SYSCLK(
    sysclk_parents,
    nitems(sysclk_parents),
    STM32_MUX(
      "sys_ck",
      STM32_RCC_CFGR,
      RCC_CFGR_SW_SHIFT,
      3,
      CLK_MUX_SET_RATE_NO_REPARENT
    ),
    traceclk_parents,
    nitems(traceclk_parents),
    STM32_MUX(
      "traceportck",
      STM32_RCC_CFGR,
      RCC_CFGR_SW_SHIFT,
      1,
      CLK_MUX_SET_RATE_NO_REPARENT
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT | CLK_GET_RATE_NOCACHE,
    STM32_DIVIDER(
      "cdcpre",
      STM32_RCC_D1CFGR,
      RCC_D1CFGR_D1CPRE_SHIFT,
      4,
      CLK_DIVIDER_POWER_OF_TWO | (8 << CLK_DIVIDER_MINDIV_OFF) |
      CLK_DIVIDER_APPLY_OFFSET | CLK_DIVIDER_ROUND_CLOSEST
    ),
    STM32_DIVIDER(
      "hpre",
      STM32_RCC_D1CFGR,
      RCC_D1CFGR_HPRE_SHIFT,
      4,
      CLK_DIVIDER_POWER_OF_TWO | (8 << CLK_DIVIDER_MINDIV_OFF) |
      CLK_DIVIDER_APPLY_OFFSET | CLK_DIVIDER_ROUND_CLOSEST
    ),
    STM32_DIVIDER(
      "cdppre",
      STM32_RCC_D1CFGR,
      RCC_D1CFGR_D1PPRE_SHIFT,
      3,
      CLK_DIVIDER_POWER_OF_TWO | (4 << CLK_DIVIDER_MINDIV_OFF) |
      CLK_DIVIDER_APPLY_OFFSET | CLK_DIVIDER_ROUND_CLOSEST
    ),
    STM32_DIVIDER(
      "cdppre1",
      STM32_RCC_D2CFGR,
      RCC_D2CFGR_D2PPRE1_SHIFT,
      3,
      CLK_DIVIDER_POWER_OF_TWO | (4 << CLK_DIVIDER_MINDIV_OFF) |
      CLK_DIVIDER_APPLY_OFFSET | CLK_DIVIDER_ROUND_CLOSEST
    ),
    STM32_DIVIDER(
      "cdppre2",
      STM32_RCC_D2CFGR,
      RCC_D2CFGR_D2PPRE2_SHIFT,
      3,
      CLK_DIVIDER_POWER_OF_TWO | (4 << CLK_DIVIDER_MINDIV_OFF) |
      CLK_DIVIDER_APPLY_OFFSET | CLK_DIVIDER_ROUND_CLOSEST
    ),
    STM32_DIVIDER(
      "sdrppre",
      STM32_RCC_D3CFGR,
      RCC_D3CFGR_D3PPRE_SHIFT,
      3,
      CLK_DIVIDER_POWER_OF_TWO | (4 << CLK_DIVIDER_MINDIV_OFF) |
      CLK_DIVIDER_APPLY_OFFSET | CLK_DIVIDER_ROUND_CLOSEST
    )
  );

/* STM32H7 gated muxes parent names */

static const char * const sdmmcsel_parents[] =
{
  "pll1_q_ck", "pll2_r_ck"
};

static const char * const qspisel_parents[] =
{
  "hpre", "pll1_q_ck", "pll2_r_ck", "per_ck"
};

static const char * const ckpersel_parents[] =
{
  "hsi_ker_ck", "csi_ker_ck", "hse_ck"
};

static const char * const fmcsel_parents[] =
{
  "hpre", "pll1_q_ck", "pll2_r_ck", "per_ck"
};

static const char * const swpsel_parents[] =
{
  "cdppre1", "hsi_ker_ck"
};

static const char * const fdcansel_parents[] =
{
  "hse_ck", "pll1_q_ck", "pll2_q_ck"
};

static const char * const dfsdm1sel_parents[] =
{
  "cdppre2", "sysclk"
};

static const char * const spdifsel_parents[] =
{
  "pll1_q_ck", "pll2_r_ck", "pll3_r_ck", "hsi_ker_ck"
};

static const char * const spi45sel_parents[] =
{
  "cdppre2", "pll2_r_ck", "pll3_r_ck",
  "hsi_ker_ck", "csi_ker_ck", "hse_ck"
};

static const char * const spi123sel_parents[] =
{
  "pll1_q_ck", "pll2_p_ck", "pll3_p_ck",
  "i2s_ckin", "per_ck"
};

static const char * const sai23sel_parents[] =
{
  "pll1_q_ck", "pll2_p_ck", "pll3_p_ck",
  "i2s_ckin", "per_ck"
};

static const char * const sai1sel_parents[] =
{
  "pll1_q_ck", "pll2_p_ck", "pll3_p_ck",
  "i2s_ckin", "per_ck"
};

static const char * const lptim1sel_parents[] =
{
  "cdppre1", "pll2_p_ck", "pll3_r_ck",
  "lse_ck", "lsi_ck", "per_ck"
};

static const char * const cecsel_parents[] =
{
  "lse_ck", "lsi_ck", "csi_ker_ck"
};

static const char * const usbsel_parents[] =
{
  "pll1_q_ck", "pll3_q_ck", "hsi48_ck"
};

static const char * const i2c123sel_parents[] =
{
  "cdppre1", "pll3_r_ck", "hsi_ker_ck", "csi_ker_ck"
};

static const char * const rngsel_parents[] =
{
  "hsi48_ck", "pll1_q_ck", "lse_ck", "lsi_ck"
};

static const char * const usart16sel_parents[] =
{
  "cdppre2", "pll2_r_ck", "pll3_r_ck",
  "hsi_ker_ck", "csi_ker_ck", "lse_ck"
};

static const char * const usart234578sel_parents[] =
{
  "cdppre1", "pll2_r_ck", "pll3_r_ck",
  "hsi_ker_ck", "csi_ker_ck", "lse_ck"
};

static const char * const spi6sel_parents[] =
{
  "sdrppre", "pll2_r_ck", "pll3_r_ck",
  "hsi_ker_ck", "csi_ker_ck", "hse_ck"
};

static const char * const sai4bsel_parents[] =
{
  "pll1_q_ck", "pll2_p_ck", "pll3_p_ck",
  "i2s_ckin", "per_ck"
};

static const char * const sai4asel_parents[] =
{
  "pll1_q_ck", "pll2_p_ck", "pll3_p_ck",
  "i2s_ckin", "per_ck"
};

static const char * const adcsel_parents[] =
{
  "pll2_p_ck", "pll3_r_ck", "per_ck"
};

static const char * const lptim345sel_parents[] =
{
  "sdrppre", "pll2_p_ck", "pll3_r_ck",
  "lse_ck", "lsi_ck", "per_ck"
};

static const char * const lptim2sel_parents[] =
{
  "sdrppre", "pll2_p_ck", "pll3_r_ck",
  "lse_ck", "lsi_ck", "per_ck"
};

static const char * const i2c4sel_parents[] =
{
  "sdrppre", "pll3_r_ck", "hsi_ker_ck", "csi_ker_ck"
};

static const char * const lpuart1sel_parents[] =
{
  "sdrppre", "pll2_r_ck", "pll3_r_ck",
  "hsi_ker_ck", "csi_ker_ck", "lse_ck"
};

/* STM32H7 gated muxes */

static const struct stm32_gated_mux_s stm32h7_gated_muxes[] =
{
  STM32_GATED_MUX_N(
    ckpersel_parents, nitems(ckpersel_parents),
    STM32_MUX(
        "per_ck", STM32_RCC_D1CCIPR, RCC_D1CCIPR_CKPERSEL_SHIFT, 2,
        CLK_MUX_SET_RATE_NO_REPARENT
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    0
  ),
  STM32_GATED_MUX_N(
    sdmmcsel_parents, nitems(sdmmcsel_parents),
    STM32_MUX(
      "sdmmcsel",
      STM32_RCC_D1CCIPR,
      RCC_D1CCIPR_SDMMC_SHIFT,
      1,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    2,
    STM32_GATE("sdmmc1_ker_ck", STM32_RCC_AHB3ENR, 16, 0),
    STM32_GATE("sdmmc2_ker_ck", STM32_RCC_AHB2ENR, 9, 0)
  ),
  STM32_GATED_MUX_N(
    qspisel_parents, nitems(qspisel_parents),
    STM32_MUX(
      "qspisel",
      STM32_RCC_D1CCIPR,
      RCC_D1CCIPR_QSPISEL_SHIFT,
      2,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("quadspi_ker_ck", STM32_RCC_AHB3ENR, 14, 0)
  ),
  STM32_GATED_MUX_N(
    fmcsel_parents, nitems(fmcsel_parents),
    STM32_MUX(
      "fmcsel",
      STM32_RCC_D1CCIPR,
      RCC_D1CCIPR_FMCSEL_SHIFT,
      2,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("fmc_ker_ck", STM32_RCC_AHB3ENR, 12, 0)
  ),
  STM32_GATED_MUX_N(
    swpsel_parents, nitems(swpsel_parents),
    STM32_MUX(
      "swpsel",
      STM32_RCC_D2CCIP1R,
      RCC_D2CCIP1R_SWPSEL_SHIFT,
      1,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("swpmi_ker_ck", STM32_RCC_APB3ENR, 2, 0)
  ),
  STM32_GATED_MUX_N(
    fdcansel_parents, nitems(fdcansel_parents),
    STM32_MUX(
      "fdcansel",
      STM32_RCC_D2CCIP1R,
      RCC_D2CCIP1R_FDCANSEL_SHIFT,
      2,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("fdcan_ker_ck", STM32_RCC_APB1HENR, 8, 0)
  ),
  STM32_GATED_MUX_N(
    dfsdm1sel_parents, nitems(dfsdm1sel_parents),
    STM32_MUX(
      "dfsdm1sel",
      STM32_RCC_D2CCIP1R,
      RCC_D2CCIP1R_DFSDM1SEL_SHIFT,
      1,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("dfsdm1_ker_ck", STM32_RCC_APB2ENR, 28, 0)
  ),
  STM32_GATED_MUX_N(
    spdifsel_parents, nitems(spdifsel_parents),
    STM32_MUX(
      "spdifsel",
      STM32_RCC_D2CCIP1R,
      RCC_D2CCIP1R_SPDIFSEL_SHIFT,
      2,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("spdifrx_ker_ck", STM32_RCC_APB1LENR, 16, 0)
  ),
  STM32_GATED_MUX_N(
    spi45sel_parents, nitems(spi45sel_parents),
    STM32_MUX(
      "spi45sel",
      STM32_RCC_D2CCIP1R,
      RCC_D2CCIP1R_SPI45SEL_SHIFT,
      3,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    2,
    STM32_GATE("spi4_ker_ck", STM32_RCC_APB2ENR, 13, 0),
    STM32_GATE("spi5_ker_ck", STM32_RCC_APB2ENR, 20, 0)
  ),
  STM32_GATED_MUX_N(
    spi123sel_parents, nitems(spi123sel_parents),
    STM32_MUX(
      "spi123sel",
      STM32_RCC_D2CCIP1R,
      RCC_D2CCIP1R_SPI123SEL_SHIFT,
      3,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    3,
    STM32_GATE("spi1_ker_ck", STM32_RCC_APB2ENR, 12, 0),
    STM32_GATE("spi2_ker_ck", STM32_RCC_APB1LENR, 14, 0),
    STM32_GATE("spi3_ker_ck", STM32_RCC_APB1LENR, 15, 0)
  ),
  STM32_GATED_MUX_N(
    sai23sel_parents, nitems(sai23sel_parents),
    STM32_MUX(
      "sai23sel",
      STM32_RCC_D2CCIP1R,
      RCC_D2CCIP1R_SAI23SEL_SHIFT,
      3,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    2,
    STM32_GATE("sai2_ker_ck", STM32_RCC_APB2ENR, 22, 0),
    STM32_GATE("sai3_ker_ck", STM32_RCC_APB2ENR, 23, 0)
  ),
  STM32_GATED_MUX_N(
    sai1sel_parents, nitems(sai1sel_parents),
    STM32_MUX(
      "sai1sel",
      STM32_RCC_D2CCIP1R,
      RCC_D2CCIP1R_SAI1SEL_SHIFT,
      3,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("sai1_ker_ck", STM32_RCC_APB2ENR, 22, 0)
  ),
  STM32_GATED_MUX_N(
    lptim1sel_parents, nitems(lptim1sel_parents),
    STM32_MUX(
      "lptim1sel",
      STM32_RCC_D2CCIP2R,
      RCC_D2CCIP2R_LPTIM1SEL_SHIFT,
      3,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("lptim1_ker_ck", STM32_RCC_APB1LENR, 9, 0)
  ),
  STM32_GATED_MUX_N(
    cecsel_parents, nitems(cecsel_parents),
    STM32_MUX(
      "cecsel",
      STM32_RCC_D2CCIP2R,
      RCC_D2CCIP2R_CECSEL_SHIFT,
      2,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("cec_ker_ck", STM32_RCC_APB1LENR, 27, 0)
  ),
  STM32_GATED_MUX_N(
    usbsel_parents, nitems(usbsel_parents),
    STM32_MUX(
      "usbsel",
      STM32_RCC_D2CCIP2R,
      RCC_D2CCIP2R_USBSEL_SHIFT,
      2,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("usb_ker_ck", STM32_RCC_AHB1ENR, 25, 0)
  ),
  STM32_GATED_MUX_N(
    i2c123sel_parents, nitems(i2c123sel_parents),
    STM32_MUX(
      "i2c123sel",
      STM32_RCC_D2CCIP2R,
      RCC_D2CCIP2R_I2C123SEL_SHIFT,
      2,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    3,
    STM32_GATE("i2c1_ker_ck", STM32_RCC_APB1LENR, 21, 0),
    STM32_GATE("i2c2_ker_ck", STM32_RCC_APB1LENR, 22, 0),
    STM32_GATE("i2c3_ker_ck", STM32_RCC_APB1LENR, 23, 0)
  ),
  STM32_GATED_MUX_N(
    rngsel_parents, nitems(rngsel_parents),
    STM32_MUX(
      "rngsel",
      STM32_RCC_D2CCIP2R,
      RCC_D2CCIP2R_RNGSEL_SHIFT,
      2,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("rng_ker_ck", STM32_RCC_AHB2ENR, 6, 0)
  ),
  STM32_GATED_MUX_N(
    usart16sel_parents, nitems(usart16sel_parents),
    STM32_MUX(
      "usart16sel",
      STM32_RCC_D2CCIP2R,
      RCC_D2CCIP2R_USART16SEL_SHIFT,
      3,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
    2,
    STM32_GATE("usart1_ker_ck", STM32_RCC_APB2ENR, 4, 0),
    STM32_GATE("usart6_ker_ck", STM32_RCC_APB2ENR, 5, 0)
  ),
  STM32_GATED_MUX_N(
    usart234578sel_parents, nitems(usart234578sel_parents),
    STM32_MUX(
      "usart234578sel",
      STM32_RCC_D2CCIP2R,
      RCC_D2CCIP2R_USART234578SEL_SHIFT,
      3,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    6,
    STM32_GATE("usart2_ker_ck", STM32_RCC_APB1LENR, 17, 0),
    STM32_GATE("usart3_ker_ck", STM32_RCC_APB1LENR, 18, 0),
    STM32_GATE("uart4_ker_ck", STM32_RCC_APB1LENR, 19, 0),
    STM32_GATE("uart5_ker_ck", STM32_RCC_APB1LENR, 20, 0),
    STM32_GATE("uart7_ker_ck", STM32_RCC_APB1LENR, 30, 0),
    STM32_GATE("uart8_ker_ck", STM32_RCC_APB1LENR, 31, 0)
  ),
  STM32_GATED_MUX_N(
    spi6sel_parents, nitems(spi6sel_parents),
    STM32_MUX(
      "spi6sel",
      STM32_RCC_D3CCIPR,
      RCC_D3CCIPR_SPI6SEL_SHIFT,
      3,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("spi6_ker_ck", STM32_RCC_APB4ENR, 5, 0)
  ),
  STM32_GATED_MUX_N(
    sai4bsel_parents, nitems(sai4bsel_parents),
    STM32_MUX(
      "sai4bsel",
      STM32_RCC_D3CCIPR,
      RCC_D3CCIPR_SAI4BSEL_SHIFT,
      3,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("sai4b_ker_ck", STM32_RCC_APB4ENR, 21, 0)
  ),
  STM32_GATED_MUX_N(
    sai4asel_parents, nitems(sai4asel_parents),
    STM32_MUX(
      "sai4asel",
      STM32_RCC_D3CCIPR,
      RCC_D3CCIPR_SAI4ASEL_SHIFT,
      3,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("sai4a_ker_ck", STM32_RCC_APB4ENR, 21, 0)
  ),
  STM32_GATED_MUX_N(
    adcsel_parents, nitems(adcsel_parents),
    STM32_MUX(
      "adcsel",
      STM32_RCC_D3CCIPR,
      RCC_D3CCIPR_ADCSEL_SHIFT,
      2,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("adc_ker_ck", STM32_RCC_AHB1ENR, 5, 0)
  ),
  STM32_GATED_MUX_N(
    lptim345sel_parents, nitems(lptim345sel_parents),
    STM32_MUX(
      "lptim345sel",
      STM32_RCC_D3CCIPR,
      RCC_D3CCIPR_LPTIM345SEL_SHIFT,
      3,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    3,
    STM32_GATE("lptim3_ker_ck", STM32_RCC_APB4ENR, 3, 0),
    STM32_GATE("lptim4_ker_ck", STM32_RCC_APB4ENR, 4, 0),
    STM32_GATE("lptim5_ker_ck", STM32_RCC_APB4ENR, 5, 0)
  ),
  STM32_GATED_MUX_N(
    lptim2sel_parents, nitems(lptim2sel_parents),
    STM32_MUX(
      "lptim2sel",
      STM32_RCC_D3CCIPR,
      RCC_D3CCIPR_LPTIM2SEL_SHIFT,
      3,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("lptim2_ker_ck", STM32_RCC_APB4ENR, 9, 0)
  ),
  STM32_GATED_MUX_N(
    i2c4sel_parents, nitems(i2c4sel_parents),
    STM32_MUX(
      "i2c4sel",
      STM32_RCC_D3CCIPR,
      RCC_D3CCIPR_I2C4SEL_SHIFT,
      2,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("i2c4_ker_ck", STM32_RCC_APB4ENR, 7, 0)
  ),
  STM32_GATED_MUX_N(
    lpuart1sel_parents, nitems(lpuart1sel_parents),
    STM32_MUX(
      "lpuart1sel",
      STM32_RCC_D3CCIPR,
      RCC_D3CCIPR_LPUART1SEL_SHIFT,
      3,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    1,
    STM32_GATE("lpuart1_ker_ck", STM32_RCC_APB4ENR, 3, 0)
  ),
};

/* STM32H7 RTC clock parent names */

static const char * const rtcsel_parents[] =
{
  "off", "lse_ck_gate", "lsi_ck_gate", "hse_ck_gate"
};

/* STM32H7 RTC clock */

static const struct stm32_gated_mux_s stm32h7_rtcsel =
  STM32_GATED_MUX_N(
    rtcsel_parents, nitems(rtcsel_parents),
    STM32_MUX(
      "rtcsel",
      STM32_RCC_BDCR,
      RCC_BDCR_RTCSEL_SHIFT,
      2,
      CLK_MUX_ROUND_CLOSEST
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
    1,
    STM32_GATE("rcc_rtc_ck", STM32_RCC_BDCR, 15, 0)
  );

/* STM32H7 MCO clocks parent names */

static const char * const mco1_parents[] =
{
  "hsi_ck", "lse_ck", "hse_ck", "pll1_q_ck", "hsi48_ck"
};

static const char * const mco2_parents[] =
{
  "sys_ck", "pll2_p_ck", "hse_ck",
  "pll1_p_ck", "csi_ck", "lsi_ck"
};

/* STM32H7 MCO clocks */

static const struct stm32_divided_mux_n_s stm32h7_mco[] =
{
  STM32_DIVIDED_MUX_N(
    mco1_parents,
    nitems(mco1_parents),
    STM32_MUX(
      "mco1",
      STM32_RCC_CFGR,
      RCC_CFGR_MCO1_SHIFT,
      3,
      0
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
    1,
    STM32_DIVIDER(
      "mco1pre",
      STM32_RCC_CFGR,
      RCC_CFGR_MCO1PRE_SHIFT,
      4,
      CLK_DIVIDER_POWER_OF_TWO
    )
  ),
  STM32_DIVIDED_MUX_N(
    mco2_parents,
    nitems(mco2_parents),
    STM32_MUX(
      "mco2",
      STM32_RCC_CFGR,
      RCC_CFGR_MCO2_SHIFT,
      3,
      0
    ),
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
    1,
    STM32_DIVIDER(
      "mco2pre",
      STM32_RCC_CFGR,
      RCC_CFGR_MCO2PRE_SHIFT,
      4,
      CLK_DIVIDER_POWER_OF_TWO
    )
  ),
};

/* STM32H7 GPIO gates */

static const struct stm32_gate_s stm32h7_gpio_gates[] =
{
  STM32_GATE("gpioa_ker_ck", STM32_RCC_AHB4ENR, 0, 0),
  STM32_GATE("gpiob_ker_ck", STM32_RCC_AHB4ENR, 1, 0),
  STM32_GATE("gpioc_ker_ck", STM32_RCC_AHB4ENR, 2, 0),
  STM32_GATE("gpiod_ker_ck", STM32_RCC_AHB4ENR, 3, 0),
  STM32_GATE("gpioe_ker_ck", STM32_RCC_AHB4ENR, 4, 0),
  STM32_GATE("gpiof_ker_ck", STM32_RCC_AHB4ENR, 5, 0),
  STM32_GATE("gpiog_ker_ck", STM32_RCC_AHB4ENR, 6, 0),
  STM32_GATE("gpioh_ker_ck", STM32_RCC_AHB4ENR, 7, 0),
  STM32_GATE("gpioi_ker_ck", STM32_RCC_AHB4ENR, 8, 0),
  STM32_GATE("gpioj_ker_ck", STM32_RCC_AHB4ENR, 9, 0),
  STM32_GATE("gpiok_ker_ck", STM32_RCC_AHB4ENR, 10, 0),
};

/* STM32H7 APB1 timer gates (parent: cdppre1) */

static const struct stm32_gate_s stm32h7_apb1_tim_gates[] =
{
  STM32_GATE("tim2_ker_ck", STM32_RCC_APB1LENR, 0, 0),
  STM32_GATE("tim3_ker_ck", STM32_RCC_APB1LENR, 1, 0),
  STM32_GATE("tim4_ker_ck", STM32_RCC_APB1LENR, 2, 0),
  STM32_GATE("tim5_ker_ck", STM32_RCC_APB1LENR, 3, 0),
  STM32_GATE("tim6_ker_ck", STM32_RCC_APB1LENR, 4, 0),
  STM32_GATE("tim7_ker_ck", STM32_RCC_APB1LENR, 5, 0),
  STM32_GATE("tim12_ker_ck", STM32_RCC_APB1LENR, 6, 0),
  STM32_GATE("tim13_ker_ck", STM32_RCC_APB1LENR, 7, 0),
  STM32_GATE("tim14_ker_ck", STM32_RCC_APB1LENR, 8, 0),
};

/* STM32H7 APB2 timer gates (parent: cdppre2) */

static const struct stm32_gate_s stm32h7_apb2_tim_gates[] =
{
  STM32_GATE("tim1_ker_ck", STM32_RCC_APB2ENR, 0, 0),
  STM32_GATE("tim8_ker_ck", STM32_RCC_APB2ENR, 1, 0),
  STM32_GATE("tim15_ker_ck", STM32_RCC_APB2ENR, 16, 0),
  STM32_GATE("tim16_ker_ck", STM32_RCC_APB2ENR, 17, 0),
  STM32_GATE("tim17_ker_ck", STM32_RCC_APB2ENR, 18, 0),
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void
stm32_register_gated_osc(const struct stm32_gated_fixed_rate_s *osc);
static void
stm32_register_pll(const struct stm32_pll_s *pll);
static void
stm32_register_sysclk(const struct stm32_sysclk_s *sysclk);
static void
stm32_register_gated_mux(const struct stm32_gated_mux_s *gated_mux);
static void
stm32_register_divided_mux_n(const struct stm32_divided_mux_n_s *divided_mux_n);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void
stm32_register_gated_osc(const struct stm32_gated_fixed_rate_s *osc)
{
  int i;

  /* Register the fixed rate clock */

  ASSERT(clk_register_fixed_rate(
    osc->fixed_rate.fixed_rate_name,
    NULL,
    osc->base_flags,
    (FAR struct clk_fixed_rate_s *)&osc->fixed_rate.fixed_rate
  ));

  /* Register all gate clocks using the fixed rate as parent */

  for (i = 0; i < osc->num_gates; i++)
    {
      ASSERT(clk_register_gate(
        osc->gates[i].gate_name,
        osc->fixed_rate.fixed_rate_name,
        osc->base_flags,
        (FAR struct clk_gate_s *)&osc->gates[i].gate
      ));
    }
}

static void
stm32_register_sysclk(const struct stm32_sysclk_s *sysclk)
{
  /* Register the sysclk mux */

  ASSERT(clk_register_mux(
    sysclk->sysclk_mux.mux_name,
    sysclk->sysclk_parent_names,
    sysclk->sysclk_parent_names_count,
    sysclk->base_flags,
    (FAR struct clk_mux_s *)&sysclk->sysclk_mux.mux
  ));

  /* Register the traceclk mux */

  ASSERT(clk_register_mux(
    sysclk->traceclk_mux.mux_name,
    sysclk->traceclk_parent_names,
    sysclk->traceclk_parent_names_count,
    sysclk->base_flags,
    (FAR struct clk_mux_s *)&sysclk->traceclk_mux.mux
  ));

  /* Register cdcpre divider */

  ASSERT(clk_register_divider(
    sysclk->cdcpre.div_name,
    sysclk->sysclk_mux.mux_name,
    sysclk->base_flags,
    (FAR struct clk_divider_s *)&sysclk->cdcpre.div
  ));

  /* Register hpre divider */

  ASSERT(clk_register_divider(
    sysclk->hpre.div_name,
    sysclk->cdcpre.div_name,
    sysclk->base_flags,
    (FAR struct clk_divider_s *)&sysclk->hpre.div
  ));

  /* Register cdppre divider */

  ASSERT(clk_register_divider(
    sysclk->cdppre.div_name,
    sysclk->hpre.div_name,
    sysclk->base_flags,
    (FAR struct clk_divider_s *)&sysclk->cdppre.div
  ));

  /* Register cdppre1 divider */

  ASSERT(clk_register_divider(
    sysclk->cdppre1.div_name,
    sysclk->hpre.div_name,
    sysclk->base_flags,
    (FAR struct clk_divider_s *)&sysclk->cdppre1.div
  ));

  /* Register cdppre2 divider */

  ASSERT(clk_register_divider(
    sysclk->cdppre2.div_name,
    sysclk->hpre.div_name,
    sysclk->base_flags,
    (FAR struct clk_divider_s *)&sysclk->cdppre2.div
  ));

  /* Register sdrppre divider */

  ASSERT(clk_register_divider(
    sysclk->sdrppre.div_name,
    sysclk->cdcpre.div_name,
    sysclk->base_flags,
    (FAR struct clk_divider_s *)&sysclk->sdrppre.div
  ));
}

static void
stm32_register_pll(const struct stm32_pll_s *pll)
{
  /* Register the PLL multiplier (divn) */

  ASSERT(clk_register_multiplier(
    pll->divn.mult_name,
    pll->parent_name,
    pll->base_flags,
    (FAR struct clk_multiplier_s *)&pll->divn.mult
  ));

  /* Register divp divider */

  ASSERT(clk_register_divider(
    pll->divp.div_name,
    pll->divn.mult_name,
    pll->base_flags,
    (FAR struct clk_divider_s *)&pll->divp.div
  ));

  /* Register divp gate */

  ASSERT(clk_register_gate(
    pll->gatep.gate_name,
    pll->divp.div_name,
    pll->base_flags,
    (FAR struct clk_gate_s *)&pll->gatep.gate
  ));

  /* Register divq divider */

  ASSERT(clk_register_divider(
    pll->divq.div_name,
    pll->divn.mult_name,
    pll->base_flags,
    (FAR struct clk_divider_s *)&pll->divq.div
  ));

  /* Register divq gate */

  ASSERT(clk_register_gate(
    pll->gateq.gate_name,
    pll->divq.div_name,
    pll->base_flags,
    (FAR struct clk_gate_s *)&pll->gateq.gate
  ));

  /* Register divr divider */

  ASSERT(clk_register_divider(
    pll->divr.div_name,
    pll->divn.mult_name,
    pll->base_flags,
    (FAR struct clk_divider_s *)&pll->divr.div
  ));

  /* Register divr gate */

  ASSERT(clk_register_gate(
    pll->gater.gate_name,
    pll->divr.div_name,
    pll->base_flags,
    (FAR struct clk_gate_s *)&pll->gater.gate
  ));
}

static void
stm32_register_gated_mux(const struct stm32_gated_mux_s *gated_mux)
{
  int i;

  /* Register the mux */

  ASSERT(clk_register_mux(
    gated_mux->mux.mux_name,
    gated_mux->parent_names,
    gated_mux->parent_names_count,
    gated_mux->base_flags,
    (FAR struct clk_mux_s *)&gated_mux->mux.mux
  ));

  /* Register all gate clocks */

  for (i = 0; i < gated_mux->num_gates; i++)
    {
      ASSERT(clk_register_gate(
        gated_mux->gates[i].gate_name,
        gated_mux->mux.mux_name,
        gated_mux->base_flags,
        (FAR struct clk_gate_s *)&gated_mux->gates[i].gate
      ));
    }
}

static void
stm32_register_divided_mux_n(const struct stm32_divided_mux_n_s *divided_mux_n)
{
  int i;

  /* Register the mux */

  ASSERT(clk_register_mux(
    divided_mux_n->mux.mux_name,
    divided_mux_n->parent_names,
    divided_mux_n->parent_names_count,
    divided_mux_n->base_flags,
    (FAR struct clk_mux_s *)&divided_mux_n->mux.mux
  ));

  /* Register all dividers */

  for (i = 0; i < divided_mux_n->num_divs; i++)
    {
      ASSERT(clk_register_divider(
        divided_mux_n->divs[i].div_name,
        divided_mux_n->mux.mux_name,
        divided_mux_n->base_flags,
        (FAR struct clk_divider_s *)&divided_mux_n->divs[i].div
      ));
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_clk_register
 *
 * Description:
 *   Initialize the STM32H7 clock system using common clock framework
 *
 ****************************************************************************/

void up_clk_register()
{
  int i;

  /* Register HSI divider */

  struct clk_divider_s hsi_div = {
    .reg = STM32_RCC_CR,
    .shift = RCC_CR_HSIDIV_SHIFT,
    .width = 2,
    .flags = CLK_DIVIDER_POWER_OF_TWO
  };

  ASSERT(clk_register_divider(
    "hsi_div",
    "hsi",
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
    &hsi_div
  ));

  /* Register HSI gates */

  struct clk_gate_s hsi_ck_gate = {
    .reg = STM32_RCC_CR,
    .bit_idx = 0,
    .flags = 0
  };

  ASSERT(clk_register_gate(
    "hsi_ck",
    "hsi_div",
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
    &hsi_ck_gate
  ));

  struct clk_gate_s hsi_ker_ck_gate = {
    .reg = STM32_RCC_CR,
    .bit_idx = 1,
    .flags = 0
  };

  ASSERT(clk_register_gate(
    "hsi_ker_ck",
    "hsi_div",
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
    &hsi_ker_ck_gate
  ));

  /* Register all gated oscillators */

  for (i = 0;
       i < sizeof(stm32h7_gated_oscs) / sizeof(stm32h7_gated_oscs[0]);
       i++)
    {
      stm32_register_gated_osc(&stm32h7_gated_oscs[i]);
    }

  /* Register PLL clock selection */

  stm32_register_divided_mux_n(&stm32h7_pll_cksel);

  /* Register all PLLs */

  for (i = 0; i < sizeof(stm32h7_plls) / sizeof(stm32h7_plls[0]); i++)
    {
      stm32_register_pll(&stm32h7_plls[i]);
    }

  /* Register system clock */

  stm32_register_sysclk(&stm32h7_sysclk);

  /* Register all gated muxes */

  for (i = 0;
       i < sizeof(stm32h7_gated_muxes) / sizeof(stm32h7_gated_muxes[0]);
       i++)
    {
      stm32_register_gated_mux(&stm32h7_gated_muxes[i]);
    }

  /* Register RTC clock */

  stm32_register_gated_mux(&stm32h7_rtcsel);

  /* Register MCO clocks */

  for (i = 0; i < sizeof(stm32h7_mco) / sizeof(stm32h7_mco[0]); i++)
    {
      stm32_register_divided_mux_n(&stm32h7_mco[i]);
    }

  /* Register GPIO clocks */

  for (i = 0;
       i < sizeof(stm32h7_gpio_gates) / sizeof(stm32h7_gpio_gates[0]);
       i++)
    {
      ASSERT(clk_register_gate(
        stm32h7_gpio_gates[i].gate_name,
        "hpre",
        CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
        (FAR struct clk_gate_s *)&stm32h7_gpio_gates[i].gate
      ));
    }

  /* Register APB1 timer clocks */

  for (i = 0;
       i < sizeof(stm32h7_apb1_tim_gates) / sizeof(stm32h7_apb1_tim_gates[0]);
       i++)
    {
      ASSERT(clk_register_gate(
        stm32h7_apb1_tim_gates[i].gate_name,
        "cdppre1",
        CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
        (FAR struct clk_gate_s *)&stm32h7_apb1_tim_gates[i].gate
      ));
    }

  /* Register APB2 timer clocks */

  for (i = 0;
       i < sizeof(stm32h7_apb2_tim_gates) / sizeof(stm32h7_apb2_tim_gates[0]);
       i++)
    {
      ASSERT(clk_register_gate(
        stm32h7_apb2_tim_gates[i].gate_name,
        "cdppre2",
        CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
        (FAR struct clk_gate_s *)&stm32h7_apb2_tim_gates[i].gate
      ));
    }
}
