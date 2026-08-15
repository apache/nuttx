/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_clk.h
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

#ifndef __ARCH_RISCV_SRC_EIC7700X_EIC7700X_CLK_H
#define __ARCH_RISCV_SRC_EIC7700X_EIC7700X_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/clk/clk_provider.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* One output of one PLL.
 *
 * The voltage controlled oscillator is deliberately not modelled as a
 * clock of its own.  Clock rates in the framework are uint32_t, and the
 * pre divider term FREF * FBDIV can reach roughly 24GHz for the largest
 * feedback divider the hardware accepts.  Each output therefore reads the
 * shared configuration words itself and keeps the intermediate result in
 * 64 bit arithmetic.
 */

struct eic7700x_clk_pll_s
{
  uintptr_t reg;             /* Address of the PLL cfg0 register          */
  uint8_t   out;             /* Which output this clock is, 0 to 3        */
  uint8_t   lockbit;         /* Bit reporting lock in EIC7700X_PLL_STATUS */
};

/* A divisor field in one of the CRG control registers.
 *
 * Most CRG divisors read "0, 1, 2: 2 divisor ... n: n divisor", so field
 * values below two are aliases for divide by two.  A few fields instead
 * document 0 and 1 as bypass.  Both are covered by lowdiv, the effective
 * divisor to apply when the field reads 0 or 1.
 */

struct eic7700x_clk_div_s
{
  uintptr_t reg;             /* Address of the control register           */
  uint8_t   shift;           /* Least significant bit of the divisor      */
  uint8_t   width;           /* Width of the divisor in bits              */
  uint8_t   lowdiv;          /* Divisor to use for field values 0 and 1   */
};

/* A selector field in one of the CRG control registers.
 *
 * The generic clk_mux returns the raw register field as a parent index
 * without bounding it against the parent count.  Several selectors here
 * are two bits wide but only have three valid values, so the table below
 * maps register values onto parent indices and anything unexpected is
 * reported and folded onto parent zero.
 */

struct eic7700x_clk_mux_s
{
  uintptr_t          reg;    /* Address of the control register           */
  FAR const uint8_t *table;  /* Selector value routing each parent        */
  uint8_t            shift;  /* Least significant bit of the selector     */
  uint8_t            width;  /* Width of the selector in bits             */
  uint8_t            nparents;
};

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

EXTERN const struct clk_ops_s g_eic7700x_clk_pll_ops;
EXTERN const struct clk_ops_s g_eic7700x_clk_div_ops;
EXTERN const struct clk_ops_s g_eic7700x_clk_mux_ops;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_clk_initialize
 *
 * Description:
 *   Register the EIC7700X Clock and Reset Generator clock tree with the
 *   clock framework.  The provider is read only: it reports the PLL, mux,
 *   divider and gate configuration left behind by the boot loader and
 *   never reprograms it.
 *
 * Returned Value:
 *   OK on success, or a negated errno if any clock failed to register.
 *
 ****************************************************************************/

int eic7700x_clk_initialize(void);

/****************************************************************************
 * Name: eic7700x_clk_count
 *
 * Description:
 *   How many clocks eic7700x_clk_initialize() registered, and how many it
 *   failed to.  Zero before it has run.
 *
 * Input Parameters:
 *   failed - Where to report the number that failed, or NULL
 *
 * Returned Value:
 *   The number registered.
 *
 ****************************************************************************/

unsigned int eic7700x_clk_count(FAR unsigned int *failed);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_EIC7700X_EIC7700X_CLK_H */
