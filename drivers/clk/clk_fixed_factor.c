/****************************************************************************
 * drivers/clk/clk_fixed_factor.c
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

#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk_provider.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define to_clk_fixed_factor(_clk) (FAR struct clk_fixed_factor_s *) \
                                  (_clk->private_data)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t clk_factor_recalc_rate(FAR struct clk_s *clk,
                                       uint32_t parent_rate)
{
  FAR struct clk_fixed_factor_s *fixed = to_clk_fixed_factor(clk);
  uint64_t rate = parent_rate;

  rate *= fixed->mult;
  rate /= fixed->div;
  return rate;
}

static uint32_t clk_factor_round_rate(FAR struct clk_s *clk,
                                      uint32_t rate,
                                      FAR uint32_t *prate)
{
  FAR struct clk_fixed_factor_s *fixed = to_clk_fixed_factor(clk);

  if (clk->flags & CLK_SET_RATE_PARENT)
    {
      uint32_t best_parent;

      best_parent = ((uint64_t)rate * fixed->div) / fixed->mult;
      *prate = clk_round_rate(clk_get_parent(clk), best_parent);
    }

  return ((uint64_t)*prate * fixed->mult) / fixed->div;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clk_ops_s g_clk_fixed_factor_ops =
{
  .round_rate = clk_factor_round_rate,
  .recalc_rate = clk_factor_recalc_rate,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct clk_s *clk_register_fixed_factor(FAR const char *name,
                                            FAR const char *parent_name,
                                            uint8_t flags, uint8_t mult,
                                            uint8_t div)
{
  struct clk_fixed_factor_s fixed;
  FAR const char **parent_names;
  uint8_t num_parents;

  parent_names = parent_name ? &parent_name : NULL;
  num_parents = parent_name ? 1 : 0;

  fixed.mult = mult;
  fixed.div = div;

  return clk_register(name, parent_names, num_parents, flags,
                      &g_clk_fixed_factor_ops, &fixed, sizeof(fixed));
}
