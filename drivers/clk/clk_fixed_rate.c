/****************************************************************************
 * drivers/clk/clk_fixed_rate.c
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

#include <nuttx/clk/clk_provider.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define to_clk_fixed_rate(_clk) (FAR struct clk_fixed_rate_s *) \
                                (_clk->private_data)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t clk_fixed_rate_recalc_rate(FAR struct clk_s *clk,
                                           uint32_t parent_rate)
{
  FAR struct clk_fixed_rate_s *fixed = to_clk_fixed_rate(clk);
  return fixed->fixed_rate;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clk_ops_s clk_fixed_rate_ops =
{
  .recalc_rate = clk_fixed_rate_recalc_rate,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct clk_s *clk_register_fixed_rate(FAR const char *name,
                                          FAR const char *parent_name,
                                          uint8_t flags, uint32_t fixed_rate)
{
  struct clk_fixed_rate_s fixed;
  FAR const char **parent_names;
  uint8_t num_parents;

  parent_names = parent_name ? &parent_name: NULL;
  num_parents = parent_name ? 1 : 0;

  fixed.fixed_rate = fixed_rate;

  return clk_register(name, parent_names, num_parents, flags,
                      &clk_fixed_rate_ops, &fixed, sizeof(fixed));
}
