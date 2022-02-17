/****************************************************************************
 * include/nuttx/clk/clk.h
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

#ifndef __INCLUDE_NUTTX_CLK_CLK_H
#define __INCLUDE_NUTTX_CLK_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct clk_s;
struct clk_ops_s;

struct clk_rate_s
{
  FAR const char *name;
  uint32_t        rate;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct clk_s *clk_get(FAR const char *name);
FAR struct clk_s *clk_get_parent(FAR struct clk_s *clk);
FAR struct clk_s *clk_get_parent_by_index(FAR struct clk_s *clk,
                                          uint8_t index);

int clk_set_parent(FAR struct clk_s *clk, FAR struct clk_s *parent);
int clk_enable(FAR struct clk_s *clk);
int clk_disable(FAR struct clk_s *clk);
int clk_is_enabled(FAR struct clk_s *clk);

uint32_t clk_round_rate(FAR struct clk_s *clk, uint32_t rate);
int      clk_set_rate(FAR struct clk_s *clk, uint32_t rate);
int      clk_set_rates(FAR const struct clk_rate_s *rates);
uint32_t clk_get_rate(FAR struct clk_s *clk);

int      clk_set_phase(FAR struct clk_s *clk, int degrees);
int      clk_get_phase(FAR struct clk_s *clk);

void clk_disable_unused(void);
FAR const char *clk_get_name(const FAR struct clk_s *clk);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CLK_CLK_H */
