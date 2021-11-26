/****************************************************************************
 * include/nuttx/clk/clk_provider.h
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

#ifndef __INCLUDE_NUTTX_CLK_CLK_PROVIDER_H
#define __INCLUDE_NUTTX_CLK_CLK_PROVIDER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/list.h>

#include <stdint.h>
#include <stddef.h>

#ifdef CONFIG_CLK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CLK_SET_RATE_GATE               0x01
#define CLK_SET_PARENT_GATE             0x02
#define CLK_SET_RATE_PARENT             0x04
#define CLK_SET_RATE_NO_REPARENT        0x08
#define CLK_GET_RATE_NOCACHE            0x10
#define CLK_NAME_IS_STATIC              0x20
#define CLK_PARENT_NAME_IS_STATIC       0x40
#define CLK_IS_CRITICAL                 0x80

#define CLK_GATE_SET_TO_DISABLE         0x01
#define CLK_GATE_HIWORD_MASK            0x02

#define CLK_DIVIDER_ONE_BASED           0x01
#define CLK_DIVIDER_HIWORD_MASK         0x02
#define CLK_DIVIDER_ROUND_CLOSEST       0x04
#define CLK_DIVIDER_READ_ONLY           0x08
#define CLK_DIVIDER_MAX_HALF            0x10
#define CLK_DIVIDER_DIV_NEED_EVEN       0x20
#define CLK_DIVIDER_POWER_OF_TWO        0x40
#define CLK_DIVIDER_MINDIV_OFF          8
#define CLK_DIVIDER_MINDIV_MSK          0xff00

#define CLK_FRAC_MUL_NEED_EVEN          0x01
#define CLK_FRAC_DIV_DOUBLE             0x02

#define CLK_MULT_ONE_BASED              0x01
#define CLK_MULT_ALLOW_ZERO             0x02
#define CLK_MULT_HIWORD_MASK            0x04
#define CLK_MULT_MAX_HALF               0x08
#define CLK_MULT_ROUND_CLOSEST          0x10

#define CLK_MUX_HIWORD_MASK             0x01
#define CLK_MUX_READ_ONLY               0x02
#define CLK_MUX_ROUND_CLOSEST           0x04

#define CLK_PHASE_HIWORD_MASK           0x01

/****************************************************************************
 * Public Types
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

struct clk_s
{
  FAR const char             *name;
  FAR const struct clk_ops_s *ops;
  FAR struct clk_s           *parent;
  uint8_t                     num_parents;
  uint8_t                     new_parent_index;
  uint8_t                     enable_count;
  uint8_t                     flags;
  uint32_t                    rate;
  uint32_t                    new_rate;
  FAR struct clk_s           *new_parent;
  FAR struct clk_s           *new_child;
  FAR void                   *private_data;
  struct list_node            children;
  struct list_node            node;
  FAR const char             *parent_names[0];
};

struct clk_ops_s
{
  CODE int       (*enable)(FAR struct clk_s *clk);
  CODE void      (*disable)(FAR struct clk_s *clk);
  CODE int       (*is_enabled)(FAR struct clk_s *clk);
  CODE uint32_t  (*recalc_rate)(FAR struct clk_s *clk, uint32_t parent_rate);
  CODE uint32_t  (*round_rate)(FAR struct clk_s *clk,
                               uint32_t rate, uint32_t *parent_rate);
  CODE uint32_t  (*determine_rate)(FAR struct clk_s *clk, uint32_t rate,
                                   uint32_t *best_parent_rate,
                                   struct clk_s **best_parent_clk);
  CODE int       (*set_parent)(FAR struct clk_s *clk, uint8_t index);
  CODE uint8_t   (*get_parent)(FAR struct clk_s *clk);
  CODE int       (*set_rate)(FAR struct clk_s *clk, uint32_t rate,
                             uint32_t parent_rate);
  CODE int       (*set_rate_and_parent)(FAR struct clk_s *clk, uint32_t rate,
                                        uint32_t parent_rate,
                                        uint8_t index);
  CODE int       (*get_phase)(FAR struct clk_s *clk);
  CODE int       (*set_phase)(FAR struct clk_s *clk, int degrees);
};

struct clk_gate_s
{
  uint32_t            reg;
  uint8_t             bit_idx;
  uint8_t             flags;
};

struct clk_fixed_rate_s
{
  uint32_t            fixed_rate;
  uint8_t             flags;
};

struct clk_fixed_factor_s
{
  uint8_t             mult;
  uint8_t             div;
};

struct clk_divider_s
{
  uint32_t            reg;
  uint8_t             shift;
  uint8_t             width;
  uint16_t            flags;
};

struct clk_phase_s
{
  uint32_t            reg;
  uint8_t             shift;
  uint8_t             width;
  uint8_t             flags;
};

struct clk_fractional_divider_s
{
  uint32_t            reg;
  uint8_t             mwidth;
  uint8_t             nwidth;
  uint8_t             mshift;
  uint8_t             nshift;
  uint8_t             flags;
};

struct clk_multiplier_s
{
  uint32_t            reg;
  uint8_t             shift;
  uint8_t             width;
  uint8_t             flags;
};

struct clk_mux_s
{
  uint32_t            reg;
  uint8_t             width;
  uint8_t             shift;
  uint8_t             flags;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct clk_s *clk_register(FAR const char *name,
                               FAR const char * const *parent_names,
                               uint8_t num_parents, uint8_t flags,
                               FAR const struct clk_ops_s *ops,
                               FAR void *private_data, size_t private_size);

FAR struct clk_s *clk_register_gate(FAR const char *name,
                                    FAR const char *parent_name,
                                    uint8_t flags, uint32_t reg,
                                    uint8_t bit_idx,
                                    uint8_t clk_gate_flags);

FAR struct clk_s *clk_register_fixed_rate(FAR const char *name,
                                          FAR const char *parent_name,
                                          uint8_t flags,
                                          uint32_t fixed_rate);

FAR struct clk_s *clk_register_fixed_factor(FAR const char *name,
                                            FAR const char *parent_name,
                                            uint8_t flags, uint8_t mult,
                                            uint8_t div);

FAR struct clk_s *clk_register_divider(FAR const char *name,
                                       FAR const char *parent_name,
                                       uint8_t flags, uint32_t reg,
                                       uint8_t shift, uint8_t width,
                                       uint16_t clk_divider_flags);

FAR struct clk_s *clk_register_phase(FAR const char *name,
                                     FAR const char *parent_name,
                                     uint8_t flags, uint32_t reg,
                                     uint8_t shift, uint8_t width,
                                     uint8_t clk_phase_flags);

FAR struct clk_s *
clk_register_fractional_divider(FAR const char *name,
                                FAR const char *parent_name,
                                uint8_t flags, uint32_t reg,
                                uint8_t mshift, uint8_t mwidth,
                                uint8_t nshift, uint8_t nwidth,
                                uint8_t clk_divider_flags);

FAR struct clk_s *clk_register_multiplier(FAR const char *name,
                                          FAR const char *parent_name,
                                          uint8_t flags, uint32_t reg,
                                          uint8_t shift, uint8_t width,
                                          uint8_t clk_multiplier_flags);

FAR struct clk_s *clk_register_mux(FAR const char *name,
                                   const char * const *parent_names,
                                   uint8_t num_parents, uint8_t flags,
                                   uint32_t reg, uint8_t shift,
                                   uint8_t width, uint8_t clk_mux_flags);

#ifdef CONFIG_CLK_RPMSG
FAR struct clk_s *clk_register_rpmsg(FAR const char *name, uint8_t flags);

int clk_rpmsg_initialize(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_CLK */
#endif /* __INCLUDE_NUTTX_CLK_CLK_PROVIDER_H */
