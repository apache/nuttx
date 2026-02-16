/****************************************************************************
 * arch/risc-v/src/common/riscv_mtimer.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "riscv_mtimer.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * oneshot_lowerhalf_s structure.
 */

struct riscv_mtimer_lowerhalf_s
{
  struct oneshot_lowerhalf_s lower;
  int                        irq;
  uintreg_t                  mtime;
  uintreg_t                  mtimecmp;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static clkcnt_t riscv_mtime_max_delay(struct oneshot_lowerhalf_s *lower);
static clkcnt_t riscv_mtime_current(struct oneshot_lowerhalf_s *lower);
static void riscv_mtime_start_absolute(struct oneshot_lowerhalf_s *lower,
                                       clkcnt_t expected);
static void riscv_mtime_start(struct oneshot_lowerhalf_s *lower,
                              clkcnt_t delta);
static void riscv_mtime_cancel(struct oneshot_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_riscv_mtimer_ops =
{
  .current        = riscv_mtime_current,
  .start          = riscv_mtime_start,
  .start_absolute = riscv_mtime_start_absolute,
  .cancel         = riscv_mtime_cancel,
  .max_delay      = riscv_mtime_max_delay
};

static struct riscv_mtimer_lowerhalf_s g_riscv_mtime_lowerhalf =
{
  .lower.ops = &g_riscv_mtimer_ops
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t riscv_mtimer_get_mtime(struct riscv_mtimer_lowerhalf_s *priv)
{
  return riscv_mtimer_get(priv->mtime);
}

static void riscv_mtimer_set_mtimecmp(struct riscv_mtimer_lowerhalf_s *priv,
                                      uint64_t value)
{
  riscv_mtimer_set(priv->mtime, priv->mtimecmp + 8 * up_cpu_index(), value);
}

static clkcnt_t riscv_mtime_max_delay(struct oneshot_lowerhalf_s *lower)
{
  return UINT64_MAX;
}

static clkcnt_t riscv_mtime_current(struct oneshot_lowerhalf_s *lower)
{
  struct riscv_mtimer_lowerhalf_s *priv = &g_riscv_mtime_lowerhalf;
  return riscv_mtimer_get_mtime(priv);
}

static void riscv_mtime_start_absolute(struct oneshot_lowerhalf_s *lower,
                                       clkcnt_t expected)
{
  struct riscv_mtimer_lowerhalf_s *priv = &g_riscv_mtime_lowerhalf;

  riscv_mtimer_set_mtimecmp(priv, expected);
}

static void riscv_mtime_start(struct oneshot_lowerhalf_s *lower,
                              clkcnt_t delta)
{
  struct riscv_mtimer_lowerhalf_s *priv = &g_riscv_mtime_lowerhalf;
  irqstate_t flags = up_irq_save();
  uint64_t   curr  = riscv_mtimer_get_mtime(priv);

  riscv_mtimer_set_mtimecmp(priv, curr + delta);

  up_irq_restore(flags);
}

static void riscv_mtime_cancel(struct oneshot_lowerhalf_s *lower)
{
  struct riscv_mtimer_lowerhalf_s *priv = &g_riscv_mtime_lowerhalf;
  riscv_mtimer_set_mtimecmp(priv, UINT64_MAX);
}

static int riscv_mtimer_interrupt(int irq, void *context, void *arg)
{
  struct riscv_mtimer_lowerhalf_s *priv = &g_riscv_mtime_lowerhalf;

  riscv_mtimer_set_mtimecmp(priv, UINT64_MAX);
  oneshot_process_callback(&priv->lower);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_mtimer_oneshot_initialize_per_cpu
 *
 * Description:
 *   Initialize the riscv mtimer for secondary CPUs.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void riscv_mtimer_oneshot_initialize_per_cpu(void)
{
  struct riscv_mtimer_lowerhalf_s *priv = &g_riscv_mtime_lowerhalf;

  riscv_mtimer_set_mtimecmp(priv, UINT64_MAX);
  up_enable_irq(priv->irq);
}

struct oneshot_lowerhalf_s *
riscv_mtimer_initialize(uintreg_t mtime, uintreg_t mtimecmp,
                        int irq, uint64_t freq)
{
  struct riscv_mtimer_lowerhalf_s *priv = &g_riscv_mtime_lowerhalf;

  priv->mtime    = mtime;
  priv->irq      = irq;
  priv->mtimecmp = mtimecmp;

  oneshot_count_init(&priv->lower, freq);

  irq_attach(priv->irq, riscv_mtimer_interrupt, priv);
  riscv_mtimer_oneshot_initialize_per_cpu();

  return &priv->lower;
}

#ifdef CONFIG_SMP
void riscv_timer_secondary_init(void)
{
  riscv_mtimer_oneshot_initialize_per_cpu();
}
#endif
