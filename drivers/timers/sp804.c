/****************************************************************************
 * drivers/timers/sp804.c
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

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/sp804.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SP804 per-timer register offsets */

#define SP804_LOAD          0x00  /* Load register */
#define SP804_VALUE         0x04  /* Current value (RO) */
#define SP804_CONTROL       0x08  /* Control register */
#define SP804_INTCLR        0x0c  /* Interrupt clear (WO) */

/* Timer 2 sits at base + 0x20 with the same layout */

#define SP804_TIMER2_OFF    0x20

/* Control register bits */

#define SP804_CTRL_ONESHOT  (1 << 0)  /* One-shot mode */
#define SP804_CTRL_32BIT    (1 << 1)  /* 32-bit counter */
#define SP804_CTRL_IE       (1 << 5)  /* Interrupt enable */
#define SP804_CTRL_ENABLE   (1 << 7)  /* Timer enable */

/* Helper macros */

#define sp804_getreg(b, o) \
  (*(FAR volatile uint32_t *)((b) + (o)))
#define sp804_putreg(v, b, o) \
  (*((FAR volatile uint32_t *)((b) + (o))) = (v))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sp804_lowerhalf_s
{
  struct oneshot_lowerhalf_s lower;
  uintptr_t cnt;    /* Timer 1: free-running counter */
  uintptr_t alarm;  /* Timer 2: oneshot alarm */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static clkcnt_t sp804_max_delay(FAR struct oneshot_lowerhalf_s *lower);
static clkcnt_t sp804_current(FAR struct oneshot_lowerhalf_s *lower);
static void sp804_start(FAR struct oneshot_lowerhalf_s *lower,
                        clkcnt_t delta);
static void sp804_cancel(FAR struct oneshot_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_sp804_ops =
{
  .current   = sp804_current,
  .start     = sp804_start,
  .cancel    = sp804_cancel,
  .max_delay = sp804_max_delay,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sp804_interrupt
 ****************************************************************************/

static int sp804_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct sp804_lowerhalf_s *priv = arg;

  sp804_putreg(1, priv->alarm, SP804_INTCLR);
  sp804_putreg(0, priv->alarm, SP804_CONTROL);
  oneshot_process_callback(&priv->lower);

  return 0;
}

static clkcnt_t sp804_max_delay(FAR struct oneshot_lowerhalf_s *lower)
{
  return UINT32_MAX;
}

static clkcnt_t sp804_current(FAR struct oneshot_lowerhalf_s *lower)
{
  FAR struct sp804_lowerhalf_s *priv =
    (FAR struct sp804_lowerhalf_s *)lower;

  /* Timer 1 counts down; invert to get increasing count */

  return UINT32_MAX - sp804_getreg(priv->cnt, SP804_VALUE);
}

static void sp804_start(FAR struct oneshot_lowerhalf_s *lower,
                        clkcnt_t delta)
{
  FAR struct sp804_lowerhalf_s *priv =
    (FAR struct sp804_lowerhalf_s *)lower;

  sp804_putreg(0, priv->alarm, SP804_CONTROL);
  sp804_putreg((uint32_t)delta, priv->alarm, SP804_LOAD);
  sp804_putreg(1, priv->alarm, SP804_INTCLR);
  sp804_putreg(SP804_CTRL_ENABLE | SP804_CTRL_32BIT |
               SP804_CTRL_ONESHOT | SP804_CTRL_IE,
               priv->alarm, SP804_CONTROL);
}

static void sp804_cancel(FAR struct oneshot_lowerhalf_s *lower)
{
  FAR struct sp804_lowerhalf_s *priv =
    (FAR struct sp804_lowerhalf_s *)lower;

  sp804_putreg(0, priv->alarm, SP804_CONTROL);
  sp804_putreg(1, priv->alarm, SP804_INTCLR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct oneshot_lowerhalf_s *
sp804_initialize(uintptr_t base, int irq, uint32_t freq)
{
  FAR struct sp804_lowerhalf_s *priv;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->lower.ops = &g_sp804_ops;
  priv->cnt   = base;
  priv->alarm = base + SP804_TIMER2_OFF;

  oneshot_count_init(&priv->lower, freq);

  /* Timer 1: free-running 32-bit counter */

  sp804_putreg(0, priv->cnt, SP804_CONTROL);
  sp804_putreg(UINT32_MAX, priv->cnt, SP804_LOAD);
  sp804_putreg(SP804_CTRL_ENABLE | SP804_CTRL_32BIT,
               priv->cnt, SP804_CONTROL);

  /* Timer 2: oneshot alarm (initially disabled) */

  sp804_putreg(0, priv->alarm, SP804_CONTROL);
  sp804_putreg(1, priv->alarm, SP804_INTCLR);

  irq_attach(irq, sp804_interrupt, priv);
  up_enable_irq(irq);

  return &priv->lower;
}
