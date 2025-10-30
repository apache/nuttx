/****************************************************************************
 * arch/sim/src/sim/sim_oneshot.c
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

#include <limits.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/list.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/timers/arch_alarm.h>

#include "sim_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the oneshot timer lower-half driver.
 */

struct sim_oneshot_lowerhalf_s
{
  /* This is the part of the lower half driver that is visible to the upper-
   * half client of the driver.  This must be the first thing in this
   * structure so that pointers to struct oneshot_lowerhalf_s are cast
   * compatible to struct sim_oneshot_lowerhalf_s and vice versa.
   */

  struct oneshot_lowerhalf_s lh;  /* Common lower-half driver fields */

  /* Private lower half data follows */

  struct list_node           node;
  uint64_t                   expire_time;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct list_node g_oneshot_list = LIST_INITIAL_VALUE(g_oneshot_list);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_timer_update_internal
 *
 * Description:
 *   This function is called periodically to deliver the tick events to the
 *   NuttX simulation.
 *
 ****************************************************************************/

static void
sim_timer_update_internal(struct sim_oneshot_lowerhalf_s *priv)
{
  struct sim_oneshot_lowerhalf_s *node;
  bool       is_head;
  irqstate_t flags = enter_critical_section();

  /* Insert the new oneshot timer to the list. */

  list_for_every_entry(&g_oneshot_list, node,
                       struct sim_oneshot_lowerhalf_s, node)
    {
      if ((int64_t)(priv->expire_time - node->expire_time) < 0)
        {
          break;
        }
    }

  is_head = list_is_head(&g_oneshot_list, &node->node);
  list_add_before(&node->node, &priv->node);

  /* Set the earliest expired timer */

#ifdef CONFIG_SIM_WALLTIME_SIGNAL
  if (is_head)
    {
      host_settimer(priv->expire_time);
    }
#else
  UNUSED(is_head);
#endif /* CONFIG_SIM_WALLTIME_SIGNAL */

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sim_timer_handler
 *
 * Description:
 *   The signal handler is called periodically and is used to deliver TICK
 *   events to the OS.
 *
 * Input Parameters:
 *   sig - the signal number
 *   si  - the signal information
 *   old_ucontext - the previous context
 *
 ****************************************************************************/

static int sim_timer_handler(int irq, void *context, void *arg)
{
  struct sim_oneshot_lowerhalf_s *priv;
  irqstate_t flags = enter_critical_section();
  uint64_t   curr  = host_gettime(false);

  /* Perform the callback if the timer is expired */

  while (!list_is_empty(&g_oneshot_list))
    {
      priv = list_first_entry(&g_oneshot_list,
                              struct sim_oneshot_lowerhalf_s, node);
      if ((int64_t)(curr - priv->expire_time) < 0)
        {
          break;
        }

      list_delete_init(&priv->node);
      oneshot_process_callback(&priv->lh);
    }

  leave_critical_section(flags);

  return OK;
}

static inline_function
void sim_oneshot_set_timer(struct sim_oneshot_lowerhalf_s *priv,
                           uint64_t expected_ns)
{
  irqstate_t flags = enter_critical_section();

  list_delete(&priv->node);
  priv->expire_time = expected_ns;

  sim_timer_update_internal(priv);
  leave_critical_section(flags);
}

static clkcnt_t sim_oneshot_max_delay(struct oneshot_lowerhalf_s *lower)
{
  return UINT64_MAX;
}

static clkcnt_t sim_oneshot_current(struct oneshot_lowerhalf_s *lower)
{
  return host_gettime(false);
}

static void sim_oneshot_start_absolute(struct oneshot_lowerhalf_s *lower,
                                       clkcnt_t expected)
{
  sim_oneshot_set_timer((struct sim_oneshot_lowerhalf_s *)lower, expected);
}

static void sim_oneshot_start(struct oneshot_lowerhalf_s *lower,
                              clkcnt_t delta)
{
  sim_oneshot_set_timer((struct sim_oneshot_lowerhalf_s *)lower,
                        host_gettime(false) + delta);
}

static void sim_oneshot_cancel(struct oneshot_lowerhalf_s *lower)
{
  sim_oneshot_set_timer((struct sim_oneshot_lowerhalf_s *)lower, UINT64_MAX);
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_sim_oneshot_ops =
{
  .current        = sim_oneshot_current,
  .start          = sim_oneshot_start,
  .start_absolute = sim_oneshot_start_absolute,
  .cancel         = sim_oneshot_cancel,
  .max_delay      = sim_oneshot_max_delay
};

static struct sim_oneshot_lowerhalf_s g_sim_oneshot_lowerhalf[2] =
{
    {
      .lh.ops = &g_sim_oneshot_ops,
      .node = LIST_INITIAL_VALUE(g_sim_oneshot_lowerhalf[0].node),
      .expire_time = 0,
    },

    {
      .lh.ops = &g_sim_oneshot_ops,
      .node = LIST_INITIAL_VALUE(g_sim_oneshot_lowerhalf[1].node),
      .expire_time = 0,
    }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer and return a oneshot lower half driver
 *   instance.
 *
 * Input Parameters:
 *   chan       Timer counter channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   On success, a non-NULL instance of the oneshot lower-half driver is
 *   returned.  NULL is return on any failure.
 *
 ****************************************************************************/

struct oneshot_lowerhalf_s *oneshot_initialize(int chan,
                                               uint16_t resolution)
{
  struct oneshot_lowerhalf_s *priv = &g_sim_oneshot_lowerhalf[chan].lh;

  /* Initialize the lower-half driver structure */

  oneshot_count_init(priv, NSEC_PER_SEC);

  return priv;
}

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer hardware.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  /* Since the "/dev/oneshot" has already used channel 0, we
   * use channel 1 here.
   */

  struct oneshot_lowerhalf_s *priv = oneshot_initialize(1, 0);

  VERIFY(host_inittimer());
  up_alarm_set_lowerhalf(priv);

#ifdef CONFIG_SIM_WALLTIME_SIGNAL
  int timer_irq = host_timerirq();

  /* Attach the interrupt to the NuttX logic and enable the alarm handler. */

  irq_attach(timer_irq, sim_timer_handler, NULL);
  up_enable_irq(timer_irq);
#endif
}

/****************************************************************************
 * Name: sim_timer_update
 *
 * Description:
 *   Called from the IDLE loop to fake one timer tick.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sim_timer_update(void)
{
  static uint64_t until;

  /* Wait a bit so that the timing is close to the correct rate. */

  until += NSEC_PER_TICK;
  host_sleepuntil(until);

#ifdef CONFIG_SIM_WALLTIME_SLEEP
  sim_timer_handler(0, NULL, NULL);
#endif
}
