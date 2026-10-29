/****************************************************************************
 * arch/arm/src/common/stm32/stm32_lptim_oneshot_lowerhalf.c
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

#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/oneshot.h>

#include "stm32_lptim.h"

#if defined(CONFIG_ONESHOT) && defined(CONFIG_STM32_LPTIM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Counts of the 16-bit counter up to the autoreload match in single mode
 * (ARR)
 */

#define LPTIM_COUNT_MIN  1
#define LPTIM_COUNT_MAX  0xffff

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_lptim_oneshot_s
{
  struct oneshot_lowerhalf_s lh;     /* Must be first */
  struct stm32_lptim_dev_s  *lptim;  /* LPTIM driver */
  bool                       running;
  uint32_t                   fmin;   /* Slowest counter clock */
  uint32_t                   freq;   /* Counter clock of the delay */
  uint32_t                   count;  /* Counts of the delay */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_lptim_os_max_delay(struct oneshot_lowerhalf_s *lower,
                                    struct timespec *ts);
static int stm32_lptim_os_start(struct oneshot_lowerhalf_s *lower,
                                const struct timespec *ts);
static int stm32_lptim_os_cancel(struct oneshot_lowerhalf_s *lower,
                                 struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_lptim_oneshot_ops =
{
  .max_delay = stm32_lptim_os_max_delay,
  .start     = stm32_lptim_os_start,
  .cancel    = stm32_lptim_os_cancel,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void stm32_lptim_os_halt(struct stm32_lptim_oneshot_s *priv)
{
  STM32_LPTIM_ENABLE(priv->lptim, false);
  STM32_LPTIM_DISABLEINT(priv->lptim, LPTIM_ISR_ARRM);
  STM32_LPTIM_SETISR(priv->lptim, NULL, NULL, 0);
  priv->running = false;
}

static int stm32_lptim_os_handler(int irq, void *context, void *arg)
{
  struct stm32_lptim_oneshot_s *priv = arg;

  STM32_LPTIM_ACKINT(priv->lptim, LPTIM_ISR_ARRM);
  stm32_lptim_os_halt(priv);
  oneshot_process_callback(&priv->lh);
  return OK;
}

static int stm32_lptim_os_max_delay(struct oneshot_lowerhalf_s *lower,
                                    struct timespec *ts)
{
  struct stm32_lptim_oneshot_s *priv = (struct stm32_lptim_oneshot_s *)lower;
  uint64_t usec = (uint64_t)LPTIM_COUNT_MAX * 1000000 / priv->fmin;

  ts->tv_sec  = usec / 1000000;
  ts->tv_nsec = (usec % 1000000) * 1000;
  return OK;
}

static int stm32_lptim_os_start(struct oneshot_lowerhalf_s *lower,
                                const struct timespec *ts)
{
  struct stm32_lptim_oneshot_s *priv = (struct stm32_lptim_oneshot_s *)lower;
  irqstate_t flags;
  uint64_t usec;
  uint64_t freq;
  uint64_t count;
  int ret;

  /* Express the delay in microseconds */

  usec = (uint64_t)ts->tv_sec * 1000000 + ts->tv_nsec / 1000;

  /* The delay must fit in the counter at the slowest counter clock */

  if (usec > (uint64_t)LPTIM_COUNT_MAX * 1000000 / priv->fmin)
    {
      tmrerr("ERROR: delay too long for the timer\n");
      return -ERANGE;
    }

  flags = enter_critical_section();

  if (priv->running)
    {
      stm32_lptim_os_halt(priv);
    }

  /* The highest counter clock, set by the prescaler, that fits the delay
   * into the 16-bit counter.
   */

  freq = usec > 0 ? (uint64_t)LPTIM_COUNT_MAX * 1000000 / usec : UINT32_MAX;
  if (freq > UINT32_MAX)
    {
      freq = UINT32_MAX;
    }

  ret = STM32_LPTIM_SETCLOCK(priv->lptim, (uint32_t)freq);
  if (ret <= 0)
    {
      leave_critical_section(flags);
      return ret < 0 ? ret : -EINVAL;
    }

  count = usec * ret / 1000000;
  if (count < LPTIM_COUNT_MIN)
    {
      count = LPTIM_COUNT_MIN;
    }

  priv->freq  = ret;
  priv->count = count;

  STM32_LPTIM_SETISR(priv->lptim, stm32_lptim_os_handler, priv, 0);
  STM32_LPTIM_ACKINT(priv->lptim, LPTIM_ISR_ARRM);
  STM32_LPTIM_ENABLEINT(priv->lptim, LPTIM_ISR_ARRM);

  STM32_LPTIM_ENABLE(priv->lptim, true);
  STM32_LPTIM_SETPERIOD(priv->lptim, count);
  STM32_LPTIM_START(priv->lptim, STM32_LPTIM_MODE_SINGLE);
  priv->running = true;

  leave_critical_section(flags);
  return OK;
}

static int stm32_lptim_os_cancel(struct oneshot_lowerhalf_s *lower,
                                 struct timespec *ts)
{
  struct stm32_lptim_oneshot_s *priv = (struct stm32_lptim_oneshot_s *)lower;
  irqstate_t flags;
  uint64_t usec = 0;
  uint32_t cnt;

  flags = enter_critical_section();

  if (priv->running)
    {
      cnt = STM32_LPTIM_GETCOUNTER(priv->lptim);
      if (cnt < priv->count)
        {
          usec = (uint64_t)(priv->count - cnt) * 1000000 / priv->freq;
        }

      stm32_lptim_os_halt(priv);
    }

  leave_critical_section(flags);

  if (ts != NULL)
    {
      ts->tv_sec  = usec / 1000000;
      ts->tv_nsec = (usec % 1000000) * 1000;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_lptim_oneshot_initialize
 *
 * Description:
 *   Initialize LPTIMn as a oneshot timer lower-half.  The counter runs on
 *   the LPTIM kernel clock, STM32_LPTIMn_CLKIN in board.h.
 *
 * Input Parameters:
 *   lptim - The LPTIM instance number
 *
 * Returned Value:
 *   The oneshot lower-half on success, NULL on failure.
 *
 ****************************************************************************/

struct oneshot_lowerhalf_s *stm32_lptim_oneshot_initialize(int lptim)
{
  struct stm32_lptim_oneshot_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(struct stm32_lptim_oneshot_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->lptim = stm32_lptim_init(lptim);
  if (priv->lptim == NULL)
    {
      kmm_free(priv);
      return NULL;
    }

  /* The slowest counter clock gives the longest delay */

  ret = STM32_LPTIM_SETCLOCK(priv->lptim, 1);
  if (ret <= 0)
    {
      tmrerr("ERROR: LPTIM%d kernel clock unknown\n", lptim);
      stm32_lptim_deinit(priv->lptim);
      kmm_free(priv);
      return NULL;
    }

  priv->fmin   = ret;
  priv->lh.ops = &g_lptim_oneshot_ops;
  return &priv->lh;
}

#endif /* CONFIG_ONESHOT && CONFIG_STM32_LPTIM */
