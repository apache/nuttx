/****************************************************************************
 * arch/arm/src/common/stm32/stm32_lptim_lowerhalf.c
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
#include <nuttx/timers/timer.h>

#include "stm32_lptim.h"

#if defined(CONFIG_TIMER) && defined(CONFIG_STM32_LPTIM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Counts of the 16-bit counter in one period (ARR + 1) */

#define LPTIM_PERIOD_MIN  2
#define LPTIM_PERIOD_MAX  0x10000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_lptim_lowerhalf_s
{
  const struct timer_ops_s *ops;      /* Lower half operations */
  struct stm32_lptim_dev_s *lptim;    /* LPTIM driver */
  tccb_t                    callback; /* Upper half callback */
  void                     *arg;      /* Argument of the callback */
  bool                      started;  /* Timer is running */
  uint32_t                  timeout;  /* Period in microseconds */
  uint32_t                  fmin;     /* Slowest counter clock */
  uint32_t                  freq;     /* Counter clock frequency */
  uint32_t                  period;   /* Counts in one period */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_lptim_start(struct timer_lowerhalf_s *lower);
static int stm32_lptim_stop(struct timer_lowerhalf_s *lower);
static int stm32_lptim_getstatus(struct timer_lowerhalf_s *lower,
                                 struct timer_status_s *status);
static int stm32_lptim_settimeout(struct timer_lowerhalf_s *lower,
                                  uint32_t timeout);
static void stm32_lptim_setcallback(struct timer_lowerhalf_s *lower,
                                    tccb_t callback, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct timer_ops_s g_lptim_timer_ops =
{
  .start       = stm32_lptim_start,
  .stop        = stm32_lptim_stop,
  .getstatus   = stm32_lptim_getstatus,
  .settimeout  = stm32_lptim_settimeout,
  .setcallback = stm32_lptim_setcallback,
  .ioctl       = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_lptim_run
 *
 * Description:
 *   Start counting continuously with the configured period.
 *
 ****************************************************************************/

static void stm32_lptim_run(struct stm32_lptim_lowerhalf_s *priv)
{
  STM32_LPTIM_ENABLE(priv->lptim, true);
  STM32_LPTIM_SETPERIOD(priv->lptim, priv->period - 1);
  STM32_LPTIM_START(priv->lptim, STM32_LPTIM_MODE_CONTINUOUS);
}

/****************************************************************************
 * Name: stm32_lptim_handler
 ****************************************************************************/

static int stm32_lptim_handler(int irq, void *context, void *arg)
{
  struct stm32_lptim_lowerhalf_s *priv = arg;
  uint32_t next = 0;
  uint64_t period;

  STM32_LPTIM_ACKINT(priv->lptim, LPTIM_ISR_ARRM);

  if (priv->callback(&next, priv->arg))
    {
      if (next > 0 && next != priv->timeout)
        {
          period = (uint64_t)next * priv->freq / 1000000;
          if (period >= LPTIM_PERIOD_MIN && period <= LPTIM_PERIOD_MAX)
            {
              STM32_LPTIM_SETPERIOD(priv->lptim, period - 1);
              priv->timeout = next;
              priv->period  = period;
            }
        }
    }
  else
    {
      stm32_lptim_stop((struct timer_lowerhalf_s *)priv);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_lptim_start
 ****************************************************************************/

static int stm32_lptim_start(struct timer_lowerhalf_s *lower)
{
  struct stm32_lptim_lowerhalf_s *priv =
    (struct stm32_lptim_lowerhalf_s *)lower;

  if (priv->started)
    {
      return -EBUSY;
    }

  if (priv->period == 0)
    {
      return -EPERM;
    }

  if (priv->callback != NULL)
    {
      STM32_LPTIM_SETISR(priv->lptim, stm32_lptim_handler, priv, 0);
      STM32_LPTIM_ACKINT(priv->lptim, LPTIM_ISR_ARRM);
      STM32_LPTIM_ENABLEINT(priv->lptim, LPTIM_ISR_ARRM);
    }

  stm32_lptim_run(priv);
  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: stm32_lptim_stop
 ****************************************************************************/

static int stm32_lptim_stop(struct timer_lowerhalf_s *lower)
{
  struct stm32_lptim_lowerhalf_s *priv =
    (struct stm32_lptim_lowerhalf_s *)lower;

  if (!priv->started)
    {
      return -ENODEV;
    }

  STM32_LPTIM_ENABLE(priv->lptim, false);
  STM32_LPTIM_DISABLEINT(priv->lptim, LPTIM_ISR_ARRM);
  STM32_LPTIM_SETISR(priv->lptim, NULL, NULL, 0);
  priv->started = false;
  return OK;
}

/****************************************************************************
 * Name: stm32_lptim_getstatus
 ****************************************************************************/

static int stm32_lptim_getstatus(struct timer_lowerhalf_s *lower,
                                 struct timer_status_s *status)
{
  struct stm32_lptim_lowerhalf_s *priv =
    (struct stm32_lptim_lowerhalf_s *)lower;
  uint32_t count;

  status->flags = 0;
  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback != NULL)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  status->timeout  = priv->timeout;
  status->timeleft = 0;

  if (priv->started && priv->freq != 0)
    {
      count = STM32_LPTIM_GETCOUNTER(priv->lptim);
      if (count < priv->period)
        {
          status->timeleft = (uint64_t)(priv->period - count) * 1000000 /
                             priv->freq;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_lptim_settimeout
 ****************************************************************************/

static int stm32_lptim_settimeout(struct timer_lowerhalf_s *lower,
                                  uint32_t timeout)
{
  struct stm32_lptim_lowerhalf_s *priv =
    (struct stm32_lptim_lowerhalf_s *)lower;
  uint64_t freq;
  uint64_t period;
  int ret;

  if (priv->started)
    {
      return -EPERM;
    }

  if (timeout == 0)
    {
      return -EINVAL;
    }

  /* The period must fit in the counter at the slowest counter clock */

  if ((uint64_t)timeout * priv->fmin / 1000000 > LPTIM_PERIOD_MAX)
    {
      tmrerr("ERROR: timeout too long for the timer\n");
      return -ERANGE;
    }

  /* The highest counter clock, set by the prescaler, that fits the period
   * into the 16-bit counter.
   */

  freq = (uint64_t)LPTIM_PERIOD_MAX * 1000000 / timeout;
  if (freq > UINT32_MAX)
    {
      freq = UINT32_MAX;
    }

  ret = STM32_LPTIM_SETCLOCK(priv->lptim, (uint32_t)freq);
  if (ret <= 0)
    {
      return ret < 0 ? ret : -EINVAL;
    }

  period = (uint64_t)timeout * ret / 1000000;
  if (period < LPTIM_PERIOD_MIN)
    {
      period = LPTIM_PERIOD_MIN;
    }

  priv->timeout = timeout;
  priv->freq    = ret;
  priv->period  = period;
  return OK;
}

/****************************************************************************
 * Name: stm32_lptim_setcallback
 ****************************************************************************/

static void stm32_lptim_setcallback(struct timer_lowerhalf_s *lower,
                                    tccb_t callback, void *arg)
{
  struct stm32_lptim_lowerhalf_s *priv =
    (struct stm32_lptim_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();

  priv->callback = callback;
  priv->arg      = arg;

  if (priv->started)
    {
      /* The interrupt enable bits can only be changed while the timer is
       * disabled.
       */

      STM32_LPTIM_ENABLE(priv->lptim, false);

      if (callback != NULL)
        {
          STM32_LPTIM_SETISR(priv->lptim, stm32_lptim_handler, priv, 0);
          STM32_LPTIM_ACKINT(priv->lptim, LPTIM_ISR_ARRM);
          STM32_LPTIM_ENABLEINT(priv->lptim, LPTIM_ISR_ARRM);
        }
      else
        {
          STM32_LPTIM_DISABLEINT(priv->lptim, LPTIM_ISR_ARRM);
          STM32_LPTIM_SETISR(priv->lptim, NULL, NULL, 0);
        }

      stm32_lptim_run(priv);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_lptim_timer_initialize
 *
 * Description:
 *   Register LPTIMn as a timer device.  The counter runs on the LPTIM
 *   kernel clock, STM32_LPTIMn_CLKIN in board.h.
 *
 * Input Parameters:
 *   devpath - The full path to the timer device, e.g. "/dev/timer1"
 *   lptim   - The LPTIM instance number
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_lptim_timer_initialize(const char *devpath, int lptim)
{
  struct stm32_lptim_lowerhalf_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(struct stm32_lptim_lowerhalf_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->lptim = stm32_lptim_init(lptim);
  if (priv->lptim == NULL)
    {
      kmm_free(priv);
      return -ENODEV;
    }

  /* The slowest counter clock gives the longest timeout */

  ret = STM32_LPTIM_SETCLOCK(priv->lptim, 1);
  if (ret <= 0)
    {
      tmrerr("ERROR: LPTIM%d kernel clock unknown\n", lptim);
      stm32_lptim_deinit(priv->lptim);
      kmm_free(priv);
      return -EINVAL;
    }

  priv->fmin = ret;
  priv->ops  = &g_lptim_timer_ops;

  if (timer_register(devpath, (struct timer_lowerhalf_s *)priv) == NULL)
    {
      stm32_lptim_deinit(priv->lptim);
      kmm_free(priv);
      return -EEXIST;
    }

  return OK;
}

#endif /* CONFIG_TIMER && CONFIG_STM32_LPTIM */
