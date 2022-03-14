/****************************************************************************
 * arch/sparc/src/bm3803/bm3803_tim_lowerhalf.c
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

#include <sys/types.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>

#include <arch/board/board.h>

#include "bm3803_tim.h"

#if defined(CONFIG_TIMER) && \
    (defined(CONFIG_BM3803_TIM1)  || defined(CONFIG_BM3803_TIM2))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BM3803_TIM1_RES   24
#define BM3803_TIM2_RES   24

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct bm3803_lowerhalf_s
{
  FAR const struct timer_ops_s *ops;        /* Lower half operations */
  FAR struct bm3803_tim_dev_s *tim;         /* stm32 timer driver */
  tccb_t                        callback;   /* Current upper half interrupt
                                             * callback */
  FAR void                     *arg;        /* Argument passed to upper half
                                             * callback */
  bool                          started;    /* True: Timer has been started */
  const uint8_t                 resolution; /* Number of bits in the timer
                                             * (16 or 32 bits) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupt handling *******************************************************/

static int bm3803_timer_handler(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static int bm3803_start(FAR struct timer_lowerhalf_s *lower);
static int bm3803_stop(FAR struct timer_lowerhalf_s *lower);
static int bm3803_getstatus(FAR struct timer_lowerhalf_s *lower,
                             FAR struct timer_status_s *status);
static int bm3803_settimeout(FAR struct timer_lowerhalf_s *lower,
                              uint32_t timeout);
static void bm3803_setcallback(FAR struct timer_lowerhalf_s *lower,
                                tccb_t callback, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = bm3803_start,
  .stop        = bm3803_stop,
  .getstatus   = bm3803_getstatus,
  .settimeout  = bm3803_settimeout,
  .setcallback = bm3803_setcallback,
  .ioctl       = NULL,
};

#ifdef CONFIG_BM3803_TIM1
static struct bm3803_lowerhalf_s g_tim1_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = BM3803_TIM1_RES,
};
#endif

#ifdef CONFIG_BM3803_TIM2
static struct bm3803_lowerhalf_s g_tim2_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = BM3803_TIM2_RES,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bm3803_timer_handler
 *
 * Description:
 *   timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int bm3803_timer_handler(int irq, void *context, void *arg)
{
  FAR struct bm3803_lowerhalf_s *lower =
                                       (FAR struct bm3803_lowerhalf_s *)arg;
  uint32_t next_interval_us = 0;

  BM3803_TIM_ACKINT(lower->tim, 0);

  if (lower->callback(&next_interval_us, lower->arg))
    {
      if (next_interval_us > 0)
        {
          BM3803_TIM_SETPERIOD(lower->tim, next_interval_us);
        }
    }
  else
    {
      bm3803_stop((struct timer_lowerhalf_s *)lower);
    }

  return OK;
}

/****************************************************************************
 * Name: bm3803_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower- A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bm3803_start(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct bm3803_lowerhalf_s *priv =
                                      (FAR struct bm3803_lowerhalf_s *)lower;

  if (!priv->started)
    {
      BM3803_TIM_SETMODE(priv->tim, BM3803_TIM_MODE_DOWN);

      if (priv->callback != NULL)
        {
          BM3803_TIM_SETISR(priv->tim, bm3803_timer_handler, priv, 0);
        }

      priv->started = true;
      return OK;
    }

  /* Return EBUSY to indicate that the timer was already running */

  return -EBUSY;
}

/****************************************************************************
 * Name: bm3803_stop
 *
 * Description:
 *   Stop the timer
 *
 * Input Parameters:
 *   lower- A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bm3803_stop(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct bm3803_lowerhalf_s *priv =
                                      (FAR struct bm3803_lowerhalf_s *)lower;

  if (priv->started)
    {
      BM3803_TIM_SETMODE(priv->tim, BM3803_TIM_MODE_DISABLED);
      BM3803_TIM_SETISR(priv->tim, NULL, NULL, 0);
      priv->started = false;
      return OK;
    }

  /* Return ENODEV to indicate that the timer was not running */

  return -ENODEV;
}

/****************************************************************************
 * Name: bm3803_getstatus
 *
 * Description:
 *   get timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the "lower-
 *            half" driver state structure.
 *   status - The location to return the status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bm3803_getstatus(FAR struct timer_lowerhalf_s *lower,
                             FAR struct timer_status_s *status)
{
  FAR struct bm3803_lowerhalf_s *priv =
                                      (FAR struct bm3803_lowerhalf_s *)lower;
  uint64_t maxtimeout;
  uint32_t timeout;
  uint32_t clock;
  uint32_t period;
  uint32_t clock_factor;

  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = 0;
  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  /* Get timeout */

  maxtimeout = (1 << priv->resolution) - 1;
  clock      = BM3803_TIM_GETCLOCK(priv->tim);
  period     = BM3803_TIM_GETPERIOD(priv->tim);

  if (clock == 1000000)
    {
      timeout = period;
    }
  else
    {
      timeout = (maxtimeout * 1000000) / clock;
    }

  status->timeout = timeout;

  /* Get the time remaining until the timer expires (in microseconds) */

  clock_factor     = (clock == 1000000) ? 1 : (clock / 1000000);
  status->timeleft = (timeout - BM3803_TIM_GETCOUNTER(priv->tim)) *
                     clock_factor;
  return OK;
}

/****************************************************************************
 * Name: bm3803_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower- A pointer the publicly visible representation of the "lower-half"
 *             driver state structure.
 *   timeout - The new timeout value in microseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bm3803_settimeout(FAR struct timer_lowerhalf_s *lower,
                              uint32_t timeout)
{
  FAR struct bm3803_lowerhalf_s *priv =
                                      (FAR struct bm3803_lowerhalf_s *)lower;
  uint64_t maxtimeout;

  if (priv->started)
    {
      return -EPERM;
    }

  maxtimeout = (1 << priv->resolution) - 1;
  if (timeout > maxtimeout)
    {
      uint64_t freq = (maxtimeout * 1000000) / timeout;
      BM3803_TIM_SETCLOCK(priv->tim, freq);
      BM3803_TIM_SETPERIOD(priv->tim, maxtimeout);
    }
  else
    {
      BM3803_TIM_SETCLOCK(priv->tim, 1000000);
      BM3803_TIM_SETPERIOD(priv->tim, timeout);
    }

  return OK;
}

/****************************************************************************
 * Name: bm3803_sethandler
 *
 * Description:
 *   Call this user provided timeout handler.
 *
 * Input Parameters:
 *   lower- A pointer the publicly visible representation of the "lower-half"
 *              driver state structure.
 *   callback - The new timer expiration function pointer.  If this
 *              function pointer is NULL, then the reset-on-expiration
 *              behavior is restored,
 *   arg      - Argument that will be provided in the callback
 *
 * Returned Value:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static void bm3803_setcallback(FAR struct timer_lowerhalf_s *lower,
                                tccb_t callback, FAR void *arg)
{
  FAR struct bm3803_lowerhalf_s *priv =
                                      (FAR struct bm3803_lowerhalf_s *)lower;
  irqstate_t flags = enter_critical_section();

  /* Save the new callback */

  priv->callback = callback;
  priv->arg     = arg;

  if (callback != NULL && priv->started)
    {
      BM3803_TIM_SETISR(priv->tim, bm3803_timer_handler, priv, 0);
    }
  else
    {
      BM3803_TIM_SETISR(priv->tim, NULL, NULL, 0);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bm3803_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *     form /dev/timer0
 *   timer - the timer's number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int bm3803_timer_initialize(FAR const char *devpath, int timer)
{
  FAR struct bm3803_lowerhalf_s *lower;

  switch (timer)
    {
#ifdef CONFIG_BM3803_TIM1
      case 1:
        lower = &g_tim1_lowerhalf;
        break;
#endif

#ifdef CONFIG_BM3803_TIM2
      case 2:
        lower = &g_tim2_lowerhalf;
        break;
#endif

      default:
        return -ENODEV;
    }

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->callback = NULL;
  lower->tim      = bm3803_tim_init(timer);

  if (lower->tim == NULL)
    {
      return -EINVAL;
    }

  /* Register the timer driver as /dev/timerX.  The returned value from
   * timer_register is a handle that could be used with timer_unregister().
   * REVISIT: The returned handle is discard here.
   */

  FAR void *drvr = timer_register(devpath,
                                  (FAR struct timer_lowerhalf_s *)lower);
  if (drvr == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the timer driver (such as if the
       * 'depath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      return -EEXIST;
    }

  return OK;
}

#endif /* CONFIG_TIMER */
