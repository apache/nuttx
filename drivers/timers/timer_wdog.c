/****************************************************************************
 * drivers/timers/timer_wdog.c
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

#include <stdint.h>
#include <stdio.h>
#include <debug.h>
#include <sys/time.h>

#include <nuttx/atomic.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>
#include <nuttx/timers/timer.h>

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct timer_wdog_dev_s
{
  struct timer_lowerhalf_s lower;       /* Lower half operations */
  struct wdog_s            wdog;        /* Software watchdog timer */
  atomic_t                 period;      /* Period of the timer */
  spinlock_t               lock;
  tccb_t                   callback;    /* used for timer handler */
  FAR void                 *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void timer_wdog_handler(wdparm_t arg);
static int timer_wdog_start(FAR struct timer_lowerhalf_s *lower);
static int timer_wdog_stop(FAR struct timer_lowerhalf_s *lower);
static void timer_wdog_setcallback(FAR struct timer_lowerhalf_s *lower,
                                   tccb_t callback, FAR void *arg);
static int timer_wdog_tick_getstatus(FAR struct timer_lowerhalf_s *lower,
                                     FAR struct timer_status_s *status);
static int timer_wdog_tick_settimeout(FAR struct timer_lowerhalf_s *lower,
                                      uint32_t timeout);
static int timer_wdog_tick_maxtimeout(FAR struct timer_lowerhalf_s *lower,
                                      FAR uint32_t *maxtimeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct timer_ops_s g_timer_wdog_ops =
{
  .start           = timer_wdog_start,
  .stop            = timer_wdog_stop,
  .setcallback     = timer_wdog_setcallback,
  .tick_getstatus  = timer_wdog_tick_getstatus,
  .tick_settimeout = timer_wdog_tick_settimeout,
  .tick_maxtimeout = timer_wdog_tick_maxtimeout,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_wdog_start
 *
 * Description:
 *   start the timer
 *
 ****************************************************************************/

static int timer_wdog_start(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct timer_wdog_dev_s *priv = (FAR struct timer_wdog_dev_s *)lower;

  DEBUGASSERT(priv);

  if (priv->callback == NULL)
    {
      /* Return EINVAL to indicate that the timer was not configured */

      return -EINVAL;
    }

  wd_start(&priv->wdog, atomic_read(&priv->period),
           timer_wdog_handler, (wdparm_t)priv);
  return 0;
}

/****************************************************************************
 * Name: timer_wdog_stop
 *
 * Description:
 *   stop the timer
 *
 ****************************************************************************/

static int timer_wdog_stop(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct timer_wdog_dev_s *priv = (FAR struct timer_wdog_dev_s *)lower;
  irqstate_t flags;

  DEBUGASSERT(priv);

  wd_cancel(&priv->wdog);

  flags = spin_lock_irqsave(&priv->lock);
  priv->callback = NULL;
  priv->arg      = NULL;
  spin_unlock_irqrestore(&priv->lock, flags);

  return 0;
}

/****************************************************************************
 * Name: timer_wdog_setcallback
 *
 * Description:
 *   set callback function of the timer
 *
 ****************************************************************************/

static void timer_wdog_setcallback(FAR struct timer_lowerhalf_s *lower,
                                   tccb_t callback, FAR void *arg)
{
  FAR struct timer_wdog_dev_s *priv = (FAR struct timer_wdog_dev_s *)lower;
  irqstate_t flags;

  DEBUGASSERT(priv);

  flags = spin_lock_irqsave(&priv->lock);
  priv->callback = callback;
  priv->arg      = arg;
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: timer_wdog_getstatus
 *
 * Description:
 *   get status of the timer
 *
 ****************************************************************************/

static int timer_wdog_tick_getstatus(FAR struct timer_lowerhalf_s *lower,
                                     FAR struct timer_status_s *status)
{
  FAR struct timer_wdog_dev_s *priv = (FAR struct timer_wdog_dev_s *)lower;
  irqstate_t flags;

  DEBUGASSERT(priv);

  status->flags = 0;

  flags = spin_lock_irqsave(&priv->lock);
  if (WDOG_ISACTIVE(&priv->wdog))
    {
      /* TIMER is running */

      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback != NULL)
    {
      /* TIMER has a user callback function to be called when
       * expiration happens
       */

      status->flags |= TCFLAGS_HANDLER;
    }

  status->timeout  = atomic_read(&priv->period);
  status->timeleft = wd_gettime(&priv->wdog);
  spin_unlock_irqrestore(&priv->lock, flags);

  return 0;
}

/****************************************************************************
 * Name: timer_wdog_settimeout
 *
 * Description:
 *   set timeout of the timer
 *
 ****************************************************************************/

static int timer_wdog_tick_settimeout(FAR struct timer_lowerhalf_s *lower,
                                      uint32_t timeout)
{
  FAR struct timer_wdog_dev_s *priv = (FAR struct timer_wdog_dev_s *)lower;

  DEBUGASSERT(priv);

  atomic_set(&priv->period, timeout);

  return 0;
}

/****************************************************************************
 * Name: timer_wdog_tick_maxtimeout
 *
 * Description:
 *   get the max timeout value (tick) of the timer
 *
 ****************************************************************************/

static int timer_wdog_tick_maxtimeout(FAR struct timer_lowerhalf_s *lower,
                                      FAR uint32_t *maxtimeout)
{
  *maxtimeout = UINT32_MAX;
  return 0;
}

/****************************************************************************
 * Name: timer_wdog_handler
 *
 * Description:
 *   wdog handler of the timer
 *
 ****************************************************************************/

static void timer_wdog_handler(wdparm_t arg)
{
  FAR struct timer_wdog_dev_s *priv = (FAR struct timer_wdog_dev_s *)arg;

  if (priv->callback)
    {
      uint32_t period = atomic_read(&priv->period);
      uint32_t next_interval_us = period;

      if (priv->callback(&next_interval_us, priv->arg))
        {
          if (next_interval_us)
            {
              atomic_set(&priv->period, next_interval_us);
              period = next_interval_us;
            }

          wd_start_next(&priv->wdog, period,
                        timer_wdog_handler, (wdparm_t)priv);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_wdog_initialize
 *
 * Description:
 *   initialize one timer which is implemented by watchdog
 *
 ****************************************************************************/

int timer_wdog_initialize(int timer_id)
{
  FAR struct timer_wdog_dev_s *dev;
  char name[64];

  dev = kmm_zalloc(sizeof(struct timer_wdog_dev_s));
  if (dev == NULL)
    {
      tmrerr("zalloc failed for timer id = %d!\n", timer_id);
      return -ENOMEM;
    }

  dev->lower.ops = &g_timer_wdog_ops;
  dev->arg       = NULL;
  atomic_set(&dev->period, 0);

  spin_lock_init(&dev->lock);

  snprintf(name, sizeof(name), "/dev/timer%d", timer_id);

  if (timer_register(name, &dev->lower) == NULL)
    {
      return -EEXIST;
    }

  return OK;
}
