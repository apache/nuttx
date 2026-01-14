/****************************************************************************
 * arch/arm/src/at32/at32_tim_lowerhalf.c
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

#include <sys/types.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>

#include <arch/board/board.h>

#include "at32_tim.h"

#if defined(CONFIG_TIMER) && \
    (defined(CONFIG_AT32_TIM1)  || defined(CONFIG_AT32_TIM2)  || \
     defined(CONFIG_AT32_TIM3)  || defined(CONFIG_AT32_TIM4)  || \
     defined(CONFIG_AT32_TIM5)  || defined(CONFIG_AT32_TIM6)  || \
     defined(CONFIG_AT32_TIM7)  || defined(CONFIG_AT32_TIM8)  || \
     defined(CONFIG_AT32_TIM9)  || defined(CONFIG_AT32_TIM10) || \
     defined(CONFIG_AT32_TIM11) || defined(CONFIG_AT32_TIM12) || \
     defined(CONFIG_AT32_TIM13) || defined(CONFIG_AT32_TIM14) || \
     defined(CONFIG_AT32_TIM20))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AT32_TIM1_RES   16

#define AT32_TIM2_RES   32

#define AT32_TIM3_RES   16
#define AT32_TIM4_RES   16

#define AT32_TIM5_RES   32

#define AT32_TIM6_RES   16
#define AT32_TIM7_RES   16
#define AT32_TIM8_RES   16
#define AT32_TIM9_RES   16
#define AT32_TIM10_RES  16
#define AT32_TIM11_RES  16
#define AT32_TIM12_RES  16
#define AT32_TIM13_RES  16
#define AT32_TIM14_RES  16
#define AT32_TIM20_RES  16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct at32_lowerhalf_s
{
  const struct timer_ops_s *ops;        /* Lower half operations */
  struct at32_tim_dev_s   *tim;         /* at32 timer driver */
  tccb_t                    callback;   /* Current user interrupt callback */
  void                     *arg;        /* Argument passed to upper half callback */
  bool                      started;    /* True: Timer has been started */
  const uint8_t             resolution; /* Number of bits in the timer (16 or 32 bits) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int at32_timer_handler(int irq, void * context, void * arg);

/* "Lower half" driver methods **********************************************/

static int at32_start(struct timer_lowerhalf_s *lower);
static int at32_stop(struct timer_lowerhalf_s *lower);
static int at32_settimeout(struct timer_lowerhalf_s *lower,
                            uint32_t timeout);
static void at32_setcallback(struct timer_lowerhalf_s *lower,
                              tccb_t callback, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = at32_start,
  .stop        = at32_stop,
  .getstatus   = NULL,
  .settimeout  = at32_settimeout,
  .setcallback = at32_setcallback,
  .ioctl       = NULL,
};

#ifdef CONFIG_AT32_TIM1
static struct at32_lowerhalf_s g_tim1_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM1_RES,
};
#endif

#ifdef CONFIG_AT32_TIM2
static struct at32_lowerhalf_s g_tim2_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM2_RES,
};
#endif

#ifdef CONFIG_AT32_TIM3
static struct at32_lowerhalf_s g_tim3_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM3_RES,
};
#endif

#ifdef CONFIG_AT32_TIM4
static struct at32_lowerhalf_s g_tim4_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM4_RES,
};
#endif

#ifdef CONFIG_AT32_TIM5
static struct at32_lowerhalf_s g_tim5_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM5_RES,
};
#endif

#ifdef CONFIG_AT32_TIM6
static struct at32_lowerhalf_s g_tim6_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM6_RES,
};
#endif

#ifdef CONFIG_AT32_TIM7
static struct at32_lowerhalf_s g_tim7_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM7_RES,
};
#endif

#ifdef CONFIG_AT32_TIM8
static struct at32_lowerhalf_s g_tim8_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM8_RES,
};
#endif

#ifdef CONFIG_AT32_TIM9
static struct at32_lowerhalf_s g_tim9_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM9_RES,
};
#endif

#ifdef CONFIG_AT32_TIM10
static struct at32_lowerhalf_s g_tim10_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM10_RES,
};
#endif

#ifdef CONFIG_AT32_TIM11
static struct at32_lowerhalf_s g_tim11_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM11_RES,
};
#endif

#ifdef CONFIG_AT32_TIM12
static struct at32_lowerhalf_s g_tim12_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM12_RES,
};
#endif

#ifdef CONFIG_AT32_TIM13
static struct at32_lowerhalf_s g_tim13_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM13_RES,
};
#endif

#ifdef CONFIG_AT32_TIM14
static struct at32_lowerhalf_s g_tim14_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM14_RES,
};
#endif

#ifdef CONFIG_AT32_TIM20
static struct at32_lowerhalf_s g_tim20_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = AT32_TIM20_RES,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_timer_handler
 *
 * Description:
 *   timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int at32_timer_handler(int irq, void * context, void * arg)
{
  struct at32_lowerhalf_s *lower = (struct at32_lowerhalf_s *) arg;
  uint32_t next_interval_us = 0;

  AT32_TIM_ACKINT(lower->tim, ATIM_DIER_UIE);

  if (lower->callback(&next_interval_us, lower->arg))
    {
      if (next_interval_us > 0)
        {
          AT32_TIM_SETPERIOD(lower->tim, next_interval_us);
        }
    }
  else
    {
      at32_stop((struct timer_lowerhalf_s *)lower);
    }

  return OK;
}

/****************************************************************************
 * Name: at32_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int at32_start(struct timer_lowerhalf_s *lower)
{
  struct at32_lowerhalf_s *priv = (struct at32_lowerhalf_s *)lower;

  if (!priv->started)
    {
      AT32_TIM_SETMODE(priv->tim, AT32_TIM_MODE_UP);

      if (priv->callback != NULL)
        {
          AT32_TIM_SETISR(priv->tim, at32_timer_handler, priv, 0);
          AT32_TIM_ENABLEINT(priv->tim, ATIM_DIER_UIE);
        }

      priv->started = true;
      return OK;
    }

  /* Return EBUSY to indicate that the timer was already running */

  return -EBUSY;
}

/****************************************************************************
 * Name: at32_stop
 *
 * Description:
 *   Stop the timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int at32_stop(struct timer_lowerhalf_s *lower)
{
  struct at32_lowerhalf_s *priv = (struct at32_lowerhalf_s *)lower;

  if (priv->started)
    {
      AT32_TIM_SETMODE(priv->tim, AT32_TIM_MODE_DISABLED);
      AT32_TIM_DISABLEINT(priv->tim, ATIM_DIER_UIE);
      AT32_TIM_SETISR(priv->tim, NULL, NULL, 0);
      priv->started = false;
      return OK;
    }

  /* Return ENODEV to indicate that the timer was not running */

  return -ENODEV;
}

/****************************************************************************
 * Name: at32_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   timeout - The new timeout value in microseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int at32_settimeout(struct timer_lowerhalf_s *lower,
                            uint32_t timeout)
{
  struct at32_lowerhalf_s *priv = (struct at32_lowerhalf_s *)lower;
  uint64_t maxtimeout;

  if (priv->started)
    {
      return -EPERM;
    }

  maxtimeout = (1 << priv->resolution) - 1;
  if (timeout > maxtimeout)
    {
      uint64_t freq = (maxtimeout * 1000000) / timeout;
      AT32_TIM_SETCLOCK(priv->tim, freq);
      AT32_TIM_SETPERIOD(priv->tim, maxtimeout);
    }
  else
    {
      AT32_TIM_SETCLOCK(priv->tim, 1000000);
      AT32_TIM_SETPERIOD(priv->tim, timeout);
    }

  return OK;
}

/****************************************************************************
 * Name: at32_setcallback
 *
 * Description:
 *   Call this user provided timeout callback.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the
 *                "lower-half" driver state structure.
 *   callback - The new timer expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *  arg          - Argument that will be provided in the callback
 *
 * Returned Value:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static void at32_setcallback(struct timer_lowerhalf_s *lower,
                              tccb_t callback, void *arg)
{
  struct at32_lowerhalf_s *priv = (struct at32_lowerhalf_s *)lower;

  irqstate_t flags = enter_critical_section();

  /* Save the new callback */

  priv->callback = callback;
  priv->arg      = arg;

  if (callback != NULL && priv->started)
    {
      AT32_TIM_SETISR(priv->tim, at32_timer_handler, priv, 0);
      AT32_TIM_ENABLEINT(priv->tim, ATIM_DIER_UIE);
    }
  else
    {
      AT32_TIM_DISABLEINT(priv->tim, ATIM_DIER_UIE);
      AT32_TIM_SETISR(priv->tim, NULL, NULL, 0);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_timer_initialize
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

int at32_timer_initialize(const char *devpath, int timer)
{
  struct at32_lowerhalf_s *lower;

  switch (timer)
    {
#ifdef CONFIG_AT32_TIM1
      case 1:
        lower = &g_tim1_lowerhalf;
        break;
#endif
#ifdef CONFIG_AT32_TIM2
      case 2:
        lower = &g_tim2_lowerhalf;
        break;
#endif
#ifdef CONFIG_AT32_TIM3
      case 3:
        lower = &g_tim3_lowerhalf;
        break;
#endif
#ifdef CONFIG_AT32_TIM4
      case 4:
        lower = &g_tim4_lowerhalf;
        break;
#endif
#ifdef CONFIG_AT32_TIM5
      case 5:
        lower = &g_tim5_lowerhalf;
        break;
#endif
#ifdef CONFIG_AT32_TIM6
      case 6:
        lower = &g_tim6_lowerhalf;
        break;
#endif
#ifdef CONFIG_AT32_TIM7
      case 7:
        lower = &g_tim7_lowerhalf;
        break;
#endif
#ifdef CONFIG_AT32_TIM8
      case 8:
        lower = &g_tim8_lowerhalf;
        break;
#endif
#ifdef CONFIG_AT32_TIM9
      case 9:
        lower = &g_tim9_lowerhalf;
        break;
#endif
#ifdef CONFIG_AT32_TIM10
      case 10:
        lower = &g_tim10_lowerhalf;
        break;
#endif
#ifdef CONFIG_AT32_TIM11
      case 11:
        lower = &g_tim11_lowerhalf;
        break;
#endif
#ifdef CONFIG_AT32_TIM12
      case 12:
        lower = &g_tim12_lowerhalf;
        break;
#endif
#ifdef CONFIG_AT32_TIM13
      case 13:
        lower = &g_tim13_lowerhalf;
        break;
#endif
#ifdef CONFIG_AT32_TIM14
      case 14:
        lower = &g_tim14_lowerhalf;
        break;
#endif
#ifdef CONFIG_AT32_TIM20
      case 20:
        lower = &g_tim20_lowerhalf;
        break;
#endif
      default:
        return -ENODEV;
    }

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->callback = NULL;
  lower->tim      = at32_tim_init(timer);

  if (lower->tim == NULL)
    {
      return -EINVAL;
    }

  /* Register the timer driver as /dev/timerX.  The returned value from
   * timer_register is a handle that could be used with timer_unregister().
   * REVISIT: The returned handle is discard here.
   */

  void *drvr = timer_register(devpath,
                              (struct timer_lowerhalf_s *)lower);
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
