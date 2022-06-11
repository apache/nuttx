/****************************************************************************
 * arch/arm/src/nrf52/nrf52_tim_lowerhalf.c
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
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/timer.h>

#include "arm_internal.h"
#include "nrf52_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TIMER configuration */

#define NRF52_TIMER_CC  (NRF52_TIM_CC0)
#define NRF52_TIMER_INT (NRF52_TIM_INT_COMPARE0)
#define NRF52_TIMER_RES (NRF52_TIM_WIDTH_32B)
#define NRF52_TIMER_MAX (0xffffffff)
#define NRF52_TIMER_PRE (NRF52_TIM_PRE_1000000)
#define NRF52_TIMER_PER (1000000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf52_timer_lowerhalf_s
{
  const struct timer_ops_s *ops;      /* Lower half operations */
  struct nrf52_tim_dev_s   *tim;      /* nrf52 timer driver */
  tccb_t                    callback; /* Current user interrupt callback */
  void                     *arg;      /* Argument passed to upper half callback */
  bool                      started;  /* True: Timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nrf52_timer_handler(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static int nrf52_timer_start(struct timer_lowerhalf_s *lower);
static int nrf52_timer_stop(struct timer_lowerhalf_s *lower);
static int nrf52_timer_getstatus(struct timer_lowerhalf_s *lower,
                                 struct timer_status_s *status);
static int nrf52_timer_settimeout(struct timer_lowerhalf_s *lower,
                                  uint32_t timeout);
static void nrf52_timer_setcallback(struct timer_lowerhalf_s *lower,
                                    tccb_t callback, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_nrf52_timer_ops =
{
  .start       = nrf52_timer_start,
  .stop        = nrf52_timer_stop,
  .getstatus   = nrf52_timer_getstatus,
  .settimeout  = nrf52_timer_settimeout,
  .setcallback = nrf52_timer_setcallback,
  .ioctl       = NULL,
};

#ifdef CONFIG_NRF52_TIMER0
/* TIMER0 lower-half */

static struct nrf52_timer_lowerhalf_s g_nrf52_timer0_lowerhalf =
{
  .ops = &g_nrf52_timer_ops,
};
#endif

#ifdef CONFIG_NRF52_TIMER1
/* TIMER1 lower-half */

static struct nrf52_timer_lowerhalf_s g_nrf52_timer1_lowerhalf =
{
  .ops = &g_nrf52_timer_ops,
};
#endif

#ifdef CONFIG_NRF52_TIMER2
/* TIMER2 lower-half */

static struct nrf52_timer_lowerhalf_s g_nrf52_timer2_lowerhalf =
{
  .ops = &g_nrf52_timer_ops,
};
#endif

#ifdef CONFIG_NRF52_TIMER3
/* TIMER3 lower-half */

static struct nrf52_timer_lowerhalf_s g_nrf52_timer3_lowerhalf =
{
  .ops = &g_nrf52_timer_ops,
};
#endif

#ifdef CONFIG_NRF52_TIMER4
/* TIMER4 lower-half */

static struct nrf52_timer_lowerhalf_s g_nrf52_timer4_lowerhalf =
{
  .ops = &g_nrf52_timer_ops,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_timer_handler
 *
 * Description:
 *   Timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int nrf52_timer_handler(int irq, void *context, void *arg)
{
  struct nrf52_timer_lowerhalf_s *priv =
    (struct nrf52_timer_lowerhalf_s *)arg;
  uint32_t next_interval_us = 0;

  NRF52_TIM_ACKINT(priv->tim, NRF52_TIMER_INT);

  if (priv->callback(&next_interval_us, priv->arg))
    {
      if (next_interval_us > 0)
        {
          NRF52_TIM_SETCC(priv->tim, NRF52_TIMER_CC, next_interval_us);
        }
    }
  else
    {
      nrf52_timer_stop((struct timer_lowerhalf_s *)priv);
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_timer_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nrf52_timer_start(struct timer_lowerhalf_s *lower)
{
  struct nrf52_timer_lowerhalf_s *priv =
    (struct nrf52_timer_lowerhalf_s *)lower;
  int ret = OK;

  DEBUGASSERT(priv);

  if (priv->started == true)
    {
      /* Return EBUSY to indicate that the timer was already running */

      ret = -EBUSY;
      goto errout;
    }

  /* Configure TIMER mode and resolution */

  NRF52_TIM_CONFIGURE(priv->tim, NRF52_TIM_MODE_TIMER, NRF52_TIMER_RES);

  /* Clear counter on COMPARE event */

  NRF52_TIM_SHORTS(priv->tim, NRF52_TIM_SHORT_COMPARE_CLEAR,
                   NRF52_TIMER_INT, true);

  /* Configure prescaler */

  NRF52_TIM_SETPRE(priv->tim, NRF52_TIMER_PRE);

  /* Configure callback */

  if (priv->callback != NULL)
    {
      NRF52_TIM_SETISR(priv->tim, nrf52_timer_handler, priv);
      NRF52_TIM_ENABLEINT(priv->tim, NRF52_TIMER_INT);
    }

  NRF52_TIM_CLEAR(priv->tim);
  NRF52_TIM_START(priv->tim);

  priv->started = true;

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_timer_stop
 *
 * Description:
 *   Stop the timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nrf52_timer_stop(struct timer_lowerhalf_s *lower)
{
  struct nrf52_timer_lowerhalf_s *priv =
    (struct nrf52_timer_lowerhalf_s *)lower;
  int ret = OK;

  DEBUGASSERT(priv);

  if (priv->started == false)
    {
      /* Return ENODEV to indicate that the timer was not running */

      ret = -ENODEV;
      goto errout;
    }

  NRF52_TIM_DISABLEINT(priv->tim, NRF52_TIMER_INT);
  NRF52_TIM_SETISR(priv->tim, NULL, NULL);
  NRF52_TIM_STOP(priv->tim);

  priv->started = false;

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_timer_getstatus
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

static int nrf52_timer_getstatus(struct timer_lowerhalf_s *lower,
                                 struct timer_status_s *status)
{
  struct nrf52_timer_lowerhalf_s *priv =
    (struct nrf52_timer_lowerhalf_s *)lower;

  DEBUGASSERT(priv);
  DEBUGASSERT(status);

  /* Return the status bit */

  status->flags = 0;

  if (priv->started == true)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  /* TODO: timeout and time left */

  status->timeout  = 0;
  status->timeleft = 0;

  return OK;
}

/****************************************************************************
 * Name: nrf52_timer_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of
 *             the "lower-half" driver state structure.
 *   timeout - The new timeout value in microseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nrf52_timer_settimeout(struct timer_lowerhalf_s *lower,
                                  uint32_t timeout)
{
  struct nrf52_timer_lowerhalf_s *priv =
    (struct nrf52_timer_lowerhalf_s *)lower;
  uint64_t cc  = 0;
  int      ret = OK;

  DEBUGASSERT(priv);

  if (priv->started == true)
    {
      ret = -EPERM;
      goto errout;
    }

  if (timeout > NRF52_TIMER_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  cc = (timeout / 1000000) * NRF52_TIMER_PER;
  NRF52_TIM_SETCC(priv->tim, NRF52_TIMER_CC, cc);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_setcallback
 *
 * Description:
 *   Call this user provided timeout handler.
 *
 * Input Parameters:
 *   lower    - A pointer the publicly visible representation of
 *              the "lower-half" driver state structure.
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

static void nrf52_timer_setcallback(struct timer_lowerhalf_s *lower,
                                    tccb_t callback, void *arg)
{
  struct nrf52_timer_lowerhalf_s *priv =
    (struct nrf52_timer_lowerhalf_s *)lower;
  irqstate_t flags;

  DEBUGASSERT(priv);

  flags = enter_critical_section();

  /* Save the new callback */

  priv->callback = callback;
  priv->arg      = arg;

  if (callback != NULL && priv->started == true)
    {
      NRF52_TIM_SETISR(priv->tim, nrf52_timer_handler, priv);
      NRF52_TIM_ENABLEINT(priv->tim, NRF52_TIMER_INT);
    }
  else
    {
      NRF52_TIM_DISABLEINT(priv->tim, NRF52_TIMER_INT);
      NRF52_TIM_SETISR(priv->tim, NULL, NULL);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_timer_initialize
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

int nrf52_timer_initialize(const char *devpath, uint8_t timer)
{
  struct nrf52_timer_lowerhalf_s *lower = NULL;
  void                           *drvr  = NULL;
  int                             ret   = OK;

  DEBUGASSERT(devpath);

  switch (timer)
    {
#ifdef CONFIG_NRF52_TIMER0
      case 0:
        {
          lower = &g_nrf52_timer0_lowerhalf;
          break;
        }
#endif

#ifdef CONFIG_NRF52_TIMER1
      case 1:
        {
          lower = &g_nrf52_timer1_lowerhalf;
          break;
        }
#endif

#ifdef CONFIG_NRF52_TIMER2
      case 2:
        {
          lower = &g_nrf52_timer2_lowerhalf;
          break;
        }
#endif

#ifdef CONFIG_NRF52_TIMER3
      case 3:
        {
          lower = &g_nrf52_timer3_lowerhalf;
          break;
        }
#endif

#ifdef CONFIG_NRF52_TIMER4
      case 4:
        {
          lower = &g_nrf52_timer4_lowerhalf;
          break;
        }
#endif

      default:
        {
          ret = -ENODEV;
          goto errout;
        }
    }

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->callback = NULL;
  lower->tim      = nrf52_tim_init(timer);

  if (lower->tim == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Register the timer driver as /dev/timerX.  The returned value from
   * timer_register is a handle that could be used with timer_unregister().
   * REVISIT: The returned handle is discard here.
   */

  drvr = timer_register(devpath, (struct timer_lowerhalf_s *)lower);
  if (drvr == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the timer driver (such as if the
       * 'depath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      ret = -EEXIST;
      goto errout;
    }

errout:
  return ret;
}
