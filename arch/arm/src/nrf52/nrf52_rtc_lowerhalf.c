/****************************************************************************
 * arch/arm/src/nrf52/nrf52_rtc_lowerhalf.h
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
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/timer.h>

#include "arm_arch.h"

#include "nrf52_rtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TIMER configuration */

#define NRF52_RTC_CC       (NRF52_RTC_CC0)
#define NRF52_RTC_INT      (NRF52_RTC_EVT_COMPARE0)
#define NRF52_RTC_MAX      (0x00ffffff)
#define NRF52_RTC_MAX_TIME (NRF52_RTC_MAX * 31)

/* Convert uS to count for f = 32768Hz, using more precision when possible:
 * 1 / 32768 ~ 30.51 uS
 * 512 / 32768 = 15625 uS
 */

#define USEC_TO_CC(t) (t > 0x7fffff ? (t / 31) : ((t * 512) / 15625))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf52_rtc_lowerhalf_s
{
  FAR const struct timer_ops_s *ops;        /* Lower half operations */
  FAR struct nrf52_rtc_dev_s   *tim;        /* nrf52 RTC driver */
  tccb_t                        callback;   /* Current user interrupt callback */
  FAR void                     *arg;        /* Argument passed to upper half callback */
  bool                          started;    /* True: Timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nrf52_rtc_handler(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static int nrf52_rtc_start(FAR struct timer_lowerhalf_s *lower);
static int nrf52_rtc_stop(FAR struct timer_lowerhalf_s *lower);
static int nrf52_rtc_getstatus(FAR struct timer_lowerhalf_s *lower,
                               FAR struct timer_status_s *status);
static int nrf52_rtc_settimeout(FAR struct timer_lowerhalf_s *lower,
                                uint32_t timeout);
static void nrf52_rtc_setcallback(FAR struct timer_lowerhalf_s *lower,
                                  tccb_t callback, FAR void *arg);
static int nrf52_rtc_maxtimeout(FAR struct timer_lowerhalf_s *lower,
                                uint32_t *maxtimeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_nrf52_timer_ops =
{
  .start       = nrf52_rtc_start,
  .stop        = nrf52_rtc_stop,
  .getstatus   = nrf52_rtc_getstatus,
  .settimeout  = nrf52_rtc_settimeout,
  .setcallback = nrf52_rtc_setcallback,
  .maxtimeout  = nrf52_rtc_maxtimeout,
  .ioctl       = NULL,
};

#ifdef CONFIG_NRF52_RTC0
/* RTC0 lower-half */

static struct nrf52_rtc_lowerhalf_s g_nrf52_rtc0_lowerhalf =
{
  .ops = &g_nrf52_timer_ops,
};
#endif

#ifdef CONFIG_NRF52_RTC1
/* RTC1 lower-half */

static struct nrf52_rtc_lowerhalf_s g_nrf52_rtc1_lowerhalf =
{
  .ops = &g_nrf52_timer_ops,
};
#endif

#ifdef CONFIG_NRF52_RTC2
/* RTC2 lower-half */

static struct nrf52_rtc_lowerhalf_s g_nrf52_rtc2_lowerhalf =
{
  .ops = &g_nrf52_timer_ops,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_rtc_handler
 *
 * Description:
 *   Timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int nrf52_rtc_handler(int irq, void *context, void *arg)
{
  FAR struct nrf52_rtc_lowerhalf_s *priv =
    (FAR struct nrf52_rtc_lowerhalf_s *)arg;
  uint32_t next_interval_us = 0;

  /* The RTC does not have shortcuts so we clear the timer manually
   * REVISIT: An alternative would be to use a PPI channel
   */

  NRF52_RTC_CLEAR(priv->tim);

  NRF52_RTC_ACKINT(priv->tim, NRF52_RTC_INT);

  if (priv->callback(&next_interval_us, priv->arg))
    {
      if (next_interval_us > 0)
        {
          NRF52_RTC_SETCC(priv->tim, NRF52_RTC_CC,
                          USEC_TO_CC(next_interval_us));
        }
    }
  else
    {
      nrf52_rtc_stop((struct timer_lowerhalf_s *)priv);
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_rtc_start
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

static int nrf52_rtc_start(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct nrf52_rtc_lowerhalf_s *priv =
    (FAR struct nrf52_rtc_lowerhalf_s *)lower;
  int ret = OK;

  DEBUGASSERT(priv);

  if (priv->started == true)
    {
      /* Return EBUSY to indicate that the timer was already running */

      ret = -EBUSY;
      goto errout;
    }

  /* Configure prescaler */

  NRF52_RTC_SETPRE(priv->tim, 0);

  /* Configure callback */

  if (priv->callback != NULL)
    {
      NRF52_RTC_SETISR(priv->tim, nrf52_rtc_handler, priv);
      NRF52_RTC_ENABLEINT(priv->tim, NRF52_RTC_INT);
    }

  NRF52_RTC_CLEAR(priv->tim);
  NRF52_RTC_START(priv->tim);

  priv->started = true;

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_rtc_stop
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

static int nrf52_rtc_stop(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct nrf52_rtc_lowerhalf_s *priv =
    (FAR struct nrf52_rtc_lowerhalf_s *)lower;
  int ret = OK;

  DEBUGASSERT(priv);

  if (priv->started == false)
    {
      /* Return ENODEV to indicate that the timer was not running */

      ret = -ENODEV;
      goto errout;
    }

  NRF52_RTC_DISABLEINT(priv->tim, NRF52_RTC_INT);
  NRF52_RTC_SETISR(priv->tim, NULL, NULL);
  NRF52_RTC_STOP(priv->tim);

  priv->started = false;

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_rtc_getstatus
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

static int nrf52_rtc_getstatus(FAR struct timer_lowerhalf_s *lower,
                               FAR struct timer_status_s *status)
{
  FAR struct nrf52_rtc_lowerhalf_s *priv =
    (FAR struct nrf52_rtc_lowerhalf_s *)lower;

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
 * Name: nrf52_rtc_settimeout
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

static int nrf52_rtc_settimeout(FAR struct timer_lowerhalf_s *lower,
                                uint32_t timeout)
{
  FAR struct nrf52_rtc_lowerhalf_s *priv =
    (FAR struct nrf52_rtc_lowerhalf_s *)lower;
  uint64_t cc  = 0;
  int      ret = OK;

  DEBUGASSERT(priv);

#if 0
  if (!priv->started)
    {
      ret = -EPERM;
      goto errout;
    }
#endif

  cc = USEC_TO_CC(timeout);

  if (cc > NRF52_RTC_MAX)
    {
      ret = -EINVAL;
      goto errout;
    }

  NRF52_RTC_SETCC(priv->tim, NRF52_RTC_CC, cc);
  NRF52_RTC_CLEAR(priv->tim);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_rtc_setcallback
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

static void nrf52_rtc_setcallback(FAR struct timer_lowerhalf_s *lower,
                                    tccb_t callback, FAR void *arg)
{
  FAR struct nrf52_rtc_lowerhalf_s *priv =
    (FAR struct nrf52_rtc_lowerhalf_s *)lower;
  irqstate_t flags;

  DEBUGASSERT(priv);

  flags = enter_critical_section();

  /* Save the new callback */

  priv->callback = callback;
  priv->arg      = arg;

  if (callback != NULL && priv->started == true)
    {
      NRF52_RTC_SETISR(priv->tim, nrf52_rtc_handler, priv);
      NRF52_RTC_ENABLEINT(priv->tim, NRF52_RTC_INT);
    }
  else
    {
      NRF52_RTC_DISABLEINT(priv->tim, NRF52_RTC_INT);
      NRF52_RTC_SETISR(priv->tim, NULL, NULL);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf52_rtc_maxtimeout
 ****************************************************************************/

static int nrf52_rtc_maxtimeout(FAR struct timer_lowerhalf_s *lower,
                                uint32_t *maxtimeout)
{
  *maxtimeout = NRF52_RTC_MAX_TIME;

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_rtc_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *     form /dev/timer0
 *   timer - the timer's number.
 *   drvr - Pointer where to store resulting timer_lowerhalf_s*
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int nrf52_rtc_initialize(FAR const char *devpath, uint8_t timer,
                         FAR struct timer_lowerhalf_s **lower_timer)
{
  struct nrf52_rtc_lowerhalf_s *lower = NULL;
  int                           ret   = OK;

  DEBUGASSERT(devpath);
  DEBUGASSERT(lower_timer);

  switch (timer)
    {
#ifdef CONFIG_NRF52_RTC0
      case 0:
        {
          lower = &g_nrf52_rtc0_lowerhalf;
          break;
        }
#endif

#ifdef CONFIG_NRF52_RTC1
      case 1:
        {
          lower = &g_nrf52_rtc1_lowerhalf;
          break;
        }
#endif

#ifdef CONFIG_NRF52_RTC2
      case 2:
        {
          lower = &g_nrf52_rtc2_lowerhalf;
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
  lower->tim      = nrf52_rtc_init(timer);

  if (lower->tim == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Register the timer driver as /dev/timerX */

  if (!timer_register(devpath, (FAR struct timer_lowerhalf_s *)lower))
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the timer driver (such as if the
       * 'depath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      ret = -EEXIST;
      goto errout;
    }

  *lower_timer = (FAR struct timer_lowerhalf_s *)lower;

errout:
  return ret;
}
