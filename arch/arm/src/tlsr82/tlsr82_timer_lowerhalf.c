/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_timer_lowerhalf.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this args for additional information regarding copyright ownership.  The
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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>

#include <arch/board/board.h>

#include "tlsr82_timer.h"
#include "tlsr82_timer_lowerhalf.h"

#if defined(CONFIG_TIMER) && \
    (defined(CONFIG_TLSR82_TIMER1) || defined(CONFIG_TLSR82_TIMER2))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct tlsr82_lowerhalf_s
{
  const struct timer_ops_s  *ops;        /* Lower half operations */
  struct tlsr82_timer_dev_s *tim;        /* tlsr82 timer driver */
  tccb_t                     callback;   /* Current user interrupt callback */
  void                      *arg;        /* Argument passed to upper half callback */
  bool                       started;    /* True: Timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int tlsr82_timer_handler(int irq, void * context, void * arg);

/* "Lower half" driver methods **********************************************/

static int tlsr82_timer_start(struct timer_lowerhalf_s *lower);
static int tlsr82_timer_stop(struct timer_lowerhalf_s *lower);
static int tlsr82_timer_getstatus(struct timer_lowerhalf_s *lower,
                                  struct timer_status_s *status);
static int tlsr82_timer_settimeout(struct timer_lowerhalf_s *lower,
                                   uint32_t timeout);
static int tlsr82_timer_maxtimeout(struct timer_lowerhalf_s *lower,
                                   uint32_t *timeout);
static void tlsr82_timer_setcallback(struct timer_lowerhalf_s *lower,
                                     tccb_t callback, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = tlsr82_timer_start,
  .stop        = tlsr82_timer_stop,
  .getstatus   = tlsr82_timer_getstatus,
  .settimeout  = tlsr82_timer_settimeout,
  .setcallback = tlsr82_timer_setcallback,
  .maxtimeout  = tlsr82_timer_maxtimeout,
  .ioctl       = NULL,
};

#ifdef CONFIG_TLSR82_TIMER1
static struct tlsr82_lowerhalf_s g_timer1_lowerhalf =
{
  .ops         = &g_timer_ops,
  .tim         = NULL,
  .started     = false,
};
#endif

#ifdef CONFIG_TLSR82_TIMER2
static struct tlsr82_lowerhalf_s g_timer2_lowerhalf =
{
  .ops         = &g_timer_ops,
  .tim         = NULL,
  .started     = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_timer_handler
 *
 * Description:
 *   timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int tlsr82_timer_handler(int irq, void * context, void * arg)
{
  struct tlsr82_lowerhalf_s *lower = (struct tlsr82_lowerhalf_s *)arg;
  struct tlsr82_timer_dev_s *timer = lower->tim;
  uint32_t next_interval_us = 0;
  uint32_t clock_mhz = 0;

  TLSR82_TIMER_ACKINT(timer);

  if (lower->callback(&next_interval_us, lower->arg))
    {
      if (next_interval_us > 0)
        {
          /* Get timer clock in MHz */

          TLSR82_TIMER_GETCLK(timer, &clock_mhz);
          TLSR82_TIMER_SETCAPTURE(timer, next_interval_us * clock_mhz);
        }
    }
  else
    {
      TLSR82_TIMER_STOP(timer);
    }

  return OK;
}

/****************************************************************************
 * Name: tlsr82_timer_start
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

static int tlsr82_timer_start(struct timer_lowerhalf_s *lower)
{
  struct tlsr82_lowerhalf_s *priv = (struct tlsr82_lowerhalf_s *)lower;
  struct tlsr82_timer_dev_s *timer = priv->tim;

  if (!priv->started)
    {
      TLSR82_TIMER_SETMODE(timer, TLSR82_TIMER_MODE_SYS_CLK);

      if (priv->callback != NULL)
        {
          TLSR82_TIMER_SETISR(timer, tlsr82_timer_handler, priv);
          TLSR82_TIMER_ENABLEINT(timer);
        }

      priv->started = true;
      return OK;
    }

  /* Return EBUSY to indicate that the timer was already running */

  return -EBUSY;
}

/****************************************************************************
 * Name: tlsr82_timer_stop
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

static int tlsr82_timer_stop(struct timer_lowerhalf_s *lower)
{
  struct tlsr82_lowerhalf_s *priv = (struct tlsr82_lowerhalf_s *)lower;
  struct tlsr82_timer_dev_s *timer = priv->tim;

  if (priv->started)
    {
      TLSR82_TIMER_DISABLEINT(timer);
      TLSR82_TIMER_STOP(timer);
      TLSR82_TIMER_SETCTR(timer, 0);
      TLSR82_TIMER_SETCAPTURE(timer, 0);
      TLSR82_TIMER_SETISR(timer, NULL, NULL);
      priv->started = false;
      return OK;
    }

  /* Return ENODEV to indicate that the timer was not running */

  return -ENODEV;
}

/****************************************************************************
 * Name: tlsr82_timer_getstatus
 *
 * Description:
 *   Get timer status.
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

static int tlsr82_timer_getstatus(struct timer_lowerhalf_s *lower,
                                  struct timer_status_s *status)
{
  struct tlsr82_lowerhalf_s *priv = (struct tlsr82_lowerhalf_s *)lower;
  int ret = OK;
  uint32_t current_counter_value;
  uint32_t capture_value;
  uint32_t clock_mhz;

  DEBUGASSERT(priv);
  DEBUGASSERT(status);

  /* Return the status bit */

  status->flags = 0;

  if (priv->started == true)
    {
      /* Timer is running */

      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback != NULL)
    {
      /* Timer has a user callback function to be called when
       * expiration happens
       */

      status->flags |= TCFLAGS_HANDLER;
    }

  /* Get the timer clock in MHz */

  TLSR82_TIMER_GETCLK(priv->tim, &clock_mhz);

  /* Get the current counter value */

  TLSR82_TIMER_GETCTR(priv->tim, &current_counter_value);

  /* Get the current configured timeout */

  TLSR82_TIMER_GETCAPTURE(priv->tim, &capture_value);

  if (clock_mhz > 0)
    {
      /* Time in microseconds */

      status->timeout  = capture_value / clock_mhz;
      status->timeleft = (capture_value - current_counter_value) /
                         clock_mhz;
    }

  return ret;
}

/****************************************************************************
 * Name: tlsr82_timer_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-
 *             half" driver state structure.
 *   timeout - The new timeout value in microseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int tlsr82_timer_settimeout(struct timer_lowerhalf_s *lower,
                                   uint32_t timeout)
{
  struct tlsr82_lowerhalf_s *priv = (struct tlsr82_lowerhalf_s *)lower;
  struct tlsr82_timer_dev_s *timer = priv->tim;
  int ret = OK;
  uint32_t maxtimeout = 0;
  uint32_t clock_mhz = 0;

  /* Exception handling  */

  if (priv->started)
    {
      tmrerr("Operation Failed, timer has started.\n");
      ret = -EPERM;
      goto errout;
    }

  ret = tlsr82_timer_maxtimeout(lower, &maxtimeout);
  if (ret < 0)
    {
      tmrerr("Operation Failed, maxtimeout get failed.\n");
      ret = -EPERM;
      goto errout;
    }

  if (timeout > maxtimeout)
    {
      tmrerr("Operation Failed, timeout value is too large,"
             "maxtimeout=%ldus\n", maxtimeout);
      ret = -EPERM;
      goto errout;
    }
  else if (timeout == 0)
    {
      tmrerr("Operation Failed, timeout is 0.\n");
      ret = -EPERM;
      goto errout;
    }

  /* Disable irq and stop timer */

  TLSR82_TIMER_DISABLEINT(timer);
  TLSR82_TIMER_STOP(timer);

  /* Get timer clock in MHZ */

  TLSR82_TIMER_GETCLK(timer, &clock_mhz);

  /* Clear timer tick register */

  TLSR82_TIMER_SETCTR(timer, 0);

  /* Set timer capture register based on timeout and frequency */

  TLSR82_TIMER_SETCAPTURE(timer, clock_mhz * timeout);

  /* Enable irq and start timer */

  TLSR82_TIMER_ENABLEINT(timer);
  TLSR82_TIMER_START(timer);

errout:
  return ret;
}

/****************************************************************************
 * Name: tlsr82_timer_maxtimeout
 *
 * Description:
 *   Get the maximum timeout value
 *
 * Input Parameters:
 *   lower       - A pointer the publicly visible representation of
 *                 the "lower-half" driver state structure.
 *   maxtimeout  - A pointer to the variable that will store the max timeout
 *                 (microseconds).
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int tlsr82_timer_maxtimeout(struct timer_lowerhalf_s *lower,
                                   uint32_t *maxtimeout)
{
  struct tlsr82_lowerhalf_s *priv = (struct tlsr82_lowerhalf_s *)lower;
  uint32_t clock_mhz = 0;

  if (maxtimeout == NULL)
    {
      tmrerr("Pointer to maxtimeout is NULL.\n");
      return -EINVAL;
    }

  /* Get timer clock in MHZ */

  TLSR82_TIMER_GETCLK(priv->tim, &clock_mhz);

  if (clock_mhz > 0)
    {
      /* *maxtimeout = UINT32_MAX * 1000000 / (clock_mhz * 1000000);
       * Simplify the denominator to obtain simpler code.
       */

      *maxtimeout = UINT32_MAX / clock_mhz;
    }
  else
    {
      *maxtimeout = 0;
      tmrerr("Timer clock is 0.\n");
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: tlsr82_timer_setcallback
 *
 * Description:
 *   Call this user provided timeout callback.
 *
 * Input Parameters:
 *   lower    - A pointer the publicly visible representation of the
 *              "lower-half" driver state structure.
 *   callback - The new timer expiration function pointer.  If this
 *              function pointer is NULL, then the reset-on-expiration
 *              behavior is restored,
 *  arg       - Argument that will be provided in the callback.
 *
 * Returned Value:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static void tlsr82_timer_setcallback(struct timer_lowerhalf_s *lower,
                                     tccb_t callback, void *arg)
{
  struct tlsr82_lowerhalf_s *priv = (struct tlsr82_lowerhalf_s *)lower;

  irqstate_t flags = enter_critical_section();

  /* Save the new callback */

  priv->callback = callback;
  priv->arg      = arg;

  if (callback != NULL && priv->started)
    {
      TLSR82_TIMER_SETISR(priv->tim, tlsr82_timer_handler, priv);
    }
  else
    {
      TLSR82_TIMER_SETISR(priv->tim, NULL, NULL);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_timer_initialize
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

int tlsr82_timer_initialize(const char *devpath, int timer)
{
  struct tlsr82_lowerhalf_s *lower;
  int ret = OK;

  switch (timer)
    {
#ifdef CONFIG_TLSR82_TIMER1
      case 1:
        lower = &g_timer1_lowerhalf;
        break;
#endif
#ifdef CONFIG_TLSR82_TIMER2
      case 2:
        lower = &g_timer2_lowerhalf;
        break;
#endif
      default:
        ret = -ECHRNG;
        goto errout;
    }

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->callback = NULL;
  lower->tim      = tlsr82_timer_init(timer);

  if (lower->tim == NULL)
    {
      ret = -EBUSY;
      goto errout;
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

      ret = -EEXIST;
      goto errout;
    }

errout:
  return ret;
}

#endif /* CONFIG_TIMER */
