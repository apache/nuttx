/****************************************************************************
 * arch/arm/src/samv7/sam_tc_lowerhalf.c
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
#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/timers/timer.h>

#include <arch/board/board.h>

#include "sam_tc.h"

#if defined(CONFIG_TIMER) && \
    (defined(CONFIG_SAMV7_TC0) || defined(CONFIG_SAMV7_TC1) || \
     defined(CONFIG_SAMV7_TC2) || defined(CONFIG_SAMV7_TC3))

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

struct sam_lowerhalf_s
{
  const struct timer_ops_s *ops;      /* Lower half operations */
  TC_HANDLE                 tch;      /* Handle returned by sam_tc_initialize() */
  tccb_t                    callback; /* Current user interrupt callback */
  void                     *arg;      /* Argument passed to upper half callback */
  bool                      started;  /* True: Timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void sam_timer_handler(TC_HANDLE tch, void *arg, uint32_t sr);

/* "Lower half" driver methods **********************************************/

static int sam_start(struct timer_lowerhalf_s *lower);
static int sam_stop(struct timer_lowerhalf_s *lower);
static int sam_getstatus(struct timer_lowerhalf_s *lower,
                         struct timer_status_s *status);
static int sam_settimeout(struct timer_lowerhalf_s *lower,
                          uint32_t timeout);
static void sam_setcallback(struct timer_lowerhalf_s *lower,
                            tccb_t callback, void *arg);
static int sam_maxtimeout(struct timer_lowerhalf_s *lower,
                          uint32_t *maxtimeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = sam_start,
  .stop        = sam_stop,
  .getstatus   = sam_getstatus,
  .settimeout  = sam_settimeout,
  .setcallback = sam_setcallback,
  .ioctl       = NULL,
  .maxtimeout  = sam_maxtimeout
};

#ifdef CONFIG_SAMV7_TC0
static struct sam_lowerhalf_s g_tc0_lowerhalf =
{
  .ops = &g_timer_ops
};

static struct sam_lowerhalf_s g_tc1_lowerhalf =
{
  .ops = &g_timer_ops
};

static struct sam_lowerhalf_s g_tc2_lowerhalf =
{
  .ops = &g_timer_ops
};
#endif

#ifdef CONFIG_SAMV7_TC1
static struct sam_lowerhalf_s g_tc3_lowerhalf =
{
  .ops = &g_timer_ops
};

static struct sam_lowerhalf_s g_tc4_lowerhalf =
{
  .ops = &g_timer_ops
};

static struct sam_lowerhalf_s g_tc5_lowerhalf =
{
  .ops = &g_timer_ops
};
#endif

#ifdef CONFIG_SAMV7_TC2
static struct sam_lowerhalf_s g_tc6_lowerhalf =
{
  .ops = &g_timer_ops
};

static struct sam_lowerhalf_s g_tc7_lowerhalf =
{
  .ops = &g_timer_ops
};

static struct sam_lowerhalf_s g_tc8_lowerhalf =
{
  .ops = &g_timer_ops
};
#endif

#ifdef CONFIG_SAMV7_TC3
static struct sam_lowerhalf_s g_tc9_lowerhalf =
{
  .ops = &g_timer_ops
};

static struct sam_lowerhalf_s g_tc10_lowerhalf =
{
  .ops = &g_timer_ops
};

static struct sam_lowerhalf_s g_tc11_lowerhalf =
{
  .ops = &g_timer_ops
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_timer_handler
 *
 * Description:
 *   timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void sam_timer_handler(TC_HANDLE tch, void *arg, uint32_t sr)
{
  struct sam_lowerhalf_s *lower = (struct sam_lowerhalf_s *)arg;
  uint32_t next_interval_us = 0;

  if (lower->callback(&next_interval_us, lower->arg))
    {
      if (next_interval_us > 0)
        {
          sam_settimeout((struct timer_lowerhalf_s *)lower,
                         next_interval_us);
        }

      /* Start the counter */

      sam_tc_start(tch);
    }
  else
    {
      sam_stop((struct timer_lowerhalf_s *)lower);
    }
}

/****************************************************************************
 * Name: sam_start
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

static int sam_start(struct timer_lowerhalf_s *lower)
{
  struct sam_lowerhalf_s *priv = (struct sam_lowerhalf_s *)lower;
  int ret = -EBUSY; /* EBUSY indicates that the timer was already running */
  irqstate_t flags;

  flags = enter_critical_section();

  if (!priv->started)
    {
      if (priv->callback != NULL)
        {
          /* Set up to receive the callback when the interrupt occurs */

          sam_tc_attach(priv->tch, sam_timer_handler, priv, TC_INT_CPCS);

          /* Start the counter */

          sam_tc_start(priv->tch);
        }

      priv->started = true;
      ret = OK;
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: sam_stop
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

static int sam_stop(struct timer_lowerhalf_s *lower)
{
  struct sam_lowerhalf_s *priv = (struct sam_lowerhalf_s *)lower;
  int ret = -ENODEV; /* ENODEV indicates that the timer was not running */
  irqstate_t flags;

  flags = enter_critical_section();

  if (priv->started)
    {
      sam_tc_stop(priv->tch);
      sam_tc_detach(priv->tch);
      priv->started = false;
      ret = OK;
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: sam_getstatus
 *
 * Description:
 *   get timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the
 *            "lower- half" driver state structure.
 *   status - The location to return the status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_getstatus(struct timer_lowerhalf_s *lower,
                         struct timer_status_s *status)
{
  struct sam_lowerhalf_s *priv = (struct sam_lowerhalf_s *)lower;
  uint32_t frequency;
  uint16_t period;
  uint16_t current;

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

  frequency  = sam_tc_divfreq(priv->tch);
  period     = sam_tc_getregister(priv->tch, TC_REGC);
  current    = sam_tc_getcounter(priv->tch);

  /* Get timeout */

  status->timeout = (1000000llu * period) / frequency;

  /* Get the time remaining until the timer expires (in microseconds) */

  status->timeleft = (1000000llu * (period - current)) / frequency;

  return OK;
}

/****************************************************************************
 * Name: sam_settimeout
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

static int sam_settimeout(struct timer_lowerhalf_s *lower,
                          uint32_t timeout)
{
  struct sam_lowerhalf_s *priv = (struct sam_lowerhalf_s *)lower;
  uint64_t maxtimeout;
  uint64_t regval;
  uint32_t desired;
  uint32_t actual;
  uint32_t tcclks = 0;

  if (priv->started)
    {
      return -EPERM;
    }

  desired = USEC_PER_SEC;
  actual = desired;
  maxtimeout = (1000000llu * 0xffff) / actual;

  while ((timeout > maxtimeout) && (desired > 0))
    {
      desired /= 10;

      if (sam_tc_clockselect(desired, &tcclks, &actual) < 0)
        {
          break;
        }

      maxtimeout = (1000000llu * 0xffff) / actual;
    }

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > maxtimeout)
    {
      tmrerr("ERROR: Cannot represent timeout=%" PRIu32 " > %" PRIu64 "\n",
             timeout, maxtimeout);
      return -ERANGE;
    }

  if (actual != sam_tc_divfreq(priv->tch))
    {
      sam_tc_settcclks(priv->tch, tcclks);
    }

  /* Get the timer counter frequency and determine the number of counts
   * needed to achieve the requested delay.
   *
   *   frequency = ticks / second
   *   ticks     = seconds * frequency
   *             = (usecs * frequency) / USEC_PER_SEC;
   */

  regval = (timeout * (uint64_t)sam_tc_divfreq(priv->tch)) / USEC_PER_SEC;

  tmrinfo("timeout=%" PRIu32 " regval=%08" PRIx64 "\n", timeout, regval);
  DEBUGASSERT(regval <= UINT16_MAX);

  /* Set RC so that an event will be triggered when TC_CV register counts
   * up to RC.
   */

  sam_tc_setregister(priv->tch, TC_REGC, regval);

  return OK;
}

/****************************************************************************
 * Name: sam_setcallback
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

static void sam_setcallback(struct timer_lowerhalf_s *lower,
                            tccb_t callback, void *arg)
{
  struct sam_lowerhalf_s *priv = (struct sam_lowerhalf_s *)lower;

  irqstate_t flags = enter_critical_section();

  /* Save the new callback */

  priv->callback = callback;
  priv->arg      = arg;

  if (callback != NULL && priv->started)
    {
      sam_tc_attach(priv->tch, sam_timer_handler, priv, TC_INT_CPCS);
    }
  else
    {
      sam_tc_detach(priv->tch);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_maxtimeout
 *
 * Description:
 *   Get the maximum supported timeout value
 *
 * Input Parameters:
 *   lower        A pointer the publicly visible representation of the
 *                "lower-half" driver state structure.
 *   maxtimeout   The max value in microseconds will be written here.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_maxtimeout(struct timer_lowerhalf_s *lower,
                          uint32_t *maxtimeout)
{
  uint64_t bigusec;
  uint32_t frequency = USEC_PER_SEC;
  int ret;

  ret = sam_tc_clockselect(BOARD_SLOWCLK_FREQUENCY, NULL, &frequency);
  if (ret < 0)
    {
      return ret;
    }

  bigusec = (1000000ull * 0xffff) / frequency;
  if (bigusec > UINT32_MAX)
    {
      *maxtimeout = UINT32_MAX;
    }
  else
    {
      *maxtimeout = bigusec;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath    The full path to the timer device.  This should be of the
 *              form /dev/timer0.
 *   chan       Timer counter channel to be used.  See the TC_CHAN*
 *              definitions in arch/arm/src/samv7/sam_tc.h.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int sam_timer_initialize(const char *devpath, int chan)
{
  struct sam_lowerhalf_s *lower;
  uint32_t actual;
  uint32_t cmr;
  int ret;

  switch (chan)
    {
#ifdef CONFIG_SAMV7_TC0
      case TC_CHAN0:
        lower = &g_tc0_lowerhalf;
        break;

      case TC_CHAN1:
        lower = &g_tc1_lowerhalf;
        break;

      case TC_CHAN2:
        lower = &g_tc2_lowerhalf;
        break;
#endif
#ifdef CONFIG_SAMV7_TC1
      case TC_CHAN3:
        lower = &g_tc3_lowerhalf;
        break;

      case TC_CHAN4:
        lower = &g_tc4_lowerhalf;
        break;

      case TC_CHAN5:
        lower = &g_tc5_lowerhalf;
        break;
#endif
#ifdef CONFIG_SAMV7_TC2
      case TC_CHAN6:
        lower = &g_tc6_lowerhalf;
        break;

      case TC_CHAN7:
        lower = &g_tc7_lowerhalf;
        break;

      case TC_CHAN8:
        lower = &g_tc8_lowerhalf;
        break;
#endif
#ifdef CONFIG_SAMV7_TC3
      case TC_CHAN9:
        lower = &g_tc9_lowerhalf;
        break;

      case TC_CHAN10:
        lower = &g_tc10_lowerhalf;
        break;

      case TC_CHAN11:
        lower = &g_tc11_lowerhalf;
        break;
#endif
      default:
        return -ENODEV;
    }

  /* The pre-calculate values to use when we start the timer */

  ret = sam_tc_clockselect(USEC_PER_SEC, &cmr, &actual);
  if (ret < 0)
    {
      tmrerr("ERROR: sam_tc_clockselect failed: %d\n", ret);
      return ret;
    }

  tmrinfo("actual=%" PRIu32 ", cmr=%08" PRIx32 "\n", actual, cmr);

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->callback = NULL;

  /* Allocate the timer/counter and select its mode of operation
   *
   *   TC_CMR_TCCLKS       - Returned by sam_tc_clockselect
   *   TC_CMR_CLKI=0       - Not inverted
   *   TC_CMR_BURST_NONE   - Not gated by an external signal
   *   TC_CMR_CPCSTOP=1    - Stop the clock on an RC compare event
   *   TC_CMR_CPCDIS=0     - Don't disable the clock on an RC compare event
   *   TC_CMR_EEVTEDG_NONE - No external events (and, hence, no edges
   *   TC_CMR_EEVT_TIOB    - ???? REVISIT
   *   TC_CMR_ENET=0       - External event trigger disabled
   *   TC_CMR_WAVSEL_UPRC  - TC_CV is incremented from 0 to the value of RC,
   *                         then automatically reset on a RC Compare
   *   TC_CMR_WAVE         - Waveform mode
   *   TC_CMR_ACPA_NONE    - RA compare has no effect on TIOA
   *   TC_CMR_ACPC_NONE    - RC compare has no effect on TIOA
   *   TC_CMR_AEEVT_NONE   - No external event effect on TIOA
   *   TC_CMR_ASWTRG_NONE  - No software trigger effect on TIOA
   *   TC_CMR_BCPB_NONE    - RB compare has no effect on TIOB
   *   TC_CMR_BCPC_NONE    - RC compare has no effect on TIOB
   *   TC_CMR_BEEVT_NONE   - No external event effect on TIOB
   *   TC_CMR_BSWTRG_NONE  - No software trigger effect on TIOB
   */

  cmr |= (TC_CMR_BURST_NONE  | TC_CMR_CPCSTOP     | TC_CMR_EEVTEDG_NONE |
          TC_CMR_EEVT_TIOB   | TC_CMR_WAVSEL_UPRC | TC_CMR_WAVE         |
          TC_CMR_ACPA_NONE   | TC_CMR_ACPC_NONE   | TC_CMR_AEEVT_NONE   |
          TC_CMR_ASWTRG_NONE | TC_CMR_BCPB_NONE   | TC_CMR_BCPC_NONE    |
          TC_CMR_BEEVT_NONE  | TC_CMR_BSWTRG_NONE);

  lower->tch = sam_tc_allocate(chan, cmr);

  if (lower->tch == NULL)
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

      sam_tc_free(lower->tch);
      return -EEXIST;
    }

  return OK;
}

#endif /* CONFIG_TIMER */
