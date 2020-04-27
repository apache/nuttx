/****************************************************************************
 * arch/mips/src/pic32mz/chip/pic32mz_timer-lowerhalf.c
 *
 *   Copyright (C) 2019 Abdelatif Guettouche. All rights reserved.
 *   Author: Abdelatif Guettouche <abdelatif.guettouche@gmail.com>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include "pic32mz_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* One of the type B timers should be defined. */

#if defined(CONFIG_TIMER) && defined(CONFIG_PIC32MZ_TIMER)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct pic32mz_lowerhalf_s
{
  FAR const struct timer_ops_s   *ops;     /* Lower half operations        */
  FAR struct pic32mz_timer_dev_s *timer;   /* pic32mz timer driver         */
  tccb_t                         callback; /* Current user interrupt cb    */
  FAR void                       *arg;     /* Argument to upper half cb    */
  uint32_t                       timeout;  /* Current timeout value (us)   */
  uint32_t                       ticks;    /* Timeout converted in ticks   */
  uint32_t                       freq;     /* Timer's frequency (Hz)       */
  uint8_t                        width;    /* Timer's width                */
  uint32_t                       maxticks; /* Maximum ticks for this timer */
  bool                           started;  /* True: Timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions *********************************************************/

static uint32_t pic32mz_usec2ticks(struct pic32mz_lowerhalf_s *priv,
                                   uint32_t usecs);
static uint32_t pic32mz_ticks2usec(struct pic32mz_lowerhalf_s *priv,
                                   uint32_t ticks);

/* Interrupt handling *******************************************************/

static int pic32mz_timer_handler(int irq, void * context, void * arg);

/* "Lower half" driver methods **********************************************/

static int  pic32mz_start(FAR struct timer_lowerhalf_s *lower);
static int  pic32mz_stop(FAR struct timer_lowerhalf_s *lower);
static int  pic32mz_getstatus(struct timer_lowerhalf_s *lower,
                              struct timer_status_s *status);
static int  pic32mz_settimeout(FAR struct timer_lowerhalf_s *lower,
                               uint32_t timeout);
static void pic32mz_setcallback(FAR struct timer_lowerhalf_s *lower,
                              tccb_t callback, FAR void *arg);
static int  pic32mz_ioctl(struct timer_lowerhalf_s *lower, int cmd,
                          unsigned long arg);
static int  pic32mz_maxtimeout(FAR struct timer_lowerhalf_s *lower,
                               FAR uint32_t *maxtimeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = pic32mz_start,
  .stop        = pic32mz_stop,
  .getstatus   = pic32mz_getstatus,
  .settimeout  = pic32mz_settimeout,
  .setcallback = pic32mz_setcallback,
  .ioctl       = pic32mz_ioctl,
  .maxtimeout  = pic32mz_maxtimeout,
};

#ifdef CONFIG_PIC32MZ_T2
static struct pic32mz_lowerhalf_s g_t2_lowerhalf =
{
  .ops         = &g_timer_ops,
};
#endif

#ifdef CONFIG_PIC32MZ_T3
static struct pic32mz_lowerhalf_s g_t3_lowerhalf =
{
  .ops         = &g_timer_ops,
};
#endif

#ifdef CONFIG_PIC32MZ_T4
static struct pic32mz_lowerhalf_s g_t4_lowerhalf =
{
  .ops         = &g_timer_ops,
};
#endif

#ifdef CONFIG_PIC32MZ_T5
static struct pic32mz_lowerhalf_s g_t5_lowerhalf =
{
  .ops         = &g_timer_ops,
};
#endif

#ifdef CONFIG_PIC32MZ_T6
static struct pic32mz_lowerhalf_s g_t6_lowerhalf =
{
  .ops         = &g_timer_ops,
};
#endif

#ifdef CONFIG_PIC32MZ_T7
static struct pic32mz_lowerhalf_s g_t7_lowerhalf =
{
  .ops         = &g_timer_ops,
};
#endif

#ifdef CONFIG_PIC32MZ_T8
static struct pic32mz_lowerhalf_s g_t8_lowerhalf =
{
  .ops         = &g_timer_ops,
};
#endif

#ifdef CONFIG_PIC32MZ_T9
static struct pic32mz_lowerhalf_s g_t9_lowerhalf =
{
  .ops         = &g_timer_ops,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_usec2ticks
 *
 * Description:
 *   Convert microseconds to timer clock ticks.
 *
 * Input Parameters:
 *   priv   A pointer to a private timer driver lower half instance
 *   usecs  The number of usecs to convert
 *
 * Returned Value:
 *   The time converted to clock ticks.
 *
 ****************************************************************************/

static uint32_t pic32mz_usec2ticks(FAR struct pic32mz_lowerhalf_s *priv,
                                   uint32_t usecs)
{
  uint64_t bigticks;

  bigticks = ((uint64_t)usecs * (uint64_t)priv->freq) / 1000000;

  if (bigticks > UINT32_MAX)
    {
      return UINT32_MAX;
    }

  return (uint32_t)bigticks;
}

/****************************************************************************
 * Name: pic32mz_ticks2usec
 *
 * Description:
 *   Convert timer clock ticks to microseconds.
 *
 * Input Parameters:
 *   priv   A pointer to a private timer driver lower half instance
 *   usecs  The number of ticks to convert
 *
 * Returned Value:
 *   The time converted to microseconds.
 *
 ****************************************************************************/

static uint32_t pic32mz_ticks2usec(FAR struct pic32mz_lowerhalf_s *priv,
                                   uint32_t ticks)
{
  uint64_t bigusec;

  bigusec = (1000000ull * (uint64_t)ticks) / priv->freq;
  if (bigusec > UINT32_MAX)
    {
      return UINT32_MAX;
    }

  return (uint32_t)bigusec;
}

/****************************************************************************
 * Name: pic32mz_timer_handler
 *
 * Description:
 *   timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int pic32mz_timer_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct pic32mz_lowerhalf_s *lower =
    (struct pic32mz_lowerhalf_s *) arg;
  uint32_t next_interval_us = 0;

  PIC32MZ_TIMER_ACKINT(lower->timer);

  if (lower->callback && lower->callback(&next_interval_us, lower->arg))
    {
      if (next_interval_us > 0)
        {
          /* Stop the timer before writing the new period,
           * to prevent unintended period matches.
           */

          PIC32MZ_TIMER_STOP(lower->timer);

          PIC32MZ_TIMER_SETPERIOD(lower->timer, next_interval_us);

          PIC32MZ_TIMER_START(lower->timer);
        }
    }
  else
    {
      pic32mz_stop((struct timer_lowerhalf_s *)lower);
    }

  return OK;
}

/****************************************************************************
 * Name: pic32mz_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower   A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int pic32mz_start(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct pic32mz_lowerhalf_s *priv =
    (FAR struct pic32mz_lowerhalf_s *)lower;

  if (!priv->started)
    {
      if (priv->callback != NULL)
        {
          PIC32MZ_TIMER_SETISR(priv->timer, pic32mz_timer_handler, priv);
        }

      PIC32MZ_TIMER_START(priv->timer);

      priv->started = true;

      return OK;
    }

  /* Return EBUSY to indicate that the timer was already running */

  return -EBUSY;
}

/****************************************************************************
 * Name: pic32mz_stop
 *
 * Description:
 *   Stop the timer
 *
 * Input Parameters:
 *   lower A pointer the publicly visible representation of the "lower-half"
 *         driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int pic32mz_stop(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct pic32mz_lowerhalf_s *priv =
    (FAR struct pic32mz_lowerhalf_s *)lower;

  if (priv->started)
    {
      PIC32MZ_TIMER_STOP(priv->timer);
      PIC32MZ_TIMER_SETISR(priv->timer, NULL, NULL);
      priv->started = false;

      return OK;
    }

  /* Return ENODEV to indicate that the timer was not running */

  return -ENODEV;
}

/****************************************************************************
 * Name: pic32mz_getstatus
 *
 * Description:
 *   Get the current timer status
 *
 * Input Parameters:
 *   lower    A pointer the publicly visible representation of the
 *            "lower-half" driver state structure.
 *   status   The location to return the status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int  pic32mz_getstatus(FAR struct timer_lowerhalf_s *lower,
                              FAR struct timer_status_s *status)
{
  FAR struct pic32mz_lowerhalf_s *priv =
    (FAR struct pic32mz_lowerhalf_s *)lower;
  uint32_t remainingticks;

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

  /* Return the actual timeout in microseconds */

  status->timeout = priv->timeout;

  /* Get the remaining ticks for the next overflow. */

  remainingticks = priv->ticks - PIC32MZ_TIMER_GETCOUNTER(priv->timer);

  /* Convert the remaining ticks to microseconds. */

  status->timeleft = pic32mz_ticks2usec(priv, remainingticks);

  return OK;
}

/****************************************************************************
 * Name: pic32mz_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower    A pointer the publicly visible representation of the
 *            "lower-half" driver state structure.
 *   timeout  The new timeout value in microseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int pic32mz_settimeout(FAR struct timer_lowerhalf_s *lower,
                              uint32_t timeout)
{
  FAR struct pic32mz_lowerhalf_s *priv =
    (FAR struct pic32mz_lowerhalf_s *)lower;

  DEBUGASSERT(priv);

  if (priv->started)
    {
      return -EPERM;
    }

  tmrinfo("Entry: timeout = %d\n", timeout);

  priv->timeout = timeout;

  /* Convert the timeout to ticks. */

  priv->ticks = pic32mz_usec2ticks(priv, timeout);

  tmrinfo("Timeout ticks = %ld\n", priv->ticks);

  if (priv->ticks == 0)
    {
      tmrerr("The timeout value can't be represented by "
             "the current config! Try a different prescaler.\n");

      /* REVISIT: This might be done by trying all the prescale values
       * (there aren't that much) and find the first that can represent
       * these ticks.
       */

      return -EAGAIN;
    }
  else if (priv->ticks > priv->maxticks)
    {
      tmrwarn("Timeout value truncated! Consider a different prescaler.\n");

      /* REVISIT: Try a different prescale value first. */

      PIC32MZ_TIMER_SETPERIOD(priv->timer, priv->maxticks);
    }
  else
    {
      PIC32MZ_TIMER_SETPERIOD(priv->timer, priv->ticks);
    }

  return OK;
}

/****************************************************************************
 * Name: pic32mz_setcallback
 *
 * Description:
 *   Call this user provided timeout callback.
 *
 * Input Parameters:
 *   lower     A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   callback  The new timer expiration function pointer.  If this
 *             function pointer is NULL, then the reset-on-expiration
 *             behavior is restored.
 *  arg        Argument that will be provided in the callback.
 *
 * Returned Value:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static void pic32mz_setcallback(FAR struct timer_lowerhalf_s *lower,
                                tccb_t callback, FAR void *arg)
{
  FAR struct pic32mz_lowerhalf_s *priv =
    (FAR struct pic32mz_lowerhalf_s *)lower;

  irqstate_t flags = enter_critical_section();

  DEBUGASSERT(priv);

  /* Save the new callback */

  priv->callback = callback;
  priv->arg      = arg;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: pic32mz_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower  A pointer the publicly visible representation of the "lower-half"
 *          driver state structure.
 *   cmd    The ioctl command value
 *   arg    The optional argument that accompanies the 'cmd'.  The
 *          interpretation of this argument depends on the particular
 *          command.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int  pic32mz_ioctl(FAR struct timer_lowerhalf_s *lower, int cmd,
                          unsigned long arg)
{
  int ret = -ENOTTY;

  DEBUGASSERT(lower);
  tmrinfo("Entry: cmd=%d arg=%ld\n", cmd, arg);

  return ret;
}

/****************************************************************************
 * Name: pic32mz_ioctl
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

static int  pic32mz_maxtimeout(FAR struct timer_lowerhalf_s *lower,
                               FAR uint32_t *maxtimeout)
{
  FAR struct pic32mz_lowerhalf_s *priv =
    (FAR struct pic32mz_lowerhalf_s *)lower;

  DEBUGASSERT(priv);

  /* Convert ticks to microseconds */

  *maxtimeout = pic32mz_ticks2usec(priv, priv->maxticks);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath   The full path to the timer device. This should be of the
 *             form /dev/timer0
 *   timer   the timer's number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int pic32mz_timer_initialize(FAR const char *devpath, int timer)
{
  FAR struct pic32mz_lowerhalf_s *lower;
  FAR void *drvr;

  tmrinfo("Initializing %s - timer %d\n", devpath, timer);

  switch (timer)
    {
#ifdef CONFIG_PIC32MZ_T2
      case 2:
        lower = &g_t2_lowerhalf;
        break;
#endif
#ifdef CONFIG_PIC32MZ_T3
      case 3:
        lower = &g_t3_lowerhalf;
        break;
#endif
#ifdef CONFIG_PIC32MZ_T4
      case 4:
        lower = &g_t4_lowerhalf;
        break;
#endif
#ifdef CONFIG_PIC32MZ_T5
      case 5:
        lower = &g_t5_lowerhalf;
        break;
#endif
#ifdef CONFIG_PIC32MZ_T6
      case 6:
        lower = &g_t6_lowerhalf;
        break;
#endif
#ifdef CONFIG_PIC32MZ_T7
      case 7:
        lower = &g_t7_lowerhalf;
        break;
#endif
#ifdef CONFIG_PIC32MZ_T8
      case 8:
        lower = &g_t8_lowerhalf;
        break;
#endif
#ifdef CONFIG_PIC32MZ_T9
      case 9:
        lower = &g_t9_lowerhalf;
        break;
#endif
      default:
        return -ENODEV;
    }

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->callback = NULL;
  lower->timer    = pic32mz_timer_init(timer);

  if (lower->timer == NULL)
    {
      tmrerr("ERROR: Failed to init timer %d.\n", timer);

      return -EINVAL;
    }

  lower->freq     = PIC32MZ_TIMER_GETFREQ(lower->timer);
  lower->width    = PIC32MZ_TIMER_GETWIDTH(lower->timer);
  lower->maxticks = ((1ull << lower->width) - 1ul);

  tmrinfo("Freq=%dHz / Width=%d-bit / Max ticks=%lu\n",
           lower->freq, lower->width, lower->maxticks);

  /* Register the timer driver as /dev/timerX.  The returned value from
   * timer_register is a handle that could be used with timer_unregister().
   * REVISIT: The returned handle is discard here.
   */

  drvr = timer_register(devpath, (struct timer_lowerhalf_s *)lower);

  if (drvr == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the timer driver (such as if the
       * 'devpath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      tmrerr("ERROR: Failed to register timer %s\n", devpath);

      return -EEXIST;
    }

  tmrinfo("Timer registered successfully\n");

  return OK;
}

#endif /* CONFIG_TIMER && CONFIG_PIC32MZ_TIMER */
