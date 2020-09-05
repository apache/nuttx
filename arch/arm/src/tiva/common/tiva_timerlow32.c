/****************************************************************************
 * arch/arm/src/tiva/common/tiva_timerlow32.c
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
#include <nuttx/kmalloc.h>
#include <nuttx/timers/timer.h>

#include <arch/board/board.h>

#include "tiva_timer.h"

#if defined(CONFIG_TIMER) && defined(CONFIG_TIVA_TIMER)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct tiva_lowerhalf_s
{
  const struct timer_ops_s *ops;     /* Lower half operations */
  struct tiva_gptm32config_s config; /* Persistent timer configuration */
  TIMER_HANDLE handle;               /* Contained timer handle */
  tccb_t callback;                   /* Current user interrupt callback */
  void *arg;                         /* Callback argument */
  uint32_t clkin;                    /* Input clock frequency */
  uint32_t timeout;                  /* The current timeout value (us) */
  uint32_t clkticks;                 /* Actual clock ticks for current
                                      * interval */
  uint32_t adjustment;               /* Time lost due to truncation (us) */
  bool started;                      /* True: Timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions *********************************************************/

static uint32_t tiva_usec2ticks(struct tiva_lowerhalf_s *priv,
                                uint32_t usecs);
static uint32_t tiva_ticks2usec(struct tiva_lowerhalf_s *priv,
                                uint32_t ticks);
static void     tiva_timeout(struct tiva_lowerhalf_s *priv,
                             uint32_t timeout);

/* Interrupt handling *******************************************************/

static void     tiva_timer_handler(TIMER_HANDLE handle, void *arg,
                                   uint32_t status);

/* "Lower half" driver methods **********************************************/

static int      tiva_start(struct timer_lowerhalf_s *lower);
static int      tiva_stop(struct timer_lowerhalf_s *lower);
static int      tiva_getstatus(struct timer_lowerhalf_s *lower,
                               struct timer_status_s *status);
static int      tiva_settimeout(struct timer_lowerhalf_s *lower,
                                uint32_t timeout);
static void     tiva_setcallback(struct timer_lowerhalf_s *lower,
                                 tccb_t callback, FAR void *arg);
static int      tiva_ioctl(struct timer_lowerhalf_s *lower, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = tiva_start,
  .stop        = tiva_stop,
  .getstatus   = tiva_getstatus,
  .settimeout  = tiva_settimeout,
  .setcallback = tiva_setcallback,
  .ioctl       = tiva_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_usec2ticks
 *
 * Description:
 *   Convert microseconds to timer clock ticks.
 *
 * Input Parameters:
 *   priv - A pointer to a private timer driver lower half instance
 *   usecs - The number of usecs to convert
 *
 * Returned Value:
 *   The time converted to clock ticks.
 *
 ****************************************************************************/

static uint32_t tiva_usec2ticks(struct tiva_lowerhalf_s *priv,
                                uint32_t usecs)
{
  uint64_t bigticks;

  bigticks = ((uint64_t)usecs * (uint64_t)priv->clkin) / 1000000;
  if (bigticks > UINT32_MAX)
    {
      return UINT32_MAX;
    }

  return (uint32_t)bigticks;
}

/****************************************************************************
 * Name: tiva_ticks2usec
 *
 * Description:
 *   Convert timer clock ticks to microseconds.
 *
 * Input Parameters:
 *   priv - A pointer to a private timer driver lower half instance
 *   usecs - The number of ticks to convert
 *
 * Returned Value:
 *   The time converted to microseconds.
 *
 ****************************************************************************/

static uint32_t tiva_ticks2usec(struct tiva_lowerhalf_s *priv,
                                uint32_t ticks)
{
  uint64_t bigusec;

  bigusec = (1000000ull * (uint64_t)ticks) / priv->clkin;
  if (bigusec > UINT32_MAX)
    {
      return UINT32_MAX;
    }

  return (uint32_t)bigusec;
}

/****************************************************************************
 * Name: tiva_timeout
 *
 * Description:
 *   Calculate a new timeout value.
 *
 * Input Parameters:
 *   priv - A pointer to a private timer driver lower half instance
 *   timeout - The new timeout value in microseconds.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tiva_timeout(struct tiva_lowerhalf_s *priv, uint32_t timeout)
{
  tmrinfo("Entry: timeout=%d\n", timeout);

  /* Save the desired timeout value */

  priv->timeout = timeout;

  /* Calculate the actual timeout value in clock ticks */

  priv->clkticks = tiva_usec2ticks(priv, timeout);

  /* Calculate an adjustment due to truncation in timer resolution */

  timeout  = tiva_ticks2usec(priv, priv->clkticks);
  priv->adjustment = priv->timeout - timeout;

  tmrinfo("clkin=%d clkticks=%d timeout=%d, adjustment=%d\n",
          priv->clkin, priv->clkticks, priv->timeout, priv->adjustment);
}

/****************************************************************************
 * Name: tiva_timer_handler
 *
 * Description:
 *   32-bit timer interrupt handler
 *
 * Input Parameters:
 *   Usual 32-bit timer interrupt handler arguments.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tiva_timer_handler(TIMER_HANDLE handle, void *arg,
                               uint32_t status)
{
  struct tiva_lowerhalf_s *priv = (struct tiva_lowerhalf_s *)arg;

  tmrinfo("Entry: status=%08x\n", status);
  DEBUGASSERT(arg && status);

  /* Check if the timeout interrupt is pending */

  if ((status & TIMER_INT_TATO) != 0)
    {
      uint32_t timeout;

      /* Is there a registered callback?  If the callback has been nullified,
       * the timer will be stopped.
       */

      if (priv->callback && priv->callback(&priv->timeout, priv->arg))
        {
          /* Calculate new ticks / dither adjustment */

          priv->clkticks = tiva_usec2ticks(priv,
                                           priv->adjustment + priv->timeout);

          /* Set next interval interval. TODO: make sure the interval is not
           * so soon it will be missed!
           */

#if 0 /* Too much in this context */
          tiva_timer32_setinterval(priv->handle, priv->clkticks);
#else
          tiva_gptm_putreg(priv->handle, TIVA_TIMER_TAILR_OFFSET,
                           priv->clkticks);
#endif

          /* Calculate the next adjustment */

          timeout = tiva_ticks2usec(priv, priv->clkticks);
          priv->adjustment = (priv->adjustment + priv->timeout) - timeout;
        }
      else
        {
          /* No callback or the callback returned false.. stop the timer */

          tiva_timer32_stop(priv->handle);
          tmrinfo("Stopped\n");
        }
    }
}

/****************************************************************************
 * Name: tiva_start
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

static int tiva_start(struct timer_lowerhalf_s *lower)
{
  struct tiva_lowerhalf_s *priv = (struct tiva_lowerhalf_s *)lower;

  tmrinfo("Entry: started %d\n", priv->started);

  /* Has the timer already been started? */

  if (!priv->started)
    {
      /* Start the timer */

      tiva_timer32_start(priv->handle);
      priv->started = true;
      return OK;
    }

  /* Return EBUSY to indicate that the timer was already running */

  return -EBUSY;
}

/****************************************************************************
 * Name: tiva_stop
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

static int tiva_stop(struct timer_lowerhalf_s *lower)
{
  struct tiva_lowerhalf_s *priv = (struct tiva_lowerhalf_s *)lower;

  tmrinfo("Entry: started %d\n", priv->started);

  /* Has the timer already been started? */

  if (priv->started)
    {
      /* Stop the timer */

      tiva_timer32_stop(priv->handle);
      priv->started = false;
      return OK;
    }

  /* Return ENODEV to indicate that the timer was not running */

  return -ENODEV;
}

/****************************************************************************
 * Name: tiva_getstatus
 *
 * Description:
 *   Get the current timer status
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *   status - The location to return the status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int tiva_getstatus(struct timer_lowerhalf_s *lower,
                          struct timer_status_s *status)
{
  struct tiva_lowerhalf_s *priv = (struct tiva_lowerhalf_s *)lower;
  uint32_t remaining;

  tmrinfo("Entry\n");
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

  /* Get the time remaining until the timer expires (in microseconds). */

  remaining = tiva_timer32_remaining(priv->handle);
  status->timeleft = tiva_ticks2usec(priv, remaining);

  tmrinfo("  flags    : %08x\n", status->flags);
  tmrinfo("  timeout  : %d\n",   status->timeout);
  tmrinfo("  timeleft : %d\n",   status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: tiva_settimeout
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

static int tiva_settimeout(struct timer_lowerhalf_s *lower, uint32_t timeout)
{
  struct tiva_lowerhalf_s *priv = (struct tiva_lowerhalf_s *)lower;

  DEBUGASSERT(priv);

  if (priv->started)
    {
      return -EPERM;
    }

  tmrinfo("Entry: timeout=%d\n", timeout);

  /* Calculate the new time settings */

  tiva_timeout(priv, timeout);

  /* Reset the timer interval */

  tiva_timer32_setinterval(priv->handle, priv->clkticks);
  return OK;
}

/****************************************************************************
 * Name: tiva_setcallback
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
 *
 * Returned Value:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static void tiva_setcallback(struct timer_lowerhalf_s *lower,
                             tccb_t callback, FAR void *arg)
{
  struct tiva_lowerhalf_s *priv = (struct tiva_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();

  DEBUGASSERT(priv);
  tmrinfo("Entry: callback=%p\n", callback);

  /* Save the new callback */

  priv->callback = callback;
  priv->arg      = arg;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: tiva_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *   cmd   - The ioctl command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int tiva_ioctl(struct timer_lowerhalf_s *lower, int cmd,
                    unsigned long arg)
{
  int ret = -ENOTTY;

  DEBUGASSERT(lower);
  tmrinfo("Entry: cmd=%d arg=%ld\n", cmd, arg);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 *   NOTES:
 *   1. Only 32-bit periodic timers are supported.
 *   2. Timeout interrupts are disabled until tiva_timer32_setinterval() is
 *      called.
 *   3. Match interrupts are disabled until tiva_timer32_relmatch() is
 *      called.
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *     form /dev/timer0
 *   config - 32-bit timer configuration values.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int tiva_timer_initialize(FAR const char *devpath,
                          struct tiva_gptm32config_s *config)
{
  struct tiva_lowerhalf_s *priv;
  void *drvr;
  int ret;

  tmrinfo("\n");
  DEBUGASSERT(devpath);

  /* Allocate an instance of the lower half state structure */

  priv = (struct tiva_lowerhalf_s *)
         kmm_zalloc(sizeof(struct tiva_lowerhalf_s));
  if (!priv)
    {
      tmrerr("ERROR: Failed to allocate driver structure\n");
      return -ENOMEM;
    }

  /* Initialize the non-zero elements of lower half state structure */

  priv->ops = &g_timer_ops;
#ifdef CONFIG_ARCH_CHIP_TM4C129
  priv->clkin = config->cmn.alternate ? ALTCLK_FREQUENCY : SYSCLK_FREQUENCY;
#else
  if (config->cmn.alternate)
    {
      tmrerr("ERROR: Alternate clock unsupported on TM4C123 architecture\n");
      return -ENOMEM;
    }
  else
    {
      priv->clkin = SYSCLK_FREQUENCY;
    }
#endif /* CONFIG_ARCH_CHIP_TM4C129 */

  config->config.handler = tiva_timer_handler;
  config->config.arg = priv;
  memcpy(&(priv->config), config, sizeof(struct tiva_gptm32config_s));

  /* Set the initial timer interval */

  tiva_timeout(priv, 0);

  /* Create the timer handle */

  priv->handle = tiva_gptm_configure((const struct tiva_gptmconfig_s *)
                                     &priv->config);
  if (!priv->handle)
    {
      tmrerr("ERROR: Failed to create timer handle\n");
      ret = -EINVAL;
      goto errout_with_alloc;
    }

  /* Register the timer driver as /dev/timerX.  The returned value from
   * timer_register is a handle that could be used with timer_unregister().
   * REVISIT: The returned handle is discard here.
   */

  drvr = timer_register(devpath, (struct timer_lowerhalf_s *)priv);
  if (!drvr)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the timer driver (such as if the
       * 'depath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      ret = -EEXIST;
      goto errout_with_timer;
    }

  return OK;

errout_with_timer:
  tiva_gptm_release(priv->handle);  /* Free timer resources */

errout_with_alloc:
  kmm_free(priv);                   /* Free the allocated state structure */
  return ret;                       /* Return the error indication */
}

#endif /* CONFIG_TIMER && CONFIG_TIVA_TIMER */
