/****************************************************************************
 * arch/arm/src/phy62xx/phyplus_timer_lowerhalf.c
 *
 *   Copyright (C) 2015 Wail Khemir. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Authors: Wail Khemir <khemirwail@gmail.com>
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

#include "phyplus_tim.h"
#include "phyplus_timer_lowerhalf.h"

#if defined(CONFIG_TIMER)

#define TMR_MAXTIMEOUT                      0xffffff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct phyplus_lowerhalf_s
{
  FAR const struct timer_ops_s   *ops;     /* Lower half operations        */
  FAR struct phyplus_timer_dev_s *tim;     /* pic32mz timer driver         */
  tccb_t                         callback; /* Current user interrupt cb    */
  FAR void                       *arg;     /* Argument to upper half cb    */
  bool                           started;  /* True: Timer has been started */
  uint32_t                       timeout;  /* Current timeout value (us)   */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int phyplus_timer_handler(int irq, void * context, void * arg);

/* "Lower half" driver methods **********************************************/

static int phyplus_start(FAR struct timer_lowerhalf_s *lower);
static int phyplus_stop(FAR struct timer_lowerhalf_s *lower);
static int phyplus_getstatus(FAR struct timer_lowerhalf_s *lower,
                             FAR struct timer_status_s *status);
static int phyplus_settimeout(FAR struct timer_lowerhalf_s *lower,
                            uint32_t timeout);
static void phyplus_setcallback(FAR struct timer_lowerhalf_s *lower,
                            tccb_t callback, FAR void *arg);

/* static int phyplus_ioctl(struct timer_lowerhalf_s *lower, int cmd,
 *                           unsigned long arg);
 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = phyplus_start,
  .stop        = phyplus_stop,
#if 1
  .getstatus   = phyplus_getstatus,
#else
  .getstatus   = NULL,
#endif  
  .settimeout  = phyplus_settimeout,
  .setcallback = phyplus_setcallback,
#if 1
  .ioctl       = NULL,
#else
  .ioctl       = phyplus_ioctl,
#endif
};

/* #ifdef CONFIG_PHYPLUS_TIM1 */

static struct phyplus_lowerhalf_s g_tim1_lowerhalf =
{
  .ops         = &g_timer_ops,
};

/* #endif */

/* #ifdef CONFIG_PHYPLUS_TIM2 */

static struct phyplus_lowerhalf_s g_tim2_lowerhalf =
{
  .ops         = &g_timer_ops,
};

/* #endif */

/* #ifdef CONFIG_PHYPLUS_TIM3 */

static struct phyplus_lowerhalf_s g_tim3_lowerhalf =
{
  .ops         = &g_timer_ops,
};

/* #endif */

/* #ifdef CONFIG_PHYPLUS_TIM4 */

static struct phyplus_lowerhalf_s g_tim4_lowerhalf =
{
  .ops         = &g_timer_ops,
};

/* #endif */

/* #ifdef CONFIG_PHYPLUS_TIM5 */

static struct phyplus_lowerhalf_s g_tim5_lowerhalf =
{
  .ops         = &g_timer_ops,
};

/* #endif */

/* #ifdef CONFIG_PHYPLUS_TIM6 */

static struct phyplus_lowerhalf_s g_tim6_lowerhalf =
{
  .ops         = &g_timer_ops,
};

/* #endif */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phyplus_timer_handler
 *
 * Description:
 *   timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int phyplus_timer_handler(int irq, void * context, void * arg)
{
  FAR struct phyplus_lowerhalf_s *lower = (struct phyplus_lowerhalf_s *) arg;

  /* PHYPLUS_TIM_ACKINT(lower->tim); */

  phyplus_tim_ackint(lower->tim);
  if (lower->callback && lower->callback(&lower->timeout, lower->arg))
    {
      /* PHYPLUS_TIM_SETCOUNTER(lower->tim, lower->timeout); */

      phyplus_tim_setcounter(lower->tim, lower->timeout);
    }
  else
    {
      phyplus_stop((struct timer_lowerhalf_s *)lower);
    }

  return OK;
}

/****************************************************************************
 * Name: phyplus_start
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

static int phyplus_start(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct phyplus_lowerhalf_s *priv =
      (FAR struct phyplus_lowerhalf_s *)lower;
  if (!(priv->started))
    {
      /* PHYPLUS_TIM_SETMODE(priv->tim, PHYPLUS_TIM_COUNT); */

      phyplus_tim_setmode(priv->tim, PHYPLUS_TIM_COUNT);
      if (priv->callback != NULL)
        {
          /* PHYPLUS_TIM_SETISR(priv->tim, phyplus_timer_handler, priv);
           * PHYPLUS_TIM_ENABLEINT(priv->tim);
           */

          phyplus_tim_setisr(priv->tim, phyplus_timer_handler, priv);
          phyplus_tim_enableint(priv->tim);
          phyplus_tim_start(priv->tim);      /* chrade add 2021_1008 */
        }

      priv->started = true;
      return OK;
    }

  return -EBUSY;
}

/****************************************************************************
 * Name: phyplus_stop
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

static int phyplus_stop(struct timer_lowerhalf_s *lower)
{
  struct phyplus_lowerhalf_s *priv = (struct phyplus_lowerhalf_s *)lower;

  if (priv->started)
    {
      /* PHYPLUS_TIM_SETMODE(priv->tim, PHYPLUS_TIM_COUNT);
       * PHYPLUS_TIM_DISABLEINT(priv->tim);
       * PHYPLUS_TIM_SETISR(priv->tim, NULL, NULL);
       */

      phyplus_tim_stop(priv->tim);    /* chrade add 2021_1008 */
      phyplus_tim_setmode(priv->tim, PHYPLUS_TIM_COUNT);
      phyplus_tim_disableint(priv->tim);
      phyplus_tim_setisr(priv->tim, NULL, NULL);
      priv->started = false;
      return OK;
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: phyplus_settimeout
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

static int phyplus_settimeout(FAR struct timer_lowerhalf_s *lower,
                            uint32_t timeout)
{
  FAR struct phyplus_lowerhalf_s *priv =
      (FAR struct phyplus_lowerhalf_s *)lower;

  wdinfo("Entry: timeout=%" PRId32 "\n", timeout);
  DEBUGASSERT(priv);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > TMR_MAXTIMEOUT)
    {
      wderr("ERROR: Cannot represent timeout=%" PRId32 " > %" PRId32 "\n",
            timeout, TMR_MAXTIMEOUT);
      return -ERANGE;
    }

  if (priv->started)
    {
      wdwarn("WARNING: Watchdog is already started\n");
      return -EPERM;
    }

  priv->timeout = timeout * 4;

  /* PHYPLUS_TIM_SETCOUNTER(priv->tim, priv->timeout); */

  phyplus_tim_setcounter(priv->tim, priv->timeout);
  return OK;
}

/****************************************************************************
 * Name: phhyplus_setcallback
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

static void phyplus_setcallback(FAR struct timer_lowerhalf_s *lower,
                              tccb_t callback, FAR void *arg)
{
  FAR struct phyplus_lowerhalf_s *priv =
      (FAR struct phyplus_lowerhalf_s *)lower;
  int ret = OK;
  irqstate_t flags = enter_critical_section();

  /* Save the new callback */

  priv->callback = callback;
  priv->arg      = arg;

  if (callback != NULL && priv->started)
    {
      /* ret = PHYPLUS_TIM_SETISR(priv->tim, phyplus_timer_handler, priv); */

      ret = phyplus_tim_setisr(priv->tim, phyplus_timer_handler, priv);

      /* PHYPLUS_TIM_ENABLEINT(priv->tim); */

      phyplus_tim_enableint(priv->tim);
    }
  else
    {
      /* PHYPLUS_TIM_DISABLEINT(priv->tim); */

      phyplus_tim_disableint(priv->tim);

      /* ret = PHYPLUS_TIM_SETISR(priv->tim, NULL, NULL); */

      ret = phyplus_tim_setisr(priv->tim, NULL, NULL);
    }

  leave_critical_section(flags);
  assert(ret == OK);

  /* #if 0
   *  irqstate_t flags = enter_critical_section();
   *
   * # Save the new callback
   *
   *  priv->callback = callback;
   *  priv->arg      = arg;
   *
   *  if (callback != NULL && priv->started)
   *    {
   *      STM32_TIM_SETISR(priv->tim, stm32_timer_handler, priv, 0);
   *      STM32_TIM_ENABLEINT(priv->tim, ATIM_DIER_UIE);
   *    }
   *  else
   *    {
   *      STM32_TIM_DISABLEINT(priv->tim, ATIM_DIER_UIE);
   *      STM32_TIM_SETISR(priv->tim, NULL, NULL, 0);
   *    }
   *  leave_critical_section(flags);
   * #endif
   */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phyplus_timer_initialize
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

int phyplus_timer_initialize(FAR const char *devpath, int timer)
{
  FAR struct phyplus_lowerhalf_s *lower;
  syslog(LOG_ERR, "phuplus_timer_initialiize enter, timer=%d\n", timer);
  switch (timer)
    {
      /* #ifdef CONFIG_PHYPLUS_TIM1 */

      case 1:
        lower = &g_tim1_lowerhalf;
        break;

      /* #endif */

      /* #ifdef CONFIG_PHYPLUS_TIM2 */

      case 2:
        lower = &g_tim2_lowerhalf;
        break;

      /* #endif */

      /* #ifdef CONFIG_PHYPLUS_TIM3 */

      case 3:
        lower = &g_tim3_lowerhalf;
        break;

      /* #endif */

      /* #ifdef CONFIG_PHYPLUS_TIM4 */

      case 4:
        lower = &g_tim4_lowerhalf;
        break;

      /* #endif */

      /* #ifdef CONFIG_PHYPLUS_TIM5 */

      case 5:
        lower = &g_tim5_lowerhalf;
        break;

      /* #endif */

      /* #ifdef CONFIG_PHYPLUS_TIM6 */

      case 6:
        lower = &g_tim6_lowerhalf;
        break;

      /* #endif */

      default:
        syslog(LOG_ERR, "err1");
        return -ENODEV;
    }

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->callback = NULL;

  /*  lower->tim      = stm32_tim_init(timer); */

  lower->tim        = phyplus_tim_init(timer);

  if (lower->tim == NULL)
    {
      syslog(LOG_ERR, "err2");
      return -EINVAL;
    }

  /* Register the timer driver as /dev/timerX.  The returned value from
   * timer_register is a handle that could be uswithed  timer_unregister().
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

      syslog(LOG_ERR, "err3");
      return -EEXIST;
    }

  syslog(LOG_ERR, "phuplus_timer_initialiize out\n");
  return OK;
}

/****************************************************************************
 * Name: phyplus_getstatus
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

static int phyplus_getstatus(FAR struct timer_lowerhalf_s *lower,
                             FAR struct timer_status_s *status)
{
  FAR struct phyplus_lowerhalf_s *priv =
      (FAR struct stm32l4_lowerhalf_s *)lower;
  uint32_t value;

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

  phyplus_tim_getcounter(priv->tim, &value);
  status->timeout = value;

  /* Get the time remaining until the timer expires (in microseconds) */

  phyplus_tim_getcurrent(priv->tim, &value);
  status->timeleft = value;

  phyplus_tim_getcontrolreg(priv->tim, &value);
  status->flags |= ((value & 0xff) << 8);

  return OK;
}

int phyplus_timer_register(FAR struct phyplus_timer_param_s
                           *phyplus_timer_param)
{
  FAR const char *fmt;
  char devname[16];
  int ret;
  fmt = "/dev/timer%u";

  if ((phyplus_timer_param->timer_idx < 1) ||
      (phyplus_timer_param->timer_idx > 6))
    {
      return -ENODEV;
    }

  snprintf(devname, 16, fmt, (unsigned int)phyplus_timer_param->timer_idx);
  return phyplus_timer_initialize(devname, phyplus_timer_param->timer_idx);
}

int phyplus_timer_ungister(FAR struct phyplus_timer_param_s
                           *phyplus_timer_param)
{
  return phyplus_timer_uninitialize(phyplus_timer_param->timer_idx);
}

#endif

