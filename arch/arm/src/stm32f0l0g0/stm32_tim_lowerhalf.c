/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_tim_lowerhalf.c
 *
 *   Copyright (C) 2019 Fundação CERTI. All rights reserved.
 *   Author: Daniel Pereira Volpato <dpo@certi.org.br>
 *
 *   Based on: arch/arm/src/stm32l4/stm32l4_tim_lowerhalf.c
 *   Authors: Wail Khemir <khemirwail@gmail.com>
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *            dev@ziggurat29.com
 *            Sebastien Lorquet <sebastien@lorquet.fr>
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

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>

#include <arch/board/board.h>

#include "stm32_tim.h"

#if defined(CONFIG_TIMER) && \
    (defined(CONFIG_STM32F0L0G0_TIM1)  || defined(CONFIG_STM32F0L0G0_TIM2)  || \
     defined(CONFIG_STM32F0L0G0_TIM3)  || defined(CONFIG_STM32F0L0G0_TIM4)  || \
     defined(CONFIG_STM32F0L0G0_TIM5)  || defined(CONFIG_STM32F0L0G0_TIM6)  || \
     defined(CONFIG_STM32F0L0G0_TIM7)  || defined(CONFIG_STM32F0L0G0_TIM8)  || \
     defined(CONFIG_STM32F0L0G0_TIM12) || defined(CONFIG_STM32F0L0G0_TIM13) || \
     defined(CONFIG_STM32F0L0G0_TIM14) || defined(CONFIG_STM32F0L0G0_TIM15) || \
     defined(CONFIG_STM32F0L0G0_TIM16) || defined(CONFIG_STM32F0L0G0_TIM17))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_TIM1_RES   16
#define STM32_TIM2_RES   16
#define STM32_TIM3_RES   16
#define STM32_TIM4_RES   16
#define STM32_TIM5_RES   16
#define STM32_TIM6_RES   16
#define STM32_TIM7_RES   16
#define STM32_TIM8_RES   16
#define STM32_TIM9_RES   16
#define STM32_TIM10_RES  16
#define STM32_TIM11_RES  16
#define STM32_TIM12_RES  16
#define STM32_TIM13_RES  16
#define STM32_TIM14_RES  16
#define STM32_TIM15_RES  16
#define STM32_TIM16_RES  16
#define STM32_TIM17_RES  16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct stm32_lowerhalf_s
{
  const struct timer_ops_s *ops;        /* Lower half operations */
  struct stm32_tim_dev_s   *tim;        /* stm32 timer driver */
  tccb_t                    callback;   /* Current user interrupt callback */
  void                     *arg;        /* Argument passed to upper half callback */
  bool                      started;    /* True: Timer has been started */
  const uint8_t             resolution; /* Number of bits in the timer (16 or 32 bits) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_timer_handler(int irq, void * context, void * arg);

/* "Lower half" driver methods **********************************************/

static int stm32_start(struct timer_lowerhalf_s *lower);
static int stm32_stop(struct timer_lowerhalf_s *lower);
static int stm32_getstatus(struct timer_lowerhalf_s *lower,
                           struct timer_status_s *status);
static int stm32_settimeout(struct timer_lowerhalf_s *lower,
                            uint32_t timeout);
static void stm32_setcallback(struct timer_lowerhalf_s *lower,
                              tccb_t callback, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = stm32_start,
  .stop        = stm32_stop,
  .getstatus   = stm32_getstatus,
  .settimeout  = stm32_settimeout,
  .setcallback = stm32_setcallback,
  .ioctl       = NULL,
};

#ifdef CONFIG_STM32F0L0G0_TIM1
static struct stm32_lowerhalf_s g_tim1_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = STM32_TIM1_RES,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM2
static struct stm32_lowerhalf_s g_tim2_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = STM32_TIM2_RES,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM3
static struct stm32_lowerhalf_s g_tim3_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = STM32_TIM3_RES,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM4
static struct stm32_lowerhalf_s g_tim4_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = STM32_TIM4_RES,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM5
static struct stm32_lowerhalf_s g_tim5_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = STM32_TIM5_RES,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM6
static struct stm32_lowerhalf_s g_tim6_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = STM32_TIM6_RES,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM7
static struct stm32_lowerhalf_s g_tim7_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = STM32_TIM7_RES,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM8
static struct stm32_lowerhalf_s g_tim8_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = STM32_TIM8_RES,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM12
static struct stm32_lowerhalf_s g_tim12_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = STM32_TIM12_RES,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM13
static struct stm32_lowerhalf_s g_tim13_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = STM32_TIM13_RES,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM14
static struct stm32_lowerhalf_s g_tim14_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = STM32_TIM14_RES,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM15
static struct stm32_lowerhalf_s g_tim15_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = STM32_TIM15_RES,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM16
static struct stm32_lowerhalf_s g_tim16_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = STM32_TIM16_RES,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM17
static struct stm32_lowerhalf_s g_tim17_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = STM32_TIM17_RES,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_timer_handler
 *
 * Description:
 *   timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int stm32_timer_handler(int irq, void * context, void * arg)
{
  struct stm32_lowerhalf_s *lower = (struct stm32_lowerhalf_s *) arg;
  uint32_t next_interval_us = 0;

  STM32_TIM_ACKINT(lower->tim, ATIM_DIER_UIE);

  if (lower->callback(&next_interval_us, lower->arg))
    {
      if (next_interval_us > 0)
        {
          STM32_TIM_SETPERIOD(lower->tim, next_interval_us);
        }
    }
  else
    {
      stm32_stop((struct timer_lowerhalf_s *)lower);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_start
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

static int stm32_start(struct timer_lowerhalf_s *lower)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;

  tmrinfo("Start\n");

  if (!priv->started)
    {
      STM32_TIM_SETMODE(priv->tim, STM32_TIM_MODE_UP);

      if (priv->callback != NULL)
        {
          STM32_TIM_SETISR(priv->tim, stm32_timer_handler, priv, 0);
          STM32_TIM_ENABLEINT(priv->tim, ATIM_DIER_UIE);
        }

      priv->started = true;
      return OK;
    }

  /* Return EBUSY to indicate that the timer was already running */

  return -EBUSY;
}

/****************************************************************************
 * Name: stm32_stop
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

static int stm32_stop(struct timer_lowerhalf_s *lower)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;

  if (priv->started)
    {
      STM32_TIM_SETMODE(priv->tim, STM32_TIM_MODE_DISABLED);
      STM32_TIM_DISABLEINT(priv->tim, ATIM_DIER_UIE);
      STM32_TIM_SETISR(priv->tim, NULL, NULL, 0);
      priv->started = false;
      return OK;
    }

  /* Return ENODEV to indicate that the timer was not running */

  return -ENODEV;
}

/****************************************************************************
 * Name: stm32_getstatus
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

static int stm32_getstatus(struct timer_lowerhalf_s *lower,
                           struct timer_status_s *status)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;
  uint32_t timeout;
  uint32_t clock;
  uint32_t period;
  uint32_t counter;

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

  clock      = STM32_TIM_GETCLOCK(priv->tim);
  period     = STM32_TIM_GETPERIOD(priv->tim);

  if (clock == 1000000)
    {
      timeout = period;
    }
  else
    {
      timeout = ((uint64_t) period * 1000000) / clock;
    }

  status->timeout = timeout;

  /* Get the time remaining until the timer expires (in microseconds) */

  counter    = STM32_TIM_GETCOUNTER(priv->tim);
  status->timeleft = ((uint64_t) (timeout - counter) * clock) / 1000000;
  tmrinfo("timeout=%" PRIu32 " counter=%" PRIu32 "\n", timeout, counter);
  tmrinfo("timeleft=%" PRIu32 "\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: stm32_settimeout
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

static int stm32_settimeout(struct timer_lowerhalf_s *lower,
                            uint32_t timeout)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;
  uint64_t maxtimeout;
  uint32_t clock;
  uint32_t period;

  if (priv->started)
    {
      return -EPERM;
    }

  tmrinfo("Set timeout=%" PRId32 "\n", timeout);

  maxtimeout = ((uint64_t)1 << priv->resolution) - 1;
  if (timeout > maxtimeout)
    {
      uint64_t freq = (maxtimeout * 1000000) / timeout;
      clock = (uint32_t) freq;
      period = (uint32_t) maxtimeout;
    }
  else
    {
      clock = (uint32_t) 1000000;
      period = (uint32_t) timeout;
    }

  tmrinfo("  clock=%lu period=%lu maxtimeout=%lu\n", clock, period,
          (uint32_t)maxtimeout);
  STM32_TIM_SETCLOCK(priv->tim, clock);
  STM32_TIM_SETPERIOD(priv->tim, period);

  return OK;
}

/****************************************************************************
 * Name: stm32_setcallback
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

static void stm32_setcallback(struct timer_lowerhalf_s *lower,
                              tccb_t callback, void *arg)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;

  irqstate_t flags = enter_critical_section();

  /* Save the new callback */

  priv->callback = callback;
  priv->arg      = arg;

  if (callback != NULL && priv->started)
    {
      STM32_TIM_SETISR(priv->tim, stm32_timer_handler, priv, 0);
      STM32_TIM_ENABLEINT(priv->tim, ATIM_DIER_UIE);
    }
  else
    {
      STM32_TIM_DISABLEINT(priv->tim, ATIM_DIER_UIE);
      STM32_TIM_SETISR(priv->tim, NULL, NULL, 0);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_timer_initialize
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

int stm32_timer_initialize(const char *devpath, int timer)
{
  struct stm32_lowerhalf_s *lower;

  tmrinfo("Init TIM%d\n", timer);

  switch (timer)
    {
#ifdef CONFIG_STM32F0L0G0_TIM1
      case 1:
        lower = &g_tim1_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM2
      case 2:
        lower = &g_tim2_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM3
      case 3:
        lower = &g_tim3_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM4
      case 4:
        lower = &g_tim4_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM5
      case 5:
        lower = &g_tim5_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM6
      case 6:
        lower = &g_tim6_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM7
      case 7:
        lower = &g_tim7_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM8
      case 8:
        lower = &g_tim8_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM12
      case 12:
        lower = &g_tim12_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM13
      case 13:
        lower = &g_tim13_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM14
      case 14:
        lower = &g_tim14_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM15
      case 15:
        lower = &g_tim15_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM16
      case 16:
        lower = &g_tim16_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM17
      case 17:
        lower = &g_tim17_lowerhalf;
        break;
#endif
      default:
        return -ENODEV;
    }

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->callback = NULL;
  lower->tim      = stm32_tim_init(timer);

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
