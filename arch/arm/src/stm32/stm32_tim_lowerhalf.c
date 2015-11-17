/****************************************************************************
 * arch/arm/src/stm32/stm32_tim_lowerhalf.c
 *
 *   Copyright (C) 2015 Wail Khemir. All rights reserved.
 *   Authors: Wail Khemir <khemirwail@gmail.com>
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

#include <nuttx/timers/timer.h>

#include <arch/board/board.h>

#include "stm32_tim.h"

#if defined(CONFIG_TIMER) && \
    (defined(CONFIG_STM32_TIM1) || defined(CONFIG_STM32_TIM2) || \
     defined(CONFIG_STM32_TIM3) || defined(CONFIG_STM32_TIM4) || \
     defined(CONFIG_STM32_TIM5) || defined(CONFIG_STM32_TIM6) || \
     defined(CONFIG_STM32_TIM7) || defined(CONFIG_STM32_TIM8) )

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

struct stm32_lowerhalf_s
{
  const struct timer_ops_s *ops;        /* Lower half operations */
  struct stm32_tim_dev_s   *tim;        /* stm32 timer driver */
  tccb_t                   handlerUsr;  /* Current user interrupt handler */
  xcpt_t                   handlerTim;  /* Current timer interrupt handler */
  bool                     started;     /* True: Timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions *********************************************************/

static struct stm32_lowerhalf_s* stm32_get_lowerhalf(int timer);
static xcpt_t stm32_get_interrupt(int timer);

/* Interrupt handling *******************************************************/

#ifdef CONFIG_STM32_TIM1
static int stm32_tim1_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM2
static int stm32_tim2_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM3
static int stm32_tim3_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM4
static int stm32_tim4_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM5
static int stm32_tim5_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM6
static int stm32_tim6_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM7
static int stm32_tim7_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM8
static int stm32_tim8_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM9
static int stm32_tim9_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM10
static int stm32_tim10_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM11
static int stm32_tim11_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM12
static int stm32_tim12_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM13
static int stm32_tim13_interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM14
static int stm32_tim14_interrupt(int irq, FAR void *context);
#endif

static int stm32_timer_handler(struct stm32_lowerhalf_s* attr);

/* "Lower half" driver methods **********************************************/

static int stm32_start(struct timer_lowerhalf_s *lower);
static int stm32_stop(struct timer_lowerhalf_s *lower);
static int stm32_settimeout(struct timer_lowerhalf_s *lower,
                            uint32_t timeout);
static tccb_t stm32_sethandler(struct timer_lowerhalf_s *lower,
                               tccb_t handler);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start      = stm32_start,
  .stop       = stm32_stop,
  .getstatus  = 0,
  .settimeout = stm32_settimeout,
  .sethandler = stm32_sethandler,
  .ioctl      = 0,
};

#ifdef CONFIG_STM32_TIM1
static struct stm32_lowerhalf_s g_tim1_lowerHalf;
#endif
#ifdef CONFIG_STM32_TIM2
static struct stm32_lowerhalf_s g_tim2_lowerHalf;
#endif
#ifdef CONFIG_STM32_TIM3
static struct stm32_lowerhalf_s g_tim3_lowerHalf;
#endif
#ifdef CONFIG_STM32_TIM4
static struct stm32_lowerhalf_s g_tim4_lowerHalf;
#endif
#ifdef CONFIG_STM32_TIM5
static struct stm32_lowerhalf_s g_tim5_lowerHalf;
#endif
#ifdef CONFIG_STM32_TIM6
static struct stm32_lowerhalf_s g_tim6_lowerHalf;
#endif
#ifdef CONFIG_STM32_TIM7
static struct stm32_lowerhalf_s g_tim7_lowerHalf;
#endif
#ifdef CONFIG_STM32_TIM8
static struct stm32_lowerhalf_s g_tim8_lowerHalf;
#endif
#ifdef CONFIG_STM32_TIM9
static struct stm32_lowerhalf_s g_tim9_lowerHalf;
#endif
#ifdef CONFIG_STM32_TIM10
static struct stm32_lowerhalf_s g_tim10_lowerHalf;
#endif
#ifdef CONFIG_STM32_TIM11
static struct stm32_lowerhalf_s g_tim11_lowerHalf;
#endif
#ifdef CONFIG_STM32_TIM12
static struct stm32_lowerhalf_s g_tim12_lowerHalf;
#endif
#ifdef CONFIG_STM32_TIM13
static struct stm32_lowerhalf_s g_tim13_lowerHalf;
#endif
#ifdef CONFIG_STM32_TIM14
static struct stm32_lowerhalf_s g_tim14_lowerHalf;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_get_lowerhalf
 *
 * Description:
 *   Get the lower half timer structure of the corresponding timer
 *
 * Input Parameters:
 *   timer - the timer's number
 *
 * Returned Values:
 *   A pointer to the lower half structure on success, NULL on failure
 *
 ****************************************************************************/

static struct stm32_lowerhalf_s* stm32_get_lowerhalf(int timer)
{
  struct stm32_lowerhalf_s* lower;

  switch(timer)
    {
#ifdef CONFIG_STM32_TIM1
    case 1:
      lower = &g_tim1_lowerHalf;
      break;
#endif
#ifdef CONFIG_STM32_TIM2
    case 2:
      lower = &g_tim2_lowerHalf;
      break;
#endif
#ifdef CONFIG_STM32_TIM3
    case 3:
      lower = &g_tim3_lowerHalf;
      break;
#endif
#ifdef CONFIG_STM32_TIM4
    case 4:
      lower = &g_tim4_lowerHalf;
      break;
#endif
#ifdef CONFIG_STM32_TIM5
    case 5:
      lower = &g_tim5_lowerHalf;
      break;
#endif
#ifdef CONFIG_STM32_TIM6
    case 6:
      lower = &g_tim6_lowerHalf;
      break;
#endif
#ifdef CONFIG_STM32_TIM7
    case 7:
      lower = &g_tim7_lowerHalf;
      break;
#endif
#ifdef CONFIG_STM32_TIM8
    case 8:
      lower = &g_tim8_lowerHalf;
      break;
#endif
#ifdef CONFIG_STM32_TIM9
    case 9:
      lower = &g_tim9_lowerHalf;
      break;
#endif
#ifdef CONFIG_STM32_TIM10
    case 10:
      lower = &g_tim10_lowerHalf;
      break;
#endif
#ifdef CONFIG_STM32_TIM11
    case 11:
      lower = &g_tim11_lowerHalf;
      break;
#endif
#ifdef CONFIG_STM32_TIM12
    case 12:
      lower = &g_tim12_lowerHalf;
      break;
#endif
#ifdef CONFIG_STM32_TIM13
    case 13:
      lower = &g_tim13_lowerHalf;
      break;
#endif
#ifdef CONFIG_STM32_TIM14
    case 14:
      lower = &g_tim14_lowerHalf;
      break;
#endif
    default:
      lower = 0;
    }

  return lower;
}

/****************************************************************************
 * Name: stm32_get_interrupt
 *
 * Description:
 *   Get a pointer to the interrupt handler of the corresponding timer
 *
 * Input Parameters:
 *   timer - the timer's number
 *
 * Returned Values:
 *   A pointer to the interrupt handler on success, NULL on failure
 *
 ****************************************************************************/

static xcpt_t stm32_get_interrupt(int timer)
{
  xcpt_t intr;

  switch(timer)
    {
#ifdef CONFIG_STM32_TIM1
    case 1:
      intr = stm32_tim1_interrupt;
      break;
#endif
#ifdef CONFIG_STM32_TIM2
    case 2:
      intr = stm32_tim2_interrupt;
      break;
#endif
#ifdef CONFIG_STM32_TIM3
    case 3:
      intr = stm32_tim3_interrupt;
      break;
#endif
#ifdef CONFIG_STM32_TIM4
    case 4:
      intr = stm32_tim4_interrupt;
      break;
#endif
#ifdef CONFIG_STM32_TIM5
    case 5:
      intr = stm32_tim5_interrupt;
      break;
#endif
#ifdef CONFIG_STM32_TIM6
    case 6:
      intr = stm32_tim6_interrupt;
      break;
#endif
#ifdef CONFIG_STM32_TIM7
    case 7:
      intr = stm32_tim7_interrupt;
      break;
#endif
#ifdef CONFIG_STM32_TIM8
    case 8:
      intr = stm32_tim8_interrupt;
      break;
#endif
#ifdef CONFIG_STM32_TIM9
    case 9:
      intr = stm32_tim9_interrupt;
      break;
#endif
#ifdef CONFIG_STM32_TIM10
    case 10:
      intr = stm32_tim10_interrupt;
      break;
#endif
#ifdef CONFIG_STM32_TIM11
    case 11:
      intr = stm32_tim11_interrupt;
      break;
#endif
#ifdef CONFIG_STM32_TIM12
    case 12:
      intr = stm32_tim12_interrupt;
      break;
#endif
#ifdef CONFIG_STM32_TIM13
    case 13:
      intr = stm32_tim13_interrupt;
      break;
#endif
#ifdef CONFIG_STM32_TIM14
    case 14:
      intr = stm32_tim14_interrupt;
      break;
#endif
    default:
      intr = 0;
    }

  return intr;
}

/****************************************************************************
 * Name: stm32_timN_interrupt, N=1..14
 *
 * Description:
 *   Individual interrupt handlers for each timer
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_TIM1
static int stm32_tim1_interrupt(int irq, FAR void *context)
{
  return stm32_timer_handler(&g_tim1_lowerHalf);
}
#endif

#ifdef CONFIG_STM32_TIM2
static int stm32_tim2_interrupt(int irq, FAR void *context)
{
  return stm32_timer_handler(&g_tim2_lowerHalf);
}
#endif

#ifdef CONFIG_STM32_TIM3
static int stm32_tim3_interrupt(int irq, FAR void *context)
{
  return stm32_timer_handler(&g_tim3_lowerHalf);
}
#endif

#ifdef CONFIG_STM32_TIM4
static int stm32_tim4_interrupt(int irq, FAR void *context)
{
  return stm32_timer_handler(&g_tim4_lowerHalf);
}
#endif

#ifdef CONFIG_STM32_TIM5
static int stm32_tim5_interrupt(int irq, FAR void *context)
{
  return stm32_timer_handler(&g_tim5_lowerHalf);
}
#endif

#ifdef CONFIG_STM32_TIM6
static int stm32_tim6_interrupt(int irq, FAR void *context)
{
  return stm32_timer_handler(&g_tim6_lowerHalf);
}
#endif

#ifdef CONFIG_STM32_TIM7
static int stm32_tim7_interrupt(int irq, FAR void *context)
{
  return stm32_timer_handler(&g_tim7_lowerHalf);
}
#endif

#ifdef CONFIG_STM32_TIM8
static int stm32_tim8_interrupt(int irq, FAR void *context)
{
  return stm32_timer_handler(&g_tim8_lowerHalf);
}
#endif

#ifdef CONFIG_STM32_TIM9
static int stm32_tim9_interrupt(int irq, FAR void *context)
{
  return stm32_timer_handler(&g_tim9_lowerHalf);
}
#endif

#ifdef CONFIG_STM32_TIM10
static int stm32_tim10_interrupt(int irq, FAR void *context)
{
  return stm32_timer_handler(&g_tim10_lowerHalf);
}
#endif

#ifdef CONFIG_STM32_TIM11
static int stm32_tim11_interrupt(int irq, FAR void *context)
{
  return stm32_timer_handler(&g_tim11_lowerHalf);
}
#endif

#ifdef CONFIG_STM32_TIM12
static int stm32_tim12_interrupt(int irq, FAR void *context)
{
  return stm32_timer_handler(&g_tim12_lowerHalf);
}
#endif

#ifdef CONFIG_STM32_TIM13
static int stm32_tim13_interrupt(int irq, FAR void *context)
{
  return stm32_timer_handler(&g_tim13_lowerHalf);
}
#endif

#ifdef CONFIG_STM32_TIM14
static int stm32_tim14_interrupt(int irq, FAR void *context)
{
  return stm32_timer_handler(&g_tim14_lowerHalf);
}
#endif

/****************************************************************************
 * Name: stm32_timer_handler
 *
 * Description:
 *   timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Values:
 *
 ****************************************************************************/

static int stm32_timer_handler(struct stm32_lowerhalf_s* lower)
{
  STM32_TIM_ACKINT(lower->tim, 0);

  uint32_t next_interval_us = 0;
  int ret = (*lower->handlerUsr)(&next_interval_us);

  if(ret == OK)
    {
      if(next_interval_us > 0)
        {
          STM32_TIM_SETPERIOD(lower->tim, next_interval_us);
        }
    }
  else
    {
      stm32_stop(lower);
    }

  return 0;
}

/****************************************************************************
 * Name: stm32_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_start(struct timer_lowerhalf_s *lower)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;

  if (!priv->started)
    {
      STM32_TIM_SETCLOCK(priv->tim, 1000000); // 1000000 Hz = 1 microsecond
      STM32_TIM_SETMODE(priv->tim, STM32_TIM_MODE_UP);

      if(priv->handlerUsr)
        {
          STM32_TIM_SETISR(priv->tim, priv->handlerTim, 0);
          STM32_TIM_ENABLEINT(priv->tim, 0);
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
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_stop(struct timer_lowerhalf_s *lower)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;

  if (priv->started)
    {
      STM32_TIM_SETMODE(priv->tim, STM32_TIM_MODE_DISABLED);
      STM32_TIM_DISABLEINT(priv->tim, 0);
      STM32_TIM_SETISR(priv->tim, 0, 0);
      priv->started = false;
      return OK;
    }

  /* Return ENODEV to indicate that the timer was not running */

  return -ENODEV;
}

/****************************************************************************
 * Name: stm32_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-half"
 *             driver state structure.
 *   timeout - The new timeout value in microseconds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_settimeout(struct timer_lowerhalf_s *lower, uint32_t timeout)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;

  if (priv->started)
    {
      return -EPERM;
    }

  STM32_TIM_SETPERIOD(priv->tim, timeout);
  return OK;
}

/****************************************************************************
 * Name: stm32_sethandler
 *
 * Description:
 *   Call this user provided timeout handler.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the "lower-half"
 *                driver state structure.
 *   newhandler - The new timer expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Values:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static tccb_t stm32_sethandler(struct timer_lowerhalf_s *lower,
                               tccb_t newhandler)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;

  irqstate_t flags = irqsave();

  /* Get the old handler return value */

  tccb_t oldhandler = priv->handlerUsr;

  /* Save the new handler */

  priv->handlerUsr = newhandler;

  if(newhandler)
    {
      STM32_TIM_SETISR(priv->tim, priv->handlerTim, 0);
      STM32_TIM_ENABLEINT(priv->tim, 0);
    }
  else
    {
      STM32_TIM_DISABLEINT(priv->tim, 0);
      STM32_TIM_SETISR(priv->tim, 0, 0);
    }

  irqrestore(flags);
  return oldhandler;
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
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int stm32_timer_initialize(FAR const char *devpath, int timer)
{
  struct stm32_lowerhalf_s *lower = stm32_get_lowerhalf(timer);
  memset(lower, 0, sizeof(struct stm32_lowerhalf_s));

  /* Initialize the non-zero elements of lower half state structure */

  lower->ops        = &g_timer_ops;
  lower->handlerTim = stm32_get_interrupt(timer);
  lower->tim        = stm32_tim_init(timer);

  if(!lower->tim)
    {
      return -EINVAL;
    }

  /* Register the timer driver as /dev/timerX.  The returned value from
   * timer_register is a handle that could be used with timer_unregister().
   * REVISIT: The returned handle is discard here.
   */

  void *drvr = timer_register(devpath, (struct timer_lowerhalf_s *)lower);
  if (!drvr)
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
