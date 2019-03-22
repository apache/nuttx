/****************************************************************************
 * arch/arm/src/stm32/stm32_freerun.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>

#include "stm32_freerun.h"

#ifdef CONFIG_STM32_FREERUN

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_freerun_handler
 *
 * Description:
 *   Timer interrupt callback.  When the freerun timer counter overflows,
 *   this interrupt will occur.  We will just increment an overflow count.
 *
 * Input Parameters:
 *   tch - The handle that represents the timer state
 *   arg - An opaque argument provided when the interrupt was registered
 *   sr  - The value of the timer interrupt status register at the time
 *         that the interrupt occurred.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_CLOCK_TIMEKEEPING
static int stm32_freerun_handler(int irq, void *context, void *arg)
{
  struct stm32_freerun_s *freerun = (struct stm32_freerun_s *) arg;

  DEBUGASSERT(freerun != NULL && freerun->overflow < UINT32_MAX);
  freerun->overflow++;

  STM32_TIM_ACKINT(freerun->tch, GTIM_SR_UIF);
  return OK;
}
#endif /* CONFIG_CLOCK_TIMEKEEPING */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_freerun_initialize
 *
 * Description:
 *   Initialize the freerun timer wrapper
 *
 * Input Parameters:
 *   freerun    Caller allocated instance of the freerun state structure
 *   chan       Timer counter channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int stm32_freerun_initialize(struct stm32_freerun_s *freerun, int chan,
                             uint16_t resolution)
{
  uint32_t frequency;

  tmrinfo("chan=%d resolution=%d usec\n", chan, resolution);
  DEBUGASSERT(freerun != NULL && resolution > 0);

  /* Get the TC frequency the corresponds to the requested resolution */

  frequency = USEC_PER_SEC / (uint32_t)resolution;
  freerun->frequency = frequency;

  freerun->tch = stm32_tim_init(chan);
  if (!freerun->tch)
    {
      tmrerr("ERROR: Failed to allocate TIM%d\n", chan);
      return -EBUSY;
    }

  STM32_TIM_SETCLOCK(freerun->tch, frequency);

  /* Initialize the remaining fields in the state structure and return
   * success.
   */

  freerun->chan         = chan;
  freerun->width        = STM32_TIM_GETWIDTH(freerun->tch);
  freerun->running      = false;

#ifdef CONFIG_CLOCK_TIMEKEEPING
  freerun->counter_mask = 0xffffffffull;
#endif

#ifndef CONFIG_CLOCK_TIMEKEEPING
  freerun->overflow     = 0;

  /* Set up to receive the callback when the counter overflow occurs */

  STM32_TIM_SETISR(freerun->tch, stm32_freerun_handler, freerun, 0);
#endif

  /* Set timer period */

  STM32_TIM_SETPERIOD(freerun->tch, (uint32_t)((1ull << freerun->width) - 1));

  /* Start the counter */

  STM32_TIM_SETMODE(freerun->tch, STM32_TIM_MODE_UP);

#ifndef CONFIG_CLOCK_TIMEKEEPING
  STM32_TIM_ACKINT(freerun->tch, GTIM_SR_UIF);
  STM32_TIM_ENABLEINT(freerun->tch, GTIM_DIER_UIE);
#endif

  return OK;
}

/****************************************************************************
 * Name: stm32_freerun_counter
 *
 * Description:
 *   Read the counter register of the free-running timer.
 *
 * Input Parameters:
 *   freerun Caller allocated instance of the freerun state structure.  This
 *           structure must have been previously initialized via a call to
 *           stm32_freerun_initialize();
 *   ts      The location in which to return the time from the free-running
 *           timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_CLOCK_TIMEKEEPING

int stm32_freerun_counter(struct stm32_freerun_s *freerun,
                          struct timespec *ts)
{
  uint64_t usec;
  uint32_t counter;
  uint32_t verify;
  uint32_t overflow;
  uint32_t sec;
  int pending;
  irqstate_t flags;

  DEBUGASSERT(freerun && freerun->tch && ts);

  /* Temporarily disable the overflow counter.  NOTE that we have to be
   * careful here because  stm32_tc_getpending() will reset the pending
   * interrupt status.  If we do not handle the overflow here then, it will
   * be lost.
   */

  flags    = enter_critical_section();

  overflow = freerun->overflow;
  counter  = STM32_TIM_GETCOUNTER(freerun->tch);
  pending  = STM32_TIM_CHECKINT(freerun->tch, 0);
  verify   = STM32_TIM_GETCOUNTER(freerun->tch);

  /* If an interrupt was pending before we re-enabled interrupts,
   * then the overflow needs to be incremented.
   */

  if (pending)
    {
      STM32_TIM_ACKINT(freerun->tch, GTIM_SR_UIF);

      /* Increment the overflow count and use the value of the
       * guaranteed to be AFTER the overflow occurred.
       */

      overflow++;
      counter = verify;

      /* Update freerun overflow counter. */

      freerun->overflow = overflow;
    }

  leave_critical_section(flags);

  tmrinfo("counter=%lu (%lu) overflow=%lu, pending=%i\n",
         (unsigned long)counter,  (unsigned long)verify,
         (unsigned long)overflow, pending);
  tmrinfo("frequency=%u\n", freerun->frequency);

  /* Convert the whole thing to units of microseconds.
   *
   *   frequency = ticks / second
   *   seconds   = ticks * frequency
   *   usecs     = (ticks * USEC_PER_SEC) / frequency;
   */

  usec = ((((uint64_t)overflow << freerun->width) +
            (uint64_t)counter) * USEC_PER_SEC) /
         freerun->frequency;

  /* And return the value of the timer */

  sec         = (uint32_t)(usec / USEC_PER_SEC);
  ts->tv_sec  = sec;
  ts->tv_nsec = (usec - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

  tmrinfo("usec=%llu ts=(%u, %lu)\n",
          usec, (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);

  return OK;
}

#else /* CONFIG_CLOCK_TIMEKEEPING */

int stm32_freerun_counter(struct stm32_freerun_s *freerun, uint64_t *counter)
{
  *counter = (uint64_t)STM32_TIM_GETCOUNTER(freerun->tch);
  return OK;
}

#endif /* CONFIG_CLOCK_TIMEKEEPING */

/****************************************************************************
 * Name: stm32_freerun_uninitialize
 *
 * Description:
 *   Stop the free-running timer and release all resources that it uses.
 *
 * Input Parameters:
 *   freerun Caller allocated instance of the freerun state structure.  This
 *           structure must have been previously initialized via a call to
 *           stm32_freerun_initialize();
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int stm32_freerun_uninitialize(struct stm32_freerun_s *freerun)
{
  DEBUGASSERT(freerun && freerun->tch);

  /* Now we can disable the timer interrupt and disable the timer. */

  STM32_TIM_DISABLEINT(freerun->tch, GTIM_DIER_UIE);
  STM32_TIM_SETMODE(freerun->tch, STM32_TIM_MODE_DISABLED);
  STM32_TIM_SETISR(freerun->tch, NULL, NULL, 0);

  /* Free the timer */

  stm32_tim_deinit(freerun->tch);
  freerun->tch = NULL;

  return OK;
}

#endif /* CONFIG_STM32_ONESHOT */
