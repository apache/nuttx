/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_freerun.c
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
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>

#include "pic32mz_freerun.h"

#ifdef CONFIG_PIC32MZ_FREERUN

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_CLOCK_TIMEKEEPING
static int pic32mz_freerun_handler(int irq, void *context, void *arg);
#endif /* CONFIG_CLOCK_TIMEKEEPING */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_freerun_handler
 *
 * Description:
 *   Timer interrupt callback.  When the freerun timer counter overflows,
 *   this interrupt will occur.  We will just increment an overflow count.
 *
 * Input Parameters:
 *   irq      Number of the IRQ that generated the interrupt
 *   context  Interrupt register state save info (architecture-specific)
 *   arg      An opaque argument provided when the interrupt was registered
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

#ifndef CONFIG_CLOCK_TIMEKEEPING
static int pic32mz_freerun_handler(int irq, void *context, void *arg)
{
  struct pic32mz_freerun_s *freerun = (struct pic32mz_freerun_s *) arg;

  DEBUGASSERT(freerun != NULL && freerun->overflow < UINT32_MAX);
  freerun->overflow++;

  PIC32MZ_TIMER_ACKINT(freerun->timer);

  return OK;
}
#endif /* CONFIG_CLOCK_TIMEKEEPING */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_freerun_initialize
 *
 * Description:
 *   Initialize the freerun timer wrapper
 *
 * Input Parameters:
 *   freerun    Caller allocated instance of the freerun state structure
 *   chan       Timer counter channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds. NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int pic32mz_freerun_initialize(struct pic32mz_freerun_s *freerun, int chan,
                               uint16_t resolution)
{
  uint32_t freq;

  tmrinfo("chan=%d resolution=%d usec\n", chan, resolution);
  DEBUGASSERT(freerun != NULL && resolution > 0);

  freerun->timer = pic32mz_timer_init(chan);
  if (!freerun->timer)
    {
      tmrerr("ERROR: Failed to allocate timer%d\n", chan);
      return -EBUSY;
    }

  /* Get the timer's frequency that corresponds to the requested resolution */

  freq = USEC_PER_SEC / (uint32_t)resolution;

  tmrinfo("Setting frequency=%luHz\n", freq);

  if (!PIC32MZ_TIMER_SETFREQ(freerun->timer, freq))
    {
      tmrerr("Cannot set frequency=%luHz\n", freq);
      return -EAGAIN;
    }

  /* Initialize the remaining fields in the state structure.
   *
   * The timer's frequency might not be the same as requested,
   * due to the lack of prescale values. Get it from the driver.
   */

  freerun->freq    = PIC32MZ_TIMER_GETFREQ(freerun->timer);
  freerun->width   = PIC32MZ_TIMER_GETWIDTH(freerun->timer);
  freerun->chan    = chan;

#ifdef CONFIG_CLOCK_TIMEKEEPING
  if (freerun->width == 32)
    {
      freerun->counter_mask = 0xffffffffull;
    }
  else
    {
      freerun->counter_mask = 0x0000ffffull;
    }
#endif

#ifndef CONFIG_CLOCK_TIMEKEEPING
  freerun->overflow = 0;

  /* Set up to receive the callback when the counter overflow occurs */

  PIC32MZ_TIMER_ACKINT(freerun->timer);
  PIC32MZ_TIMER_SETISR(freerun->timer, pic32mz_freerun_handler, freerun);
#endif

  /* Set the period */

  PIC32MZ_TIMER_SETPERIOD(freerun->timer,
                          (uint32_t)((1ull << freerun->width) - 1ul));

  /* Start the timer */

  PIC32MZ_TIMER_START(freerun->timer);

  return OK;
}

/****************************************************************************
 * Name: pic32mz_freerun_counter
 *
 * Description:
 *   Read the counter register of the free-running timer.
 *
 * Input Parameters:
 *   freerun Caller allocated instance of the freerun state structure.  This
 *           structure must have been previously initialized via a call to
 *           pic32mz_freerun_initialize();
 *   ts      The location in which to return the time from the free-running
 *           timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_CLOCK_TIMEKEEPING

int pic32mz_freerun_counter(struct pic32mz_freerun_s *freerun,
                            struct timespec *ts)
{
  uint64_t usec;
  uint32_t counter;
  uint32_t verify;
  uint32_t overflow;
  uint32_t sec;
  bool pending;
  irqstate_t flags;

  DEBUGASSERT(freerun && freerun->timer && ts);

  /* Temporarily disable the overflow counter. */

  flags    = enter_critical_section();

  overflow = freerun->overflow;
  counter  = PIC32MZ_TIMER_GETCOUNTER(freerun->timer);
  pending  = PIC32MZ_TIMER_CHECKINT(freerun->timer);
  verify   = PIC32MZ_TIMER_GETCOUNTER(freerun->timer);

  /* If an interrupt was pending before we re-enabled interrupts,
   * then the overflow needs to be incremented.
   */

  if (pending)
    {
      PIC32MZ_TIMER_ACKINT(freerun->timer);

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
  tmrinfo("frequency=%u\n", freerun->freq);

  /* Convert the whole thing to units of microseconds.
   *
   *   frequency = ticks / second
   *   seconds   = ticks * frequency
   *   usecs     = (ticks * USEC_PER_SEC) / frequency;
   */

  usec = ((((uint64_t)overflow << freerun->width) +
            (uint64_t)counter) * USEC_PER_SEC) / freerun->freq;

  /* And return the value of the timer */

  sec         = (uint32_t)(usec / USEC_PER_SEC);
  ts->tv_sec  = sec;
  ts->tv_nsec = (usec - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

  tmrinfo("usec=%llu ts=(%u, %lu)\n",
          usec, (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);

  return OK;
}

#else /* CONFIG_CLOCK_TIMEKEEPING */

int pic32mz_freerun_counter(struct pic32mz_freerun_s *freerun,
                            uint64_t *counter)
{
  *counter = (uint64_t)PIC32MZ_TIMER_GETCOUNTER(freerun->timer) &
              freerun->counter_mask;
  return OK;
}

#endif /* CONFIG_CLOCK_TIMEKEEPING */

/****************************************************************************
 * Name: pic32mz_freerun_uninitialize
 *
 * Description:
 *   Stop the free-running timer and release all resources that it uses.
 *
 * Input Parameters:
 *   freerun Caller allocated instance of the freerun state structure.  This
 *           structure must have been previously initialized via a call to
 *           pic32mz_freerun_initialize();
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int pic32mz_freerun_uninitialize(struct pic32mz_freerun_s *freerun)
{
  DEBUGASSERT(freerun && freerun->timer);

  /* Now we can disable the timer interrupt */

  PIC32MZ_TIMER_SETISR(freerun->timer, NULL, NULL);

  /* Free the timer, this will stop the timer as well */

  pic32mz_timer_deinit(freerun->timer);

  freerun->timer = NULL;

  return OK;
}

#endif /* CONFIG_PIC32MZ_FREERUN */
