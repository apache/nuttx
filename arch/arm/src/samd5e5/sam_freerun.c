/****************************************************************************
 * arch/arm/src/samd5e5/sam_freerun.c
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
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>

#include "sam_freerun.h"

#ifdef CONFIG_SAMD5E5_FREERUN

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_freerun_handler
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

static void sam_freerun_handler(TC_HANDLE tch, void *arg, uint32_t sr)
{
  struct sam_freerun_s *freerun = (struct sam_freerun_s *)arg;

  DEBUGASSERT(freerun && freerun->overflow < UINT16_MAX);
  freerun->overflow++;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_freerun_initialize
 *
 * Description:
 *   Initialize the freerun timer wrapper
 *
 * Input Parameters:
 *   freerun    Caller allocated instance of the freerun state structure
 *   chan       Timer counter channel to be used.  See the TC_CHAN*
 *              definitions in arch/arm/src/samd5e5/sam_tc.h.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int sam_freerun_initialize(struct sam_freerun_s *freerun, int chan,
                           uint16_t resolution)
{
  uint32_t frequency;

  tmrinfo("chan=%d resolution=%d usec\n", chan, resolution);
  DEBUGASSERT(freerun && resolution > 0);

  /* Get the TC frequency the corresponds to the requested resolution */

  frequency = USEC_PER_SEC / (uint32_t)resolution;

  freerun->tch = sam_tc_allocate(chan, frequency);
  if (!freerun->tch)
    {
      tmrerr("ERROR: Failed to allocate timer channel %d\n", chan);
      return -EBUSY;
    }

  /* Initialize the remaining fields in the state structure and return
   * success.
   */

  freerun->chan     = chan;
  freerun->overflow = 0;

  /* Set up to receive the callback when the counter overflow occurs */

  sam_tc_attach(freerun->tch, sam_freerun_handler, freerun, TC_INTFLAG_OVF);

  /* Start the counter */

  sam_tc_start(freerun->tch);
  return OK;
}

/****************************************************************************
 * Name: sam_freerun_counter
 *
 * Description:
 *   Read the counter register of the free-running timer.
 *
 * Input Parameters:
 *   freerun Caller allocated instance of the freerun state structure.  This
 *           structure must have been previously initialized via a call to
 *           sam_freerun_initialize();
 *   ts      The location in which to return the time from the free-running
 *           timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int sam_freerun_counter(struct sam_freerun_s *freerun, struct timespec *ts)
{
  uint64_t usec;
  uint32_t counter;
  uint32_t verify;
  uint8_t sr;
  uint32_t overflow;
  uint32_t sec;
  irqstate_t flags;

  DEBUGASSERT(freerun && freerun->tch && ts);

  /* Temporarily disable the overflow counter.
   * NOTE that we have to be here because  sam_tc_getpending()
   * will reset the pending interrupt status.
   * If we do not handle the overflow here then, it will be lost.
   */

  flags    = enter_critical_section();
  overflow = freerun->overflow;
  counter  = sam_tc_getcounter(freerun->tch);
  sr       = sam_tc_getpending(freerun->tch);
  verify   = sam_tc_getcounter(freerun->tch);

  /* If an interrupt was pending before we re-enabled interrupts,
   * then the overflow needs to be incremented.
   */

  if ((sr & TC_INTFLAG_OVF) != 0)
    {
      /* Increment the overflow count and use the value of the
       * guaranteed to be AFTER the overflow occurred.
       */

      overflow++;
      counter = verify;

      /* Update freerun overflow counter. */

      freerun->overflow = overflow;
    }

  leave_critical_section(flags);

  tmrinfo("counter=%u (%u) overflow=%u, sr=0x%x\n",
          counter, verify, overflow, sr);

  /* Convert the whole thing to units of microseconds.
   *
   *   frequency = ticks / second
   *   seconds   = ticks * frequency
   *   usecs     = (ticks * USEC_PER_SEC) / frequency;
   */

  usec = ((((uint64_t)overflow << 32) + (uint64_t)counter) * USEC_PER_SEC) /
                                              sam_tc_divfreq(freerun->tch);

  /* And return the value of the timer */

  sec         = (uint32_t)(usec / USEC_PER_SEC);
  ts->tv_sec  = sec;
  ts->tv_nsec = (usec - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

  tmrinfo("usec=%llu ts=(%lu, %lu)\n",
          usec, (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);

  return OK;
}

/****************************************************************************
 * Name: sam_freerun_uninitialize
 *
 * Description:
 *   Stop the free-running timer and release all resources that it uses.
 *
 * Input Parameters:
 *   freerun Caller allocated instance of the freerun state structure.  This
 *           structure must have been previously initialized via a call to
 *           sam_freerun_initialize();
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int sam_freerun_uninitialize(struct sam_freerun_s *freerun)
{
  DEBUGASSERT(freerun && freerun->tch);

  /* Now we can disable the timer interrupt and disable the timer. */

  sam_tc_detach(freerun->tch);
  sam_tc_stop(freerun->tch);

  /* Free the timer */

  sam_tc_free(freerun->tch);
  freerun->tch = NULL;
  return OK;
}

#endif /* CONFIG_SAMD5E5_FREERUN */
