/****************************************************************************
 * arch/arm/src/sama5/sam_freerun.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The Atmel sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2011, Atmel Corporation
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

/* References:
 *
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 */

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

#include "sam_freerun.h"

#ifdef CONFIG_SAMA5_FREERUN

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
 *              definitions in arch/arm/src/sama5/sam_tc.h.
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
  uint32_t divisor;
  uint32_t cmr;
  int ret;

  tmrinfo("chan=%d resolution=%d usec\n", chan, resolution);
  DEBUGASSERT(freerun && resolution > 0);

  /* Get the TC frequency the corresponds to the requested resolution */

  frequency = USEC_PER_SEC / (uint32_t)resolution;

  /* The pre-calculate values to use when we start the timer */

  ret = sam_tc_divisor(frequency, &divisor, &cmr);
  if (ret < 0)
    {
      tmrerr("ERROR: sam_tc_divisor failed: %d\n", ret);
      return ret;
    }

  tmrinfo("frequency=%lu, divisor=%u, cmr=%08lx\n",
          (unsigned long)frequency, (unsigned long)divisor,
          (unsigned long)cmr);

  /* Allocate the timer/counter and select its mode of operation
   *
   *   CMR_TCCLKS          - Returned by sam_tc_divisor
   *   TC_CMR_CLKI=0       - Not inverted
   *   TC_CMR_BURST_NONE   - Not gated by an external signal
   *   TC_CMR_CPCSTOP=0    - Don't stop the clock on an RC compare event
   *   TC_CMR_CPCDIS=0     - Don't disable the clock on an RC compare event
   *   TC_CMR_EEVTEDG_NONE - No external events (and, hence, no edges
   *   TC_CMR_EEVT_TIOB    - ???? REVISIT
   *   TC_CMR_ENET=0       - External event trigger disabled
   *   TC_CMR_WAVSEL_UP    - TC_CV is incremented from 0 to 0xffffffff
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

  cmr |= (TC_CMR_BURST_NONE  | TC_CMR_EEVTEDG_NONE | TC_CMR_EEVT_TIOB   |
          TC_CMR_WAVSEL_UP   | TC_CMR_WAVE         | TC_CMR_ACPA_NONE   |
          TC_CMR_ACPC_NONE   | TC_CMR_AEEVT_NONE   | TC_CMR_ASWTRG_NONE |
          TC_CMR_BCPB_NONE   | TC_CMR_BCPC_NONE    | TC_CMR_BEEVT_NONE  |
          TC_CMR_BSWTRG_NONE);

  freerun->tch = sam_tc_allocate(chan, cmr);
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

  sam_tc_attach(freerun->tch, sam_freerun_handler, freerun,
                TC_INT_COVFS);

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
  uint32_t sr;
  uint32_t overflow;
  uint32_t sec;
  irqstate_t flags;

  DEBUGASSERT(freerun && freerun->tch && ts);

  /* Temporarily disable the overflow counter.
   * NOTE that we have to be careful here because  sam_tc_getpending() will
   * reset the pending interrupt status.
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

  if ((sr & TC_INT_COVFS) != 0)
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

  tmrinfo("counter=%lu (%lu) overflow=%lu, sr=%08lx\n",
          (unsigned long)counter,  (unsigned long)verify,
          (unsigned long)overflow, (unsigned long)sr);

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

#endif /* CONFIG_SAMA5_FREERUN */
