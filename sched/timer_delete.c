/********************************************************************************
 * timer_delete.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <nuttx/kmalloc.h>
#include "timer_internal.h"

#ifndef CONFIG_DISABLE_POSIX_TIMERS

/********************************************************************************
 * Definitions
 ********************************************************************************/

/********************************************************************************
 * Private Data
 ********************************************************************************/

/********************************************************************************
 * Public Data
 ********************************************************************************/

/********************************************************************************
 * Private Functions
 ********************************************************************************/

/********************************************************************************
 * Function:  timer_free
 *
 * Description:
 *   Remove the timer from the allocated timer list and free it or return it to
 *   the free list (depending on whether or not the timer is one of the
 *   preallocated timers)
 *
 ********************************************************************************/

static void timer_free(struct posix_timer_s *timer)
{
  irqstate_t flags;

  /* Remove the timer from the allocated list */

  flags = irqsave();
  sq_rem((FAR sq_entry_t*)timer, (sq_queue_t*)&g_alloctimers);

  /* Return it to the free list if it is one of the preallocated timers */

#if CONFIG_PREALLOC_TIMERS > 0
  if ((timer->pt_flags & PT_FLAGS_PREALLOCATED) != 0)
    {
      sq_addlast((FAR sq_entry_t*)timer, (FAR sq_queue_t*)&g_freetimers);
      irqrestore(flags);
    }
  else
#endif
    {
      /* Otherwise, return it to the heap */

      irqrestore(flags);
      sched_free(timer);
    }
}

/********************************************************************************
 * Public Functions
 ********************************************************************************/

/********************************************************************************
 * Function:  timer_delete
 *
 * Description:
 *   The timer_delete() function deletes the specified timer, timerid, previously
 *   created by the timer_create() function. If the timer is armed when
 *   timer_delete() is called, the timer will be automatically disarmed before
 *   removal. The disposition of pending signals for the deleted timer is unspecified.
 *
 * Parameters:
 *   timerid - The pre-thread timer, previously created by the call to
 *   timer_create(), to be deleted.
 *
 * Return Value:
 *   If the call succeeds, timer_create() will return 0 (OK).  Otherwise, the
 *   function will return a value of -1 (ERROR) and set errno to indicate the error.
 *
 *   EINVAL - The timer specified timerid is not valid.
 *
 * Assumptions:
 *
 ********************************************************************************/

int timer_delete(timer_t timerid)
{
  FAR struct posix_timer_s *timer = (FAR struct posix_timer_s *)timerid;

  /* Some sanity checks */

  if (!timer)
    {
      *get_errno_ptr() = EINVAL;
      return ERROR;
    }

  /* Disarm the timer */

  (void)wd_cancel(timer->pt_wdog);

  /* Release the timer structure */

  timer_free(timer);
  return OK;
}

#endif /* CONFIG_DISABLE_POSIX_TIMERS */
