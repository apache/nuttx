/****************************************************************************
 * sched/sched/sched_gettcb.c
 *
 *   Copyright (C) 2007, 2009, 2011, 2018 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>

#include <sched.h>

#include "nuttx/irq.h"

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_gettcb
 *
 * Description:
 *   Given a task ID, this function will return the a pointer to the
 *   corresponding TCB (or NULL if there is no such task ID).
 *
 *   NOTE:  This function holds a critical section while examining TCB data
 *   data structures but releases that critical section before returning.
 *   When it is released, the TCB may become unstable.  If the caller
 *   requires absolute stability while using the TCB, then the caller
 *   should establish the critical section BEFORE calling this function and
 *   hold that critical section as long as necessary.
 *
 ****************************************************************************/

FAR struct tcb_s *sched_gettcb(pid_t pid)
{
  FAR struct tcb_s *ret = NULL;
  irqstate_t flags;
  int hash_ndx;

  /* Verify that the PID is within range */

  if (pid >= 0)
    {
      /* Get the hash_ndx associated with the pid */

      hash_ndx = PIDHASH(pid);

      /* The test and the return setup should be atomic.  This still does
       * not provide proper protection if the recipient of the TCB does not
       * also protect against the task associated with the TCB from
       * terminating asynchronously.
       */

      flags = enter_critical_section();

      /* Verify that the correct TCB was found. */

      if (pid == g_pidhash[hash_ndx].pid)
        {
          /* Return the TCB associated with this pid (if any) */

          ret = g_pidhash[hash_ndx].tcb;
        }

      leave_critical_section(flags);
    }

  /* Return the TCB. */

  return ret;
}

