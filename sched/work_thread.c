/****************************************************************************
 * sched/work_thread.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#include <unistd.h>
#include <queue.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include "os_internal.h"
#include "work_internal.h"

#ifdef CONFIG_SCHED_WORKQUEUE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* The queue of pending work */

struct dq_queue_s g_work;

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: work_thread
 *
 * Description:
 *   This is the main worker thread that performs actions placed on the work
 *   list.  It also performs periodic garbage collection (that is performed
 *   by the idle thread if CONFIG_SCHED_WORKQUEUE is not defined).
 *
 * Input parameters:
 *   argc, argv (not used)
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

int work_thread(int argc, char *argv[])
{
  volatile FAR struct work_s *work;
  irqstate_t flags;

  /* Loop forever */

  for (;;)
    {
      /* Wait awhile to check the work list.  We will wait here until either
       * the time elapses or until we are awakened by a signal.
       */

      usleep(CONFIG_SCHED_WORKPERIOD);

      /* First, clean-up any delayed memory deallocations */

      sched_garbagecollection();

      /* Then process queued work.  We need to keep interrupts disabled while
       * we process items in the work list.
       */

      flags = irqsave();
      work  = (FAR struct work_s *)g_work.head;
      while (work)
        {
           /* Is this work ready?  It is ready if there is no delay or if
            * the delay has elapsed.
            */

           if (work->delay == 0 || g_system_timer - work->qtime > work->delay)
             {
               /* Remove the work at the head of the list.  And re-enable
                * interrupts while the work is performed.
                */

               (void)dq_remfirst(&g_work);

               /* Do the work.  Re-enable interrupts while the work is being
                * performed... we don't have any idea how long that will take
                */

               irqrestore(flags);
               work->worker(work->arg);

               /* Now, unfortunately, since we re-enabled interrupts we have
                * to start back at the head of the list.
                */

               flags = irqsave();
               work  = (FAR struct work_s *)g_work.head;
             }
           else
             {
               /* This one is not ready, try the next in the list. */

               work = (FAR struct work_s *)work->dq.flink;
             }
        }
      irqrestore(flags);
    }

  return OK; /* To keep some compilers happy */
}
#endif /* CONFIG_SCHED_WORKQUEUE */
