/****************************************************************************
 * libc/wqueue/work_usrthread.c
 *
 *   Copyright (C) 2009-2014 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/wqueue.h>

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_USRWORK) && \
   !defined(__KERNEL__)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The state of the user mode work queue. */

extern struct wqueue_s g_usrwork;

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_usrthread
 *
 * Description:
 *   This is the worker thread that performs the actions placed on the user
 *   work queue.
 *
 *   This is a user mode work queue.  It must be used by applications for
 *   miscellaneous operations.  The user work thread must be started by
 *   application start-up logic by calling work_usrstart().
 *
 * Input parameters:
 *   argc, argv (not used)
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

static int work_usrthread(int argc, char *argv[])
{
  /* Loop forever */

  for (;;)
    {
      /* Then process queued work.  We need to keep interrupts disabled while
       * we process items in the work list.
       */

      work_process(&g_usrwork);
    }

  return OK; /* To keep some compilers happy */
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_usrstart
 *
 * Description:
 *   Start the user mode work queue.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   The task ID of the worker thread is returned on success.  A negated
 *   errno value is returned on failure.
 *
 ****************************************************************************/

int work_usrstart(void)
{
  /* Start a user-mode worker thread for use by applications. */

  svdbg("Starting user-mode worker thread\n");

  g_usrwork.pid = task_create("uwork",
                              CONFIG_SCHED_USRWORKPRIORITY,
                              CONFIG_SCHED_USRWORKSTACKSIZE,
                              (main_t)work_usrthread,
                              (FAR char * const *)NULL);

  DEBUGASSERT(g_usrwork.pid > 0);
  if (g_usrwork.pid < 0)
    {
      int errcode = errno;
      DEBUGASSERT(errcode > 0);

      sdbg("task_create failed: %d\n", errcode);
      return -errcode;
    }

  return g_usrwork.pid;
}

#endif /* CONFIG_SCHED_WORKQUEUE && CONFIG_SCHED_USRWORK && !__KERNEL__*/
