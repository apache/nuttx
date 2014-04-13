/****************************************************************************
 * sched/os_bringup.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * With extensions by:
 *
 *   Author: Uros Platise <uros.platise@isotel.eu>
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

#include <sched.h>
#include <stdlib.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/wqueue.h>
#include <nuttx/kthread.h>
#include <nuttx/userspace.h>

#include "os_internal.h"
#ifdef CONFIG_PAGING
# include "pg_internal.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If NuttX is built as a separately compiled module, then the config.h header
 * file should contain the address of the user module entry point.  If not
 * then the default entry point is user_start.
 */

/* Customize some strings */

#ifdef CONFIG_SCHED_WORKQUEUE
#  ifdef CONFIG_SCHED_HPWORK
#    if defined(CONFIG_SCHED_LPWORK)
#      define HPWORKNAME "hpwork"
#      define LPWORKNAME "lpwork"
#    elif defined(CONFIG_SCHED_USRWORK)
#      define HPWORKNAME "knlwork"
#    else
#      define HPWORKNAME "work"
#    endif
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: os_bringup
 *
 * Description:
 *   Start all initial system tasks.  This does the "system bring-up" after
 *   the conclusion of basic OS initialization.  These initial system tasks
 *   may include:
 *
 *   - pg_worker:   The page-fault worker thread (only if CONFIG_PAGING is
 *                  defined.
 *   - work_thread: The work thread.  This general thread can be used to
 *                  perform most any kind of queued work.  Its primary
 *                  function is to serve as the "bottom half" of device
 *                  drivers.
 *
 *   And the main application entry point:
 *   symbols:
 *
 *   - USER_ENTRYPOINT: This is the default user application entry point.
 *
 ****************************************************************************/

int os_bringup(void)
{
  int taskid;

  /* Setup up the initial environment for the idle task.  At present, this
   * may consist of only the initial PATH variable.  The PATH variable is
   * (probably) not used by the IDLE task.  However, the environment
   * containing the PATH variable will be inherited by all of the threads
   * created by the IDLE task.
   */

#if !defined(CONFIG_DISABLE_ENVIRON) && defined(CONFIG_PATH_INITIAL)
  (void)setenv("PATH", CONFIG_PATH_INITIAL, 1);
#endif

  /* Start the page fill worker kernel thread that will resolve page faults.
   * This should always be the first thread started because it may have to
   * resolve page faults in other threads
   */

#ifdef CONFIG_PAGING
  svdbg("Starting paging thread\n");

  g_pgworker = KERNEL_THREAD("pgfill", CONFIG_PAGING_DEFPRIO,
                             CONFIG_PAGING_STACKSIZE,
                             (main_t)pg_worker, (FAR char * const *)NULL);
  DEBUGASSERT(g_pgworker > 0);
#endif

  /* Start the worker thread that will serve as the device driver "bottom-
   * half" and will perform misc garbage clean-up.
   */

#ifdef CONFIG_SCHED_WORKQUEUE
#ifdef CONFIG_SCHED_HPWORK

#ifdef CONFIG_SCHED_LPWORK
  svdbg("Starting high-priority kernel worker thread\n");
#else
  svdbg("Starting kernel worker thread\n");
#endif

  g_work[HPWORK].pid = KERNEL_THREAD(HPWORKNAME, CONFIG_SCHED_WORKPRIORITY,
                                     CONFIG_SCHED_WORKSTACKSIZE,
                                     (main_t)work_hpthread, (FAR char * const *)NULL);
  DEBUGASSERT(g_work[HPWORK].pid > 0);

  /* Start a lower priority worker thread for other, non-critical continuation
   * tasks
   */

#ifdef CONFIG_SCHED_LPWORK

  svdbg("Starting low-priority kernel worker thread\n");

  g_work[LPWORK].pid = KERNEL_THREAD(LPWORKNAME, CONFIG_SCHED_LPWORKPRIORITY,
                                     CONFIG_SCHED_LPWORKSTACKSIZE,
                                     (main_t)work_lpthread, (FAR char * const *)NULL);
  DEBUGASSERT(g_work[LPWORK].pid > 0);

#endif /* CONFIG_SCHED_LPWORK */
#endif /* CONFIG_SCHED_HPWORK */

#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_SCHED_USRWORK)
  /* Start the user-space work queue */

  DEBUGASSERT(USERSPACE->work_usrstart != NULL);
  taskid = USERSPACE->work_usrstart();
  DEBUGASSERT(taskid > 0);
#endif

#endif /* CONFIG_SCHED_WORKQUEUE */

  /* Once the operating system has been initialized, the system must be
   * started by spawning the user init thread of execution.  This is the
   * first user-mode thead.
   */

  svdbg("Starting init thread\n");

  /* Perform any last-minute, board-specific initialization, if so
   * configured.
   */

#ifdef CONFIG_BOARD_INITIALIZE
  board_initialize();
#endif

  /* Start the default application.  In a flat build, this is entrypoint
   * is given by the definitions, CONFIG_USER_ENTRYPOINT.  In the kernel
   * build, however, we must get the address of the entrypoint from the
   * header at the beginning of the user-space blob.
   */

#ifdef CONFIG_NUTTX_KERNEL
  DEBUGASSERT(USERSPACE->us_entrypoint != NULL);
  taskid = TASK_CREATE("init", SCHED_PRIORITY_DEFAULT,
                       CONFIG_USERMAIN_STACKSIZE, USERSPACE->us_entrypoint,
                       (FAR char * const *)NULL);
#else
  taskid = TASK_CREATE("init", SCHED_PRIORITY_DEFAULT,
                       CONFIG_USERMAIN_STACKSIZE,
                       (main_t)CONFIG_USER_ENTRYPOINT,
                       (FAR char * const *)NULL);
#endif
  ASSERT(taskid > 0);

  /* We an save a few bytes by discarding the IDLE thread's environment. */

#if !defined(CONFIG_DISABLE_ENVIRON) && defined(CONFIG_PATH_INITIAL)
  (void)clearenv();
#endif

  return OK;
}
