/************************************************************
 * pthread_create.c
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <pthread.h>
#include <sched.h>
#include <debug.h>
#include <errno.h>
#include <queue.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include "os_internal.h"
#include "pthread_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/* Default pthread attributes */

FAR pthread_attr_t g_default_pthread_attr =
{
  PTHREAD_STACK_DEFAULT,    /* stacksize */
  PTHREAD_DEFAULT_PRIORITY, /* priority */
  SCHED_RR,                 /* policy */
  PTHREAD_EXPLICIT_SCHED,   /* inheritsched */
};

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Function:  pthread_addjoininfo
 *
 * Description:
 *   Add a join_t to the local data set.
 *
 * Parameters:
 *   pjoin
 *
 * Return Value:
 *   None or pointer to the found entry.
 *
 * Assumptions:
 *   The caller has provided protection from re-entrancy.
 *
 ************************************************************/

static void pthread_addjoininfo(FAR join_t *pjoin)
{
  pjoin->next = NULL;
  if (!g_pthread_tail)
    {
      g_pthread_head = pjoin;
    }
  else
    {
      g_pthread_tail->next = pjoin;
    }
  g_pthread_tail = pjoin;
}

/************************************************************
 * Name:  pthread_start
 *
 * Description:
 *   This function is the low level entry point into the
 *   pthread
 *
 * Parameters:
 * None
 *
 ************************************************************/

static void pthread_start(void)
{
  FAR _TCB   *ptcb  = (FAR _TCB*)g_readytorun.head;
  FAR join_t *pjoin = (FAR join_t*)ptcb->joininfo;
  pthread_addr_t exit_status;

  /* Sucessfully spawned, add the pjoin to our data set.
   * Don't re-enable pre-emption until this is done.
   */

  (void)pthread_takesemaphore(&g_join_semaphore);
  pthread_addjoininfo(pjoin);
  (void)pthread_givesemaphore(&g_join_semaphore);

  /* Report to the spawner that we successfully started. */

  pjoin->started = TRUE;
  (void)pthread_givesemaphore(&pjoin->data_sem);

  /* Pass control to the thread entry point.  The argument is
   * argv[1].  argv[0] (the thread name) and argv[2-4] are not made
   * available to the pthread.
   */

  exit_status = (*ptcb->entry.pthread)((pthread_addr_t)ptcb->argv[1]);

  /* The thread has returned */

  pthread_exit(exit_status);
}

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Name:  pthread_create
 *
 * Description:
 *   This function creates and activates a new thread with a
 *   specified attributes.
 *
 * Input Parameters:
 *    thread
 *    attr
 *    start_routine
 *    arg
 ************************************************************/

int pthread_create(pthread_t *thread, pthread_attr_t *attr,
                   pthread_startroutine_t start_routine,
                   pthread_addr_t arg)
{
  FAR _TCB *ptcb;
  FAR join_t *pjoin;
  STATUS status;
  char *argv[2];
  int priority;
#if CONFIG_RR_INTERVAL > 0
  int policy;
#endif
  pid_t pid;

  /* If attributes were not supplied, use the default attributes */

  if (!attr)
    {
      attr = &g_default_pthread_attr;
    }

  /* Allocate a TCB for the new task. */

  ptcb = (FAR _TCB*)kzmalloc(sizeof(_TCB));
  if (!ptcb)
    {
      *get_errno_ptr() = ENOMEM;
      return ERROR;
    }

  /* Associate file descriptors with the new task */

  if (sched_setuppthreadfiles(ptcb) != OK)
    {
      sched_releasetcb(ptcb);
      return ERROR;
    }

  /* Allocate a detachable structure to support pthread_join logic */

  pjoin = (FAR join_t*)kzmalloc(sizeof(join_t));
  if (!pjoin)
    {
      sched_releasetcb(ptcb);
      return ERROR;
    }

  /* Allocate the stack for the TCB */

  status = up_create_stack(ptcb, attr->stacksize);
  if (status != OK)
    {
      sched_releasetcb(ptcb);
      sched_free(pjoin);
      return ERROR;
    }

  /* Should we use the priority and scheduler specified in the
   * pthread attributes?  Or should we use the current thread's
   * priority and scheduler?
   */

  if (attr->inheritsched == PTHREAD_INHERIT_SCHED)
    {
      /* Get the priority of this thread. */

      struct sched_param param;
      status = sched_getparam(0, &param);
      if (status == OK)
        {
          priority = param.sched_priority;
        }
      else
        {
          priority = SCHED_FIFO;
        }

      /* Get the scheduler policy for this thread */

#if CONFIG_RR_INTERVAL > 0
      policy = sched_getscheduler(0);
      if (policy == ERROR)
        {
          policy = SCHED_FIFO;
        }
#endif
    }
  else
    {
      /* Use the priority and scheduler from the attributes */

      priority = attr->priority;
#if CONFIG_RR_INTERVAL > 0
      policy   = attr->policy;
#endif
    }

  /* Initialize the task control block */

  status  = task_schedsetup(ptcb, priority, pthread_start,
                            (main_t)start_routine);
  if (status != OK)
    {

      sched_releasetcb(ptcb);
      sched_free(pjoin);
      return ERROR;
    }

  /* Configure the TCB for a pthread receiving on parameter
   * passed by value
   */

  argv[0] = (char *)arg;
  argv[1] = NULL;
  (void)task_argsetup(ptcb, NULL, TRUE, argv);

  /* Attach the join info to the TCB. */

  ptcb->joininfo = (void*)pjoin;

  /* If round robin scheduling is selected, set the appropriate flag
   * in the TCB.
   */

#if CONFIG_RR_INTERVAL > 0
  if (policy == SCHED_RR)
    {
      ptcb->flags    |= TCB_FLAG_ROUND_ROBIN;
      ptcb->timeslice = CONFIG_RR_INTERVAL;
    }
#endif

  /* Get the assigned pid before we start the task (who knows what
   * could happen to ptcb after this!).  Copy this ID into the join structure
   * as well.
   */

  pid = (int)ptcb->pid;
  pjoin->thread = (pthread_t)pid;

  /* Initialize the semaphores in the join structure to zero. */

  status = sem_init(&pjoin->data_sem, 0, 0);
  if (status == OK) status = sem_init(&pjoin->exit_sem, 0, 0);

  /* Activate the task */

  sched_lock();
  if (status == OK)
    {
      status = task_activate(ptcb);
    }

  if (status == OK)
    {
      /* Wait for the task to actually get running and to register
       * its join_t
       */

      (void)pthread_takesemaphore(&pjoin->data_sem);

      /* Return the thread information to the caller */

      if (thread) *thread = (pthread_t)pid;
      if (!pjoin->started) status = ERROR;

      sched_unlock();
      (void)sem_destroy(&pjoin->data_sem);
    }
  else
    {
      sched_unlock();
      dq_rem((FAR dq_entry_t*)ptcb, &g_inactivetasks);
      (void)sem_destroy(&pjoin->data_sem);
      (void)sem_destroy(&pjoin->exit_sem);
      sched_releasetcb(ptcb);
      sched_free(pjoin);
      return ERROR;
    }
  return OK;
}
