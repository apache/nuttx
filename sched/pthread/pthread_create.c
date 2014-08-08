/****************************************************************************
 * sched/pthread/pthread_create.c
 *
 *   Copyright (C) 2007-2009, 2011, 2013 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <debug.h>
#include <errno.h>
#include <queue.h>

#include <nuttx/kmalloc.h>
#include <nuttx/pthread.h>
#include <nuttx/arch.h>

#include "os_internal.h"
#include "group/group.h"
#include "clock_internal.h"
#include "pthread/pthread.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/* Default pthread attributes */

pthread_attr_t g_default_pthread_attr = PTHREAD_ATTR_INITIALIZER;

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/* This is the name for name-less pthreads */

static const char g_pthreadname[] = "<pthread>";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_argsetup
 *
 * Description:
 *   This functions sets up parameters in the Task Control Block (TCB) in
 *   preparation for starting a new thread.
 *
 *   pthread_argsetup() is called from task_init() and task_start() to create
 *   a new task (with arguments cloned via strdup) or pthread_create() which
 *   has one argument passed by value (distinguished by the pthread boolean
 *   argument).
 *
 * Input Parameters:
 *   tcb        - Address of the new task's TCB
 *   arg        - The argument to provide to the pthread on startup.
 *
 * Return Value:
 *  None
 *
 ****************************************************************************/

static inline void pthread_argsetup(FAR struct pthread_tcb_s *tcb, pthread_addr_t arg)
{
#if CONFIG_TASK_NAME_SIZE > 0
  /* Copy the pthread name into the TCB */

  strncpy(tcb->cmn.name, g_pthreadname, CONFIG_TASK_NAME_SIZE);
#endif /* CONFIG_TASK_NAME_SIZE */

  /* For pthreads, args are strictly pass-by-value; that actual
   * type wrapped by pthread_addr_t is unknown.
   */

  tcb->arg = arg;
}

/****************************************************************************
 * Name: pthread_addjoininfo
 *
 * Description:
 *   Add a join structure to the local data set.
 *
 * Parameters:
 *   pjoin
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   The caller has provided protection from re-entrancy.
 *
 ****************************************************************************/

static inline void pthread_addjoininfo(FAR struct task_group_s *group,
                                       FAR struct join_s *pjoin)
{
  pjoin->next = NULL;
  if (!group->tg_jointail)
    {
      group->tg_joinhead = pjoin;
    }
  else
    {
      group->tg_jointail->next = pjoin;
    }

  group->tg_jointail = pjoin;
}

/****************************************************************************
 * Name:  pthread_start
 *
 * Description:
 *   This function is the low level entry point into the pthread
 *
 * Parameters:
 * None
 *
 ****************************************************************************/

static void pthread_start(void)
{
  FAR struct pthread_tcb_s *ptcb = (FAR struct pthread_tcb_s*)g_readytorun.head;
  FAR struct task_group_s *group = ptcb->cmn.group;
  FAR struct join_s *pjoin = (FAR struct join_s*)ptcb->joininfo;
  pthread_addr_t exit_status;

  DEBUGASSERT(group && pjoin);

  /* Sucessfully spawned, add the pjoin to our data set. */

  (void)pthread_takesemaphore(&group->tg_joinsem);
  pthread_addjoininfo(group, pjoin);
  (void)pthread_givesemaphore(&group->tg_joinsem);

  /* Report to the spawner that we successfully started. */

  pjoin->started = true;
  (void)pthread_givesemaphore(&pjoin->data_sem);

  /* Pass control to the thread entry point. In the kernel build this has to
   * be handled differently if we are starting a user-space pthread; we have
   * to switch to user-mode before calling into the pthread.
   */

#ifdef CONFIG_NUTTX_KERNEL
  up_pthread_start(ptcb->cmn.entry.pthread, ptcb->arg);
  exit_status = NULL;
#else
  exit_status = (*ptcb->cmn.entry.pthread)(ptcb->arg);
#endif

  /* The thread has returned (should never happen in the kernel mode case) */

  pthread_exit(exit_status);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  pthread_create
 *
 * Description:
 *   This function creates and activates a new thread with a specified
 *   attributes.
 *
 * Input Parameters:
 *    thread
 *    attr
 *    start_routine
 *    arg
 *
 * Returned value:
 *   OK (0) on success; a (non-negated) errno value on failure. The errno
 *   variable is not set.
 *
 ****************************************************************************/

int pthread_create(FAR pthread_t *thread, FAR pthread_attr_t *attr,
                   pthread_startroutine_t start_routine, pthread_addr_t arg)
{
  FAR struct pthread_tcb_s *ptcb;
  FAR struct join_s *pjoin;
  int priority;
#if CONFIG_RR_INTERVAL > 0
  int policy;
#endif
  int errcode;
  pid_t pid;
  int ret;

  /* If attributes were not supplied, use the default attributes */

  if (!attr)
    {
      attr = &g_default_pthread_attr;
    }

  /* Allocate a TCB for the new task. */

  ptcb = (FAR struct pthread_tcb_s *)kzalloc(sizeof(struct pthread_tcb_s));
  if (!ptcb)
    {
      sdbg("ERROR: Failed to allocate TCB\n");
      return ENOMEM;
    }

  /* Bind the parent's group to the new TCB (we have not yet joined the
   * group).
   */

#ifdef HAVE_TASK_GROUP
  ret = group_bind(ptcb);
  if (ret < 0)
    {
      errcode = ENOMEM;
      goto errout_with_tcb;
    }
#endif

  /* Share the address environment of the parent task.  NOTE:  Only tasks
   * created throught the nuttx/binfmt loaders may have an address
   * environment.
   */

#ifdef CONFIG_ADDRENV
  ret = up_addrenv_share((FAR const struct tcb_s *)g_readytorun.head,
                         (FAR struct tcb_s *)ptcb);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_tcb;
    }
#endif

  /* Allocate a detachable structure to support pthread_join logic */

  pjoin = (FAR struct join_s*)kzalloc(sizeof(struct join_s));
  if (!pjoin)
    {
      sdbg("ERROR: Failed to allocate join\n");
      errcode = ENOMEM;
      goto errout_with_tcb;
    }

  /* Allocate the stack for the TCB */

  ret = up_create_stack((FAR struct tcb_s *)ptcb, attr->stacksize,
                        TCB_FLAG_TTYPE_PTHREAD);
  if (ret != OK)
    {
      errcode = ENOMEM;
      goto errout_with_join;
    }

  /* Should we use the priority and scheduler specified in the
   * pthread attributes?  Or should we use the current thread's
   * priority and scheduler?
   */

  if (attr->inheritsched == PTHREAD_INHERIT_SCHED)
    {
      struct sched_param param;

      /* Get the priority for this thread. */

      ret = sched_getparam(0, &param);
      if (ret == OK)
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

  ret = pthread_schedsetup(ptcb, priority, pthread_start, start_routine);
  if (ret != OK)
    {
      errcode = EBUSY;
      goto errout_with_join;
    }

  /* Configure the TCB for a pthread receiving on parameter
   * passed by value
   */

  pthread_argsetup(ptcb, arg);

  /* Join the parent's task group */

#ifdef HAVE_TASK_GROUP
  ret = group_join(ptcb);
  if (ret < 0)
    {
      errcode = ENOMEM;
      goto errout_with_join;
    }
#endif

  /* Attach the join info to the TCB. */

  ptcb->joininfo = (FAR void *)pjoin;

  /* If round robin scheduling is selected, set the appropriate flag
   * in the TCB.
   */

#if CONFIG_RR_INTERVAL > 0
  if (policy == SCHED_RR)
    {
      ptcb->cmn.flags    |= TCB_FLAG_ROUND_ROBIN;
      ptcb->cmn.timeslice = MSEC2TICK(CONFIG_RR_INTERVAL);
    }
#endif

  /* Get the assigned pid before we start the task (who knows what
   * could happen to ptcb after this!).  Copy this ID into the join structure
   * as well.
   */

  pid = (int)ptcb->cmn.pid;
  pjoin->thread = (pthread_t)pid;

  /* Initialize the semaphores in the join structure to zero. */

  ret = sem_init(&pjoin->data_sem, 0, 0);
  if (ret == OK)
    {
      ret = sem_init(&pjoin->exit_sem, 0, 0);
    }

  /* Activate the task */

  sched_lock();
  if (ret == OK)
    {
      ret = task_activate((FAR struct tcb_s *)ptcb);
    }

  if (ret == OK)
    {
      /* Wait for the task to actually get running and to register
       * its join structure.
       */

      (void)pthread_takesemaphore(&pjoin->data_sem);

      /* Return the thread information to the caller */

      if (thread)
       {
         *thread = (pthread_t)pid;
       }

      if (!pjoin->started)
        {
          ret = EINVAL;
        }

      sched_unlock();
      (void)sem_destroy(&pjoin->data_sem);
    }
  else
    {
      sched_unlock();
      dq_rem((FAR dq_entry_t*)ptcb, (dq_queue_t*)&g_inactivetasks);
      (void)sem_destroy(&pjoin->data_sem);
      (void)sem_destroy(&pjoin->exit_sem);

      errcode = EIO;
      goto errout_with_join;
    }

  return ret;

errout_with_join:
  sched_kfree(pjoin);
  ptcb->joininfo = NULL;

errout_with_tcb:
  sched_releasetcb((FAR struct tcb_s *)ptcb, TCB_FLAG_TTYPE_PTHREAD);
  return errcode;
}
