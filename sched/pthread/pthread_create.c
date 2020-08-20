/****************************************************************************
 * sched/pthread/pthread_create.c
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
#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>
#include <queue.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pthread.h>

#include "sched/sched.h"
#include "group/group.h"
#include "clock/clock.h"
#include "pthread/pthread.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Default pthread attributes (see include/nuttx/pthread.h).  When configured
 * to build separate kernel- and user-address spaces, this global is
 * duplicated in each address spaced.  This copy can only be shared within
 * the kernel address space.
 */

const pthread_attr_t g_default_pthread_attr = PTHREAD_ATTR_INITIALIZER;

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_TASK_NAME_SIZE > 0
/* This is the name for name-less pthreads */

static const char g_pthreadname[] = "<pthread>";
#endif

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
 *   pthread_argsetup() is called from nxtask_init() and nxtask_start() to
 *   create a new task (with arguments cloned via strdup) or pthread_create()
 *   which has one argument passed by value (distinguished by the pthread
 *   boolean argument).
 *
 * Input Parameters:
 *   tcb        - Address of the new task's TCB
 *   arg        - The argument to provide to the pthread on startup.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static inline void pthread_argsetup(FAR struct pthread_tcb_s *tcb,
                                    pthread_addr_t arg)
{
#if CONFIG_TASK_NAME_SIZE > 0
  /* Copy the pthread name into the TCB */

  strncpy(tcb->cmn.name, g_pthreadname, CONFIG_TASK_NAME_SIZE);
  tcb->cmn.name[CONFIG_TASK_NAME_SIZE] = '\0';
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
 * Input Parameters:
 *   pjoin
 *
 * Returned Value:
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
 * Input Parameters:
 * None
 *
 ****************************************************************************/

static void pthread_start(void)
{
  FAR struct pthread_tcb_s *ptcb = (FAR struct pthread_tcb_s *)this_task();
  FAR struct task_group_s *group = ptcb->cmn.group;
  FAR struct join_s *pjoin = (FAR struct join_s *)ptcb->joininfo;
  pthread_addr_t exit_status;

  DEBUGASSERT(group && pjoin);

  /* Successfully spawned, add the pjoin to our data set. */

  pthread_sem_take(&group->tg_joinsem, NULL, false);
  pthread_addjoininfo(group, pjoin);
  pthread_sem_give(&group->tg_joinsem);

  /* Report to the spawner that we successfully started. */

  pjoin->started = true;
  pthread_sem_give(&pjoin->data_sem);

  /* The priority of this thread may have been boosted to avoid priority
   * inversion problems.  If that is the case, then drop to the correct
   * execution priority.
   */

  if (ptcb->cmn.sched_priority > ptcb->cmn.init_priority)
    {
      DEBUGVERIFY(nxsched_set_priority(&ptcb->cmn, ptcb->cmn.init_priority));
    }

  /* Pass control to the thread entry point. In the kernel build this has to
   * be handled differently if we are starting a user-space pthread; we have
   * to switch to user-mode before calling into the pthread.
   */

#ifdef CONFIG_BUILD_FLAT
  exit_status = (*ptcb->cmn.entry.pthread)(ptcb->arg);
#else
  up_pthread_start(ptcb->cmn.entry.pthread, ptcb->arg);
  exit_status = NULL;
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
 * Returned Value:
 *   OK (0) on success; a (non-negated) errno value on failure. The errno
 *   variable is not set.
 *
 ****************************************************************************/

int pthread_create(FAR pthread_t *thread, FAR const pthread_attr_t *attr,
                   pthread_startroutine_t start_routine, pthread_addr_t arg)
{
  FAR struct pthread_tcb_s *ptcb;
  FAR struct join_s *pjoin;
  struct sched_param param;
  int policy;
  int errcode;
  pid_t pid;
  int ret;
  bool group_joined = false;

  /* If attributes were not supplied, use the default attributes */

  if (!attr)
    {
      attr = &g_default_pthread_attr;
    }

  /* Allocate a TCB for the new task. */

  ptcb = (FAR struct pthread_tcb_s *)
            kmm_zalloc(sizeof(struct pthread_tcb_s));
  if (!ptcb)
    {
      serr("ERROR: Failed to allocate TCB\n");
      return ENOMEM;
    }

  /* Bind the parent's group to the new TCB (we have not yet joined the
   * group).
   */

  ret = group_bind(ptcb);
  if (ret < 0)
    {
      errcode = ENOMEM;
      goto errout_with_tcb;
    }

#ifdef CONFIG_ARCH_ADDRENV
  /* Share the address environment of the parent task group. */

  ret = up_addrenv_attach(ptcb->cmn.group, this_task());
  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_tcb;
    }
#endif

  /* Allocate a detachable structure to support pthread_join logic */

  pjoin = (FAR struct join_s *)kmm_zalloc(sizeof(struct join_s));
  if (!pjoin)
    {
      serr("ERROR: Failed to allocate join\n");
      errcode = ENOMEM;
      goto errout_with_tcb;
    }

  if (attr->detachstate == PTHREAD_CREATE_DETACHED)
    {
      pjoin->detached = true;
    }

  if (attr->stackaddr)
    {
      /* Use pre-allocated stack */

      ret = up_use_stack((FAR struct tcb_s *)ptcb, attr->stackaddr,
                         attr->stacksize);
    }
  else
    {
      /* Allocate the stack for the TCB */

      ret = up_create_stack((FAR struct tcb_s *)ptcb, attr->stacksize,
                            TCB_FLAG_TTYPE_PTHREAD);
    }

  if (ret != OK)
    {
      errcode = ENOMEM;
      goto errout_with_join;
    }

  /* Should we use the priority and scheduler specified in the pthread
   * attributes?  Or should we use the current thread's priority and
   * scheduler?
   */

  if (attr->inheritsched == PTHREAD_INHERIT_SCHED)
    {
      /* Get the priority (and any other scheduling parameters) for this
       * thread.
       */

      ret = nxsched_get_param(0, &param);
      if (ret < 0)
        {
          errcode = -ret;
          goto errout_with_join;
        }

      /* Get the scheduler policy for this thread */

      policy = nxsched_get_scheduler(0);
      if (policy < 0)
        {
          errcode = -policy;
          goto errout_with_join;
        }
    }
  else
    {
      /* Use the scheduler policy and policy the attributes */

      policy                             = attr->policy;
      param.sched_priority               = attr->priority;

#ifdef CONFIG_SCHED_SPORADIC
      param.sched_ss_low_priority        = attr->low_priority;
      param.sched_ss_max_repl            = attr->max_repl;
      param.sched_ss_repl_period.tv_sec  = attr->repl_period.tv_sec;
      param.sched_ss_repl_period.tv_nsec = attr->repl_period.tv_nsec;
      param.sched_ss_init_budget.tv_sec  = attr->budget.tv_sec;
      param.sched_ss_init_budget.tv_nsec = attr->budget.tv_nsec;
#endif
    }

#ifdef CONFIG_SCHED_SPORADIC
  if (policy == SCHED_SPORADIC)
    {
      FAR struct sporadic_s *sporadic;
      sclock_t repl_ticks;
      sclock_t budget_ticks;

      /* Convert timespec values to system clock ticks */

      clock_time2ticks(&param.sched_ss_repl_period, &repl_ticks);
      clock_time2ticks(&param.sched_ss_init_budget, &budget_ticks);

      /* The replenishment period must be greater than or equal to the
       * budget period.
       */

      if (repl_ticks < budget_ticks)
        {
          errcode = EINVAL;
          goto errout_with_join;
        }

      /* Initialize the sporadic policy */

      ret = nxsched_initialize_sporadic(&ptcb->cmn);
      if (ret >= 0)
        {
          sporadic               = ptcb->cmn.sporadic;
          DEBUGASSERT(sporadic != NULL);

          /* Save the sporadic scheduling parameters */

          sporadic->hi_priority  = param.sched_priority;
          sporadic->low_priority = param.sched_ss_low_priority;
          sporadic->max_repl     = param.sched_ss_max_repl;
          sporadic->repl_period  = repl_ticks;
          sporadic->budget       = budget_ticks;

          /* And start the first replenishment interval */

          ret = nxsched_start_sporadic(&ptcb->cmn);
        }

      /* Handle any failures */

      if (ret < 0)
        {
          errcode = -ret;
          goto errout_with_join;
        }
    }
#endif

  /* Initialize the task control block */

  ret = pthread_setup_scheduler(ptcb, param.sched_priority, pthread_start,
                                start_routine);
  if (ret != OK)
    {
      errcode = EBUSY;
      goto errout_with_join;
    }

#ifdef CONFIG_SMP
  /* pthread_setup_scheduler() will set the affinity mask by inheriting the
   * setting from the parent task.  We need to override this setting
   * with the value from the pthread attributes unless that value is
   * zero:  Zero is the default value and simply means to inherit the
   * parent thread's affinity mask.
   */

  if (attr->affinity != 0)
    {
      ptcb->cmn.affinity = attr->affinity;
    }
#endif

  /* Configure the TCB for a pthread receiving on parameter
   * passed by value
   */

  pthread_argsetup(ptcb, arg);

  /* Join the parent's task group */

  ret = group_join(ptcb);
  if (ret < 0)
    {
      errcode = ENOMEM;
      goto errout_with_join;
    }

  group_joined = true;

  /* Attach the join info to the TCB. */

  ptcb->joininfo = (FAR void *)pjoin;

  /* Set the appropriate scheduling policy in the TCB */

  ptcb->cmn.flags &= ~TCB_FLAG_POLICY_MASK;
  switch (policy)
    {
      default:
        DEBUGPANIC();
      case SCHED_FIFO:
        ptcb->cmn.flags    |= TCB_FLAG_SCHED_FIFO;
        break;

#if CONFIG_RR_INTERVAL > 0
      case SCHED_RR:
        ptcb->cmn.flags    |= TCB_FLAG_SCHED_RR;
        ptcb->cmn.timeslice = MSEC2TICK(CONFIG_RR_INTERVAL);
        break;
#endif

#ifdef CONFIG_SCHED_SPORADIC
      case SCHED_SPORADIC:
        ptcb->cmn.flags    |= TCB_FLAG_SCHED_SPORADIC;
        break;
#endif

#if 0 /* Not supported */
      case SCHED_OTHER:
        ptcb->cmn.flags    |= TCB_FLAG_SCHED_OTHER;
        break;
#endif
    }

#ifdef CONFIG_CANCELLATION_POINTS
  /* Set the deferred cancellation type */

  ptcb->cmn.flags |= TCB_FLAG_CANCEL_DEFERRED;
#endif

  /* Get the assigned pid before we start the task (who knows what
   * could happen to ptcb after this!).  Copy this ID into the join structure
   * as well.
   */

  pid = (int)ptcb->cmn.pid;
  pjoin->thread = (pthread_t)pid;

  /* Initialize the semaphores in the join structure to zero. */

  ret = nxsem_init(&pjoin->data_sem, 0, 0);
  if (ret == OK)
    {
      ret = nxsem_init(&pjoin->exit_sem, 0, 0);
    }

  if (ret < 0)
    {
      ret = -ret;
    }

  /* Thse semaphores are used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  if (ret == OK)
    {
      ret = nxsem_set_protocol(&pjoin->data_sem, SEM_PRIO_NONE);

      if (ret == OK)
        {
          ret = nxsem_set_protocol(&pjoin->exit_sem, SEM_PRIO_NONE);
        }

      if (ret < 0)
        {
          ret = -ret;
        }
    }

  /* If the priority of the new pthread is lower than the priority of the
   * parent thread, then starting the pthread could result in both the
   * parent and the pthread to be blocked.  This is a recipe for priority
   * inversion issues.
   *
   * We avoid this here by boosting the priority of the (inactive) pthread
   * so it has the same priority as the parent thread.
   */

  if (ret == OK)
    {
      FAR struct tcb_s *parent = this_task();
      DEBUGASSERT(parent != NULL);

      if (ptcb->cmn.sched_priority < parent->sched_priority)
        {
          ret = nxsched_set_priority(&ptcb->cmn, parent->sched_priority);
          if (ret < 0)
            {
              ret = -ret;
            }
        }
    }

  /* Then activate the task */

  sched_lock();
  if (ret == OK)
    {
      nxtask_activate((FAR struct tcb_s *)ptcb);

      /* Wait for the task to actually get running and to register
       * its join structure.
       */

      pthread_sem_take(&pjoin->data_sem, NULL, false);

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
      nxsem_destroy(&pjoin->data_sem);
    }
  else
    {
      sched_unlock();
      dq_rem((FAR dq_entry_t *)ptcb, (FAR dq_queue_t *)&g_inactivetasks);
      nxsem_destroy(&pjoin->data_sem);
      nxsem_destroy(&pjoin->exit_sem);

      errcode = EIO;
      goto errout_with_join;
    }

  return ret;

errout_with_join:
  kmm_free(pjoin);
  ptcb->joininfo = NULL;

errout_with_tcb:

  /* Clear group binding */

  if (ptcb && !group_joined)
    {
      ptcb->cmn.group = NULL;
    }

  nxsched_release_tcb((FAR struct tcb_s *)ptcb, TCB_FLAG_TTYPE_PTHREAD);
  return errcode;
}
