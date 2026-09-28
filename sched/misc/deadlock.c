/****************************************************************************
 * sched/misc/deadlock.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/mutex.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

struct deadlock_info_s
{
  FAR pid_t *holders;
  size_t arraylen;
  size_t holdercnt;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getmutex
 ****************************************************************************/

static FAR mutex_t *getmutex(FAR struct tcb_s *tcb)
{
  FAR sem_t *sem;

  if (tcb == NULL)
    {
      return NULL;
    }

  if (tcb->task_state == TSTATE_WAIT_SEM)
    {
      sem = tcb->waitobj;
      if (sem != NULL && (sem->flags & SEM_TYPE_MUTEX) != 0)
        {
          return (FAR mutex_t *)sem;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: deadlock_next
 ****************************************************************************/

static FAR struct tcb_s *deadlock_next(FAR struct tcb_s *tcb)
{
  FAR mutex_t *mutex;
  pid_t holder;

  mutex = getmutex(tcb);
  if (mutex == NULL)
    {
      return NULL;
    }

  holder = nxmutex_get_holder(mutex);
  if (holder < 0)
    {
      return NULL;
    }

  return nxsched_get_tcb(holder);
}

/****************************************************************************
 * Name: find_deadlock_cycle
 ****************************************************************************/

static FAR struct tcb_s *find_deadlock_cycle(FAR struct tcb_s *tcb)
{
  FAR struct tcb_s *slow = tcb;
  FAR struct tcb_s *fast = tcb;

  /* Each thread has at most one outgoing edge in the mutex wait-for graph.
   * Use Floyd's algorithm to determine whether the chain contains a cycle.
   */

  do
    {
      slow = deadlock_next(slow);
      fast = deadlock_next(fast);
      if (fast != NULL)
        {
          fast = deadlock_next(fast);
        }

      if (slow == NULL || fast == NULL)
        {
          return NULL;
        }
    }
  while (slow != fast);

  /* Then locate the first TCB in the cycle. */

  slow = tcb;
  while (slow != fast)
    {
      slow = deadlock_next(slow);
      fast = deadlock_next(fast);
    }

  return slow;
}

/****************************************************************************
 * Name: deadlock_contains
 ****************************************************************************/

static bool deadlock_contains(FAR const struct deadlock_info_s *info,
                              pid_t pid)
{
  size_t index;

  for (index = 0; index < info->holdercnt; index++)
    {
      if (info->holders[index] == pid)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: collect_deadlock
 ****************************************************************************/

static void collect_deadlock(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct deadlock_info_s *info = arg;
  FAR struct tcb_s *entry;
  FAR struct tcb_s *current;

  if (info->holdercnt >= info->arraylen ||
      deadlock_contains(info, tcb->pid))
    {
      return;
    }

  entry = find_deadlock_cycle(tcb);
  if (entry == NULL || deadlock_contains(info, entry->pid))
    {
      return;
    }

  /* Only copy TCBs which are members of the cycle.  Threads which merely
   * wait on a deadlocked thread are not themselves part of the deadlock.
   */

  current = entry;
  do
    {
      if (info->holdercnt >= info->arraylen)
        {
          return;
        }

      info->holders[info->holdercnt++] = current->pid;
      current = deadlock_next(current);
    }
  while (current != entry);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_collect_deadlock
 *
 * Description:
 *   Find mutex deadlocks and collect the IDs of participating threads.
 *
 * Input parameters:
 *   pid   - The array to store deadlocked thread IDs.
 *   count - The maximum number of thread IDs to store.
 *
 * Returned Value:
 *   The number of thread IDs stored in pid.  A return value equal to count
 *   may indicate that the result was truncated.
 *
 ****************************************************************************/

size_t nxsched_collect_deadlock(FAR pid_t *pid, size_t count)
{
  struct deadlock_info_s info;

  if (pid == NULL || count == 0)
    {
      return 0;
    }

  info.holders = pid;
  info.arraylen = count;
  info.holdercnt = 0;
  nxsched_foreach(collect_deadlock, &info);
  return info.holdercnt;
}
