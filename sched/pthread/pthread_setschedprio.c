/****************************************************************************
 * sched/pthread/pthread_setschedprio.c
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
#include <sched.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  pthread_setsetprio
 *
 * Description:
 *   The pthread_setschedprio() function sets the scheduling priority for
 *   the thread whose thread ID is given by 'thread' to the value given by
 *   'prio'.  If the  thread_setschedprio() function fails, the scheduling
 *   priority of the target thread will not be changed.
 *
 * Input Parameters:
 *   thread - the thread ID of the task to reprioritize.
 *   prio - The new thread priority.  The range of valid priority numbers is
 *     from SCHED_PRIORITY_MIN through SCHED_PRIORITY_MAX.
 *
 * Returned Value:
 *    OK if successful, otherwise an error number.  This function can
 *    fail for the following reasons:
 *
 *    EINVAL - prio is out of range.
 *    ESRCH  - thread ID does not correspond to any thread.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_setschedprio(pthread_t thread, int prio)
{
  struct sched_param param;
  int ret;

#ifdef CONFIG_SCHED_SPORADIC
  /* Get the current sporadic scheduling parameters.  Those will not be
   * modified.
   */

  ret = nxsched_get_param((pid_t)thread, &param);
  if (ret < 0)
    {
      return -ret;
    }
#endif

  /* Call nxsched_set_param() to change the priority */

  param.sched_priority = prio;
  ret = nxsched_set_param((pid_t)thread, &param);
  if (ret < 0)
    {
      return -ret;
    }

  return OK;
}
