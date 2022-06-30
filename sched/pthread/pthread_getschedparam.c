/****************************************************************************
 * sched/pthread/pthread_getschedparam.c
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
#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sched.h>

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_getschedparam
 *
 * Description:
 *   The pthread_getschedparam() functions will get the scheduling policy and
 *   parameters of threads. For SCHED_FIFO and SCHED_RR, the only required
 *   member of the sched_param structure is the priority sched_priority.
 *
 *   The pthread_getschedparam() function will retrieve the scheduling
 *   policy and scheduling parameters for the thread whose thread ID is
 *   given by 'thread' and will store those values in 'policy' and 'param',
 *   respectively. The priority value returned from pthread_getschedparam()
 *   will be the value specified by the most recent pthread_setschedparam(),
 *   pthread_setschedprio(), or pthread_create() call affecting the target
 *   thread. It will not reflect any temporary adjustments to its priority
 *   (such as might result of any priority inheritance, for example).
 *
 *   The policy parameter may have the value SCHED_FIFO, or SCHED_RR
 *   (SCHED_OTHER and SCHED_SPORADIC, in particular, are not supported).
 *   The SCHED_FIFO and SCHED_RR policies will have a single scheduling
 *   parameter, sched_priority.
 *
 * Input Parameters:
 *   thread - The ID of thread whose scheduling parameters will be queried.
 *   policy - The location to store the thread's scheduling policy.
 *   param  - The location to store the thread's priority.
 *
 * Returned Value:
 *   0 if successful.  Otherwise, the error code ESRCH if the value specified
 *   by thread does not refer to an existing thread.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_getschedparam(pthread_t thread, FAR int *policy,
                          FAR struct sched_param *param)
{
  int ret;

  sinfo("Thread ID=%d policy=%p param=%p\n", thread, policy, param);

  if (policy == NULL || param == NULL)
    {
      ret = EINVAL;
    }
  else
    {
      /* Get the scheduler parameters of the thread. */

      ret = nxsched_get_param((pid_t)thread, param);
      if (ret < 0)
        {
          ret = -ret;
        }
      else
        {
          /* Get the scheduler policy. */

          ret = nxsched_get_scheduler((pid_t)thread);
          if (ret < 0)
            {
              ret = -ret;
            }
          else
            {
              *policy = ret;
              ret = OK;
            }
        }
    }

  sinfo("Returning %d\n", ret);
  return ret;
}
