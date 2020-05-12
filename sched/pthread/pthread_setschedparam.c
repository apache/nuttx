/****************************************************************************
 * sched/pthread/pthread_setschedparam.c
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
 * Name: pthread_setschedparam
 *
 * Description:
 *   The pthread_setschedparam() functions will set the scheduling policy
 *   and parameters of threads. For SCHED_FIFO and SCHED_RR, the only
 *   required member of the sched_param structure is the priority
 *   sched_priority.
 *
 *   The pthread_setschedparam() function will set the scheduling policy
 *   and associated scheduling parameters for the thread whose thread ID
 *   is given by 'thread' to the policy and associated parameters provided
 *   in 'policy' and 'param', respectively.
 *
 *   The policy parameter may have the value SCHED_FIFO, or SCHED_RR
 *   (SCHED_OTHER and SCHED_SPORADIC, in particular, are not supported).
 *   The SCHED_FIFO and SCHED_RR policies will have a single scheduling
 *   parameter, sched_priority.
 *
 *   If the pthread_setschedparam() function fails, the scheduling parameters
 *   will not be changed for the target thread.
 *
 * Input Parameters:
 *   thread - The ID of thread whose scheduling parameters will be modified.
 *   policy - The new scheduling policy of the thread.  Either SCHED_FIFO or
 *            SCHED_RR. SCHED_OTHER and SCHED_SPORADIC are not supported.
 *   param  - Provides the new priority of the thread.
 *
 * Returned Value:
 *   0 if successful.  Otherwise, an error code identifying the cause of the
 *   failure:
 *
 *   EINVAL  The value specified by 'policy' or one of the scheduling
 *           parameters associated with the scheduling policy 'policy' is
 *           invalid.
 *   ENOTSUP An attempt was made to set the policy or scheduling parameters
 *           to an unsupported value (SCHED_OTHER and SCHED_SPORADIC in
 *           particular are not supported)
 *   EPERM   The caller does not have the appropriate permission to set
 *           either the scheduling parameters or the scheduling policy of
 *           the specified thread. Or, the implementation does not allow
 *           the application to modify one of the parameters to the value
 *           specified.
 *   ESRCH   The value specified by thread does not refer to a existing
 *           thread.
 *
 ****************************************************************************/

int pthread_setschedparam(pthread_t thread, int policy,
                          FAR const struct sched_param *param)
{
  int ret;

  sinfo("thread ID=%d policy=%d param=0x%p\n", thread, policy, param);

  /* Let nxsched_set_scheduler do all of the work */

  ret = nxsched_set_scheduler((pid_t)thread, policy, param);
  if (ret < 0)
    {
      /* If nxsched_set_scheduler() fails, return the positive errno value */

      ret = -ret;
    }

  return ret;
}
