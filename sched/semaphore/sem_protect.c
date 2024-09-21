/****************************************************************************
 * sched/semaphore/sem_protect.c
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

#include <nuttx/config.h>

#include <stdbool.h>
#include <errno.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"

#ifdef CONFIG_PRIORITY_PROTECT

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_protect_wait
 *
 * Description:
 *   This function attempts to lock the protected semaphore, set the
 *   holder tcb priority to ceiling priority.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sem_wait except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   sem - Semaphore descriptor.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *   - EINVAL:  Invalid attempt to get the semaphore
 *
 ****************************************************************************/

int nxsem_protect_wait(FAR sem_t *sem)
{
  if ((sem->flags & SEM_PRIO_MASK) == SEM_PRIO_PROTECT)
    {
      FAR struct tcb_s *rtcb = this_task();

      if (rtcb->sched_priority > sem->ceiling)
        {
          return -EINVAL;
        }

      sem->saved = rtcb->sched_priority;
      rtcb->sched_priority = sem->ceiling;
    }

  return OK;
}

/****************************************************************************
 * Name: nxsem_protect_post
 *
 * Description:
 *   This function attempts to unlock the protected semaphore, restore the
 *   holder tcb priority.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sem_wait except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   sem - Semaphore descriptor.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 ****************************************************************************/

void nxsem_protect_post(FAR sem_t *sem)
{
  if (sem->saved > 0)
    {
      nxsched_set_priority(this_task(), sem->saved);
      sem->saved = 0;
    }
}

#endif /* CONFIG_PRIORITY_PROTECT */
