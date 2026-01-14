/****************************************************************************
 * libs/libc/semaphore/sem_setprioceiling.c
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

#include <assert.h>
#include <errno.h>
#include <sched.h>

#include <nuttx/semaphore.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_setprioceiling
 *
 * Description:
 *   Set the priority ceiling of a semaphore.
 *
 * Input Parameters:
 *   mutex       - The mutex in which to set the mutex priority ceiling.
 *   prioceiling - The mutex priority ceiling value to set.
 *   old_ceiling - Location to return the mutex ceiling priority set before.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure
 *
 ****************************************************************************/

int nxsem_setprioceiling(FAR sem_t *sem, int prioceiling,
                         FAR int *old_ceiling)
{
  DEBUGASSERT(sem != NULL);

  if ((sem->flags & SEM_PRIO_MASK) == SEM_PRIO_PROTECT &&
      prioceiling >= sched_get_priority_min(SCHED_FIFO) &&
      prioceiling <= sched_get_priority_max(SCHED_FIFO))
    {
      if (old_ceiling != NULL)
        {
          *old_ceiling = sem->ceiling;
        }

      sem->ceiling = prioceiling;
      return OK;
    }

  return -EINVAL;
}
