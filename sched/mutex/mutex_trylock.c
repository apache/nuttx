/****************************************************************************
 * sched/mutex/mutex_trylock.c
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

#include <nuttx/sched.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmutex_trylock
 *
 * Description:
 *   This function locks the mutex only if the mutex is currently not locked.
 *   If the mutex has been locked already, the call returns without blocking.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *     -EINVAL - Invalid attempt to lock the mutex
 *     -EAGAIN - The mutex is not available.
 *
 ****************************************************************************/

int nxmutex_trylock(FAR mutex_t *mutex)
{
  int ret;

  if (nxmutex_is_hold(mutex))
    {
      return -EDEADLK;
    }

  ret = nxsem_trywait(&mutex->sem);
  if (ret >= 0)
    {
      mutex->holder = nxsched_gettid();
    }

  return ret;
}
