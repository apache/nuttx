/****************************************************************************
 * libs/libc/pthread/pthread_mutexattr_getrobust.c
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
#include <pthread.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutexattr_getrobust
 *
 * Description:
 *   Return the mutex robustneess from the mutex attributes.
 *
 * Input Parameters:
 *   attr   - The mutex attributes to query
 *   robust - Location to return the robustness indication
 *
 * Returned Value:
 *   0, if the robustness was successfully return in 'robust', or
 *   EINVAL, if any NULL pointers provided.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_mutexattr_getrobust(FAR const pthread_mutexattr_t *attr,
                                FAR int *robust)
{
  if (attr != NULL && robust != NULL)
    {
#if defined(CONFIG_PTHREAD_MUTEX_UNSAFE)
      *robust = PTHREAD_MUTEX_STALLED;
#elif defined(CONFIG_PTHREAD_MUTEX_BOTH)
      *robust = attr->robust;
#else /* Default: CONFIG_PTHREAD_MUTEX_ROBUST */
      *robust = PTHREAD_MUTEX_ROBUST;
#endif
      return 0;
    }

  return EINVAL;
}
