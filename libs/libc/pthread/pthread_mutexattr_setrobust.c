/****************************************************************************
 * libs/libc/pthread/pthread_mutexattr_setrobust.c
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
 * Name: pthread_mutexattr_setrobust
 *
 * Description:
 *   Set the mutex robustness in the mutex attributes.
 *
 * Input Parameters:
 *   attr   - The mutex attributes in which to set the mutex type.
 *   robust - The mutex type value to set.
 *
 * Returned Value:
 *   0, if the mutex robustness was successfully set in 'attr', or
 *   EINVAL, if 'attr' is NULL or 'robust' unrecognized.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_mutexattr_setrobust(pthread_mutexattr_t *attr, int robust)
{
#if defined(CONFIG_PTHREAD_MUTEX_UNSAFE)

  if (attr != NULL && robust == PTHREAD_MUTEX_STALLED)
    {
      return OK;
    }

  return EINVAL;

#elif defined(CONFIG_PTHREAD_MUTEX_BOTH)

  if (attr != NULL && (robust == PTHREAD_MUTEX_STALLED ||
      robust == PTHREAD_MUTEX_ROBUST))
    {
      attr->robust = robust;
      return OK;
    }

  return EINVAL;

#else /* Default: CONFIG_PTHREAD_MUTEX_ROBUST */

  if (attr != NULL && robust == _PTHREAD_MFLAGS_ROBUST)
    {
      return OK;
    }

  return EINVAL;
#endif
}
