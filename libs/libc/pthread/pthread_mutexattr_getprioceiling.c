/****************************************************************************
 * libs/libc/pthread/pthread_mutexattr_getprioceiling.c
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
#include <pthread.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutexattr_getprioceiling
 *
 * Description:
 *   Return the mutex ceiling priority from the mutex attributes.
 *
 * Input Parameters:
 *   attr - The mutex attributes to query
 *   prioceiling - Location to return the mutex ceiling priority
 *
 * Returned Value:
 *   0, if the mutex type was successfully return in 'prioceiling', or
 *   EINVAL, if any NULL pointers provided.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_mutexattr_getprioceiling(FAR const pthread_mutexattr_t *attr,
                                     FAR int *prioceiling)
{
#ifdef CONFIG_PRIORITY_PROTECT
  if (attr != NULL && prioceiling != NULL)
    {
      *prioceiling = attr->ceiling;
      return OK;
    }
#endif

  return EINVAL;
}
