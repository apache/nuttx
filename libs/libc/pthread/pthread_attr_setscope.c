/****************************************************************************
 * libs/libc/pthread/pthread_attr_setscope.c
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

#include <pthread.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_attr_setscope
 *
 * Description:
 *   The function sets the contention scope attribute of the thread
 *   attributes object referred to by attr to the value specified in scope.
 *   The contention scope attributes defines the set of threads against
 *   which a thread competes for resources such as the CPU.
 *
 *   POSIX.1 specifies two possible values for scope:
 *   PTHREAD_SCOPE_SYSTEM:
 *     The thread competes for resources with all other threads in all
 *     processes on the system that are in the same scheduling allocation
 *     domain (a group of one or  more  processors).
 *     PTHREAD_SCOPE_SYSTEM threads are scheduled relative to one another
 *     according to their scheduling policy and priority.
 *
 *   PTHREAD_SCOPE_PROCESS
 *     The thread competes for resources with all other threads in the
 *     same process that were also created with the PTHREAD_SCOPE_PROCESS
 *     contention scope. PTHREAD_SCOPE_PROCESS threads are scheduled relative
 *     to other threads in the process according to their scheduling policy
 *     and priority. POSIX.1 leaves it unspecified how these threads contend
 *     with other threads in other process on the system or with other
 *     threads in the same process that were created with the
 *     PTHREAD_SCOPE_SYSTEM contention scope.
 *
 * Input Parameters:
 *   attr  - The pointer to pthread attr.
 *   scope - contention scope.
 *
 * Returned Value:
 *   On success, these functions return 0; on error, they return a nonzero
 *   error number.
 *
 ****************************************************************************/

int pthread_attr_setscope(FAR pthread_attr_t *attr, int scope)
{
  switch (scope)
    {
      case PTHREAD_SCOPE_SYSTEM:
        return 0;
      case PTHREAD_SCOPE_PROCESS:
        return ENOTSUP;
      default:
        return EINVAL;
    }
}
