/****************************************************************************
 * libs/libc/pthread/pthread_mutexattr_setprioceiling.c
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
#include <sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutexattr_setprioceiling
 *
 * Description:
 *   Set the mutex ceiling priority in the mutex attributes.
 *
 * Input Parameters:
 *   attr - The mutex attributes in which to set the mutex type.
 *   prioceiling - The mutex ceiling priority value to set.
 *
 * Returned Value:
 *   0, if the mutex type was successfully set in 'attr', or
 *   EINVAL, if 'attr' is NULL or 'prioceiling' out of range.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_mutexattr_setprioceiling(FAR pthread_mutexattr_t *attr,
                                     int prioceiling)
{
#ifdef CONFIG_PRIORITY_PROTECT
  if (attr && prioceiling >= sched_get_priority_min(SCHED_FIFO) &&
      prioceiling <= sched_get_priority_max(SCHED_FIFO))
    {
      attr->ceiling = prioceiling;
      return OK;
    }
#endif

  return EINVAL;
}
