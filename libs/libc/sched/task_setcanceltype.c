/****************************************************************************
 * libs/libc/sched/task_setcanceltype.c
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

#include <sched.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_setcanceltype
 *
 * Description:
 *   The task_setcanceltype() function atomically both sets the calling
 *   task's cancellability type to the indicated type and returns the
 *   previous cancellability type at the location referenced by oldtype
 *   Legal values for type are TASK_CANCEL_DEFERRED and
 *   TASK_CANCEL_ASYNCHRONOUS.
 *
 *   The cancellability state and type of any newly created tasks are
 *   TASK_CANCEL_ENABLE and TASK_CANCEL_DEFERRED respectively.
 *
 ****************************************************************************/

int task_setcanceltype(int type, FAR int *oldtype)
{
  /* Return the current type if so requested */

  if (oldtype != NULL)
    {
      *oldtype = TASK_CANCEL_ASYNCHRONOUS;
    }

  /* Check the requested cancellation type */

  return (type == TASK_CANCEL_ASYNCHRONOUS) ? OK : EINVAL;
}
