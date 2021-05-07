/****************************************************************************
 * libs/libc/pthread/pthread_setcancelstate.c
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
#include <sched.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* These are defined in different header files but must have the same
 * values.
 */

#if PTHREAD_CANCEL_ENABLE != TASK_CANCEL_ENABLE
#  error We must have  PTHREAD_CANCEL_ENABLE == TASK_CANCEL_ENABLE
#endif

#if PTHREAD_CANCEL_DISABLE != TASK_CANCEL_DISABLE
#  error We must have  PTHREAD_CANCEL_DISABLE == TASK_CANCEL_DISABLE
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_setcancelstate
 *
 * Description:
 *   The pthread_setcancelstate() function atomically both sets the calling
 *   thread's cancelability state to the indicated state and returns the
 *   previous cancelability state at the location referenced by oldstate.
 *   Legal values for state are PTHREAD_CANCEL_ENABLE and
 *   PTHREAD_CANCEL_DISABLE.
 *
 *   The cancelability state and type of any newly created threads,
 *   including the thread in which main() was first invoked, are
 *   PTHREAD_CANCEL_ENABLE and PTHREAD_CANCEL_DEFERRED respectively.
 *
 ****************************************************************************/

int pthread_setcancelstate(int state, FAR int *oldstate)
{
  int ret;

  /* task_setcancelstate() can do this */

  ret = task_setcancelstate(state, oldstate);
  if (ret < 0)
    {
      ret = get_errno();
    }

  return ret;
}
