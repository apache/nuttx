/****************************************************************************
 * libs/libc/pthread/pthread_once.c
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
#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_once
 *
 * Description:
 *   The  first call to pthread_once() by any thread with a given
 *   once_control, will call the init_routine with no arguments. Subsequent
 *   calls to pthread_once() with the same once_control will have no effect.
 *   On return from pthread_once(), init_routine will have completed.
 *
 * Input Parameters:
 *   once_control - Determines if init_routine should be called.
 *     once_control should be declared and initializeed as follows:
 *
 *        pthread_once_t once_control = PTHREAD_ONCE_INIT;
 *
 *     PTHREAD_ONCE_INIT is defined in pthread.h
 *   init_routine - The initialization routine that will be called once.
 *
 * Returned Value:
 *   0 (OK) on success or EINVAL if either once_control or init_routine are
 *   invalid
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_once(FAR pthread_once_t *once_control,
                 CODE void (*init_routine)(void))
{
  /* Sanity checks */

  if (once_control && init_routine)
    {
      /* Prohibit pre-emption while we test and set the once_control */

      sched_lock();
      if (!*once_control)
        {
          *once_control = true;

          /* Call the init_routine with pre-emption enabled. */

          sched_unlock();
          init_routine();
          return OK;
        }

      /* The init_routine has already been called.
       * Restore pre-emption and return
       */

      sched_unlock();
      return OK;
    }

  /* One of the two arguments is NULL */

  return EINVAL;
}
