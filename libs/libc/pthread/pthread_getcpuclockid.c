/****************************************************************************
 * libs/libc/pthread/pthread_getcpuclockid.c
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
#include <time.h>
#include <nuttx/clock.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_getcpuclockid
 *
 * Description:
 *   The pthread_getcpuclockid() function using to access a thread CPU-time
 *   clock
 *
 * Input Parameters:
 *   thread_id - The id of the thread to fetch the running time
 *   clockid - The clockid with type CLOCK_THREAD_CPUTIME_ID that required
 *
 * Returned Value:
 *    Return 0 on success, return error number on error
 *
 ****************************************************************************/

int pthread_getcpuclockid(pthread_t thread_id, FAR clockid_t *clockid)
{
  if (pthread_kill(thread_id, 0) != 0)
    {
      return ESRCH;
    }

  /* for pthread_getcpuclockid, the clock type are
   * CLOCK_THREAD_CPUTIME_ID
   */

  *clockid = (thread_id << CLOCK_SHIFT) | CLOCK_THREAD_CPUTIME_ID;
  return OK;
}
