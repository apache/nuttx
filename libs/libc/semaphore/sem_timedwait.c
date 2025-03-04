/****************************************************************************
 * libs/libc/semaphore/sem_timedwait.c
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

#include <nuttx/semaphore.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sem_timedwait
 *
 * Description:
 *   This function will lock the semaphore referenced by sem as in the
 *   sem_wait() function. However, if the semaphore cannot be locked without
 *   waiting for another process or thread to unlock the semaphore by
 *   performing a sem_post() function, this wait will be terminated when the
 *   specified timeout expires.
 *
 *   The timeout will expire when the absolute time specified by abstime
 *   passes, as measured by the clock on which timeouts are based (that is,
 *   when the value of that clock equals or exceeds abstime), or if the
 *   absolute time specified by abstime has already been passed at the
 *   time of the call.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   abstime - The absolute time to wait until a timeout is declared.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  On failure, -1 (ERROR) is returned
 *   and the errno is set appropriately:
 *
 *   EINVAL    The sem argument does not refer to a valid semaphore.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The semaphore could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *   EINTR     A signal interrupted this function.
 *   ECANCELED May be returned if the thread is canceled while waiting.
 *
 ****************************************************************************/

int sem_timedwait(FAR sem_t *sem, FAR const struct timespec *abstime)
{
  return sem_clockwait(sem, CLOCK_REALTIME, abstime);
}
