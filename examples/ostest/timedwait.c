/***********************************************************************
 * timedwait.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************/

#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include "ostest.h"

static pthread_mutex_t mutex;
static pthread_cond_t  cond;

static void *thread_waiter(void *parameter)
{
  struct timespec time;
  int status;

  /* Take the mutex */

  printf("%s: Taking mutex\n", __FUNCTION__);
  status = pthread_mutex_lock(&mutex);
  if (status != 0)
    {
      printf("%s: ERROR pthread_mutex_lock failed, status=%d\n", __FUNCTION__, status);
    }

  printf("%s: Starting 5 second wait for condition\n", __FUNCTION__);

  status = clock_gettime(CLOCK_REALTIME, &time);
  if (status != 0)
    {
      printf("%s: ERROR clock_gettime failed\n", __FUNCTION__);
    }
  time.tv_sec += 5;

  /* The wait -- no-one is ever going to awaken us */

  status = pthread_cond_timedwait(&cond, &mutex, &time);
  if (status != 0)
    {
      printf("%s: ERROR pthread_cond_timedwait failed, status=%d\n", __FUNCTION__, status);
    }

  /* Release the mutex */

  printf("%s: Releasing mutex\n", __FUNCTION__);
  status = pthread_mutex_unlock(&mutex);
  if (status != 0)
    {
      printf("%s: ERROR pthread_mutex_unlock failed, status=%d\n", __FUNCTION__, status);
    }

  printf("%s: Exit with status 0x12345678\n", __FUNCTION__);
  pthread_exit((void*)0x12345678);
  return NULL;
}

void timedwait_test(void)
{
  pthread_t waiter;
  pthread_attr_t attr;
  struct sched_param sparam;
  void *result;
  int prio_max;
  int status;

  /* Initialize the mutex */

  printf("%s: Initializing mutex\n", __FUNCTION__);
  status = pthread_mutex_init(&mutex, NULL);
  if (status != 0)
    {
      printf("%s: ERROR pthread_mutex_init failed, status=%d\n", __FUNCTION__, status);
    }

  /* Initialize the condition variable */

  printf("%s: Initializing cond\n", __FUNCTION__);
  status = pthread_cond_init(&cond, NULL);
  if (status != 0)
    {
      printf("%s: ERROR pthread_condinit failed, status=%d\n", __FUNCTION__, status);
    }

  /* Start the waiter thread at higher priority */

  printf("%s: Starting waiter\n", __FUNCTION__);
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("%s: pthread_attr_init failed, status=%d\n", __FUNCTION__, status);
    }

  prio_max = sched_get_priority_max(SCHED_FIFO);
  status = sched_getparam (getpid(), &sparam);
  if (status != 0)
    {
      printf("%s: sched_getparam failed\n", __FUNCTION__);
      sparam.sched_priority = PTHREAD_DEFAULT_PRIORITY;
    }

  sparam.sched_priority = (prio_max + sparam.sched_priority) / 2;
  status = pthread_attr_setschedparam(&attr,&sparam);
  if (status != OK)
    {
      printf("%s: pthread_attr_setschedparam failed, status=%d\n", __FUNCTION__, status);
    }
  else
    {
      printf("%s: Set thread 2 priority to %d\n", __FUNCTION__, sparam.sched_priority);
    }

  status = pthread_create(&waiter, &attr, thread_waiter, NULL);
  if (status != 0)
    {
      printf("%s: pthread_create failed, status=%d\n", __FUNCTION__, status);
    }

  printf("%s: Joining\n", __FUNCTION__);
  status = pthread_join(waiter, &result);
  if (status != 0)
    {
      printf("%s: ERROR pthread_join failed, status=%d\n", __FUNCTION__, status);
    }
  else
    {
      printf("%s: waiter exited with result=%p\n", __FUNCTION__, result);
    }
}
