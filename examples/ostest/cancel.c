/***********************************************************************
 * cancel.c
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
#include <errno.h>
#include "ostest.h"

static pthread_mutex_t mutex;
static pthread_cond_t  cond;

static void *thread_waiter(void *parameter)
{
  int status;

  /* Take the mutex */

  printf("%s: Taking mutex\n", __FUNCTION__);
  status = pthread_mutex_lock(&mutex);
  if (status != 0)
    {
       printf("%s: ERROR pthread_mutex_lock failed, status=%d\n", __FUNCTION__, status);
    }

  printf("%s: Starting wait for condition\n", __FUNCTION__);

  /* Are we a non-cancelable thread?   Yes, set the non-cancelable state */

  if (!parameter)
    {
      printf("%s: Setting non-cancelable\n", __FUNCTION__);
      status = pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
      if (status != 0)
        {
           printf("%s: ERROR pthread_setcancelstate failed, status=%d\n", __FUNCTION__, status);
        }
    }

  /* The wait -- we will never awaken from this. */

  status = pthread_cond_wait(&cond, &mutex);
  if (status != 0)
    {
      printf("%s: ERROR pthread_cond_wait failed, status=%d\n", __FUNCTION__, status);
    }

  /* Release the mutex */

  printf("%s: Releasing mutex\n", __FUNCTION__);
  status = pthread_mutex_unlock(&mutex);
  if (status != 0)
    {
      printf("%s: ERROR pthread_mutex_unlock failed, status=%d\n", __FUNCTION__, status);
    }

  /* Set the cancelable state */

  printf("%s: Setting cancelable\n", __FUNCTION__);
  status = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
  if (status != 0)
    {
      printf("%s: ERROR pthread_setcancelstate failed, status=%d\n", __FUNCTION__, status);
    }

  printf("%s: Exit with status 0x12345678\n", __FUNCTION__);
  pthread_exit((void*)0x12345678);
  return NULL;
}

static void start_thread(pthread_t *waiter, int cancelable)
{
  pthread_attr_t attr;
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
      printf("%s: ERROR pthread_cond_init failed, status=%d\n", __FUNCTION__, status);
    }

  /* Set up attributes */

  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("%s: pthread_attr_init failed, status=%d\n", __FUNCTION__, status);
    }

  status = pthread_attr_setstacksize(&attr, 16384);
  if (status != 0)
    {
      printf("%s: pthread_attr_setstacksize failed, status=%d\n", __FUNCTION__, status);
    }

  /* Start the waiter thread  */

  printf("%s: Starting thread\n", __FUNCTION__);
  status = pthread_create(waiter, NULL, thread_waiter, (void*)cancelable);
  if (status != 0)
    {
      printf("%s: ERROR pthread_create failed, status=%d\n", __FUNCTION__, status);
    }

  /* Make sure that the waiter thread gets a chance to run */

  printf("%s: Yielding\n", __FUNCTION__);
  pthread_yield();

}

static void restart_thread(pthread_t *waiter, int cancelable)
{
  int status;

  /* Destroy the condition variable */

  printf("%s: Destroying cond\n", __FUNCTION__);
  status = pthread_cond_destroy(&cond);
  if (status != 0)
    {
      printf("%s: ERROR pthread_cond_destroy failed, status=%d\n", __FUNCTION__, status);
    }

  /* Destroy the mutex */

  printf("%s: Destroying mutex\n", __FUNCTION__);
  status = pthread_cond_destroy(&cond);
  if (status != 0)
    {
      printf("%s: ERROR pthread_mutex_destroy failed, status=%d\n", __FUNCTION__, status);
    }

  /* Then restart the thread */

  printf("%s: Re-starting thread\n", __FUNCTION__);
  start_thread(waiter, cancelable);
}

void cancel_test(void)
{
  pthread_t waiter;
  void *result;
  int status;

  /* Test 1: Normal Cancel *********************************************/
  /* Start the waiter thread  */

  printf("%s: Test 1: Normal Cancelation\n", __FUNCTION__);
  printf("%s: Starting thread\n", __FUNCTION__);
  start_thread(&waiter, 1);

  /* Then cancel it.  It should be in the pthread_cond_wait now */

  printf("%s: Canceling thread\n", __FUNCTION__);
  status = pthread_cancel(waiter);
  if (status != 0)
    {
      printf("%s: ERROR pthread_cancel failed, status=%d\n", __FUNCTION__, status);
    }

  /* Then join to the thread to pick up the result */

  printf("%s: Joining\n", __FUNCTION__);
  status = pthread_join(waiter, &result);
  if (status != 0)
    {
      printf("%s: ERROR pthread_join failed, status=%d\n", __FUNCTION__, status);
    }
  else
    {
      printf("%s: waiter exited with result=%p\n", __FUNCTION__, result);
      if (result != PTHREAD_CANCELED)
        {
          printf("%s: ERROR expected result=%p\n", __FUNCTION__, PTHREAD_CANCELED);
        }
      else
        {
          printf("%s: PASS thread terminated with PTHREAD_CANCELED\n", __FUNCTION__);
        }
    }

  /* Test 2: Cancel Detached Thread ************************************/

  printf("%s: Test 2: Cancelation of detached thread\n", __FUNCTION__);
  printf("%s: Re-starting thread\n", __FUNCTION__);
  restart_thread(&waiter, 1);

  /* Detach the thread */

  status = pthread_detach(waiter);
  if (status != 0)
    {
      printf("%s: ERROR pthread_detach, status=%d\n", __FUNCTION__, status);
    }

  /* Then cancel it.  It should be in the pthread_cond_wait now */

  printf("%s: Canceling thread\n", __FUNCTION__);
  status = pthread_cancel(waiter);
  if (status != 0)
    {
      printf("%s: ERROR pthread_cancel failed, status=%d\n", __FUNCTION__, status);
    }

  /* Join should now fail */

  printf("%s: Joining\n", __FUNCTION__);
  status = pthread_join(waiter, &result);
  if (status == 0)
    {
      printf("%s: ERROR pthread_join succeeded\n", __FUNCTION__);
    }
  else if (status != ESRCH)
    {
      printf("%s: ERROR pthread_join failed but with wrong status=%d\n", __FUNCTION__, status);
    }
  else
    {
      printf("%s: PASS pthread_join failed with status=ESRCH\n", __FUNCTION__);
    }

  /* Test 3: Non-cancelable threads ************************************/

  printf("%s: Test 3: Non-cancelable threads\n", __FUNCTION__);
  printf("%s: Re-starting thread (non-cancelable)\n", __FUNCTION__);
  restart_thread(&waiter, 0);

  /* Then cancel it.  It should be in the pthread_cond_wait now.  The
   * behavior here is non-standard:  when the thread is at a cancelation
   * point, it should be cancelable, even when cancelation is disable.
   *
   * The cancelation should succeed, because the cancelation is pending.
   */

  printf("%s: Canceling thread\n", __FUNCTION__);
  status = pthread_cancel(waiter);
  if (status != 0)
    {
      printf("%s: ERROR pthread_cancel failed, status=%d\n", __FUNCTION__, status);
    }

  /* Signal the thread.  It should wake up an restore the cancelable state.
   * When the cancelable state is re-enabled, the thread should be canceled.
   */

  status = pthread_mutex_lock(&mutex);
  if (status != 0)
    {
      printf("%s: ERROR pthread_mutex_lock failed, status=%d\n", __FUNCTION__, status);
    }

  status = pthread_cond_signal(&cond);
  if (status != 0)
    {
      printf("%s: ERROR pthread_cond_signal failed, status=%d\n", __FUNCTION__, status);
    }

  status = pthread_mutex_unlock(&mutex);
  if (status != 0)
    {
      printf("%s: ERROR pthread_mutex_unlock failed, status=%d\n", __FUNCTION__, status);
    }
}
