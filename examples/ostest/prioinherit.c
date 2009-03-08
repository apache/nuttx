/****************************************************************************
 * examples/ostest/prioinherit.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <semaphore.h>
#include <pthread.h>

#include "ostest.h"

#if defined(CONFIG_PRIORITY_INHERITANCE) && !defined(CONFIG_DISABLE_SIGNALS) && !defined(CONFIG_DISABLE_PTHREAD)

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static sem_t g_sem;
static volatile int g_middle = 0;

/****************************************************************************
 * Name: highpri_thread
 ****************************************************************************/

static void *highpri_thread(void *parameter)
{
  int ret;

  printf("highpri_thread: Thread %d started\n");

  ret = sem_wait(&g_sem);
  if (ret != 0)
    {
      printf("highpri_thread: sem_take failed: %d\n", ret);
    }
  else if (g_middle == 1)
    {
      printf("highpri_thread: Success midpri_thread is still running!\n");
    }
  else
    {
      printf("highpri_thread: ERROR --  midpri_thread has already exited!\n");
    }

  sem_post(&g_sem);
  printf("medpri_thread: Okay... I'm done!\n");
  fflush(stdout);
  return NULL;
}

/****************************************************************************
 * Name: medpri_thread
 ****************************************************************************/

static void *medpri_thread(void *parameter)
{
  volatile long i;

  printf("medpri_thread: Thread %d started ... I won't let go of the CPU!\n");
  g_middle = 1;
  for (i = 0; i < 0x7ffffff; i++);
  printf("medpri_thread: Okay... I'm done!\n");
  fflush(stdout);
  return NULL;
}

/****************************************************************************
 * Name: lowpri_thread
 ****************************************************************************/

static void *lowpri_thread(void *parameter)
{
  void *retval = (void*)-1;
  int ret;

  printf("lowpri_thread: Thread %d started\n");

  ret = sem_wait(&g_sem);
  if (ret != 0)
    {
      printf("lowpri_thread: sem_take failed: %d\n", ret);
    }
  else
    {
      /* Hang on to the thread until the middle priority thread runs */

      while (g_middle == 0)
        {
          printf("lowpri_thread: Waiting for the midle pri task to run\n");
          printf("lowpri_thread: I still have the semaphore\n");
          sleep(1);
        }

      /* The middle priority task is running, let go of the semaphore */

     if (g_middle == 1)
       {
         /* Good.. the middle priority task is still running but we got priority! */

         retval = NULL;
       }
     else
       {
         printf("lowpri_thread: ERROR the middle priority task has already exitted!\n");
       }
    }

  printf("lowpri_thread: Letting go of the semaphore\n");
  sem_post(&g_sem);
  printf("lowpri_thread: Okay... I'm done!\n");
  fflush(stdout);
  return retval;  
}
#endif /* CONFIG_PRIORITY_INHERITANCE && !CONFIG_DISABLE_SIGNALS && !CONFIG_DISABLE_PTHREAD */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: priority_inheritance
 ****************************************************************************/

void priority_inheritance(void)
{
#if defined(CONFIG_PRIORITY_INHERITANCE) && !defined(CONFIG_DISABLE_SIGNALS) && !defined(CONFIG_DISABLE_PTHREAD)
  pthread_t lowpri;
  pthread_t medpri;
  pthread_t highpri;
  pthread_addr_t result;
  pthread_attr_t attr;
  struct sched_param sparam;
  int my_pri;
  int max_pri;
  int mid_pri;
  int min_pri;
  int status;

  printf("priority_inheritance: Started\n");

  status = sched_getparam (getpid(), &sparam);
  if (status != 0)
    {
      printf("priority_inheritance: sched_getparam failed\n");
      sparam.sched_priority = PTHREAD_DEFAULT_PRIORITY;
    }
  my_pri  = sparam.sched_priority;

  max_pri = sched_get_priority_max(SCHED_FIFO);
  min_pri = sched_get_priority_min(SCHED_FIFO);
  mid_pri = my_pri - 1;

  sem_init(&g_sem, 0, 1);

  /* Start the low priority task */

  printf("priority_inheritance: Starting lowpri_thread at %d\n", min_pri);
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("priority_inheritance: pthread_attr_init failed, status=%d\n", status);
    }
  sparam.sched_priority = min_pri;
  status = pthread_attr_setschedparam(&attr,& sparam);
  if (status != OK)
    {
      printf("priority_inheritance: pthread_attr_setschedparam failed, status=%d\n", status);
    }
  else
    {
      printf("priority_inheritance: Set lowpri_thread priority to %d\n", sparam.sched_priority);
    }

  status = pthread_create(&lowpri, &attr, lowpri_thread, NULL);
  if (status != 0)
    {
      printf("priority_inheritance: pthread_create failed, status=%d\n", status);
    }

  printf("priority_inheritance: Waiting...\n");
  sleep(5);

  /* Start the medium priority task */

  printf("priority_inheritance: Starting medpri_thread at %d\n", mid_pri);
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("priority_inheritance: pthread_attr_init failed, status=%d\n", status);
    }

  sparam.sched_priority = mid_pri;
  status = pthread_attr_setschedparam(&attr,& sparam);
  if (status != OK)
    {
      printf("priority_inheritance: pthread_attr_setschedparam failed, status=%d\n", status);
    }
  else
    {
      printf("priority_inheritance: Set medpri_thread priority to %d\n", sparam.sched_priority);
    }

  status = pthread_create(&medpri, &attr, medpri_thread, NULL);
  if (status != 0)
    {
      printf("priority_inheritance: pthread_create failed, status=%d\n", status);
    }

  /* Start the high priority task */

  printf("priority_inheritance: Starting highpri_thread at %d\n", max_pri);
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("priority_inheritance: pthread_attr_init failed, status=%d\n", status);
    }

  sparam.sched_priority = max_pri;
  status = pthread_attr_setschedparam(&attr,& sparam);
  if (status != OK)
    {
      printf("priority_inheritance: pthread_attr_setschedparam failed, status=%d\n", status);
    }
  else
    {
      printf("priority_inheritance: Set highpri_thread priority to %d\n", sparam.sched_priority);
    }

  status = pthread_create(&medpri, &attr, highpri_thread, NULL);
  if (status != 0)
    {
      printf("priority_inheritance: pthread_create failed, status=%d\n", status);
    }

  /* Wait for all thread instances to complete */

  printf("priority_inheritance: Waiting for highpri_thread to complete\n");
  (void)pthread_join(highpri, &result);
  printf("priority_inheritance: Waiting for medpri_thread to complete\n");
  (void)pthread_join(medpri, &result);
  printf("priority_inheritance: Waiting for lowpri_thread to complete\n");
  (void)pthread_join(lowpri, &result);

  printf("priority_inheritance: Finished\n");
  sem_destroy(&g_sem);
  fflush(stdout);
#endif /* CONFIG_PRIORITY_INHERITANCE && !CONFIG_DISABLE_SIGNALS && !CONFIG_DISABLE_PTHREAD */
}
