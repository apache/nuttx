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

#ifdef CONFIG_ARCH_SIM
#  include <nuttx/arch.h>
#endif

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

enum thstate_e
{
  NOTSTARTED = 0,
  RUNNING,
  WAITING,
  DONE
};

static sem_t g_sem;
static volatile enum thstate_e g_middlestate = NOTSTARTED;
static volatile enum thstate_e g_highstate   = NOTSTARTED;
static volatile enum thstate_e g_lowstate    = NOTSTARTED;
static int g_highpri;
static int g_medpri;
static int g_lowpri;

/****************************************************************************
 * Name: highpri_thread
 ****************************************************************************/

static void *highpri_thread(void *parameter)
{
  int ret;

  printf("highpri_thread: Started\n");
  fflush(stdout);

  g_highstate = WAITING;
  ret = sem_wait(&g_sem);
  g_highstate = DONE;

  if (ret != 0)
    {
      printf("highpri_thread: sem_take failed: %d\n", ret);
    }
  else if (g_middlestate == RUNNING)
    {
      printf("highpri_thread: SUCCESS midpri_thread is still running!\n");
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
 * Name: hog_cpu
 ****************************************************************************/

static inline void hog_cpu(void)
{
#ifdef CONFIG_ARCH_SIM
  /* The simulator doesn't have any mechanism to do asynchronous pre-emption
   * (basically because it doesn't have any interupts/asynchronous events).
   * The simulator does "fake" a timer interrupt in up_idle() -- the idle
   * thread that only executes when nothing else is running.  In the simulator,
   * we cannot suspend the middle priority task, or we wouldn't have the
   * test that we want.  So, we have no option but to pump the fake clock
   * here by calling up_idle().  Sigh!
   */

  up_idle();
#else
  /* On real platforms with a real timer interrupt, we really can hog the
   * CPU.  When the sleep() goes off in priority_inheritance(), it will
   * wake up and start the high priority thread.
   */

  volatile int i;
  for (i = 0; i < INT_MAX; i++);
#endif
}

/****************************************************************************
 * Name: medpri_thread
 ****************************************************************************/

static void *medpri_thread(void *parameter)
{
  printf("medpri_thread: Started ... I won't let go of the CPU!\n");
  g_middlestate = RUNNING;
  fflush(stdout);

  /* The following loop will completely block lowpri_thread from running.
   * UNLESS priority inheritance is working.  In that case, its priority
   * will be boosted.
   */

  while (g_highstate != DONE)
    {
      hog_cpu();
    }

  printf("medpri_thread: Okay... I'm done!\n");
  fflush(stdout);
  g_middlestate = DONE;
  return NULL;
}

/****************************************************************************
 * Name: lowpri_thread
 ****************************************************************************/

static void *lowpri_thread(void *parameter)
{
  void *retval = (void*)-1;
  struct sched_param sparam;
  int policy;
  int ret;

  g_lowstate = RUNNING;
  printf("lowpri_thread: Started\n");

  ret = pthread_getschedparam(pthread_self(), &policy, &sparam);
  if (ret != 0)
    {
      printf("lowpri_thread: ERROR pthread_getschedparam failed: %d\n", ret);
    }
  else
    {
      printf("lowpri_thread: initial priority: %d\n", sparam.sched_priority);
      if (sparam.sched_priority != g_lowpri)
        {
          printf("               ERROR should have been %d\n", g_lowpri);
        } 
    }

  g_lowstate = WAITING;
  ret = sem_wait(&g_sem);
  if (ret != 0)
    {
      printf("lowpri_thread: sem_take failed: %d\n", ret);
    }
  else
    {
      /* Hang on to the thread until the middle priority thread runs */

      while (g_middlestate == NOTSTARTED && g_highstate != WAITING)
        {
          printf("lowpri_thread: Waiting for the midle pri task to run\n");
          printf("               g_middlestate=%d g_highstate=%d\n", (int)g_middlestate, (int)g_highstate);
          printf("               I still have the semaphore\n");
          fflush(stdout);
          sleep(1);
        }

      /* The middle priority task is running, let go of the semaphore */

     if (g_middlestate == RUNNING && g_highstate == WAITING)
       {
         /* Good.. the middle priority task is still running but we got priority! */

         retval = NULL;
       }
     else
       {
         printf("lowpri_thread: ERROR the middle priority task has already exitted!\n");
         printf("               g_middlestate=%d g_highstate=%d\n", (int)g_middlestate, (int)g_highstate);
       }
    }

  ret = pthread_getschedparam(pthread_self(), &policy, &sparam);
  sem_post(&g_sem);
  if (ret != 0)
    {
      printf("lowpri_thread: ERROR pthread_getschedparam failed: %d\n", ret);
    }
  else
    {
      printf("lowpri_thread: Priority before sem_post: %d\n", sparam.sched_priority);
      if (sparam.sched_priority != g_highpri)
        {
          printf("               ERROR should have been %d\n", g_highpri);
        } 
    }


  ret = pthread_getschedparam(pthread_self(), &policy, &sparam);
  if (ret != 0)
    {
      printf("lowpri_thread: ERROR pthread_getschedparam failed: %d\n", ret);
    }
  else
    {
      printf("lowpri_thread: Final priority: %d\n", sparam.sched_priority);
      if (sparam.sched_priority != g_lowpri)
        {
          printf("               ERROR should have been %d\n", g_lowpri);
        } 
    }

  printf("lowpri_thread: Okay... I'm done!\n");
  fflush(stdout);
  g_lowstate = DONE;
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
  int status;

  printf("priority_inheritance: Started\n");

  status = sched_getparam (getpid(), &sparam);
  if (status != 0)
    {
      printf("priority_inheritance: sched_getparam failed\n");
      sparam.sched_priority = PTHREAD_DEFAULT_PRIORITY;
    }
  my_pri  = sparam.sched_priority;

  g_highpri = sched_get_priority_max(SCHED_FIFO);
  g_lowpri = sched_get_priority_min(SCHED_FIFO);
  g_medpri = my_pri - 1;

  sem_init(&g_sem, 0, 1);

  /* Start the low priority task */

  printf("priority_inheritance: Starting lowpri_thread at %d\n", g_lowpri);
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("priority_inheritance: pthread_attr_init failed, status=%d\n", status);
    }
  sparam.sched_priority = g_lowpri;
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
  sleep(2);

  /* Start the medium priority task */

  printf("priority_inheritance: Starting medpri_thread at %d\n", g_medpri);
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("priority_inheritance: pthread_attr_init failed, status=%d\n", status);
    }

  sparam.sched_priority = g_medpri;
  status = pthread_attr_setschedparam(&attr,& sparam);
  if (status != OK)
    {
      printf("priority_inheritance: pthread_attr_setschedparam failed, status=%d\n", status);
    }
  else
    {
      printf("priority_inheritance: Set medpri_thread priority to %d\n", sparam.sched_priority);
    }
  fflush(stdout);

  status = pthread_create(&medpri, &attr, medpri_thread, NULL);
  if (status != 0)
    {
      printf("priority_inheritance: pthread_create failed, status=%d\n", status);
    }
  printf("priority_inheritance: Waiting...\n");
  sleep(1);

  /* Start the high priority task */

  printf("priority_inheritance: Starting highpri_thread at %d\n", g_highpri);
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("priority_inheritance: pthread_attr_init failed, status=%d\n", status);
    }

  sparam.sched_priority = g_highpri;
  status = pthread_attr_setschedparam(&attr,& sparam);
  if (status != OK)
    {
      printf("priority_inheritance: pthread_attr_setschedparam failed, status=%d\n", status);
    }
  else
    {
      printf("priority_inheritance: Set highpri_thread priority to %d\n", sparam.sched_priority);
    }
  fflush(stdout);

  status = pthread_create(&medpri, &attr, highpri_thread, NULL);
  if (status != 0)
    {
      printf("priority_inheritance: pthread_create failed, status=%d\n", status);
    }

  /* Wait for all thread instances to complete */

  printf("priority_inheritance: Waiting for highpri_thread to complete\n");
  fflush(stdout);
  (void)pthread_join(highpri, &result);
  printf("priority_inheritance: Waiting for medpri_thread to complete\n");
  fflush(stdout);
  (void)pthread_join(medpri, &result);
  printf("priority_inheritance: Waiting for lowpri_thread to complete\n");
  fflush(stdout);
  (void)pthread_join(lowpri, &result);

  printf("priority_inheritance: Finished\n");
  sem_destroy(&g_sem);
  fflush(stdout);
#endif /* CONFIG_PRIORITY_INHERITANCE && !CONFIG_DISABLE_SIGNALS && !CONFIG_DISABLE_PTHREAD */
}
