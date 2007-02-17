/***********************************************************************
 * sighand.c
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

#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <sched.h>
#include <errno.h>
#include "ostest.h"

#ifndef NULL
# define NULL (void*)0
#endif

#define WAKEUP_SIGNAL 17
#define SIGVALUE_INT  42

static sem_t sem;
static boolean sigreceived = FALSE;
static boolean threadexited = FALSE;

static void wakeup_action(int signo, siginfo_t *info, void *ucontext)
{
  sigset_t oldset;
  sigset_t allsigs;
  int status;

  printf("%s: Received signal %d\n", __FUNCTION__, signo);

  sigreceived = TRUE;

  /* Check signo */

  if (signo != WAKEUP_SIGNAL)
    {
      printf("%s: ERROR expected signo=%d\n",  __FUNCTION__, WAKEUP_SIGNAL);
    }

  /* Check siginfo */

  if (info->si_value.sival_int != SIGVALUE_INT)
    {
      printf("%s: ERROR sival_int=%d expected %d\n",
             __FUNCTION__, info->si_value.sival_int, SIGVALUE_INT);
    }
  else
    {
      printf("%s: sival_int=%d\n", __FUNCTION__, info->si_value.sival_int);
    }

  if (info->si_signo != WAKEUP_SIGNAL)
    {
      printf("%s: ERROR expected si_signo=%d, got=%d\n",
              __FUNCTION__, WAKEUP_SIGNAL, info->si_signo);
    }

  printf("%s: si_code=%d\n", __FUNCTION__, info->si_code);

  /* Check ucontext_t */

  printf("%s: ucontext=%p\n", __FUNCTION__, ucontext);

  /* Check sigprocmask */

  (void)sigfillset(&allsigs);
  status = sigprocmask(SIG_SETMASK, NULL, &oldset);
  if (status != OK)
    {
      printf("%s: ERROR sigprocmask failed, status=%d\n",
             __FUNCTION__, status);
    }

  if (oldset != allsigs)
    {
      printf("%s: ERROR sigprocmask=%x expected=%x\n",
             __FUNCTION__, oldset, allsigs);
    }

}

static int waiter_main(int argc, char *argv[])
{
  sigset_t sigset;
  struct sigaction act;
  struct sigaction oact;
  int status;

  printf("%s: Waiter started\n", __FUNCTION__);

  printf("%s: Unmasking signal %d\n", __FUNCTION__, WAKEUP_SIGNAL);
  (void)sigemptyset(&sigset);
  (void)sigaddset(&sigset, WAKEUP_SIGNAL);
  status = sigprocmask(SIG_UNBLOCK, &sigset, NULL);
  if (status != OK)
    {
      printf("%s: ERROR sigprocmask failed, status=%d\n",
             __FUNCTION__, status);
    }

  printf("%s: Registering signal handler\n", __FUNCTION__);
  act.sa_sigaction = wakeup_action;
  act.sa_flags  = SA_SIGINFO;

  (void)sigfillset(&act.sa_mask);
  (void)sigdelset(&act.sa_mask, WAKEUP_SIGNAL);

  status = sigaction(WAKEUP_SIGNAL, &act, &oact);
  if (status != OK)
    {
      printf("%s: ERROR sigaction failed, status=%d\n", __FUNCTION__, status);
    }

  printf("%s: oact.sigaction=%p oact.sa_flags=%x oact.sa_mask=%x\n",
         __FUNCTION__, oact.sa_sigaction, oact.sa_flags, oact.sa_mask);

  /* Take the semaphore */

  printf("%s: Waiting on semaphore\n", __FUNCTION__);
  fflush(stdout);
  status = sem_wait(&sem);
  if (status != 0)
    {
      int error = *get_errno_ptr();
      if (error == EINTR)
        {
          printf("%s: sem_wait() successfully interrupted by signal\n", __FUNCTION__);
        }
      else
        {
          printf("%s: ERROR sem_wait failed, errno=%d\n", __FUNCTION__, error);
        }
    }
  else
    {
      printf("%s: ERROR awakened with no error!\n", __FUNCTION__);
    }

  printf("%s: done\n", __FUNCTION__);
  fflush(stdout);
  threadexited = TRUE;
  return 0;
}

void sighand_test(void)
{
  struct sched_param param;
  union sigval sigvalue;
  pid_t waiterpid;
  int policy;
  int status;

  printf("%s: Initializing semaphore to 0\n", __FUNCTION__);
  sem_init(&sem, 0, 0);

  /* Start waiter thread  */

  printf("%s: Starting waiter task\n", __FUNCTION__);


  status = sched_getparam (0, &param);
  if (status != OK)
    {
      printf("%s: ERROR sched_getparam() failed\n", __FUNCTION__);
      param.sched_priority = PTHREAD_DEFAULT_PRIORITY;
    }

  policy = sched_getscheduler(0);
  if (policy == ERROR)
    {
      printf("%s: ERROR sched_getscheduler() failed\n", __FUNCTION__);
      policy = SCHED_FIFO;
    }

  waiterpid = task_create("waiter", param.sched_priority,
                           PTHREAD_STACK_DEFAULT, waiter_main, 0, 0, 0, 0);
  if (waiterpid == ERROR)
    {
      printf("%s: ERROR failed to start waiter_main\n", __FUNCTION__);
    }
  else
    {
       printf("%s: Started waiter_main pid=%d\n", __FUNCTION__, waiterpid);
    }

  /* Wait a bit */

  fflush(stdout);
  usleep(500*1000);

  /* Then signal the waiter thread. */

  sigvalue.sival_int = SIGVALUE_INT;
  status = sigqueue(waiterpid, WAKEUP_SIGNAL, sigvalue);
  if (status != OK)
    {
      printf("%s: ERROR sigqueue failed\n", __FUNCTION__);
      task_delete(waiterpid);
    }

  /* Wait a bit */

  fflush(stdout);
  usleep(500*1000);

  /* Then check the result */

  if (!threadexited)
    {
      printf("%s: ERROR waiter task did not exit\n", __FUNCTION__);
    }

  if (!sigreceived)
    {
      printf("%s: ERROR signal handler did not run\n", __FUNCTION__);
    }

  printf("%s: done\n", __FUNCTION__);
  fflush(stdout);
}
