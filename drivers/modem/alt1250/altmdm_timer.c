/****************************************************************************
 * drivers/modem/alt1250/altmdm_timer.c
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

#include <errno.h>
#include <debug.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <signal.h>

#include "altmdm_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MY_TIMER_SIGNAL SIGUSR1

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int set_signal(int signal_no, FAR _sa_sigaction_t handler)
{
  int ret;
  sigset_t mask;
  struct sigaction sa;

  sigemptyset(&mask);
  nxsig_addset(&mask, signal_no);

  ret = nxsig_procmask(SIG_UNBLOCK, &mask, NULL);
  if (ret != OK)
    {
      return ERROR;
    }

  sa.sa_sigaction = handler;
  sa.sa_flags = SA_SIGINFO;
  sigfillset(&sa.sa_mask);
  nxsig_delset(&sa.sa_mask, signal_no);

  ret = nxsig_action(signal_no, &sa, NULL, false);
  if (ret != OK)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

timer_t altmdm_timer_start(int first_ms, int interval_ms,
  FAR _sa_sigaction_t handler, FAR void *ptr_param)
{
  int ret;
  struct sigevent sev;
  struct itimerspec timer;
  timer_t timerid;

  if (set_signal(MY_TIMER_SIGNAL, handler) != OK)
    {
      return NULL;
    }

  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = MY_TIMER_SIGNAL;
  sev.sigev_value.sival_int = 0;
  sev.sigev_value.sival_ptr = ptr_param;

  ret = timer_create(CLOCK_REALTIME, &sev, &timerid);
  if (ret != OK)
    {
      return NULL;
    }

  timer.it_value.tv_sec = first_ms / 1000;
  timer.it_value.tv_nsec = (first_ms % 1000) * 1000 * 1000;
  timer.it_interval.tv_sec = interval_ms / 1000;
  timer.it_interval.tv_nsec = (interval_ms % 1000) * 1000 * 1000;

  ret = timer_settime(timerid, 0, &timer, NULL);
  if (ret != OK)
    {
      return NULL;
    }

  return timerid;
}

int altmdm_timer_restart(timer_t timerid, int first_ms, int interval_ms)
{
  int ret;
  struct itimerspec timer;

  timer.it_value.tv_sec = first_ms / 1000;
  timer.it_value.tv_nsec = (first_ms % 1000) * 1000 * 1000;
  timer.it_interval.tv_sec = interval_ms / 1000;
  timer.it_interval.tv_nsec = (interval_ms % 1000) * 1000 * 1000;

  ret = timer_settime(timerid, 0, &timer, NULL);
  if (ret != OK)
    {
      return ret;
    }

  return ret;
}

int altmdm_timer_is_running(timer_t timerid)
{
  struct itimerspec timer;

  timer_gettime(timerid, &timer);

  return (timer.it_value.tv_sec != 0 || timer.it_value.tv_nsec != 0);
}

void altmdm_timer_stop(timer_t timerid)
{
  sigset_t mask;

  timer_delete(timerid);

  sigfillset(&mask);
  nxsig_procmask(SIG_SETMASK, &mask, NULL);
}

