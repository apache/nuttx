/****************************************************************************
 * arch/sim/src/sim/up_hosttime.c
 *
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Sebastian Ene <nuttx@fitbit.com>
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

#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <sys/types.h>
#include <sys/time.h>
#include <sys/wait.h>

#include <time.h>
#include <unistd.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The callback invoked from signal handler */

typedef void (*sched_timer_callback_t)(void);

/* The primask bit index corresponding to a host signal */

enum host_primask_bitset_e
{
  PRIMASK_BIT_0_SIGALARM = 0,
  PRIMASK_BIT_1_SIGUSR1  = 1,
  PRIMASK_BIT_SIZE
};

typedef enum host_primask_bitset_e host_primask_bitset_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SIM_PREEMPTIBLE
static void host_signal_handler(int sig, siginfo_t *si, void *old_ucontext);

static int host_setup_timer(void);
static int host_setup_signals(void (*action)(int, siginfo_t *, void *));

static uint32_t
host_get_irqstate_from_signal_mask(const sigset_t *signal_mask);
static void host_get_signal_mask_from_irqstate(sigset_t *restored_mask,
                                               uint32_t irqstate);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The callback to the NuttX scheduler that will be invoked from sig
 * handler.
 */

static sched_timer_callback_t g_sched_process_timer_cb;

/* The map between the host signal and the bit index the irqstate_t from
 * NuttX.
 */

static const int g_host_signal_map[PRIMASK_BIT_SIZE] =
{
  SIGALRM,
  SIGUSR1
};

#endif /* CONFIG_SIM_PREEMPTIBLE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SIM_PREEMPTIBLE
/****************************************************************************
 * Name: host_signal_handler
 *
 * Description:
 *   The signal handler is called periodically and is used to deliver TICK
 *   events to the OS.
 *
 * Input Parameters:
 *   sig - the signal number
 *   si  - the signal information
 *   old_ucontext - the previous context
 *
 ****************************************************************************/

static void host_signal_handler(int sig, siginfo_t *si, void *old_ucontext)
{
  g_sched_process_timer_cb();
}

/****************************************************************************
 * Name: host_setup_timer
 *
 * Description:
 *   Set up a timer to send periodic signals.
 *
 ****************************************************************************/

static int host_setup_timer(void)
{
  int ret;
  struct itimerval it;

  it.it_interval.tv_sec  = 0;
  it.it_interval.tv_usec = CONFIG_USEC_PER_TICK;

  it.it_value = it.it_interval;
  ret = setitimer(ITIMER_REAL, &it, NULL);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR %d settimer\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: host_setup_signals
 *
 * Description:
 *   Set up a signal to deliver periodic TICK events.
 *
 * Input Parameters:
 *   action - the callback invoked when we are interrupted by a signal
 *
 * Returned Value:
 *   This function returns 0 on success, on error -1 is returned and host
 *   errno is set to indicate an error.
 *
 ****************************************************************************/

static int host_setup_signals(void (*action)(int, siginfo_t *, void *))
{
  struct sigaction act;
  int ret;
  sigset_t set;

  act.sa_sigaction = action;
  sigemptyset(&act.sa_mask);
  act.sa_flags = SA_SIGINFO;

  sigemptyset(&set);
  sigaddset(&set, SIGALRM);
  pthread_sigmask(SIG_UNBLOCK, &set, NULL);

  if ((ret = sigaction(SIGALRM, &act, NULL)) != 0)
    {
      fprintf(stderr, "ERROR %d signal handler", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: host_get_irqstate_from_signal_mask
 *
 * Description:
 *   A helper function that creates the NuttX IRQ mask from the host signal
 *   mask.
 *
 * Input Parameters:
 *   signal_mask - the signal mask for the current running process on host
 *
 * Description:
 *   Return a bitmask with the current active signals.
 *
 ****************************************************************************/

static uint32_t
host_get_irqstate_from_signal_mask(const sigset_t *signal_mask)
{
  uint32_t nx_irq_mask = 0;

  for (int bit_index = 0; bit_index < PRIMASK_BIT_SIZE; bit_index++)
    {
      if (sigismember(signal_mask, g_host_signal_map[bit_index]))
        {
          nx_irq_mask |= (1 << bit_index);
        }
    }

  return nx_irq_mask;
}

/****************************************************************************
 * Name: host_get_signal_mask_from_irq_state
 *
 * Description:
 *   A helper function that creates a host signal mask from NuttX IRQ mask.
 *
 * Input Parameters:
 *   restored_mask - (out) buffer where we store the active signals
 *   irqstate      - the mask with the active signals
 *
 ****************************************************************************/

static void host_get_signal_mask_from_irqstate(sigset_t *restored_mask,
                                               uint32_t irqstate)
{
  if (restored_mask == NULL)
    {
      return;
    }

  for (int bit_index = 0; bit_index < PRIMASK_BIT_SIZE; bit_index++)
    {
      if ((irqstate & (1 << bit_index)) != 0)
        {
          sigaddset(restored_mask, g_host_signal_map[bit_index]);
        }
    }
}
#endif /* CONFIG_SIM_PREEMPTIBLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_gettime
 ****************************************************************************/

uint64_t host_gettime(bool rtc)
{
  struct timespec tp;

  clock_gettime(rtc ? CLOCK_REALTIME : CLOCK_MONOTONIC, &tp);
  return 1000000000ull * tp.tv_sec + tp.tv_nsec;
}

/****************************************************************************
 * Name: host_sleep
 ****************************************************************************/

void host_sleep(uint64_t nsec)
{
  usleep(nsec);
}

/****************************************************************************
 * Name: host_sleepuntil
 ****************************************************************************/

void host_sleepuntil(uint64_t nsec)
{
  static uint64_t base;
  uint64_t now;

  now = host_gettime(false);
  if (base == 0)
    {
      base = now;
    }

  now -= base;

  if (nsec > now + 1000)
    {
      usleep((nsec - now) / 1000);
    }
}

#ifdef CONFIG_SIM_PREEMPTIBLE
/****************************************************************************
 * Name: host_prepare_timer
 *
 * Description:
 *   Blocks all the signals in the current host thread so that the new
 *   threads can inherit a blocked signal mask.
 *
 * Returned Value:
 *   This function returns 0 on success otherwise -1.
 *
 * Assumptions/Limitations:
 *   This function should be called prior to spawning new threads as we
 *   don't want to end up running nxsched_process_timer on a different
 *   host thread.
 *
 ****************************************************************************/

int host_prepare_timer(void)
{
  sigset_t signal_mask;
  sigemptyset(&signal_mask);

  return pthread_sigmask(SIG_SETMASK, &signal_mask, NULL);
}

/****************************************************************************
 * Name: host_init_timer
 *
 * Description:
 *   Creates a periodic timer and sets up a signal handler on the host.
 *
 * Input Parameters:
 *   nxsched_process_timer - the NuttX scheduler callback
 *
 * Returned Value:
 *   This function returns 0 on success otherwise a negative error code.
 *
 ****************************************************************************/

int host_init_timer(sched_timer_callback_t sched_cb)
{
  g_sched_process_timer_cb = sched_cb;

  int ret = host_setup_signals(host_signal_handler);
  if (ret < 0)
    {
      return ret;
    }

  ret = host_setup_timer();
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: host_signal_save
 *
 * Description:
 *   Block signals and return the previous signal mask.
 *
 * Returned Value:
 *   This function returns the list of active signals before blocking them.
 *
 ****************************************************************************/

uint32_t host_signal_save(void)
{
  sigset_t oldset, newset;
  pthread_sigmask(0, NULL, &oldset);

  /* Disable signals */

  sigfillset(&newset);
  pthread_sigmask(SIG_BLOCK, &newset, NULL);

  return host_get_irqstate_from_signal_mask(&oldset);
}

/****************************************************************************
 * Name: host_signal_restore
 *
 * Description:
 *   Re-enable signals by restoring the signal mask.
 *
 * Input Parameters:
 *   flags - mask to restore the signals
 *
 ****************************************************************************/

void host_signal_restore(uint32_t flags)
{
  sigset_t oldset;

  sigemptyset(&oldset);
  host_get_signal_mask_from_irqstate(&oldset, flags);
  pthread_sigmask(SIG_SETMASK, &oldset, NULL);
}
#endif
