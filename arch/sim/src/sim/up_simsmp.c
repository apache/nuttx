/****************************************************************************
 * arch/sim/src/sim/up_simsmp.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <sched.h>
#include <errno.h>

#include "up_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_cpuinfo_s
{
  int cpu;                       /* CPU number */
  pthread_cond_t cpu_init_done;  /* For synchronization */
  pthread_mutex_t cpu_init_lock;
  bool is_cpu_initialized;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_key_t g_cpu_key;
static pthread_t     g_cpu_thread[CONFIG_SMP_NCPUS];

/* These spinlocks are used in the SMP configuration in order to implement
 * up_cpu_pause().  The protocol for CPUn to pause CPUm is as follows
 *
 * 1. The up_cpu_pause() implementation on CPUn locks both g_cpu_wait[m]
 *    and g_cpu_paused[m].  CPUn then waits spinning on g_cpu_paused[m].
 * 2. CPUm receives the interrupt it (1) unlocks g_cpu_paused[m] and
 *    (2) locks g_cpu_wait[m].  The first unblocks CPUn and the second
 *    blocks CPUm in the interrupt handler.
 *
 * When CPUm resumes, CPUn unlocks g_cpu_wait[m] and the interrupt handler
 * on CPUm continues.  CPUm must, of course, also then unlock g_cpu_wait[m]
 * so that it will be ready for the next pause operation.
 */

volatile uint8_t g_cpu_wait[CONFIG_SMP_NCPUS];
volatile uint8_t g_cpu_paused[CONFIG_SMP_NCPUS];

/****************************************************************************
 * NuttX domain function prototypes
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION
void sched_note_cpu_start(struct tcb_s *tcb, int cpu);
void sched_note_cpu_pause(struct tcb_s *tcb, int cpu);
void sched_note_cpu_resume(struct tcb_s *tcb, int cpu);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_idle_trampoline
 *
 * Description:
 *   This is a pthread task entry point.  A (host) pthread is used to
 *   simulate a CPU.  Multiple pthreads is a good analog to tasks running on
 *   multiple CPUs
 *
 *   This function is simply a wrapper that sets the pthread specific data
 *   that presents the CPU number and then calls into the IDLE task entry
 *   point.
 *
 * Input Parameters:
 *   arg - Standard pthread argument
 *
 * Returned Value:
 *   This function does not return
 *
 ****************************************************************************/

static void *sim_idle_trampoline(void *arg)
{
  struct sim_cpuinfo_s *cpuinfo = (struct sim_cpuinfo_s *)arg;
#ifdef CONFIG_SIM_WALLTIME
  uint64_t now = 0;
#endif
  sigset_t set;
  int ret;

  /* Set the CPU number for the CPU thread */

  ret = pthread_setspecific(g_cpu_key,
                           (const void *)((uintptr_t)cpuinfo->cpu));
  if (ret != 0)
    {
      return NULL;
    }

  /* Make sure the SIGUSR1 is not masked */

  sigemptyset(&set);
  sigaddset(&set, SIGUSR1);

  ret = pthread_sigmask(SIG_UNBLOCK, &set, NULL);
  if (ret < 0)
    {
      return NULL;
    }

  /* Let up_cpu_start() continue */

  pthread_mutex_lock(&cpuinfo->cpu_init_lock);

  cpuinfo->is_cpu_initialized = true;
  pthread_cond_signal(&cpuinfo->cpu_init_done);

  pthread_mutex_unlock(&cpuinfo->cpu_init_lock);

  /* up_cpu_started() is logically a part of this function but needs to be
   * inserted in the path because in needs to access NuttX domain definition.
   */

  up_cpu_started();

  /* The idle Loop */

  for (; ; )
    {
#ifdef CONFIG_SIM_WALLTIME
      /* Wait a bit so that the timing is close to the correct rate. */

      now += 1000 * CONFIG_USEC_PER_TICK;
      host_sleepuntil(now);
#else
      /* Give other pthreads/CPUs a shot */

      sched_yield();
#endif
    }

  return NULL;
}

/****************************************************************************
 * Name: sim_handle_signal
 *
 * Description:
 *   This is the SIGUSR signal handler.  It implements the core logic of
 *   up_cpu_pause() on the thread of execution the simulated CPU.
 *
 * Input Parameters:
 *   arg - Standard sigaction arguments
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sim_handle_signal(int signo, siginfo_t *info, void *context)
{
  int cpu = (int)((uintptr_t)pthread_getspecific(g_cpu_key));

  up_cpu_paused(cpu);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_cpu0_start
 *
 * Description:
 *   Create the pthread-specific data key and set the indication of CPU0
 *   the main thread.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

void sim_cpu0_start(void)
{
  struct sigaction act;
  sigset_t set;
  int ret;

  g_cpu_thread[0] = pthread_self();

  /* Create the pthread key */

  ret = pthread_key_create(&g_cpu_key, NULL);
  if (ret != 0)
    {
      return;
    }

  /* Set the CPU number zero for the CPU thread */

  ret = pthread_setspecific(g_cpu_key, (const void *)0);
  if (ret != 0)
    {
      return;
    }

  /* Register the common signal handler for all threads */

  act.sa_sigaction = sim_handle_signal;
  act.sa_flags     = SA_SIGINFO;
  sigemptyset(&act.sa_mask);

  ret = sigaction(SIGUSR1, &act, NULL);
  if (ret < 0)
    {
      return;
    }

  /* Make sure the SIGUSR1 is not masked */

  sigemptyset(&set);
  sigaddset(&set, SIGUSR1);

  ret = pthread_sigmask(SIG_UNBLOCK, &set, NULL);
  if (ret < 0)
    {
      return;
    }
}

/****************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   Return an index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 ****************************************************************************/

int up_cpu_index(void)
{
  void *value = pthread_getspecific(g_cpu_key);
  return (int)((uintptr_t)value);
}

/****************************************************************************
 * Name: up_cpu_start
 *
 * Description:
 *   In an SMP configution, only one CPU is initially active (CPU 0). System
 *   initialization occurs on that single thread. At the completion of the
 *   initialization of the OS, just before beginning normal multitasking,
 *   the additional CPUs would be started by calling this function.
 *
 *   Each CPU is provided the entry point to is IDLE task when started.  A
 *   TCB for each CPU's IDLE task has been initialized and placed in the
 *   CPU's g_assignedtasks[cpu] list.  Not stack has been allocated or
 *   initialized.
 *
 *   The OS initialization logic calls this function repeatedly until each
 *   CPU has been started, 1 through (CONFIG_SMP_NCPUS-1).
 *
 * Input Parameters:
 *   cpu - The index of the CPU being started.  This will be a numeric
 *         value in the range of from one to (CONFIG_SMP_NCPUS-1).  (CPU
 *         0 is already active)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpu_start(int cpu)
{
  struct sim_cpuinfo_s cpuinfo;
  int ret;

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the start event */

  sched_note_cpu_start(up_this_task(), cpu);
#endif

  /* Initialize the CPU info */

  cpuinfo.cpu = cpu;
  cpuinfo.is_cpu_initialized = false;

  pthread_mutex_init(&cpuinfo.cpu_init_lock, NULL);
  pthread_cond_init(&cpuinfo.cpu_init_done, NULL);

  /* Start the CPU emulation thread.  This is analogous to starting the CPU
   * in a multi-CPU hardware model.
   */

  ret = pthread_create(&g_cpu_thread[cpu],
                       NULL, sim_idle_trampoline, &cpuinfo);
  if (ret != 0)
    {
      ret = -ret;  /* REVISIT:  That is a host errno value. */
      goto errout_with_cond;
    }

  /* This will block until the pthread post the semaphore */

  pthread_mutex_lock(&cpuinfo.cpu_init_lock);

  while (!cpuinfo.is_cpu_initialized)
    {
      pthread_cond_wait(&cpuinfo.cpu_init_done, &cpuinfo.cpu_init_lock);
    }

  pthread_mutex_unlock(&cpuinfo.cpu_init_lock);

errout_with_cond:
  pthread_mutex_destroy(&cpuinfo.cpu_init_lock);
  pthread_cond_destroy(&cpuinfo.cpu_init_done);
  return ret;
}

/****************************************************************************
 * Name: up_cpu_pause
 *
 * Description:
 *   Save the state of the current task at the head of the
 *   g_assignedtasks[cpu] task list and then pause task execution on the
 *   CPU.
 *
 *   This function is called by the OS when the logic executing on one CPU
 *   needs to modify the state of the g_assignedtasks[cpu] list for another
 *   CPU.
 *
 * Input Parameters:
 *   cpu - The index of the CPU to be stopped/
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpu_pause(int cpu)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the pause event */

  sched_note_cpu_pause(up_this_task(), cpu);
#endif

  /* Take the spinlock that will prevent the CPU thread from running */

  g_cpu_wait[cpu]   = 1;
  g_cpu_paused[cpu] = 1;

  /* Signal the CPU thread */

  pthread_kill(g_cpu_thread[cpu], SIGUSR1);

  /* Spin, waiting for the thread to pause */

  while (g_cpu_paused[cpu] != 0)
    {
      sched_yield();
    }

  return 0;
}

/****************************************************************************
 * Name: up_cpu_resume
 *
 * Description:
 *   Restart the cpu after it was paused via up_cpu_pause(), restoring the
 *   state of the task at the head of the g_assignedtasks[cpu] list, and
 *   resume normal tasking.
 *
 *   This function is called after up_cpu_pause in order resume operation of
 *   the CPU after modifying its g_assignedtasks[cpu] list.
 *
 * Input Parameters:
 *   cpu - The index of the CPU being re-started.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpu_resume(int cpu)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the resume event */

  sched_note_cpu_resume(up_this_task(), cpu);
#endif

  /* Release the spinlock that will alloc the CPU thread to continue */

  g_cpu_wait[cpu] = 0;
  return 0;
}
