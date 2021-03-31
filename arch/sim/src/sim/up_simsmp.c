/****************************************************************************
 * arch/sim/src/sim/up_simsmp.c
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

static pthread_t     g_timer_thread;

/****************************************************************************
 * NuttX domain function prototypes
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION
void sched_note_cpu_start(struct tcb_s *tcb, int cpu);
void sched_note_cpu_pause(struct tcb_s *tcb, int cpu);
void sched_note_cpu_resume(struct tcb_s *tcb, int cpu);
#endif

void up_irqinitialize(void);

extern uint8_t g_nx_initstate;

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
  int ret;

  /* Set the CPU number for the CPU thread */

  ret = pthread_setspecific(g_cpu_key,
                           (const void *)((uintptr_t)cpuinfo->cpu));
  if (ret != 0)
    {
      return NULL;
    }

  /* Initialize IRQ */

  up_irqinitialize();

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
 * Name: sim_host_timer_handler
 ****************************************************************************/

static void *sim_host_timer_handler(void *arg)
{
  /* Wait until OSINIT_OSREADY(5) */

  while (g_nx_initstate < 5)
    {
      host_sleep(10 * 1000); /* 10ms */
    }

  /* Send a periodic timer event to CPU0 */

  while (1)
    {
      pthread_kill(g_cpu_thread[0], SIGUSR1);
      host_sleep(10 * 1000); /* 10ms */
    }

  return NULL;
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

  /* NOTE: IRQ initialization will be done in up_irqinitialize */

  /* Create timer thread to send a periodic timer event */

  ret = pthread_create(&g_timer_thread,
                       NULL, sim_host_timer_handler, NULL);
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
 * Name: sim_send_ipi(int cpu)
 ****************************************************************************/

void sim_send_ipi(int cpu)
{
  pthread_kill(g_cpu_thread[cpu], SIGUSR1);
}
