/****************************************************************************
 * arch/sim/src/up_simsmp.c
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
#include <semaphore.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef int (*main_t)(int argc, char **argv);

struct sim_cpuinfo_s
{
  int cpu;                /* CPU number */
  pthread_mutex_t mutex;  /* For synchronization */
  main_t idletask;        /* IDLE task entry point */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_key_t g_cpukey;

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
 *   arg - Stanard pthread argument
 *
 * Returned Value:
 *   This function does not return
 *
 ****************************************************************************/

static void *sim_idle_trampoline(void *arg)
{
  struct sim_cpuinfo_s *cpuinfo = (struct sim_cpuinfo_s *)arg;
  int ret;

  /* Set the CPU number zero for the CPU thread */

  ret = pthread_setspecific(g_cpukey, (const void *)((uintptr_t)cpuinfo->cpu));
  if (ret != 0)
    {
      return NULL;
    }

  /* Let up_cpustart() continue */

  (void)pthread_mutex_unlock(&cpuinfo->mutex);

  /* Give control to the IDLE task */

  (void)cpuinfo->idletask(0, (char **)0);

  /* The IDLE task will not return.  This is just to keep the compiler happy */

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_cpu0initialize
 *
 * Description:
 *   Create the pthread-specific data key and set the indication of CPU0
 *   the the main thread.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 ****************************************************************************/

int sim_cpu0initialize(void)
{
  int ret;

  /* Create the pthread key */

  ret = pthread_key_create(&g_cpukey, NULL);
  if (ret != 0)
    {
      return -ret;
    }

  /* Set the CPU number zero for the main thread */

  ret = pthread_setspecific(g_cpukey, (const void *)0);
  if (ret != 0)
    {
      return -ret;
    }

  return 0;
}

/****************************************************************************
 * Name: up_cpundx
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

int up_cpundx(void)
{
  void *value = pthread_getspecific(g_cpukey);
  return (int)((uintptr_t)value);
}

/****************************************************************************
 * Name: up_cpustart
 *
 * Description:
 *   In an SMP configution, only one CPU is initially active (CPU 0). System
 *   initialization occurs on that single thread. At the completion of the
 *   initialization of the OS, just before beginning normal multitasking,
 *   the additional CPUs would be started by calling this function.
 *
 *   Each CPU is provided the entry point to is IDLE task when started.  A
 *   TCB for each CPU's IDLE task has been initialized and placed in the
 *   CPU's g_assignedtasks[cpu] list.  Not stack has been alloced or
 *   initialized.
 *
 *   The OS initialization logic calls this function repeatedly until each
 *   CPU has been started, 1 through (CONFIG_SMP_NCPUS-1).
 *
 * Input Parameters:
 *   cpu - The index of the CPU being started.  This will be a numeric
 *         value in the range of from one to (CONFIG_SMP_NCPUS-1).  (CPU
 *         0 is already active)
 *   idletask - The entry point to the IDLE task.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpustart(int cpu, main_t idletask)
{
  struct sim_cpuinfo_s cpuinfo;
  pthread_t thread;
  int ret;

  /* Initialize the CPU info */

  cpuinfo.cpu = cpu;
  cpuinfo.idletask = idletask;
  ret = pthread_mutex_init(&cpuinfo.mutex, NULL);
  if (ret != 0)
    {
      return -ret;  /* REVISIT:  That is a host errno value. */
    }

  /* Lock the mutex */

  ret = pthread_mutex_lock(&cpuinfo.mutex);
  if (ret != 0)
    {
      ret = -ret;  /* REVISIT: This is a host errno value. */
      goto errout_with_mutex;
    }

  /* Start the CPU emulation thread.  This is analogous to starting the CPU
   * in a multi-CPU hardware model.
   */

  ret = pthread_create(&thread, NULL, sim_idle_trampoline, &cpuinfo);
  if (ret != 0)
    {
      ret = -ret;  /* REVISIT:  That is a host errno value. */
      goto errout_with_lock;
    }

  /* Try to lock the mutex again.  This will block until the pthread unlocks
   * the mutex.
   */

  ret = pthread_mutex_lock(&cpuinfo.mutex);
  if (ret != 0)
    {
      ret = -ret;  /* REVISIT:  That is a host errno value. */
    }

errout_with_lock:
  (void)pthread_mutex_unlock(&cpuinfo.mutex);

errout_with_mutex:
  (void)pthread_mutex_destroy(&cpuinfo.mutex);
  return ret;
}

/****************************************************************************
 * Name: up_cpustop
 *
 * Description:
 *   Save the state of the current task at the head of the
 *   g_assignedtasks[cpu] task list and then stop the CPU.
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

int up_cpustop(int cpu)
{
#warning Missing SMP logic
  return 0;
}

/****************************************************************************
 * Name: up_cpurestart
 *
 * Description:
 *   Restart the cpu, restoring the state of the task at the head of the
 *   g_assignedtasks[cpu] list.
 *
 *   This function is called after up_cpustop in order resume operation of
 *   the CPU after modifying its g_assignedtasks[cpu] list.
 *
 * Input Parameters:
 *   cpu - The index of the CPU being re-started.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpurestart(int cpu)
{
#warning Missing SMP logic
  return 0;
}
