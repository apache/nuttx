/****************************************************************************
 * arch/sim/src/sim/up_hostirq.c
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
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "up_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile void *g_current_regs[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Private Types
 ****************************************************************************/

union sigset_u
{
  uint64_t flags;
  sigset_t sigset;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_handle_irq
 ****************************************************************************/

static void up_handle_irq(int irq, siginfo_t *info, void *context)
{
  up_doirq(irq, context);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Disable interrupts and returned the mask before disabling them.
 *
 ****************************************************************************/

uint64_t up_irq_save(void)
{
  union sigset_u nmask;
  union sigset_u omask;

  sigfillset(&nmask.sigset);
  pthread_sigmask(SIG_SETMASK, &nmask.sigset, &omask.sigset);

  return omask.flags;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Input Parameters:
 *   flags - the mask used to restore interrupts
 *
 * Description:
 *   Re-enable interrupts using the specified mask in flags argument.
 *
 ****************************************************************************/

void up_irq_restore(uint64_t flags)
{
  union sigset_u nmask;

  sigemptyset(&nmask.sigset);
  nmask.flags = flags;
  pthread_sigmask(SIG_SETMASK, &nmask.sigset, NULL);
}

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
#ifdef CONFIG_SMP
  /* Register the pause handler */

  up_init_ipi(SIGUSR1);
#endif
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  struct sigaction act;
  sigset_t set;

  /* Register signal handler */

  memset(&act, 0, sizeof(act));
  act.sa_sigaction = up_handle_irq;
  act.sa_flags     = SA_SIGINFO;
  sigfillset(&act.sa_mask);
  sigaction(irq, &act, NULL);

  /* Unmask the signal */

  sigemptyset(&set);
  sigaddset(&set, irq);
  pthread_sigmask(SIG_UNBLOCK, &set, NULL);
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  /* Since it's hard to mask the signal on all threads,
   * let's change the signal handler to ignore instead.
   */

  signal(irq, SIG_IGN);
}
