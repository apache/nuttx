/****************************************************************************
 * sched/sched/sched_thistask.c
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

#include <sys/types.h>
#include <arch/irq.h>

#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "sched/sched.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: this_task
 *
 * Description:
 *   The functions will safely obtain the TCB that is currently running
 *   on the current CPU. In SMP, this must be done by disabling local
 *   interrupts to avoid CPU switching during access to current_task()
 *
 * Returned Value:
 *   the TCB that is currently running on the current CPU.
 *
 ****************************************************************************/

FAR struct tcb_s *this_task(void)
{
  FAR struct tcb_s *tcb;
  irqstate_t flags;

  /* If the CPU supports suppression of interprocessor interrupts, then
   * simple disabling interrupts will provide sufficient protection for
   * the following operations.
   */

  flags = up_irq_save();

  /* Obtain the TCB which is currently running on this CPU */

  tcb = current_task(this_cpu());

  /* Enable local interrupts */

  up_irq_restore(flags);
  return tcb;
}

#endif /* CONFIG_SMP */
