/****************************************************************************
 * arch/x86_64/src/common/x86_64_tlb.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/irq.h>
#include <sched.h>

#include "sched/sched.h"

#include "x86_64_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_tlb_handler
 *
 * Description:
 *   Reload CR3 to invalidate the TLB.
 *
 ****************************************************************************/

int x86_64_tlb_handler(int irq, void *c, void *arg)
{
  volatile uint64_t cr3 = get_cr3();

  set_cr3(cr3);

  UNUSED(irq);
  UNUSED(c);
  UNUSED(arg);

  return OK;
}

/****************************************************************************
 * Name: x86_64_tlb_shootdown
 ****************************************************************************/

void x86_64_tlb_shootdown(void)
{
  cpu_set_t cpuset = ((1 << CONFIG_SMP_NCPUS) - 1);

  CPU_CLR(this_cpu(), &cpuset);

  up_trigger_irq(SMP_IPI_TLBSHOOTDOWN_IRQ, cpuset);
}
