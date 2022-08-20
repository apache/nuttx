/****************************************************************************
 * mm/mm_check/mm_check.c
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

#include <nuttx/mm/mm.h>
#include <nuttx/sched.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/config.h>

#include <time.h>
#include <assert.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_LPWORK
#error "Low priority work queue is required for the memory health checks."
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

static struct work_s work_q;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mm_check_worker(FAR void * arg)
{
  UNUSED(arg);

  int i;
  FAR struct tcb_s *tcb;
  irqstate_t flags;
  extern FAR struct tcb_s **g_pidhash;
  extern volatile int g_npidhash;

  for (i = 0; i < g_npidhash; i++)
    {
      flags = enter_critical_section();

      tcb = g_pidhash[i];

      if (tcb && ((up_check_tcbstack(tcb) * 100 / tcb->adj_stack_size)
          > CONFIG_MM_STACK_USAGE_SAFE_PERCENT))
        {
          PANIC();
        }

      leave_critical_section(flags);
    }

  kmm_checkcorruption();

  work_queue(LPWORK, &work_q, mm_check_worker, NULL,
             (CONFIG_MM_CHECK_PERIOD * CLOCKS_PER_SEC));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mm_check_init()
{
  work_queue(LPWORK, &work_q, mm_check_worker, NULL,
             (CONFIG_MM_CHECK_PERIOD * CLOCKS_PER_SEC));
}
