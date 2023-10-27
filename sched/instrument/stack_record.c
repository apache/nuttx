/****************************************************************************
 * sched/instrument/stack_record.c
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

#include <nuttx/instrument.h>
#include <sys/param.h>

#include "sched/sched.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void stack_record_enter(FAR void *this_fn, FAR void *call_site,
                               FAR void *arg) noinstrument_function
                               nooptimiziation_function;
static void stack_record_leave(FAR void *this_fn, FAR void *call_site,
                               FAR void *arg) noinstrument_function
                               nooptimiziation_function;

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct instrument_s g_stack_record =
{
  .enter = stack_record_enter,
  .leave = stack_record_leave
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stack_statistics
 ****************************************************************************/

static void stack_record_enter(FAR void *this_fn, FAR void *call_site,
                               FAR void *arg)
{
  FAR struct tcb_s *tcb = running_task();
  FAR void *sp = &tcb;
  size_t i;

  if (tcb == NULL || sp < tcb->stack_base_ptr ||
      sp >= tcb->stack_base_ptr + tcb->adj_stack_size)
    {
      return;
    }

  if (tcb->level < CONFIG_SCHED_STACK_RECORD)
    {
      tcb->stackrecord_sp[tcb->level] = sp;
      tcb->stackrecord_pc[tcb->level++] = this_fn;
    }
  else if (tcb->caller_deepest < ++tcb->level)
    {
      tcb->caller_deepest = tcb->level;
    }

  if (tcb->sp_deepest == NULL || sp < tcb->sp_deepest)
    {
      tcb->level_deepest = MIN(tcb->level, CONFIG_SCHED_STACK_RECORD);
      for (i = 0; i < tcb->level_deepest; i++)
        {
          tcb->stackrecord_pc_deepest[i] = tcb->stackrecord_pc[i];
          tcb->stackrecord_sp_deepest[i] = tcb->stackrecord_sp[i];
        }

      tcb->sp_deepest = sp;
    }
}

/****************************************************************************
 * Name: stackrecord_leave
 ****************************************************************************/

static void stack_record_leave(FAR void *this_fn, FAR void *call_site,
                               FAR void *arg)
{
  FAR struct tcb_s *tcb = running_task();
  FAR void *sp = &tcb;

  if (tcb == NULL || tcb->level == 0 || sp < tcb->stack_base_ptr ||
      sp >= tcb->stack_base_ptr + tcb->adj_stack_size)
    {
      return;
    }

  tcb->level--;
}
