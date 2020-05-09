/****************************************************************************
 * sched/sched/sched_foreach.c
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

#include <sched.h>

#include <nuttx/irq.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_foreach
 *
 * Description:
 *   Enumerate over each task and provide the TCB of each task to a user
 *   callback functions.
 *
 *   NOTE:  This function examines the TCB and calls each handler within a
 *   critical section.  However, that critical section is released and
 *   reacquired for each TCB.  When it is released, there may be changes in
 *   tasking.  If the caller requires absolute stability through the
 *   traversal, then the caller should establish the critical section BEFORE
 *   calling this function.
 *
 * Input Parameters:
 *   handler - The function to be called with the TCB of
 *     each task
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_foreach(nxsched_foreach_t handler, FAR void *arg)
{
  irqstate_t flags;
  int ndx;

  /* Visit each active task */

  for (ndx = 0; ndx < CONFIG_MAX_TASKS; ndx++)
    {
      /* This test and the function call must be atomic */

      flags = enter_critical_section();
      if (g_pidhash[ndx].tcb)
        {
          handler(g_pidhash[ndx].tcb, arg);
        }

      leave_critical_section(flags);
    }
}
