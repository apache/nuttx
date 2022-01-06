/****************************************************************************
 * sched/sched/sched_verifytcb.c
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

#include <stdbool.h>
#include <sched.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_verify_tcb
 *
 * Description:
 *   Return true if the tcb refers to an active task; false if it is a stale
 *   TCB handle.
 *
 ****************************************************************************/

bool nxsched_verify_tcb(FAR struct tcb_s *tcb)
{
  /* Return true if the PID hashes to this TCB.  This will catch the case
   * where the task associated with the TCB has terminated (note that
   * sched_releasedtcb() will nullify the TCB field in that case).  The
   * following logic will also detect the case where the task associated
   * with the TCB has terminated and another task has been started with a
   * different TCB but with a PID hashing to the same entry.
   *
   * NOTE: In the event that the TCB has terminated, the 'tcb' parameter
   * will point at either a stale or a re-allocated memory allocation.  The
   * PID fetched by the use of the bad pointer(tcb->pid) should not cause
   * any memory faults because we do at least know that the pointer refers
   * to valid memory in the kernel address space and that the hash macro,
   * PIDHASH(), will return a valid, in-range index into the g_pidhash[]
   * table.
   *
   * REVISIT:  This logic will not, however, catch the case where the task
   * originally associated with the TCB has terminated, but a new task was
   * started reusing the same memory allocation for its TDB that was freed
   * by the terminated task.  In this case, a false positive value will be
   * returned:  The TCB is valid but does not refer to the same task as
   * before.  This case is not detectable with the limited amount of
   * information available.
   */

  irqstate_t flags;
  bool vaild;

  flags = enter_critical_section();
  vaild = tcb == g_pidhash[PIDHASH(tcb->pid)].tcb;
  leave_critical_section(flags);

  return vaild;
}
