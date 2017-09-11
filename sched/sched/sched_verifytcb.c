/****************************************************************************
 * sched/ched_verifytcb.c
 *
 *   Copyright (C) 2009, 2017 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <stdbool.h>
#include <sched.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_verifytcb
 *
 * Description:
 *   Return true if the tcb refers to an active task; false if it is a stale
 *   TCB handle.
 *
 ****************************************************************************/

bool sched_verifytcb(FAR struct tcb_s *tcb)
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

  return tcb == g_pidhash[PIDHASH(tcb->pid)].tcb;
}
