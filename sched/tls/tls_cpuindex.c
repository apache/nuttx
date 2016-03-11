/****************************************************************************
 * libc/fixedmath/tls_cpuindex.c
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

#include <nuttx/config.h>

#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/tls.h>

#if defined(CONFIG_TLS) && defined(CONFIG_SMP)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_cpu_index
 *
 * Description:
 *   Return an index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.  This is index is retained
 *   in the task TCB which is accessible via the tls_info_s structure.
 *
 *   There is a race condition in that this thread could be swapped out and
 *   be running on a different CPU when the function returns.  If that is a
 *   problem, then the calling function should disable pre-emption before
 *   calling this function.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, an integer index is returned in the range of 0 through
 *   (CONFIG_SMP_NCPUS-1) that corresponds to the currently executing CPU.
 *
 ****************************************************************************/

int tls_cpu_index(void)
{
  FAR struct tls_info_s *info;
  FAR struct tcb_s *tcb;

  /* Get the TLS info structure from the current threads stack */

  info = up_tls_info();
  DEBUGASSERT(info != NULL && info->tl_tcb != NULL);

  /* Get the TCB from the TLS info.  We expect the TCB state to indicate that
   * the task is running (it must be because it is this thread).
   */

  tcb = info->tl_tcb;
  DEBUGASSERT(tcb->task_state == TSTATE_TASK_RUNNING &&
              tcb->cpu <= (CONFIG_SMP_NCPUS-1));

  return tcb->cpu;
}

#endif /* CONFIG_TLS && CONFIG_SMP */
