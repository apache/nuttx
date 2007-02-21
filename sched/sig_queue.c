/************************************************************
 * sig_queue.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <signal.h>
#include <debug.h>
#include <sched.h>
#include "os_internal.h"
#include "sig_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Function: sigqueue
 *
 * Description:
 *   This function sends the signal specified by signo with
 *   the signal parameter value to the process specified by
 *   pid.
 *
 *   If the receiving process has the signal blocked via the
 *   sigprocmask, the signal will pend until it is unmasked.
 *   Only one pending signal (per signo) is retained.  This
 *   is consistent with POSIX which states, "If a subsequent
 *   occurrence of a pending signal is generated, it is
 *   implementation defined as to whether the signal is
 *   delivered more than once."
 *
 * Parameters:
 *   pid - Process ID of task to receive signal
 *   signo - Signal number
 *   value - Value to pass to task with signal
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ************************************************************/

#ifdef CONFIG_CAN_PASS_STRUCTS
int sigqueue (int pid, int signo, const union sigval value)
#else
int sigqueue(int pid, int signo, void *sival_ptr)
#endif
{
  _TCB     *stcb;
  siginfo_t info;
  int       ret = ERROR;

  sched_lock();

  /* Get the TCB of the receiving task */

  stcb = sched_gettcb(pid);
#ifdef CONFIG_CAN_PASS_STRUCTS
  dbg("TCB=0x%08x signo=%d value=%d\n", stcb, signo, value.sival_int);
#else
  dbg("TCB=0x%08x signo=%d value=%p\n", stcb, signo, sival_ptr);
#endif

  /* Create the siginfo structure */

  info.si_signo = signo;
  info.si_code  = SI_QUEUE;
#ifdef CONFIG_CAN_PASS_STRUCTS
  info.si_value = value;
#else
  info.si_value.sival_ptr = sival_ptr;
#endif

  /* Verify that we can perform the signalling operation */

  if (stcb && GOOD_SIGNO(signo))
    {
      /* Process the receipt of the signal */

      ret = sig_received(stcb, &info);
    }

  sched_unlock();
  return ret;
}

