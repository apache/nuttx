/****************************************************************************
 * sched/signal/sig_mqnotempty.c
 *
 *   Copyright (C) 2007-2009, 2013 Gregory Nutt. All rights reserved.
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
#include <nuttx/compiler.h>

#include <signal.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include "sched/sched.h"
#include "signal/signal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functionss
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sig_mqnotempty
 *
 * Description:
 *   This function is equivalent to sigqueue(), but supports the messaging
 *   system's requirement to signal a task when a message queue becomes
 *   non-empty.  It is identical to sigqueue(), except that it sets the
 *   si_code field in the siginfo structure to SI_MESGQ rather than SI_QUEUE.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_PASS_STRUCTS
int sig_mqnotempty(int pid, int signo, union sigval value)
#else
int sig_mqnotempty(int pid, int signo, void *sival_ptr)
#endif
{
#ifdef CONFIG_SCHED_HAVE_PARENT
  FAR struct tcb_s *rtcb = (FAR struct tcb_s *)g_readytorun.head;
#endif
  siginfo_t info;
  int ret;

#ifdef CONFIG_CAN_PASS_STRUCTS
  sdbg("pid=%p signo=%d value=%d\n", pid, signo, value.sival_int);
#else
  sdbg("pid=%p signo=%d sival_ptr=%p\n", pid, signo, sival_ptr);
#endif

  /* Verify that we can perform the signalling operation */

  if (!GOOD_SIGNO(signo))
    {
      return -EINVAL;
    }

  /* Create the siginfo structure */

  info.si_signo           = signo;
  info.si_code            = SI_MESGQ;
#ifdef CONFIG_CAN_PASS_STRUCTS
  info.si_value           = value;
#else
  info.si_value.sival_ptr = sival_ptr;
#endif
#ifdef CONFIG_SCHED_HAVE_PARENT
  info.si_pid             = rtcb->pid;
  info.si_status          = OK;
#endif

  /* Process the receipt of the signal */

  sched_lock();
  ret = sig_dispatch(pid, &info);
  sched_unlock();

  return ret;
}
