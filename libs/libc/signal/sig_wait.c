/****************************************************************************
 * libs/libc/signal/sig_wait.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <signal.h>
#include <assert.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigwait
 *
 * Description:
 *   The sigwait() function selects a pending signal from set, atomically
 *   clears it from the system's set of pending signals, and returns that
 *   signal number in the location referenced by sig.  If prior to the call
 *   to sigwait() there are multiple pending instances of a single signal
 *   number, it is implementation-dependent whether upon successful return
 *   there are any remaining pending signals for that signal number. If the
 *   implementation supports queued signals and there are multiple signals
 *   queued for the signal number selected, the first such queued signal
 *   causes a return from sigwait() and the remainder remain queued. If no
 *   signal in set is pending at the time of the call, the thread is
 *   suspended until one or more becomes pending. The signals defined by set
 *   will been blocked at the time of the call to sigwait(); otherwise the
 *   behavior is undefined. The effect of sigwait() on the signal actions
 *   for the signals in set is unspecified.
 *
 *   If more than one thread is using sigwait() to wait for the same
 *   signal, no more than one of these threads will return from sigwait()
 *   with the signal number. Which thread returns from sigwait() if more
 *   than a single thread is waiting is unspecified.
 *
 *   Should any of the multiple pending signals in the range SIGRTMIN to
 *   SIGRTMAX be selected, it shall be the lowest numbered one. The selection
 *   order between realtime and non-realtime signals, or between multiple
 *   pending non-realtime signals, is unspecified.
 *
 * Input Parameters:
 *   set  - The set of pending signals to wait for
 *   sig  - The location in which to return the pending signal number.
 *
 * Returned Value:
 *   Upon successful completion, sigwait() stores the signal number of the
 *   received signal at the location referenced by sig and returns zero.
 *   Otherwise, an error number is returned to indicate the error.
 *
 ****************************************************************************/

int sigwait(FAR const sigset_t *set, FAR int *sig)
{
  int signo;

  DEBUGASSERT(set != NULL && sig != NULL);

  /* The standard sigwait() function behaves that same as sigwaitinfo() with
   * the info argument set to NULL.
   */

  signo = sigwaitinfo(set, NULL);
  if (signo < 0)
    {
      /* If sigwaitinfo() fails, return the error number */

      return get_errno();
    }

  /* Return the signal number in the user provided location */

  *sig = signo;
  return OK;
}
