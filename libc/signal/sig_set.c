/****************************************************************************
 * libc/signal/sig_set.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigset
 *
 * Description:
 *   The sigset() function will modify signal dispositions. The 'signo'
 *   argument specifies the signal. The 'disp' argument specifies the
 *   signal's disposition, which may be SIG_DFL, SIG_IGN, or the address
 *   of a signal handler. If 'disp' is the address of a signal handler, the
 *   system will add 'signo' to the calling process' signal mask before
 *   executing the signal handler; when the signal handler returns, the
 *   system will restore the calling process' signal mask to its state prior
 *   to the delivery of the signal.  'signo' will be removed from the calling
 *   process' signal mask.
 *
 *   NOTE:  The value SIG_HOLD for 'disp' is not supported.  It should work
 *   like this:  If 'disp' is equal to SIG_HOLD, 'signo' will be added to,
 *   not removed from, the calling process' signal mask and 'signo''s
 *   disposition will remain unchanged.
 *
 * Input Parameters:
 *   signo - Identifies the signal to operate on
 *   disp  - The new disposition of the signal
 *
 * Returned Value:
 *   Upon successful completion, sigset() will the previous disposition of
 *   the signal. Otherwise, SIG_ERR will be returned and errno set to
 *   indicate the error.
 *
 *   NOTE: sigset() would return SIG_HOLD if the signal had been blocked and
 *   the signal's previous disposition if it had not been blocked.
 *
 ****************************************************************************/

void (*sigset(int signo, void (*disp)(int)))(int)
{
  struct sigaction act;
  struct sigaction oact;
  int ret;

  /* Initialize the sigaction structure */

  act.sa_handler = disp;
  act.sa_flags   = 0;
  (void)sigemptyset(&act.sa_mask);

  /* Check for SIG_IGN and SIG_DFL (and someday SIG_HOLD)
   *
   * REVISIT:  Currently SIG_IGN, SIG_DFL, and SIG_HOLD have the same value
   * and cannot be distinguished.
   */

  if (disp != SIG_DFL /* && disp != SIG_IGN */)
    {
      /* Add the signal to the set of signals to be ignored when the signal
       * handler executes.
       */

      ret = sigaddset(&act.sa_mask, signo);
      if (ret < 0)
        {
          /* Would happen if signo were invalid */

          return SIG_ERR;
        }
    }

  /* Set the signal disposition */

  ret = sigaction(signo, &act, &oact);

  /* Upon successful completion, sigset() will the signal's previous
   * disposition. Otherwise, SIG_ERR will be returned and errno set to
   * indicate the error.
   */

 if (ret == OK)
   {
     return oact.sa_handler;
   }

  return SIG_ERR;
}
