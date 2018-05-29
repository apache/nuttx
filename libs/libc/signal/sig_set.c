/****************************************************************************
 * libs/libc/signal/sig_set.c
 *
 *   Copyright (C) 2015-2017 Gregory Nutt. All rights reserved.
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

#include <signal.h>
#include <assert.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigset
 *
 * Description:
 *   The signal() function will modify signal dispositions. The 'signo'
 *   argument specifies the signal. The 'func' argument specifies the
 *   signal's disposition, which may be SIG_DFL, SIG_IGN, or the address
 *   of a signal handler.
 *
 *   The System V sigset function is very similar to the (obsolete) POSIX
 *   signal() function except that it includes additional managment of the
 *   tasks' signal mask.  This function is then simply a wrapper around
 *   signal() with this additional signal mask logic added.
 *
 *   - The sigset set function also accepts the SIG_HOLD value for 'func':
 *     If 'func' is equal to SIG_HOLD, 'signo' will be added to the signal
 *     mask of the calling process and 'signo's disposition will remain
 *     unchanged.
 *   - If 'func' is not equal to SIG_HOLD, 'signo' will be removed from the
 *     signal mask of the calling process.
 *
 * Input Parameters:
 *   signo - Identifies the signal to operate on
 *   func  - The new disposition of the signal
 *
 * Returned Value:
 *   Upon successful completion, sigset() shall return SIG_HOLD if the
 *   signal had been blocked and the signal's previous disposition if it had
 *   not been blocked. Otherwise, SIG_ERR shall be returned and errno set to
 *   indicate the error.
 *
 *   Hmm.. this wording is not clear to me.  I assume this to mean:
 *
 *     if (func == SIG_HOLD)
 *       {
 *          Set mask
 *          if (mask successfuly set)
 *            {
 *              return SIG_HOLD
 *            }
 *          else
 *            {
 *              return SIG_ERR
 *            }
 *     else
 *       {
 *         Set disposition
 *          if (disposition successfuly set)
 *            {
 *              return old disposition
 *            }
 *          else
 *            {
 *              return SIG_ERR
 *            }
 *       }
 *
 *   But you could also argue that the English means to return SIG_HOLD in
 *   any case is the signal is blocked.  And, in that case would you set the
 *   disposition or not?  Unclear.
 *
 ****************************************************************************/

_sa_handler_t sigset(int signo, _sa_handler_t func)
{
  _sa_handler_t disposition;
  sigset_t set;
  int ret;

  DEBUGASSERT(GOOD_SIGNO(signo) && func != SIG_ERR);

  (void)sigemptyset(&set);
  (void)sigaddset(&set, signo);

  /* Check if we are being asked to block the signal */

  if (func == SIG_HOLD)
    {
      ret = sigprocmask(SIG_BLOCK, &set, NULL);
      disposition = ret < 0 ? SIG_ERR : SIG_HOLD;
    }

  /* No.. then signal can handle the other cases */

  else
    {
      /* Set the signal handler disposition */

      disposition = signal(signo, func);
      if (disposition != SIG_ERR)
        {
          /* And unblock the signal */

          ret = sigprocmask(SIG_UNBLOCK, &set, NULL);
          if (ret < 0)
            {
              /* Restore the original signal disposition and return and
               * error.
               */

              (void)signal(signo, disposition);
              disposition = SIG_ERR;
            }
        }
    }

  return disposition;
}
