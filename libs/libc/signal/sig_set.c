/****************************************************************************
 * libs/libc/signal/sig_set.c
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

#include <signal.h>
#include <assert.h>
#include <errno.h>

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
 *   signal() function except that it includes additional management of the
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
 *          if (mask successfully set)
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
 *          if (disposition successfully set)
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
  int ret = -EINVAL;

  if (signo == SIGKILL || signo == SIGSTOP || !GOOD_SIGNO(signo))
    {
      goto err;
    }

  DEBUGASSERT(func != SIG_ERR);

  sigemptyset(&set);
  sigaddset(&set, signo);

  /* Check if we are being asked to block the signal */

  if (func == SIG_HOLD)
    {
      ret = sigprocmask(SIG_BLOCK, &set, NULL);
      if (ret < 0)
        {
          goto err;
        }

      disposition = SIG_HOLD;
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

              signal(signo, disposition);
              goto err;
            }
        }
    }

  return disposition;
err:
  set_errno(-ret);
  return (_sa_handler_t)SIG_ERR;
}
