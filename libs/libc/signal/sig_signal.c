/****************************************************************************
 * libs/libc/signal/sig_signal.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: signal
 *
 * Description:
 *   The signal() function will modify signal dispositions. The 'signo'
 *   argument specifies the signal. The 'func' argument specifies the
 *   signal's disposition, which may be SIG_DFL, SIG_IGN, or the address
 *   of a signal handler.  If 'func' is the address of a signal handler, the
 *   system will add 'signo' to the calling process' signal mask before
 *   executing the signal handler; when the signal handler returns, the
 *   system will restore the calling process' signal mask to its state prior
 *   to the delivery of the signal.
 *
 * Input Parameters:
 *   signo - Identifies the signal to operate on
 *   func  - The new disposition of the signal
 *
 * Returned Value:
 *   Upon successful completion, signal() will return the previous
 *   disposition of the signal handling. Otherwise, SIG_ERR will be returned
 *   and errno set to indicate the nature of the error.
 *
 ****************************************************************************/

_sa_handler_t signal(int signo, _sa_handler_t func)
{
  struct sigaction act;
  struct sigaction oact;
  int ret;

  DEBUGASSERT(GOOD_SIGNO(signo) && func != SIG_ERR && func != SIG_HOLD);

  /* Initialize the sigaction structure */

  act.sa_handler = func;
  act.sa_flags   = 0;
  sigemptyset(&act.sa_mask);

  /* Check for SIG_IGN and SIG_DFL (and someday SIG_HOLD)
   *
   * REVISIT:  Currently SIG_IGN, SIG_DFL, and SIG_HOLD have the same value
   * and cannot be distinguished.
   */

  if (func != SIG_DFL /* && func != SIG_IGN */)
    {
      /* Add the signal to the set of signals to be ignored when the signal
       * handler executes.
       */

      ret = sigaddset(&act.sa_mask, signo);
      if (ret < 0)
        {
          /* Would happen if signo were invalid */

          return (_sa_handler_t)SIG_ERR;
        }
    }

  /* Set the signal disposition */

  ret = sigaction(signo, &act, &oact);

  /* Upon successful completion, signal() will the signal's previous
   * disposition. Otherwise, SIG_ERR will be returned and errno set to
   * indicate the error.
   */

  if (ret == OK)
    {
      return oact.sa_handler;
    }

  return (_sa_handler_t)SIG_ERR;
}
