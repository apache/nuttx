/****************************************************************************
 * libs/libc/signal/sig_raise.c
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

#include <signal.h>
#include <pthread.h>
#include <assert.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sig_raise
 *
 * Description:
 *   The raise() function sends the signal signo to the executing thread or
 *   process. If a signal handler is called, the raise() function does not
 *   return until after the signal handler does.
 *
 *   If the implementation supports the Threads option, the effect of the
 *   raise() function is equivalent to calling:
 *
 *     pthread_kill(pthread_self(), signo);
 *
 *   except that on failures, -1 (ERROR) is returned and the errno() variable
 *   is set accordingly.  Otherwise, the effect of the raise() function is
 *   equivalent to calling:
 *
 *     kill(getpid(), signo)
 *
 ****************************************************************************/

int raise(int signo)
{
#ifndef CONFIG_DISABLE_PTHREAD
  int errcode = pthread_kill(pthread_self(), signo);
  if (errcode != OK)
    {
      DEBUGASSERT(errcode > 0);
      set_errno(errcode);
      return ERROR;
    }

  return OK;

#else
  return kill(getpid(), signo);
#endif
}
