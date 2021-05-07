/****************************************************************************
 * libs/libc/signal/sig_raise.c
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
