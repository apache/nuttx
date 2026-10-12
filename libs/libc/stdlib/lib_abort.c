/****************************************************************************
 * libs/libc/stdlib/lib_abort.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: Abort
 *
 * Description:
 *   The abort() first unblocks the SIGABRT signal, and then raises that
 *   signal for the calling process. This results in the abnormal
 *   termination of the process unless the SIGABRT signal is caught and
 *   the signal handler does not return.
 *
 *   If the abort() function causes process termination, all open
 *   streams are closed and flushed.
 *
 *   If the SIGABRT signal is ignored, or caught by a handler that
 *   returns, the abort() function will still terminate the process.
 *   It does this by restoring the default disposition for SIGABRT and
 *   then raising the signal for a second time.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *  This function does not return,
 *
 ****************************************************************************/

void abort(void)
{
#ifdef CONFIG_ENABLE_ALL_SIGNALS
  struct sigaction act;
  sigset_t set;

  /* Unblock SIGABRT and raise it: a handler can catch it */

  sigemptyset(&set);
  sigaddset(&set, SIGABRT);
  sigprocmask(SIG_UNBLOCK, &set, NULL);
  raise(SIGABRT);

  /* The handler returned, or SIGABRT is ignored: restore the default
   * action and raise SIGABRT again.
   */

  memset(&act, 0, sizeof(act));
  act.sa_handler = SIG_DFL;
  sigaction(SIGABRT, &act, NULL);
  raise(SIGABRT);
#endif

  /* SIGABRT has no default action, or signals are not enabled.  _exit()
   * closes all open files and terminates the process.
   */

  _exit(EXIT_FAILURE);
}
