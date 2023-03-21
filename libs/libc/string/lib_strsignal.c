/****************************************************************************
 * libs/libc/string/lib_strsignal.c
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

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIBC_STRSIGNAL_SHORT
#  define CASE_SIG_STR(sig, msg) \
    case (sig): \
      return (FAR char *)#sig
#else
#  define CASE_SIG_STR(sig, msg) \
    case (sig): \
      return (FAR char *)(msg)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strsignal
 *
 * Description:
 *   The strsignal() function will map the signal number in signum to an
 *   implementation-defined string and will return a pointer to it.
 *
 ****************************************************************************/

FAR char *strsignal(int signum)
{
  static char sigstr[32];

  /* Handle invalid signals */

  if (!GOOD_SIGNO(signum))
    {
      return (FAR char *)"Invalid Signal";
    }

  /* Handle named signals */

  switch (signum)
    {
#ifdef CONFIG_LIBC_STRSIGNAL
      /* Standard signals */

      CASE_SIG_STR(SIGHUP,    "Hangup");
      CASE_SIG_STR(SIGINT,    "Interrupt");
      CASE_SIG_STR(SIGQUIT,   "Quit");
      CASE_SIG_STR(SIGILL,    "Illegal instruction");
      CASE_SIG_STR(SIGTRAP,   "Trace/breakpoint trap");
      CASE_SIG_STR(SIGABRT,   "Aborted");
      CASE_SIG_STR(SIGBUS,    "Bus error");
      CASE_SIG_STR(SIGFPE,    "Arithmetic exception");
      CASE_SIG_STR(SIGKILL,   "Killed");
      CASE_SIG_STR(SIGUSR1,   "User defined signal 1");
      CASE_SIG_STR(SIGSEGV,   "Invalid memory reference");
      CASE_SIG_STR(SIGUSR2,   "User defined signal 2");
      CASE_SIG_STR(SIGPIPE,   "Broken pipe");
      CASE_SIG_STR(SIGALRM,   "Alarm clock");
      CASE_SIG_STR(SIGTERM,   "Terminated");
      CASE_SIG_STR(SIGCHLD,   "Child status changed");
      CASE_SIG_STR(SIGCONT,   "Continued");
      CASE_SIG_STR(SIGSTOP,   "Stopped (signal)");
      CASE_SIG_STR(SIGTSTP,   "Stopped");
      CASE_SIG_STR(SIGTTIN,   "Stopped (tty input)");
      CASE_SIG_STR(SIGTTOU,   "Stopped (tty output)");
      CASE_SIG_STR(SIGURG,    "Urgent I/O condition");
      CASE_SIG_STR(SIGXCPU,   "CPU time limit exceeded");
      CASE_SIG_STR(SIGXFSZ,   "File size limit exceeded");
      CASE_SIG_STR(SIGVTALRM, "Virtual timer expired");
      CASE_SIG_STR(SIGPROF,   "Profiling timer expired");
      CASE_SIG_STR(SIGPOLL,   "Pollable event occurred");
      CASE_SIG_STR(SIGSYS,    "Bad system call");

#endif /* CONFIG_LIBC_STRSIGNAL */

    default:
      if (signum >= SIGRTMIN && signum <= SIGRTMAX)
        {
          snprintf(sigstr, sizeof(sigstr), "Real-time Signal %d",
                   signum - SIGRTMIN);
        }
      else
        {
          snprintf(sigstr, sizeof(sigstr), "Signal %d", signum);
        }
      break;
    }

  /* Return a string devoid is meaning */

  return sigstr;
}
