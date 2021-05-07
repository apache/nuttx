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

#include <signal.h>
#include <string.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* We don't know what signals names will be assigned to which signals in
 * advance and we do not want to return a volatile value.  One solution is
 * this silly array of useless names:
 */

static FAR const char *g_default_sigstr[32] =
{
  "Signal 0",
  "Signal 1",
  "Signal 2",
  "Signal 3",
  "Signal 4",
  "Signal 5",
  "Signal 6",
  "Signal 7",
  "Signal 8",
  "Signal 9",
  "Signal 10",
  "Signal 11",
  "Signal 12",
  "Signal 13",
  "Signal 14",
  "Signal 15",
  "Signal 16",
  "Signal 17",
  "Signal 18",
  "Signal 19",
  "Signal 20",
  "Signal 21",
  "Signal 22",
  "Signal 23",
  "Signal 24",
  "Signal 25",
  "Signal 26",
  "Signal 27",
  "Signal 28",
  "Signal 29",
  "Signal 30",
  "Signal 31",
};

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
  /* Handle invalid signals */

  if (!GOOD_SIGNO(signum))
    {
      return (FAR char *)"Invalid Signal";
    }

  /* Handle named signals */

  switch (signum)
    {
      /* Standard signals */

#ifdef SIGUSR1
      case SIGUSR1:
        return (FAR char *)"SIGUSR1";
#endif

#ifdef SIGUSR2
      case SIGUSR2:
        return (FAR char *)"SIGUSR2";
#endif

#ifdef SIGALRM
      case SIGALRM:
        return (FAR char *)"SIGALRM";
#endif

#ifdef SIGCHLD
      case SIGCHLD:
        return (FAR char *)"SIGCHLD";
#endif

#ifdef SIGPOLL
      case SIGPOLL:
        return (FAR char *)"SIGPOLL";
#endif

#ifdef SIGSTOP
      case SIGSTOP:
        return (FAR char *)"SIGSTOP";
#endif

#ifdef SIGTSTP
      case SIGTSTP:
        return (FAR char *)"SIGTSTP";
#endif

#ifdef SIGCONT
      case SIGCONT:
        return (FAR char *)"SIGCONT";
#endif

#ifdef SIGKILL
      case SIGKILL:
        return (FAR char *)"SIGKILL";
#endif

#ifdef SIGINT
      case SIGINT:
        return (FAR char *)"SIGINT";
#endif

#ifdef SIGQUIT
      case SIGQUIT:
        return (FAR char *)"SIGQUIT";
#endif

#ifdef SIGTERM
      case SIGTERM:
        return (FAR char *)"SIGTERM";
#endif

      /* Non-standard signals */

#ifdef SIGCONDTIMEDOUT
      case SIGCONDTIMEDOUT:
        return (FAR char *)"SIGCONDTIMEDOUT";
#endif

#ifdef SIGWORK
      case SIGWORK:
        return (FAR char *)"SIGWORK";
#endif

      default:
        break;
    }

  /* Return a string devoid is meaning */

  return (FAR char *)g_default_sigstr[signum];
}
