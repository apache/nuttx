/****************************************************************************
 * libs/libc/string/lib_strsignal.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifdef SIGSTP
      case SIGSTP:
        return (FAR char *)"SIGSTP";
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
