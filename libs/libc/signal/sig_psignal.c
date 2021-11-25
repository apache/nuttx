/****************************************************************************
 * libs/libc/signal/sig_psignal.c
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

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

/* Uses streams... not available to kernel code */

#ifndef __KERNEL__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psignal
 *
 * Description:
 *   The psignal() functions will write a language-dependent message
 *   associated with a signal number to the standard error stream as
 *   follows:
 *
 *     First, if message is not a null pointer and is not the empty string,
 *     the string pointed to by the message argument will be written,
 *     followed by a colon and a space.
 *
 *     Then the signal description string associated with signum will be
 *     written, followed by a newline.
 *
 * Returned Value
 *  None.  The errno value is never set in this implementation.
 *
 ****************************************************************************/

void psignal(int signum, FAR const char *message)
{
#ifdef CONFIG_FILE_STREAM
  /* For now, just a brainless write to stderr (fd == 2).  C buffered I/O is
   * used!
   */

  if (message != NULL)
    {
      fprintf(stderr, "%s: %s\n", message, strsignal(signum));
    }
  else
    {
      fprintf(stderr, "%s\n", strsignal(signum));
    }
#else
  /* No stream! Write to file handle(fd == 2) directly without buffer */

  if (message != NULL)
    {
      dprintf(STDERR_FILENO, "%s: %s\n", message, strsignal(signum));
    }
  else
    {
      dprintf(STDERR_FILENO, "%s\n", strsignal(signum));
    }
#endif
}

/****************************************************************************
 * Name: psiginfo
 *
 * Description:
 *   The psiginfo() functions will write a language-dependent message
 *   associated with a signal number to the standard error stream as
 *   follows:
 *
 *     First, if message is not a null pointer and is not the empty string,
 *     the string pointed to by the message argument will be written,
 *     followed by a colon and a space.
 *
 *     Then the signal description string associated with the signal
 *     indicated by pinfo will be written, followed by a newline.
 *
 * Returned Value
 *  None.  Since no value is returned, an application wishing to check for
 *  error situations should set errno to 0, then call psiginfo() then check
 *  errno.
 *
 ****************************************************************************/

void psiginfo(FAR const siginfo_t *pinfo, FAR const char *message)
{
  if (pinfo == NULL)
    {
      set_errno(EINVAL);
    }
  else
    {
      psignal(pinfo->si_signo, message);
    }
}

#endif /* !__KERNEL__ */
