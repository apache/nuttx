/****************************************************************************
 * libs/libc/signal/sig_psignal.c
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

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

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
  /* No stderr!  Write to whatever alternative console is available */

  if (message != NULL)
    {
      printf("%s: %s\n", message, strsignal(signum));
    }
  else
    {
      printf("%s\n", strsignal(signum));
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
