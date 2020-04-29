/****************************************************************************
 * libs/libc/signal/sig_addset.c
 *
 *   Copyright (C) 2007, 2008, 2011 Gregory Nutt. All rights reserved.
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
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_addset
 *
 * Description:
 *   This function adds the signal specified by signo to the signal set
 *   specified by set.
 *
 * Input Parameters:
 *   set - Signal set to add signal to
 *   signo - Signal to add
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *    EINVAL - The signo argument is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int nxsig_addset(FAR sigset_t *set, int signo)
{
  /* Verify the signal */

  if (!GOOD_SIGNO(signo))
    {
      return -EINVAL;
    }
  else
    {
      /* Add the signal to the set */

      *set |= SIGNO2SET(signo);
      return OK;
    }
}

/****************************************************************************
 * Name: sigaddset
 *
 * Description:
 *   This function adds the signal specified by signo to the signal set
 *   specified by set.
 *
 * Input Parameters:
 *   set - Signal set to add signal to
 *   signo - Signal to add
 *
 * Returned Value:
 *   0 (OK), or -1 (ERROR) if the signal number is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sigaddset(FAR sigset_t *set, int signo)
{
  int ret;

  /* Let nxsig_addset do all the work. */

  ret = nxsig_addset(set, signo);
  if (ret < 0)
    {
      set_errno(EINVAL);
      ret = ERROR;
    }

  return ret;
}
