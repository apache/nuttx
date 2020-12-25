/****************************************************************************
 * sched/pthread/pthread_sigmask.c
 *
 *   Copyright (C) 2007, 2009, 2017 Gregory Nutt. All rights reserved.
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
#include <errno.h>
#include <debug.h>

#include <nuttx/signal.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_sigmask
 *
 * Description:
 *   This function is a simple wrapper around nxsig_procmask().
 *   See the nxsig_procmask() function description for further
 *   information.
 *
 * Input Parameters:
 *   how - How the signal mask will be changed:
 *         SIG_BLOCK   - The resulting set is the union of
 *                       the current set and the signal set
 *                       pointed to by 'set'.
 *         SIG_UNBLOCK - The resulting set is the intersection
 *                       of the current set and the complement
 *                       of the signal set pointed to by 'set'.
 *         SIG_SETMASK - The resulting set is the signal set
 *                       pointed to by 'set'.
 *   set - Location of the new signal mask
 *   oset - Location to store the old signal mask
 *
 * Returned Value:
 *   On success, this function will return 0 (OK).  It will return EINVAL if
 *   how is invalid.
 *
 ****************************************************************************/

int pthread_sigmask(int how, FAR const sigset_t *set, FAR sigset_t *oset)
{
  int ret = nxsig_procmask(how, set, oset);
  return ret < 0 ? -ret : OK;
}
