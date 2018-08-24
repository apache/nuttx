/****************************************************************************
 * nuttx/graphics/nxterm/nxterm_sem.c
 *
 *   Copyright (C) 2012, 2017 Gregory Nutt. All rights reserved.
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

#include <unistd.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>

#include "nxterm.h"

#ifdef CONFIG_DEBUG_FEATURES

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_semwait and nxterm_sempost
 *
 * Description:
 *   If debug is on, then lower level code may attempt console output while
 *   we are doing console output!  In this case, we will toss the nested
 *   output to avoid deadlocks and infinite loops.
 *
 * Input Parameters:
 *   priv - Driver data structure
 *
 * Returned Value:
 *
 *
 ****************************************************************************/

int nxterm_semwait(FAR struct nxterm_state_s *priv)
{
  pid_t me;
  int ret;

  /* Does I already hold the semaphore */

  me = getpid();
  if (priv->holder != me)
    {
      /* No.. then wait until the thread that does hold it is finished with it */

      ret = nxsem_wait(&priv->exclsem);
      if (ret == OK)
        {
          /* No I hold the semaphore */

          priv->holder = me;
        }

      return ret;
    }

  /* Abort, abort, abort!  I have been re-entered */

  return -EBUSY;
}

int nxterm_sempost(FAR struct nxterm_state_s *priv)
{
  pid_t me = getpid();

  /* Make sure that I really hold the semaphore */

  DEBUGASSERT(priv->holder == me);

  /* Then let go of it */

  priv->holder = NO_HOLDER;
  return nxsem_post(&priv->exclsem);
}

#endif /* CONFIG_DEBUG_FEATURES */
