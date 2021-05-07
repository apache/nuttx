/****************************************************************************
 * graphics/nxterm/nxterm_sem.c
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

#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include "nxterm.h"

#ifdef CONFIG_DEBUG_GRAPHICS

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
      /* No..
       * then wait until the thread that does hold it is finished with it
       */

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
  /* Make sure that I really hold the semaphore */

  DEBUGASSERT(priv->holder == getpid());

  /* Then let go of it */

  priv->holder = NO_HOLDER;
  return nxsem_post(&priv->exclsem);
}

#endif /* CONFIG_DEBUG_GRAPHICS */
