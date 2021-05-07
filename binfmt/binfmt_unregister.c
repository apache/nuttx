/****************************************************************************
 * binfmt/binfmt_unregister.c
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

#include <string.h>
#include <sched.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/binfmt/binfmt.h>

#include "binfmt.h"

#ifndef CONFIG_BINFMT_DISABLE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: unregister_binfmt
 *
 * Description:
 *   Unregister a loader for a binary format
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int unregister_binfmt(FAR struct binfmt_s *binfmt)
{
  FAR struct binfmt_s *curr;
  FAR struct binfmt_s *prev;
  int ret = -EINVAL;

  if (binfmt)
    {
      /* Disabling pre-emption should be sufficient protection while
       * accessing the list of registered binary format handlers.
       */

      sched_lock();

      /* Search the list of registered binary format handlers for the
       * one to be unregistered.
       */

      for (prev = NULL, curr = g_binfmts;
           curr && curr != binfmt;
           prev = curr, curr = curr->next);

      /* Was it in the list? */

      if (curr)
        {
          /* Yes.. was it at the head of the list? */

          if (!prev)
            {
              /* Yes.. remove it from the head of the list */

              g_binfmts = binfmt->next;
            }
          else
            {
              /* No.. remove it from the middle/end of the list */

              prev->next = binfmt->next;
            }

          binfmt->next = NULL;
          ret = OK;
        }

      sched_unlock();
    }

  return ret;
}

#endif /* CONFIG_BINFMT_DISABLE */
