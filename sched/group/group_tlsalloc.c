/****************************************************************************
 * sched/group/group_tlsalloc.c
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

#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/tls.h>

#include "sched/sched.h"
#include "group/group.h"

#if CONFIG_TLS_NELEM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_alloc
 *
 * Description:
 *   Allocate a group-unique TLS data index
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A TLS index that is unique for use within this task group.
 *
 ****************************************************************************/

int tls_alloc(void)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct task_group_s *group = rtcb->group;
  irqstate_t flags;
  int candidate;
  int ret = -EAGAIN;

  DEBUGASSERT(group != NULL);

  /* Search for an unused index.  This is done in a critical section here to
   * avoid concurrent modification of the group TLS index set.
   */

  flags = spin_lock_irqsave();
  for (candidate = 0; candidate < CONFIG_TLS_NELEM; candidate++)
    {
      /* Is this candidate index available? */

      tls_ndxset_t mask = (1 << candidate);
      if ((group->tg_tlsset & mask) == 0)
        {
          /* Yes.. allocate the index and break out of the loop */

          group->tg_tlsset |= mask;
          break;
        }
    }

  spin_unlock_irqrestore(flags);

  /* Check if found a valid TLS data index. */

  if (candidate < CONFIG_TLS_NELEM)
    {
      /* Yes.. Return the TLS index and success */

      ret = candidate;
    }

  return ret;
}

#endif /* CONFIG_TLS_NELEM > 0 */
