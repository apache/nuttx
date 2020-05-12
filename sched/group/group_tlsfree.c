/****************************************************************************
 * sched/group/group_tlsfree.c
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

#include <nuttx/irq.h>
#include <nuttx/tls.h>

#include "sched/sched.h"
#include "group/group.h"

#if CONFIG_TLS_NELEM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_free
 *
 * Description:
 *   Release a group-unique TLS data index previous obtained by tls_alloc()
 *
 * Input Parameters:
 *   tlsindex - The previously allocated TLS index to be freed
 *
 * Returned Value:
 *   OK is returned on success; a negated errno value will be returned on
 *   failure:
 *
 *     -EINVAL - the index to be freed is out of range.
 *
 ****************************************************************************/

int tls_free(int tlsindex)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct task_group_s *group = rtcb->group;
  tls_ndxset_t mask;
  irqstate_t flags;
  int ret = -EINVAL;

  DEBUGASSERT((unsigned)tlsindex < CONFIG_TLS_NELEM && group != NULL);
  if ((unsigned)tlsindex < CONFIG_TLS_NELEM)
    {
      /* This is done in a critical section here to avoid concurrent
       * modification of the group TLS index set.
       */

      mask  = (1 << tlsindex);
      flags = spin_lock_irqsave();

      DEBUGASSERT((group->tg_tlsset & mask) != 0);
      group->tg_tlsset &= ~mask;
      spin_unlock_irqrestore(flags);

      ret = OK;
    }

  return ret;
}

#endif /* CONFIG_TLS_NELEM > 0 */
