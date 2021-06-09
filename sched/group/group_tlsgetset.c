/****************************************************************************
 * sched/group/group_tlsgetset.c
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

#include <stdint.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/tls.h>
#include <arch/tls.h>

#include "sched/sched.h"
#include "group/group.h"

#if CONFIG_TLS_NELEM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_get_set
 *
 * Description:
 *   Get the set map of TLE element index.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   TLS element index set map.
 *
 ****************************************************************************/

tls_ndxset_t tls_get_set(void)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct task_group_s *group = rtcb->group;
  irqstate_t flags;
  tls_ndxset_t tlsset;

  DEBUGASSERT(group != NULL);

  flags = spin_lock_irqsave(NULL);
  tlsset = group->tg_tlsset;
  spin_unlock_irqrestore(NULL, flags);

  return tlsset;
}

#endif /* CONFIG_TLS_NELEM > 0 */
