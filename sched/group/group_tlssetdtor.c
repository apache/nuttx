/****************************************************************************
 * sched/group/group_tlssetdtor.c
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
 * Name: tls_set_dtor
 *
 * Description:
 *   Set the TLS element destructor associated with the 'tlsindex' to 'destr'
 *
 * Input Parameters:
 *   tlsindex - Index of TLS data destructor to set
 *   destr    - The destr of TLS data element
 *
 * Returned Value:
 *   Zero is returned on success, a negated errno value is return on
 *   failure:
 *
 *     EINVAL - tlsindex is not in range.
 *
 ****************************************************************************/

int tls_set_dtor(int tlsindex, tls_dtor_t destr)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct task_group_s *group = rtcb->group;
  irqstate_t flags;

  DEBUGASSERT(group != NULL);
  DEBUGASSERT(tlsindex >= 0 && tlsindex < CONFIG_TLS_NELEM);

  flags = spin_lock_irqsave(NULL);
  group->tg_tlsdestr[tlsindex] = destr;
  spin_unlock_irqrestore(NULL, flags);

  return OK;
}

#endif /* CONFIG_TLS_NELEM > 0 */
