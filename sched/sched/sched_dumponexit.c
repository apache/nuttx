/****************************************************************************
 * sched/sched/sched_dumponexit.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <syslog.h>

#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SCHED_DUMP_ON_EXIT

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_dumponexit
 *
 * Description:
 *   When the thread exits, dump the information of thread.
 *
 ****************************************************************************/

void nxsched_dumponexit(void)
{
  FAR struct tcb_s *tcb = this_task();
  FAR const char *name = get_task_name(tcb);

  syslog(LOG_INFO, "task exit! tcb=%p name=%s, tid:%d, priority=%d "
         "entry:%p pid: %d, stack_alloc_ptr: %p, adj_stack_size: %zu\n",
         tcb, name, tcb->pid, tcb->sched_priority, tcb->entry.main,
         tcb->group->tg_pid, tcb->stack_base_ptr, tcb->adj_stack_size);
}

#endif /* CONFIG_SCHED_DUMP_ON_EXIT */
