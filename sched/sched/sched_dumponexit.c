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

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>

#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SCHED_DUMP_ON_EXIT

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dumphandler
 *
 * Description:
 *   Dump the state of all tasks whenever on task exits.  This is debug
 *   instrumentation that was added to check file-related reference counting
 *   but could be useful again sometime in the future.
 *
 ****************************************************************************/

static void dumphandler(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct filelist *filelist;

  syslog(LOG_INFO, "tcb=%p name=%s, pid:%d, priority=%d state=%d "
         "stack_alloc_ptr: %p, adj_stack_size: %zu\n",
         tcb, tcb->name, tcb->pid, tcb->sched_priority, tcb->task_state,
         tcb->stack_alloc_ptr, tcb->adj_stack_size);

  filelist = &tcb->group->tg_filelist;
  files_dumplist(filelist);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_dumponexit
 *
 * Description:
 *   Dump the state of all tasks whenever on task exits.  This is debug
 *   instrumentation that was added to check file-related reference counting
 *   but could be useful again sometime in the future.
 *
 ****************************************************************************/

void nxsched_dumponexit(void)
{
  sinfo("Other tasks:\n");
  nxsched_foreach(dumphandler, NULL);
}

#endif /* CONFIG_SCHED_DUMP_ON_EXIT */
