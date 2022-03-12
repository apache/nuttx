/****************************************************************************
 * sched/group/group_setuptaskfiles.c
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
#include <assert.h>

#include <nuttx/fs/fs.h>

#include "sched/sched.h"
#include "group/group.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_setuptaskfiles
 *
 * Description:
 *   Configure a newly allocated TCB so that it will inherit
 *   file descriptors and streams from the parent task.
 *
 * Input Parameters:
 *   tcb - tcb of the new task.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int group_setuptaskfiles(FAR struct task_tcb_s *tcb)
{
  FAR struct task_group_s *group = tcb->cmn.group;
#ifndef CONFIG_FDCLONE_DISABLE
  FAR struct tcb_s *rtcb = this_task();
  int ret;
#endif

  DEBUGASSERT(group);
#ifndef CONFIG_DISABLE_PTHREAD
  DEBUGASSERT((tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) !=
              TCB_FLAG_TTYPE_PTHREAD);
#endif

#ifndef CONFIG_FDCLONE_DISABLE
  DEBUGASSERT(rtcb->group);

  /* Duplicate the parent task's file descriptors */

  ret = files_duplist(&rtcb->group->tg_filelist, &group->tg_filelist);
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* Allocate file/socket streams for the new TCB */

#ifdef CONFIG_FILE_STREAM
  return group_setupstreams(tcb);
#else
  return OK;
#endif
}
