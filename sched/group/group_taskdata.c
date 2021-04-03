/****************************************************************************
 * sched/group/group_taskdata.c
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

#include <nuttx/tls.h>

#include "group/group.h"

#ifndef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_set_taskdata
 *
 * Description:
 *   Set task-specific data pointer in TLS.
 *
 * Input Parameters:
 *   tcb - Identifies task to set TLS data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tls_set_taskdata(FAR struct tcb_s *tcb)
{
  FAR struct tls_info_s *info;
  FAR struct task_group_s *group;

  DEBUGASSERT(tcb != NULL && tcb->group != NULL);
  group = tcb->group;

  /* This currently assumes a push-down stack.  The TLS data lies at the
   * lowest address of the stack allocation.
   */

  info = (FAR struct tls_info_s *)tcb->stack_alloc_ptr;

  /* Copy the task data point buffer in the group structure into the
   * thread's TLS data.
   */

  info->tl_libvars = group->tg_libvars;
}

#endif /* !CONFIG_BUILD_KERNEL */