/****************************************************************************
 * sched/tls/tls_initinfo.c
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

#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/tls.h>

#include "tls.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_init_info
 *
 * Description:
 *   Allocate and initilize tls_info_s structure.
 *
 * Input Parameters:
 *   - tcb: The TCB of new task
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tls_init_info(FAR struct tcb_s *tcb)
{
  FAR struct tls_info_s *info;

  /* Allocate thread local storage */

  info = up_stack_frame(tcb, up_tls_size());
  if (info == NULL)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(info == tcb->stack_alloc_ptr);

  /* Initialize thread local storage */

  up_tls_initialize(info);

  /* Attach per-task info in group to TLS */

  info->tl_task = tcb->group->tg_info;
  return OK;
}
