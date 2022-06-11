/****************************************************************************
 * sched/tls/tls_dupinfo.c
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
#include <string.h>

#include "tls.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_dup_info
 *
 * Description:
 *   Allocate and duplicate tls_info_s structure.
 *
 * Input Parameters:
 *   - dst: The TCB of new task
 *   - src: The TCB of source task
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tls_dup_info(FAR struct tcb_s *dst, FAR struct tcb_s *src)
{
  FAR struct tls_info_s *info;

  /* Allocate thread local storage */

  info = up_stack_frame(dst, tls_info_size());
  if (info == NULL)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(info == dst->stack_alloc_ptr);

  /* Copy thread local storage */

  memcpy(info, src->stack_alloc_ptr, tls_info_size());

  /* Attach per-task info in group to TLS */

  info->tl_task = dst->group->tg_info;
  return OK;
}
