/****************************************************************************
 * arch/arm64/src/common/arm64_task_sched.c
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

#include <stdbool.h>
#include <sched.h>
#include <debug.h>
#include <assert.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <arch/syscall.h>

#include "sched/sched.h"
#include "group/group.h"
#include "arm64_internal.h"
#include "arm64_fatal.h"

#ifdef CONFIG_ARCH_FPU
#include "arm64_fpu.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_fullcontextrestore
 *
 * Description:
 *   Restore the current thread context.  Full prototype is:
 *
 *   void arm64_fullcontextrestore(uint64_t *restoreregs) noreturn_function;
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm64_fullcontextrestore(uint64_t *restoreregs)
{
  sys_call1(SYS_restore_context, (uintptr_t)restoreregs);

  __builtin_unreachable();
}

/****************************************************************************
 * Name: arm64_switchcontext
 *
 * Description:
 *   Save the current thread context and restore the specified context.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm64_switchcontext(uint64_t **saveregs, uint64_t *restoreregs)
{
  sys_call2(SYS_switch_context, (uintptr_t)saveregs, (uintptr_t)restoreregs);
}
