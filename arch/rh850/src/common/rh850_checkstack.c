/****************************************************************************
 * arch/rh850/src/common/rh850_checkstack.c
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

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <nuttx/tls.h>

#include "rh850_internal.h"

#ifdef CONFIG_STACK_COLORATION

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rh850_stack_check
 *
 * Description:
 *   Bytes used from the top of a colored stack: the stack grows down, the
 *   first word above the base without STACK_COLOR is the high mark.
 *
 ****************************************************************************/

static size_t rh850_stack_check(uintptr_t alloc, size_t size)
{
  uintptr_t start;
  uintptr_t end;
  uint32_t *ptr;
  size_t mark;

  if (size == 0)
    {
      return 0;
    }

  start = (alloc + 3) & ~3;
  end   = (alloc + size) & ~3;
  size  = end - start;

  for (ptr = (uint32_t *)start, mark = (size >> 2);
       mark > 0 && *ptr == STACK_COLOR;
       ptr++, mark--);

  return mark << 2;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t up_check_tcbstack(FAR struct tcb_s *tcb, size_t check_size)
{
  return rh850_stack_check((uintptr_t)tcb->stack_base_ptr, check_size);
}

size_t up_check_intstack(int cpu, size_t check_size)
{
  if (check_size == 0)
    {
      check_size = CONFIG_ARCH_INTERRUPTSTACK & ~3;
    }

  return rh850_stack_check((uintptr_t)g_intstackalloc, check_size);
}

#endif /* CONFIG_STACK_COLORATION */
