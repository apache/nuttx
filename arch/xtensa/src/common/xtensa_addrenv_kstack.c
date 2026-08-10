/****************************************************************************
 * arch/xtensa/src/common/xtensa_addrenv_kstack.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>

#include "xtensa.h"

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_KERNEL_STACK)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Xtensa windowed ABI requires 16-byte stack alignment */

#define KSTACK_ALIGNMENT  16
#define KSTACK_ALIGN_DOWN(a) ((a) & ~(KSTACK_ALIGNMENT - 1))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_kstackalloc
 *
 * Description:
 *   This function is called when a new thread is created to allocate the
 *   new thread's kernel stack.  This function may be called for certain
 *   terminating threads which have no kernel stack.  It must be tolerant of
 *   that case.
 *
 *   The stack comes from the kernel heap, which lives in internal SRAM and
 *   is mapped identically no matter which address environment is selected.
 *   That is the whole point of it:  the kernel needs somewhere to keep the
 *   exception frame and its spilled register windows that does not move when
 *   up_addrenv_select() reprograms the user cache-MMU windows.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that requires the kernel stack.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_kstackalloc(struct tcb_s *tcb)
{
  DEBUGASSERT(tcb && tcb->xcp.kstack == NULL);

  tcb->xcp.kstack = kmm_memalign(KSTACK_ALIGNMENT, ARCH_KERNEL_STACKSIZE);
  if (tcb->xcp.kstack == NULL)
    {
      berr("ERROR: Failed to allocate the kernel stack\n");
      return -ENOMEM;
    }

  /* Xtensa stacks grow down and must stay aligned, so the usable top is the
   * far end of the allocation.
   */

  tcb->xcp.ktopstk = (uint32_t *)
    KSTACK_ALIGN_DOWN((uintptr_t)tcb->xcp.kstack + ARCH_KERNEL_STACKSIZE);

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_kstackfree
 *
 * Description:
 *   This function is called when any thread exits.  This function frees
 *   the kernel stack.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that no longer requires the kernel stack.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_kstackfree(struct tcb_s *tcb)
{
  DEBUGASSERT(tcb);

  /* Does the exiting thread have a kernel stack? */

  if (tcb->xcp.kstack != NULL)
    {
      kmm_free(tcb->xcp.kstack);
      tcb->xcp.kstack  = NULL;
      tcb->xcp.ktopstk = NULL;
    }

  return OK;
}

#endif /* CONFIG_ARCH_ADDRENV && CONFIG_ARCH_KERNEL_STACK */
