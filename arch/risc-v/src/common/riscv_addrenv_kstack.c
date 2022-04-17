/****************************************************************************
 * arch/risc-v/src/common/riscv_addrenv_kstack.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/addrenv.h>
#include <nuttx/arch.h>

#include "addrenv.h"
#include "riscv_internal.h"

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_KERNEL_STACK)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_kstackalloc
 *
 * Description:
 *   This function is called when a new thread is created to allocate
 *   the new thread's kernel stack.   This function may be called for certain
 *   terminating threads which have no kernel stack.  It must be tolerant of
 *   that case.
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

  /* Allocate the kernel stack */

  tcb->xcp.kstack = (uintptr_t *)kmm_memalign(STACK_ALIGNMENT,
                                              ARCH_KERNEL_STACKSIZE);
  if (!tcb->xcp.kstack)
    {
      berr("ERROR: Failed to allocate the kernel stack\n");
      return -ENOMEM;
    }

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

  if (tcb->xcp.kstack)
    {
      /* Yes.. Free the kernel stack */

      kmm_free(tcb->xcp.kstack);
      tcb->xcp.kstack = NULL;
    }

  return OK;
}

#endif /* CONFIG_ARCH_ADDRENV && CONFIG_ARCH_KERNEL_STACK */
