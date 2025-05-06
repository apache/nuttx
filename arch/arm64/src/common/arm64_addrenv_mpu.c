/****************************************************************************
 * arch/arm64/src/common/arm64_addrenv_mpu.c
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
 * Address Environment Interfaces
 *
 * Low-level interfaces used in binfmt/ to instantiate tasks with address
 * environments.  These interfaces all operate on type arch_addrenv_t which
 * is an abstract representation of a task group's address environment and
 * must be defined in arch/arch.h if CONFIG_ARCH_ADDRENV is defined.
 *
 *   up_addrenv_create   - Create an address environment
 *   up_addrenv_destroy  - Destroy an address environment.
 *   up_addrenv_vtext    - Returns the virtual base address of the .text
 *                         address environment
 *   up_addrenv_vdata    - Returns the virtual base address of the .bss/.data
 *                         address environment
 *   up_addrenv_heapsize - Returns the size of the initial heap allocation.
 *   up_addrenv_select   - Instantiate an address environment
 *   up_addrenv_clone    - Copy an address environment from one location to
 *                         another.
 *
 * Higher-level interfaces used by the tasking logic.  These interfaces are
 * used by the functions in sched/ and all operate on the thread which whose
 * group been assigned an address environment by up_addrenv_clone().
 *
 *   up_addrenv_attach   - Clone the address environment assigned to one TCB
 *                         to another.  This operation is done when a pthread
 *                         is created that share's the same group address
 *                         environment.
 *   up_addrenv_detach   - Release the thread's reference to an address
 *                         environment when a task/thread exits.
 *
 * If CONFIG_ARCH_KERNEL_STACK is selected, then each user process will have
 * two stacks:  (1) a large (and possibly dynamic) user stack and (2) a
 * smaller kernel stack.  However, this option is *required* if both
 * CONFIG_BUILD_KERNEL and CONFIG_LIBC_EXECFUNCS are selected.  Why?  Because
 * when we instantiate and initialize the address environment of the new
 * user process, we will temporarily lose the address environment of the old
 * user process, including its stack contents.  The kernel C logic will crash
 * immediately with no valid stack in place.
 *
 * If CONFIG_ARCH_KERNEL_STACK=y is selected then the platform specific
 * code must export these additional interfaces:
 *
 *   up_addrenv_kstackalloc  - Create a stack in the kernel address
 *                             environment
 *   up_addrenv_kstackfree   - Destroy the kernel stack.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/tls.h>
#include <nuttx/mm/mm.h>

#include "arm64_mpu.h"
#include "sched/sched.h"
#include "group/group.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_create
 *
 * Description:
 *   This function is called when a new task is created in order to
 *   instantiate an address environment for the new task group.
 *   up_addrenv_create() is essentially the allocator of the physical
 *   memory for the new task.
 *
 * Input Parameters:
 *   textsize - The size (in bytes) of the .text address environment needed
 *     by the task.  This region may be read/execute only.
 *   datasize - The size (in bytes) of the .data/.bss address environment
 *     needed by the task.  This region may be read/write only.  NOTE: The
 *     actual size of the data region that is allocated will include a
 *     OS private reserved region at the beginning.  The size of the
 *     private, reserved region is give by ARCH_DATA_RESERVE_SIZE.
 *   heapsize - The initial size (in bytes) of the heap address environment
 *     needed by the task.  This region may be read/write only.
 *   addrenv - The location to return the representation of the task address
 *     environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_create(size_t textsize, size_t datasize, size_t heapsize,
                      arch_addrenv_t *addrenv)
{
  binfo("addrenv=%p textsize=%lu datasize=%lu heapsize=%lu\n",
        addrenv,
        (unsigned long)textsize,
        (unsigned long)datasize,
        (unsigned long)heapsize);

  DEBUGASSERT(addrenv);

  /* Implement it later */

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_destroy
 *
 * Description:
 *   This function is called when a final thread leaves the task group and
 *   the task group is destroyed.  This function then destroys the defunct
 *   address environment, releasing the underlying physical memory.
 *
 * Input Parameters:
 *   addrenv - The address environment to be destroyed.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_destroy(arch_addrenv_t *addrenv)
{
  binfo("addrenv=%p\n", addrenv);
  DEBUGASSERT(addrenv);

  /* Implement it later */

  memset(addrenv, 0, sizeof(arch_addrenv_t));
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_vtext
 *
 * Description:
 *   Return the virtual address associated with the newly create .text
 *   address environment.  This function is used by the binary loaders in
 *   order get an address that can be used to initialize the new task.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *      returned by up_addrenv_create.
 *   vtext - The location to return the virtual address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_vtext(arch_addrenv_t *addrenv, void **vtext)
{
  DEBUGASSERT(addrenv && vtext);

  /* Implement it later */

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_vdata
 *
 * Description:
 *   Return the virtual address associated with the newly create .text
 *   address environment.  This function is used by the binary loaders in
 *   order get an address that can be used to initialize the new task.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *      returned by up_addrenv_create.
 *   textsize - For some implementations, the text and data will be saved
 *      in the same memory region (read/write/execute) and, in this case,
 *      the virtual address of the data just lies at this offset into the
 *      common region.
 *   vdata - The location to return the virtual address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_vdata(arch_addrenv_t *addrenv, uintptr_t textsize,
                     void **vdata)
{
  DEBUGASSERT(addrenv && vdata);

  /* Implement it later */

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_select
 *
 * Description:
 *   After an address environment has been established for a task group (via
 *   up_addrenv_create().  This function may be called to instantiate
 *   that address environment in the virtual address space.  this might be
 *   necessary, for example, to load the code for the task group from a file
 *   or to access address environment private data.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_select(const arch_addrenv_t *addrenv)
{
  /* Implement it later */

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_coherent
 *
 * Description:
 *   Flush D-Cache and invalidate I-Cache in preparation for a change in
 *   address environments.  This should immediately precede a call to
 *   up_addrenv_select();
 *
 * Input Parameters:
 *   addrenv - Describes the address environment to be made coherent.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_coherent(const arch_addrenv_t *addrenv)
{
  DEBUGASSERT(addrenv);
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_clone
 *
 * Description:
 *   Duplicate an address environment.  This does not copy the underlying
 *   memory, only the representation that can be used to instantiate that
 *   memory as an address environment.
 *
 * Input Parameters:
 *   src - The address environment to be copied.
 *   dest - The location to receive the copied address environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_clone(const arch_addrenv_t *src,
                     arch_addrenv_t *dest)
{
  binfo("src=%p dest=%p\n", src, dest);
  DEBUGASSERT(src && dest);

  /* Just copy the address environment from the source to the destination */

  memcpy(dest, src, sizeof(arch_addrenv_t));
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_attach
 *
 * Description:
 *   This function is called from the core scheduler logic when a thread
 *   is created that needs to share the address environment of its task
 *   group.
 *
 * Input Parameters:
 *   ptcb  - The tcb of the parent task.
 *   tcb   - The tcb of the thread needing the address environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_attach(struct tcb_s *ptcb, struct tcb_s *tcb)
{
  binfo("parent=%p tcb=%p\n", ptcb, tcb);

  /* Nothing needs to be done in this implementation */

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_detach
 *
 * Description:
 *   This function is called when a task or thread exits in order to release
 *   its reference to an address environment.  The address environment,
 *   however, should persist until up_addrenv_destroy() is called when the
 *   task group is itself destroyed.  Any resources unique to this thread
 *   may be destroyed now.
 *
 * Input Parameters:
 *   tcb - The TCB of the task or thread whose the address environment will
 *     be released.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_detach(struct tcb_s *tcb)
{
  binfo("tcb=%p\n", tcb);
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_mprot
 *
 * Description:
 *   Modify access rights to an address range.
 *
 * Input Parameters:
 *   addrenv - The address environment to be modified.
 *   addr - Base address of the region.
 *   len - Size of the region.
 *   prot - Access right flags.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_mprot(arch_addrenv_t *addrenv, uintptr_t addr, size_t len,
                     int prot)
{
  /* Nothing needs to be done */

  return OK;
}

#ifdef CONFIG_ARCH_STACK_PROTECT
/****************************************************************************
 * Name: up_addrenv_ustackswitch
 *
 * Description:
 *   After an address environment has been established for a task's stack
 *   (via up_addrenv_ustackalloc().  This function may be called to
 *   instantiate that address environment in the virtual address space.
 *   This is a necessary step before each context switch to the newly created
 *   thread (including the initial thread startup).
 *
 * Input Parameters:
 *   tcb - The TCB of the thread with the stack address environment to be
 *     instantiated.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_ustackswitch(struct tcb_s *tcb)
{
  /* Implement it later */

  return OK;
}

#  ifdef CONFIG_ARCH_KSTACK_PROTECT
/****************************************************************************
 * Name: up_addrenv_kstackswitch
 *
 * Description:
 *   After an address environment has been established for a task's stack
 *   (via up_addrenv_kstackswitch().  This function may be called to
 *   instantiate that address environment in the virtual address space.
 *   This is a necessary step before each context switch to the newly created
 *   thread (including the initial thread startup).
 *
 * Input Parameters:
 *   tcb - The TCB of the thread with the stack address environment to be
 *     instantiated.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_kstackswitch(struct tcb_s *tcb)
{
  /* Implement it later */

  return OK;
}
#  endif
#endif
