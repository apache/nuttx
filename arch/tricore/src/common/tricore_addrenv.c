/****************************************************************************
 * arch/tricore/src/common/tricore_addrenv.c
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

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/tls.h>

#include <arch/irq.h>

#include "tricore_mpu.h"
#include "sched/sched.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Records the stack mpu region id */

#ifdef CONFIG_ARCH_STACK_PROTECT
static unsigned int g_addrenv_stack_region;
#endif

/****************************************************************************
 * Public Data
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

  /* When use mpu addrenv info must set by modlib */

  addrenv->textsize = textsize;
  if (textsize != 0)
    {
      DEBUGASSERT(addrenv->text);
    }

  addrenv->datasize = datasize;
  if (datasize != 0)
    {
      DEBUGASSERT(addrenv->data);
    }

#ifdef CONFIG_BUILD_PROTECTED
  addrenv->heapsize = heapsize;
  if (heapsize != 0)
    {
      DEBUGASSERT(addrenv->heap);
    }
#endif

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
  *vtext = (void *)addrenv->text;
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
  *vdata = (void *)addrenv->data;
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
  binfo("addrenv=%p\n", addrenv);
  DEBUGASSERT(addrenv);

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

  /* Nothing needs to be done in this implementation */

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
  struct mpu_region_s conf;

  conf.base = (uintptr_t)tcb->stack_alloc_ptr;
  conf.size = (uintptr_t)tcb->stack_base_ptr + tcb->adj_stack_size -
              (uintptr_t)tcb->stack_alloc_ptr;
  conf.flags[PSW_PRS_KERNEL_SET] = REGION_TYPE_DATA | REGION_ATTR_RW;
  conf.flags[PSW_PRS_USER_SET] = REGION_TYPE_DATA | REGION_ATTR_RW;

  if (g_addrenv_stack_region == 0)
    {
      g_addrenv_stack_region = mpu_allocdataregion();
    }

#ifdef CONFIG_BUILD_FLAT
  mpu_modify_region(PSW_PRS_KERNEL_SET, g_addrenv_stack_region,
                    conf.base, conf.size,
                    conf.flags[PSW_PRS_KERNEL_SET]);
#else
  mpu_modify_region(PSW_PRS_USER_SET, g_addrenv_stack_region,
                    conf.base, conf.size,
                    conf.flags[PSW_PRS_USER_SET]);
#endif
  return OK;
}

#ifdef CONFIG_ARCH_KSTACK_PROTECT
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
  struct mpu_region_s conf;

  conf.base = (uintptr_t)tcb->stack_alloc_ptr;
  conf.size = (uintptr_t)tcb->stack_base_ptr + tcb->adj_stack_size -
              (uintptr_t)tcb->stack_alloc_ptr;
  conf.flags[PSW_PRS_KERNEL_SET] = REGION_TYPE_DATA | REGION_ATTR_RW;
  conf.flags[PSW_PRS_USER_SET] = REGION_TYPE_DATA | REGION_ATTR_RO;

  if (g_addrenv_stack_region == 0)
    {
      g_addrenv_stack_region = mpu_allocdataregion();
    }

  mpu_modify_region(PSW_PRS_KERNEL_SET, g_addrenv_stack_region,
                    conf.base, conf.size,
                    conf.flags[PSW_PRS_KERNEL_SET]);
  return OK;
}
#endif

#endif
