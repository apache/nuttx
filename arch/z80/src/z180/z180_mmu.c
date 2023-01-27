/****************************************************************************
 * arch/z80/src/z180/z180_mmu.c
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

/* See arch/z80/src/z180/z180_mmu.txt for additional information */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/mm/gran.h>

#include <arch/irq.h>
#include <arch/io.h>

#include "z80_internal.h"
#include "z180_mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_ARCH_ADDRENV
#  warning "OS address environment support is required (CONFIG_ARCH_ADDRENV)"
#endif

#ifndef CONFIG_GRAN
#  warning "This file requires the granual allocator (CONFIG_GRAN)"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static GRAN_HANDLE g_physhandle;
static struct z180_cbr_s g_cbrs[CONFIG_Z180_MAX_TASKS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z180_mmu_alloccbr
 *
 * Description:
 *   Find an unused structure in g_cbrs
 *   (i.e., one with reference count == 0).
 *   If a structure is found, its reference count is set to one and a pointer
 *   to the structure is returned.
 *
 ****************************************************************************/

static inline FAR struct z180_cbr_s *z180_mmu_alloccbr(void)
{
  int i;

  for (i = 0; i < CONFIG_Z180_MAX_TASKS; i++)
    {
      FAR struct z180_cbr_s *cbr = &g_cbrs[i];
      if (cbr->crefs == 0)
        {
          cbr->crefs = 1;
          return cbr;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: z180_mmu_freecbr
 *
 * Description:
 *   Free a structure in g_cbrs by setting its reference count to 0;
 *
 ****************************************************************************/

#define z180_mmu_freecbr(cbr) (cbr)->crefs = 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z180_mmu_lowinit
 *
 * Description:
 *   Low-level, power-up initialization of the z180 MMU.  this must be
 *   called very early in the boot process to get the basic operating
 *   memory configuration correct.  This function does *not* perform all
 *   necessray MMU initialization... only the basics needed at power-up.
 *   z80_mmu_initialize() must be called later to complete the entire MMU
 *   initialization.
 *
 ****************************************************************************/

void z180_mmu_lowinit(void) __naked
{
  /* Set the CBAR register to set up the virtual address of the Bank Area and
   * Common Area 1.  Set the BBR register to set up the physical mapping for
   * the Bank Area (the physical mapping for Common Area 1 will not be done
   * until the first task is started.
   */

  __asm
  ld c, #Z180_MMU_CBAR ; port
  ld a, #Z180_CBAR_VALUE ; value
  out (c), a

  ld c, #Z180_MMU_BBR ; port
  ld a, #Z180_BBR_VALUE ; value
  out (c), a
  __endasm;
}

/****************************************************************************
 * Name: z80_mmu_initialize
 *
 * Description:
 *   Perform higher level initialization of the MMU and physical memory
 *   memory management logic.
 *
 ****************************************************************************/

int z80_mmu_initialize(void)
{
  /* Here we use the granule allocator as a page allocator.  We lie and
   * say that 1 page is 1 byte.
   */

  g_physhandle = gran_initialize((FAR void *)Z180_PHYSHEAP_STARTPAGE,
                               Z180_PHYSHEAP_NPAGES, 0, 0);
  return g_physhandle ? OK : -ENOMEM;
}

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
 *                        another.
 *
 * Higher-level interfaces used by the tasking logic.  These interfaces are
 * used by the functions in sched/ and all operate on the thread which whose
 * group been assigned an address environment by up_addrenv_clone().
 *
 *   up_addrenv_attach   - Clone the address environment assigned to one TCB
 *                         to another.  This operation is done when a pthread
 *                         is created that share's the same address
 *                         environment.
 *   up_addrenv_detach   - Release the threads reference to an address
 *                         environment when a task/thread exits.
 *
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
                      FAR arch_addrenv_t *addrenv)
{
  FAR struct z180_cbr_s *cbr;
  irqstate_t flags;
  size_t envsize;
  uintptr_t alloc;
  unsigned int npages;
  int ret;

  /* Convert the size from bytes to numbers of pages */

#ifdef CONFIG_BUILD_KERNEL
  envsize = textsize + datasize + heapsize;
#else
  envsize = textsize + datasize;
#endif

  npages  = PHYS_ALIGNUP(envsize);
  if (npages < 1)
    {
      /* No address environment... but I suppose that is not an error */

      serr("ERROR: npages is zero\n");
      return OK;
    }

  /* Allocate a structure in the common .bss to hold information about the
   * task's address environment.  NOTE that this is not a part of the TCB,
   * but rather a break-away structure that can be shared by the task as
   * well as other threads.  That is necessary because the life of the
   * address of environment might be longer than the life of the task.
   */

  flags = enter_critical_section();
  cbr = z180_mmu_alloccbr();
  if (!cbr)
    {
      serr("ERROR: No free CBR structures\n");
      ret = -ENOMEM;
      goto errout_with_irq;
    }

  /* Now allocate the physical memory to back up the address environment */

  alloc = (uintptr_t)gran_alloc(g_physhandle, npages);
  if (alloc == NULL)
    {
      serr("ERROR: Failed to allocate %d pages\n", npages);
      ret = -ENOMEM;
      goto errout_with_cbr;
    }

  /* Save the information in the CBR structure.  Note that alloc is in
   * 4KB pages, already in the right form for the CBR.
   */

  DEBUGASSERT(alloc <= 0xff);

  cbr->cbr     = (uint8_t)alloc;
  cbr->pages   = (uint8_t)npages;
  *addrenv     = (arch_addrenv_t)cbr;

  leave_critical_section(flags);
  return OK;

errout_with_cbr:
  z180_mmu_freecbr(cbr);

errout_with_irq:
  leave_critical_section(flags);
  return ret;
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

int up_addrenv_destroy(FAR arch_addrenv_t *addrenv)
{
  FAR struct z180_cbr_s *cbr = (FAR struct z180_cbr_s *)*addrenv;

  DEBUGASSERT(cbr);

  /* Free the physical address space backing up the mapping */

  gran_free(g_physhandle, (FAR void *)cbr->cbr, cbr->pages);

  /* And make the CBR structure available for re-use */

  z180_mmu_freecbr(cbr);
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

int up_addrenv_vtext(FAR arch_addrenv_t *addrenv, FAR void **vtext)
{
  return CONFIG_Z180_COMMON1AREA_VIRTBASE;
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

int up_addrenv_vdata(FAR arch_addrenv_t *addrenv, uintptr_t textsize,
                     FAR void **vdata)
{
  return CONFIG_Z180_COMMON1AREA_VIRTBASE + textsize;
}

/****************************************************************************
 * Name: up_addrenv_vheap
 *
 * Description:
 *   Return the heap virtual address associated with the newly created
 *   address environment.  This function is used by the binary loaders in
 *   order get an address that can be used to initialize the new task.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *      returned by up_addrenv_create.
 *   vheap - The location to return the virtual address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int up_addrenv_vheap(FAR const arch_addrenv_t *addrenv, FAR void **vheap)
{
  /* Not implemented */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: up_addrenv_heapsize
 *
 * Description:
 *   Return the initial heap allocation size.  That is the amount of memory
 *   allocated by up_addrenv_create() when the heap memory region was first
 *   created.  This may or may not differ from the heapsize parameter that
 *   was passed to up_addrenv_create()
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *
 * Returned Value:
 *   The initial heap size allocated is returned on success; a negated
 *   errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
ssize_t up_addrenv_heapsize(FAR const arch_addrenv_t *addrenv)
{
  /* Not implemented */

  return (ssize_t)-ENOSYS;
}
#endif

/****************************************************************************
 * Name: up_addrenv_select
 *
 * Description:
 *   After an address environment has been established for a task (via
 *   up_addrenv_create()), this function may be called to instantiate
 *   that address environment in the virtual address space.  This might be
 *   necessary, for example, to load the code for the task from a file or
 *   to access address environment private data.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_select(FAR const arch_addrenv_t *addrenv)
{
  FAR struct z180_cbr_s *cbr = (FAR struct z180_cbr_s *)addrenv;
  irqstate_t flags;

  DEBUGASSERT(cbr);

  flags = enter_critical_section();

  /* Write the new CBR value into CBR register */

  outp(Z180_MMU_CBR, cbr->cbr);
  leave_critical_section(flags);
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

int up_addrenv_coherent(FAR const arch_addrenv_t *addrenv)
{
  /* There are no caches */

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

int up_addrenv_clone(FAR const arch_addrenv_t *src,
                     FAR arch_addrenv_t *dest)
{
  DEBUGASSERT(src && dest);

  /* Copy the CBR structure.  This is an atomic operation so no special
   * precautions should be needed.
   */

  *dest = *src;
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

int up_addrenv_attach(FAR struct tcb_s *ptcb, FAR struct tcb_s *tcb)
{
  /* There is nothing that needs to be done */

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
 *   NOTE: In some platforms, nothing will need to be done in this case.
 *   Simply being a member of the group that has the address environment
 *   may be sufficient.
 *
 * Input Parameters:
 *   tcb - The TCB of the task or thread whose the address environment will
 *     be released.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_detach(FAR struct tcb_s *tcb)
{
  /* There is nothing that needs to be done */

  return OK;
}
