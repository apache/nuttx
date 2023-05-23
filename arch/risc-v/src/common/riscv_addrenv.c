/****************************************************************************
 * arch/risc-v/src/common/riscv_addrenv.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <nuttx/pgalloc.h>

#include <arch/barriers.h>

#include "addrenv.h"
#include "pgalloc.h"
#include "riscv_mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only CONFIG_BUILD_KERNEL is supported (i.e. tested) */

#ifndef CONFIG_BUILD_KERNEL
#  error "This module is intended to be used with CONFIG_BUILD_KERNEL"
#endif

/* Entries per PGT */

#define ENTRIES_PER_PGT     (RV_MMU_PAGE_ENTRIES)

/* Make sure the address environment virtual address boundary is valid */

static_assert((ARCH_ADDRENV_VBASE & RV_MMU_SECTION_ALIGN) == 0,
              "Addrenv start address is not aligned to section boundary");

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uintptr_t            g_kernel_mappings;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: map_spgtables
 *
 * Description:
 *   Map vaddr to the static page tables.
 *
 * Input Parameters:
 *   addrenv - Describes the address environment
 *
 ****************************************************************************/

static void map_spgtables(arch_addrenv_t *addrenv, uintptr_t vaddr)
{
  int       i;
  uintptr_t prev;

  /* Start from L1, and connect until max level - 1 */

  prev = riscv_pgvaddr(addrenv->spgtables[0]);

  /* Check if the mapping already exists */

  if (mmu_ln_getentry(1, prev, vaddr) != 0)
    {
      return;
    }

  /* No mapping yet, create it */

  for (i = 0; i < (ARCH_SPGTS - 1); i++)
    {
      uintptr_t next = riscv_pgvaddr(addrenv->spgtables[i + 1]);
      mmu_ln_setentry(i + 1, prev, next, vaddr, MMU_UPGT_FLAGS);
      prev = next;
    }
}

/****************************************************************************
 * Name: create_spgtables
 *
 * Description:
 *   Create the static page tables. Allocate memory for them and connect them
 *   together.
 *
 * Input Parameters:
 *   addrenv - Describes the address environment
 *
 * Returned value:
 *   Amount of pages created on success; a negated errno value on failure
 *
 ****************************************************************************/

static int create_spgtables(arch_addrenv_t *addrenv)
{
  int       i;
  uintptr_t paddr;

  for (i = 0; i < ARCH_SPGTS; i++)
    {
      paddr = mm_pgalloc(1);
      if (!paddr)
        {
          return -ENOMEM;
        }

      /* Wipe the memory and assign it */

      riscv_pgwipe(paddr);
      addrenv->spgtables[i] = paddr;
    }

  /* Flush the data cache, so the changes are committed to memory */

  __DMB();

  return i;
}

/****************************************************************************
 * Name: copy_kernel_mappings
 *
 * Description:
 *   Copy kernel mappings to address environment. Expects that the user page
 *   table does not contain any mappings yet (as they will be wiped).
 *
 * Input Parameters:
 *   addrenv - Describes the address environment. The page tables must exist
 *      at this point.
 *
 * Returned value:
 *   OK on success, ERROR on failure
 *
 ****************************************************************************/

static int copy_kernel_mappings(arch_addrenv_t *addrenv)
{
  uintptr_t user_mappings = riscv_pgvaddr(addrenv->spgtables[0]);

  /* Copy the L1 references */

  if (user_mappings == 0)
    {
      return -EINVAL;
    }

  memcpy((void *)user_mappings, (void *)g_kernel_mappings, RV_MMU_PAGE_SIZE);

  return OK;
}

/****************************************************************************
 * Name: create_region
 *
 * Description:
 *   Map a single region of memory to MMU. Assumes that the static page
 *   tables exist. Allocates the final level page tables and commits the
 *   region memory to physical memory.
 *
 * Input Parameters:
 *   addrenv - Describes the address environment
 *   vaddr - Base virtual address for the mapping
 *   size - Size of the region in bytes
 *   mmuflags - MMU flags to use
 *
 * Returned value:
 *   Amount of pages created on success; a negated errno value on failure
 *
 ****************************************************************************/

static int create_region(arch_addrenv_t *addrenv, uintptr_t vaddr,
                         size_t size, uint32_t mmuflags)
{
  uintptr_t ptlast;
  uintptr_t ptprev;
  uintptr_t paddr;
  uint32_t  ptlevel;
  int       npages;
  int       nmapped;
  int       i;
  int       j;

  nmapped   = 0;
  npages    = MM_NPAGES(size);
  ptprev    = riscv_pgvaddr(addrenv->spgtables[ARCH_SPGTS - 1]);
  ptlevel   = ARCH_SPGTS;

  /* Create mappings for the lower level tables */

  map_spgtables(addrenv, vaddr);

  /* Begin allocating memory for the page tables */

  for (i = 0; i < npages; i += ENTRIES_PER_PGT)
    {
      /* Get the current final level entry corresponding to this vaddr */

      paddr = mmu_pte_to_paddr(mmu_ln_getentry(ptlevel, ptprev, vaddr));

      if (!paddr)
        {
          /* Nothing yet, allocate one page for final level page table */

          paddr = mm_pgalloc(1);
          if (!paddr)
            {
              return -ENOMEM;
            }

          /* Map the page table to the prior level */

          mmu_ln_setentry(ptlevel, ptprev, paddr, vaddr, MMU_UPGT_FLAGS);

          /* This is then used to map the final level */

          riscv_pgwipe(paddr);
        }

      ptlast = riscv_pgvaddr(paddr);

      /* Then allocate memory for the region data */

      for (j = 0; j < ENTRIES_PER_PGT && nmapped < size; j++)
        {
          paddr = mm_pgalloc(1);
          if (!paddr)
            {
              return -ENOMEM;
            }

          /* Wipe the physical page memory */

          riscv_pgwipe(paddr);

          /* Then map the virtual address to the physical address */

          mmu_ln_setentry(ptlevel + 1, ptlast, paddr, vaddr, mmuflags);
          nmapped += MM_PGSIZE;
          vaddr   += MM_PGSIZE;
        }
    }

  /* Flush the data cache, so the changes are committed to memory */

  __DMB();

  return npages;
}

/****************************************************************************
 * Name: vaddr_is_shm
 *
 * Description:
 *   Check if a vaddr is part of the SHM area
 *
 * Input Parameters:
 *   vaddr - Virtual address to check
 *
 * Returned value:
 *   true if it is; false if not
 *
 ****************************************************************************/

static inline bool vaddr_is_shm(uintptr_t vaddr)
{
#if defined (CONFIG_ARCH_SHM_VBASE) && defined(ARCH_SHM_VEND)
  return vaddr >= CONFIG_ARCH_SHM_VBASE && vaddr < ARCH_SHM_VEND;
#else
  UNUSED(vaddr);
  return false;
#endif
}

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
  int       ret;
  uintptr_t resvbase;
  uintptr_t resvsize;
  uintptr_t textbase;
  uintptr_t database;
  uintptr_t heapbase;

  DEBUGASSERT(addrenv);
  DEBUGASSERT(MM_ISALIGNED(ARCH_ADDRENV_VBASE));

  /* Make sure the address environment is wiped before doing anything */

  memset(addrenv, 0, sizeof(arch_addrenv_t));

  /* Create the static page tables */

  ret = create_spgtables(addrenv);

  if (ret < 0)
    {
      serr("ERROR: Failed to create static page tables\n");
      goto errout;
    }

  /* Map the kernel memory for the user */

  ret = copy_kernel_mappings(addrenv);

  if (ret < 0)
    {
      serr("ERROR: Failed to copy kernel mappings to new environment");
      goto errout;
    }

  /* Calculate the base addresses for convenience */

#if (CONFIG_ARCH_TEXT_VBASE != 0x0) && (CONFIG_ARCH_HEAP_VBASE != 0x0)
  resvbase = CONFIG_ARCH_DATA_VBASE;
  resvsize = ARCH_DATA_RESERVE_SIZE;
  textbase = CONFIG_ARCH_TEXT_VBASE;
  database = resvbase + MM_PGALIGNUP(resvsize);
  heapbase = CONFIG_ARCH_HEAP_VBASE;
#else
  resvbase = ARCH_ADDRENV_VBASE;
  resvsize = ARCH_DATA_RESERVE_SIZE;
  textbase = resvbase + MM_PGALIGNUP(resvsize);
  database = textbase + MM_PGALIGNUP(textsize);
  heapbase = database + MM_PGALIGNUP(datasize);
#endif

  /* Allocate 1 extra page for heap, temporary fix for #5811 */

  heapsize = heapsize + MM_PGALIGNUP(1);

  /* Map the reserved area */

  ret = create_region(addrenv, resvbase, resvsize, MMU_UDATA_FLAGS);

  if (ret < 0)
    {
      berr("ERROR: Failed to create reserved region: %d\n", ret);
      goto errout;
    }

  /* Map each region in turn */

  ret = create_region(addrenv, textbase, textsize, MMU_UTEXT_FLAGS);

  if (ret < 0)
    {
      berr("ERROR: Failed to create .text region: %d\n", ret);
      goto errout;
    }

  ret = create_region(addrenv, database, datasize, MMU_UDATA_FLAGS);

  if (ret < 0)
    {
      berr("ERROR: Failed to create .bss/.data region: %d\n", ret);
      goto errout;
    }

  ret = create_region(addrenv, heapbase, heapsize, MMU_UDATA_FLAGS);

  if (ret < 0)
    {
      berr("ERROR: Failed to create heap region: %d\n", ret);
      goto errout;
    }

  /* Save the heap base and initial size allocated. These will be needed when
   * the heap data structures are initialized.
   */

  addrenv->heapvbase = heapbase;
  addrenv->heapsize = (size_t)ret << MM_PGSHIFT;

  /* Save the text base */

  addrenv->textvbase = textbase;

  /* Save the data base */

  addrenv->datavbase = database;

  /* Provide the satp value for context switch */

  addrenv->satp = mmu_satp_reg(addrenv->spgtables[0], 0);

  /* When all is set and done, flush the data caches */

  __ISB();
  __DMB();

  return OK;

errout:
  up_addrenv_destroy(addrenv);
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

int up_addrenv_destroy(arch_addrenv_t *addrenv)
{
  /* Recursively destroy it all, need to table walk */

  uintptr_t *ptprev;
  uintptr_t *ptlast;
  uintptr_t  paddr;
  uintptr_t  vaddr;
  size_t     pgsize;
  int        i;
  int        j;

  DEBUGASSERT(addrenv);

  /* Make sure the caches are flushed before doing this */

  __ISB();
  __DMB();

  /* Things start from the beginning of the user virtual memory */

  vaddr  = ARCH_ADDRENV_VBASE;
  pgsize = mmu_get_region_size(ARCH_SPGTS);

  /* First destroy the allocated memory and the final level page table */

  ptprev = (uintptr_t *)riscv_pgvaddr(addrenv->spgtables[ARCH_SPGTS - 1]);
  if (ptprev)
    {
      for (i = 0; i < ENTRIES_PER_PGT; i++, vaddr += pgsize)
        {
          if (vaddr_is_shm(vaddr))
            {
              /* Do not free memory from SHM area */

              continue;
            }

          ptlast = (uintptr_t *)riscv_pgvaddr(mmu_pte_to_paddr(ptprev[i]));
          if (ptlast)
            {
              /* Page table allocated, free any allocated memory */

              for (j = 0; j < ENTRIES_PER_PGT; j++)
                {
                  paddr = mmu_pte_to_paddr(ptlast[j]);
                  if (paddr)
                    {
                      mm_pgfree(paddr, 1);
                    }
                }

              /* Then free the page table itself */

              mm_pgfree((uintptr_t)ptlast, 1);
            }
        }
    }

  /* Then destroy the static tables */

  for (i = 0; i < ARCH_SPGTS; i++)
    {
      paddr = addrenv->spgtables[i];
      if (paddr)
        {
          mm_pgfree(paddr, 1);
        }
    }

  /* When all is set and done, flush the caches */

  __ISB();
  __DMB();

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
  *vtext = (void *)addrenv->textvbase;
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
  *vdata = (void *)addrenv->datavbase;
  return OK;
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
int up_addrenv_vheap(const arch_addrenv_t *addrenv, void **vheap)
{
  DEBUGASSERT(addrenv && vheap);
  *vheap = (void *)addrenv->heapvbase;
  return OK;
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
ssize_t up_addrenv_heapsize(const arch_addrenv_t *addrenv)
{
  DEBUGASSERT(addrenv);
  return (ssize_t)addrenv->heapsize;
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

int up_addrenv_select(const arch_addrenv_t *addrenv)
{
  DEBUGASSERT(addrenv && addrenv->satp);
  mmu_write_satp(addrenv->satp);
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
  /* Flush the instruction and data caches */

  __ISB();
  __DMB();
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
  DEBUGASSERT(src && dest);
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
  /* There is nothing that needs to be done */

  return OK;
}
