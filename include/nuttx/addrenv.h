/****************************************************************************
 * include/nuttx/addrenv.h
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

#ifndef __INCLUDE_NUTTX_ADDRENV_H
#define __INCLUDE_NUTTX_ADDRENV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_BUILD_KERNEL
#  include <signal.h>
#  include <nuttx/mm/mm.h>
#endif

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/wqueue.h>

#include <arch/arch.h>

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Pre-requisites */

#ifndef CONFIG_MM_PGALLOC
#  error CONFIG_MM_PGALLOC not defined
#endif

#ifndef CONFIG_MM_PGSIZE
#  error CONFIG_MM_PGSIZE not defined
#endif

/* .text region */

#ifndef CONFIG_ARCH_TEXT_VBASE
#  error CONFIG_ARCH_TEXT_VBASE not defined
#  define CONFIG_ARCH_TEXT_VBASE 0
#endif

#if (CONFIG_ARCH_TEXT_VBASE & CONFIG_MM_MASK) != 0
#  error CONFIG_ARCH_TEXT_VBASE not aligned to page boundary
#endif

#ifndef CONFIG_ARCH_TEXT_NPAGES
#  warning CONFIG_ARCH_TEXT_NPAGES not defined
#  define CONFIG_ARCH_TEXT_NPAGES 1
#endif

#define ARCH_TEXT_SIZE  (CONFIG_ARCH_TEXT_NPAGES * CONFIG_MM_PGSIZE)
#define ARCH_TEXT_VEND  (CONFIG_ARCH_TEXT_VBASE + ARCH_TEXT_SIZE)

/* .bss/.data region */

#ifndef CONFIG_ARCH_DATA_VBASE
#  error CONFIG_ARCH_DATA_VBASE not defined
#  define CONFIG_ARCH_DATA_VBASE ARCH_TEXT_VEND
#endif

#if (CONFIG_ARCH_DATA_VBASE & CONFIG_MM_MASK) != 0
#  error CONFIG_ARCH_DATA_VBASE not aligned to page boundary
#endif

#ifndef CONFIG_ARCH_DATA_NPAGES
#  warning CONFIG_ARCH_DATA_NPAGES not defined
#  define CONFIG_ARCH_DATA_NPAGES 1
#endif

#define ARCH_DATA_SIZE  (CONFIG_ARCH_DATA_NPAGES * CONFIG_MM_PGSIZE)
#define ARCH_DATA_VEND  (CONFIG_ARCH_DATA_VBASE + ARCH_DATA_SIZE)

/* Reserved .bss/.data region.  In the kernel build (CONFIG_BUILD_KERNEL),
 * the region at the beginning of the .bss/.data region is reserved for use
 * by the OS.  This reserved region contains support for:
 *
 *   1. The task group's heap memory management data structures and
 *   2. Support for delivery of signals.
 *
 * We don't use sizeof(struct addrenv_reserve_s) but, instead, a nice
 * even number that we must be assure is greater than or equal to
 * sizeof(struct addrenv_reserve_s)
 */

#ifdef CONFIG_BUILD_KERNEL
#  define ARCH_DATA_RESERVE_SIZE 512
#else
#  define ARCH_DATA_RESERVE_SIZE 0
#endif

/* Heap region */

#ifndef CONFIG_ARCH_HEAP_VBASE
#  error CONFIG_ARCH_HEAP_VBASE not defined
#  define CONFIG_ARCH_HEAP_VBASE ARCH_DATA_VEND
#endif

#if (CONFIG_ARCH_HEAP_VBASE & CONFIG_MM_MASK) != 0
#  error CONFIG_ARCH_HEAP_VBASE not aligned to page boundary
#endif

#ifndef CONFIG_ARCH_HEAP_NPAGES
#  warning CONFIG_ARCH_HEAP_NPAGES not defined
#  define CONFIG_ARCH_HEAP_NPAGES 1
#endif

#define ARCH_HEAP_SIZE  (CONFIG_ARCH_HEAP_NPAGES * CONFIG_MM_PGSIZE)
#define ARCH_HEAP_VEND  (CONFIG_ARCH_HEAP_VBASE + ARCH_HEAP_SIZE)

#ifdef CONFIG_ARCH_STACK_DYNAMIC
  /* User stack region */

#  ifndef CONFIG_ARCH_STACK_VBASE
#    error CONFIG_ARCH_STACK_VBASE not defined
#    define CONFIG_ARCH_STACK_VBASE ARCH_HEAP_VEND
#  endif

#  if (CONFIG_ARCH_STACK_VBASE & CONFIG_MM_MASK) != 0
#    error CONFIG_ARCH_STACK_VBASE not aligned to page boundary
#  endif

#  ifndef CONFIG_ARCH_STACK_NPAGES
#    warning CONFIG_ARCH_STACK_NPAGES not defined
#    define CONFIG_ARCH_STACK_NPAGES 1
#  endif

#  define ARCH_STACK_SIZE (CONFIG_ARCH_STACK_NPAGES * CONFIG_MM_PGSIZE)
#  define ARCH_STACK_VEND (CONFIG_ARCH_STACK_VBASE + ARCH_STACK_SIZE)

#ifdef CONFIG_ARCH_KERNEL_STACK
/* Kernel stack */

#  ifndef CONFIG_ARCH_KERNEL_STACKSIZE
#    define CONFIG_ARCH_KERNEL_STACKSIZE 1568
#  endif
#endif

  /* A single page scratch region used for temporary mappings */

#  define __ARCH_SHM_VBASE ARCH_STACK_VEND
#else
  /* A single page scratch region used for temporary mappings */

#  define __ARCH_SHM_VBASE ARCH_HEAP_VEND
#endif

/* Shared memory regions */

#ifdef CONFIG_ARCH_VMA_MAPPING
#  ifndef CONFIG_ARCH_SHM_VBASE
#    error CONFIG_ARCH_SHM_VBASE not defined
#    define CONFIG_ARCH_SHM_VBASE __ARCH_SHM_VBASE
#  endif

#  if (CONFIG_ARCH_SHM_VBASE & CONFIG_MM_MASK) != 0
#    error CONFIG_ARCH_SHM_VBASE not aligned to page boundary
#  endif

#  ifndef CONFIG_ARCH_SHM_MAXREGIONS
#    warning CONFIG_ARCH_SHM_MAXREGIONS not defined
#    define CONFIG_ARCH_SHM_MAXREGIONS 1
#  endif

#  ifndef CONFIG_ARCH_SHM_NPAGES
#    warning CONFIG_ARCH_SHM_NPAGES not defined
#    define CONFIG_ARCH_SHM_NPAGES 1
#  endif

#  define ARCH_SHM_MAXPAGES   (CONFIG_ARCH_SHM_NPAGES * CONFIG_ARCH_SHM_MAXREGIONS)
#  define ARCH_SHM_REGIONSIZE (CONFIG_ARCH_SHM_NPAGES * CONFIG_MM_PGSIZE)
#  define ARCH_SHM_SIZE       (CONFIG_ARCH_SHM_MAXREGIONS * ARCH_SHM_REGIONSIZE)
#  define ARCH_SHM_VEND       (CONFIG_ARCH_SHM_VBASE + ARCH_SHM_SIZE - 1)

#  define ARCH_SCRATCH_VBASE   ARCH_SHM_VEND
#else
#  define ARCH_SCRATCH_VBASE   __ARCH_SHM_VBASE
#endif

/* There is no need to use the scratch memory region if the page pool memory
 * is statically mapped.
 */

#ifdef CONFIG_ARCH_PGPOOL_MAPPING

#  ifndef CONFIG_ARCH_PGPOOL_PBASE
#    error CONFIG_ARCH_PGPOOL_PBASE not defined
#  endif

#  ifndef CONFIG_ARCH_PGPOOL_VBASE
#    error CONFIG_ARCH_PGPOOL_VBASE not defined
#  endif

#  ifndef CONFIG_ARCH_PGPOOL_SIZE
#    error CONFIG_ARCH_PGPOOL_SIZE not defined
#  endif

#  define CONFIG_ARCH_PGPOOL_PEND \
     (CONFIG_ARCH_PGPOOL_PBASE + CONFIG_ARCH_PGPOOL_SIZE)
#  define CONFIG_ARCH_PGPOOL_VEND \
     (CONFIG_ARCH_PGPOOL_VBASE + CONFIG_ARCH_PGPOOL_SIZE)

#endif
/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct tcb_s;                  /* Forward reference to TCB */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct addrenv_s
{
  struct arch_addrenv_s addrenv; /* The address environment page directory  */
  struct work_s         work;    /* Worker to free address environment      */
  int                   refs;    /* Users of address environment            */
};

typedef struct addrenv_s addrenv_t;

/* Reserved .bss/.data region.  In the kernel build (CONFIG_BUILD_KERNEL),
 * the region at the beginning of the .bss/.data region is reserved for use
 * by the OS.  This reserved region contains support for:
 *
 *   1. The task group's heap memory management data structures and
 *   2. Support for delivery of signals.
 */

#ifdef CONFIG_BUILD_KERNEL
/* This is the type of the signal handler trampoline routine.  This
 * trampoline is called directly from kernel logic.  It simply forwards the
 * signal information to the real signal handler.  When the signal handler
 * returns, this function issues a system call in order to return in kernel
 * mode.
 */

typedef CODE void (*addrenv_sigtramp_t)(_sa_sigaction_t sighand, int signo,
                                        FAR siginfo_t *info,
                                        FAR void *ucontext);

/* This structure describes the format of the .bss/.data reserved area */

struct addrenv_reserve_s
{
  addrenv_sigtramp_t ar_sigtramp;  /* Signal trampoline */
  struct mm_heap_s  *ar_usrheap;   /* User space heap structure */
};

/* Each instance of this structure resides at the beginning of the user-
 * space .bss/.data region.  This macro is provided to simplify access:
 */

#define ARCH_DATA_RESERVE \
  ((FAR struct addrenv_reserve_s *)CONFIG_ARCH_DATA_VBASE)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: addrenv_allocate
 *
 * Description:
 *   Allocate an address environment for a new process.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Pointer to the new address environment, or NULL if out of memory.
 *
 ****************************************************************************/

FAR struct addrenv_s *addrenv_allocate(void);

/****************************************************************************
 * Name: addrenv_switch
 *
 * Description:
 *   Switch to an address environment.
 *
 * Input Parameters:
 *   tcb - The tcb of the task to switch to, or NULL to use the task at the
 *         head of the ready-to-run list.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int addrenv_switch(FAR struct tcb_s *tcb);

/****************************************************************************
 * Name: addrenv_attach
 *
 * Description:
 *   Attach address environment to a newly process. Called by exec() right
 *   before injecting the new process into the system.
 *
 * Input Parameters:
 *   tcb     - The tcb of the newly loaded task.
 *   addrenv - The address environment that is attached.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int addrenv_attach(FAR struct tcb_s *tcb, FAR struct addrenv_s *addrenv);

/****************************************************************************
 * Name: addrenv_join
 *
 * Description:
 *   Join the parent process's address environment.
 *
 * Input Parameters:
 *   ptcb - The tcb of the parent process.
 *   tcb  - The tcb of the child process.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int addrenv_join(FAR struct tcb_s *ptcb, FAR struct tcb_s *tcb);

/****************************************************************************
 * Name: addrenv_leave
 *
 * Description:
 *   Leave a process's address environment.
 *
 * Input Parameters:
 *   tcb  - The tcb of the process.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int addrenv_leave(FAR struct tcb_s *tcb);

/****************************************************************************
 * Name: addrenv_select
 *
 * Description:
 *   Temporarily select a different address environment for the currently
 *   running process.
 *
 * Input Parameters:
 *   addrenv - The address environment.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int addrenv_select(FAR struct addrenv_s *addrenv);

/****************************************************************************
 * Name: addrenv_restore
 *
 * Description:
 *   Switch back to the procces's own address environment.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int addrenv_restore(void);

/****************************************************************************
 * Name: addrenv_take
 *
 * Description:
 *   Take a reference to an address environment.
 *
 * Input Parameters:
 *   addrenv - The address environment.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void addrenv_take(FAR struct addrenv_s *addrenv);

/****************************************************************************
 * Name: addrenv_give
 *
 * Description:
 *   Give back a reference to an address environment, obtaining the resulting
 *   reference counter as returned value.
 *
 * Input Parameters:
 *   addrenv - The address environment.
 *
 * Returned Value:
 *   Remaining reference count.
 *
 ****************************************************************************/

int addrenv_give(FAR struct addrenv_s *addrenv);

/****************************************************************************
 * Name: addrenv_drop
 *
 * Description:
 *   Drop an address environment.
 *
 * Input Parameters:
 *   addrenv - The address environment.
 *   deferred - yes: The address environment should be dropped by the worker
 *              no:  The address environment can be dropped at once
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void addrenv_drop(FAR struct addrenv_s *addrenv, bool deferred);

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
 *   up_addrenv_vheap    - Returns the virtual base address of the heap
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
 * CONFIG_ARCH_STACK_DYNAMIC=y indicates that the user process stack resides
 * in its own address space.  This options is also *required* if
 * CONFIG_BUILD_KERNEL and CONFIG_LIBC_EXECFUNCS are selected.  Why?
 * Because the caller's stack must be preserved in its own address space
 * when we instantiate the environment of the new process in order to
 * initialize it.
 *
 * NOTE: The naming of the CONFIG_ARCH_STACK_DYNAMIC selection implies that
 * dynamic stack allocation is supported.  Certainly this option must be set
 * if dynamic stack allocation is supported by a platform.  But the more
 * general meaning of this configuration environment is simply that the
 * stack has its own address space.
 *
 * If CONFIG_ARCH_STACK_DYNAMIC=y is selected then the platform specific
 * code must export these additional interfaces:
 *
 *   up_addrenv_ustackalloc  - Create a stack address environment
 *   up_addrenv_ustackfree   - Destroy a stack address environment.
 *   up_addrenv_vustack      - Returns the virtual base address of the stack
 *   up_addrenv_ustackselect - Instantiate a stack address environment
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
 * If CONFIG_ARCH_STACK_DYNAMIC=y is selected then the platform specific
 * code must export these additional interfaces:
 *
 *   up_addrenv_kstackalloc  - Create a stack in the kernel address
 *                             environment
 *   up_addrenv_kstackfree   - Destroy the kernel stack.
 *
 ****************************************************************************/

/* Prototyped in include/nuttx/arch.h as part of the OS/platform interface */

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_ARCH_ADDRENV */
#endif /* __INCLUDE_NUTTX_ADDRENV_H */
