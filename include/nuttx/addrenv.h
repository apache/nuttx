/****************************************************************************
 * include/nuttx/addrenv.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_ADDRENV_H
#define __INCLUDE_NUTTX_ADDRENV_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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
#  define CONFIG_ARCH_DATA_VBASE (CONFIG_ARCH_TEXT_VBASE + ARCH_TEXT_SIZE)
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

/* Heap region */

#ifndef CONFIG_ARCH_HEAP_VBASE
#  error CONFIG_ARCH_HEAP_VBASE not defined
#  define CONFIG_ARCH_HEAP_VBASE (CONFIG_ARCH_DATA_VBASE + ARCH_DATA_SIZE)
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

/* Stack region */

#ifndef CONFIG_ARCH_STACK_VBASE
#  error CONFIG_ARCH_STACK_VBASE not defined
#  define CONFIG_ARCH_STACK_VBASE (CONFIG_ARCH_HEAP_VBASE + ARCH_HEAP_SIZE)
#endif

#if (CONFIG_ARCH_STACK_VBASE & CONFIG_MM_MASK) != 0
#  error CONFIG_ARCH_STACK_VBASE not aligned to page boundary
#endif

#ifndef CONFIG_ARCH_STACK_NPAGES
#  warning CONFIG_ARCH_STACK_NPAGES not defined
#  define CONFIG_ARCH_STACK_NPAGES 1
#endif

#define ARCH_STACK_SIZE (CONFIG_ARCH_STACK_NPAGES * CONFIG_MM_PGSIZE)
#define ARCH_STACK_VEND (CONFIG_ARCH_STACK_VBASE + ARCH_STACK_SIZE)

/* A single page scratch region used for temporary mappings */

#define ARCH_SCRATCH_VBASE (CONFIG_ARCH_STACK_VBASE + ARCH_STACK_SIZE)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Address Environment Interfaces
 *
 * Low-level interfaces used in binfmt/ to instantiate tasks with address
 * environments.  These interfaces all operate on type group_addrenv_t which
 * is an abstract representation of a task group's address environment and
 * must be defined in arch/arch.h if CONFIG_ARCH_ADDRENV is defined.
 *
 *   up_addrenv_create  - Create an address environment
 *   up_addrenv_destroy - Destroy an address environment.
 *   up_addrenv_vtext   - Returns the virtual base address of the .text
 *                        address environment
 *   up_addrenv_vdata   - Returns the virtual base address of the .bss/.data
 *                        address environment
 *   up_addrenv_select  - Instantiate an address environment
 *   up_addrenv_restore - Restore an address environment
 *   up_addrenv_clone   - Copy an address environment from one location to
 *                        another.
 *
 * Higher-level interfaces used by the tasking logic.  These interfaces are
 * used by the functions in sched/ and all operate on the thread which whose
 * group been assigned an address environment by up_addrenv_clone().
 *
 *   up_addrenv_attach  - Clone the address environment assigned to one TCB
 *                        to another.  This operation is done when a pthread
 *                        is created that share's the same address
 *                        environment.
 *   up_addrenv_detach  - Release the threads reference to an address
 *                        environment when a task/thread exits.
 *
 ****************************************************************************/

/* Prototyped in include/nuttx/arch.h as part of the OS/platform interface */

#endif /* CONFIG_ARCH_ADDRENV */
#endif /* __INCLUDE_NUTTX_ADDRENV_H */
