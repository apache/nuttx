/****************************************************************************
 * include/nuttx/kmalloc.h
 *
 *   Copyright (C) 2007-2008, 2011, 2013, 2016, 2018 Gregory Nutt. All
 *     rights reserved.
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

#ifndef __INCLUDE_NUTTX_KMALLOC_H
#define __INCLUDE_NUTTX_KMALLOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdlib.h>

#include <nuttx/mm/mm.h>
#include <nuttx/userspace.h>

#if !defined(CONFIG_BUILD_PROTECTED) || defined(__KERNEL__)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef KMALLOC_EXTERN
#if defined(__cplusplus)
#  define KMALLOC_EXTERN extern "C"
extern "C"
{
#else
#  define KMALLOC_EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* For a monolithic, kernel-mode NuttX build.  Special allocators must be
 * used.  Otherwise, the standard allocators prototyped in stdlib.h may
 * be used for both the kernel- and user-mode objects.
 */

/* This family of allocators is used to manage user-accessible memory
 * from the kernel.  In the flat build, the following are declared in
 * stdlib.h and are directly callable.  In the kernel-phase of the kernel
 * build, the following are defined in userspace.h as macros that call
 * into user-space via a header at the beginning of the user-space blob.
 */

#define kumm_initialize(h,s)     umm_initialize(h,s)
#define kumm_addregion(h,s)      umm_addregion(h,s)
#define kumm_trysemaphore()      umm_trysemaphore()
#define kumm_givesemaphore()     umm_givesemaphore()

#define kumm_calloc(n,s)         calloc(n,s);
#define kumm_malloc(s)           malloc(s)
#define kumm_zalloc(s)           zalloc(s)
#define kumm_realloc(p,s)        realloc(p,s)
#define kumm_memalign(a,s)       memalign(a,s)
#define kumm_free(p)             free(p)
#ifdef CONFIG_CAN_PASS_STRUCTS
#  define kumm_mallinfo()        mallinfo()
#else
#  define kumm_mallinfo(i)       mallinfo(i)
#endif

/* This family of allocators is used to manage kernel protected memory */

#if !defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_MM_KERNEL_HEAP)
/* If this is not a kernel build, then these map to the same interfaces
 * as were used for the user-mode function.
 */

#  define kmm_initialize(h,s)    /* Initialization done by kumm_initialize */
#  define kmm_addregion(h,s)     umm_addregion(h,s)
#  define kmm_trysemaphore()     umm_trysemaphore()
#  define kmm_givesemaphore()    umm_givesemaphore()

#  define kmm_calloc(n,s)        calloc(n,s);
#  define kmm_malloc(s)          malloc(s)
#  define kmm_zalloc(s)          zalloc(s)
#  define kmm_realloc(p,s)       realloc(p,s)
#  define kmm_memalign(a,s)      memalign(a,s)
#  define kmm_free(p)            free(p)
#ifdef CONFIG_CAN_PASS_STRUCTS
#  define kmm_mallinfo()         mallinfo()
#else
#  define kmm_mallinfo(i)        mallinfo(i)
#endif

#elif !defined(CONFIG_MM_KERNEL_HEAP)
/* If this the kernel phase of a kernel build, and there are only user-space
 * allocators, then the following are defined in userspace.h as macros that
 * call into user-space via a header at the beginning of the user-space blob.
 */

#  define kmm_initialize(h,s)    /* Initialization done by kumm_initialize */
#  define kmm_addregion(h,s)     umm_addregion(h,s)
#  define kmm_trysemaphore()     umm_trysemaphore()
#  define kmm_givesemaphore()    umm_givesemaphore()

#  define kmm_calloc(n,s)        calloc(n,s);
#  define kmm_malloc(s)          malloc(s)
#  define kmm_zalloc(s)          zalloc(s)
#  define kmm_realloc(p,s)       realloc(p,s)
#  define kmm_memalign(a,s)      memalign(a,s)
#  define kmm_free(p)            free(p)
#ifdef CONFIG_CAN_PASS_STRUCTS
#  define kmm_mallinfo()         mallinfo()
#else
#  define kmm_mallinfo(i)        mallinfo(i)
#endif

#else
/* Otherwise, the kernel-space allocators are declared in include/nuttx/mm/mm.h
 * and we can call them directly.
 */

#endif

#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
     defined(CONFIG_MM_KERNEL_HEAP)
/****************************************************************************
 * Group memory management
 *
 *   Manage memory allocations appropriately for the group type.  If the
 *   memory is part of a privileged group, then it should be allocated so
 *   that it is only accessible by privileged code;  Otherwise, it is a
 *   user mode group and must be allocated so that it accessible by
 *   unprivileged code.
 *
 ****************************************************************************/
/* Functions defined in group/group_malloc.c ********************************/

FAR void *group_malloc(FAR struct task_group_s *group, size_t nbytes);

/* Functions defined in group/group_zalloc.c ********************************/

FAR void *group_zalloc(FAR struct task_group_s *group, size_t nbytes);

/* Functions defined in group/group_free.c **********************************/

void group_free(FAR struct task_group_s *group, FAR void *mem);

#else
  /* In the flat build, there is only one memory allocator and no distinction
   * in privileges.
   */

#  define group_malloc(g,n) kumm_malloc(n)
#  define group_zalloc(g,n) kumm_zalloc(n)
#  define group_free(g,m)   kumm_free(m)

#endif

/* Functions defined in sched/sched_kfree.c **********************************/

/* Handles memory freed from an interrupt handler.  In that context, kmm_free()
 * (or kumm_free()) cannot be called.  Instead, the allocations are saved in a
 * list of delayed allocations that will be periodically cleaned up by
 * sched_garbage_collection().
 */

void sched_ufree(FAR void *address);

#if defined(CONFIG_MM_KERNEL_HEAP) && defined(__KERNEL__)
void sched_kfree(FAR void *address);
#else
#  define sched_kfree(a) sched_ufree(a)
#endif

/* Signal the worker thread that is has some clean up to do */

void sched_signal_free(void);

/* Functions defined in sched/sched_garbage *********************************/

/* Must be called periodically to clean up deallocations delayed by
 * sched_kmm_free().  This may be done from either the IDLE thread or from a
 * worker thread.  The IDLE thread has very low priority and could starve
 * the system for memory in some context.
 */

void sched_garbage_collection(void);

/* Is is not a good idea for the IDLE threads to take the KMM semaphore.
 * That can cause the IDLE thread to take processing time from higher
 * priority tasks.  The IDLE threads will only take the KMM semaphore if
 * there is garbage to be collected.
 *
 * Certainly there is a race condition involved in sampling the garbage
 * state.  The looping nature of the IDLE loops should catch any missed
 * garbage from the test on the next time arround.
 */

bool sched_have_garbage(void);

#undef KMALLOC_EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* !CONFIG_BUILD_PROTECTED || __KERNEL__ */
#endif /* __INCLUDE_NUTTX_KMALLOC_H */
