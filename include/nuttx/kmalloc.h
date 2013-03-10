/****************************************************************************
 * include/nuttx/kmalloc.h
 *
 *   Copyright (C) 2007-2008, 2011, 2013 Gregory Nutt. All rights reserved.
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
#include <stdlib.h>

#include <nuttx/mm.h>

#if !defined(CONFIG_NUTTX_KERNEL) || defined(__KERNEL__)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef KMALLOC_EXTERN
#if defined(__cplusplus)
# define KMALLOC_EXTERN extern "C"
extern "C" 
{
#else
# define KMALLOC_EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* For a monolithic, kernel-mode NuttX build.  Special allocators must be
 * used.  Otherwise, the standard allocators prototyped in stdlib.h may
 * be used for both the kernel- and user-mode objects.
 */

/* This familiy of allocators is used to manage user-accessible memory
 * from the kernel.
 */

#ifndef CONFIG_NUTTX_KERNEL

# define kumm_initialize(h,s)    umm_initialize(h,s)
# define kumm_addregion(h,s)     umm_addregion(h,s)
# define kumm_trysemaphore()     umm_trysemaphore()
# define kumm_givesemaphore()    umm_givesemaphore()

# define kumalloc(s)             malloc(s)
# define kuzalloc(s)             zalloc(s)
# define kurealloc(p,s)          realloc(p,s)
# define kufree(p)               free(p)

#else

/* This familiy of allocators is used to manage kernel protected memory */

void kumm_initialize(FAR void *heap_start, size_t heap_size);
void kumm_addregion(FAR void *heapstart, size_t heapsize);
int  kumm_trysemaphore(void);
void kumm_givesemaphore(void);

FAR void *kumalloc(size_t size);
FAR void *kuzalloc(size_t size);
FAR void *kurealloc(FAR void *oldmem, size_t newsize);
void kufree(FAR void *mem);

#endif

/* This familiy of allocators is used to manage kernel protected memory */

#ifndef CONFIG_NUTTX_KERNEL

# define kmm_initialize(h,s)    /* Initialization done by kumm_initialize */
# define kmm_addregion(h,s)     umm_addregion(h,s)
# define kmm_trysemaphore()     umm_trysemaphore()
# define kmm_givesemaphore()    umm_givesemaphore()

# define kmalloc(s)             malloc(s)
# define kzalloc(s)             zalloc(s)
# define krealloc(p,s)          realloc(p,s)
# define kfree(p)               free(p)

#elif !defined(CONFIG_MM_KERNEL_HEAP)

# define kmm_initialize(h,s)    /* Initialization done by kumm_initialize */
# define kmm_addregion(h,s)     kumm_addregion(h,s)
# define kmm_trysemaphore()     kumm_trysemaphore()
# define kmm_givesemaphore()    kumm_givesemaphore()

# define kmalloc(s)             kumalloc(s)
# define kzalloc(s)             kuzalloc(s)
# define krealloc(p,s)          kurealloc(p,s)
# define kfree(p)               kufree(p)

#else

void kmm_initialize(FAR void *heap_start, size_t heap_size);
void kmm_addregion(FAR void *heapstart, size_t heapsize);
int  kmm_trysemaphore(void);
void kmm_givesemaphore(void);

FAR void *kmalloc(size_t size);
FAR void *kzalloc(size_t size);
FAR void *krealloc(FAR void *oldmem, size_t newsize);
void kfree(FAR void *mem);

#ifdef CONFIG_DEBUG
bool kmm_heapmember(FAR void *mem);
#endif
#endif

/* Functions defined in sched/sched_kfree.c **********************************/

/* Handles memory freed from an interrupt handler.  In that context, kfree()
 * (or kufree()) cannot be called.  Instead, the allocations are saved in a
 * list of delayed allocations that will be periodically cleaned up by
 * sched_garbagecollection().
 */

void sched_ufree(FAR void *address);

#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
void sched_kfree(FAR void *address);
#else
#  define sched_kfree(a) sched_ufree(a)
#endif

/* Functions defined in sched/sched_garbage *********************************/

/* Must be called periodically to clean up deallocations delayed by
 * sched_kfree().  This may be done from either the IDLE thread or from a
 * worker thread.  The IDLE thread has very low priority and could starve
 * the system for memory in some context.
 */

void sched_garbagecollection(void);

#undef KMALLOC_EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* !CONFIG_NUTTX_KERNEL || __KERNEL__ */
#endif /* __INCLUDE_NUTTX_KMALLOC_H */
