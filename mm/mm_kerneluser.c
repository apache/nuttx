/************************************************************************
 * mm/mm_kerneluser.c
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <assert.h>

#include <nuttx/kmalloc.h>

#if defined(CONFIG_NUTTX_KERNEL) && defined(__KERNEL__)

/* This logic is all tentatively and, hopefully, will grow in usability.
 * For now, the kernel-mode build uses the memory manager that is
 * provided in the user-space build.  That is awkward but reasonable for
 * the current level of support:  At present, only memory protection is
 * provided.  Kernel-mode code may call into user-mode code, but not
 * vice-versa.  So hosting the memory manager in user-space allows the
 * memory manager to be shared in both kernel- and user-mode spaces.
 *
 * In the longer run, if an MMU is support that can provide virtualized
 * memory, then some SLAB memory manager will be required in kernel-space
 * with some kind of brk() system call to obtain mapped heap space.
 *
 * In the current build model, the user-space module is built first. The
 * file user_map.h is generated in the first pass and contains the
 * addresses of the memory manager needed in this file:
 */

#include <arch/board/user_map.h>

/************************************************************************
 * Pre-processor definition
 ************************************************************************/

/* These values are obtained from user_map.h */

#define KINITIALIZE(h,s) ((kminitialize_t)CONFIG_USER_MMINIT)(h,s)
#define KADDREGION(h,s)  ((kmaddregion_t)CONFIG_USER_MMADDREGION)(h,s)
#define KMALLOC(s)       ((kmalloc_t)CONFIG_USER_MALLOC)(s)
#define KZALLOC(s)       ((kzalloc_t)CONFIG_USER_ZALLOC)(s)
#define KREALLOC(p,s)    ((krealloc_t)CONFIG_USER_REALLOC)(p,s)
#define KFREE(p)         ((kfree_t)CONFIG_USER_FREE)(p)
#define KTRYSEMAPHORE()  ((kmtrysemaphore_t) CONFIG_USER_MMTRYSEM )()
#define KGIVESEMAPHORE() ((kmgivesemaphore_t)CONFIG_USER_MMGIVESEM)()

/************************************************************************
 * Private Types
 ************************************************************************/

typedef void (*kminitialize_t)(FAR void*, size_t);
typedef void (*kmaddregion_t)(FAR void*, size_t);
typedef FAR void *(*kmalloc_t)(size_t);
typedef FAR void *(*kzalloc_t)(size_t);
typedef FAR void *(*krealloc_t)(FAR void*, size_t);
typedef void (*kfree_t)(FAR void *);
typedef int  (*kmtrysemaphore_t)(void);
typedef void (*kmgivesemaphore_t)(void);

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: kumm_initialize
 *
 * Description:
 *   This is a simple redirection to the user-space mm_initialize()
 *   function.
 *
 * Parameters:
 *   heap_start - Address of the beginning of the (initial) memory region
 *   heap_size  - The size (in bytes) if the (initial) memory region.
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   1. mm_initialize() resides in user-space
 *   2. The address of the user space mm_initialize() is provided in
 *      user_map.h
 *   3. The user-space mm_initialize() is callable from kernel-space.
 *
 ************************************************************************/

void kumm_initialize(FAR void *heap_start, size_t heap_size)
{
  return KINITIALIZE(heap_start, heap_size);
}

/************************************************************************
 * Name: kumm_addregion
 *
 * Description:
 *   This is a simple redirection to the user-space mm_addregion()
 *   function.
 *
 * Parameters:
 *   heap_start - Address of the beginning of the memory region
 *   heap_size  - The size (in bytes) if the memory region.
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   1. mm_addregion() resides in user-space
 *   2. The address of the user space mm_addregion() is provided in
 *      user_map.h
 *   3. The user-space mm_addregion() is callable from kernel-space.
 *
 ************************************************************************/

void kumm_addregion(FAR void *heap_start, size_t heap_size)
{
  return KADDREGION(heap_start, heap_size);
}

/************************************************************************
 * Name: kumalloc
 *
 * Description:
 *   This is a simple redirection to the user-space malloc() function.
 *
 * Parameters:
 *   size - Size (in bytes) of the memory region to be allocated.
 *
 * Return Value:
 *   The address of the allocated memory (NULL on failure to allocate)
 *
 * Assumptions:
 *   1. malloc() resides in user-space
 *   2. The address of the user space malloc() is provided in user_map.h
 *   3. The user-space malloc() is callable from kernel-space.
 *
 ************************************************************************/

FAR void *kumalloc(size_t size)
{
  return KMALLOC(size);
}

/************************************************************************
 * Name: kuzalloc
 *
 * Description:
 *   This is a simple redirection to the user-space zalloc() function.
 *
 * Parameters:
 *   size - Size (in bytes) of the memory region to be allocated.
 *
 * Return Value:
 *   The address of the allocated memory (NULL on failure to allocate)
 *
 * Assumptions:
 *   1. zalloc() resides in user-space
 *   2. The address of the user space zalloc() is provided in user_map.h
 *   3. The user-space zalloc() is callable from kernel-space.
 *
 ************************************************************************/

FAR void *kuzalloc(size_t size)
{
  return KZALLOC(size);
}

/************************************************************************
 * Name: kurealloc
 *
 * Description:
 *   This is a simple redirection to the user-space realloc() function.
 *
 * Parameters:
 *   oldmem  - The old memory allocated
 *   newsize - Size (in bytes) of the new memory region to be re-allocated.
 *
 * Return Value:
 *   The address of the re-allocated memory (NULL on failure to re-allocate)
 *
 * Assumptions:
 *   1. realloc() resides in user-space
 *   2. The address of the user space realloc() is provided in user_map.h
 *   3. The user-space realloc() is callable from kernel-space.
 *
 ************************************************************************/

FAR void *kurealloc(FAR void *oldmem, size_t newsize)
{
  return KREALLOC(oldmem, newsize);
}

/************************************************************************
 * Name: kufree
 *
 * Description:
 *   This is a simple redirection to the user-space free() function.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   1. free() resides in user-space
 *   2. The address of the user space free() is provided in user_map.h
 *   3. The user-space free() is callable from kernel-space.
 *
 ************************************************************************/

void kufree(FAR void *mem)
{
#if defined(CONFIG_MM_KERNEL_HEAP) && defined(CONFIG_DEBUG)
  DEBUGASSERT(!kmm_heapmember(mem));
#endif
  return KFREE(mem);
}

/************************************************************************
 * Name: kumm_trysemaphore
 *
 * Description:
 *   This is a simple redirection to the user-space mm_trysemaphore()
 *   function.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   1. mm_trysemaphore() resides in user-space
 *   2. The address of the user space mm_trysemaphore() is provided in
 *      user_map.h
 *   3. The user-space mm_semaphore() is callable from kernel-space.
 *
 ************************************************************************/

int kumm_trysemaphore(void)
{
  return KTRYSEMAPHORE();
}

/************************************************************************
 * Name: kumm_givesemaphore
 *
 * Description:
 *   This is a simple redirection to the user-space mm_givesemaphore()
 *   function.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   1. mm_givesemaphore() resides in user-space
 *   2. The address of the user space mm_givesemaphore() is provided in
 *      user_map.h
 *   3. The user-space mm_semaphore() is callable from kernel-space.
 *
 ************************************************************************/

void kumm_givesemaphore(void)
{
  KGIVESEMAPHORE();
}

#endif /* CONFIG_NUTTX_KERNEL && __KERNEL__ */
