/************************************************************************
 * mm/mm_user.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#include <nuttx/mm.h>

#if !defined(CONFIG_NUTTX_KERNEL) || !defined(__KERNEL__)

/************************************************************************
 * Pre-processor definition
 ************************************************************************/

/************************************************************************
 * Private Types
 ************************************************************************/

/************************************************************************
 * Public Data
 ************************************************************************/

/* This is the user heap */

struct mm_heap_s g_mmheap;

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: umm_initialize
 *
 * Description:
 *   This is a simple wrapper for the mm_initialize() function.  This
 *   function is exported from the user-space blob so that the kernel
 *   can initialize the user-mode allocator.
 *
 * Parameters:
 *   heap_start - Address of the beginning of the (initial) memory region
 *   heap_size  - The size (in bytes) if the (initial) memory region.
 *
 * Return Value:
 *   None
 *
 ************************************************************************/

void umm_initialize(FAR void *heap_start, size_t heap_size)
{
  mm_initialize(&g_mmheap, heap_start, heap_size);
}

/************************************************************************
 * Name: umm_addregion
 *
 * Description:
 *   This is a simple wrapper for the mm_addregion() function.  This
 *   function is exported from the user-space blob so that the kernel
 *   can initialize the user-mode allocator.
 *
 * Parameters:
 *   heap_start - Address of the beginning of the memory region
 *   heap_size  - The size (in bytes) if the memory region.
 *
 * Return Value:
 *   None
 *
 ************************************************************************/

void umm_addregion(FAR void *heap_start, size_t heap_size)
{
  mm_addregion(&g_mmheap, heap_start, heap_size);
}

/************************************************************************
 * Name: umm_trysemaphore
 *
 * Description:
 *   This is a simple wrapper for the mm_trysemaphore() function.  This
 *   function is exported from the user-space blob so that the kernel
 *   can manage the user-mode allocator.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   OK on success; a negated errno on failure
 *
 ************************************************************************/

int umm_trysemaphore(void)
{
  return mm_trysemaphore(&g_mmheap);
}

/************************************************************************
 * Name: umm_givesemaphore
 *
 * Description:
 *   This is a simple wrapper for the mm_givesemaphore() function.  This
 *   function is exported from the user-space blob so that the kernel
 *   can manage the user-mode allocator.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   OK on success; a negated errno on failure
 *
 ************************************************************************/

void umm_givesemaphore(void)
{
  mm_givesemaphore(&g_mmheap);
}

#endif /* !CONFIG_NUTTX_KERNEL || !__KERNEL__ */
