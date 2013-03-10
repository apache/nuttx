/************************************************************************
 * mm/mm_kernel.c
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

#include <nuttx/kmalloc.h>

#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP) && defined(__KERNEL__)

/************************************************************************
 * Pre-processor definition
 ************************************************************************/

/************************************************************************
 * Private Types
 ************************************************************************/

/************************************************************************
 * Public Data
 ************************************************************************/

/* This is the kernel heap */

struct mm_heap_s g_kmmheap;

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: kmm_initialize
 *
 * Description:
 *   Initialize the kernel heap data structures, providing the initial
 *   heap region.
 *
 * Parameters:
 *   heap_start - Address of the beginning of the (initial) memory region
 *   heap_size  - The size (in bytes) if the (initial) memory region.
 *
 * Return Value:
 *   None
 *
 ************************************************************************/

void kmm_initialize(FAR void *heap_start, size_t heap_size)
{
  return mm_initialize(&g_kmmheap, heap_start, heap_size);
}

/************************************************************************
 * Name: kmm_addregion
 *
 * Description:
 *   This function adds a region of contiguous memory to the kernel heap.
 *
 * Parameters:
 *   heap_start - Address of the beginning of the memory region
 *   heap_size  - The size (in bytes) if the memory region.
 *
 * Return Value:
 *   None
 *
 ************************************************************************/

void kmm_addregion(FAR void *heap_start, size_t heap_size)
{
  return mm_addregion(&g_kmmheap, heap_start, heap_size);
}

/************************************************************************
 * Name: kmalloc
 *
 * Description:
 *   Allocate memory from the kernel heap.
 *
 * Parameters:
 *   size - Size (in bytes) of the memory region to be allocated.
 *
 * Return Value:
 *   The address of the allocated memory (NULL on failure to allocate)
 *
 ************************************************************************/

FAR void *kmalloc(size_t size)
{
  return mm_malloc(&g_kmmheap, size);
}

/************************************************************************
 * Name: kzalloc
 *
 * Description:
 *   Allocate and zero memory from the kernel heap.
 *
 * Parameters:
 *   size - Size (in bytes) of the memory region to be allocated.
 *
 * Return Value:
 *   The address of the allocated memory (NULL on failure to allocate)
 *
 ************************************************************************/

FAR void *kzalloc(size_t size)
{
  return mm_zalloc(&g_kmmheap, size);
}

/************************************************************************
 * Name: krealloc
 *
 * Description:
 *   Re-allocate memory in the kernel heap.
 *
 * Parameters:
 *   oldmem  - The old memory allocated
 *   newsize - Size (in bytes) of the new memory region to be re-allocated.
 *
 * Return Value:
 *   The address of the re-allocated memory (NULL on failure to re-allocate)
 *
 ************************************************************************/

FAR void *krealloc(FAR void *oldmem, size_t newsize)
{
  return mm_realloc(&g_kmmheap, oldmem, newsize);
}

/************************************************************************
 * Name: kfree
 *
 * Description:
 *   Return allocated memory to the kernel heap.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 ************************************************************************/

void kfree(FAR void *mem)
{
  DEBUGASSERT(kmm_heapmember(mem));
  return mm_free(&g_kmmheap, mem);
}

/************************************************************************
 * Name: kmm_trysemaphore
 *
 * Description:
 *   Try to take the kernel heap semaphore.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   OK on success; a negated errno on failure
 *
 ************************************************************************/

int kmm_trysemaphore(void)
{
  return mm_trysemaphore(&g_kmmheap);
}

/************************************************************************
 * Name: kmm_givesemaphore
 *
 * Description:
 *   Give the kernel heap semaphore.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   OK on success; a negated errno on failure
 *
 ************************************************************************/

void kmm_givesemaphore(void)
{
  return mm_givesemaphore(&g_kmmheap);
}

/************************************************************************
 * Name: kmm_heapmember
 *
 * Description:
 *   Check if an address lies in the kernel heap.
 *
 * Parameters:
 *   mem - The address to check
 *
 * Return Value:
 *   true if the address is a member of the kernel heap.  false if not
 *   not.  If the address is not a member of the kernel heap, then it
 *   must be a member of the user-space heap (unchecked)
 *
 ************************************************************************/

#ifdef CONFIG_DEBUG
bool kmm_heapmember(FAR void *mem)
{
#if CONFIG_MM_REGIONS > 1
  int i;

  /* A valid address from the kernel heap for this region would have to lie
   * between the region's two guard nodes.
   */

  for (i = 0; i < g_kmmheap.mm_nregions; i++)
    {
      if (mem > (FAR void *)g_kmmheap.mm_heapstart[i] &&
          mem < (FAR void *)g_kmmheap.mm_heapend[i])
        {
          return true;
        }
    }

  /* The address does not like any any region assigned to kernel heap */

  return false;

#else
  /* A valid address from the kernel heap would have to lie between the
   * two guard nodes.
   */

  if (mem > (FAR void *)g_kmmheap.mm_heapstart[0] &&
      mem < (FAR void *)g_kmmheap.mm_heapend[0])
    {
      return true;
    }

  /* Otherwise, the address does not lie in the kernel heap */

  return false;

#endif
}
#endif

#endif /* CONFIG_NUTTX_KERNEL && CONFIG_MM_KERNEL_HEAP && __KERNEL__ */
