/****************************************************************************
 * mm/kmm_heap/kmm_kernel.c
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>

#include <nuttx/kmalloc.h>

#if ((defined(CONFIG_BUILD_PROTECTED) && defined(__KERNEL__)) || \
      defined(CONFIG_BUILD_KERNEL)) && defined(CONFIG_MM_KERNEL_HEAP)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kmm_heapmember
 *
 * Description:
 *   Check if an address lies in the kernel heap.
 *
 * Input Parameters:
 *   mem - The address to check
 *
 * Returned Value:
 *   true if the address is a member of the kernel heap.  false if not
 *   not.  If the address is not a member of the kernel heap, then it
 *   must be a member of the user-space heap (unchecked)
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
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

#endif /* ((CONFIG_BUILD_PROTECTED && __KERNEL__) || CONFIG_BUILD_KERNEL)  && CONFIG_MM_KERNEL_HEAP*/
