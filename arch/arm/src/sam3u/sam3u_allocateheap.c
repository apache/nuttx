/****************************************************************************
 * arch/arm/src/common/sam3u_allocateheap.c
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "sam3u_internal.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#if CONFIG_MM_REGIONS < 2
#  warning "CONFIG_MM_REGIONS < 2: SRAM1 not included in HEAP"
#endif

#if CONFIG_MM_REGIONS < 3 && !defined(CONFIG_SAM3U_NAND)
#  warning "CONFIG_MM_REGIONS < 3: NFC SRAM not included in HEAP"
#endif

#if CONFIG_MM_REGIONS > 2 && defined(CONFIG_SAM3U_NAND)
#  error "CONFIG_MM_REGIONS > 3 but cannot used NFC SRAM"
#  undef CONFIG_MM_REGIONS
#  define CONFIG_MM_REGIONS 2
#endif

#if CONFIG_DRAM_END > (SAM3U_INTSRAM0_BASE+CONFIG_SAM3U_SRAM0_SIZE)
#  error "CONFIG_DRAM_END is beyond the end of SRAM0"
#  undef CONFIG_DRAM_END
#  define CONFIG_DRAM_END (SAM3U_INTSRAM0_BASE+CONFIG_SAM3U_SRAM0_SIZE)
#elif CONFIG_DRAM_END < (SAM3U_INTSRAM0_BASE+CONFIG_SAM3U_SRAM0_SIZE)
#  warning "CONFIG_DRAM_END is before end of SRAM0... not all of SRAM0 used"
#endif

#ifdef CONFIG_MM_KERNEL_HEAPSIZE
#  if CONFIG_MM_KERNEL_HEAPSIZE < (1 << 5)     /* Kernel heap size < 2**5 */
#    define KHEAP_SIZE (1 << 4)                /*   Use size 2**4 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 6)   /* Kernel heap size < 2**6 */
#    define KHEAP_SIZE (1 << 5)                /*   Use size 2**5 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 7)   /* Kernel heap size < 2**7 */
#    define KHEAP_SIZE (1 << 6)                /*   Use size 2**6 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 8)   /* Kernel heap size < 2**8 */
#    define KHEAP_SIZE (1 << 7)                /*   Use size 2**7 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 9)   /* Kernel heap size < 2**9 */
#    define KHEAP_SIZE (1 << 8)                /*   Use size 2**8 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 10)  /* Kernel heap size < 2**10 */
#    define KHEAP_SIZE (1 << 9)                /*   Use size 2**9 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 11)  /* Kernel heap size < 2**11 */
#    define KHEAP_SIZE (1 << 10)               /*   Use size 2**10 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 12)  /* Kernel heap size < 2**12 */
#    define KHEAP_SIZE (1 << 11)               /*   Use size 2**11 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 13)  /* Kernel heap size < 2**13 */
#    define KHEAP_SIZE (1 << 12)               /*   Use size 2**12 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 14)  /* Kernel heap size < 2**14 */
#    define KHEAP_SIZE (1 << 13)               /*   Use size 2**13 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 15)  /* Kernel heap size < 2**15 */
#    define KHEAP_SIZE (1 << 14)               /*   Use size 2**14 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 16)  /* Kernel heap size < 2**16 */
#    define KHEAP_SIZE (1 << 15)               /*   Use size 2**15 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 17)  /* Kernel heap size < 2**17 */
#    define KHEAP_SIZE (1 << 16)               /*   Use size 2**16 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 18)  /* Kernel heap size < 2**18 */
#    define KHEAP_SIZE (1 << 17)               /*   Use size 2**17 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 19)  /* Kernel heap size < 2**19 */
#    define KHEAP_SIZE (1 << 18)               /*   Use size 2**18 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 20)  /* Kernel heap size < 2**20 */
#    define KHEAP_SIZE (1 << 19)               /*   Use size 2**19 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 21)  /* Kernel heap size < 2**21 */
#    define KHEAP_SIZE (1 << 20)               /*   Use size 2**20 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 22)  /* Kernel heap size < 2**22 */
#    define KHEAP_SIZE (1 << 21)               /*   Use size 2**21 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 23)  /* Kernel heap size < 2**23 */
#    define KHEAP_SIZE (1 << 22)               /*   Use size 2**22 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 24)  /* Kernel heap size < 2**24 */
#    define KHEAP_SIZE (1 << 23)               /*   Use size 2**23 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 25)  /* Kernel heap size < 2**25 */
#    define KHEAP_SIZE (1 << 24)               /*   Use size 2**24 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 26)  /* Kernel heap size < 2**26 */
#    define KHEAP_SIZE (1 << 25)               /*   Use size 2**25 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 27)  /* Kernel heap size < 2**27 */
#    define KHEAP_SIZE (1 << 26)               /*   Use size 2**26 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 28)  /* Kernel heap size < 2**28 */
#    define KHEAP_SIZE (1 << 27)               /*   Use size 2**27 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 29)  /* Kernel heap size < 2**29 */
#    define KHEAP_SIZE (1 << 28)               /*   Use size 2**28 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 30)  /* Kernel heap size < 2**30 */
#    define KHEAP_SIZE (1 << 29)               /*   Use size 2**29 */
#  elif CONFIG_MM_KERNEL_HEAPSIZE < (1 << 31)  /* Kernel heap size < 2**31 */
#    define KHEAP_SIZE (1 << 30)               /*   Use size 2**30 */
#  else
#    define KHEAP_SIZE (1 << 31)               /*   Use size 2**31 */
#  endif

#  define KHEAP_MASK   (KHEAP_SIZE - 1)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   For the kernel build (CONFIG_NUTTX_KERNEL=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function provides the
 *   size of the unprotected, user-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 ****************************************************************************/

void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
{
#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
  uintptr_t kbase = ((uintptr_t)g_heapbase + KHEAP_MASK) & ~KHEAP_MASK;
  uintptr_t ubase = kbase + KHEAP_SIZE;
  size_t    usize = CONFIG_DRAM_END - ubase;

  DEBUGASSERT(ubase < (uintptr_t)CONFIG_DRAM_END);

  /* Return the heap settings */

  up_ledon(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)ubase;
  *heap_size  = usize;

  /* Allow access to the heap memory */

   sam3u_mpu_uheap((uintptr_t)ubase, usize);
#else

  size_t size = CONFIG_DRAM_END - g_heapbase;

  /* Return the heap settings */

  up_ledon(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)g_heapbase;
  *heap_size  = size;

  /* Allow user access to the user heap memory */

   sam3u_mpu_uheap((uintptr_t)g_heapbase, size);
   
#endif
}

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel build (CONFIG_NUTTX_KERNEL=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   (and protects) the kernel-space heap.
 *
 ****************************************************************************/

#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
void up_allocate_kheap(FAR void **heap_start, size_t *heap_size)
{
  uintptr_t kbase = ((uintptr_t)g_heapbase + KHEAP_MASK) & ~KHEAP_MASK;

  DEBUGASSERT((kbase + KHEAP_SIZE) < (uintptr_t)CONFIG_DRAM_END);

  /* Return the heap settings */

  *heap_start = (FAR void*)kbase;
  *heap_size  = KHEAP_SIZE;
}
#endif

/************************************************************************
 * Name: up_addregion
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ************************************************************************/

#if CONFIG_MM_REGIONS > 1
void up_addregion(void)
{
  /* Add the region */

  kmm_addregion((FAR void*)SAM3U_INTSRAM1_BASE, CONFIG_SAM3U_SRAM1_SIZE);

  /* Allow user access to the heap memory */

  sam3u_mpu_uheap(SAM3U_INTSRAM1_BASE, CONFIG_SAM3U_SRAM1_SIZE);

  /* Add the region */

#if CONFIG_MM_REGIONS > 2
  kmm_addregion((FAR void*)SAM3U_NFCSRAM_BASE, CONFIG_SAM3U_NFCSRAM_SIZE);

  /* Allow user access to the heap memory */

  sam3u_mpu_uheap(SAM3U_NFCSRAM_BASE, CONFIG_SAM3U_NFCSRAM_SIZE);
#endif
}
#endif
