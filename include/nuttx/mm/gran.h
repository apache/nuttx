/****************************************************************************
 * include/nuttx/mm/gran.h
 * General purpose granule memory allocator.
 *
 *   Copyright (C) 2012, 2014 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_MM_GRAN_H
#define __INCLUDE_NUTTX_MM_GRAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#ifdef CONFIG_GRAN

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_GRAN - Enable granule allocator support
 * CONFIG_GRAN_SINGLE - Select if there is only one instance of the
 *   granule allocator (i.e., gran_initialize will be called only once.
 *   In this case, (1) there are a few optimizations that can can be done
 *   and (2) the GRAN_HANDLE is not needed.
 * CONFIG_GRAN_INTR - Normally mutual exclusive access to granule allocator
 *   data is assured using a semaphore.  If this option is set then, instead,
 *   mutual exclusion logic will disable interrupts.  While this options is
 *   more invasive to system performance, it will also support use of the
 *   granule allocator from interrupt level logic.
 * CONFIG_DEBUG_GRAN - Just like CONFIG_DEBUG_MM, but only generates output
 *   from the gran allocation logic.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef CONFIG_GRAN_SINGLE
typedef FAR void *GRAN_HANDLE;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: gran_initialize
 *
 * Description:
 *   Set up one granule allocator instance.  Allocations will be aligned to
 *   the alignment size (log2align; allocations will be in units of the
 *   granule size (log2gran). Larger granules will give better performance
 *   and less overhead but more losses of memory due to quantization waste.
 *   Additional memory waste can occur from alignment; log2align should be
 *   set to 0 unless you are using the granule allocator to manage DMA
 *   or page-aligned memory and your hardware has specific memory alignment
 *   requirements.
 *
 *   General Usage Summary.  This is an example using the GCC section
 *   attribute to position a DMA heap in memory (logic in the linker script
 *   would assign the section .dmaheap to the DMA memory.
 *
 *     FAR uint32_t g_dmaheap[DMAHEAP_SIZE] __attribute__((section(.dmaheap)));
 *
 *   The heap is created by calling gran_initialize.  Here the granule size
 *   is set to 64 bytes and the alignment to 16 bytes:
 *
 *     GRAN_HANDLE handle = gran_initialize(g_dmaheap, DMAHEAP_SIZE, 6, 4);
 *
 *   Then the GRAN_HANDLE can be used to allocate memory (There is no
 *   GRAN_HANDLE if CONFIG_GRAN_SINGLE=y):
 *
 *     FAR uint8_t *dma_memory = (FAR uint8_t *)gran_alloc(handle, 47);
 *
 *   The actual memory allocates will be 64 byte (wasting 17 bytes) and
 *   will be aligned at least to (1 << log2align).
 *
 *   NOTE: The current implementation also restricts the maximum allocation
 *   size to 32 granules.  That restriction could be eliminated with some
 *   additional coding effort.
 *
 * Input Parameters:
 *   heapstart - Start of the granule allocation heap
 *   heapsize  - Size of heap in bytes
 *   log2gran  - Log base 2 of the size of one granule.  0->1 byte,
 *               1->2 bytes, 2->4 bytes, 3->8 bytes, etc.
 *   log2align - Log base 2 of required alignment.  0->1 byte,
 *               1->2 bytes, 2->4 bytes, 3->8 bytes, etc.  Note that
 *               log2gran must be greater than or equal to log2align
 *               so that all contiguous granules in memory will meet
 *               the minimum alignment requirement. A value of zero
 *               would mean that no alignment is required.
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned that may be used with other
 *   granule allocator interfaces.
 *
 ****************************************************************************/

#ifdef CONFIG_GRAN_SINGLE
int gran_initialize(FAR void *heapstart, size_t heapsize, uint8_t log2gran,
                    uint8_t log2align);
#else
GRAN_HANDLE gran_initialize(FAR void *heapstart, size_t heapsize,
                            uint8_t log2gran, uint8_t log2align);
#endif

/****************************************************************************
 * Name: gran_release
 *
 * Description:
 *   Uninitialize a gram memory allocator and release resources held by the
 *   allocator.
 *
 * Input Parameters:
 *   handle - The handle previously returned by gran_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_GRAN_SINGLE
void gran_release(void);
#else
void gran_release(GRAN_HANDLE handle);
#endif

/****************************************************************************
 * Name: gran_reserve
 *
 * Description:
 *   Reserve memory in the granule heap.  This will reserve the granules
 *   that contain the start and end addresses plus all of the granules
 *   in between.  This should be done early in the initialization sequence
 *   before any other allocations are made.
 *
 *   Reserved memory can never be allocated (it can be freed however which
 *   essentially unreserves the memory).
 *
 * Input Parameters:
 *   handle - The handle previously returned by gran_initialize
 *   start  - The address of the beginning of the region to be reserved.
 *   size   - The size of the region to be reserved
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_GRAN_SINGLE
void gran_reserve(uintptr_t start, size_t size);
#else
void gran_reserve(GRAN_HANDLE handle, uintptr_t start, size_t size);
#endif

/****************************************************************************
 * Name: gran_alloc
 *
 * Description:
 *   Allocate memory from the granule heap.
 *
 *   NOTE: The current implementation also restricts the maximum allocation
 *   size to 32 granules.  That restriction could be eliminated with some
 *   additional coding effort.
 *
 * Input Parameters:
 *   handle - The handle previously returned by gran_initialize
 *   size   - The size of the memory region to allocate.
 *
 * Returned Value:
 *   On success, either a non-NULL pointer to the allocated memory (if
 *   CONFIG_GRAN_SINGLE) or zero  (if !CONFIG_GRAN_SINGLE) is returned.
 *
 ****************************************************************************/

#ifdef CONFIG_GRAN_SINGLE
FAR void *gran_alloc(size_t size);
#else
FAR void *gran_alloc(GRAN_HANDLE handle, size_t size);
#endif

/****************************************************************************
 * Name: gran_free
 *
 * Description:
 *   Return memory to the granule heap.
 *
 * Input Parameters:
 *   handle - The handle previously returned by gran_initialize
 *   memory - A pointer to memory previously allocated by gran_alloc.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_GRAN_SINGLE
void gran_free(FAR void *memory, size_t size);
#else
void gran_free(GRAN_HANDLE handle, FAR void *memory, size_t size);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_GRAN */
#endif /* __INCLUDE_NUTTX_MM_GRAN_H */
