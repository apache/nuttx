/************************************************************************
 * arch/arm/src/lpc313x/lpc313x_allocateheap.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <sys/types.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "lpc313x_memorymap.h"

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/* Configuration ********************************************************/

/* Some sanity checking.  If external memory regions are defined, verify
 * that CONFIG_MM_REGIONS is set to match, exactly, the number of external
 * memory regions that we have been asked to add to the heap.
 */

#if defined(CONFIG_LPC313X_EXTSRAM0) && defined(CONFIG_LPC313X_EXTSRAM0HEAP)
#  if defined(CONFIG_LPC313X_EXTSRAM1) && defined(CONFIG_LPC313X_EXTSRAM1HEAP)
#    if defined(CONFIG_LPC313X_EXTSDRAM) && defined(CONFIG_LPC313X_EXTSDRAMHEAP)
#      /* SRAM+EXTSRAM0+EXTSRAM1+EXTSDRAM */
#      define LPC313X_NEXT_REGIONS 4
#    else
#      /* SRAM+EXTSRAM0+EXTSRAM1 */
#      define LPC313X_NEXT_REGIONS 3
#    endif
#  elif defined(CONFIG_LPC313X_EXTSDRAM) && defined(CONFIG_LPC313X_EXTSDRAMHEAP)
#      /* SRAM+EXTSRAM0+EXTSDRAM */
#      define LPC313X_NEXT_REGIONS 3
#  else
#      /* SRAM+EXTSRAM0 */
#      define LPC313X_NEXT_REGIONS 2
#  endif
#elif defined(CONFIG_LPC313X_EXTSRAM1) && defined(CONFIG_LPC313X_EXTSRAM1HEAP)
#  if defined(CONFIG_LPC313X_EXTSDRAM) && defined(CONFIG_LPC313X_EXTSDRAMHEAP)
#      /* SRAM+EXTSRAM1+EXTSDRAM */
#      define LPC313X_NEXT_REGIONS 3
#  else
#      /* SRAM+EXTSRAM1 */
#      define LPC313X_NEXT_REGIONS 2
#  endif
#elif defined(CONFIG_LPC313X_EXTSDRAM) && defined(CONFIG_LPC313X_EXTSDRAMHEAP)
#      /* SRAM+EXTSDRAM */
#      define LPC313X_NEXT_REGIONS 2
#else
#      /* SRAM */
#      define LPC313X_NEXT_REGIONS 1
#endif

#if CONFIG_MM_REGIONS != LPC313X_NEXT_REGIONS
#  if CONFIG_MM_REGIONS < LPC313X_NEXT_REGIONS
#    error "CONFIG_MM_REGIONS is large enough for the selected memory regions"
#  else
#    error "CONFIG_MM_REGIONS is too large for the selected memory regions"
#  endif
#  if defined(CONFIG_LPC313X_EXTSRAM0) && defined(CONFIG_LPC313X_EXTSRAM0HEAP)
#    error "External SRAM0 is selected for heap"
#  endif
#  if defined(CONFIG_LPC313X_EXTSRAM1) && defined(CONFIG_LPC313X_EXTSRAM1HEAP)
#    error "External SRAM1 is selected for heap"
#  endif
#  if defined(CONFIG_LPC313X_EXTSDRAM) && defined(CONFIG_LPC313X_EXTSDRAMHEAP)
#    error "External SRAM1 is selected for heap"
#  endif
#endif

/************************************************************************
 * Private Data
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   The heap may be statically allocated by defining CONFIG_HEAP_BASE
 *   and CONFIG_HEAP_SIZE.  If these are not defined, then this function
 *   will be called to dynamically set aside the heap region to the end
 *   of SRAM.
 *
 *   SRAM layout:
 *   Start of SRAM:   .data
 *                    .bss
 *                    IDLE thread stack
 *   End of SRAm:     heap
 *
 *   NOTE: Ignore the erroneous nomenclature DRAM and SDRAM.  That names
 *   date back to an earlier platform that had SDRAM.
 *
 ************************************************************************/

void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
{
  up_ledon(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)g_heapbase;
  *heap_size  = (LPC313X_INTSRAM_VSECTION + LPC313X_ISRAM_SIZE) - g_heapbase;
}

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
#if defined(CONFIG_LPC313X_EXTSRAM0) && defined(CONFIG_LPC313X_EXTSRAM0HEAP)
  mm_addregion((FAR void*)LPC313X_EXTSRAM0_VSECTION, CONFIG_LPC313X_EXTSRAM0SIZE);
#endif

#if defined(CONFIG_LPC313X_EXTSRAM1) && defined(CONFIG_LPC313X_EXTSRAM1HEAP)
  mm_addregion((FAR void*)LPC313X_EXTSRAM1_VSECTION, CONFIG_LPC313X_EXTSRAM1SIZE);
#endif

#if defined(CONFIG_LPC313X_EXTSDRAM) && defined(CONFIG_LPC313X_EXTSDRAMHEAP)
  mm_addregion((FAR void*)LPC313X_EXTSDRAM_VSECTION, CONFIG_LPC313X_EXTSDRAMSIZE);
#endif
}
#endif
