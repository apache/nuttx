/****************************************************************************
 * arch/risc-v/src/ab32vg1/ab32vg1_allocateheap.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "chip.h"
#include "ab32vg1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory layout (see the board linker script):
 *
 *   SRAM    0x10000 .. _stext       free (ROM scratch during boot)
 *   SRAM    _stext  .. _esram       image, loaded by the ROM or boot code
 *   SRAM    _esram  .. 0x30000      free
 *   AECRAM  0x50000 .. _eaecram     code placed in AECRAM (C library)
 *   AECRAM  _eaecram.. 0x57a00      free
 *   ICRAM   0x60000 .. _ebss        .bss
 *   ICRAM   _ebss   .. idle top     idle thread stack
 *   ICRAM   idle top.. 0x68000      free
 *
 * The primary heap is SRAM after the image; the other free chunks are added
 * as extra regions up to CONFIG_MM_REGIONS.
 */

#define HEAP_ALIGN(a)   (((uintptr_t)(a) + 15) & ~(uintptr_t)15)

#define SRAM_END        (AB32VG1_SRAM_BASE + AB32VG1_SRAM_SIZE)
#define AECRAM_END      (AB32VG1_AECRAM_BASE + AB32VG1_AECRAM_SIZE)
#define ICRAM_END       (AB32VG1_ICRAM_BASE + AB32VG1_ICRAM_SIZE)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
static void ab32vg1_addchunk(uintptr_t start, uintptr_t end)
{
  start = HEAP_ALIGN(start);
  if (start + 64 < end)
    {
      kumm_addregion((void *)start, end - start);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   Return the primary heap: SRAM between the end of the image and the end
 *   of SRAM.
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  uintptr_t start = HEAP_ALIGN(_esram);

  DEBUGASSERT(start < SRAM_END);

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)start;
  *heap_size  = SRAM_END - start;
}

/****************************************************************************
 * Name: riscv_addregion
 *
 * Description:
 *   RAM may be added in non-contiguous chunks.  This routine adds all chunks
 *   that may be used for heap.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void riscv_addregion(void)
{
  /* ICRAM after .bss and the idle stack */

  ab32vg1_addchunk(g_idle_topstack, ICRAM_END);

#if CONFIG_MM_REGIONS > 2
  /* AECRAM after the code placed there */

  ab32vg1_addchunk((uintptr_t)_eaecram, AECRAM_END);
#endif

#if CONFIG_MM_REGIONS > 3
  /* SRAM below the image */

  ab32vg1_addchunk(AB32VG1_SRAM_BASE, (uintptr_t)_stext);
#endif
}
#endif
