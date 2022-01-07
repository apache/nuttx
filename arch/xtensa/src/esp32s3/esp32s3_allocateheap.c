/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_allocateheap.c
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

#include <sys/types.h>
#include <debug.h>

#include <nuttx/mm/mm.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "xtensa.h"

#include "hardware/esp32s3_rom_layout.h"

/****************************************************************************
 * Pre-processor Definitions
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
 *   For the kernel build (CONFIG_BUILD_KERNEL=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function provides the
 *   size of the unprotected, user-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  board_autoled_on(LED_HEAPALLOCATE);

  *heap_start = (void *)&_sheap;
  *heap_size = (size_t)(ets_rom_layout_p->dram0_rtos_reserved_start -
                (uintptr_t)&_sheap);
}

/****************************************************************************
 * Name: xtensa_add_region
 *
 * Description:
 *   RAM may be added in non-contiguous chunks. This routine adds all chunks
 *   that may be used for heap.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void xtensa_add_region(void)
{
}
#endif
