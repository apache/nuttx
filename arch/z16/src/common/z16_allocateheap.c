/****************************************************************************
 * arch/z16/src/common/z16_allocateheap.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>

#include "chip.h"
#include "z16_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Use ZDS-II linker settings to get the unused external RAM and use this
 * for the NuttX heap.
 */

#ifndef CONFIG_HEAP1_BASE
  extern _Far unsigned long far_heapbot;
  #define CONFIG_HEAP1_BASE ((unsigned long)&far_heapbot)
#endif

#ifndef CONFIG_HEAP1_END
  extern _Far unsigned long far_heaptop;
  #define CONFIG_HEAP1_END ((unsigned long)&far_heaptop)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 ****************************************************************************/

void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
{
  *heap_start = (FAR void *)CONFIG_HEAP1_BASE;
  *heap_size = CONFIG_HEAP1_END - CONFIG_HEAP1_BASE;
  board_autoled_on(LED_HEAPALLOCATE);
}

/****************************************************************************
 * Name: z16_addregions
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void z16_addregion(void)
{
  kmm_addregion((FAR void *)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);
}
#endif
