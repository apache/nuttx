/****************************************************************************
 * mm/umm_heap/umm_initialize.c
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

#include <assert.h>

#include <nuttx/mm/mm.h>

#include "umm_heap/umm_heap.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: umm_initialize
 *
 * Description:
 *   This is a simple wrapper for the mm_initialize() function.  This
 *   function will initialize the user heap.
 *
 *   CONFIG_BUILD_FLAT:
 *     There is only kernel mode "blob" containing both kernel and
 *     application code.  There is only one heap that is used by both the
 *     kernel and application logic.
 *
 *     In this configuration, this function is called early in nx_start()
 *     to initialize the common heap.
 *
 *   CONFIG_BUILD_PROTECTED
 *     In this configuration, there are two "blobs", one containing
 *     protected kernel logic and one containing unprotected application
 *     logic.  Depending upon the setting of CONFIG_MM_KERNEL_HEAP there
 *     may be only a single shared heap, much as with CONFIG_BUILD_FLAT.
 *     Or there may be separate protected/kernel and unprotected/user
 *     heaps.
 *
 *     In either case, this function is still called early in nx_start()
 *     to initialize the user heap.
 *
 *   CONFIG_BUILD_KERNEL
 *     In this configuration there are multiple user heaps, one for each
 *     user process.  Furthermore, each heap is initially empty; memory
 *     is added to each heap dynamically via sbrk().  The heap data
 *     structure was set to zero when the address environment was created.
 *     Otherwise, the heap is uninitialized.
 *
 *     This function is not called at all.  Rather, this function is called
 *     when each user process is created before the first allocation is made.
 *
 * Input Parameters:
 *   heap_start - Address of the beginning of the (initial) memory region
 *   heap_size  - The size (in bytes) if the (initial) memory region.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void umm_initialize(FAR void *heap_start, size_t heap_size)
{
  USR_HEAP = mm_initialize("Umem", heap_start, heap_size);
}

/****************************************************************************
 * Name: umm_try_initialize
 *
 * Description:
 *   Allocate and initialize the user heap if not yet.
 *
 * Input Parameters:
 * None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
void umm_try_initialize(void)
{
  uintptr_t allocbase;

  /* Return if the user heap is already initialized. */

  if (USR_HEAP != NULL)
    {
      return;
    }

  /* Allocate one page. If we provide a zero brkaddr to pgalloc(),
   * it will create the first block in the correct virtual address
   * space and return the start address of that block.
   */

  allocbase = pgalloc(0, 1);
  DEBUGASSERT(allocbase != 0);

  /* Let umm_initialize do the real work. */

  umm_initialize((FAR void *)allocbase, CONFIG_MM_PGSIZE);
}
#endif
