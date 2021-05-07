/****************************************************************************
 * mm/umm_heap/umm_addregion.c
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

#include <nuttx/mm/mm.h>

#include "umm_heap/umm_heap.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: umm_addregion
 *
 * Description:
 *   This is a simple wrapper for the mm_addregion() function.  This
 *   function is exported from the user-space blob so that the kernel
 *   can initialize the user-mode allocator.
 *
 * Input Parameters:
 *   heap_start - Address of the beginning of the memory region
 *   heap_size  - The size (in bytes) if the memory region.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void umm_addregion(FAR void *heap_start, size_t heap_size)
{
  mm_addregion(USR_HEAP, heap_start, heap_size);
}
