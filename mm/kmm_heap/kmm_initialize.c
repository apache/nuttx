/****************************************************************************
 * mm/kmm_heap/kmm_initialize.c
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

#include <nuttx/mm/mm.h>

#ifdef CONFIG_MM_KERNEL_HEAP

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the kernel heap */

FAR struct mm_heap_s *g_kmmheap;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kmm_initialize
 *
 * Description:
 *   Initialize the kernel heap data structures, providing the initial
 *   heap region.
 *
 * Input Parameters:
 *   heap_start - Address of the beginning of the (initial) memory region
 *   heap_size  - The size (in bytes) if the (initial) memory region.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kmm_initialize(FAR void *heap_start, size_t heap_size)
{
  g_kmmheap = mm_initialize_pool("Kmem", heap_start, heap_size, NULL);
}

#endif /* CONFIG_MM_KERNEL_HEAP */
