/****************************************************************************
 * mm/kmm_heap/kmm_memalign.c
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

#include <stdlib.h>

#include <nuttx/mm/mm.h>

#ifdef CONFIG_MM_KERNEL_HEAP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kmm_memalign
 *
 * Description:
 *   Allocate aligned memory in the kernel heap.
 *
 * Input Parameters:
 *   alignment - Log2 byte alignment
 *   size - Size (in bytes) of the new memory region to be allocated.
 *
 * Returned Value:
 *   The address of the re-allocated memory (NULL on failure to allocate)
 *
 ****************************************************************************/

FAR void *kmm_memalign(size_t alignment, size_t size)
{
  return mm_memalign(&g_kmmheap, alignment, size);
}

#endif /* CONFIG_MM_KERNEL_HEAP */
