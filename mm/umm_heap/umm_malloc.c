/****************************************************************************
 * mm/umm_heap/umm_malloc.c
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
#include <unistd.h>

#include <nuttx/mm/mm.h>

#include "umm_heap/umm_heap.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: malloc
 *
 * Description:
 *   Allocate memory from the user heap.
 *
 * Input Parameters:
 *   size - Size (in bytes) of the memory region to be allocated.
 *
 * Returned Value:
 *   The address of the allocated memory (NULL on failure to allocate)
 *
 ****************************************************************************/

FAR void *malloc(size_t size)
{
#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
  FAR void *brkaddr;
  FAR void *mem;

  if (size < 1)
    {
      return NULL;
    }

  /* Loop until we successfully allocate the memory or until an error
   * occurs. If we fail to allocate memory on the first pass, then call
   * sbrk to extend the heap by one page.  This may require several
   * passes if more the size of the allocation is more than one page.
   *
   * An alternative would be to increase the size of the heap by the
   * full requested allocation in sbrk().  Then the loop should never
   * execute more than twice (but more memory than we need may be
   * allocated).
   */

  do
    {
      mem = mm_malloc(USR_HEAP, size);
      if (!mem)
        {
          brkaddr = sbrk(size);
          if (brkaddr == (FAR void *)-1)
            {
              return NULL;
            }
        }
    }
  while (mem == NULL);

  return mem;
#else
  return mm_malloc(USR_HEAP, size);
#endif
}
