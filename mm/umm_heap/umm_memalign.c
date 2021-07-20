/****************************************************************************
 * mm/umm_heap/umm_memalign.c
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
 * Name: memalign
 *
 * Description:
 *   memalign requests more than enough space from malloc, finds a region
 *   within that chunk that meets the alignment request and then frees any
 *   leading or trailing space.
 *
 *   The alignment argument must be a power of two (not checked).  8-byte
 *   alignment is guaranteed by normal malloc calls.
 *
 ****************************************************************************/

#undef memalign /* See mm/README.txt */
FAR void *memalign(size_t alignment, size_t size)
{
#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
  FAR void *brkaddr;
  FAR void *mem;

  /* Initialize the user heap if it wasn't yet */

  umm_try_initialize();

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
      mem = mm_memalign(USR_HEAP, alignment, size);
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
  return mm_memalign(USR_HEAP, alignment, size);
#endif
}
