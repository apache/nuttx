/****************************************************************************
 * mm/umm_heap/umm_calloc.c
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

#include "umm_heap/umm_heap.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: calloc
 *
 * Description:
 *   calloc is a thin wrapper for mm_calloc()
 *
 ****************************************************************************/

#undef calloc /* See mm/README.txt */
FAR void *calloc(size_t n, size_t elem_size)
{
#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
  /* Use zalloc() because it implements the sbrk() logic */

  FAR void *mem = NULL;
  /* Verify input parameters
   *
   * elem_size or n is zero treats as valid input.
   *
   * Assure that the following multiplication cannot overflow the size_t
   * type, i.e., that:  SIZE_MAX >= n * elem_size
   *
   * Refer to SEI CERT C Coding Standard.
   */

  if (elem_size == 0 || n <= (SIZE_MAX / elem_size))
    {
      /* Use zalloc() because it implements the sbrk() logic */

      mem = zalloc(n * elem_size);
    }

  return mem;
#else
  /* Use mm_calloc() because it implements the clear */

  return mm_calloc(USR_HEAP, n, elem_size);
#endif
}
