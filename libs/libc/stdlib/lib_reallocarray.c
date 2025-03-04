/****************************************************************************
 * libs/libc/stdlib/lib_reallocarray.c
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

#include <stdlib.h>
#include <unistd.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Set overflow control only if larger than 65536 for bit platforms. The
 * limit is the same as in OpenBSD.
 */

#define CHECK_OVERFLOW_LIMIT (1UL << (sizeof(size_t) * 4))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: reallocarray
 *
 * Description:
 *   The reallocarray function has the same functionality as realloc but
 *   it fails safely if multiplication overflow occurs.
 *
 * Input Parameters:
 *   ptr   - old memory to be reallocated and freed
 *   nmemb - number of elements
 *   size  - size of one element in bytes
 *
 * Returned Value:
 *   Upon successful completion, the address of the re-allocated memory
 *   is returned and previous pointer is freed. NULL is returned on error
 *   with original block of memory left unchanged.
 *
 ****************************************************************************/

FAR void *reallocarray(FAR void *ptr, size_t nmemb, size_t size)
{
  if (nmemb != 0 && (nmemb >= CHECK_OVERFLOW_LIMIT ||
      size >= CHECK_OVERFLOW_LIMIT))
    {
      /* Do division only if at least one element is larget than limit */

      if ((SIZE_MAX / nmemb) < size)
        {
          set_errno(ENOMEM);
          return NULL;
        }
    }

  return lib_realloc(ptr, nmemb * size);
}
