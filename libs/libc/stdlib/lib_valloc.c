/****************************************************************************
 * libs/libc/stdlib/lib_valloc.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: valloc
 *
 * Description:
 *   valloc - page-aligned memory allocator (OBSOLETE/LEGACY)
 *
 *   valloc has the same effect as malloc(), except that the allocated
 *   memory is aligned to a page boundary.
 *
 * Input Parameters:
 *   size - Specifies the size (in bytes) of the allocated block of memory.
 *   If size is zero, a unique pointer to the heap is returned.
 *
 * Returned Value:
 *   The address of the allocated memory (NULL on failure to allocate)
 *
 ****************************************************************************/

FAR void *valloc(size_t size)
{
  return lib_memalign(sysconf(_SC_PAGESIZE), size);
}
