/****************************************************************************
 * libs/libc/stdlib/lib_malloc_usable_size.c
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

#include <malloc.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: malloc_usable_size
 *
 * Description:
 *   Return the number of usable bytes in the block of memory at ptr, which
 *   malloc() or a related function returned.  This is the same as
 *   malloc_size(), the name of the function on other systems.
 *
 * Input Parameters:
 *   ptr - The block of memory, or NULL.
 *
 * Returned Value:
 *   The number of usable bytes; 0 if ptr is NULL.
 *
 ****************************************************************************/

size_t malloc_usable_size(FAR void *ptr)
{
  return lib_malloc_size(ptr);
}
