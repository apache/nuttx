/****************************************************************************
 * libs/libc/string/lib_mempcpy.c
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
#include <sys/types.h>
#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mempcpy
 ****************************************************************************/

/****************************************************************************
 * Name: mempcpy
 *
 * Description:
 *   Like memcpy(), but returns address of byte after the last copied byte
 *   in dest. This allows constructing data piecewise by copying its parts
 *   into consecutive memory locations.
 *
 *   This function is not in POSIX.
 *
 * Input Parameters:
 *   src  - Source location from which to copy.
 *   dest - Destination location into which to copy.
 *   n    - Size of data to be copied, in bytes.
 *
 * Returned Value:
 *   A pointer to the byte in dest which immediately follows the last copied
 *   byte.
 *
 ****************************************************************************/

FAR void *mempcpy(FAR void *dest, FAR const void *src, size_t n)
{
  return (FAR char *)memcpy(dest, src, n) + n;
}
