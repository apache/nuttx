/****************************************************************************
 * libs/libc/string/lib_memcpy.c
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
 * Name: memcpy
 ****************************************************************************/

#ifndef CONFIG_LIBC_ARCH_MEMCPY
#undef memcpy /* See mm/README.txt */
FAR void *memcpy(FAR void *dest, FAR const void *src, size_t n)
{
  FAR unsigned char *pout = (FAR unsigned char *)dest;
  FAR unsigned char *pin  = (FAR unsigned char *)src;
  while (n-- > 0) *pout++ = *pin++;
  return dest;
}
#endif
