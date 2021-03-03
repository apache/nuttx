/****************************************************************************
 * libs/libc/endian/lib_swap64.c
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

#include <nuttx/compiler.h>

#include <stdint.h>
#include <endian.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_HAVE_LONG_LONG
uint64_t __swap_uint64(uint64_t n)
{
  return (uint64_t)(((((uint64_t)(n)) & 0x00000000000000ffull) << 56) |
                    ((((uint64_t)(n)) & 0x000000000000ff00ull) << 40) |
                    ((((uint64_t)(n)) & 0x0000000000ff0000ull) << 24) |
                    ((((uint64_t)(n)) & 0x00000000ff000000ull) <<  8) |
                    ((((uint64_t)(n)) & 0x000000ff00000000ull) >>  8) |
                    ((((uint64_t)(n)) & 0x0000ff0000000000ull) >> 24) |
                    ((((uint64_t)(n)) & 0x00ff000000000000ull) >> 40) |
                    ((((uint64_t)(n)) & 0xff00000000000000ull) >> 56));
}
#endif
