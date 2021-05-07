/****************************************************************************
 * libs/libc/endian/lib_swap32.c
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

#include <stdint.h>
#include <endian.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef __SWAP_UINT32_ISMACRO
uint32_t __swap_uint32(uint32_t n)
{
  return (uint32_t)(((((uint32_t)(n)) & 0x000000fful) << 24) |
                    ((((uint32_t)(n)) & 0x0000ff00ul) <<  8) |
                    ((((uint32_t)(n)) & 0x00ff0000ul) >>  8) |
                    ((((uint32_t)(n)) & 0xff000000ul) >> 24));
}
#endif
