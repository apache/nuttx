/****************************************************************************
 * include/nuttx/memoryregion.h
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

#ifndef __INCLUDE_MEMORYREGION_H
#define __INCLUDE_MEMORYREGION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes binfmt coredump filed */

struct memory_region_s
{
  uintptr_t start;   /* Start address of this region */
  uintptr_t end;     /* End address of this region */
  uint32_t  flags;   /* Figure 5-3: Segment Flag Bits: PF_[X|W|R] */
};

/****************************************************************************
 * Public Function
 ****************************************************************************/

/****************************************************************************
 * Name: parse_memory_region
 *
 * Input Parameters:
 *   format - The format string to parse. <start>,<end>,<flags>,...
 *            start - The start address of the memory region
 *            end   - The end address of the memory region
 *            flags - Readable 0x1, writable 0x2, executable 0x4
 *  region - The memory region to populate
 *  num    - The number of memory regions to parse
 *
 *  example: 0x1000,0x2000,0x1,0x2000,0x3000,0x3,0x3000,0x4000,0x7
 *
 ****************************************************************************/

ssize_t parse_memory_region(FAR const char *format,
                            FAR struct memory_region_s *region,
                            size_t num);

/****************************************************************************
 * Name: alloc_memory_region
 *
 * Input Parameters:
 *   format - The format string to parse. <start>,<end>,<flags>,...
 *            start - The start address of the memory region
 *            end   - The end address of the memory region
 *            flags - Readable 0x1, writable 0x2, executable 0x4
 *  example: 0x1000,0x2000,0x1,0x2000,0x3000,0x3,0x3000,0x4000,0x7
 *
 * Return:
 *   The parsed memory region list on success; NULL on failure.
 *   The boundary value of the memory region is zero.
 *   The return value need free by caller.
 *
 ****************************************************************************/

FAR struct memory_region_s *
alloc_memory_region(FAR const char *format);

/****************************************************************************
 * Name: free_memory_region
 *
 * Input Parameters:
 *   region - The memory region list to free.
 *
 ****************************************************************************/

void free_memory_region(FAR struct memory_region_s *region);

#endif /* __INCLUDE_MEMORYREGION_H */
