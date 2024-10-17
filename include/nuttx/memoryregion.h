/****************************************************************************
 * include/nuttx/memoryregion.h
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

#endif /* __INCLUDE_MEMORYREGION_H */
