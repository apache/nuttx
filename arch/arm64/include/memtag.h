/****************************************************************************
 * arch/arm64/include/memtag.h
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

#ifndef ___ARCH_ARM64_INCLUDE_MEMTAG_H
#define ___ARCH_ARM64_INCLUDE_MEMTAG_H

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize MTE settings and enable memory tagging */

void up_memtag_init(void);

/* Check if MTE is enabled */

bool up_memtag_is_enable(void);

/* Set MTE state */

void up_memtag_bypass(bool bypass);

/* Set memory tags for a given memory range */

void up_memtag_set_tag(const void *addr, size_t size);

/* Get a random label based on the address through the mte register */

uint8_t up_memtag_get_random_tag(const void *addr);

/* Get the address without label */

void *up_memtag_get_untagged_addr(const void *addr);

/* Get the address with label */

void *up_memtag_get_tagged_addr(const void *addr, uint8_t tag);

#endif /* ___ARCH_ARM64_INCLUDE_MEMTAG_H */
