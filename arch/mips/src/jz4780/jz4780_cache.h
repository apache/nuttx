/****************************************************************************
 * arch/mips/src/jz4780/jz4780_cache.h
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

#include <nuttx/config.h>
#include <assert.h>
#include <stdint.h>

#include <nuttx/cache.h>
#include <nuttx/debug.h>

#include <arch/board/board.h>

#ifndef __ARCH_MIPS_SRC_JZ4780_JZ4780_CACHE_H
#define __ARCH_MIPS_SRC_JZ4780_JZ4780_CACHE_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void m32_size_cache(void);
void m32_flush_dcache(void);
void m32_flush_icache(void);
void m32_clean_cache(uint32_t kva, size_t n);
void m32_sync_icache(uint32_t kva, size_t n);
void m32_dcache_clean_invalidate(uint32_t kva, size_t n);
void m32_dcache_invalidate(uint32_t kva, size_t n);
void m32_dcache_clean(unsigned kva, size_t n);

extern int mips_icache_size;
extern int mips_icache_linesize;
extern int mips_icache_ways;
extern int mips_dcache_size;
extern int mips_dcache_linesize;
extern int mips_dcache_ways;
extern int mips_scache_size;
extern int mips_scache_linesize;
extern int mips_scache_ways;

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_JZ4780_JZ4780_CACHE_H */

