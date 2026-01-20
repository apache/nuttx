/****************************************************************************
 * include/nuttx/drivers/block_cache.h
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

#ifndef __INCLUDE_NUTTX_MISC_BLOCK_CACHE_H
#define __INCLUDE_NUTTX_MISC_BLOCK_CACHE_H

/****************************************************************************
 * Provides a block device driver which can wrap an exiting block device.
 * The new block device provides cached access to the underlying device.
 *
 * Caching Strategy Summary:
 *
 * The block_cache_read_internal and block_cache_write_internal functions
 * implement a caching strategy that optimizes read and write operations to
 * a block device. The strategy utilizes a cache consisting of multiple
 * cache windows, each of which can store a portion of the device's blocks.
 *
 * Key Points:
 *
 * 1. Cache Windows:
 *    - The cache is divided into cache windows, each of size 'cache_width'
 *      blocks.
 *    - Each cache window holds a subset of the device's blocks in memory.
 *
 * 2. Block Alignment:
 *    - The caching is aligned to block boundaries to minimize cache misses.
 *
 * 3. Read Operation (block_cache_read_internal):
 *    - Reads data from the cache when available, minimizing access to the
 *      underlying device. Performs read ahead operation up to the size of
 *      the cache window.
 *    - Handles partial reads, synchronizing data between the cache and the
 *      device.
 *
 * 4. Write Operation (block_cache_write_internal):
 *    - Writes data to the cache, marking blocks as dirty.
 *    - Synchronizes dirty blocks with the underlying device when necessary.
 *
 * 5. Cache Management:
 *    - Cache windows are managed to ensure they contain the most relevant
 *      blocks.
 *    - Cache hits result in faster access times, while cache misses trigger
 *      cache loading and eviction.
 *
 * 6. Multiplier:
 *    - A 'geo_multiplier' factor is applied to the device geometry, allowing
 *      the cache to work with a multiple of the device's block size. This
 *      can allow the encapsulated device to be exposed with a larger block
 *      size.
 *
 * This caching strategy enhances performance by reducing the number of
 * read/write operations to the underlying block device and minimizing
 * redundant data transfers.
 *
 * Warning:
 *  The cache is only flushed to the underlying under the following
 *  conditions:
 *   - Close is called on the device handle, and it is the final remaining
 *     open reference.
 *   - BIOC_FLUSH IOCTL command is used. This is probably done by the
 *     filesystem when needed or requested by the user, through fsync or
 *     a similar mechanism.
 *
 *  If this driver is used with removable media, and the device is removed
 *  without the cache being flushed. The data is gone!
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#ifndef __ASSEMBLY__

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

/****************************************************************************
 * Name: block_cache_initialize
 *
 * Description:
 *   Initialize to provide a cached block device wrapper around an existing
 *    block device
 *
 * Input Parameters:
 *   source:         The source block driver to encapsulate
 *   destination:    The path to the exposed block driver
 *   cache_width:    The number of blocks in a single cache window.
 *   cache_count:    The number of cache windows
 *   geo_multiplier: A multiplier applied between the encapsulated and
 *                   exposed block devices.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int block_cache_initialize(FAR const char *source,
                           FAR const char *destination,
                           const size_t cache_width,
                           const size_t cache_count,
                           const size_t geo_multiplier);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_MISC_BLOCK_CACHE_H */
