/****************************************************************************
 * include/nuttx/himem/himem.h
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_HIMEM_HIMEM_H
#define __INCLUDE_NUTTX_HIMEM_HIMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#include <signal.h>

#ifdef CONFIG_ESP32_SPIRAM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESP32 MMU block size */

#define ESP_HIMEM_BLKSZ (0x8000)

/* Command:     HIMEMIOC_ALLOC_BLOCKS
 * Description: Allocate a certain number of physical RAM blocks.
 * Arguments:   A structure containing the size of physical block to allocate
 *              and its esp_himem_handle_t.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define HIMEMIOC_ALLOC_BLOCKS      _HIMEMIOC(0x0001)

/* Command:     HIMEMIOC_FREE_BLOCKS
 * Description: Free a certain number of physical RAM blocks.
 * Arguments:   A structure containing the size of physical block to allocate
 *              and its esp_himem_handle_t.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define HIMEMIOC_FREE_BLOCKS       _HIMEMIOC(0x0002)

/* Command:     HIMEMIOC_ALLOC_MAP_RANGE
 * Description: Free the physical RAM blocks
 * Arguments:   A structure containing the block size and its
 *              esp_himem_rangehandle_t.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define HIMEMIOC_ALLOC_MAP_RANGE   _HIMEMIOC(0x0003)

/* Command:     HIMEMIOC_FREE_MAP_RANGE
 * Description: Maps the memory addresses to the physical psram range.
 * Arguments:   A structure containing the esp_himem_handle_t handle, the
 *              esp_himem_rangehandle_t, the ram offset, the range offset,
 *              the length, the memory flags and the output pointer.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define HIMEMIOC_FREE_MAP_RANGE    _HIMEMIOC(0x0004)

/* Command:     HIMEMIOC_MAP
 * Description: Maps the memory addresses to the physical psram range.
 * Arguments:   A structure containing the esp_himem_handle_t handle, the
 *              esp_himem_rangehandle_t, the ram offset, the range offset,
 *              the length, the memory flags and the output pointer.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define HIMEMIOC_MAP               _HIMEMIOC(0x0005)

/* Command:     HIMEMIOC_UNMAP
 * Description: Unmaps the memory addresses to the physical psram range.
 * Arguments:   A structure containing the esp_himem_rangehandle_t, the
 *              memory pointer and the memory length.
 *              the length, the memory flags and the output pointer.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define HIMEMIOC_UNMAP             _HIMEMIOC(0x0006)

/* Command:     HIMEMIOC_GET_PHYS_SIZE
 * Description: Get the size of physical external memory
 * Arguments:   None
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define HIMEMIOC_GET_PHYS_SIZE     _HIMEMIOC(0x0007)

/* Command:     HIMEMIOC_GET_FREE_SIZE
 * Description: Get the amount of free memory
 * Arguments:   None
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define HIMEMIOC_GET_FREE_SIZE     _HIMEMIOC(0x0008)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Handle for a window of address space */

typedef struct esp_himem_rangedata_t
{
  int block_ct;
  int block_start;
} esp_himem_rangedata_t;

/* Handle for a range of physical memory */

typedef struct esp_himem_ramdata_t
{
  int block_ct;
  uint16_t *block;
} esp_himem_ramdata_t;

/* Opaque pointers as handles for ram/range data */

typedef struct esp_himem_ramdata_t *esp_himem_handle_t;
typedef struct esp_himem_rangedata_t *esp_himem_rangehandle_t;

/* Structs with the parameters passed to the IOCTLs */

struct esp_himem_par
{
  esp_himem_handle_t handle;
  esp_himem_rangehandle_t range;
  size_t ram_offset;
  size_t range_offset;
  size_t memfree;
  size_t memcnt;
  size_t len;
  int flags;
  uint32_t *ptr;
};

/****************************************************************************
 * Public Data
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

int esp_himem_init(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ESP32_SPIRAM */
#endif /* __INCLUDE_NUTTX_HIMEM_HIMEM_H */
