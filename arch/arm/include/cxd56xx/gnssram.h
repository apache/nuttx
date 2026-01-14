/****************************************************************************
 * arch/arm/include/cxd56xx/gnssram.h
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_GNSSRAM_H
#define __ARCH_ARM_INCLUDE_CXD56XX_GNSSRAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/compiler.h>
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
 * Pre-processor Definitions
 ****************************************************************************/

#define SECTION_GNSSRAM_TEXT   ".gnssram.text"
#define SECTION_GNSSRAM_DATA   ".gnssram.data"
#define SECTION_GNSSRAM_BSS    ".gnssram.bss"

/* Locate code and data into GNSS RAM */

#ifdef CONFIG_CXD56_GNSS_RAM
#  define GNSSRAM_CODE  locate_code(SECTION_GNSSRAM_TEXT)
#  define GNSSRAM_DATA  locate_data(SECTION_GNSSRAM_DATA)
#  define GNSSRAM_BSS   locate_data(SECTION_GNSSRAM_BSS)
#else
#  define GNSSRAM_CODE
#  define GNSSRAM_DATA
#  define GNSSRAM_BSS
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_gnssram_initialize
 *
 * Description:
 *   Initialize the GNSS heap.
 *
 ****************************************************************************/

void up_gnssram_initialize(void);

/****************************************************************************
 * Name: up_gnssram_uninitialize
 *
 * Description:
 *   Uninitialize the GNSS heap.
 *
 ****************************************************************************/

void up_gnssram_uninitialize(void);

/****************************************************************************
 * Name: up_gnssram_malloc
 *
 * Description:
 *   Allocate memory from the GNSS heap.
 *
 ****************************************************************************/

void *up_gnssram_malloc(size_t size);

/****************************************************************************
 * Name: up_gnssram_calloc
 *
 * Description:
 *   Calculates the size of the allocation and allocate memory from
 *   the GNSS heap.
 *
 ****************************************************************************/

void *up_gnssram_calloc(size_t n, size_t elem_size);

/****************************************************************************
 * Name: up_gnssram_realloc
 *
 * Description:
 *   Reallocate memory from the GNSS heap.
 *
 ****************************************************************************/

void *up_gnssram_realloc(void *ptr, size_t size);

/****************************************************************************
 * Name: up_gnssram_zalloc
 *
 * Description:
 *   Allocate and zero memory from the GNSS heap.
 *
 ****************************************************************************/

void *up_gnssram_zalloc(size_t size);

/****************************************************************************
 * Name: up_gnssram_free
 *
 * Description:
 *   Free memory from the GNSS heap.
 *
 ****************************************************************************/

void up_gnssram_free(void *mem);

/****************************************************************************
 * Name: up_gnssram_memalign
 *
 * Description:
 *   memalign requests more than enough space from malloc, finds a region
 *   within that chunk that meets the alignment request and then frees any
 *   leading or trailing space.
 *
 *   The alignment argument must be a power of two (not checked). 8-byte
 *   alignment is guaranteed by normal malloc calls.
 *
 ****************************************************************************/

void *up_gnssram_memalign(size_t alignment, size_t size);

/****************************************************************************
 * Name: up_gnssram_heapmember
 *
 * Description:
 *   Check if an address lies in the GNSS heap.
 *
 * Parameters:
 *   mem - The address to check
 *
 * Return Value:
 *   true if the address is a member of the GNSS heap. false if not
 *
 ****************************************************************************/

bool up_gnssram_heapmember(void *mem);

/****************************************************************************
 * Name: up_gnssram_mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current heap information for the
 *   GNSS heap.
 *
 ****************************************************************************/

struct mallinfo up_gnssram_mallinfo(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_GNSSRAM_H */
