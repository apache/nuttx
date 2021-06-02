/****************************************************************************
 * include/nuttx/progmem.h
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

#ifndef __INCLUDE_NUTTX_PROGMEM_H
#define __INCLUDE_NUTTX_PROGMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdbool.h>

#ifdef CONFIG_ARCH_HAVE_PROGMEM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/* NOTE: Since vendors use the terms page, block, and sector interchangably,
 * it causes confusion and therefore a neutral term is used. We use the term
 * "index" to refer to the ordinal number of erasable units of memory.
 */

/****************************************************************************
 * Name: up_progmem_maxeraseindex
 *
 * Description:
 *   Return the total number of erasable units of memory.
 *
 ****************************************************************************/

size_t up_progmem_maxeraseindex(void);

/****************************************************************************
 * Name: up_progmem_isuniform
 *
 * Description:
 *  Are all erasable units of memory the same size?
 *
 * Returned Value:
 *   true if all erasable units are the same size, false if they vary in size
 *
 ****************************************************************************/

bool up_progmem_isuniform(void);

/****************************************************************************
 * Name: up_progmem_writegranularity
 *
 * Description:
 *   Return the number of bytes of the smallest allowable write operation.
 *   All writes MUST be a multiple of the granularity. Some FLASH memories
 *   require write operations in multiples of 8-bit, 16-bit, 32-bit, 128-bit,
 *   256-bit, etc. The caller must ensure the size of the data passed into
 *   up_progmem_write is an even multiple of the writegranularity. Padding
 *   may be required.
 *
 * Returned Value:
 *   Required granularity of a write operation for the FLASH memory.
 *
 ****************************************************************************/

size_t up_progmem_writegranularity(void);

/****************************************************************************
 * Name: up_progmem_erasesize
 *
 * Description:
 *   Return the erase size for a specified index. Will always be a multiple of
 *   the write granularity.
 *
 * Input Parameters:
 *   index - ordinal number of the set of erasable units of memory
 *
 * Returned Value:
 *   Erase size for erasable unit of memory, SIZE_MAX if index is invalid.
 *
 ****************************************************************************/

size_t up_progmem_erasesize(size_t index);

/****************************************************************************
 * Name: up_progmem_getindex
 *
 * Description:
 *   Get the index of memory, given a FLASH memory address
 *
 * Input Parameters:
 *  addr - Address with or without flash offset
 *         (absolute or aligned to sector0)
 *
 * Returned Value:
 *   Index or negative value on error.
 *   The following errors are reported (errno is not set!):
 *
 *     -EFAULT: On invalid address
 *
 ****************************************************************************/

ssize_t up_progmem_getindex(size_t addr);

/****************************************************************************
 * Name: up_progmem_getaddress
 *
 * Description:
 *   Get the base address of a unit of memory, given an index
 *
 * Input Parameters:
 *   index - ordinal number of the set of erasable units of memory
 *
 * Returned Value:
 *   Base address of given section, SIZE_MAX if index is not valid.
 *
 ****************************************************************************/

size_t up_progmem_getaddress(size_t index);

/****************************************************************************
 * Name: up_progmem_erase
 *
 * Description:
 *   Erase a specified unit of memory, given an index
 *
 * Input Parameters:
 *   index - ordinal number of the set of erasable units of memory
 *
 * Returned Value:
 *   section size or negative value on error.
 *   The following errors are reported (errno is not set!):
 *
 *     -EFAULT: On invalid section
 *     -EIO:    On unsuccessful erase
 *     -EROFS:  On access to write protected area
 *     -EACCES: Insufficient permissions (read/write protected)
 *     -EPERM:  If operation is not permitted due to some other constraints
 *              (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

ssize_t up_progmem_erase(size_t index);

/****************************************************************************
 * Name: up_progmem_issectionerased
 *
 * Description:
 *   Checks whether a specified unit of memory is erased
 *
 * Input Parameters:
 *   index - ordinal number of the set of erasable units of memory
 *
 * Returned Value:
 *   Returns number of bytes NOT erased or negative value on error. If it
 *   returns zero then complete section is erased.
 *
 *   The following errors are reported:
 *     -EFAULT: On invalid index
 *
 ****************************************************************************/

ssize_t up_progmem_iserased(size_t index);

/****************************************************************************
 * Name: up_progmem_write
 *
 * Description:
 *   Program data at given address
 *
 *   Note: This function may cross memory boundaries so long as the caller
 *         has ensured all units of memory that will be programmed have been
 *         erased. It also does not require that the write be aligned with the
 *         beginning of a unit of memory.
 *
 * Input Parameters:
 *   addr  - Address with or without flash offset
 *           (absolute or aligned to sector0)
 *   buf   - Pointer to buffer
 *   count - Number of bytes to write
 *
 * Returned Value:
 *   Bytes written or negative value on error.  The following errors are
 *   reported (errno is not set!)
 *
 *     EINVAL: If count is not aligned with the write granularity (i.e.
 *             some MCU's require per half-word or even word access)
 *     EFAULT: On invalid address
 *     EIO:    On unsuccessful write
 *     EROFS:  On access to write protected area
 *     EACCES: Insufficient permissions (read/write protected)
 *     EPERM:  If operation is not permitted due to some other constraints
 *             (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

ssize_t up_progmem_write(size_t addr, FAR const void *buf, size_t count);

/****************************************************************************
 * Name: up_progmem_read
 *
 * Description:
 *   Read data at given address
 *
 *   NOTE: This function does not require the address to be aligned with a
 *         unit of memory. It is also not limited to single unit of memory
 *         (i.e. multiple units of memory may be read, including partial unit
 *         reads)
 *
 * Input Parameters:
 *   addr  - Address with or without flash offset
 *           (absolute or aligned to sector0)
 *   buf   - Pointer to buffer
 *   count - Number of bytes to read
 *
 * Returned Value:
 *   Bytes read or negative value on error.  The following errors are
 *   reported (errno is not set!)
 *
 *     EINVAL: If count is not aligned with the flash boundaries (i.e.
 *             some MCU's require per half-word or even word access)
 *     EFAULT: On invalid address
 *     EIO:    On unsuccessful read
 *     EACCES: Insufficient permissions (read/write protected)
 *     EPERM:  If operation is not permitted due to some other constraints
 *             (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_PROGMEM_READ
ssize_t up_progmem_read(size_t addr, FAR void *buf, size_t count);
#endif

/****************************************************************************
 * Name: up_progmem_enableprogramming
 *
 * Description:
 *   Enable the ability to erase/write the FLASH memory.
 *
 *   NOTE: Write-protection may still be required for an individual
 *         unit of memory.
 *
 * Returned Value:
 *   0 on success, or negative value on error.
 *
 ****************************************************************************/

int up_progmem_enableprogramming(void);

/****************************************************************************
 * Name: up_progmem_disableprogramming
 *
 * Description:
 *   Disable the ability to erase/write the FLASH memory.
 *
 * Returned Value:
 *   0 on success, or negative value on error.
 *
 ****************************************************************************/

int up_progmem_disableprogramming(void);

/****************************************************************************
 * Name: up_progmem_enablewriteprotect
 *
 * Description:
 *   Enable write protection for the specified unit of memory
 *
 * Input Parameters:
 *   index - ordinal number of the set of erasable units of memory
 *
 * Returned Value:
 *   0 on success, or negative value on error.
 *
 ****************************************************************************/

int up_progmem_enablewriteprotect(size_t index);

/****************************************************************************
 * Name: up_progmem_disablewriteprotect
 *
 * Description:
 *   Disable write protection for the specified unit of memory
 *
 * Input Parameters:
 *   index - ordinal number of the set of erasable units of memory
 *
 * Returned Value:
 *   0 on success, or negative value on error.
 *
 ****************************************************************************/

int up_progmem_disablewriteprotect(size_t index);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_ARCH_HAVE_PROGMEM */
#endif /* __INCLUDE_NUTTX_PROGMEM_H */
