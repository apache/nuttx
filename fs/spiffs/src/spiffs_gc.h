/****************************************************************************
 * fs/spiffs/src/spiffs_gc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __FS_SPIFFS_SRC_SPIFFS_GC_H
#define __FS_SPIFFS_SRC_SPIFFS_GC_H

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug */

#ifdef CONFIG_SPIFFS_GCDBG
#  define spiffs_gcinfo                   _info
#else
#  define spiffs_gcinfo                   _none
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct spiffs_s; /* Forward reference */

/****************************************************************************
 * Name: spiffs_gc_quick
 *
 * Description:
 *   Searches for blocks where all entries are deleted - if one is found,
 *   the block is erased. Compared to the non-quick gc, the quick one ensures
 *   that no updates are needed on existing objects on pages that are erased.
 *
 * Input Parameters:
 *   fs             - A reference to the SPIFFS volume object instance
 *   max_free_pages - The maximum pages to free
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.  -ENOSPC is returned if no blocks were deleted.
 *
 ****************************************************************************/

int spiffs_gc_quick(FAR struct spiffs_s *fs, uint16_t max_free_pages);

/****************************************************************************
 * Name: spiffs_gc_check
 *
 * Description:
 *   Checks if garbage collecting is necessary. If so a candidate block is
 *   found, cleansed and erased
 *
 *   This function will  try to make room for given amount of bytes in the
 *   filesystem by moving pages and erasing blocks.
 *
 *   If it is physically impossible, err_no will be set to -ENOSPC. If
 *   there already is this amount (or more) of free space, SPIFFS_gc will
 *   silently return. It is recommended to call statfs() before invoking
 *   this method in order to determine what amount of bytes to give.
 *
 *   NB: the garbage collector is automatically called when spiffs needs free
 *   pages. The reason for this function is to give possibility to do
 *   background tidying when user knows the system is idle.
 *
 * Input Parameters:
 *   fs            the file system struct
 *   len           amount of data that should be freed
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.  -ENODATA is returned if no blocks were deleted.
 *
 ****************************************************************************/

int spiffs_gc_check(FAR struct spiffs_s *fs, off_t len);

#if defined(__cplusplus)
}
#endif

#endif /* __FS_SPIFFS_SRC_SPIFFS_GC_H */
