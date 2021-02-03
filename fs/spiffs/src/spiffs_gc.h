/****************************************************************************
 * fs/spiffs/src/spiffs_gc.h
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
