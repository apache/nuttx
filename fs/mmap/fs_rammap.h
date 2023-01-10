/****************************************************************************
 * fs/mmap/fs_rammap.h
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

/* This driver manages files that have been copied to memory and
 * managed as a share-able "memory mapped" file.  This functionality is
 * intended to provide a substitute for memory mapped files for architectures
 * that do not have MMUs and, hence, cannot support on demand paging of
 * blocks of a file.
 *
 * This copied file has many of the properties of a standard memory mapped
 * file except:
 *
 * - All of the file must be present in memory.  This limits the size of
 *   files that may be memory mapped (especially on MCUs with no significant
 *   RAM resources).
 * - All mapped files are read-only.  You can write to the in-memory image,
 *   but the file contents will not change.
 * - There are not access privileges.
 */

#ifndef __FS_MMAP_FS_RAMMAP_H
#define __FS_MMAP_FS_RAMMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <nuttx/mm/map.h>

#ifdef CONFIG_FS_RAMMAP

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rammmap
 *
 * Description:
 *   Support simulation of memory mapped files by copying files into RAM.
 *
 * Input Parameters:
 *   filep   file descriptor of the backing file -- required.
 *   length  The length of the mapping.  For exception #1 above, this length
 *           ignored:  The entire underlying media is always accessible.
 *   offset  The offset into the file to map
 *   kernel  kmm_zalloc or kumm_zalloc
 *   mapped  The pointer to the mapped area
 *
 * Returned Value:
 *   On success rammmap returns 0. Otherwise errno is returned appropriately.
 *
 *     EBADF
 *      'fd' is not a valid file descriptor.
 *     EINVAL
 *       'length' or 'offset' are invalid
 *     ENOMEM
 *       Insufficient memory is available to map the file.
 *
 ****************************************************************************/

int rammap(FAR struct file *filep, FAR struct mm_map_entry_s *entry,
           bool kernel);
#else
#  define rammap(file, entry, kernel) (-ENOSYS)
#endif /* CONFIG_FS_RAMMAP */

#endif /* __FS_MMAP_FS_RAMMAP_H */
