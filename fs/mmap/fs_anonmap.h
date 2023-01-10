/****************************************************************************
 * fs/mmap/fs_anonmap.h
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

#ifndef __FS_MMAP_FS_ANONMAP_H
#define __FS_MMAP_FS_ANONMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <nuttx/mm/map.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: map_anonymous
 *
 * Description:
 *   Support simulation of private anonymous mapping by allocating memory
 *   from heap
 *
 * Input Parameters:
 *   map     Input struct containing user request
 *   kernel  kmm_zalloc or kumm_zalloc
 *
 * Returned Value:
 *   On success returns 0. Otherwise negated errno is returned appropriately.
 *
 *     ENOMEM
 *       Insufficient memory is available to simulate mapping
 *
 ****************************************************************************/

#ifdef CONFIG_FS_ANONMAP
int map_anonymous(FAR struct mm_map_entry_s *entry, bool kernel);
#else
#  define map_anonymous(entry, kernel) (-ENOSYS)
#endif /* CONFIG_FS_ANONMAP */

#endif /* __FS_MMAP_FS_ANONMAP_H */
