/****************************************************************************
 * fs/shm/shmfs.h
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

#ifndef __FS_SHM_SHMFS_H
#define __FS_SHM_SHMFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/fs.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const struct file_operations shmfs_operations;

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct shmfs_object_s
{
  /* Total number of bytes needed from physical memory. */

  size_t length;

  /* Vector of allocations from physical memory.
   *
   * - In flat and protected builds this is a pointer to the
   *   allocated memory.
   *
   * - In kernel build this is start of a malloc'd vector of void pointers
   *   and the length of the vector is MM_NPAGES(length).
   */

  FAR void *paddr;

  /* The struct continues here as malloc'd array of void pointers
   * if CONFIG_BUILD_KERNEL
   */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct shmfs_object_s *shmfs_alloc_object(size_t length);

void shmfs_free_object(FAR struct shmfs_object_s *object);

#endif
