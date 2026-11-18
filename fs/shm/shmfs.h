/****************************************************************************
 * fs/shm/shmfs.h
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

#ifndef __FS_SHM_SHMFS_H
#define __FS_SHM_SHMFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const struct file_operations g_shmfs_operations;

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct shmfs_object_s
{
  /* Protects length and paddr against concurrent ftruncate, O_TRUNC,
   * read, write and mmap.
   */

  mutex_t lock;

  /* Total number of bytes needed from physical memory. */

  size_t length;

  /* The backing store, NULL when none is allocated.
   *
   * - In flat and protected builds this points to the allocated memory.
   *
   * - In kernel build this points to a separately allocated vector of
   *   MM_NPAGES(length) page pointers.
   */

  FAR void *paddr;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct shmfs_object_s *shmfs_alloc_object(size_t length);
FAR void *shmfs_alloc_buffer(size_t length);

void shmfs_free_object(FAR struct shmfs_object_s *object);
void shmfs_free_buffer(FAR struct shmfs_object_s *object);

/****************************************************************************
 * Name: shmfs_initialize
 *
 * Description:
 *   Create the directory of the shared memory objects at boot.
 *
 ****************************************************************************/

void shmfs_initialize(void);

#endif
