/****************************************************************************
 * include/nuttx/mm/map.h
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

#ifndef __INCLUDE_NUTTX_MM_MAP_H
#define __INCLUDE_NUTTX_MM_MAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/queue.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Forward declarations
 ****************************************************************************/

struct task_group_s;

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A memory mapping list item */

struct mm_map_entry_s
{
  FAR struct mm_map_entry *flink;  /* this is used as sq_entry_t */
  FAR const void *vaddr;
  size_t length;
  off_t offset;
  int prot;
  int flags;
  FAR void *priv;

  /* Drivers which register mappings may also
   * implement the unmap function to undo anything done in mmap.
   * Nb. Implementation must NOT use "this_task()->group" since
   * this is not valid during process exit. The argument "group" will be
   * NULL in this case.
   */

  int (*munmap)(FAR struct task_group_s *group,
                FAR struct mm_map_entry_s *map,
                FAR void *start,
                size_t length);
};

/* A structure for the task group */

struct mm_map_s
{
  sq_queue_t mm_map_sq;
  mutex_t mm_map_mutex;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_MM_MM_MAP_H */
