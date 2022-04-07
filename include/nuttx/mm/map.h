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
  FAR void *vaddr;
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
                FAR struct mm_map_entry_s *entry,
                FAR void *start,
                size_t length);
};

/* A structure for the task group */

struct mm_map_s
{
  sq_queue_t mm_map_sq;
  rmutex_t mm_map_mutex;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mm_map_lock
 *
 * Description:
 *   Get exclusive access current task_group's mm_map
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on failure
 *
 ****************************************************************************/

int mm_map_lock(void);

/****************************************************************************
 * Name: mm_map_unlock
 *
 * Description:
 *   Relinquish exclusive access to current task_group's mm_map
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mm_map_unlock(void);

/****************************************************************************
 * Name: mm_map_initialize
 *
 * Description:
 *   Initialization function, called only by group_initialize
 *
 * Input Parameters:
 *   mm - Pointer to the mm_map structure to be initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mm_map_initialize(FAR struct mm_map_s *mm);

/****************************************************************************
 * Name: mm_map_destroy
 *
 * Description:
 *   Uninitialization function, called only by group_release
 *
 * Input Parameters:
 *   mm - Pointer to the mm_map structure to be initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mm_map_destroy(FAR struct mm_map_s *mm);

/****************************************************************************
 * Name: mm_map_add
 *
 * Description:
 *   Adds a virtual memory area into the list of mappings
 *
 * Input Parameters:
 *   entry - A pointer to mm_map_entry_s, mapping info to be added
 *
 * Returned Value:
 *   OK        Added successfully
 *   -EINVAL:  Invalid attempt to get the semaphore
 *   -EINTR:   The wait was interrupted by the receipt of a signal.
 *   -ENOMEM:  Out of memory
 *
 ****************************************************************************/

int mm_map_add(FAR struct mm_map_entry_s *entry);

/****************************************************************************
 * Name: mm_map_next
 *
 * Description:
 *   Returns the next mapping in the list, following the argument.
 *   Can be used to iterate through all the mappings. Returns the first
 *   mapping when the argument "entry" is NULL.
 *
 * Input Parameters:
 *   entry  - Pointer to a single mapping in this task group or NULL to get
 *            the first one
 *
 * Returned Value:
 *   Pointer to the next mapping
 *
 ****************************************************************************/

FAR struct mm_map_entry_s *mm_map_next(
                           FAR const struct mm_map_entry_s *entry);

/****************************************************************************
 * Name: mm_map_find
 *
 * Description:
 *   Find the first mapping matching address and length
 *
 * Input Parameters:
 *   vaddr   - Start address of the mapped area
 *   length  - Length of the mapping
 *
 * Returned Value:
 *   Pointer to the mapping, NULL if not found
 *
 ****************************************************************************/

FAR struct mm_map_entry_s *mm_map_find(FAR const void *vaddr,
                                       size_t length);

/****************************************************************************
 * Name: mm_map_remove
 *
 * Description:
 *   Removes a virtual memory area from the list of mappings
 *   Sets the given pointer argument to NULL after successful removal
 *
 * Input Parameters:
 *   mm      - Pointer to the list of entries, from which the entry is
 *             removed. If passed mm is NULL, the function doesn't do
 *             anything, but just returns OK.
 *
 *   entry   - Pointer to the entry to be removed. If the passed entry is
 *             NULL the function doesn't do anything but just returns OK
 *
 * Returned Value:
 *   OK:       Removed successfully
 *   -EINVAL:  Invalid attempt to get the semaphore
 *   -EINTR:   The wait was interrupted by the receipt of a signal.
 *   -ENOENT:  Memory area not found
 *
 ****************************************************************************/

int mm_map_remove(FAR struct mm_map_s *mm,
                  FAR struct mm_map_entry_s *entry);

#endif /* __INCLUDE_NUTTX_MM_MAP_H */
