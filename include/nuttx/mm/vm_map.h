/****************************************************************************
 * include/nuttx/mm/vm_map.h
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

#ifndef __INCLUDE_NUTTX_MM_VM_MAP_H
#define __INCLUDE_NUTTX_MM_VM_MAP_H

#ifdef CONFIG_MM_VM_MAP

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/fs.h>
#include <queue.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Type of the mapping */

enum vm_map_type
{
  VM_MAP_ANONYMOUS,
  VM_MAP_NAMED,
  VM_MAP_FILE,
  VM_MAP_SHM
};

/* Id of the mapping */

union vm_map_id_u
{
  FAR const char *name;
  FAR struct inode *inode;
  int shmid;
  uintptr_t val;
};

/* A vm mapping list item */

struct vm_map_entry_s
{
  FAR struct vm_map_entry *flink;  /* this is used as sq_entry_t */
  enum vm_map_type type;
  union vm_map_id_u id;
  FAR const void *vaddr;
  size_t length;
};

/* A structure for the task group */

struct vm_map_s
{
  sq_queue_t vm_map_sq;
  sem_t vm_map_sem;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* These may only be called within kernel context */

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)

/****************************************************************************
 * Name: vm_map_initialize
 *
 * Description:
 *   Initialization function, called only by group_initialize
 *
 * Input Parameters:
 *   mm - Pointer to the vm_map structure to be initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void vm_map_initialize(FAR struct vm_map_s *mm);

/****************************************************************************
 * Name: vm_map_destroy
 *
 * Description:
 *   Uninitialization function, called only by group_release
 *
 * Input Parameters:
 *   mm - Pointer to the vm_map structure to be initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void vm_map_destroy(FAR struct vm_map_s *mm);

/****************************************************************************
 * Name: vm_map_add
 *
 * Description:
 *   Adds a virtual memory area into the list of mappings
 *
 * Input Parameters:
 *   type   - Type of the mapping, defined by enum vm_map_type
 *   id     - Id of the type. E.g. name or file pointer
 *   vaddr  - Start address of the vm region
 *   length - Size of the vm region
 *
 * Returned Value:
 *   OK        Added succesfully
 *   -EINVAL:  Invalid attempt to get the semaphore
 *   -EINTR:   The wait was interrupted by the receipt of a signal.
 *   -ENOMEM:  Out of memory
 *
 ****************************************************************************/

int vm_map_add(enum vm_map_type type, union vm_map_id_u id,
               FAR const void *vaddr, size_t length);

/****************************************************************************
 * Name: vm_map_find
 *
 * Description:
 *   Find the first mapping containing an address from the task group's list
 *
 * Input Parameters:
 *   vaddr   - Address within the mapped aread
 *
 * Returned Value:
 *   Pointer to the mapping, NULL if not found
 *
 ****************************************************************************/

FAR const struct vm_map_entry_s *vm_map_find(FAR const void *vaddr);

/****************************************************************************
 * Name: vm_map_rm
 *
 * Description:
 *   Removes a virtual memory area from the list of mappings
 *
 * Input Parameters:
 *   vaddr   - Address within the mapped aread
 *
 * Returned Value:
 *   OK:       Removed succesfully
 *   -EINVAL:  Invalid attempt to get the semaphore
 *   -EINTR:   The wait was interrupted by the receipt of a signal.
 *   -ENOENT:  Memory area not found
 *
 ****************************************************************************/

int vm_map_rm(FAR const void *vaddr);

#endif /* defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__) */

#endif /* CONFIG_MM_VM_MAP */

#endif /* __INCLUDE_NUTTX_MM_VM_MAP_H */
