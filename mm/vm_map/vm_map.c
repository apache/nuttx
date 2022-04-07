/****************************************************************************
 * mm/vm_map/vm_map.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <stddef.h>
#include <queue.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/mm/vm_map.h>
#include <nuttx/kmalloc.h>
#include <sys/mman.h>
#include <assert.h>

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool vaddr_in_area(FAR const void *addr, FAR const void *start,
                          size_t length)
{
  uintptr_t u_addr = (uintptr_t)addr;
  uintptr_t u_start = (uintptr_t)start;
  return (u_addr >= u_start) && (u_addr < u_start + length);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vm_map_initialize
 *
 * Description:
 *   Allocates a task group specific vm_map stucture. Called when the group
 *   is initialized
 *
 ****************************************************************************/

void vm_map_initialize(FAR struct vm_map_s *mm)
{
  memset(mm, 0, sizeof(struct vm_map_s));
  sq_init(&mm->vm_map_sq);
  nxsem_init(&mm->vm_map_sem, 0, 1);
}

/****************************************************************************
 * Name: vm_map_destroy
 *
 * Description:
 *   De-allocates a task group specific vm_map stucture and the vm_map_sem
 *
 ****************************************************************************/

void vm_map_destroy(FAR struct vm_map_s *mm)
{
  FAR struct vm_map_entry_s *map =
    (struct vm_map_entry_s *)sq_peek(&mm->vm_map_sq);
  FAR struct vm_map_entry_s *next;
  while (map)
    {
      next = (struct vm_map_entry_s *)sq_next(((sq_entry_t *)map));
      munmap((void *)map->vaddr, map->length);

      /* file_unmap should have successfully destroyed the first mapping
       * but just in case it fails continue on the next one
       */

      DEBUGASSERT(next == (struct vm_map_entry_s *)sq_peek(&mm->vm_map_sq));
      map = next;
    }

  nxsem_destroy(&mm->vm_map_sem);
}

/****************************************************************************
 * Name: vm_map_add
 *
 * Description:
 *   Add a mapping to task group's vm_map list
 *
 ****************************************************************************/

int vm_map_add(enum vm_map_type type, union vm_map_id_u id,
               FAR const void *vaddr, size_t length)
{
  FAR struct tcb_s *tcb = nxsched_self();
  FAR struct task_group_s *group = tcb->group;
  FAR struct vm_map_s *mm = &group->tg_vm_map;
  FAR struct vm_map_entry_s *map = kmm_zalloc(sizeof(struct vm_map_entry_s));
  int ret;

  if (!map)
    {
      return -ENOMEM;
    }

  map->type = type;
  map->id = id;
  map->vaddr = vaddr;
  map->length = length;

  ret = nxsem_wait(&mm->vm_map_sem);
  if (ret < 0)
    {
      kmm_free(map);
      return ret;
    }

  sq_addfirst((sq_entry_t *)map, &mm->vm_map_sq);

  nxsem_post(&mm->vm_map_sem);

  return OK;
}

/****************************************************************************
 * Name: vm_map_find
 *
 * Description:
 *   Find the first mapping containing an address from the task group's list
 *
 ****************************************************************************/

FAR const struct vm_map_entry_s *vm_map_find(FAR const void *vaddr)
{
  FAR struct tcb_s *tcb = nxsched_self();
  FAR struct task_group_s *group = tcb->group;
  FAR struct vm_map_s *mm = &group->tg_vm_map;
  FAR struct vm_map_entry_s *map = NULL;

  if (nxsem_wait(&mm->vm_map_sem) == OK)
    {
      map = (struct vm_map_entry_s *)sq_peek(&mm->vm_map_sq);

      while (map && !vaddr_in_area(vaddr, map->vaddr, map->length))
        {
          map = (struct vm_map_entry_s *)sq_next(((sq_entry_t *)map));
        }

      nxsem_post(&mm->vm_map_sem);
    }

  return map;
}

/****************************************************************************
 * Name: vm_map_rm
 *
 * Description:
 *   Completely remove first mapping containing an address from the task
 *   group's list
 *
 ****************************************************************************/

int vm_map_rm(FAR const void *vaddr)
{
  FAR struct tcb_s *tcb = nxsched_self();
  FAR struct task_group_s *group = tcb->group;
  FAR struct vm_map_s *mm = &group->tg_vm_map;
  FAR struct vm_map_entry_s *prev;
  FAR struct vm_map_entry_s *r = NULL;

  int ret = nxsem_wait(&mm->vm_map_sem);
  if (ret < 0)
    {
      return ret;
    }

  prev = (struct vm_map_entry_s *)sq_peek(&mm->vm_map_sq);

  if (!prev)
    {
      nxsem_post(&mm->vm_map_sem);
      return -ENOENT;
    }

  if (vaddr_in_area(vaddr, prev->vaddr, prev->length))
    {
      sq_remfirst(&mm->vm_map_sq);
      r = prev;
    }
  else
    {
      while ((r = (struct vm_map_entry_s *)sq_next(((sq_entry_t *)prev))))
        {
          if (vaddr_in_area(vaddr, r->vaddr, r->length))
            {
              sq_remafter((sq_entry_t *)prev, &mm->vm_map_sq);
              break;
            }

          prev = r;
        }
    }

  nxsem_post(&mm->vm_map_sem);

  if (r)
    {
      kmm_free(r);
      return OK;
    }

  return -ENOENT;
}

#endif /* defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__) */
