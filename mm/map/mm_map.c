/****************************************************************************
 * mm/map/mm_map.c
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

#include <nuttx/mm/map.h>
#include <nuttx/pgalloc.h>
#include <nuttx/addrenv.h>
#include <stdbool.h>
#include <stddef.h>
#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <assert.h>
#include <debug.h>

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool in_range(FAR const void *start, size_t length,
                     FAR const void *range_start, size_t range_length)
{
  FAR const char *u_start = (FAR const char *)start;
  FAR const char *u_end = u_start + length;
  FAR const char *r_start = (FAR const char *)range_start;
  FAR const char *r_end = r_start + range_length;

  return (u_start >= r_start && u_start < r_end && /* Start is in range. */
          u_end >= r_start && u_end <= r_end);     /* End is in range. */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_map_lock
 *
 * Description:
 *   Get exclusive access to task_group's mm_map
 *
 ****************************************************************************/

int mm_map_lock(void)
{
  return nxrmutex_lock(&get_current_mm()->mm_map_mutex);
}

/****************************************************************************
 * Name: mm_map_unlock
 *
 * Description:
 *   Relinquish exclusive access to task_group's mm_map
 *
 ****************************************************************************/

void mm_map_unlock(void)
{
  DEBUGVERIFY(nxrmutex_unlock(&get_current_mm()->mm_map_mutex));
}

/****************************************************************************
 * Name: mm_map_initialize
 *
 * Description:
 *   Allocates a task group specific mm_map stucture. Called when the group
 *   is initialized
 *
 ****************************************************************************/

void mm_map_initialize(FAR struct mm_map_s *mm, bool kernel)
{
  sq_init(&mm->mm_map_sq);
  nxrmutex_init(&mm->mm_map_mutex);

  /* Create the virtual pages allocator for user process */

#ifdef CONFIG_ARCH_VMA_MAPPING
  if (!kernel)
    {
      mm->mm_map_vpages = gran_initialize((FAR void *)CONFIG_ARCH_SHM_VBASE,
                                          ARCH_SHM_MAXPAGES << MM_PGSHIFT,
                                          MM_PGSHIFT, MM_PGSHIFT);
      if (!mm->mm_map_vpages)
        {
          merr("gran_initialize() failed\n");
        }
    }
  else
    {
      mm->mm_map_vpages = NULL;
    }
#endif
}

/****************************************************************************
 * Name: mm_map_destroy
 *
 * Description:
 *   De-allocates a task group specific mm_map stucture and the mm_map_mutex
 *
 ****************************************************************************/

void mm_map_destroy(FAR struct mm_map_s *mm)
{
  FAR struct mm_map_entry_s *entry;

  while ((entry = (FAR struct mm_map_entry_s *)sq_remfirst(&mm->mm_map_sq)))
    {
      /* Pass null as group argument to indicate that actual MMU mappings
       * must not be touched. The process is being deleted and we don't
       * know in which context we are. Only kernel memory allocations
       * need to be freed by drivers
       */

      /* Unmap the whole region */

      if (entry->munmap)
        {
          if (entry->munmap(NULL, entry, entry->vaddr, entry->length) < 0)
            {
              /* This would be an error in the driver. It has defined munmap,
               * but is not able to munmap the full area which it has mapped
               */

              merr("Driver munmap failed\n");
            }
        }

      kmm_free(entry);
    }

  nxrmutex_destroy(&mm->mm_map_mutex);

  /* Release the virtual pages allocator */

#ifdef CONFIG_ARCH_VMA_MAPPING
  if (mm->mm_map_vpages)
    {
      gran_release(mm->mm_map_vpages);
    }
#endif
}

/****************************************************************************
 * Name: mm_map_add
 *
 * Description:
 *   Add a mapping to task group's mm_map list
 *
 ****************************************************************************/

int mm_map_add(FAR struct mm_map_entry_s *entry)
{
  FAR struct mm_map_s *mm = get_current_mm();
  FAR struct mm_map_entry_s *new_entry;
  int ret;

  if (!entry)
    {
      return -EINVAL;
    }

  /* Copy the provided mapping and add to the list */

  new_entry = kmm_malloc(sizeof(struct mm_map_entry_s));
  if (!new_entry)
    {
      return -EINVAL;
    }

  *new_entry = *entry;

  ret = nxrmutex_lock(&mm->mm_map_mutex);
  if (ret < 0)
    {
      kmm_free(new_entry);
      return ret;
    }

  sq_addfirst((sq_entry_t *)new_entry, &mm->mm_map_sq);

  nxrmutex_unlock(&mm->mm_map_mutex);

  return OK;
}

/****************************************************************************
 * Name: mm_map_next
 *
 * Description:
 *   Returns the next mapping in the list.
 *
 ****************************************************************************/

FAR struct mm_map_entry_s *mm_map_next(
                           FAR const struct mm_map_entry_s *entry)
{
  FAR struct mm_map_s *mm = get_current_mm();
  FAR struct mm_map_entry_s *next_entry = NULL;

  if (nxrmutex_lock(&mm->mm_map_mutex) == OK)
    {
      if (entry == NULL)
        {
          next_entry = (struct mm_map_entry_s *)sq_peek(&mm->mm_map_sq);
        }
      else
        {
          next_entry = (struct mm_map_entry_s *)
            sq_next(((sq_entry_t *)entry));
        }

      nxrmutex_unlock(&mm->mm_map_mutex);
    }

  return next_entry;
}

/****************************************************************************
 * Name: mm_map_find
 *
 * Description:
 *   Find the first mapping containing the range from the task group's list
 *
 ****************************************************************************/

FAR struct mm_map_entry_s *mm_map_find(FAR const void *vaddr, size_t length)
{
  FAR struct mm_map_s *mm = get_current_mm();
  FAR struct mm_map_entry_s *found_entry = NULL;

  if (nxrmutex_lock(&mm->mm_map_mutex) == OK)
    {
      found_entry = (struct mm_map_entry_s *)sq_peek(&mm->mm_map_sq);

      while (found_entry && !in_range(vaddr, length,
                                      found_entry->vaddr,
                                      found_entry->length))
        {
          found_entry = (struct mm_map_entry_s *)
            sq_next(((sq_entry_t *)found_entry));
        }

      nxrmutex_unlock(&mm->mm_map_mutex);
    }

  return found_entry;
}

/****************************************************************************
 * Name: mm_map_remove
 *
 * Description:
 *   Remove a mapping from the task  group's list
 *
 ****************************************************************************/

int mm_map_remove(FAR struct mm_map_s *mm,
                  FAR struct mm_map_entry_s *entry)
{
  FAR struct mm_map_entry_s *prev_entry;
  FAR struct mm_map_entry_s *removed_entry = NULL;
  int ret;

  if (!mm || !entry)
    {
      return OK;
    }

  ret = nxrmutex_lock(&mm->mm_map_mutex);
  if (ret < 0)
    {
      return ret;
    }

  prev_entry = (struct mm_map_entry_s *)sq_peek(&mm->mm_map_sq);

  /* Check if the list was empty */

  if (!prev_entry)
    {
      nxrmutex_unlock(&mm->mm_map_mutex);
      return -ENOENT;
    }

  /* Check if removing the first item */

  if (entry == prev_entry)
    {
      sq_remfirst(&mm->mm_map_sq);
      removed_entry = prev_entry;
    }
  else
    {
      /* Loop through the remaining items to find the one to be removed */

      while ((removed_entry = (struct mm_map_entry_s *)
              sq_next(((sq_entry_t *)prev_entry))))
        {
          if (entry == removed_entry)
            {
              sq_remafter((sq_entry_t *)prev_entry, &mm->mm_map_sq);
              break;
            }

          prev_entry = removed_entry;
        }
    }

  nxrmutex_unlock(&mm->mm_map_mutex);

  /* If the item was removed, also delete the entry struct */

  if (removed_entry)
    {
      kmm_free(removed_entry);
      return OK;
    }

  return -ENOENT;
}

#endif /* defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__) */
