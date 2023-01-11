/****************************************************************************
 * mm/map/vm_region.c
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
#include <nuttx/mm/gran.h>
#include <debug.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vm_alloc_region
 *
 * Description:
 *   Allocate virtual memory region from the process virtual memory area.
 *
 * Input Parameters:
 *   mm    - A reference to the process mm_map struct
 *   vaddr - Virtual start address where the allocation starts, if NULL, will
 *           seek and return an address that satisfies the 'size' parameter
 *   size - Size of the area to allocate
 *
 * Returned Value:
 *   Pointer to reserved vaddr, or NULL if out-of-memory
 *
 ****************************************************************************/

FAR void *vm_alloc_region(FAR struct mm_map_s *mm, FAR void *vaddr,
                          size_t size)
{
  FAR void *ret = NULL;

  DEBUGASSERT(mm != NULL);

  if (mm->mm_map_vpages != NULL)
    {
      if (vaddr == NULL)
        {
          ret = gran_alloc(mm->mm_map_vpages, size);
        }
      else
        {
          ret = gran_reserve(mm->mm_map_vpages, (uintptr_t)vaddr,
                             size);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: vm_release_region
 *
 * Description:
 *   Free a previously allocated virtual memory region
 *
 * Input Parameters:
 *   mm    - A reference to the process' mm_map struct
 *   vaddr - Virtual start address where the allocation starts.
 *   size  - Size of the allocated area.
 *
 ****************************************************************************/

void vm_release_region(FAR struct mm_map_s *mm, FAR void *vaddr, size_t size)
{
  DEBUGASSERT(mm != NULL);

  if (mm->mm_map_vpages != NULL)
    {
      gran_free(mm->mm_map_vpages, vaddr, size);
    }
}
