/****************************************************************************
 * mm/map/vm_region.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/arch.h>
#include <nuttx/mm/map.h>
#include <nuttx/mm/gran.h>
#include <nuttx/pgalloc.h>
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
          ret = gran_reserve(mm->mm_map_vpages, (uintptr_t)vaddr, size);
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

/* map physical region to userspace */

FAR void *vm_map_region(uintptr_t paddr, size_t size)
{
  FAR void *vaddr;
  uintptr_t tvaddr;
  size_t    npages;
  uintptr_t tpaddr = MM_PGALIGNDOWN(paddr);
  uint      i      = 0;
  int       ret    = OK;

  DEBUGASSERT(paddr);

  size += (paddr & MM_PGMASK);
  npages = MM_NPAGES(size);
  DEBUGASSERT(npages);

  vaddr = vm_alloc_region(get_current_mm(), 0, size);
  if (vaddr)
    {
      tvaddr = (uintptr_t)vaddr;
      for (; i < npages; i++, tvaddr += MM_PGSIZE, tpaddr += MM_PGSIZE)
        {
          ret = up_shmat(&tpaddr, 1, tvaddr);
          if (ret)
            {
              goto error;
            }
        }
    }

  return (FAR void *)((uintptr_t)vaddr + (MM_PGMASK & paddr));

error:
  if (i)
    {
      /* Undo alway mapped pages */

      up_shmdt((uintptr_t)vaddr, i);
    }

  vm_release_region(get_current_mm(), vaddr, size);
  return 0;
}

/* unmap userspace device pointer */

int vm_unmap_region(FAR void *vaddr, size_t size)
{
  int ret;

  DEBUGASSERT(size && vaddr);
  size += ((uintptr_t)vaddr & MM_PGMASK);
  vaddr = (FAR void *)MM_PGALIGNDOWN(vaddr);
  ret = up_shmdt((uintptr_t)vaddr, MM_NPAGES(size));
  vm_release_region(get_current_mm(), vaddr, size);
  return ret;
}
