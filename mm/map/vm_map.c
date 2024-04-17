/****************************************************************************
 * mm/map/vm_map.c
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

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mm/map.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_VMA_MAPPING

/* map physical region to userspace */

FAR void *vm_map_region(uintptr_t paddr, size_t size)
{
  FAR void *vaddr;
  uintptr_t tvaddr;
  uint      i      = 0;
  int       ret    = OK;
  size_t    npages = MM_NPAGES(size);

  DEBUGASSERT(npages > 0);
  DEBUGASSERT(MM_ISALIGNED(paddr));

  vaddr = vm_alloc_region(get_current_mm(), 0, size);
  if (vaddr)
    {
      tvaddr = (uintptr_t)vaddr;
      for (; i < npages; i++, tvaddr += MM_PGSIZE, paddr += MM_PGSIZE)
        {
          ret = up_shmat(&paddr, 1, tvaddr);
          if (ret)
            {
              goto errorout;
            }
        }
    }

  return vaddr;

errorout:
  if (i)   /* undo mapped pages */
    {
      up_shmdt((uintptr_t)vaddr, i);
    }

  vm_release_region(get_current_mm(), vaddr, size);
  return 0;
}

/* unmap userspace device pointer */

int vm_unmap_region(FAR void *vaddr, size_t size)
{
  size_t npages = MM_NPAGES(size);
  int ret;

  DEBUGASSERT(MM_ISALIGNED(vaddr));
  DEBUGASSERT(npages);
  ret = up_shmdt((uintptr_t)vaddr, npages);
  vm_release_region(get_current_mm(), vaddr, size);
  return ret;
}

#endif /* CONFIG_ARCH_VMA_MAPPING */
