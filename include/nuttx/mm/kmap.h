/****************************************************************************
 * include/nuttx/mm/kmap.h
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

#ifndef __INCLUDE_NUTTX_MM_KMAP_H_
#define __INCLUDE_NUTTX_MM_KMAP_H_

/****************************************************************************
 * Name: kmm_map_initialize
 *
 * Description:
 *   Initialize the kernel dynamic mapping module.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void kmm_map_initialize(void);

/****************************************************************************
 * Name: kmm_map_pages
 *
 * Description:
 *   Map pages into kernel virtual memory.
 *
 * Input Parameters:
 *   pages  - Pointer to buffer that contains the physical page addresses.
 *   npages - Amount of pages.
 *   prot   - Access right flags.
 *
 * Returned Value:
 *   Pointer to the mapped virtual memory on success; NULL on failure
 *
 ****************************************************************************/

FAR void *kmm_map(FAR void **pages, size_t npages, int prot);

/****************************************************************************
 * Name: kmm_unmap
 *
 * Description:
 *   Unmap a previously allocated kernel virtual memory area.
 *
 * Input Parameters:
 *   kaddr - The kernel virtual address where the mapping begins.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void kmm_unmap(FAR void *kaddr);

/****************************************************************************
 * Name: kmm_user_map
 *
 * Description:
 *   Map a region of user memory (physical pages) for kernel use through
 *   a continuous virtual memory area.
 *
 * Input Parameters:
 *   uaddr - The user virtual address where mapping begins.
 *   size  - Size of the region.
 *
 * Returned Value:
 *   Pointer to the virtual memory area, or NULL if out of memory.
 *
 ****************************************************************************/

FAR void *kmm_user_map(FAR void *uaddr, size_t size);

/****************************************************************************
 * Name: kmm_map_user_page
 *
 * Description:
 *   Map a single physical page into kernel virtual memory. Typically just
 *   returns the kernel addressable page pool virtual address.
 *
 * Input Parameters:
 *   uaddr - The virtual address of the user page.
 *
 * Returned Value:
 *   Pointer to the new address environment, or NULL if out of memory.
 *
 ****************************************************************************/

FAR void *kmm_map_user_page(FAR void *uaddr);

#endif /* __INCLUDE_NUTTX_MM_KMAP_H_ */
