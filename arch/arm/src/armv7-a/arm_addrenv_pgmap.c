/****************************************************************************
 * arch/arm/src/armv7-a/arm_addrenv_pgmap.c
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

#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/pgalloc.h>

#include "pgalloc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_page_vaddr
 *
 * Description:
 *   Find the kernel virtual address associated with physical page.
 *
 * Input Parameters:
 *   page - The page physical address.
 *
 * Returned Value:
 *   Page kernel virtual address on success; NULL on failure.
 *
 ****************************************************************************/

uintptr_t up_addrenv_page_vaddr(uintptr_t page)
{
  return arm_pgvaddr(page);
}

/****************************************************************************
 * Name: up_addrenv_page_wipe
 *
 * Description:
 *   Wipe a page of physical memory, first mapping it into kernel virtual
 *   memory.
 *
 * Input Parameters:
 *   page - The page physical address.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_addrenv_page_wipe(uintptr_t page)
{
  uintptr_t vaddr = arm_pgvaddr(page);
  memset((void *)vaddr, 0, MM_PGSIZE);
}
