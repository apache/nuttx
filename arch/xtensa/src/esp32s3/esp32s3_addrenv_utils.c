/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_addrenv_utils.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/pgalloc.h>

#include "esp32s3_addrenv.h"

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_find_page
 *
 * Description:
 *   Find the physical address of the page that backs a user virtual address
 *   in the given address environment.  On the ESP32-S3 there is no per-task
 *   page table to walk; the physical pages are recorded per region in the
 *   address environment, so the lookup reduces to selecting the window that
 *   contains 'vaddr' and indexing that region's page array.
 *
 * Returned Value:
 *   Physical address of the backing page (page-aligned) on success, or 0 if
 *   'vaddr' is not a mapped user address.
 *
 ****************************************************************************/

uintptr_t up_addrenv_find_page(arch_addrenv_t *addrenv, uintptr_t vaddr)
{
  const uintptr_t *pages;
  uintptr_t        base;
  uint16_t         count;
  uint16_t         index;

  DEBUGASSERT(addrenv);

  if (vaddr >= ESP32S3_TEXT_VBASE && vaddr < ESP32S3_TEXT_VEND)
    {
      base  = ESP32S3_TEXT_VBASE;
      pages = addrenv->textpages;
      count = addrenv->ntext;
    }
  else if (vaddr >= ESP32S3_DATA_VBASE && vaddr < ESP32S3_DATA_VEND)
    {
      base  = ESP32S3_DATA_VBASE;
      pages = addrenv->datapages;
      count = addrenv->ndata;
    }
  else if (vaddr >= ESP32S3_HEAP_VBASE && vaddr < ESP32S3_HEAP_VEND)
    {
      base  = ESP32S3_HEAP_VBASE;
      pages = addrenv->heappages;
      count = addrenv->nheap;
    }
  else
    {
      return 0;
    }

  index = (uint16_t)((vaddr - base) >> MM_PGSHIFT);
  if (index >= count)
    {
      return 0;
    }

  return pages[index];
}

/****************************************************************************
 * Name: up_addrenv_page_vaddr
 *
 * Description:
 *   Get the kernel virtual address of a physical page allocated for an
 *   address environment.  Since the PSRAM page pool is permanently mapped
 *   into the kernel, this is a fixed offset translation.
 *
 ****************************************************************************/

uintptr_t up_addrenv_page_vaddr(uintptr_t page)
{
  return esp32s3_pgvaddr(page);
}

/****************************************************************************
 * Name: up_addrenv_user_vaddr
 *
 * Description:
 *   Check if a virtual address is a user virtual address.
 *
 ****************************************************************************/

bool up_addrenv_user_vaddr(uintptr_t vaddr)
{
  return esp32s3_uservaddr(vaddr);
}

/****************************************************************************
 * Name: up_addrenv_page_wipe
 *
 * Description:
 *   Wipe a page of physical memory, first mapping it into kernel virtual
 *   memory.
 *
 ****************************************************************************/

void up_addrenv_page_wipe(uintptr_t page)
{
  esp32s3_pgwipe(page);
}

/****************************************************************************
 * Name: up_addrenv_pa_to_va
 *
 * Description:
 *   Map a physical page-pool address to the kernel virtual address that
 *   currently maps it.
 *
 ****************************************************************************/

void *up_addrenv_pa_to_va(uintptr_t pa)
{
  return (void *)esp32s3_pgvaddr(pa);
}

/****************************************************************************
 * Name: up_addrenv_va_to_pa
 *
 * Description:
 *   Map a kernel page-pool virtual address back to its physical address.
 *
 ****************************************************************************/

uintptr_t up_addrenv_va_to_pa(void *va)
{
  return esp32s3_pgpaddr((uintptr_t)va);
}

#endif /* CONFIG_ARCH_ADDRENV */
