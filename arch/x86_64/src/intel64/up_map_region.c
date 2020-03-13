/****************************************************************************
 * arch/x86/src/intel64/up_map_region.c
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

#include <debug.h>
#include <nuttx/irq.h>

#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_map_region
 *
 * Description:
 *   Map a memory region as 1:1 by MMU
 *
 ****************************************************************************/

int up_map_region(void *base, int size, int flags)
{
  uint64_t bb;
  uint64_t num_of_pages;
  uint64_t entry;
  uint64_t curr;
  int i;

  /* Round to page boundary */

  bb = (uint64_t)base & ~(PAGE_SIZE - 1);

  /* Increase size if the base address is rounded off */

  size += (uint64_t)base - bb;
  num_of_pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;

  if (bb > 0xffffffff)
    {
      return -1;  /* Only < 4GB can be mapped */
    }

  curr = bb;
  for (i = 0; i < num_of_pages; i++)
    {
      entry = (curr >> 12) & 0x7ffffff;

      pt[entry] = curr | flags;
      curr += PAGE_SIZE;
    }

  return 0;
}
