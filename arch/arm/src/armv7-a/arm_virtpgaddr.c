/****************************************************************************
 * arch/arm/src/armv7-a/arm_virtpgaddr.c
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

#include "pgalloc.h"

#if defined(CONFIG_MM_PGALLOC) && defined(CONFIG_ARCH_PGPOOL_MAPPING)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_virtpgaddr
 *
 * Description:
 *   Check if the physical address lies in the page pool and, if so
 *   get the mapping to the virtual address in the user data area.
 *
 ****************************************************************************/

uintptr_t arm_virtpgaddr(uintptr_t paddr)
{
  /* REVISIT: Not implemented correctly.  The reverse lookup from physical
   * to virtual.  This will return a kernel accessible virtual address, but
   * not an address usable by the user code.
   *
   * The correct solutions is complex and, perhaps, will never be needed.
   */

  if (paddr >= CONFIG_ARCH_PGPOOL_PBASE && paddr < CONFIG_ARCH_PGPOOL_PEND)
    {
      return paddr - CONFIG_ARCH_PGPOOL_PBASE + CONFIG_ARCH_PGPOOL_VBASE;
    }

  return 0;
}

#endif /* CONFIG_MM_PGALLOC && CONFIG_ARCH_PGPOOL_MAPPING */
