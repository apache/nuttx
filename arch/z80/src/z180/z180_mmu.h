/****************************************************************************
 * arch/z80/src/z180/z180_mmu.h
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

#ifndef __ARCH_Z80_SRC_Z180_Z180_MMU_H
#define __ARCH_Z80_SRC_Z180_Z180_MMU_H

/* See arch/z80/src/z180/z180_mmu.txt for additional information */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "z180_iomap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Virtual addresses */

#ifndef CONFIG_Z180_BANKAREA_VIRTBASE
#  warning "Assuming Bank Area at virtual address 0x8000"
#  define CONFIG_Z180_BANKAREA_VIRTBASE 0x8000
#endif

#ifndef CONFIG_Z180_COMMON1AREA_VIRTBASE
#  warning "Assuming Common Area 1 at virtual address 0xc000"
#  define CONFIG_Z180_COMMON1AREA_VIRTBASE 0xc000
#endif

#if CONFIG_Z180_BANKAREA_VIRTBASE > CONFIG_Z180_COMMON1AREA_VIRTBASE
#  error "CONFIG_Z180_BANKAREA_VIRTBASE > CONFIG_Z180_COMMON1AREA_VIRTBASE"
#endif

/* Physical addresses */

#ifndef CONFIG_Z180_BANKAREA_PHYSBASE
#  warning "Assuming Bank Area 1 at physical address 0x080000"
#  define CONFIG_Z180_BANKAREA_PHYSBASE 0x08000
#endif

#ifndef CONFIG_Z180_PHYSHEAP_START
#  warning "Assuming physical heap starts at physical address 0x0c000"
#  define CONFIG_Z180_PHYSHEAP_START 0x0c000
#endif

#ifndef CONFIG_Z180_PHYSHEAP_END
#  warning "Assuming physical heap ends at physical address 0x100000"
#  define CONFIG_Z180_PHYSHEAP_END 0x100000
#endif

#if CONFIG_Z180_BANKAREA_PHYSBASE > CONFIG_Z180_PHYSHEAP_START
#  error "CONFIG_Z180_BANKAREA_PHYSBASE > CONFIG_Z180_PHYSHEAP_START"
#endif

#if CONFIG_Z180_PHYSHEAP_START > CONFIG_Z180_PHYSHEAP_END
#  error "CONFIG_Z180_PHYSHEAP_START > CONFIG_Z180_PHYSHEAP_END"
#endif

/* Each page is 4KB */

#define Z180_PAGESHIFT          (12)
#define Z180_PAGESIZE           (1 << Z180_PAGESHIFT)
#define Z180_PAGEMASK           (Z180_PAGESIZE - 1)
#define PHYS_ALIGN(phys)        ((phys) >> Z180_PAGESHIFT)
#define PHYS_ALIGNUP(phys)      (((phys) + Z180_PAGEMASK) >> Z180_PAGESHIFT)

/* Physical pages */

#define Z180_BANKAREA_PHYSPAGE  PHYS_ALIGN(CONFIG_Z180_BANKAREA_PHYSBASE)
#define Z180_PHYSHEAP_STARTPAGE PHYS_ALIGN(CONFIG_Z180_PHYSHEAP_START)
#define Z180_PHYSHEAP_ENDPAGE   PHYS_ALIGN(CONFIG_Z180_PHYSHEAP_END)
#define Z180_PHYSHEAP_NPAGES    (Z180_PHYSHEAP_ENDPAGE - Z180_PHYSHEAP_STARTPAGE + 1)

/* MMU register values */

#define Z180_CBAR_BA_VALUE  (((CONFIG_Z180_BANKAREA_VIRTBASE >> 12) & 0x0f) << CBAR_BA_SHIFT)
#define Z180_CBAR_CA_VALUE  (((CONFIG_Z180_COMMON1AREA_VIRTBASE >> 12) & 0x0f) << CBAR_CA_SHIFT)
#define Z180_CBAR_VALUE     (Z180_CBAR_BA_VALUE | Z180_CBAR_CA_VALUE)
#define Z180_BBR_VALUE      ((CONFIG_Z180_BANKAREA_PHYSBASE >> 12) & 0xff)

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: z180_mmu_lowinit
 *
 * Description:
 *   Low-level, power-up initialization of the z180 MMU.  this must be
 *   called very early in the boot process to get the basic operating
 *   memory configuration correct.  This function does *not* perform all
 *   necessray MMU initialization... only the basics needed at power-up.
 *   z80_mmu_initialize() must be called later to complete the entire MMU
 *   initialization.
 *
 ****************************************************************************/

void z180_mmu_lowinit(void) __naked;

/****************************************************************************
 * Name: z80_mmu_initialize
 *
 * Description:
 *   Perform higher level initialization of the MMU and physical memory
 *   memory management logic.  More correctly prototypes in z80_internal.h.
 *
 ****************************************************************************/

int z80_mmu_initialize(void);

#endif /* __ARCH_Z80_SRC_Z180_Z180_MMU_H */
