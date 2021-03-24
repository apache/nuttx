/****************************************************************************
 * arch/arm/src/sama5/sam_memorymap.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_MEMORYMAP_H
#define __ARCH_ARM_SRC_SAMA5_SAM_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/sama5/chip.h>

#include "mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The vectors are, by default, positioned at the beginning of the text
 * section.  Under what conditions do we have to remap these vectors?
 *
 * 1) If we are using high vectors (CONFIG_ARCH_LOWVECTORS=n).  In this case,
 *    the vectors will lie at virtual address 0xffff:000 and we will need
 *    to a) copy the vectors to another location, and b) map the vectors
 *    to that address, and
 *
 *    For the case of CONFIG_ARCH_LOWVECTORS=y, defined.  The SAMA5 boot-up
 *    logic will map the beginning of the boot memory to address 0x0000:0000
 *    using both the MMU and the AXI matrix REMAP register.  No vector copy
 *    is required because the vectors are position at the beginning of the
 *    boot memory at link time and no additional MMU mapping required.
 *
 * 2) We are not using a ROM page table.  We cannot set any custom mappings
 *    in the case and the build must conform to the ROM page table properties
 */

#if !defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_ARCH_ROMPGTABLE)
#  error High vector remap cannot be performed if we are using a ROM page table
#endif

/* If SDRAM needs to be configured, then it will be configured twice:  It
 * will first be configured to a temporary state to support low-level
 * initialization.  After the SDRAM has been fully initialized, SRAM be used
 * to set the SDRM in its final, fully cache-able state.
 */

#undef NEED_SDRAM_CONFIGURATION
#if defined(CONFIG_SAMA5_DDRCS) && !defined(CONFIG_SAMA5_BOOT_SDRAM)
#  define NEED_SDRAM_CONFIGURATION 1
#endif

#undef NEED_SDRAM_MAPPING
#undef NEED_SDRAM_REMAPPING
#if defined(NEED_SDRAM_CONFIGURATION) && !defined(CONFIG_ARCH_ROMPGTABLE)
#  define NEED_SDRAM_MAPPING 1
#  define NEED_SDRAM_REMAPPING 1
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This table describes how to map a set of 1Mb pages to space the physical
 * address space of the SAMA5.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE
extern const struct section_mapping_s g_section_mapping[];
extern const size_t g_num_mappings;
#endif

/* SAMA5 External SDRAM Memory.  Final configuration.  The SDRAM was
 * configured in a temporary state to support low-level ininitialization.
 * After the SDRAM has been fully initialized, this structure is used to
 * set the SDRM in its final, fully cache-able state.
 */

#ifdef NEED_SDRAM_REMAPPING
extern const struct section_mapping_s g_operational_mapping[];
extern const size_t g_num_opmappings;
#endif

#endif /* __ARCH_ARM_SRC_SAMA5_SAM_MEMORYMAP_H */
