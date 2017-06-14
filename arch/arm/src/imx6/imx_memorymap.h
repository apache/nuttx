/************************************************************************************
 * arch/arm/src/imx6/imx_memorymap.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMX6_IMX_MEMORYMAP_H
#define __ARCH_ARM_SRC_IMX6_IMX_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/imx6/chip.h>

#include "mmu.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* The vectors are, by default, positioned at the beginning of the text
 * section.  Under what conditions do we have to remap these vectors?
 *
 * 1) If we are using high vectors (CONFIG_ARCH_LOWVECTORS=n).  In this case,
 *    the vectors will lie at virtual address 0xffff:000 and we will need
 *    to a) copy the vectors to another location, and b) map the vectors
 *    to that address, and
 *
 *    For the case of CONFIG_ARCH_LOWVECTORS=y, defined.  The i.MX6 boot-up
 *    logic will map the beginning of the boot memory to address 0x0000:0000
 *    using both the MMU and the AXI matrix REMAP register.  No vector copy
 *    is required because the vectors are position at the beginning of the
 *    boot memory at link time and no additional MMU mapping required.
 *
 * 2) We are not using a ROM page table.  We cannot set any custom mappings in
 *    the case and the build must conform to the ROM page table properties
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
#if defined(CONFIG_IMX6_MMDC) && !defined(CONFIG_IMX6_BOOT_SDRAM)
#  define NEED_SDRAM_CONFIGURATION 1
#endif

#undef NEED_SDRAM_MAPPING
#undef NEED_SDRAM_REMAPPING
#if defined(NEED_SDRAM_CONFIGURATION) && !defined(CONFIG_ARCH_ROMPGTABLE)
#  define NEED_SDRAM_MAPPING 1
#  define NEED_SDRAM_REMAPPING 1
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

 /* This table describes how to map a set of 1Mb pages to space the physical
 * address space of the i.MX6.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE
extern const struct section_mapping_s g_section_mapping[];
extern const size_t g_num_mappings;
#endif

/* i.MX6 External SDRAM Memory.  Final configuration.  The SDRAM was
 * configured in a temporary state to support low-level ininitialization.
 * After the SDRAM has been fully initialized, this structure is used to
 * set the SDRM in its final, fully cache-able state.
 */

#ifdef NEED_SDRAM_REMAPPING
extern const struct section_mapping_s g_operational_mapping[];
extern const size_t g_num_opmappings;
#endif

#endif /* __ARCH_ARM_SRC_IMX6_IMX_MEMORYMAP_H */
