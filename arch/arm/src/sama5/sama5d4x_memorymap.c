/****************************************************************************
 * arch/arm/src/sama5/sama5d4x_memorymap.c
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "mmu.h"

#include "chip/sam_memorymap.h"
#include "sam_lcd.h"
#include "sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This table describes how to map a set of 1Mb pages to space the physical
 * address space of the SAMA5.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE
const struct section_mapping_s g_section_mapping[] =
{
  /* SAMA5 Internal Memories */

  /* If CONFIG_ARCH_LOWVECTORS is defined, then the vectors located at the
   * beginning of the .text region must appear at address at the address
   * specified in the VBAR.  There are three ways to accomplish this:
   *
   *   1. By explicitly mapping the beginning of .text region with a page
   *      table entry so that the virtual address zero maps to the beginning
   *      of the .text region.  VBAR == 0x0000:0000.
   *
   *   2. A second way is to map the use the AXI MATRIX remap register to
   *      map physical address zero to the beginning of the text region,
   *      either internal SRAM or EBI CS 0.  Then we can set an identity
   *      mapping to map the boot region at 0x0000:0000 to virtual address
   *      0x0000:00000.   VBAR == 0x0000:0000.
   *
   *      This method is used when booting from ISRAM or NOR FLASH.  In
   *      that case, vectors must lie at the beginning of NOFR FLASH.
   *
   *   3. Set the Cortex-A5 VBAR register so that the vector table address
   *      is moved to a location other than 0x0000:0000.
   *
   *      This is the method used when booting from SDRAM.
   *
   * - When executing from NOR FLASH, the first level bootloader is supposed
   *   to provide the AXI MATRIX mapping for us at boot time base on the state
   *   of the BMS pin.  However, I have found that in the test environments
   *   that I use, I cannot always be assured of that physical address mapping.
   *
   *   So we do both here.  If we are executing from NOR FLASH, then we provide
   *   the MMU to map the physical address of FLASH to address 0x0000:0000;
   *
   * - If we are executing out of ISRAM, then the SAMA5 primary bootloader
   *   probably copied us into ISRAM and set the AXI REMAP bit for us.
   *
   * - If we are executing from external SDRAM, then a secondary bootloader must
   *   have loaded us into SDRAM.  In this case, simply set the VBAR register
   *   to the address of the vector table (not necessary at the beginning
   *   or SDRAM).
   */

#if defined(CONFIG_ARCH_LOWVECTORS) && !defined(CONFIG_SAMA5_BOOT_ISRAM) && \
    !defined(CONFIG_SAMA5_BOOT_SDRAM)
  { CONFIG_FLASH_START,    0x00000000,
    MMU_ROMFLAGS,          1
  },
#else
  { SAM_BOOTMEM_PSECTION,  SAM_BOOTMEM_VSECTION,
    SAM_BOOTMEM_MMUFLAGS,  SAM_BOOTMEM_NSECTIONS
  },
#endif

  { SAM_ROM_PSECTION,      SAM_ROM_VSECTION,
    SAM_ROM_MMUFLAGS,      SAM_ROM_NSECTIONS
  },
  { SAM_NFCSRAM_PSECTION,  SAM_NFCSRAM_VSECTION,
    SAM_NFCSRAM_MMUFLAGS,  SAM_NFCSRAM_NSECTIONS
  },

#ifndef CONFIG_PAGING /* Internal SRAM is already fully mapped */
  { SAM_ISRAM_PSECTION,    SAM_ISRAM_VSECTION,
    SAM_ISRAM_MMUFLAGS,    SAM_ISRAM_NSECTIONS
  },
#endif

  { SAM_VDEC_PSECTION,     SAM_VDEC_VSECTION,
    SAM_VDEC_MMUFLAGS,     SAM_VDEC_NSECTIONS
  },
  { SAM_SMD_PSECTION,      SAM_SMD_VSECTION,
    SAM_SMD_MMUFLAGS,      SAM_SMD_NSECTIONS
  },
  { SAM_UDPHSRAM_PSECTION, SAM_UDPHSRAM_VSECTION,
    SAM_UDPHSRAM_MMUFLAGS, SAM_UDPHSRAM_NSECTIONS
  },
  { SAM_UHPOHCI_PSECTION,  SAM_UHPOHCI_VSECTION,
    SAM_UHPOHCI_MMUFLAGS,  SAM_UHPOHCI_NSECTIONS
  },
  { SAM_UHPEHCI_PSECTION,  SAM_UHPEHCI_VSECTION,
    SAM_UHPEHCI_MMUFLAGS,  SAM_UHPEHCI_NSECTIONS
  },
  { SAM_AXIMX_PSECTION,    SAM_AXIMX_VSECTION,
    SAM_AXIMX_MMUFLAGS,    SAM_AXIMX_NSECTIONS
  },
  { SAM_DAP_PSECTION,      SAM_DAP_VSECTION,
    SAM_DAP_MMUFLAGS,      SAM_DAP_NSECTIONS
  },
  { SAM_L2CC_PSECTION,     SAM_L2CC_VSECTION,
    SAM_L2CC_MMUFLAGS,     SAM_L2CC_NSECTIONS
  },

  /* SAMA5 CS0 External Memories */

#ifdef CONFIG_SAMA5_EBICS0
  { SAM_EBICS0_PSECTION, SAM_EBICS0_VSECTION,
    SAM_EBICS0_MMUFLAGS, SAM_EBICS0_NSECTIONS
  },
#endif

  /* SAMA5 External SDRAM Memory.  The SDRAM is not usable until it has been
   * initialized.  If we are running out of SDRAM now, we can assume that some
   * second level boot loader has properly configured SRAM for us.  In that
   * case, we set the MMU flags for the final, fully cache-able state.
   *
   * Also, in this case, the mapping for the SDRAM was done in arm_head.S and
   * need not be repeated here.
   *
   * If we are running from ISRAM or NOR flash, then we will need to configure
   * the SDRAM ourselves.  In this case, we set the MMU flags to the strongly
   * ordered, non-cacheable state.  We need this direct access to SDRAM in
   * order to configure it.  Once SDRAM has been initialized, it will be re-
   * configured in its final state.
   */

#ifdef NEED_SDRAM_MAPPING
  { SAM_DDRCS_PSECTION,    SAM_DDRCS_VSECTION,
    MMU_STRONGLY_ORDERED,  SAM_DDRCS_NSECTIONS
  },
#endif

  /* SAMA5 CS1-3 External Memories */

#ifdef CONFIG_SAMA5_EBICS1
  { SAM_EBICS1_PSECTION,   SAM_EBICS1_VSECTION,
    SAM_EBICS1_MMUFLAGS,   SAM_EBICS1_NSECTIONS
  },
#endif
#ifdef CONFIG_SAMA5_EBICS2
  { SAM_EBICS2_PSECTION,   SAM_EBICS2_VSECTION,
    SAM_EBICS2_MMUFLAGS,   SAM_EBICS2_NSECTIONS
  },
#endif
#ifdef CONFIG_SAMA5_EBICS3
  { SAM_EBICS3_PSECTION,   SAM_EBICS3_VSECTION,
    SAM_EBICS3_MMUFLAGS,   SAM_EBICS3_NSECTIONS
  },
#endif
#ifdef CONFIG_SAMA5_HAVE_NAND
  { SAM_NFCCR_PSECTION,   SAM_NFCCR_VSECTION,
    SAM_NFCCR_MMUFLAGS,   SAM_NFCCR_NSECTIONS
  },
#endif

  /* SAMA5 Internal Peripherals
   *
   * Naming of peripheral sections differs between the SAMA5D3 and SAMA5D4.
   * There is nothing called SYSC in the SAMA5D4 memory map.  The third
   * peripheral section is un-named in the SAMA5D4 memory map, but I have
   * chosen the name PERIPHC for this usage.
   */

  { SAM_PERIPHA_PSECTION, SAM_PERIPHA_VSECTION,
    SAM_PERIPHA_MMUFLAGS, SAM_PERIPHA_NSECTIONS
  },
  { SAM_PERIPHB_PSECTION, SAM_PERIPHB_VSECTION,
    SAM_PERIPHB_MMUFLAGS, SAM_PERIPHB_NSECTIONS
  },
  { SAM_PERIPHC_PSECTION, SAM_PERIPHC_VSECTION,
    SAM_PERIPHC_MMUFLAGS, SAM_PERIPHC_NSECTIONS
  },

  /* LCDC Framebuffer.  This entry reprograms a part of one of the above
   * regions, making it non-cacheable and non-buffereable.
   *
   * If SDRAM will be reconfigured, then we will defer setup of the framebuffer
   * until after the SDRAM remapping (since the framebuffer problem resides) in
   * SDRAM.
   */

#if defined(CONFIG_SAMA5_LCDC) && !defined(NEED_SDRAM_REMAPPING)
  { CONFIG_SAMA5_LCDC_FB_PBASE, CONFIG_SAMA5_LCDC_FB_VBASE,
    MMU_IOFLAGS, SAMA5_LCDC_FBNSECTIONS
  },
#endif
};

/* The number of entries in the mapping table */

#define NMAPPINGS \
  (sizeof(g_section_mapping) / sizeof(struct section_mapping_s))

const size_t g_num_mappings = NMAPPINGS;

#endif /* CONFIG_ARCH_ROMPGTABLE */

/* SAMA5 External SDRAM Memory.  Final configuration.  The SDRAM was
 * configured in a temporary state to support low-level ininitialization.
 * After the SDRAM has been fully initialized, this structure is used to
 * set the SDRM in its final, fully cache-able state.
 */

#ifdef NEED_SDRAM_REMAPPING
const struct section_mapping_s g_operational_mapping[] =
{
  /* This entry reprograms the SDRAM entry, making it cacheable and
   * bufferable.
   */

  { SAM_DDRCS_PSECTION, SAM_DDRCS_VSECTION,
    SAM_DDRCS_MMUFLAGS, SAM_DDRCS_NSECTIONS
  },

  /* LCDC Framebuffer.  This entry reprograms a part of one of the above
   * regions, making it non-cacheable and non-buffereable.
   */

#ifdef CONFIG_SAMA5_LCDC
  { CONFIG_SAMA5_LCDC_FB_PBASE, CONFIG_SAMA5_LCDC_FB_VBASE,
    MMU_IOFLAGS, SAMA5_LCDC_FBNSECTIONS
  },
#endif
};

/* The number of entries in the operational mapping table */

#define NREMAPPINGS \
  (sizeof(g_operational_mapping) / sizeof(struct section_mapping_s))

const size_t g_num_opmappings = NREMAPPINGS;

#endif /* NEED_SDRAM_REMAPPING */
