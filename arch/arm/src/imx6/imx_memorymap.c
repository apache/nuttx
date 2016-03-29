/****************************************************************************
 * arch/arm/src/imx6/imx_memorymap.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "mmu.h"

#include "chip/imx_memorymap.h"
#include "imx_memorymap.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
/* This table describes how to map a set of 1Mb pages to space the physical
 * address space of the i.MX6.
 */

const struct section_mapping_s g_section_mapping[] =
{
  /* i.MX6 Address Sections Memories */

  /* If CONFIG_ARCH_LOWVECTORS is defined, then the vectors located at the
   * beginning of the .text region must appear at address at the address
   * specified in the VBAR.  There are two ways to accomplish this:
   *
   *   1. By explicitly mapping the beginning of .text region with a page
   *      table entry so that the virtual address zero maps to the beginning
   *      of the .text region.  VBAR == 0x0000:0000.
   *
   *   2. Set the Cortex-A5 VBAR register so that the vector table address
   *      is moved to a location other than 0x0000:0000.
   *
   *      This is the method used by this logic.
   *
   * The system always boots from the ROM memory at address 0x0.  After
   * reset, and until the Remap command is performed, the OCRAM is accessible
   * at address 0x0090 0000.
   *
   * If we are executing from external SDRAM, then a secondary bootloader must
   * have loaded us into SDRAM.  In this case, simply set the VBAR register
   * to the address of the vector table (not necessary at the beginning
   * or SDRAM).
   */

  { IMX_ROMCP_PSECTION,     IMX_ROMCP_VSECTION,     /* Boot ROM (ROMCP) */
    IMX_ROMCP_MMUFLAGS,     IMX_ROMCP_NSECTIONS
  },
  { IMX_DMA_PSECTION,       IMX_DMA_VSECTION,       /* "DMA" sectinon peripherals */
    IMX_DMA_MMUFLAGS,       IMX_DMA_NSECTIONS
  },
  { IMX_GPV2_PSECTION,      IMX_GPV2_VSECTION,      /* GPV_2 PL301 (per1) configuration port */
    IMX_GPV2_MMUFLAGS,      IMX_GPV2_NSECTIONS
  },
  { IMX_GPV3_PSECTION,      IMX_GPV3_VSECTION,      /* GPV_3 PL301 (per2) configuration port */
    IMX_GPV3_MMUFLAGS,      IMX_GPV3_NSECTIONS
  },
  { IMX_GPV4_PSECTION,      IMX_GPV4_VSECTION,      /* GPV_4 PL301 (fast3) configuration port */
    IMX_GPV4_MMUFLAGS,      IMX_GPV4_NSECTIONS
  },
  { IMX_OCRAM_PSECTION,     IMX_OCRAM_VSECTION,     /* OCRAM */
    IMX_OCRAM_MMUFLAGS,     IMX_OCRAM_NSECTIONS
  },
  { IMX_ARMMP_PSECTION,     IMX_ARMMP_VSECTION,     /* ARM MP */
    IMX_ARMMP_MMUFLAGS,     IMX_ARMMP_NSECTIONS
  },
  { IMX_GPV0PL301_PSECTION, IMX_GPV0PL301_VSECTION, /* GPV0 PL301 (fast2) configuration port */
    IMX_GPV0PL301_MMUFLAGS, IMX_GPV0PL301_NSECTIONS
  },
  { IMX_GPV1PL301_PSECTION, IMX_GPV1PL301_VSECTION, /* GPV1 PL301 (fast1) configuration port */
    IMX_GPV1PL301_MMUFLAGS, IMX_GPV1PL301_NSECTIONS
  },
  { IMX_PCIE_PSECTION,      IMX_PCIE_VSECTION,      /* PCIe */
    IMX_PCIE_MMUFLAGS,      IMX_PCIE_NSECTIONS
  },
  { IMX_AIPS1_PSECTION,     IMX_AIPS1_VSECTION,     /* Peripheral IPs via AIPS-1 */
    IMX_AIPS1_MMUFLAGS,     IMX_AIPS1_NSECTIONS
  },
  { IMX_AIPS2_PSECTION,     IMX_AIPS2_VSECTION,     /* Peripheral IPs via AIPS-2 */
    IMX_AIPS2_MMUFLAGS,     IMX_AIPS2_NSECTIONS
  },
  { IMX_SATA_PSECTION,      IMX_SATA_VSECTION,      /* SATA */
    IMX_SATA_MMUFLAGS,      IMX_SATA_NSECTIONS
  },
  { IMX_IPU1_PSECTION,      IMX_IPU1_VSECTION,      /* IPU-1 */
    IMX_IPU1_MMUFLAGS,      IMX_IPU1_NSECTIONS
  },
  { IMX_IPU2_PSECTION,      IMX_IPU2_VSECTION,      /* IPU-2 */
    IMX_IPU2_MMUFLAGS,      IMX_IPU2_NSECTIONS
  },

#ifdef CONFIG_IMX6_EIM
  { IMX_EIM_PSECTION,       IMX_EIM_VSECTION,       /* EIM - (NOR/SRAM) */
    IMX_EIM_MMUFLAGS,       IMX_EIM_NSECTIONS
  },
#endif

  /* i.MX6 External SDRAM Memory.  The SDRAM is not usable until it has been
   * initialized.  If we are running out of SDRAM now, we can assume that some
   * second level boot loader has properly configured SRAM for us.  In that
   * case, we set the MMU flags for the final, fully cache-able state.
   *
   * Also, in this case, the mapping for the SDRAM was done in arm_head.S and
   * need not be repeated here.
   *
   * If we are running from OCRAM or NOR flash, then we will need to configure
   * the SDRAM ourselves.  In this case, we set the MMU flags to the strongly
   * ordered, non-cacheable state.  We need this direct access to SDRAM in
   * order to configure it.  Once SDRAM has been initialized, it will be re-
   * configured in its final state.
   */

#ifdef NEED_SDRAM_MAPPING
  { IMX_MMDCDDR_PSECTION,   IMX_MMDCDDR_VSECTION,   /* MMDC-DDR Controller */
    MMU_STRONGLY_ORDERED,   IMX_MMDCDDR_NSECTIONS
  },
#else
  { IMX_MMDCDDR_PSECTION,   IMX_MMDCDDR_VSECTION,   /* MMDC-DDR Controller */
    IMX_MMDCDDR_MMUFLAGS,   IMX_MMDCDDR_NSECTIONS
  },
#endif

  /* LCDC Framebuffer.  This entry reprograms a part of one of the above
   * regions, making it non-cacheable and non-buffereable.
   *
   * If SDRAM will be reconfigured, then we will defer setup of the framebuffer
   * until after the SDRAM remapping (since the framebuffer problem resides) in
   * SDRAM.
   */

#if defined(CONFIG_IMX6_LCDC) && !defined(NEED_SDRAM_REMAPPING)
  { CONFIG_IMX6_LCDC_FB_PBASE, CONFIG_IMX6_LCDC_FB_VBASE,
    MMU_IOFLAGS, IMX6_LCDC_FBNSECTIONS
  },
#endif
};

/* The number of entries in the mapping table */

#define NMAPPINGS \
  (sizeof(g_section_mapping) / sizeof(struct section_mapping_s))

const size_t g_num_mappings = NMAPPINGS;

#endif /* CONFIG_ARCH_ROMPGTABLE */

/* i.MX6 External SDRAM Memory.  Final configuration.  The SDRAM was
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

  { IMX_MMDCDDR_PSECTION,   IMX_MMDCDDR_VSECTION,   /* MMDC-DDR Controller */
    IMX_MMDCDDR_MMUFLAGS,   IMX_MMDCDDR_NSECTIONS
  },

  /* LCDC Framebuffer.  This entry reprograms a part of one of the above
   * regions, making it non-cacheable and non-buffereable.
   */

#ifdef CONFIG_IMX6_LCDC
  {CONFIG_IMX6_LCDC_FB_PBASE, CONFIG_IMX6_LCDC_FB_VBASE,
    MMU_IOFLAGS, IMX6_LCDC_FBNSECTIONS
  },
#endif

};

/* The number of entries in the operational mapping table */

#define NREMAPPINGS \
  (sizeof(g_operational_mapping) / sizeof(struct section_mapping_s))

const size_t g_num_opmappings = NREMAPPINGS;

#endif /* NEED_SDRAM_REMAPPING */
