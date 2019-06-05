/****************************************************************************************************
 * arch/arm/src/lpc54xx/lpc54_emc.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_EMC_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_EMC_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

#define LPC54_EMC_CS0                     0
#define LPC54_EMC_CS1                     1
#define LPC54_EMC_CS2                     2
#define LPC54_EMC_CS3                     3
#define LPC54_EMC_NCS                     4

/* Register offsets *********************************************************************************/

#define LPC54_EMC_CONTROL_OFFSET          0x0000  /* Controls operation of the memory controller */
#define LPC54_EMC_STATUS_OFFSET           0x0004  /* Provides EMC status information */
#define LPC54_EMC_CONFIG_OFFSET           0x0008  /* Configures operation of the memory controller */
#define LPC54_EMC_DYNCONTROL_OFFSET       0x0020  /* Controls dynamic memory operation */
#define LPC54_EMC_DYNREFRESH_OFFSET       0x0024  /* Configures dynamic memory refresh */
#define LPC54_EMC_DYNREADCONFIG_OFFSET    0x0028  /* Configures dynamic memory read strategy */
#define LPC54_EMC_DYNRP_OFFSET            0x0030  /* Precharge command period */
#define LPC54_EMC_DYNRAS_OFFSET           0x0034  /* Active to precharge command period */
#define LPC54_EMC_DYNSREX_OFFSET          0x0038  /* Self-refresh exit time */
#define LPC54_EMC_DYNAPR_OFFSET           0x003c  /* Last-data-out to active command time */
#define LPC54_EMC_DYNDAL_OFFSET           0x0040  /* Data-in to active command time */
#define LPC54_EMC_DYNWR_OFFSET            0x0044  /* Write recovery time */
#define LPC54_EMC_DYNRC_OFFSET            0x0048  /* Selects the active to active command period */
#define LPC54_EMC_DYNRFC_OFFSET           0x004c  /* Selects the auto-refresh period */
#define LPC54_EMC_DYNXSR_OFFSET           0x0050  /* Time for exit self-refresh to active command */
#define LPC54_EMC_DYNRRD_OFFSET           0x0054  /* Latency for active bank A to active bank B */
#define LPC54_EMC_DYNMRD_OFFSET           0x0058  /* Time for load mode register to active command */
#define LPC54_EMC_STATEXTWAIT_OFFSET      0x0080  /* Time for long static memory read and write transfers */

/* Per-chip select dynamic memory registers */

#define LPC54_EMC_DYNCS_OFFSET(n)         (0x0100 + ((uintptr_t)(n) << 5))
#define LPC54_EMC_DYNCONFIG_OFFSET        0x0000  /* Configuration information for CSn */
#define LPC54_EMC_DYNRASCAS_OFFSET        0x0004  /* RAS and CAS latencies for CSn */

#define LPC54_EMC_DYNCONFIGn_OFFSET(n)    (0x0100 + ((uintptr_t)(n) << 5))
#define LPC54_EMC_DYNRASCASn_OFFSET(n)    (0x0104 + ((uintptr_t)(n) << 5))

/* Per-chip select static memory registers */

#define LPC54_EMC_STATCS_OFFSET(n)        (0x0200 + ((uintptr_t)(n) << 5))
#define LPC54_EMC_STATCONFIG_OFFSET       0x0000  /* Configuration for CSn */
#define LPC54_EMC_STATWAITWEN_OFFSET      0x0004  /* Delay to write enable */
#define LPC54_EMC_STATWAITOEN_OFFSET      0x0008  /* Delay to output enable */
#define LPC54_EMC_STATWAITRD_OFFSET       0x000c  /* Delay to read access */
#define LPC54_EMC_STATWAITPAGE_OFFSET     0x0010  /* Delay for asynchronous page mode accesses */
#define LPC54_EMC_STATWAITWR_OFFSET       0x0014  /* Delay from EMC_CS0 to a write access */
#define LPC54_EMC_STATWAITTURN_OFFSET     0x0018  /* Number of bus turnaround cycles */

#define LPC54_EMC_STATCONFIGn_OFFSET(n)   (0x0200 + ((uintptr_t)(n) << 5))
#define LPC54_EMC_STATWAITWENn_OFFSET(n)  (0x0204 + ((uintptr_t)(n) << 5))
#define LPC54_EMC_STATWAITOENn_OFFSET(n)  (0x0208 + ((uintptr_t)(n) << 5))
#define LPC54_EMC_STATWAITRDn_OFFSET(n)   (0x020c + ((uintptr_t)(n) << 5))
#define LPC54_EMC_STATWAITPAGEn_OFFSET(n) (0x0210 + ((uintptr_t)(n) << 5))
#define LPC54_EMC_STATWAITWRn_OFFSET(n)   (0x0214 + ((uintptr_t)(n) << 5))
#define LPC54_EMC_STATWAITTURNn_OFFSET(n) (0x0218 + ((uintptr_t)(n) << 5))

/* Register addresses *******************************************************************************/

#define LPC54_EMC_CONTROL                 (LPC54_EMC_BASE + LPC54_EMC_CONTROL_OFFSET)
#define LPC54_EMC_STATUS                  (LPC54_EMC_BASE + LPC54_EMC_STATUS_OFFSET)
#define LPC54_EMC_CONFIG                  (LPC54_EMC_BASE + LPC54_EMC_CONFIG_OFFSET)
#define LPC54_EMC_DYNCONTROL              (LPC54_EMC_BASE + LPC54_EMC_DYNCONTROL_OFFSET)
#define LPC54_EMC_DYNREFRESH              (LPC54_EMC_BASE + LPC54_EMC_DYNREFRESH_OFFSET)
#define LPC54_EMC_DYNREADCONFIG           (LPC54_EMC_BASE + LPC54_EMC_DYNREADCONFIG_OFFSET)
#define LPC54_EMC_DYNRP                   (LPC54_EMC_BASE + LPC54_EMC_DYNRP_OFFSET)
#define LPC54_EMC_DYNRAS                  (LPC54_EMC_BASE + LPC54_EMC_DYNRAS_OFFSET)
#define LPC54_EMC_DYNSREX                 (LPC54_EMC_BASE + LPC54_EMC_DYNSREX_OFFSET)
#define LPC54_EMC_DYNAPR                  (LPC54_EMC_BASE + LPC54_EMC_DYNAPR_OFFSET)
#define LPC54_EMC_DYNDAL                  (LPC54_EMC_BASE + LPC54_EMC_DYNDAL_OFFSET)
#define LPC54_EMC_DYNWR                   (LPC54_EMC_BASE + LPC54_EMC_DYNWR_OFFSET)
#define LPC54_EMC_DYNRC                   (LPC54_EMC_BASE + LPC54_EMC_DYNRC_OFFSET)
#define LPC54_EMC_DYNRFC                  (LPC54_EMC_BASE + LPC54_EMC_DYNRFC_OFFSET)
#define LPC54_EMC_DYNXSR                  (LPC54_EMC_BASE + LPC54_EMC_DYNXSR_OFFSET)
#define LPC54_EMC_DYNRRD                  (LPC54_EMC_BASE + LPC54_EMC_DYNRRD_OFFSET)
#define LPC54_EMC_DYNMRD                  (LPC54_EMC_BASE + LPC54_EMC_DYNMRD_OFFSET)
#define LPC54_EMC_STATEXTWAIT             (LPC54_EMC_BASE + LPC54_EMC_STATEXTWAIT_OFFSET)

/* Per-chip select dynamic memory registers */

#define LPC54_EMC_DYNCS_BASE(n)           (LPC54_EMC_BASE + LPC54_EMC_DYNCS_OFFSET(n))
#define LPC54_EMC_DYNCONFIG(n)            (LPC54_EMC_DYNCS_BASE(n) + LPC54_EMC_DYNCONFIG_OFFSET)
#define LPC54_EMC_DYNRASCAS(n)            (LPC54_EMC_DYNCS_BASE(n) + LPC54_EMC_DYNRASCAS_OFFSET)

/* Per-chip select static memory registers */

#define LPC54_EMC_STATCS_BASE(n)          (LPC54_EMC_BASE + LPC54_EMC_STATCS_OFFSET(n))
#define LPC54_EMC_STATCONFIG(n)           (LPC54_EMC_STATCS_BASE(n) + LPC54_EMC_STATCONFIG_OFFSET)
#define LPC54_EMC_STATWAITWEN(n)          (LPC54_EMC_STATCS_BASE(n) + LPC54_EMC_STATWAITWEN_OFFSET)
#define LPC54_EMC_STATWAITOEN(n)          (LPC54_EMC_STATCS_BASE(n) + LPC54_EMC_STATWAITOEN_OFFSET)
#define LPC54_EMC_STATWAITRD(n)           (LPC54_EMC_STATCS_BASE(n) + LPC54_EMC_STATWAITRD_OFFSET)
#define LPC54_EMC_STATWAITPAGE(n)         (LPC54_EMC_STATCS_BASE(n) + LPC54_EMC_STATWAITPAGE_OFFSET)
#define LPC54_EMC_STATWAITWR(n)           (LPC54_EMC_STATCS_BASE(n) + LPC54_EMC_STATWAITWR_OFFSET)
#define LPC54_EMC_STATWAITTURN(n)         (LPC54_EMC_STATCS_BASE(n) + LPC54_EMC_STATWAITTURN_OFFSET)

/* Register bit definitions *************************************************************************/

/* Controls operation of the memory controller */

#define EMC_CONTROL_E                     (1 << 0)  /* Bit 0:  EMC Enable */
#define EMC_CONTROL_M                     (1 << 1)  /* Bit 1:  Address mirror */
#define EMC_CONTROL_L                     (1 << 2)  /* Bit 2:  Low-power mode */

/* Provides EMC status information */

#define EMC_STATUS_B                      (1 << 0)  /* Bit 0:  Busy */
#define EMC_STATUS_S                      (1 << 1)  /* Bit 1:  Write buffer status */
#define EMC_STATUS_SA                     (1 << 2)  /* Bit 2:  Self-refresh acknowledge */

/* Configures operation of the memory controller */

#define EMC_CONFIG_EM                     (1 << 0)  /* Bit 0:  EM Endian mode */
#define EMC_CONFIG_CLKR                   (1 << 8)  /* Bit 8:  Must be zero */

/* Controls dynamic memory operation */

#define EMC_DYNCONTROL_CE                 (1 << 0)  /* Bit 0:  Dynamic memory clock enable */
#define EMC_DYNCONTROL_CS                 (1 << 1)  /* Bit 1:  Dynamic memory clock control */
#define EMC_DYNCONTROL_SR                 (1 << 2)  /* Bit 2:  Self-refresh request, EMCSREFREQ */
#define EMC_DYNCONTROL_MMC                (1 << 5)  /* Bit 5:  Memory clock control */
#define EMC_DYNCONTROL_I_SHIFT            (7)       /* Bit 7-8: SDRAM initialization */
#define EMC_DYNCONTROL_I_MASK             (3 << EMC_DYNCONTROL_I_SHIFT)
#  define EMC_DYNCONTROL_I_NORMAL         (0 << EMC_DYNCONTROL_I_SHIFT) /* Issue SDRAM NORMAL operation command */
#  define EMC_DYNCONTROL_I_MODE           (1 << EMC_DYNCONTROL_I_SHIFT) /* Issue SDRAM MODE command */
#  define EMC_DYNCONTROL_I_PALL           (2 << EMC_DYNCONTROL_I_SHIFT) /* Issue SDRAM PALL (precharge all) command */
#  define EMC_DYNCONTROL_I_NOP            (3 << EMC_DYNCONTROL_I_SHIFT) /* Issue SDRAM NOP (no operation) command */

/* Configures dynamic memory refresh */

#define EMC_DYNREFRESH_SHIFT              (0)       /* Bits 0-10: Refresh timer */
#define EMC_DYNREFRESH_MASK               (0x7ff << EMC_DYNREFRESH_SHIFT)
#  define EMC_DYNREFRESH_DISABLE(n)       (0 << EMC_DYNREFRESH_SHIFT)
#  define EMC_DYNREFRESH(n)               ((uint32_t)((n) >> 4) << EMC_DYNREFRESH_SHIFT)

/* Configures dynamic memory read strategy */

#define EMC_DYNREADCONFIG_SHIFT           (0)       /* Bits 0-1: Read data strategy */
#define EMC_DYNREADCONFIG_MASK            (3 << EMC_DYNREADCONFIG_SHIFT)
#  define EMC_DYNREADCONFIG(n)            ((uint32_t)(n) << EMC_DYNREADCONFIG_SHIFT)
#  define EMC_DYNREADCONFIG_PLUS0         (1 << EMC_DYNREADCONFIG_SHIFT) /* Using EMCCLKDELAY */
#  define EMC_DYNREADCONFIG_PLUS1         (2 << EMC_DYNREADCONFIG_SHIFT) /* Plus one clock cycle using EMCCLKDELAY */
#  define EMC_DYNREADCONFIG_PLUS2         (3 << EMC_DYNREADCONFIG_SHIFT) /* Plus two clock cycles using EMCCLKDELAY */

/* Precharge command period */

#define EMC_DYNRP_SHIFT                   (0)       /* Bits 0-3: Precharge command period */
#define EMC_DYNRP_MASK                    (15 << EMC_DYNRP_SHIFT)
#  define EMC_DYNRP(n)                    ((uint32_t)((n)-1) << EMC_DYNRP_SHIFT)

/* Active to precharge command period */

#define EMC_DYNRAS_SHIFT                  (0)       /* Bits 0-3: Active to precharge command period */
#define EMC_DYNRAS_MASK                   (15 << EMC_DYNRAS_SHIFT)
#  define EMC_DYNRAS(n)                   ((uint32_t)((n)-1) << EMC_DYNRAS_SHIFT)

/* Self-refresh exit time */

#define EMC_DYNSREX_SHIFT                 (0)       /* Bits 0-3: Self-refresh exit time */
#define EMC_DYNSREX_MASK                  (15 << EMC_DYNSREX_SHIFT)
#  define EMC_DYNSREX(n)                  ((uint32_t)((n)-1) << EMC_DYNSREX_SHIFT)

/* Last-data-out to active command time */

#define EMC_DYNAPR_SHIFT                  (0)       /* Bits 0-3: Self-refresh exit time */
#define EMC_DYNAPR_MASK                   (15 << EMC_DYNAPR_SHIFT)
#  define EMC_DYNAPR(n)                   ((uint32_t)((n)-1) << EMC_DYNAPR_SHIFT)

/* Data-in to active command time */

#define EMC_DYNDAL_SHIFT                  (0)       /* Bits 0-3: Data-in to active command */
#define EMC_DYNDAL_MASK                   (15 << EMC_DYNDAL_SHIFT)
#  define EMC_DYNDAL(n)                   ((uint32_t)(n) << EMC_DYNDAL_SHIFT)

/* Write recovery time */

#define EMC_DYNWR_SHIFT                   (0)       /* Bits 0-3: Data-in to active command */
#define EMC_DYNWR_MASK                    (15 << EMC_DYNWR_SHIFT)
#  define EMC_DYNWR(n)                    ((uint32_t)((n)-1) << EMC_DYNWR_SHIFT)

/* Selects the active to active command period */

#define EMC_DYNRC_SHIFT                   (0)       /* Bits 0-4: Data-in to active command */
#define EMC_DYNRC_MASK                    (31 << EMC_DYNRC_SHIFT)
#  define EMC_DYNRC(n)                    ((uint32_t)((n)-1) << EMC_DYNRC_SHIFT)

/* Selects the auto-refresh period */

#define EMC_DYNRFC_SHIFT                  (0)       /* Bits 0-4: Auto-refresh period and auto-refresh to active command period */
#define EMC_DYNRFC_MASK                   (31 << EMC_DYNRFC_SHIFT)
#  define EMC_DYNRFC(n)                   ((uint32_t)((n)-1) << EMC_DYNRFC_SHIFT)

/* Time for exit self-refresh to active command */

#define EMC_DYNXSR_SHIFT                  (0)       /* Bits 0-4: Exit self-refresh to active command time */
#define EMC_DYNXSR_MASK                   (31 << EMC_DYNXSR_SHIFT)
#  define EMC_DYNXSR(n)                   ((uint32_t)((n)-1) << EMC_DYNXSR_SHIFT)

/* Latency for active bank A to active bank B */

#define EMC_DYNRRD_SHIFT                  (0)       /* Bits 0-3: Active bank A to active bank B latency */
#define EMC_DYNRRD_MASK                   (15 << EMC_DYNRRD_SHIFT)
#  define EMC_DYNRRD(n)                   ((uint32_t)((n)-1) << EMC_DYNRRD_SHIFT)

/* Time for load mode register to active command */

#define EMC_DYNMRD_SHIFT                  (0)       /* Bits 0-3: Load mode register to active command time */
#define EMC_DYNMRD_MASK                   (15 << EMC_DYNMRD_SHIFT)
#  define EMC_DYNMRD(n)                   ((uint32_t)((n)-1) << EMC_DYNMRD_SHIFT)

/* Time for long static memory read and write transfers */

#define EMC_STATEXTWAIT_SHIFT             (0)       /* Bits 0-9: Extended wait time out */
#define EMC_STATEXTWAIT_MASK              (0x3ff << EMC_STATEXTWAIT_SHIFT)
#  define EMC_STATEXTWAIT(n)              ((uint32_t)((n)-1) << EMC_STATEXTWAIT_SHIFT)

/* Per-chip select dynamic memory registers */
/* Dynamic Memory Configuration registers */
#define EMC_DYNCONFIG_
#define EMC_DYNCONFIG_MD_SHIFT            (3)       /* Bits 3-4: Memory device */
#define EMC_DYNCONFIG_MD_MASK             (3 << EMC_DYNCONFIG_MD_SHIFT)
#  define EMC_DYNCONFIG_MD(n)             ((uint32_t)(n) << EMC_DYNCONFIG_MD_SHIFT)
#  define EMC_DYNCONFIG_MD_SDRAM          (0 << EMC_DYNCONFIG_MD_SHIFT) /* SDRAM */
#  define EMC_DYNCONFIG_MD_LPDRAM         (1 << EMC_DYNCONFIG_MD_SHIFT) /* Low-power SDRAM */
#define EMC_DYNCONFIG_AM0_SHIFT           (7)       /* Bits 7-12: See Table 656 in User Manual */
#define EMC_DYNCONFIG_AM0_MASK            (0x3f << EMC_DYNCONFIG_AM0_SHIFT)
#  define EMC_DYNCONFIG_AM0(n)            ((uint32_t)(n) << EMC_DYNCONFIG_AM0_SHIFT)
#define EMC_DYNCONFIG_AM1                 (1 << 14) /* Bit 14: See Table 656 in User Manual */
#define EMC_DYNCONFIG_B                   (1 << 19) /* Bit 19: Buffer enable */
#define EMC_DYNCONFIG_P                   (1 << 20) /* Bit 20: Write protect */

#define EMC_DYNCONFIG_ADDRMAP_SHIFT       EMC_DYNCONFIG_AM0_SHIFT
#define EMC_DYNCONFIG_ADDRMAP_MASK        (EMC_DYNCONFIG_AM0_MASK | EMC_DYNCONFIG_AM1)
#  define EMC_DYNCONFIG_ADDRMAP(n)        ((uint32_t)(n) << EMC_DYNCONFIG_ADDRMAP_SHIFT)

/* Dynamic Memory RAS and CAS Delay registers */

#define EMC_DYNRASCAS_RAS_SHIFT           (0)       /* Bits 0-1: RAS latency */
#define EMC_DYNRASCAS_RAS_MASK            (3 << EMC_DYNRASCAS_RAS_SHIFT)
#  define EMC_DYNRASCAS_RAS(n)            ((uint32_t)(n) << EMC_DYNRASCAS_RAS_SHIFT)
#define EMC_DYNRASCAS_CAS_SHIFT           (8)       /* Bits 8-9: CAS latency */
#define EMC_DYNRASCAS_CAS_MASK            (3 << EMC_DYNRASCAS_CAS_SHIFT)
#  define EMC_DYNRASCAS_CAS(n)            ((uint32_t)(n) << EMC_DYNRASCAS_CAS_SHIFT)

/* Per-chip select static memory registers */
/* Static Memory Configuration registers */

#define EMC_STATCONFIG_MW_SHIFT           (0)       /* Bits 0-1: Memory width */
#define EMC_STATCONFIG_MW_MASK            (3 << EMC_STATCONFIG_MW_SHIFT)
#  define EMC_STATCONFIG_MW_8BIT          (0 << EMC_STATCONFIG_MW_SHIFT) /* 8 bit */
#  define EMC_STATCONFIG_MW_16BIT         (1 << EMC_STATCONFIG_MW_SHIFT) /* 16 bit */
#  define EMC_STATCONFIG_MW_32BIT         (2 << EMC_STATCONFIG_MW_SHIFT) /* 32 bit */
#define EMC_STATCONFIG_PM                 (1 << 3)  /* Bit 3:  Page mode */
#define EMC_STATCONFIG_PC                 (1 << 6)  /* Bit 6:  Chip select polarity */
#define EMC_STATCONFIG_PB                 (1 << 7)  /* Bit 7:  Byte lane state */
#define EMC_STATCONFIG_EW                 (1 << 8)  /* Bit 8:  Extended wait */
#define EMC_STATCONFIG_B                  (1 << 19) /* Bit 19: Buffer enable */
#define EMC_STATCONFIG_P                  (1 << 20) /* Bit 20: Write protect */

/* Static Memory Write Enable Delay registers */

#define EMC_STATWAITWEN_SHIFT             (0)       /* Bits 0-3: Wait write enable */
#define EMC_STATWAITWEN_MASK              (15 << EMC_STATWAITWEN_SHIFT)
#  define EMC_STATWAITWEN(n)              ((uint32_t)((n)-1) << EMC_STATWAITWEN_SHIFT)

/* Static Memory Output Enable delay registers */

#define EMC_STATWAITOEN_SHIFT             (0)       /* Bits 0-3: Wait output enable */
#define EMC_STATWAITOEN_MASK              (15 << EMC_STATWAITOEN_SHIFT)
#  define EMC_STATWAITOEN_NONE            (0 << EMC_STATWAITOEN_SHIFT)
#  define EMC_STATWAITOEN(n)              ((uint32_t)(n) << EMC_STATWAITOEN_SHIFT)

/* Static Memory Read Delay registers */

#define EMC_STATWAITRD_SHIFT              (0)       /* Bits 0-4: Non-page mode read wait states */
#define EMC_STATWAITRD_MASK               (31 << EMC_STATWAITRD_SHIFT)
#  define EMC_STATWAITRD(n)               ((uint32_t)((n)-1) << EMC_STATWAITRD_SHIFT)

/* Static Memory Page Mode Read Delay registers */

#define EMC_STATWAITPAGE_SHIFT            (0)       /* Bits 0-4: Page mode erad wait states */
#define EMC_STATWAITPAGE_MASK             (31 << EMC_STATWAITPAGE_SHIFT)
#  define EMC_STATWAITPAGE(n)             ((uint32_t)((n)-1) << EMC_STATWAITPAGE_SHIFT)

/* Static Memory Write Delay registers */

#define EMC_STATWAITWR_SHIFT              (0)       /* Bits 0-4: Write wait states */
#define EMC_STATWAITWR_MASK               (31 << EMC_STATWAITWR_SHIFT)
#  define EMC_STATWAITWR(n)               ((uint32_t)((n)-2) << EMC_STATWAITWR_SHIFT)

/* Static Memory Turn-around Delay registers */

#define EMC_STATWAITTURN_SHIFT            (0)       /* Bits 0-3: Bus turn-around cycles */
#define EMC_STATWAITTURN_MASK             (15 << EMC_STATWAITTURN_SHIFT)
#  define EMC_STATWAITTURN(n)             ((uint32_t)((n)-1) << EMC_STATWAITTURN_SHIFT)

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_EMC_H */
