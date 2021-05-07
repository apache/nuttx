/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/hardware/lpc17_40_emc.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_EMC_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_EMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/lpc17_40_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets */

#define LPC17_40_EMC_CONTROL_OFFSET             0x0000 /* EMC Control register */
#define LPC17_40_EMC_STATUS_OFFSET              0x0004 /* EMC Status register */
#define LPC17_40_EMC_CONFIG_OFFSET              0x0008 /* EMC Configuration register */
#define LPC17_40_EMC_DYNAMICCONTROL_OFFSET      0x0020 /* Dynamic Memory Control register */
#define LPC17_40_EMC_DYNAMICREFRESH_OFFSET      0x0024 /* Dynamic Memory Refresh Timer register */
#define LPC17_40_EMC_DYNAMICREADCONFIG_OFFSET   0x0028 /* Dynamic Memory Read Configuration register */
#define LPC17_40_EMC_DYNAMICRP_OFFSET           0x0030 /* Dynamic Memory Precharge Command Period register */
#define LPC17_40_EMC_DYNAMICRAS_OFFSET          0x0034 /* Dynamic Memory Active to Precharge Command Period register */
#define LPC17_40_EMC_DYNAMICSREX_OFFSET         0x0038 /* Dynamic Memory Self-refresh Exit Time register */
#define LPC17_40_EMC_DYNAMICAPR_OFFSET          0x003c /* Dynamic Memory Last Data Out to Active Time register */
#define LPC17_40_EMC_DYNAMICDAL_OFFSET          0x0040 /* Dynamic Memory Data-in to Active Command Time register */
#define LPC17_40_EMC_DYNAMICWR_OFFSET           0x0044 /* Dynamic Memory Write Recovery Time register */
#define LPC17_40_EMC_DYNAMICRC_OFFSET           0x0048 /* Dynamic Memory Active to Active Command Period register */
#define LPC17_40_EMC_DYNAMICRFC_OFFSET          0x004c /* Dynamic Memory Auto-refresh Period register */
#define LPC17_40_EMC_DYNAMICXSR_OFFSET          0x0050 /* Dynamic Memory Exit Self-refresh register */
#define LPC17_40_EMC_DYNAMICRRD_OFFSET          0x0054 /* Dynamic Memory Active Bank A to Active Bank B Time register */
#define LPC17_40_EMC_DYNAMICMRD_OFFSET          0x0058 /* Dynamic Memory Load Mode register to Active Command Time */
#define LPC17_40_EMC_STATICEXTENDEDWAIT_OFFSET  0x0080 /* Static Memory Extended Wait register */

#define LPC17_40_EMC_DYNAMICCONFIG0_OFFSET      0x0100 /* Dynamic Memory Configuration register 0 */
#define LPC17_40_EMC_DYNAMICRASCAS0_OFFSET      0x0104 /* Dynamic Memory RAS & CAS Delay register 0 */

#define LPC17_40_EMC_DYNAMICCONFIG1_OFFSET      0x0120 /* Dynamic Memory Configuration register 1 */
#define LPC17_40_EMC_DYNAMICRASCAS1_OFFSET      0x0124 /* Dynamic Memory RAS & CAS Delay register 1 */

#define LPC17_40_EMC_DYNAMICCONFIG2_OFFSET      0x0140 /* Dynamic Memory Configuration register 2 */
#define LPC17_40_EMC_DYNAMICRASCAS2_OFFSET      0x0144 /* Dynamic Memory RAS & CAS Delay register 2 */

#define LPC17_40_EMC_DYNAMICCONFIG3_OFFSET      0x0160 /* Dynamic Memory Configuration register 3 */
#define LPC17_40_EMC_DYNAMICRASCAS3_OFFSET      0x0164 /* Dynamic Memory RAS & CAS Delay register 3 */

#define LPC17_40_EMC_STATICCONFIG0_OFFSET       0x0200 /* Static Memory Configuration register 0 */
#define LPC17_40_EMC_STATICWAITWEN0_OFFSET      0x0204 /* Static Memory Write Enable Delay register 0 */
#define LPC17_40_EMC_STATICWAITOEN0_OFFSET      0x0208 /* Static Memory Output Enable Delay registers 0 */
#define LPC17_40_EMC_STATICWAITRD0_OFFSET       0x020c /* Static Memory Read Delay register 0 */
#define LPC17_40_EMC_STATICWAITPAGE0_OFFSET     0x0210 /* Static Memory Page Mode Read Delay register 0*/
#define LPC17_40_EMC_STATICWAITWR0_OFFSET       0x0214 /* Static Memory Write Delay register 0 */
#define LPC17_40_EMC_STATICWAITTURN0_OFFSET     0x0218 /* Static Memory Turn Round Delay register 0 */

#define LPC17_40_EMC_STATICCONFIG1_OFFSET       0x0220 /* Static Memory Configuration register 1 */
#define LPC17_40_EMC_STATICWAITWEN1_OFFSET      0x0224 /* Static Memory Write Enable Delay register 1 */
#define LPC17_40_EMC_STATICWAITOEN1_OFFSET      0x0228 /* Static Memory Output Enable Delay register 1 */
#define LPC17_40_EMC_STATICWAITRD1_OFFSET       0x022c /* Static Memory Read Delay register 1 */
#define LPC17_40_EMC_STATICWAITPAGE1_OFFSET     0x0230 /* Static Memory Page Mode Read Delay register 1 */
#define LPC17_40_EMC_STATICWAITWR1_OFFSET       0x0234 /* Static Memory Write Delay register 1 */
#define LPC17_40_EMC_STATICWAITTURN1_OFFSET     0x0238 /* Static Memory Turn Round Delay register 1 */

#define LPC17_40_EMC_STATICCONFIG2_OFFSET       0x0240 /* Static Memory Configuration register 2 */
#define LPC17_40_EMC_STATICWAITWEN2_OFFSET      0x0244 /* Static Memory Write Enable Delay register 2 */
#define LPC17_40_EMC_STATICWAITOEN2_OFFSET      0x0248 /* Static Memory Output Enable Delay register 2 */
#define LPC17_40_EMC_STATICWAITRD2_OFFSET       0x024c /* Static Memory Read Delay register 2 */
#define LPC17_40_EMC_STATICWAITPAGE2_OFFSET     0x0250 /* Static Memory Page Mode Read Delay registers 3 */
#define LPC17_40_EMC_STATICWAITWR2_OFFSET       0x0254 /* Static Memory Write Delay register 2 */
#define LPC17_40_EMC_EMCSTATICWAITTURN2_OFFSET  0x0258 /* Static Memory Turn Round Delay register 2 */

#define LPC17_40_EMC_STATICCONFIG3_OFFSET       0x0260 /* Static Memory Configuration register 3 */
#define LPC17_40_EMC_STATICWAITWEN3_OFFSET      0x0264 /* Static Memory Write Enable Delay register 3 */
#define LPC17_40_EMC_STATICWAITOEN3_OFFSET      0x0268 /* Static Memory Output Enable Delay register 3 */
#define LPC17_40_EMC_STATICWAITRD3_OFFSET       0x026c /* Static Memory Read Delay register 3 */
#define LPC17_40_EMC_STATICWAITPAGE3_OFFSET     0x0270 /* Static Memory Page Mode Read Delay register 4 */
#define LPC17_40_EMC_STATICWAITWR3_OFFSET       0x0274 /* Static Memory Write Delay register 3 */
#define LPC17_40_EMC_STATICWAITTURN3_OFFSET     0x0278 /* Static Memory Turn Round Delay register 3 */

/* Register Addresses */

#define LPC17_40_EMC_CONTROL                    (LPC17_40_EMC_BASE+LPC17_40_EMC_CONTROL_OFFSET)
#define LPC17_40_EMC_STATUS                     (LPC17_40_EMC_BASE+LPC17_40_EMC_STATUS_OFFSET)
#define LPC17_40_EMC_CONFIG                     (LPC17_40_EMC_BASE+LPC17_40_EMC_CONFIG_OFFSET)
#define LPC17_40_EMC_DYNAMICCONTROL             (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICCONTROL_OFFSET)
#define LPC17_40_EMC_DYNAMICREFRESH             (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICREFRESH_OFFSET)
#define LPC17_40_EMC_DYNAMICREADCONFIG          (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICREADCONFIG_OFFSET)
#define LPC17_40_EMC_DYNAMICRP                  (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICRP_OFFSET)
#define LPC17_40_EMC_DYNAMICRAS                 (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICRAS_OFFSET)
#define LPC17_40_EMC_DYNAMICSREX                (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICSREX_OFFSET)
#define LPC17_40_EMC_DYNAMICAPR                 (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICAPR_OFFSET)
#define LPC17_40_EMC_DYNAMICDAL                 (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICDAL_OFFSET)
#define LPC17_40_EMC_DYNAMICWR                  (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICWR_OFFSET)
#define LPC17_40_EMC_DYNAMICRC                  (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICRC_OFFSET)
#define LPC17_40_EMC_DYNAMICRFC                 (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICRFC_OFFSET)
#define LPC17_40_EMC_DYNAMICXSR                 (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICXSR_OFFSET)
#define LPC17_40_EMC_DYNAMICRRD                 (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICRRD_OFFSET)
#define LPC17_40_EMC_DYNAMICMRD                 (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICMRD_OFFSET)
#define LPC17_40_EMC_STATICEXTENDEDWAIT         (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICEXTENDEDWAIT_OFFSET)

#define LPC17_40_EMC_DYNAMICCONFIG0             (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICCONFIG0_OFFSET)
#define LPC17_40_EMC_DYNAMICRASCAS0             (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICRASCAS0_OFFSET)

#define LPC17_40_EMC_DYNAMICCONFIG1             (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICCONFIG1_OFFSET)
#define LPC17_40_EMC_DYNAMICRASCAS1             (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICRASCAS1_OFFSET)

#define LPC17_40_EMC_DYNAMICCONFIG2             (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICCONFIG2_OFFSET)
#define LPC17_40_EMC_DYNAMICRASCAS2             (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICRASCAS2_OFFSET)

#define LPC17_40_EMC_DYNAMICCONFIG3             (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICCONFIG3_OFFSET)
#define LPC17_40_EMC_DYNAMICRASCAS3             (LPC17_40_EMC_BASE+LPC17_40_EMC_DYNAMICRASCAS3_OFFSET)

#define LPC17_40_EMC_STATICCONFIG0              (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICCONFIG0_OFFSET)
#define LPC17_40_EMC_STATICWAITWEN0             (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITWEN0_OFFSET)
#define LPC17_40_EMC_STATICWAITOEN0             (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITOEN0_OFFSET)
#define LPC17_40_EMC_STATICWAITRD0              (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITRD0_OFFSET)
#define LPC17_40_EMC_STATICWAITPAGE0            (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITPAGE0_OFFSET)
#define LPC17_40_EMC_STATICWAITWR0              (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITWR0_OFFSET)
#define LPC17_40_EMC_STATICWAITTURN0            (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITTURN0_OFFSET)

#define LPC17_40_EMC_STATICCONFIG1              (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICCONFIG1_OFFSET)
#define LPC17_40_EMC_STATICWAITWEN1             (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITWEN1_OFFSET)
#define LPC17_40_EMC_STATICWAITOEN1             (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITOEN1_OFFSET)
#define LPC17_40_EMC_STATICWAITRD               (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITRD1_OFFSET)
#define LPC17_40_EMC_STATICWAITPAGE1            (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITPAGE1_OFFSET)
#define LPC17_40_EMC_STATICWAITWR1              (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITWR1_OFFSET)
#define LPC17_40_EMC_STATICWAITTURN1            (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITTURN1_OFFSET)

#define LPC17_40_EMC_STATICCONFIG2              (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICCONFIG2_OFFSET)
#define LPC17_40_EMC_STATICWAITWEN2             (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITWEN2_OFFSET)
#define LPC17_40_EMC_STATICWAITOEN2             (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITOEN2_OFFSET)
#define LPC17_40_EMC_STATICWAITRD2              (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITRD2_OFFSET)
#define LPC17_40_EMC_STATICWAITPAGE2            (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITPAGE2_OFFSET)
#define LPC17_40_EMC_STATICWAITWR2              (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITWR2_OFFSET)
#define LPC17_40_EMC_EMCSTATICWAITTURN2         (LPC17_40_EMC_BASE+LPC17_40_EMC_EMCSTATICWAITTURN2_OFFSET)

#define LPC17_40_EMC_STATICCONFIG3              (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICCONFIG3_OFFSET)
#define LPC17_40_EMC_STATICWAITWEN3             (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITWEN3_OFFSET)
#define LPC17_40_EMC_STATICWAITOEN3             (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITOEN3_OFFSET)
#define LPC17_40_EMC_STATICWAITRD3              (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITRD3_OFFSET)
#define LPC17_40_EMC_STATICWAITPAGE3            (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITPAGE3_OFFSET)
#define LPC17_40_EMC_STATICWAITWR3              (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITWR3_OFFSET)
#define LPC17_40_EMC_STATICWAITTURN3            (LPC17_40_EMC_BASE+LPC17_40_EMC_STATICWAITTURN3_OFFSET)

/* Register Bitfield Definitions */

/* EMC Control register */

#define EMC_CONTROL_E                           (1 << 0)  /* Bit 0:  EMC Enable */
#define EMC_CONTROL_M                           (1 << 1)  /* Bit 1:  Address mirror */
#define EMC_CONTROL_L                           (1 << 2)  /* Bit 2:  Low-power mode */

/* EMC Status register */

#define EMC_STATUS_B                            (1 << 0)  /* Bit 0:  Busy */
#define EMC_STATUS_S                            (1 << 1)  /* Bit 1:  Write buffer status */
#define EMC_STATUS_SA                           (1 << 2)  /* Bit 2:  Self-refresh acknowledge */

/* EMC Configuration register */

#define EMC_CONFIG_EM                           (1 << 0)  /* Bit 0:  Endian mode */
#define EMC_CONFIG_CLKR                         (1 << 8)  /* Bit 8:  CCLK:CLKOUT ratio */

/* Dynamic Memory Control register */

#define EMC_DYNAMICCONTROL_CE                   (1 << 0)  /* Bit 0:  Dynamic memory clock enable */
#define EMC_DYNAMICCONTROL_CS                   (1 << 1)  /* Bit 1:  Dynamic memory clock control */
#define EMC_DYNAMICCONTROL_SR                   (1 << 2)  /* Bit 2:  Self-refresh request */
#define EMC_DYNAMICCONTROL_MMC                  (1 << 5)  /* Bit 5:  Memory clock control */
#define EMC_DYNAMICCONTROL_I_SHIFT              (7)       /* Bits 7-8: SDRAM initialization */
#define EMC_DYNAMICCONTROL_I_MASK               (3 << EMC_DYNAMICCONTROL_I_SHIFT)
#  define EMC_DYNAMICCONTROL_I_NORMAL           (0 << EMC_DYNAMICCONTROL_I_SHIFT) /* SDRAM NORMAL operation command */
#  define EMC_DYNAMICCONTROL_I_MODE             (1 << EMC_DYNAMICCONTROL_I_SHIFT) /* SDRAM MODE command */
#  define EMC_DYNAMICCONTROL_I_PALL             (2 << EMC_DYNAMICCONTROL_I_SHIFT) /* SDRAM PALL (precharge all) command */
#  define EMC_DYNAMICCONTROL_I_NOP              (3 << EMC_DYNAMICCONTROL_I_SHIFT) /* SDRAM NOP (no operation) command) */

/* Dynamic Memory Refresh Timer register */

#define EMC_DYNAMICREFRESH_MASK                 (0x000007ff) /* Bits 0-10: REFRESH Refresh timer */

/* Dynamic Memory Read Configuration register */

#define EMC_DYNAMICREADCONFIG_RD_SHIFT          (0)          /* Bits 0-1: Read data strategy */
#define EMC_DYNAMICREADCONFIG_RD_MASK           (3 << EMC_DYNAMICREADCONFIG_RD_SHIFT)
#  define EMC_DYNAMICREADCONFIG_RD_CLKOUT       (0 << EMC_DYNAMICREADCONFIG_RD_SHIFT) /* Clock out delayed strategy */
#  define EMC_DYNAMICREADCONFIG_RD_CMD          (1 << EMC_DYNAMICREADCONFIG_RD_SHIFT) /* Command delayed strategy */
#  define EMC_DYNAMICREADCONFIG_RD_CMD1         (2 << EMC_DYNAMICREADCONFIG_RD_SHIFT) /* Command delayed strategy + 1 cycle */
#  define EMC_DYNAMICREADCONFIG_RD_CMD2         (3 << EMC_DYNAMICREADCONFIG_RD_SHIFT) /* Command delayed strategy + 2 cycles */

/* Dynamic Memory Precharge Command Period register */

#define EMC_DYNAMICRP_TRP_MASK                  (0x0000000f) /* Bits 0-3: Precharge command period */

/* Dynamic Memory Active to Precharge Command Period register */

#define EMC_DYNAMICRAS_TRAS_MASK                (0x0000000f) /* Bits 0-3: Active to precharge command period */

/* Dynamic Memory Self-refresh Exit Time register */

#define EMC_DYNAMICSREX_TSREX_MASK              (0x0000000f) /* Bits 0-3: Self-refresh exit time */

/* Dynamic Memory Last Data Out to Active Time register */

#define EMC_DYNAMICAPR_TAPR_MASK                (0x0000000f) /* Bits 0-3: Last-data-out to active command time */

/* Dynamic Memory Data-in to Active Command Time register */

#define EMC_DYNAMICDAL_TDAL_MASK                (0x0000000f) /* Bits 0-3: Data-in to active command */

/* Dynamic Memory Write Recovery Time register */

#define EMC_DYNAMICWR_TWR_MASK                  (0x0000000f) /* Bits 0-3: Write recovery time */

/* Dynamic Memory Active to Active Command Period register */

#define EMC_DYNAMICRC_TRC_MASK                  (0x0000001f) /* Bits 0-4: Active to active command period */

/* Dynamic Memory Auto-refresh Period register */

#define EMC_DYNAMICRFC_TRFC_MASK                (0x0000001f) /* Bits 0-4: Auto-refresh period and auto-refresh to active command period */

/* Dynamic Memory Exit Self-refresh register */

#define EMC_DYNAMICXSR_TXSR_MASK                (0x0000001f) /* Bits 0-4: Exit self-refresh to active command time */

/* Dynamic Memory Active Bank A to Active Bank B Time register */

#define EMC_DYNAMICRRD_TRRD_MASK                (0x0000000f) /* Bits 0-3: Active bank A to active bank B latency */

/* Dynamic Memory Load Mode register to Active Command Time */

#define EMC_DYNAMICMRD_TMRD_MASK                (0x0000000f) /* Bits 0-3: Load mode register to active command time */

/* Static Memory Extended Wait register */

#define EMC_STATICEXTENDEDWAIT_MASK             (0x000003ff) /* Bits 0-9: Extended wait time out */

/* Dynamic Memory Configuration registers (0-3) */

#define EMC_DYNAMICCONFIG_MD_SHIFT              (3)       /* Bits 3-4: Memory device */
#define EMC_DYNAMICCONFIG_MD_MASK               (3 << EMC_DYNAMICCONFIG_MD_SHIFT)
#  define EMC_DYNAMICCONFIG_MD_SDRAM            (0 << EMC_DYNAMICCONFIG_MD_SHIFT) /* SDRAM */
#  define EMC_DYNAMICCONFIG_MD_LOWPOWER         (1 << EMC_DYNAMICCONFIG_MD_SHIFT) /* Low-power SDRAM */

#define EMC_DYNAMICCONFIG_AM0_SHIFT             (7)       /* Bits 7-12: */
#define EMC_DYNAMICCONFIG_AM0_MASK              (63 << EMC_DYNAMICCONFIG_AM0_SHIFT)
#  define EMC_DYNAMICCONFIG_AM0(n)              ((n) << EMC_DYNAMICCONFIG_AM0_SHIFT)
#define EMC_DYNAMICCONFIG_AM1                   (1 << 14) /* Bit 14:  */
#define EMC_DYNAMICCONFIG_B                     (1 << 19) /* Bit 19:  Buffer enable */
#define EMC_DYNAMICCONFIG_P                     (1 << 20) /* Bit 20:  Write protect */

/* Dynamic Memory RAS & CAS Delay registers (0-3) */

#define EMC_DYNAMICRASCAS_RAS_SHIFT             (0)       /* Bits 0-1: RAS latency (active to read/write delay) */
#define EMC_DYNAMICRASCAS_RAS_MASK              (3 << EMC_DYNAMICRASCAS_RAS_SHIFT)
#  define EMC_DYNAMICRASCAS_RAS_1CCLK           (1 << EMC_DYNAMICRASCAS_RAS_SHIFT) /* One CCLK cycle */
#  define EMC_DYNAMICRASCAS_RAS_2CCLK           (2 << EMC_DYNAMICRASCAS_RAS_SHIFT) /* Two CCLK cycles */
#  define EMC_DYNAMICRASCAS_RAS_3CCLK           (3 << EMC_DYNAMICRASCAS_RAS_SHIFT) /* Three CCLK cycles */

#define EMC_DYNAMICRASCAS_CAS_SHIFT             (8)       /* Bits 8-9: CAS latency */
#define EMC_DYNAMICRASCAS_CAS_MASK              (3 << EMC_DYNAMICRASCAS_CAS_SHIFT)
#  define EMC_DYNAMICRASCAS_CAS_1CCLK           (1 << EMC_DYNAMICRASCAS_CAS_SHIFT) /* One CCLK cycle */
#  define EMC_DYNAMICRASCAS_CAS_2CCLK           (2 << EMC_DYNAMICRASCAS_CAS_SHIFT) /* Two CCLK cycles */
#  define EMC_DYNAMICRASCAS_CAS_3CCLK           (3 << EMC_DYNAMICRASCAS_CAS_SHIFT) /* Three CCLK cycles */

/* Static Memory Configuration registers (0-3) */

#define EMC_STATICCONFIG_MW_SHIFT               (0)       /* Bits 0-1: Memory width */
#define EMC_STATICCONFIG_MW_MASK                (3 << EMC_STATICCONFIG_MW_SHIFT)
#  define EMC_STATICCONFIG_MW_8BIT              (0 << EMC_STATICCONFIG_MW_SHIFT)
#  define EMC_STATICCONFIG_MW_16BIT             (1 << EMC_STATICCONFIG_MW_SHIFT)
#  define EMC_STATICCONFIG_MW_32BIT             (2 << EMC_STATICCONFIG_MW_SHIFT)
#define EMC_STATICCONFIG_PM                     (1 << 3)  /* Bit 3:  Page mode */
#define EMC_STATICCONFIG_PC                     (1 << 6)  /* Bit 6:  Chip select polarity */
#define EMC_STATICCONFIG_PB                     (1 << 7)  /* Bit 7:  Byte lane state */
#define EMC_STATICCONFIG_EW                     (1 << 8)  /* Bit 8:  Extended wait */
#define EMC_STATICCONFIG_B                      (1 << 19) /* Bit 19: Buffer enable */
#define EMC_STATICCONFIG_P                      (1 << 20) /* Bit 20: Write protect */

/* Static Memory Write Enable Delay registers (0-3) */

#define EMC_STATICWAITWEN_MASK                  (0x0000000f) /* Bits 0-3: Wait write enable */

/* Static Memory Output Enable Delay registers (0-3) */

#define EMC_STATICWAITOEN_MASK                  (0x0000000f) /* Bits 0-3: Wait output enable */

/* Static Memory Read Delay registers (0-3) */

#define EMC_STATICWAITRD_MASK                   (0x0000001f) /* Bits 0-4: Exit self-refresh to active command time */

/* Static Memory Page Mode Read Delay registers (0-3) */

#define EMC_STATICWAITPAGE_MASK                 (0x0000001f) /* Bits 0-4: Asynchronous page mode read after the first read wait states */

/* Static Memory Write Delay registers (0-3) */

#define EMC_STATICWAITWR_MASK                   (0x0000001f) /* Bits 0-4: Write wait states */

/* Static Memory Turn Round Delay registers (0-3) */

#define EMC_STATICWAITTURN_MASK                 (0x0000000f) /* Bits 0-3: Bus turn-around cycles */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_EMC_H */
