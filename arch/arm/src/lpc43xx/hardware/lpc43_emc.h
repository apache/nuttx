/****************************************************************************
 * arch/arm/src/lpc43xx/hardware/lpc43_emc.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_EMC_H
#define __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_EMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define LPC43_EMC_CONTROL_OFFSET          0x0000 /* EMC Control register */
#define LPC43_EMC_STATUS_OFFSET           0x0004 /* EMC Status register */
#define LPC43_EMC_CONFIG_OFFSET           0x0008 /* EMC Configuration register */
#define LPC43_EMC_DYNCONTROL_OFFSET       0x0020 /* Dynamic Memory Control register */
#define LPC43_EMC_DYNREFRESH_OFFSET       0x0024 /* Dynamic Memory Refresh Timer register */
#define LPC43_EMC_DYNREADCONFIG_OFFSET    0x0028 /* Dynamic Memory Read Configuration register */
#define LPC43_EMC_DYNRP_OFFSET            0x0030 /* Dynamic Memory Precharge Command Period register */
#define LPC43_EMC_DYNRAS_OFFSET           0x0034 /* Dynamic Memory Active to Precharge Command Period register */
#define LPC43_EMC_DYNSREX_OFFSET          0x0038 /* Dynamic Memory Self Refresh Exit Time register */
#define LPC43_EMC_DYNAPR_OFFSET           0x003c /* Dynamic Memory Last Data Out to Active Time register */
#define LPC43_EMC_DYNDAL_OFFSET           0x0040 /* Dynamic Memory Data In to Active Command Time register */
#define LPC43_EMC_DYNWR_OFFSET            0x0044 /* Dynamic Memory Write Recovery Time register */
#define LPC43_EMC_DYNRC_OFFSET            0x0048 /* Dynamic Memory Active to Active Command Period register */
#define LPC43_EMC_DYNRFC_OFFSET           0x004c /* Dynamic Memory Auto-refresh Period register */
#define LPC43_EMC_DYNXSR_OFFSET           0x0050 /* Dynamic Memory Exit Self Refresh register */
#define LPC43_EMC_DYNRRD_OFFSET           0x0054 /* Dynamic Memory Active Bank A to Active Bank B Time register */
#define LPC43_EMC_DYNMRD_OFFSET           0x0058 /* Dynamic Memory Load Mode register to Active Command Time */
#define LPC43_EMC_STATEXTWAIT_OFFSET      0x0080 /* Static Memory Extended Wait register */

#define LPC43_EMC_DYNCONFIG_CSOFFSET      0x0000
#define LPC43_EMC_DYNRASCAS_CSOFFSET      0x0004
#define LPC43_EMC_DYNCS_OFFSET(n)         (0x100 + ((n) << 5))
#define LPC43_EMC_DYNCONFIG_OFFSET(n)     (DYNCS_OFFSET(n)+DYNCONFIG_CSOFFSET)
#define LPC43_EMC_DYNRASCAS_OFFSET(n)     (DYNCS_OFFSET(n)+DYNRASCAS_CSOFFSET)

#define LPC43_EMC_DYNCONFIG0_OFFSET       0x0100 /* Dynamic Memory Configuration register CS0 */
#define LPC43_EMC_DYNRASCAS0_OFFSET       0x0104 /* Dynamic Memory RAS & CAS Delay register CS0 */
#define LPC43_EMC_DYNCONFIG1_OFFSET       0x0120 /* Dynamic Memory Configuration register CS1 */
#define LPC43_EMC_DYNRASCAS1_OFFSET       0x0124 /* Dynamic Memory RAS & CAS Delay register CS1 */
#define LPC43_EMC_DYNCONFIG2_OFFSET       0x0140 /* Dynamic Memory Configuration register CS2 */
#define LPC43_EMC_DYNRASCAS2_OFFSET       0x0144 /* Dynamic Memory RAS & CAS Delay register CS2 */
#define LPC43_EMC_DYNCONFIG3_OFFSET       0x0160 /* Dynamic Memory Configuration register CS3 */
#define LPC43_EMC_DYNRASCAS3_OFFSET       0x0164 /* ynamic Memory RAS & CAS Delay register CS3 */

#define LPC43_EMC_STATCONFIG_CSOFFSET     0x0000 /* Static Memory Configuration register */
#define LPC43_EMC_STATWAITWEN_CSOFFSET    0x0004 /* Static Memory Write Enable Delay register */
#define LPC43_EMC_STATWAITOEN_CSOFFSET    0x0008 /* Static Memory Output Enable Delay register */
#define LPC43_EMC_STATWAITRD_CSOFFSET     0x000c /* Static Memory Read Delay register */
#define LPC43_EMC_STATWAITPAGE_CSOFFSET   0x0010 /* Static Memory Write Delay registers */
#define LPC43_EMC_STATWAITWR_CSOFFSET     0x0014 /* Static Memory Page Mode Read Delay register  */
#define LPC43_EMC_STATWAITTURN_CSOFFSET   0x0018 /* Static Memory Turn Round Delay register */
#define LPC43_EMC_STATCS_OFFSET(n)        (0x0200 + ((n) << 5))
#define LPC43_EMC_STATCONFIG_OFFSET(n)    (LPC43_EMC_STATCS_OFFSET(n)+LPC43_EMC_STATCONFIG_CSOFFSET)
#define LPC43_EMC_STATWAITWEN_OFFSET(n)   (LPC43_EMC_STATCS_OFFSET(n)+LPC43_EMC_STATWAITWEN_CSOFFSET)
#define LPC43_EMC_STATWAITOEN_OFFSET(n)   (LPC43_EMC_STATCS_OFFSET(n)+LPC43_EMC_STATWAITOEN_CSOFFSET)
#define LPC43_EMC_STATWAITRD_OFFSET(n)    (LPC43_EMC_STATCS_OFFSET(n)+LPC43_EMC_STATWAITRD_CSOFFSET)
#define LPC43_EMC_STATWAITPAGE_OFFSET(n)  (LPC43_EMC_STATCS_OFFSET(n)+LPC43_EMC_STATWAITPAGE_CSOFFSET)
#define LPC43_EMC_STATWAITWR_OFFSET(n)    (LPC43_EMC_STATCS_OFFSET(n)+LPC43_EMC_STATWAITWR_CSOFFSET)
#define LPC43_EMC_STATWAITTURN_OFFSET(n)  (LPC43_EMC_STATCS_OFFSET(n)+LPC43_EMC_STATWAITTURN_CSOFFSET)

#define LPC43_EMC_STATCONFIG0_OFFSET      0x0200 /* Static Memory Configuration register CS0 */
#define LPC43_EMC_STATWAITWEN0_OFFSET     0x0204 /* Static Memory Write Enable Delay register CS0 */
#define LPC43_EMC_STATWAITOEN0_OFFSET     0x0208 /* Static Memory Output Enable Delay register CS0 */
#define LPC43_EMC_STATWAITRD0_OFFSET      0x020c /* Static Memory Read Delay register CS0 */
#define LPC43_EMC_STATWAITPAGE0_OFFSET    0x0210 /* Static Memory Page Mode Read Delay register CS0 */
#define LPC43_EMC_STATWAITWR0_OFFSET      0x0214 /* Static Memory Write Delay register CS0 */
#define LPC43_EMC_STATWAITTURN0_OFFSET    0x0218 /* Static Memory Turn Round Delay register CS0 */

#define LPC43_EMC_STATCONFIG1_OFFSET      0x0220 /* Static Memory Configuration register CS1 */
#define LPC43_EMC_STATWAITWEN1_OFFSET     0x0224 /* Static Memory Write Enable Delay register CS1 */
#define LPC43_EMC_STATWAITOEN1_OFFSET     0x0228 /* Static Memory Output Enable Delay register CS1 */
#define LPC43_EMC_STATWAITRD1_OFFSET      0x022c /* Static Memory Read Delay register CS1 */
#define LPC43_EMC_STATWAITPAGE1_OFFSET    0x0230 /* Static Memory Page Mode Read Delay register CS1 */
#define LPC43_EMC_STATWAITWR1_OFFSET      0x0234 /* Static Memory Write Delay registers CS1 */
#define LPC43_EMC_STATWAITTURN1_OFFSET    0x0238 /* Static Memory Turn Round Delay register CS1 */

#define LPC43_EMC_STATCONFIG2_OFFSET      0x0240 /* Static Memory Configuration register CS2 */
#define LPC43_EMC_STATWAITWEN2_OFFSET     0x0244 /* Static Memory Write Enable Delay register CS2 */
#define LPC43_EMC_STATWAITOEN2_OFFSET     0x0248 /* Static Memory Output Enable Delay register CS2 */
#define LPC43_EMC_STATWAITRD2_OFFSET      0x024c /* Static Memory Read Delay register CS2 */
#define LPC43_EMC_STATWAITPAGE2_OFFSET    0x0250 /* Static Memory Page Mode Read Delay register CS2 */
#define LPC43_EMC_STATWAITWR2_OFFSET      0x0254 /* Static Memory Write Delay registers CS2 */
#define LPC43_EMC_STATWAITTURN2_OFFSET    0x0258 /* Static Memory Turn Round Delay register CS2 */

#define LPC43_EMC_STATCONFIG3_OFFSET      0x0260 /* Static Memory Configuration register CS3 */
#define LPC43_EMC_STATWAITWEN3_OFFSET     0x0264 /* Static Memory Write Enable Delay register CS3 */
#define LPC43_EMC_STATWAITOEN3_OFFSET     0x0268 /* Static Memory Output Enable Delay register CS3 */
#define LPC43_EMC_STATWAITRD3_OFFSET      0x026c /* Static Memory Read Delay register CS3 */
#define LPC43_EMC_STATWAITPAGE3_OFFSET    0x0270 /* Static Memory Page Mode Read Delay register CS3 */
#define LPC43_EMC_STATWAITWR3_OFFSET      0x0274 /* Static Memory Write Delay registers CS3 */
#define LPC43_EMC_STATWAITTURN3_OFFSET    0x0278 /* Static Memory Turn Round Delay register CS3 */

/* Register Addresses *******************************************************/

#define LPC43_EMC_CONTROL                 (LPC43_EMC_BASE+LPC43_EMC_CONTROL_OFFSET)
#define LPC43_EMC_STATUS                  (LPC43_EMC_BASE+LPC43_EMC_STATUS_OFFSET)
#define LPC43_EMC_CONFIG                  (LPC43_EMC_BASE+LPC43_EMC_CONFIG_OFFSET)
#define LPC43_EMC_DYNCONTROL              (LPC43_EMC_BASE+LPC43_EMC_DYNCONTROL_OFFSET)
#define LPC43_EMC_DYNREFRESH              (LPC43_EMC_BASE+LPC43_EMC_DYNREFRESH_OFFSET)
#define LPC43_EMC_DYNREADCONFIG           (LPC43_EMC_BASE+LPC43_EMC_DYNREADCONFIG_OFFSET)
#define LPC43_EMC_DYNRP                   (LPC43_EMC_BASE+LPC43_EMC_DYNRP_OFFSET)
#define LPC43_EMC_DYNRAS                  (LPC43_EMC_BASE+LPC43_EMC_DYNRAS_OFFSET)
#define LPC43_EMC_DYNSREX                 (LPC43_EMC_BASE+LPC43_EMC_DYNSREX_OFFSET)
#define LPC43_EMC_DYNAPR                  (LPC43_EMC_BASE+LPC43_EMC_DYNAPR_OFFSET)
#define LPC43_EMC_DYNDAL                  (LPC43_EMC_BASE+LPC43_EMC_DYNDAL_OFFSET)
#define LPC43_EMC_DYNWR                   (LPC43_EMC_BASE+LPC43_EMC_DYNWR_OFFSET)
#define LPC43_EMC_DYNRC                   (LPC43_EMC_BASE+LPC43_EMC_DYNRC_OFFSET)
#define LPC43_EMC_DYNRFC                  (LPC43_EMC_BASE+LPC43_EMC_DYNRFC_OFFSET)
#define LPC43_EMC_DYNXSR                  (LPC43_EMC_BASE+LPC43_EMC_DYNXSR_OFFSET)
#define LPC43_EMC_DYNRRD                  (LPC43_EMC_BASE+LPC43_EMC_DYNRRD_OFFSET)
#define LPC43_EMC_DYNMRD                  (LPC43_EMC_BASE+LPC43_EMC_DYNMRD_OFFSET)
#define LPC43_EMC_STATEXTWAIT             (LPC43_EMC_BASE+LPC43_EMC_STATEXTWAIT_OFFSET)

#define LPC43_EMC_DYNCS(n)                (LPC43_EMC_BASE+LPC43_EMC_DYNCS_OFFSET(n))
#define LPC43_EMC_DYNCONFIG(n)            (LPC43_EMC_BASE+LPC43_EMC_DYNCONFIG_OFFSET(n))
#define LPC43_EMC_DYNRASCAS(n)            (LPC43_EMC_BASE+LPC43_EMC_DYNRASCAS_OFFSET(n))

#define LPC43_EMC_DYNCONFIG0              (LPC43_EMC_BASE+LPC43_EMC_DYNCONFIG0_OFFSET)
#define LPC43_EMC_DYNRASCAS0              (LPC43_EMC_BASE+LPC43_EMC_DYNRASCAS0_OFFSET)
#define LPC43_EMC_DYNCONFIG1              (LPC43_EMC_BASE+LPC43_EMC_DYNCONFIG1_OFFSET)
#define LPC43_EMC_DYNRASCAS1              (LPC43_EMC_BASE+LPC43_EMC_DYNRASCAS1_OFFSET)
#define LPC43_EMC_DYNCONFIG2              (LPC43_EMC_BASE+LPC43_EMC_DYNCONFIG2_OFFSET)
#define LPC43_EMC_DYNRASCAS2              (LPC43_EMC_BASE+LPC43_EMC_DYNRASCAS2_OFFSET)
#define LPC43_EMC_DYNCONFIG3              (LPC43_EMC_BASE+LPC43_EMC_DYNCONFIG3_OFFSET)
#define LPC43_EMC_DYNRASCAS3              (LPC43_EMC_BASE+LPC43_EMC_DYNRASCAS3_OFFSET)

#define LPC43_EMC_STATCS(n)               (LPC43_EMC_BASE+LPC43_EMC_STATCS_OFFSET(n))
#define LPC43_EMC_STATCONFIG(n)           (LPC43_EMC_BASE+LPC43_EMC_STATCONFIG_OFFSET(n))
#define LPC43_EMC_STATWAITWEN(n)          (LPC43_EMC_BASE+LPC43_EMC_STATWAITWEN_OFFSET(n))
#define LPC43_EMC_STATWAITOEN(n)          (LPC43_EMC_BASE+LPC43_EMC_STATWAITOEN_OFFSET(n))
#define LPC43_EMC_STATWAITRD(n)           (LPC43_EMC_BASE+LPC43_EMC_STATWAITRD_OFFSET(n))
#define LPC43_EMC_STATWAITPAGE(n)         (LPC43_EMC_BASE+LPC43_EMC_STATWAITPAGE_OFFSET(n))
#define LPC43_EMC_STATWAITWR(n)           (LPC43_EMC_BASE+LPC43_EMC_STATWAITWR_OFFSET(n))
#define LPC43_EMC_STATWAITTURN(n)         (LPC43_EMC_BASE+LPC43_EMC_STATWAITTURN_OFFSET(n))

#define LPC43_EMC_STATCONFIG0             (LPC43_EMC_BASE+LPC43_EMC_STATCONFIG0_OFFSET)
#define LPC43_EMC_STATWAITWEN0            (LPC43_EMC_BASE+LPC43_EMC_STATWAITWEN0_OFFSET)
#define LPC43_EMC_STATWAITOEN0            (LPC43_EMC_BASE+LPC43_EMC_STATWAITOEN0_OFFSET)
#define LPC43_EMC_STATWAITRD0             (LPC43_EMC_BASE+LPC43_EMC_STATWAITRD0_OFFSET)
#define LPC43_EMC_STATWAITPAGE0           (LPC43_EMC_BASE+LPC43_EMC_STATWAITPAGE0_OFFSET)
#define LPC43_EMC_STATWAITWR0             (LPC43_EMC_BASE+LPC43_EMC_STATWAITWR0_OFFSET)
#define LPC43_EMC_STATWAITTURN0           (LPC43_EMC_BASE+LPC43_EMC_STATWAITTURN0_OFFSET)

#define LPC43_EMC_STATCONFIG1             (LPC43_EMC_BASE+LPC43_EMC_STATCONFIG1_OFFSET)
#define LPC43_EMC_STATWAITWEN1            (LPC43_EMC_BASE+LPC43_EMC_STATWAITWEN1_OFFSET)
#define LPC43_EMC_STATWAITOEN1            (LPC43_EMC_BASE+LPC43_EMC_STATWAITOEN1_OFFSET)
#define LPC43_EMC_STATWAITRD1             (LPC43_EMC_BASE+LPC43_EMC_STATWAITRD1_OFFSET)
#define LPC43_EMC_STATWAITPAGE1           (LPC43_EMC_BASE+LPC43_EMC_STATWAITPAGE1_OFFSET)
#define LPC43_EMC_STATWAITWR1             (LPC43_EMC_BASE+LPC43_EMC_STATWAITWR1_OFFSET)
#define LPC43_EMC_STATWAITTURN1           (LPC43_EMC_BASE+LPC43_EMC_STATWAITTURN1_OFFSET)

#define LPC43_EMC_STATCONFIG2             (LPC43_EMC_BASE+LPC43_EMC_STATCONFIG2_OFFSET)
#define LPC43_EMC_STATWAITWEN2            (LPC43_EMC_BASE+LPC43_EMC_STATWAITWEN2_OFFSET)
#define LPC43_EMC_STATWAITOEN2            (LPC43_EMC_BASE+LPC43_EMC_STATWAITOEN2_OFFSET)
#define LPC43_EMC_STATWAITRD2             (LPC43_EMC_BASE+LPC43_EMC_STATWAITRD2_OFFSET)
#define LPC43_EMC_STATWAITPAGE2           (LPC43_EMC_BASE+LPC43_EMC_STATWAITPAGE2_OFFSET)
#define LPC43_EMC_STATWAITWR2             (LPC43_EMC_BASE+LPC43_EMC_STATWAITWR2_OFFSET)
#define LPC43_EMC_STATWAITTURN2           (LPC43_EMC_BASE+LPC43_EMC_STATWAITTURN2_OFFSET)

#define LPC43_EMC_STATCONFIG3             (LPC43_EMC_BASE+LPC43_EMC_STATCONFIG3_OFFSET)
#define LPC43_EMC_STATWAITWEN3            (LPC43_EMC_BASE+LPC43_EMC_STATWAITWEN3_OFFSET)
#define LPC43_EMC_STATWAITOEN3            (LPC43_EMC_BASE+LPC43_EMC_STATWAITOEN3_OFFSET)
#define LPC43_EMC_STATWAITRD3             (LPC43_EMC_BASE+LPC43_EMC_STATWAITRD3_OFFSET)
#define LPC43_EMC_STATWAITPAGE3           (LPC43_EMC_BASE+LPC43_EMC_STATWAITPAGE3_OFFSET)
#define LPC43_EMC_STATWAITWR3             (LPC43_EMC_BASE+LPC43_EMC_STATWAITWR3_OFFSET)
#define LPC43_EMC_STATWAITTURN3           (LPC43_EMC_BASE+LPC43_EMC_STATWAITTURN3_OFFSET)

/* Register Bit Definitions *************************************************/

/* EMC Control register */

#define EMC_CONTROL_ENA                   (1 << 0)  /* Bit 0:  EMC Enable */
#define EMC_CONTROL_ADDRMIRROR            (1 << 1)  /* Bit 1:  Address mirror */
#define EMC_CONTROL_LOWPOWER              (1 << 2)  /* Bit 2:  Low-power mode */
                                                    /* Bits 3-31: Reserved */

/* EMC Status register */

#define EMC__
#define EMC_STATUS_BUSY                   (1 << 0)  /* Bit 0:  Busy */
#define EMC_STATUS_WB                     (1 << 1)  /* Bit 1:  Write buffer status */
#define EMC_CONFIG_SA                     (1 << 2)  /* Bit 2:  Self-refresh acknowledge */
                                                    /* Bits 3-31: Reserved */

/* EMC Configuration register */

#define EMC_CONFIG_EM                     (1 << 0)  /* Bit 0:  Endian mode */
                                                    /* Bits 1-7: Reserved */
#define EMC_CONFIG_CR                     (1 << 8)  /* Bit 8:  Clock Ratio */
                                                    /* Bits 9-31: Reserved */

/* Dynamic Memory Control register */

#define EMC_DYNCONTROL_
#define EMC_DYNCONTROL_CE                 (1 << 0)  /* Bit 0:  Dynamic memory clock enable */
#define EMC_DYNCONTROL_CS                 (1 << 1)  /* Bit 1:  Dynamic memory clock control */
#define EMC_DYNCONTROL_SR                 (1 << 2)  /* Bit 2:  Self-refresh request, EMCSREFREQ */
                                                    /* Bits 3-4: Reserved */
#define EMC_DYNCONTROL_MMC                (1 << 5)  /* Bit 5:  Memory clock control */
                                                    /* Bit 6: Reserved */
#define EMC_DYNCONTROL_SI_SHIFT           (7)       /* Bits 7-8: SDRAM initialization */
#define EMC_DYNCONTROL_SI_MASK            (3 << EMC_DYNCONTROL_SI_SHIFT)
#  define EMC_DYNCONTROL_SI_NORMAL        (0 << EMC_DYNCONTROL_SI_SHIFT) /* SDRAM NORMAL command */
#  define EMC_DYNCONTROL_SI_MODE          (1 << EMC_DYNCONTROL_SI_SHIFT) /* SDRAM MODE command */
#  define EMC_DYNCONTROL_SI_PALL          (2 << EMC_DYNCONTROL_SI_SHIFT) /* SDRAM PALL (precharge all) command */
#  define EMC_DYNCONTROL_SI_ MASK         (3 << EMC_DYNCONTROL_SI_SHIFT) /* SDRAM NOP (no operation) command) */

                                                    /* Bits 9-31: Reserved */

/* Dynamic Memory Refresh Timer register */

#define EMC_DYNREFRESH_SHIFT              (0)       /* Bits 0-10: Refresh timer */
#define EMC_DYNREFRESH_MASK               (0x7ff << EMC_DYNREFRESH_SHIFT)

                                                    /* Bits 11-31: Reserved */

/* Dynamic Memory Read Configuration register */

#define EMC_DYNREADCONFIG_SHIFT           (0)       /* Bits 0-1: Read data strategy */
#define EMC_DYNREADCONFIG_MASK            (3 << EMC_DYNREADCONFIG_SHIFT)
#  define EMC_DYNREADCONFIG_0p5CCLK       (1 << EMC_DYNREADCONFIG_SHIFT) /* Command delayed by 0.5 CCLK */
#  define EMC_DYNREADCONFIG_1p5CCLK       (2 << EMC_DYNREADCONFIG_SHIFT) /* Command delayed by 1.5 CCLK */
#  define EMC_DYNREADCONFIG_2p5CCLK       (3 << EMC_DYNREADCONFIG_SHIFT) /* Command delayed by 2.5 CCLK */

                                                    /* Bits 2-31: Reserved */

/* Dynamic Memory Precharge Command Period register */

#define EMC_DYNRP_SHIFT                   (0)       /* Bits 0-3: Precharge command period */
#define EMC_DYNRP_MASK                    (15 << EMC_DYNRP_SHIFT)
#  define EMC_DYNRP(n)                    (((n)-1) << EMC_DYNRP_SHIFT) /* Delay n CCLK cycles */

                                                    /* Bits 2-31: Reserved */

/* Dynamic Memory Active to Precharge Command Period register */

#define EMC_DYNRAS_SHIFT                   (0)       /* Bits 0-3: Active to precharge command period */
#define EMC_DYNRAS_MASK                    (15 << EMC_DYNRAS_SHIFT)
#  define EMC_DYNRAS(n)                    (((n)-1) << EMC_DYNRAS_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 4-31: Reserved */

/* Dynamic Memory Self Refresh Exit Time register */

#define EMC_DYNSREX_SHIFT                  (0)       /* Bits 0-3: Self-refresh exit time */
#define EMC_DYNSREX_MASK                   (15 << EMC_DYNSREX_SHIFT)
#  define EMC_DYNSREX(n)                   (((n)-1) << EMC_DYNSREX_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 4-31: Reserved */

/* Dynamic Memory Last Data Out to Active Time register */

#define EMC_DYNAPR_SHIFT                   (0)       /* Bits 0-3: Last-data-out to active command time */
#define EMC_DYNAPR_MASK                    (15 << EMC_DYNAPR_SHIFT)
#  define EMC_DYNAPR(n)                    (((n)-1) << EMC_DYNAPR_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 4-31: Reserved */

/* Dynamic Memory Data In to Active Command Time register */

#define EMC_DYNDAL_SHIFT                   (0)       /* Bits 0-3: Data-in to active command */
#define EMC_DYNDAL_MASK                    (15 << EMC_DYNDAL_SHIFT)
#  define EMC_DYNDAL(n)                    (((n)-1) << EMC_DYNDAL_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 4-31: Reserved */

/* Dynamic Memory Write Recovery Time register */

#define EMC_DYNWR_SHIFT                    (0)       /* Bits 0-3: Write recovery time */
#define EMC_DYNWR_MASK                     (15 << EMC_DYNWR_SHIFT)
#  define EMC_DYNWR(n)                     (((n)-1) << EMC_DYNWR_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 4-31: Reserved */

/* Dynamic Memory Active to Active Command Period register */

#define EMC_DYNRC_SHIFT                    (0)       /* Bits 0-4: Active to active command period */
#define EMC_DYNRC_MASK                     (31 << EMC_DYNRC_SHIFT)
#  define EMC_DYNRC(n)                     (((n)-1) << EMC_DYNRC_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 5-31: Reserved */

/* Dynamic Memory Auto-refresh Period register */

#define EMC_DYNRFC_SHIFT                   (0)       /* Bits 0-4: Auto-refresh period and
                                                      * auto-refresh to active command period */
#define EMC_DYNRFC_MASK                    (31 << EMC_DYNRFC_SHIFT)
#  define EMC_DYNRFC(n)                    (((n)-1) << EMC_DYNRFC_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 5-31: Reserved */

/* Dynamic Memory Exit Self Refresh register */

#define EMC_DYNXSR_SHIFT                   (0)       /* Bits 0-4: Exit self-refresh to active command time */
#define EMC_DYNXSR_MASK                    (31 << EMC_DYNXSR_SHIFT)
#  define EMC_DYNXSR(n)                    (((n)-1) << EMC_DYNXSR_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 5-31: Reserved */

/* Dynamic Memory Active Bank A to Active Bank B Time register */

#define EMC_DYNRRD_SHIFT                   (0)       /* Bits 0-3: Active bank A to active bank B latency */
#define EMC_DYNRRD_MASK                    (15 << EMC_DYNRRD_SHIFT)
#  define EMC_DYNRRD(n)                    (((n)-1) << EMC_DYNRRD_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 4-31: Reserved */

/* Dynamic Memory Load Mode register to Active Command Time */

#define EMC_DYNMRD_SHIFT                   (0)       /* Bits 0-3: Load mode register to active command time */
#define EMC_DYNMRD_MASK                    (15 << EMC_DYNMRD_SHIFT)
#  define EMC_DYNMRD(n)                    (((n)-1) << EMC_DYNMRD_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 4-31: Reserved */

/* Static Memory Extended Wait register */

#define EMC_STATEXTWAIT_SHIFT              (0)       /* Bits 0-9: Extended wait time out */
#define EMC_STATEXTWAIT_MASK               (0x3ff << EMC_STATEXTWAIT_SHIFT)
#  define EMC_STATEXTWAIT(n)               (((n)-1) << EMC_STATEXTWAIT_SHIFT) /* Delay n CCLK cycles */

                                             /* Bits 10-31: Reserved */

/* Dynamic Memory Configuration registers */

                                                     /* Bits 0-2: Reserved */
#define EMC_DYNCONFIG_MD_SHIFT             (3)       /* Bits 3-4: Memory device */
#define EMC_DYNCONFIG_MD_MASK              (3 << EMC_DYNCONFIG_MD_SHIFT)
#  define EMC_DYNCONFIG_MD_SDRAM           (0 << EMC_DYNCONFIG_MD_SHIFT) /* SDRAM (POR reset value) */

                                                     /* Bits 5-6: Reserved */
#define EMC_DYNCONFIG_AM0_SHIFT            (7)       /* Bits 7-12: AM0 Address mapping (see user manual) */
#define EMC_DYNCONFIG_AM0_MASK             (0x3F << EMC_DYNCONFIG_AM0_SHIFT)
                                                     /* Bit 13: Reserved */
#define EMC_DYNCONFIG_AM1                  (1 << 14) /* Bit 14: AM1 Address mapping (see user manual) */
                                                     /* Bits 15-18: Reserved */
#define EMC_DYNCONFIG_BENA                 (1 << 19) /* Bit 19: Buffer enable */
#define EMC_DYNCONFIG_WP                   (1 << 20) /* Bit 20: Write protect. */
                                                     /* Bits 21-31: Reserved */

/* Dynamic Memory Configuration register  Memory Configuration Values */

/* TODO: complete configuration */

/* Data Bus Width Value in LPC43_EMC_DYNCONFIG register (bit 14) */

#define EMC_DYNCONFIG_DATA_BUS_16          (0 << 14) /* Data bus width 16 bit */
#define EMC_DYNCONFIG_DATA_BUS_32          (1 << 14) /* Data bus width 32 bit */

/* Low power SDRAM value in LPC43_EMC_DYNCONFIG register (bit 12) */

#define EMC_DYNCONFIG_LPSDRAM              (1 << 12) /* Low power SDRAM value (Bank, Row, Column)*/
#define EMC_DYNCONFIG_HPSDRAM              (0 << 12) /* High performance SDRAM value (Row, Bank, Column)*/

/* Address mapping table for LPC43_EMC_DYNCONFIG register (bits 7-11) */

/* Device size bits in LPC43_EMC_DYNCONFIG register (bits 9-11) */

#define EMC_DYNCONFIG_DEV_SIZE_SHIFT       (9)
#define EMC_DYNCONFIG_DEV_SIZE_MASK        (0x7)
#  define EMC_DYNCONFIG_DEV_SIZE_16Mb      (0x00 << EMC_DYNCONFIG_DEV_SIZE_SHIFT)
#  define EMC_DYNCONFIG_DEV_SIZE_64Mb      (0x01 << EMC_DYNCONFIG_DEV_SIZE_SHIFT)
#  define EMC_DYNCONFIG_DEV_SIZE_128Mb     (0x02 << EMC_DYNCONFIG_DEV_SIZE_SHIFT)
#  define EMC_DYNCONFIG_DEV_SIZE_256Mb     (0x03 << EMC_DYNCONFIG_DEV_SIZE_SHIFT)
#  define EMC_DYNCONFIG_DEV_SIZE_512Mb     (0x04 << EMC_DYNCONFIG_DEV_SIZE_SHIFT)

/* Bus width bits in LPC43_EMC_DYNCONFIG register (bits 7-8) */

#define EMC_DYNCONFIG_DEV_BUS_SHIFT        (7)
#define EMC_DYNCONFIG_DEV_BUS_MASK         (0x3)
#  define EMC_DYNCONFIG_DEV_BUS_8          (0x00 << EMC_DYNCONFIG_DEV_BUS_SHIFT)
#  define EMC_DYNCONFIG_DEV_BUS_16         (0x01 << EMC_DYNCONFIG_DEV_BUS_SHIFT)
#  define EMC_DYNCONFIG_DEV_BUS_32         (0x02 << EMC_DYNCONFIG_DEV_BUS_SHIFT)

#define EMC_DYNCONFIG_2Mx8_2BANKS_11ROWS_9COLS    ((0x0 << 9) | (0x0 << 7))  /* 16Mb  (2Mx8),   2 banks, row length = 11, column length = 9  */
#define EMC_DYNCONFIG_1Mx16_2BANKS_11ROWS_8COLS   ((0x0 << 9) | (0x1 << 7))  /* 16Mb  (1Mx16),  2 banks, row length = 11, column length = 8  */
#define EMC_DYNCONFIG_8Mx8_4BANKS_12ROWS_9COLS    ((0x1 << 9) | (0x0 << 7))  /* 64Mb  (8Mx8),   4 banks, row length = 12, column length = 9  */
#define EMC_DYNCONFIG_4Mx16_4BANKS_12ROWS_8COLS   ((0x1 << 9) | (0x1 << 7))  /* 64Mb  (4Mx16),  4 banks, row length = 12, column length = 8  */
#define EMC_DYNCONFIG_2Mx32_4BANKS_11ROWS_8COLS   ((0x1 << 9) | (0x2 << 7))  /* 64Mb  (2Mx32),  4 banks, row length = 11, column length = 8, 32 bit bus only */
#define EMC_DYNCONFIG_16Mx8_4BANKS_12ROWS_10COLS  ((0x2 << 9) | (0x0 << 7))  /* 128Mb (16Mx8),  4 banks, row length = 12, column length = 10 */
#define EMC_DYNCONFIG_8Mx16_4BANKS_12ROWS_9COLS   ((0x2 << 9) | (0x1 << 7))  /* 128Mb (8Mx16),  4 banks, row length = 12, column length = 9  */
#define EMC_DYNCONFIG_4Mx32_4BANKS_12ROWS_8COLS   ((0x2 << 9) | (0x2 << 7))  /* 128Mb (4Mx32),  4 banks, row length = 12, column length = 8  */
#define EMC_DYNCONFIG_32Mx8_4BANKS_13ROWS_10COLS  ((0x3 << 9) | (0x0 << 7))  /* 256Mb (32Mx8),  4 banks, row length = 13, column length = 10, 32 bit bus only */
#define EMC_DYNCONFIG_16Mx16_4BANKS_13ROWS_9COLS  ((0x3 << 9) | (0x1 << 7))  /* 256Mb (16Mx16), 4 banks, row length = 13, column length = 9  */
#define EMC_DYNCONFIG_8Mx32_4BANKS_13ROWS_8COLS   ((0x3 << 9) | (0x2 << 7))  /* 256Mb (8Mx32),  4 banks, row length = 13, column length = 8, 32 bit bus only  */
#define EMC_DYNCONFIG_8Mx32_4BANKS_12ROWS_9COLS   ((0x2 << 9) | (0x1 << 7))  /* 256Mb (8Mx32),  4 banks, row length = 12, column length = 9, 32 bit bus only  */
#define EMC_DYNCONFIG_64Mx8_4BANKS_13ROWS_10COLS  ((0x4 << 9) | (0x0 << 7))  /* 512Mb (64Mx8),  4 banks, row length = 13, column length = 11 */
#define EMC_DYNCONFIG_32Mx16_4BANKS_13ROWS_10COLS ((0x4 << 9) | (0x1 << 7))  /* 512Mb (32Mx16), 4 banks, row length = 13, column length = 10 */
#define EMC_DYNCONFIG_16Mx32_4BANKS_13ROWS_9COLS  ((0x3 << 9) | (0x1 << 7))  /* 512Mb (16Mx32), 4 banks, row length = 13, column length = 9, 32 bit bus only  */
#define EMC_DYNCONFIG_32Mx32_4BANKS_13ROWS_10COLS ((0x4 << 9) | (0x1 << 7))  /* 1Gb   (32Mx32), 4 banks, row length = 13, column length = 10,32 bit bus only  */

/* Dynamic Memory RAS & CAS Delay registers */

#define EMC_DYNRASCAS_RAS_SHIFT            (0)       /* Bits 0-1: RAS latency (active to read/write delay) */
#define EMC_DYNRASCAS_RAS_MASK             (3 << EMC_DYNRASCAS_RAS_SHIFT)
#  define EMC_DYNRASCAS_RAS_1CCLK          (1 << EMC_DYNRASCAS_RAS_SHIFT) /* One CCLK cycle */
#  define EMC_DYNRASCAS_RAS_2CCLK          (2 << EMC_DYNRASCAS_RAS_SHIFT) /* Two CCLK cycles */
#  define EMC_DYNRASCAS_RAS_3CCLK          (3 << EMC_DYNRASCAS_RAS_SHIFT) /* Three CCLK cycles (POR reset value) */

                                                     /* Bits 2-7: Reserved */
#define EMC_DYNRASCAS_CAS_SHIFT            (8)       /* Bits 8-9: CAS latency */
#define EMC_DYNRASCAS_CAS_MASK             (3 << EMC_DYNRASCAS_CAS_SHIFT)
#  define EMC_DYNRASCAS_CAS_1CCLK          (1 << EMC_DYNRASCAS_CAS_SHIFT) /* One CCLK cycle */
#  define EMC_DYNRASCAS_CAS_2CCLK          (2 << EMC_DYNRASCAS_CAS_SHIFT) /* Two CCLK cycles */
#  define EMC_DYNRASCAS_CAS_3CCLK          (3 << EMC_DYNRASCAS_CAS_SHIFT) /* Three CCLK cycles (POR reset value) */

                                             /* Bits 10-31: Reserved */

/* Dynamic SDRAM mode register definitions */

/*                                        Bits 0-2: Burst length.
 *                                             All other values are reserved.
 */
#define EMC_DYNMODE_BURST_LENGTH_SHIFT     (0)
#define EMC_DYNMODE_BURST_LENGTH_MASK      (0x7)
#  define EMC_DYNMODE_BURST_LENGTH_1       (0 << EMC_DYNMODE_BURST_LENGTH_SHIFT)
#  define EMC_DYNMODE_BURST_LENGTH_2       (1 << EMC_DYNMODE_BURST_LENGTH_SHIFT)
#  define EMC_DYNMODE_BURST_LENGTH_4       (2 << EMC_DYNMODE_BURST_LENGTH_SHIFT)
#  define EMC_DYNMODE_BURST_LENGTH_8       (3 << EMC_DYNMODE_BURST_LENGTH_SHIFT)
                                                 /* Bit 3: Burst mode type */
#define EMC_DYNMODE_BURST_TYPE_SHIFT            (3)
#  define EMC_DYNMODE_BURST_TYPE_SEQUENTIAL     (0 << EMC_DYNMODE_BURST_TYPE_SHIFT)       /* burst type sequential */
#  define EMC_DYNMODE_BURST_TYPE_INTERLEAVED    (1 << EMC_DYNMODE_BURST_TYPE_INTERLEAVED) /* burst type interleaved */

/*                                          Bits 4-6: Latency mode.
 *                                             All other values are reserved.
 */
#define EMC_DYNMODE_CAS_SHIFT              (4)
#define EMC_DYNMODE_CAS_MASK               (0x7)
#  define EMC_DYNMODE_CAS_2                (2 << EMC_DYNMODE_CAS_SHIFT) /* CAS latency of 2 cycles */
#  define EMC_DYNMODE_CAS_3                (3 << EMC_DYNMODE_CAS_SHIFT) /* CAS latency of 3 cycles */

/*                                          Bits 7-8: Operating mode.
 *                                             All other values are reserved.
 */
#define EMC_DYNMODE_OPMODE_SHIFT           (7)
#define EMC_DYNMODE_OPMODE_MASK            (0x3)
#  define EMC_DYNMODE_OPMODE_STANDARD      (0 << EMC_DYNMODE_OPMODE_SHIFT) /* dynamic standard operation mode */

                                            /* Bit 9: Write burst mode */
#define EMC_DYNMODE_WBMODE_SHIFT           (9)
#  define EMC_DYNMODE_WBMODE_PROGRAMMED    (0 << EMC_DYNMODE_WBMODE_SHIFT) /* write burst mode programmed */
#  define EMC_DYNMODE_WBMODE_SINGLE_LOC    (1 << EMC_DYNMODE_WBMODE_SHIFT) /* write burst mode single loc */

                                                    /* Bits 10-11: Reserved */

/* Static Memory Configuration registers */

#define EMC_STATCONFIG_MW_SHIFT            (0)       /* Bits 0-1: Memory width */
#define EMC_STATCONFIG_MW_MASK             (3 << EMC_STATCONFIG_MW_SHIFT)
#  define EMC_STATCONFIG_MW_8BITS          (0 << EMC_STATCONFIG_MW_SHIFT)
#  define EMC_STATCONFIG_MW_16BITS         (1 << EMC_STATCONFIG_MW_SHIFT)
#  define EMC_STATCONFIG_MW_32BITS         (2 << EMC_STATCONFIG_MW_SHIFT)
                                                     /* Bit 2:  Reserved */
#define EMC_STATCONFIG_PM                  (1 << 3)  /* Bit 3:  Page mode */
                                                     /* Bits 4-5: Reserved */
#define EMC_STATCONFIG_PC                  (1 << 6)  /* Bit 6:  Chip select polarity */
#define EMC_STATCONFIG_PB                  (1 << 7)  /* Bit 7:  Byte lane state */
#define EMC_STATCONFIG_EW                  (1 << 8)  /* Bit 8:  Extended wait */
                                                     /* Bits 9-18: Reserved */
#define EMC_STATCONFIG_BENA                (1 << 19) /* Bit 19: Buffer enable */
#define EMC_STATCONFIG_WP                  (1 << 20) /* Bit 20: Write protect */
                                                     /* Bits 21-31: Reserved */

/* Static Memory Write Enable Delay registers */

#define EMC_STATWAITWEN_SHIFT              (0)       /* Bits 0-3: Wait write enable */
#define EMC_STATWAITWEN_MASK               (15 << EMC_STATWAITWEN_SHIFT)
#  define EMC_STATWAITWEN(n)               (((n)-1) << EMC_STATWAITWEN_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 4-31: Reserved */

/* Static Memory Output Enable Delay registers */

#define EMC_STATWAITOEN_SHIFT              (0)       /* Bits 0-3: Wait output enable */
#define EMC_STATWAITOEN_MASK               (15 << EMC_STATWAITOEN_SHIFT)
#  define EMC_STATWAITOEN(n)               (((n)-1) << EMC_STATWAITOEN_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 4-31: Reserved */

/* Static Memory Read Delay registers */

#define EMC_STATWAITRD_SHIFT               (0)       /* Bits 0-4: Non-page mode read wait states or
                                                      * asynchronous page mode read first access wait state */
#define EMC_STATWAITRD_MASK                (31 << EMC_STATWAITRD_SHIFT)
#  define EMC_STATWAITRD(n)                (((n)-1) << EMC_STATWAITRD_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 5-31: Reserved */

/* Static Memory Page Mode Read Delay registers */

#define EMC_STATWAITPAGE_SHIFT             (0)       /* Bits 0-4: Asynchronous page mode read after the
                                                      * first read wait states */
#define EMC_STATWAITPAGE_MASK              (31 << EMC_STATWAITPAGE_SHIFT)
#  define EMC_STATWAITPAGE(n)              (((n)-1) << EMC_STATWAITPAGE_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 5-31: Reserved */

/* Static Memory Write Delay registers */

#define EMC_STATWAITWR_SHIFT               (0)       /* Bits 0-4: Write wait states */
#define EMC_STATWAITWR_MASK                (31 << EMC_STATWAITWR_SHIFT)
#  define EMC_STATWAITWR(n)                (((n)-1) << EMC_STATWAITWR_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 5-31: Reserved */

/* Static Memory Turn Round Delay registers */

#define EMC_STATWAITTURN_SHIFT             (0)       /* Bits 0-3: Bus turnaround cycles */
#define EMC_STATWAITTURN_MASK              (15 << EMC_STATWAITTURN_SHIFT)
#  define EMC_STATWAITTURN(n)              (((n)-1) << EMC_STATWAITTURN_SHIFT) /* Delay n CCLK cycles */

                                                     /* Bits 5-31: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_EMC_H */
