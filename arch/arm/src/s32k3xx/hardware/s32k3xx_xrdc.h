/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_xrdc.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_XRDC_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_XRDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* XRDC Register Offsets ****************************************************/

#define S32K3XX_XRDC_CR_OFFSET             (0x0000) /* Control Register (CR) */
#define S32K3XX_XRDC_HWCFG0_OFFSET         (0x00f0) /* Hardware Configuration Register 0 (HWCFG0) */
#define S32K3XX_XRDC_HWCFG1_OFFSET         (0x00f4) /* Hardware Configuration Register 1 (HWCFG1) */
#define S32K3XX_XRDC_HWCFG2_OFFSET         (0x00f8) /* Hardware Configuration Register 2 (HWCFG2) */
#define S32K3XX_XRDC_MDACFG0_OFFSET        (0x0100) /* Master Domain Assignment Configuration Register 0 (MDACFG0) */
#define S32K3XX_XRDC_MDACFG1_OFFSET        (0x0101) /* Master Domain Assignment Configuration Register 1 (MDACFG1) */
#define S32K3XX_XRDC_MDACFG2_OFFSET        (0x0102) /* Master Domain Assignment Configuration Register 2 (MDACFG2) */
#define S32K3XX_XRDC_MDACFG3_OFFSET        (0x0103) /* Master Domain Assignment Configuration Register 3 (MDACFG3) */
#define S32K3XX_XRDC_MDACFG4_OFFSET        (0x0104) /* Master Domain Assignment Configuration Register 4 (MDACFG4) */
#define S32K3XX_XRDC_MDACFG5_OFFSET        (0x0105) /* Master Domain Assignment Configuration Register 5 (MDACFG5) */
#define S32K3XX_XRDC_MRCFG0_OFFSET         (0x0140) /* Memory Region Configuration Register 0 (MRCFG0) */
#define S32K3XX_XRDC_MRCFG1_OFFSET         (0x0141) /* Memory Region Configuration Register 1 (MRCFG1) */
#define S32K3XX_XRDC_MRCFG2_OFFSET         (0x0142) /* Memory Region Configuration Register 2 (MRCFG2) */
#define S32K3XX_XRDC_DERRLOC0_OFFSET       (0x0200) /* Domain Error Location Register 0 (DERRLOC0) */
#define S32K3XX_XRDC_DERRLOC1_OFFSET       (0x0204) /* Domain Error Location Register 1 (DERRLOC1) */
#define S32K3XX_XRDC_DERRLOC2_OFFSET       (0x0208) /* Domain Error Location Register 2 (DERRLOC2) */
#define S32K3XX_XRDC_DERR_W0_0_OFFSET      (0x0400) /* Domain Error Word 0 (DERR_W0_0) */
#define S32K3XX_XRDC_DERR_W1_0_OFFSET      (0x0404) /* Domain Error Word 1 (DERR_W1_0) */
#define S32K3XX_XRDC_DERR_W3_0_OFFSET      (0x040c) /* Domain Error Word 3 (DERR_W3_0) */
#define S32K3XX_XRDC_DERR_W0_1_OFFSET      (0x0410) /* Domain Error Word 0 (DERR_W0_1) */
#define S32K3XX_XRDC_DERR_W1_1_OFFSET      (0x0414) /* Domain Error Word 1 (DERR_W1_1) */
#define S32K3XX_XRDC_DERR_W3_1_OFFSET      (0x041c) /* Domain Error Word 3 (DERR_W3_1) */
#define S32K3XX_XRDC_DERR_W0_2_OFFSET      (0x0420) /* Domain Error Word 0 (DERR_W0_2) */
#define S32K3XX_XRDC_DERR_W1_2_OFFSET      (0x0424) /* Domain Error Word 1 (DERR_W1_2) */
#define S32K3XX_XRDC_DERR_W3_2_OFFSET      (0x042c) /* Domain Error Word 3 (DERR_W3_2) */
#define S32K3XX_XRDC_DERR_W0_16_OFFSET     (0x0500) /* Domain Error Word 0 (DERR_W0_16) */
#define S32K3XX_XRDC_DERR_W1_16_OFFSET     (0x0504) /* Domain Error Word 1 (DERR_W1_16) */
#define S32K3XX_XRDC_DERR_W3_16_OFFSET     (0x050c) /* Domain Error Word 3 (DERR_W3_16) */
#define S32K3XX_XRDC_DERR_W0_17_OFFSET     (0x0510) /* Domain Error Word 0 (DERR_W0_17) */
#define S32K3XX_XRDC_DERR_W1_17_OFFSET     (0x0514) /* Domain Error Word 1 (DERR_W1_17) */
#define S32K3XX_XRDC_DERR_W3_17_OFFSET     (0x051c) /* Domain Error Word 3 (DERR_W3_17) */
#define S32K3XX_XRDC_DERR_W0_18_OFFSET     (0x0520) /* Domain Error Word 0 (DERR_W0_18) */
#define S32K3XX_XRDC_DERR_W1_18_OFFSET     (0x0524) /* Domain Error Word 1 (DERR_W1_18) */
#define S32K3XX_XRDC_DERR_W3_18_OFFSET     (0x052c) /* Domain Error Word 3 (DERR_W3_18) */
#define S32K3XX_XRDC_PID0_OFFSET           (0x0700) /* Process Identifier (PID0) */
#define S32K3XX_XRDC_PID3_OFFSET           (0x070c) /* Process Identifier (PID3) */
#define S32K3XX_XRDC_PID4_OFFSET           (0x0710) /* Process Identifier (PID4) */
#define S32K3XX_XRDC_MDA_W0_0_DFMT0_OFFSET (0x0800) /* Master Domain Assignment (MDA_W0_0_DFMT0) */
#define S32K3XX_XRDC_MDA_W0_1_DFMT1_OFFSET (0x0820) /* Master Domain Assignment (MDA_W0_1_DFMT1) */
#define S32K3XX_XRDC_MDA_W0_2_DFMT1_OFFSET (0x0840) /* Master Domain Assignment (MDA_W0_2_DFMT1) */
#define S32K3XX_XRDC_MDA_W0_3_DFMT0_OFFSET (0x0860) /* Master Domain Assignment (MDA_W0_3_DFMT0) */
#define S32K3XX_XRDC_MDA_W0_4_DFMT0_OFFSET (0x0870) /* Master Domain Assignment (MDA_W0_4_DFMT0) */
#define S32K3XX_XRDC_MDA_W0_5_DFMT1_OFFSET (0x08a0) /* Master Domain Assignment (MDA_W0_5_DFMT1) */

#define S32K3XX_XRDC_PDAC_W0_OFFSET(n)     (0x1100 + (((n) - 32) << 3)) /* Peripheral Domain Access Control (PDAC_W0_n, n=32..315) */
#define S32K3XX_XRDC_PDAC_W1_OFFSET(n)     (0x1104 + (((n) - 32) << 3)) /* Peripheral Domain Access Control (PDAC_W1_n, n=32..315) */

#define S32K3XX_XRDC_MRGD_W0_OFFSET(n)     (0x2000 + ((n) << 5)) /* Memory Region Descriptor (MRGD_W0_n, n=0..35) */
#define S32K3XX_XRDC_MRGD_W1_OFFSET(n)     (0x2004 + ((n) << 5)) /* Memory Region Descriptor (MRGD_W1_n, n=0..35) */
#define S32K3XX_XRDC_MRGD_W2_OFFSET(n)     (0x2008 + ((n) << 5)) /* Memory Region Descriptor (MRGD_W2_n, n=0..35) */
#define S32K3XX_XRDC_MRGD_W3_OFFSET(n)     (0x200c + ((n) << 5)) /* Memory Region Descriptor (MRGD_W3_n, n=0..35) */

/* XRDC Register Addresses **************************************************/

#define S32K3XX_XRDC_CR                    (S32K3XX_XRDC_BASE + S32K3XX_XRDC_CR_OFFSET)
#define S32K3XX_XRDC_HWCFG0                (S32K3XX_XRDC_BASE + S32K3XX_XRDC_HWCFG0_OFFSET)
#define S32K3XX_XRDC_HWCFG1                (S32K3XX_XRDC_BASE + S32K3XX_XRDC_HWCFG1_OFFSET)
#define S32K3XX_XRDC_HWCFG2                (S32K3XX_XRDC_BASE + S32K3XX_XRDC_HWCFG2_OFFSET)
#define S32K3XX_XRDC_MDACFG0               (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MDACFG0_OFFSET)
#define S32K3XX_XRDC_MDACFG1               (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MDACFG0_OFFSET)
#define S32K3XX_XRDC_MDACFG2               (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MDACFG0_OFFSET)
#define S32K3XX_XRDC_MDACFG3               (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MDACFG0_OFFSET)
#define S32K3XX_XRDC_MDACFG4               (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MDACFG0_OFFSET)
#define S32K3XX_XRDC_MDACFG5               (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MDACFG0_OFFSET)
#define S32K3XX_XRDC_MRCFG0                (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MRCFG0_OFFSET)
#define S32K3XX_XRDC_MRCFG1                (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MRCFG1_OFFSET)
#define S32K3XX_XRDC_MRCFG2                (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MRCFG2_OFFSET)
#define S32K3XX_XRDC_DERRLOC0              (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERRLOC0_OFFSET)
#define S32K3XX_XRDC_DERRLOC1              (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERRLOC1_OFFSET)
#define S32K3XX_XRDC_DERRLOC2              (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERRLOC2_OFFSET)
#define S32K3XX_XRDC_DERR_W0_0             (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W0_0_OFFSET)
#define S32K3XX_XRDC_DERR_W1_0             (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W1_0_OFFSET)
#define S32K3XX_XRDC_DERR_W3_0             (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W3_0_OFFSET)
#define S32K3XX_XRDC_DERR_W0_1             (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W0_1_OFFSET)
#define S32K3XX_XRDC_DERR_W1_1             (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W1_1_OFFSET)
#define S32K3XX_XRDC_DERR_W3_1             (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W3_1_OFFSET)
#define S32K3XX_XRDC_DERR_W0_2             (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W0_2_OFFSET)
#define S32K3XX_XRDC_DERR_W1_2             (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W1_2_OFFSET)
#define S32K3XX_XRDC_DERR_W3_2             (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W3_2_OFFSET)
#define S32K3XX_XRDC_DERR_W0_16            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W0_16_OFFSET)
#define S32K3XX_XRDC_DERR_W1_16            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W1_16_OFFSET)
#define S32K3XX_XRDC_DERR_W3_16            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W3_16_OFFSET)
#define S32K3XX_XRDC_DERR_W0_17            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W0_17_OFFSET)
#define S32K3XX_XRDC_DERR_W1_17            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W1_17_OFFSET)
#define S32K3XX_XRDC_DERR_W3_17            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W3_17_OFFSET)
#define S32K3XX_XRDC_DERR_W0_18            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W0_18_OFFSET)
#define S32K3XX_XRDC_DERR_W1_18            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W1_18_OFFSET)
#define S32K3XX_XRDC_DERR_W3_18            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_DERR_W3_18_OFFSET)
#define S32K3XX_XRDC_PID0                  (S32K3XX_XRDC_BASE + S32K3XX_XRDC_PID0_OFFSET)
#define S32K3XX_XRDC_PID3                  (S32K3XX_XRDC_BASE + S32K3XX_XRDC_PID3_OFFSET)
#define S32K3XX_XRDC_PID4                  (S32K3XX_XRDC_BASE + S32K3XX_XRDC_PID4_OFFSET)
#define S32K3XX_XRDC_MDA_W0_0_DFMT0        (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MDA_W0_0_DFMT0_OFFSET)
#define S32K3XX_XRDC_MDA_W0_1_DFMT1        (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MDA_W0_1_DFMT1_OFFSET)
#define S32K3XX_XRDC_MDA_W0_2_DFMT1        (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MDA_W0_2_DFMT1_OFFSET)
#define S32K3XX_XRDC_MDA_W0_3_DFMT0        (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MDA_W0_3_DFMT0_OFFSET)
#define S32K3XX_XRDC_MDA_W0_4_DFMT0        (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MDA_W0_4_DFMT0_OFFSET)
#define S32K3XX_XRDC_MDA_W0_5_DFMT1        (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MDA_W0_5_DFMT1_OFFSET)

#define S32K3XX_XRDC_PDAC_W0(n)            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_PDAC_W0_OFFSET(n))
#define S32K3XX_XRDC_PDAC_W1(n)            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_PDAC_W1_OFFSET(n))
#define S32K3XX_XRDC_MRGD_W0(n)            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MRGD_W0_OFFSET(n))
#define S32K3XX_XRDC_MRGD_W1(n)            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MRGD_W1_OFFSET(n))
#define S32K3XX_XRDC_MRGD_W2(n)            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MRGD_W2_OFFSET(n))
#define S32K3XX_XRDC_MRGD_W3(n)            (S32K3XX_XRDC_BASE + S32K3XX_XRDC_MRGD_W3_OFFSET(n))

/* XRDC Register Bitfield Definitions ***************************************/

/* Control Register (CR) */

#define XRDC_CR_GVLD                      (1 << 0)  /* Bit 0: Global Valid (XRDC Global Enable/Disable) (GLVD) */
#define XRDC_CR_HRL_SHIFT                 (1)       /* Bits 1-4: Hardware Revision Level (HRL) */
#define XRDC_CR_HRL_MASK                  (0x0f << XRDC_CR_HRL_SHIFT)
                                                    /* Bits 5-6: Reserved */
#define XRDC_CR_MRF                       (1 << 7)  /* Bit 7: Memory Region Format (MRF) */
#define XRDC_CR_VAW                       (1 << 8)  /* Bit 8: Virtualization Aware (VAW) */
                                                    /* Bit 9-29: Reserved */
#define XRDC_CR_LK1                       (1 << 30) /* Bit 30: Lock (LK1) */
                                                    /* Bit 31: Reserved */

/* Hardware Configuration Register 0 (HWCFG0) */

#define XRDC_HWCFG0_NDID_SHIFT            (0)       /* Bits 0-7: Number Of Domains (NDID) */
#define XRDC_HWCFG0_NDID_MASK             (0xff << XRDC_HWCFG0_NDID_SHIFT)
#define XRDC_HWCFG0_NMSTR_SHIFT           (8)       /* Bits 8-15: Number Of Bus Masters (NMSTR) */
#define XRDC_HWCFG0_NMSTR_MASK            (0xff << XRDC_HWCFG0_NMSTR_SHIFT)
#define XRDC_HWCFG0_NMRC_SHIFT            (16)     /* Bits 16-23: Number Of MRCs (NMRC) */
#define XRDC_HWCFG0_NMRC_MASK             (0xff << XRDC_HWCFG0_NMRC_SHIFT)
#define XRDC_HWCFG0_NPAC_SHIFT            (24)     /* Bits 24-27: Number of PACs (NPAC) */
#define XRDC_HWCFG0_NPAC_MASK             (0x0f << XRDC_HWCFG0_NPAC_SHIFT)
#define XRDC_HWCFG0_MID_SHIFT             (28)     /* Bits 28-31: Module ID (MID) */
#define XRDC_HWCFG0_MID_MASK              (0x0f << XRDC_HWCFG0_MID_SHIFT)

/* Hardware Configuration Register 1 (HWCFG1) */

#define XRDC_HWCFG1_DID_SHIFT             (0)      /* Bits 0-3: Domain Identifier Number (DID) */
#define XRDC_HWCFG1_DID_MASK              (0x0f << XRDC_HWCFG1_DID_SHIFT)
                                                   /* Bits 4-31: Reserved */

/* Hardware Configuration Register 2 (HWCFG2) */

#define XRDC_HWCFG2_PIDP(n)               (1 << (n)) /* Bit n: Process Identifier n Present (PIDPn) */

/* Master Domain Assignment Configuration Register n (MDACFGn) */

#define XRDC_MDACFG_NMDAR_SHIFT           (0)       /* Bits 0-3: Number Of Master Domain Assignment Registers for Bus Master n (NMDAR) */
#define XRDC_MDACFG_NMDAR_MASK            (0x0f << XRDC_MDACFG_NMDAR_SHIFT)
                                                    /* Bits 4-6: Reserved */
#define XRDC_MDACFG_NCM                   (1 << 7)  /* Bit 7: Non-CPU Master (NCM) */

/* Memory Region Configuration Register n (MRCFGn) */

#define XRDC_MRCFG_NMRGD_SHIFT            (0)       /* Bits 0-4: Number Of Memory Region Descriptor For Memory Region Controller n (NMRGD) */
#define XRDC_MRCFG_NMRGD_MASK             (0x1f << XRDC_MRCFG_NMRGD_SHIFT)
                                                    /* Bits 5-7: Reserved */

/* Domain Error Location Register n (DERRLOCn) */

#define XRDC_DERRLOC_MRCINST_SHIFT        (0)       /* Bits 0-15: MRC Instance (MCRINST) */
#define XRDC_DERRLOC_MRCINST_MASK         (0xffff << XRDC_DERRLOC_MRCINST_SHIFT)
#define XRDC_DERRLOC_PACINST_SHIFT        (16)      /* Bits 16-19: PAC Instance (PACINST) */
#define XRDC_DERRLOC_PACINST_MASK         (0x0f << XRDC_DERRLOC_PACINST_SHIFT)
                                                    /* Bits 20-31: Reserved */

/* Domain Error Word 0 (DERR_W0_n) */

#define XRDC_DERR_W0_EADDR_SHIFT          (0)       /* Bits 0-31: Error Address (EADDR) */
#define XRDC_DERR_W0_EADDR_MASK           (0xffffffff << XRDC_DERR_W0_EADDR_SHIFT)

/* Domain Error Word 1 (DERR_W1_n) */

#define XRDC_DERR_W1_EDID_SHIFT           (0)       /* Bits 0-3: Error Domain Identifier (EDID) */
#define XRDC_DERR_W1_EDID_MASK            (0x0f << XRDC_DERR_W1_EDID_SHIFT)
                                                    /* Bits 4-7: Reserved */
#define XRDC_DERR_W1_EATR_SHIFT           (8)       /* Bits 8-10: Error Attributes (EATR) */
#define XRDC_DERR_W1_EATR_MASK            (0x07 << XRDC_DERR_W1_EATR_SHIFT)
#  define XRDC_DERR_W1_EATR_SUM_IFA       (0x00 << XRDC_DERR_W1_EATR_SHIFT) /* Secure user mode, instruction fetch access */
#  define XRDC_DERR_W1_EATR_SUM_DA        (0x01 << XRDC_DERR_W1_EATR_SHIFT) /* Secure user mode, data access */
#  define XRDC_DERR_W1_EATR_SPM_IFA       (0x02 << XRDC_DERR_W1_EATR_SHIFT) /* Secure privileged mode, instruction fetch access */
#  define XRDC_DERR_W1_EATR_SPM_DA        (0x03 << XRDC_DERR_W1_EATR_SHIFT) /* Secure privileged mode, data access */
#  define XRDC_DERR_W1_EATR_NSUM_IFA      (0x04 << XRDC_DERR_W1_EATR_SHIFT) /* Nonsecure user mode, instruction fetch access */
#  define XRDC_DERR_W1_EATR_NSUM_DA       (0x05 << XRDC_DERR_W1_EATR_SHIFT) /* Nonsecure user mode, data access */
#  define XRDC_DERR_W1_EATR_NSPM_IFA      (0x06 << XRDC_DERR_W1_EATR_SHIFT) /* Nonsecure privileged mode, instruction fetch access */
#  define XRDC_DERR_W1_EATR_NSPM_DA       (0x07 << XRDC_DERR_W1_EATR_SHIFT) /* Nonsecure privileged mode, data access */

#define XRDC_DERR_W1_ERW                  (1 << 11) /* Bit 11: Error Read/Write (ERW) */
                                                    /* Bits 12-23: Reserved */
#define XRDC_DERR_W1_EPORT_SHIFT          (24)      /* Bits 24-26: Error Port (EPORT) */
#define XRDC_DERR_W1_EPORT_MASK           (0x07 << XRDC_DERR_W1_EPORT_SHIFT)
                                                    /* Bits 27-29: Reserved */
#define XRDC_DERR_W1_EST_SHIFT            (30)      /* Bits 30-31: Error State (EST) */
#define XRDC_DERR_W1_EST_MASK             (0x03 << XRDC_DERR_W1_EST_SHIFT)

/* Domain Error Word 3 (DERR_W3_n) */

                                                    /* Bits 0-29: Reserved */
#define XRDC_DERR_W3_RECR_SHIFT           (30)      /* Bits 30-31: Rearm Error Capture Registers (RECR) */
#define XRDC_DERR_W3_RECR_MASK            (0x03 << XRDC_DERR_W3_RECR_SHIFT)
#define XRDC_DERR_W3_RECR_REARM           (0x01 << XRDC_DERR_W3_RECR_SHIFT) /* Rearm the error capture mechanism and clear registers DERR_W0_n and DERR_W1_n */

/* Process Identifier (PIDn) */

#define XRDC_PID_PID_SHIFT                (0)       /* Bits 0-5: Process Identifier (PID) */
#define XRDC_PID_PID_MASK                 (0x3f << XRDC_PID_PID_SHIFT)
                                                    /* Bits 6-15: Reserved */
#define XRDC_PID_LMNUM_SHIFT              (16)      /* Bits 16-21: Locked Master Number (LMNUM) */
#define XRDC_PID_LMNUM_MASK               (0x03f << XRDC_PID_LMNUM_SHIFT)
                                                    /* Bits 22-23: Reserved */
#define XRDC_PID_ELK22H                   (1 << 24) /* Bit 24: Enable (LK2 = 2) Special Handling (ELK22H) */
                                                    /* Bits 25-27: Reserved */
#define XRDC_PID_TSM                      (1 << 28) /* Bit 28: Three-State Model (TSM) */
#define XRDC_PID_LK2_SHIFT                (29)      /* Bits 29-30: Lock (LK2) */
#define XRDC_PID_LK2_MASK                 (0x03 << XRDC_PID_LK2_SHIFT)
#  define XRDC_PID_LK2_ANY                (0x00 << XRDC_PID_LK2_SHIFT) /* Register can be written to by any secure privileged write */
#  define XRDC_PID_LK2_MASTER             (0x02 << XRDC_PID_LK2_SHIFT) /* Register can only be written by a secure privileged write from bus master n */
#  define XRDC_PID_LK2_LOCKED             (0x03 << XRDC_PID_LK2_SHIFT) /* Register locked (read-only) until the next reset */

                                                    /* Bit 31: Reserved */

/* Master Domain Assignment (MDA_W0_n_DFMT0, n=0,3,4) */

#define XRDC_MDA_W0_DFMT0_DID_SHIFT       (0)       /* Bits 0-1: Domain Identifier (DID) */
#define XRDC_MDA_W0_DFMT0_DID_MASK        (0x03 << XRDC_MDA_W0_DFMT0_DID_SHIFT)
                                                    /* Bits 2-3: Reserved */
#define XRDC_MDA_W0_DFMT0_DIDS_SHIFT      (4)       /* Bits 4-5: DID Select (DIDS) */
#define XRDC_MDA_W0_DFMT0_DIDS_MASK       (0x03 << XRDC_MDA_W0_DFMT0_DIDS_SHIFT)
#  define XRDC_MDA_W0_DFMT0_DIDS_REGISTER (0x00 << XRDC_MDA_W0_DFMT0_DIDS_SHIFT) /* Use the DID field of this register as the domain identifier */
#  define XRDC_MDA_W0_DFMT0_DIDS_INPUT    (0x01 << XRDC_MDA_W0_DFMT0_DIDS_SHIFT) /* Use the input DID sa the domain identifier */
#  define XRDC_MDA_W0_DFMT0_DIDS_COMBINED (0x02 << XRDC_MDA_W0_DFMT0_DIDS_SHIFT) /* Use bits [3:2] of this register concatenated with the low-order 2 bits of the input DID (DID_in[1:0]) as the domain identifier */

#define XRDC_MDA_W0_DFMT0_PE_SHIFT        (6)       /* Bits 6-7: Process Identifier Enable (PE) */
#define XRDC_MDA_W0_DFMT0_PE_MASK         (0x03 << XRDC_MDA_W0_DFMT0_PE_SHIFT)
#  define XRDC_MDA_W0_DFMT0_PE_NOPID      (0x00 << XRDC_MDA_W0_DFMT0_PE_SHIFT) /* No process identifier is included in the domain hit evaluation */
#  define XRDC_MDA_W0_DFMT0_PE2           (0x02 << XRDC_MDA_W0_DFMT0_PE_SHIFT) /* The process identifier is included in the domain hit evaluation (see reference manual) */
#  define XRDC_MDA_W0_DFMT0_PE3           (0x03 << XRDC_MDA_W0_DFMT0_PE_SHIFT) /* The process identifier is included in the domain hit evaluation (see reference manual) */

#define XRDC_MDA_W0_DFMT0_PIDM_SHIFT      (8)       /* Bits 8-13: Process Identifier Mask (PIDM) */
#define XRDC_MDA_W0_DFMT0_PIDM_MASK       (0x3f << XRDC_MDA_W0_DFMT0_PIDM_SHIFT)
                                                    /* Bits 14-15: Reserved */
#define XRDC_MDA_W0_DFMT0_PID_SHIFT       (16)      /* Bits 16-21: Process Identifier (PID) */
#define XRDC_MDA_W0_DFMT0_PID_MASK        (0x3f << XRDC_MDA_W0_DFMT0_PID_SHIFT)
                                                    /* Bits 22-28: Reserved */
#define XRDC_MDA_W0_DFMT0_DFMT            (1 << 29) /* Bit 29: Domain Format (DFMT) */
#define XRDC_MDA_W0_DFMT0_LK1             (1 << 30) /* Bit 30: Lock (LK1) */
#define XRDC_MDA_W0_DFMT0_VLD             (1 << 31) /* Bit 31: Valid (VLD) */

/* Master Domain Assignment (MDA_W0_n_DFMT1, n=1,2,5) */

#define XRDC_MDA_W0_DFMT1_DID_SHIFT       (0)       /* Bits 0-1: Domain Identifier (DID) */
#define XRDC_MDA_W0_DFMT1_DID_MASK        (0x03 << XRDC_MDA_W0_DFMT0_DID_SHIFT)
                                                    /* Bits 2-3: Reserved */
#define XRDC_MDA_W0_DFMT1_PA_SHIFT        (4)       /* Bits 4-5: Privileged Attribute (PA) */
#define XRDC_MDA_W0_DFMT1_PA_MASK         (0x03 << XRDC_MDA_W0_DFMT1_PA_SHIFT)
#define XRDC_MDA_W0_DFMT1_SA_SHIFT        (6)       /* Bits 6-7: Secure Attribute (SA) */
#define XRDC_MDA_W0_DFMT1_SA_MASK         (0x03 << XRDC_MDA_W0_DFMT1_SA_SHIFT)
#define XRDC_MDA_W0_DFMT1_DIDB            (1 << 8)  /* Bit 8: DID Bypass (DIDB) */
                                                    /* Bits 9-28: Reserved */
#define XRDC_MDA_W0_DFMT1_DFMT            (1 << 29) /* Bit 29: Domain Format (DFMT) */
#define XRDC_MDA_W0_DFMT1_LK1             (1 << 30) /* Bit 30: Lock (LK1) */
#define XRDC_MDA_W0_DFMT1_VLD             (1 << 31) /* Bit 31: Valid (VLD) */

/* Peripheral Domain Access Control (PDAC_W0_n, n=32..315) */

#define XRDC_PDAC_W0_D0ACP_SHIFT          (0)       /* Bits 0-2: Domain 0 Access Control Policy (D0ACP) */
#define XRDC_PDAC_W0_D0ACP_MASK           (0x07 << XRDC_PDAC_W0_D0ACP_SHIFT)
#define XRDC_PDAC_W0_D1ACP_SHIFT          (3)       /* Bits 3-5: Domain 1 Access Control Policy (D1ACP) */
#define XRDC_PDAC_W0_D1ACP_MASK           (0x07 << XRDC_PDAC_W0_D1ACP_SHIFT)
#define XRDC_PDAC_W0_D2ACP_SHIFT          (6)       /* Bits 6-8: Domain 2 Access Control Policy (D2ACP) */
#define XRDC_PDAC_W0_D2ACP_MASK           (0x07 << XRDC_PDAC_W0_D2ACP_SHIFT)
                                                    /* Bits 9-23: Reserved */
#define XRDC_PDAC_W0_SNUM_SHIFT           (24)      /* Bits 24-27: Semaphore Number (SNUM) */
#define XRDC_PDAC_W0_SNUM_MASK            (0x0f << XRDC_PDAC_W0_SNUM_SHIFT)
                                                    /* Bits 28-29: Reserved */
#define XRDC_PDAC_W0_SE                   (1 << 30) /* Bit 30: Semaphore Enable (SE) */
                                                    /* Bit 31: Reserved */

/* Peripheral Domain Access Control (PDAC_W1_n, n=32..315) */

                                                    /* Bits 0-28: Reserved */
#define XRDC_PDAC_W1_LK2_SHIFT            (29)      /* Bits 29-30: Lock (LK2) */
#define XRDC_PDAC_W1_LK2_MASK             (0x03 << XRDC_PDAC_W1_LK2_SHIFT)
#define XRDC_PDAC_W1_VLD                  (1 << 31) /* Bit 31: Valid (VLD) */

/* Memory Region Descriptor (MRGD_WD0_n, n=0..35) */

                                                    /* Bits 0-4: Reserved */
#define XRDC_MRGD_W0_SRTADDR_SHIFT        (5)       /* Bits 5-31: Start Address (SRTADDR) */
#define XRDC_MRGD_W0_SRTADDR_MASK         (0x07ffffff << XRDC_MRGD_W0_SRTADDR_SHIFT)

/* Memory Region Descriptor (MRGD_W1_n, n=0..35) */

                                                    /* Bits 0-4: Reserved */
#define XRDC_MRGD_W1_ENDADDR_SHIFT        (5)       /* Bits 5-31: End Address (ENDADDR) */
#define XRDC_MRGD_W1_ENDADDR_MASK         (0x07ffffff << XRDC_MRGD_W1_ENDADDR_SHIFT)

/* Memory Region Descriptor (MRGD_WD2_n, n=0..35) */

#define XRDC_MRGD_W2_D0ACP_SHIFT          (0)       /* Bits 0-2: Domain 0 Access Control Policy (D0ACP) */
#define XRDC_MRGD_W2_D0ACP_MASK           (0x07 << XRDC_MRGD_W2_D0ACP_SHIFT)
#define XRDC_MRGD_W2_D1ACP_SHIFT          (3)       /* Bits 3-5: Domain 1 Access Control Policy (D1ACP) */
#define XRDC_MRGD_W2_D1ACP_MASK           (0x07 << XRDC_MRGD_W2_D1ACP_SHIFT)
#define XRDC_MRGD_W2_D2ACP_SHIFT          (6)       /* Bits 6-8: Domain 2 Access Control Policy (D2ACP) */
#define XRDC_MRGD_W2_D2ACP_MASK           (0x07 << XRDC_MRGD_W2_D2ACP_SHIFT)
                                                    /* Bits 9-23: Reserved */
#define XRDC_MRGD_W2_SNUM_SHIFT           (24)      /* Bits 24-27: Semaphore Number (SNUM) */
#define XRDC_MRGD_W2_SNUM_MASK            (0x0f << XRDC_MRGD_W2_SNUM_SHIFT)
                                                    /* Bits 28-29: Reserved */
#define XRDC_MRGD_W2_SE                   (1 << 30) /* Bit 30: Semaphore Enable (SE) */
                                                    /* Bit 31: Reserved */

/* Memory Region Descriptor (MRGD_WD3_n, n=0..35) */

                                                    /* Bits 0-28: Reserved */
#define XRDC_MRGD_W3_LK2_SHIFT            (29)      /* Bits 29-30: Lock (LK2) */
#define XRDC_MRGD_W3_LK2_MASK             (0x03 << XRDC_MRGD_W3_LK2_SHIFT)
#define XRDC_MRGD_W3_VLD                  (1 << 31) /* Bit 31: Valid (VLD) */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_XRDC_H */
