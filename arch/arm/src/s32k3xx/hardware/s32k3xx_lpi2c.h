/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_lpi2c.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_LPI2C_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_LPI2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LPI2C Register Offsets ***************************************************/

#define S32K3XX_LPI2C_VERID_OFFSET          (0x0000) /* Version ID Register (VERID) */
#define S32K3XX_LPI2C_PARAM_OFFSET          (0x0004) /* Parameter Register (PARAM) */
#define S32K3XX_LPI2C_MCR_OFFSET            (0x0010) /* Master Control Register (MCR) */
#define S32K3XX_LPI2C_MSR_OFFSET            (0x0014) /* Master Status Register (MSR) */
#define S32K3XX_LPI2C_MIER_OFFSET           (0x0018) /* Master Interrupt Enable Register (MIER) */
#define S32K3XX_LPI2C_MDER_OFFSET           (0x001c) /* Master DMA Enable Register (MDER) */
#define S32K3XX_LPI2C_MCFGR0_OFFSET         (0x0020) /* Master Config Register 0 (MCFGR0) */
#define S32K3XX_LPI2C_MCFGR1_OFFSET         (0x0024) /* Master Config Register 1 (MCFGR1) */
#define S32K3XX_LPI2C_MCFGR2_OFFSET         (0x0028) /* Master Config Register 2 (MCFGR2) */
#define S32K3XX_LPI2C_MCFGR3_OFFSET         (0x002c) /* Master Config Register 3 (MCFGR3) */
#define S32K3XX_LPI2C_MDMR_OFFSET           (0x0040) /* Master Data Match Register (MDMR) */
#define S32K3XX_LPI2C_MCCR0_OFFSET          (0x0048) /* Master Clock Configuration Register 0 (MCCR0) */
#define S32K3XX_LPI2C_MCCR1_OFFSET          (0x0050) /* Master Clock Configuration Register 1 (MCCR1) */
#define S32K3XX_LPI2C_MFCR_OFFSET           (0x0058) /* Master FIFO Control Register (MFCR) */
#define S32K3XX_LPI2C_MFSR_OFFSET           (0x005c) /* Master FIFO Status Register (MFSR) */
#define S32K3XX_LPI2C_MTDR_OFFSET           (0x0060) /* Master Transmit Data Register (MTDR) */
#define S32K3XX_LPI2C_MRDR_OFFSET           (0x0070) /* Master Receive Data Register (MRDR) */
#define S32K3XX_LPI2C_SCR_OFFSET            (0x0110) /* Slave Control Register (SCR) */
#define S32K3XX_LPI2C_SSR_OFFSET            (0x0114) /* Slave Status Register (SSR) */
#define S32K3XX_LPI2C_SIER_OFFSET           (0x0118) /* Slave Interrupt Enable Register (SIER) */
#define S32K3XX_LPI2C_SDER_OFFSET           (0x011c) /* Slave DMA Enable Register (SDER) */
#define S32K3XX_LPI2C_SCFGR1_OFFSET         (0x0124) /* Slave Config Register 1 (SCFGR1) */
#define S32K3XX_LPI2C_SCFGR2_OFFSET         (0x0128) /* Slave Config Register 2 (SCFGR2) */
#define S32K3XX_LPI2C_SAMR_OFFSET           (0x0140) /* Slave Address Match Register (SAMR) */
#define S32K3XX_LPI2C_SASR_OFFSET           (0x0150) /* Slave Address Status Register (SASR) */
#define S32K3XX_LPI2C_STAR_OFFSET           (0x0154) /* Slave Transmit ACK Register (STAR) */
#define S32K3XX_LPI2C_STDR_OFFSET           (0x0160) /* Slave Transmit Data Register (STDR) */
#define S32K3XX_LPI2C_SRDR_OFFSET           (0x0170) /* Slave Receive Data Register (SRDR) */

/* LPI2C Register Addresses *************************************************/

#define S32K3XX_LPI2C0_VERID                (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_VERID_OFFSET)
#define S32K3XX_LPI2C0_PARAM                (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_PARAM_OFFSET)
#define S32K3XX_LPI2C0_MCR                  (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MCR_OFFSET)
#define S32K3XX_LPI2C0_MSR                  (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MSR_OFFSET)
#define S32K3XX_LPI2C0_MIER                 (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MIER_OFFSET)
#define S32K3XX_LPI2C0_MDER                 (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MDER_OFFSET)
#define S32K3XX_LPI2C0_MCFGR0               (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MCFGR0_OFFSET)
#define S32K3XX_LPI2C0_MCFGR1               (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MCFGR1_OFFSET)
#define S32K3XX_LPI2C0_MCFGR2               (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MCFGR2_OFFSET)
#define S32K3XX_LPI2C0_MCFGR3               (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MCFGR3_OFFSET)
#define S32K3XX_LPI2C0_MDMR                 (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MDMR_OFFSET)
#define S32K3XX_LPI2C0_MCCR0                (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MCCR0_OFFSET)
#define S32K3XX_LPI2C0_MCCR1                (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MCCR1_OFFSET)
#define S32K3XX_LPI2C0_MFCR                 (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MFCR_OFFSET)
#define S32K3XX_LPI2C0_MFSR                 (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MFSR_OFFSET)
#define S32K3XX_LPI2C0_MTDR                 (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MTDR_OFFSET)
#define S32K3XX_LPI2C0_MRDR                 (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_MRDR_OFFSET)
#define S32K3XX_LPI2C0_SCR                  (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_SCR_OFFSET)
#define S32K3XX_LPI2C0_SSR                  (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_SSR_OFFSET)
#define S32K3XX_LPI2C0_SIER                 (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_SIER_OFFSET)
#define S32K3XX_LPI2C0_SDER                 (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_SDER_OFFSET)
#define S32K3XX_LPI2C0_SCFGR1               (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_SCFGR1_OFFSET)
#define S32K3XX_LPI2C0_SCFGR2               (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_SCFGR2_OFFSET)
#define S32K3XX_LPI2C0_SAMR                 (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_SAMR_OFFSET)
#define S32K3XX_LPI2C0_SASR                 (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_SASR_OFFSET)
#define S32K3XX_LPI2C0_STAR                 (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_STAR_OFFSET)
#define S32K3XX_LPI2C0_STDR                 (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_STDR_OFFSET)
#define S32K3XX_LPI2C0_SRDR                 (S32K3XX_LPI2C0_BASE + S32K3XX_LPI2C_SRDR_OFFSET)

#define S32K3XX_LPI2C1_VERID                (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_VERID_OFFSET)
#define S32K3XX_LPI2C1_PARAM                (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_PARAM_OFFSET)
#define S32K3XX_LPI2C1_MCR                  (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MCR_OFFSET)
#define S32K3XX_LPI2C1_MSR                  (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MSR_OFFSET)
#define S32K3XX_LPI2C1_MIER                 (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MIER_OFFSET)
#define S32K3XX_LPI2C1_MDER                 (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MDER_OFFSET)
#define S32K3XX_LPI2C1_MCFGR0               (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MCFGR0_OFFSET)
#define S32K3XX_LPI2C1_MCFGR1               (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MCFGR1_OFFSET)
#define S32K3XX_LPI2C1_MCFGR2               (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MCFGR2_OFFSET)
#define S32K3XX_LPI2C1_MCFGR3               (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MCFGR3_OFFSET)
#define S32K3XX_LPI2C1_MDMR                 (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MDMR_OFFSET)
#define S32K3XX_LPI2C1_MCCR0                (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MCCR0_OFFSET)
#define S32K3XX_LPI2C1_MCCR1                (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MCCR1_OFFSET)
#define S32K3XX_LPI2C1_MFCR                 (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MFCR_OFFSET)
#define S32K3XX_LPI2C1_MFSR                 (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MFSR_OFFSET)
#define S32K3XX_LPI2C1_MTDR                 (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MTDR_OFFSET)
#define S32K3XX_LPI2C1_MRDR                 (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_MRDR_OFFSET)
#define S32K3XX_LPI2C1_SCR                  (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_SCR_OFFSET)
#define S32K3XX_LPI2C1_SSR                  (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_SSR_OFFSET)
#define S32K3XX_LPI2C1_SIER                 (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_SIER_OFFSET)
#define S32K3XX_LPI2C1_SDER                 (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_SDER_OFFSET)
#define S32K3XX_LPI2C1_SCFGR1               (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_SCFGR1_OFFSET)
#define S32K3XX_LPI2C1_SCFGR2               (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_SCFGR2_OFFSET)
#define S32K3XX_LPI2C1_SAMR                 (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_SAMR_OFFSET)
#define S32K3XX_LPI2C1_SASR                 (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_SASR_OFFSET)
#define S32K3XX_LPI2C1_STAR                 (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_STAR_OFFSET)
#define S32K3XX_LPI2C1_STDR                 (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_STDR_OFFSET)
#define S32K3XX_LPI2C1_SRDR                 (S32K3XX_LPI2C1_BASE + S32K3XX_LPI2C_SRDR_OFFSET)

/* LPI2C Register Bitfield Definitions **************************************/

/* Version ID Register (VERID) */

#define LPI2C_VERID_FEATURE_SHIFT         (0)       /* Bits 0-15: Feature Specification Number (FEATURE) */
#define LPI2C_VERID_FEATURE_MASK          (0xffff << LPI2C_VERID_FEATURE_SHIFT)
#  define LPI2C_VERID_FEATURE_MSTRONL     (0x0002 << LPI2C_VERID_FEATURE_SHIFT) /* Master only, with standard feature set */
#  define LPI2C_VERID_FEATURE_MSTRSLV     (0x0003 << LPI2C_VERID_FEATURE_SHIFT) /* Master and slave, with standard feature set */

#define LPI2C_VERID_MINOR_SHIFT           (16)      /* Bits 16-23: Minor Version Number (MINOR) */
#define LPI2C_VERID_MINOR_MASK            (0xff << LPI2C_VERID_MINOR_SHIFT)
#define LPI2C_VERID_MAJOR_SHIFT           (24)      /* Bits 24-31: Major Version Number (MAJOR) */
#define LPI2C_VERID_MAJOR_MASK            (0xff << LPI2C_VERID_MAJOR_SHIFT)

/* Parameter Register (PARAM) */

#define LPI2C_PARAM_MTXFIFO_SHIFT         (0)       /* Bits 0-3: Master Transmit FIFO Size (MTXFIFO) */
#define LPI2C_PARAM_MTXFIFO_MASK          (0x0f << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_1_WORDS     (0x00 << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_2_WORDS     (0x01 << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_4_WORDS     (0x02 << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_8_WORDS     (0x03 << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_16_WORDS    (0x04 << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_32_WORDS    (0x05 << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_64_WORDS    (0x06 << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_128_WORDS   (0x07 << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_256_WORDS   (0x08 << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_512_WORDS   (0x09 << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_1024_WORDS  (0x0a << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_2048_WORDS  (0x0b << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_4096_WORDS  (0x0c << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_8192_WORDS  (0x0d << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_16384_WORDS (0x0e << LPI2C_PARAM_MTXFIFO_SHIFT)
#  define LPI2C_PARAM_MTXFIFO_32768_WORDS (0x0f << LPI2C_PARAM_MTXFIFO_SHIFT)
                                                    /* Bits 4-7: Reserved */
#define LPI2C_PARAM_MRXFIFO_SHIFT         (8)       /* Bits 8-11: Master Receive FIFO Size (MRXFIFO) */
#define LPI2C_PARAM_MRXFIFO_MASK          (0x0f << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_1_WORDS     (0x00 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_2_WORDS     (0x01 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_4_WORDS     (0x02 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_8_WORDS     (0x03 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_16_WORDS    (0x04 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_32_WORDS    (0x05 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_64_WORDS    (0x06 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_128_WORDS   (0x07 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_256_WORDS   (0x08 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_512_WORDS   (0x09 << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_1024_WORDS  (0x0a << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_2048_WORDS  (0x0b << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_4096_WORDS  (0x0c << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_8192_WORDS  (0x0d << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_16384_WORDS (0x0e << LPI2C_PARAM_MRXFIFO_SHIFT)
#  define LPI2C_PARAM_MRXFIFO_32768_WORDS (0x0f << LPI2C_PARAM_MRXFIFO_SHIFT)
                                                    /* Bits 12-31: Reserved */

/* Master Control Register (MCR) */

#define LPI2C_MCR_MEN                     (1 << 0)  /* Bit 0: Master Enable (MEN) */
#define LPI2C_MCR_RST                     (1 << 1)  /* Bit 1: Software Reset (RST) */
#define LPI2C_MCR_DOZEN                   (1 << 2)  /* Bit 2: Doze Mode Enable (DOZEN) */
#define LPI2C_MCR_DBGEN                   (1 << 3)  /* Bit 3: Debug Enable (DBGEN) */
                                                    /* Bits 4-7: Reserved */
#define LPI2C_MCR_RTF                     (1 << 8)  /* Bit 8: Reset Transmit FIFO (RTF) */
#define LPI2C_MCR_RRF                     (1 << 9)  /* Bit 9: Reset Receive FIFO (RRF) */
                                                    /* Bits 10-31: Reserved */

/* Master Status Register (MSR) */

#define LPI2C_MSR_TDF                     (1 << 0)  /* Bit 0: Transmit Data Flag (TDF) */
#define LPI2C_MSR_RDF                     (1 << 1)  /* Bit 1: Receive Data Flag (RDF) */
                                                    /* Bits 2-7: Reserved */
#define LPI2C_MSR_EPF                     (1 << 8)  /* Bit 8: End Packet Flag (EPF) */
#define LPI2C_MSR_SDF                     (1 << 9)  /* Bit 9: STOP Detect Flag (SDF) */
#define LPI2C_MSR_NDF                     (1 << 10) /* Bit 10: NACK Detect Flag (NDF) */
#define LPI2C_MSR_ALF                     (1 << 11) /* Bit 11: Arbitration Lost Flag (ALF) */
#define LPI2C_MSR_FEF                     (1 << 12) /* Bit 12: FIFO Error Flag (FEF) */
#define LPI2C_MSR_PLTF                    (1 << 13) /* Bit 13: Pin Low Timeout Flag (PLTF) */
#define LPI2C_MSR_DMF                     (1 << 14) /* Bit 14: Data Match Flag (DMF) */
                                                    /* Bits 15-23: Reserved */
#define LPI2C_MSR_MBF                     (1 << 24) /* Bit 24: Master Busy Flag (MBF) */
#define LPI2C_MSR_BBF                     (1 << 25) /* Bit 25: Bus Busy Flag (BBF) */
                                                    /* Bits 26-31: Reserved */

#define LPI2C_MSR_ERROR_MASK              (LPI2C_MSR_NDF | LPI2C_MSR_ALF | LPI2C_MSR_FEF)

/* Master Interrupt Enable Register (MIER) */

#define LPI2C_MIER_TDIE                   (1 << 0)  /* Bit 0: Transmit Data Interrupt Enable (TDIE) */
#define LPI2C_MIER_RDIE                   (1 << 1)  /* Bit 1: Receive Data Interrupt Enable (RDIE) */
                                                    /* Bits 2-7: Reserved */
#define LPI2C_MIER_EPIE                   (1 << 8)  /* Bit 8: End Packet Interrupt Enable (EPIE) */
#define LPI2C_MIER_SDIE                   (1 << 9)  /* Bit 9: STOP Detect Interrupt Enable (SDIE) */
#define LPI2C_MIER_NDIE                   (1 << 10) /* Bit 10: NACK Detect Interrupt Enable (NDIE) */
#define LPI2C_MIER_ALIE                   (1 << 11) /* Bit 11: Arbitration Lost Interrupt Enable (ALIE) */
#define LPI2C_MIER_FEIE                   (1 << 12) /* Bit 12: FIFO Error Interrupt Enable (FEIE) */
#define LPI2C_MIER_PLTIE                  (1 << 13) /* Bit 13: Pin Low Timeout Interrupt Enable (PLTIE) */
#define LPI2C_MIER_DMIE                   (1 << 14) /* Bit 14: Data Match Interrupt Enable (DMIE) */
                                                    /* Bits 15-31: Reserved */

/* Master DMA Enable Register (MDER) */

#define LPI2C_MDER_TDDE                   (1 << 0)  /* Bit 0: Transmit Data DMA Enable (TDDE) */
#define LPI2C_MDER_RDDE                   (1 << 1)  /* Bit 1: Receive Data DMA Enable (RDDE) */
                                                    /* Bits 2-31: Reserved */

/* Master Config Register 0 (MCFGR0) */

#define LPI2C_MCFGR0_HREN                 (1 << 0)  /* Bit 0: Host Request Enable (HREN) */
#define LPI2C_MCFGR0_HRPOL                (1 << 1)  /* Bit 1: Host Request Polarity (HRPOL) */
#define LPI2C_MCFGR0_HRSEL                (1 << 2)  /* Bit 2: Host Request Select (HRSEL) */
                                                    /* Bits 3-7: Reserved */
#define LPI2C_MCFGR0_CIRFIFO              (1 << 8)  /* Bit 8: Circular FIFO Enable (CIRFIFO) */
#define LPI2C_MCFGR0_RDMO                 (1 << 9)  /* Bit 9: Receive Data Match Only (RDMO) */
                                                    /* Bits 10-31: Reserved */

/* Master Config Register 1 (MCFGR1) */

#define LPI2C_MCFGR1_PRESCALE_SHIFT       (0)       /* Bits 0-2: Prescaler (PRESCALE) */
#define LPI2C_MCFGR1_PRESCALE_MASK        (0x07 << LPI2C_MCFGR1_PRESCALE_SHIFT)
#  define LPI2C_MCFGR1_PRESCALE(n)        (((n) << LPI2C_MCFGR1_PRESCALE_SHIFT) & LPI2C_MCFGR1_PRESCALE_MASK)
#  define LPI2C_MCFGR1_PRESCALE_1         (0x00 << LPI2C_MCFGR1_PRESCALE_SHIFT) /* Divide by 1 */
#  define LPI2C_MCFGR1_PRESCALE_2         (0x01 << LPI2C_MCFGR1_PRESCALE_SHIFT) /* Divide by 2 */
#  define LPI2C_MCFGR1_PRESCALE_4         (0x02 << LPI2C_MCFGR1_PRESCALE_SHIFT) /* Divide by 4 */
#  define LPI2C_MCFGR1_PRESCALE_8         (0x03 << LPI2C_MCFGR1_PRESCALE_SHIFT) /* Divide by 8 */
#  define LPI2C_MCFGR1_PRESCALE_16        (0x04 << LPI2C_MCFGR1_PRESCALE_SHIFT) /* Divide by 16 */
#  define LPI2C_MCFGR1_PRESCALE_32        (0x05 << LPI2C_MCFGR1_PRESCALE_SHIFT) /* Divide by 32 */
#  define LPI2C_MCFGR1_PRESCALE_64        (0x06 << LPI2C_MCFGR1_PRESCALE_SHIFT) /* Divide by 64 */
#  define LPI2C_MCFGR1_PRESCALE_128       (0x07 << LPI2C_MCFGR1_PRESCALE_SHIFT) /* Divide by 128 */

                                                    /* Bits 3-7: Reserved */
#define LPI2C_MCFGR1_AUTOSTOP             (1 << 8)  /* Bit 8: Automatic STOP Generation (AUTOSTOP) */
#define LPI2C_MCFGR1_IGNACK               (1 << 9)  /* Bit 9: Ignore NACK (IGNACK) */
#define LPI2C_MCFGR1_TIMECFG              (1 << 10) /* Bit 10: Timeout Configuration (TIMECFG) */
                                                    /* Bits 11-15: Reserved */
#define LPI2C_MCFGR1_MATCFG_SHIFT         (16)      /* Bits 16-18: Match Configuration (MATCFG) */
#define LPI2C_MCFGR1_MATCFG_MASK          (0x07 << LPI2C_MCFGR1_MATCFG_SHIFT)
#  define LPI2C_MCFGR1_MATCFG(n)          (((n) << LPI2C_MCFGR1_MATCFG_SHIFT) & LPI2C_MCFGR1_MATCFG_MASK)
#  define LPI2C_MCFGR1_MATCFG_DISABLE     (0x00 << LPI2C_MCFGR1_MATCFG_SHIFT) /* Match is disabled */
#  define LPI2C_MCFGR1_MATCFG2            (0x02 << LPI2C_MCFGR1_MATCFG_SHIFT) /* Match is enabled (1st data word equals MDMR[MATCH0] OR MDMR[MATCH1]) */
#  define LPI2C_MCFGR1_MATCFG3            (0x03 << LPI2C_MCFGR1_MATCFG_SHIFT) /* Match is enabled (any data word equals MDMR[MATCH0] OR MDMR[MATCH1]) */
#  define LPI2C_MCFGR1_MATCFG4            (0x04 << LPI2C_MCFGR1_MATCFG_SHIFT) /* Match is enabled (1st data word equals MDMR[MATCH0] AND 2nd data word equals MDMR[MATCH1]) */
#  define LPI2C_MCFGR1_MATCFG5            (0x05 << LPI2C_MCFGR1_MATCFG_SHIFT) /* Match is enabled (any data word equals MDMR[MATCH0] AND next data word equals MDMR[MATCH1]) */
#  define LPI2C_MCFGR1_MATCFG6            (0x06 << LPI2C_MCFGR1_MATCFG_SHIFT) /* Match is enabled (1st data word AND MDMR[MATCH1] equals MDMR[MATCH0] AND MDMR[MATCH1]) */
#  define LPI2C_MCFGR1_MATCFG7            (0x07 << LPI2C_MCFGR1_MATCFG_SHIFT) /* Match is enabled (any data word AND MDMR[MATCH1] equals MDMR[MATCH0] AND MDMR[MATCH1]) */

                                                    /* Bits 19-23: Reserved */
#define LPI2C_MCFGR1_PINCFG_SHIFT         (24)      /* Bits 24-26: Pin Configuration (PINCFG) */
#define LPI2C_MCFGR1_PINCFG_MASK          (0x07 << LPI2C_MCFGR1_PINCFG_SHIFT)
#  define LPI2C_MCFGR1_PINCFG(n)          (((n) << LPI2C_MCFGR1_PINCFG_SHIFT) & LPI2C_MCFGR1_PINCFG_MASK)
#  define LPI2C_MCFGR1_PINCFG0            (0x00 << LPI2C_MCFGR1_PINCFG_SHIFT) /* SCL/SDA: Bi-directional open drain for master and slave */
#  define LPI2C_MCFGR1_PINCFG1            (0x01 << LPI2C_MCFGR1_PINCFG_SHIFT) /* SCL/SDA: Output-only (ultra-fast mode) open drain for master and slave */
#  define LPI2C_MCFGR1_PINCFG2            (0x02 << LPI2C_MCFGR1_PINCFG_SHIFT) /* SCL/SDA: Bi-directional push-pull for master and slave */
#  define LPI2C_MCFGR1_PINCFG3            (0x03 << LPI2C_MCFGR1_PINCFG_SHIFT) /* SCL/SDA: Input only for master and slave, SCLS/SDAS: Output-only push-pull for master and slave */
#  define LPI2C_MCFGR1_PINCFG4            (0x04 << LPI2C_MCFGR1_PINCFG_SHIFT) /* SCL/SDA: Bi-directional open drain for master, SCLS/SDAS: Bi-directional open drain for slave */
#  define LPI2C_MCFGR1_PINCFG5            (0x05 << LPI2C_MCFGR1_PINCFG_SHIFT) /* SCL/SDA: Output-only (ultra-fast mode) open drain for master, SCLS/SDAS: Output-only for slave */
#  define LPI2C_MCFGR1_PINCFG6            (0x06 << LPI2C_MCFGR1_PINCFG_SHIFT) /* SCL/SDA: Bi-directional push-pull for master, SCLS/SDAS: Bi-directional push-pull for slave */
#  define LPI2C_MCFGR1_PINCFG7            (0x07 << LPI2C_MCFGR1_PINCFG_SHIFT) /* SCL/SDA: Input only for master and slave, SCLS/SDAS: Inverted output-only push-pull for master and slave */

                                                    /* Bits 27-31: Reserved */

/* Master Config Register 2 (MCFGR2) */

#define LPI2C_MCFGR2_BUSIDLE_SHIFT        (0)       /* Bits 0-11: Bus Idle Timeout (BUSIDLE) */
#define LPI2C_MCFGR2_BUSIDLE_MASK         (0x0fff << LPI2C_MCFGR2_BUSIDLE_SHIFT)
#define LPI2C_MCFGR2_BUSIDLE_DISABLE      (0x0000 << LPI2C_MCFGR2_BUSIDLE_SHIFT)
#  define LPI2C_MCFGR2_BUSIDLE(n)         (((n) << LPI2C_MCFGR2_BUSIDLE_SHIFT) & LPI2C_MCFGR2_BUSIDLE_MASK)
                                                    /* Bits 12-15: Reserved */
#define LPI2C_MCFGR2_FILTSCL_SHIFT        (16)      /* Bits 16-19: Glitch Filter SCL (FILTSCL) */
#define LPI2C_MCFGR2_FILTSCL_MASK         (0x0f << LPI2C_MCFGR2_FILTSCL_SHIFT)
#define LPI2C_MCFGR2_FILTSCL_DISABLE      (0x00 << LPI2C_MCFGR2_FILTSCL_SHIFT)
#  define LPI2C_MCFGR2_FILTSCL_CYCLES(n)  (((n) << LPI2C_MCFGR2_FILTSCL_SHIFT) & LPI2C_MCFGR2_FILTSCL_MASK)
                                                    /* Bits 20-23: Reserved */
#define LPI2C_MCFGR2_FILTSDA_SHIFT        (24)      /* Bits 24-27: Glitch Filter SDA (FILTSDA) */
#define LPI2C_MCFGR2_FILTSDA_MASK         (0x0f << LPI2C_MCFGR2_FILTSDA_SHIFT)
#define LPI2C_MCFGR2_FILTSDA_DISABLE      (0x00 << LPI2C_MCFGR2_FILTSDA_SHIFT)
#  define LPI2C_MCFGR2_FILTSDA_CYCLES(n)  (((n) << LPI2C_MCFGR2_FILTSDA_SHIFT) & LPI2C_MCFGR2_FILTSDA_MASK)
                                                    /* Bits 28-31: Reserved */

/* Master Config Register 3 (MCFGR3) */

                                                    /* Bits 0-7: Reserved */
#define LPI2C_MCFGR3_PINLOW_SHIFT         (8)       /* Bits 8-19: Pin Low Timeout (PINLOW) */
#define LPI2C_MCFGR3_PINLOW_MASK          (0x0fff << LPI2C_MCFGR3_PINLOW_SHIFT)
#  define LPI2C_MCFGR3_PINLOW_CYCLES(n)   (((n) << LPI2C_MCFGR3_PINLOW_SHIFT) & LPI2C_MCFGR3_PINLOW_MASK)
                                                    /* Bits 20-31: Reserved */

/* Master Data Match Register (MDMR) */

#define LPI2C_MDMR_MATCH0_SHIFT           (0)       /* Bits 0-7: Match 0 Value (MATCH0) */
#define LPI2C_MDMR_MATCH0_MASK            (0xff << LPI2C_MDMR_MATCH0_SHIFT)
#  define LPI2C_MDMR_MATCH0(n)            (((n) << LPI2C_MDMR_MATCH0_SHIFT) & LPI2C_MDMR_MATCH0_MASK)
                                                    /* Bits 8-15: Reserved */
#define LPI2C_MDMR_MATCH1_SHIFT           (16)      /* Bits 16-23: Match 1 Value (MATCH1) */
#define LPI2C_MDMR_MATCH1_MASK            (0xff << LPI2C_MDMR_MATCH1_SHIFT)
#  define LPI2C_MDMR_MATCH1(n)            (((n) << LPI2C_MDMR_MATCH1_SHIFT) & LPI2C_MDMR_MATCH1_MASK)
                                                    /* Bits 24-31: Reserved */

/* Master Clock Configuration Register 0 (MCCR0) */

#define LPI2C_MCCR0_CLKLO_SHIFT           (0)       /* Bits 0-5: Clock Low Period (CLKLO) */
#define LPI2C_MCCR0_CLKLO_MASK            (0x3f << LPI2C_MCCR0_CLKLO_SHIFT)
#  define LPI2C_MCCR0_CLKLO(n)            (((n) << LPI2C_MCCR0_CLKLO_SHIFT) & LPI2C_MCCR0_CLKLO_MASK)
                                                    /* Bits 6-7: Reserved */
#define LPI2C_MCCR0_CLKHI_SHIFT           (8)       /* Bits 8-13: Clock High Period (CLKHI) */
#define LPI2C_MCCR0_CLKHI_MASK            (0x3f << LPI2C_MCCR0_CLKHI_SHIFT)
#  define LPI2C_MCCR0_CLKHI(n)            (((n) << LPI2C_MCCR0_CLKHI_SHIFT) & LPI2C_MCCR0_CLKHI_MASK)
                                                    /* Bits 14-15: Reserved */
#define LPI2C_MCCR0_SETHOLD_SHIFT         (16)      /* Bits 16-21: Setup Hold Delay (SETHOLD) */
#define LPI2C_MCCR0_SETHOLD_MASK          (0x3f << LPI2C_MCCR0_SETHOLD_SHIFT)
#  define LPI2C_MCCR0_SETHOLD(n)          (((n) << LPI2C_MCCR0_SETHOLD_SHIFT) & LPI2C_MCCR0_SETHOLD_MASK)
                                                    /* Bits 22-23: Reserved */
#define LPI2C_MCCR0_DATAVD_SHIFT          (24)      /* Bits 24-29: Data Valid Delay (DATAVD) */
#define LPI2C_MCCR0_DATAVD_MASK           (0x3f << LPI2C_MCCR0_DATAVD_SHIFT)
#  define LPI2C_MCCR0_DATAVD(n)           (((n) << LPI2C_MCCR0_DATAVD_SHIFT) & LPI2C_MCCR0_DATAVD_MASK)
                                                    /* Bits 30-31: Reserved */

/* Master Clock Configuration Register 1 (MCCR1) */

#define LPI2C_MCCR1_CLKLO_SHIFT           (0)       /* Bits 0-5: Clock Low Period (CLKLO) */
#define LPI2C_MCCR1_CLKLO_MASK            (0x3f << LPI2C_MCCR1_CLKLO_SHIFT)
#  define LPI2C_MCCR1_CLKLO(n)            (((n) << LPI2C_MCCR1_CLKLO_SHIFT) & LPI2C_MCCR1_CLKLO_MASK)
                                                    /* Bits 6-7: Reserved */
#define LPI2C_MCCR1_CLKHI_SHIFT           (8)       /* Bits 8-13: Clock High Period (CLKHI) */
#define LPI2C_MCCR1_CLKHI_MASK            (0x3f << LPI2C_MCCR1_CLKHI_SHIFT)
#  define LPI2C_MCCR1_CLKHI(n)            (((n) << LPI2C_MCCR1_CLKHI_SHIFT) & LPI2C_MCCR1_CLKHI_MASK)
                                                    /* Bits 14-15: Reserved */
#define LPI2C_MCCR1_SETHOLD_SHIFT         (16)      /* Bits 16-21: Setup Hold Delay (SETHOLD) */
#define LPI2C_MCCR1_SETHOLD_MASK          (0x3f << LPI2C_MCCR1_SETHOLD_SHIFT)
#  define LPI2C_MCCR1_SETHOLD(n)          (((n) << LPI2C_MCCR1_SETHOLD_SHIFT) & LPI2C_MCCR1_SETHOLD_MASK)
                                                    /* Bits 22-23: Reserved */
#define LPI2C_MCCR1_DATAVD_SHIFT          (24)      /* Bits 24-29: Data Valid Delay (DATAVD) */
#define LPI2C_MCCR1_DATAVD_MASK           (0x3f << LPI2C_MCCR1_DATAVD_SHIFT)
#  define LPI2C_MCCR1_DATAVD(n)           (((n) << LPI2C_MCCR1_DATAVD_SHIFT) & LPI2C_MCCR1_DATAVD_MASK)
                                                    /* Bits 30-31: Reserved */

/* Master FIFO Control Register (MFCR) */

#define LPI2C_MFCR_TXWATER_SHIFT          (0)       /* Bits 0-1: Transmit FIFO Watermark (TXWATER) */
#define LPI2C_MFCR_TXWATER_MASK           (0x03 << LPI2C_MFCR_TXWATER_SHIFT)
#  define LPI2C_MFCR_TXWATER(n)           (((n) << LPI2C_MFCR_TXWATER_SHIFT) &  LPI2C_MFCR_TXWATER_MASK)
                                                    /* Bits 2-15: Reserved */
#define LPI2C_MFCR_RXWATER_SHIFT          (16)      /* Bits 16-17: Receive FIFO Watermark (RXWATER) */
#define LPI2C_MFCR_RXWATER_MASK           (0x03 << LPI2C_MFCR_RXWATER_SHIFT)
#  define LPI2C_MFCR_RXWATER(n)           (((n) << LPI2C_MFCR_RXWATER_SHIFT) &  LPI2C_MFCR_RXWATER_MASK)
                                                    /* Bits 18-31: Reserved */

/* Master FIFO Status Register (MFSR) */

#define LPI2C_MFSR_TXCOUNT_SHIFT          (0)       /* Bits 0-2: Transmit FIFO Count (TXCOUNT) */
#define LPI2C_MFSR_TXCOUNT_MASK           (0x07 << LPI2C_MFSR_TXCOUNT_SHIFT)
                                                    /* Bits 15-2 Reserved */
#define LPI2C_MFSR_RXCOUNT_SHIFT          (16)      /* Bits 16-18: Receive FIFO Count (RXCOUNT) */
#define LPI2C_MFSR_RXCOUNT_MASK           (0x07 << LPI2C_MFSR_RXCOUNT_SHIFT)
                                                    /* Bits 18-31: Reserved */

/* Master Transmit Data Register (MTDR) */

#define LPI2C_MTDR_DATA_SHIFT             (0)       /* Bits 0-7: Transmit Data (DATA) */
#define LPI2C_MTDR_DATA_MASK              (0xff << LPI2C_MTDR_DATA_SHIFT)
#  define LPI2C_MTDR_DATA(n)              (((n) << LPI2C_MTDR_DATA_SHIFT) & LPI2C_MTDR_DATA_MASK)
#define LPI2C_MTDR_CMD_SHIFT              (8)       /* Bits 8-10: Command Data (CMD) */
#define LPI2C_MTDR_CMD_MASK               (0x07 << LPI2C_MTDR_CMD_SHIFT)
#  define LPI2C_MTDR_CMD(n)               (((n) << LPI2C_MTDR_CMD_SHIFT) & LPI2C_MTDR_CMD_MASK)
#  define LPI2C_MTDR_CMD_TXD              (0x00 << LPI2C_MTDR_CMD_SHIFT) /* Transmit DATA[7:0] */
#  define LPI2C_MTDR_CMD_RXD              (0x01 << LPI2C_MTDR_CMD_SHIFT) /* Receive (DATA[7:0] + 1) bytes */
#  define LPI2C_MTDR_CMD_STOP             (0x02 << LPI2C_MTDR_CMD_SHIFT) /* Generate STOP condition */
#  define LPI2C_MTDR_CMD_RXD_DISC         (0x03 << LPI2C_MTDR_CMD_SHIFT) /* Receive and discard (DATA[7:0] + 1) bytes */
#  define LPI2C_MTDR_CMD_START            (0x04 << LPI2C_MTDR_CMD_SHIFT) /* Generate (repeated) START and transmit address in DATA[7:0] */
#  define LPI2C_MTDR_CMD_START_NACK       (0x05 << LPI2C_MTDR_CMD_SHIFT) /* Generate (repeated) START and transmit address in DATA[7:0]. This transfer expects a NACK to be returned. */
#  define LPI2C_MTDR_CMD_START_HI         (0x06 << LPI2C_MTDR_CMD_SHIFT) /* Generate (repeated) START and transmit address in DATA[7:0] using high speed mode */
#  define LPI2C_MTDR_CMD_START_HI_NACK    (0x07 << LPI2C_MTDR_CMD_SHIFT) /* Generate (repeated) START and transmit address in DATA[7:0] using high speed mode. This transfer expects a NACK to be returned.*/

                                                    /* Bits 11-31: Reserved */

/* Master Receive Data Register (MRDR) */

#define LPI2C_MRDR_DATA_SHIFT             (0)       /* Bits 0-7: Receive Data (DATA) */
#define LPI2C_MRDR_DATA_MASK              (0xff << LPI2C_MRDR_DATA_SHIFT)
                                                    /* Bits 8-13: Reserved */
#define LPI2C_MRDR_RXEMPTY                (1 << 14) /* Bit 14: RX Empty (RXEMPTY) */
                                                    /* Bits 15-31: Reserved */

/* Slave Control Register (SCR) */

#define LPI2C_SCR_SEN                     (1 << 0)  /* Bit 0: Slave Enable (SEN) */
#define LPI2C_SCR_RST                     (1 << 1)  /* Bit 1: Software Reset (RST) */
                                                    /* Bits 2-3: Reserved */
#define LPI2C_SCR_FILTEN                  (1 << 4)  /* Bit 4: Filter Enable (FILTEN) */
#define LPI2C_SCR_FILTDZ                  (1 << 5)  /* Bit 5: Filter Doze Enable (FILTDZ) */
                                                    /* Bits 6-7: Reserved */
#define LPI2C_SCR_RTF                     (1 << 8)  /* Bit 8: Reset Transmit FIFO (RTF) */
#define LPI2C_SCR_RRF                     (1 << 9)  /* Bit 9: Reset Receive FIFO (RRF) */
                                                    /* Bits 10-31: Reserved */

/* Slave Status Register (SSR) */

#define LPI2C_SSR_TDF                     (1 << 0)  /* Bit 0: Transmit Data Flag (TDF) */
#define LPI2C_SSR_RDF                     (1 << 1)  /* Bit 1: Receive Data Flag (RDF) */
#define LPI2C_SSR_AVF                     (1 << 2)  /* Bit 2: Address Valid Flag (AVF) */
#define LPI2C_SSR_TAF                     (1 << 3)  /* Bit 3: Transmit ACK Flag (TAF) */
                                                    /* Bits 4-7: Reserved */
#define LPI2C_SSR_RSF                     (1 << 8)  /* Bit 8: Repeated Start Flag (RSF) */
#define LPI2C_SSR_SDF                     (1 << 9)  /* Bit 9: STOP Detect Flag (SDF) */
#define LPI2C_SSR_BEF                     (1 << 10) /* Bit 10: Bit Error Flag (BEF) */
#define LPI2C_SSR_FEF                     (1 << 11) /* Bit 11: FIFO Error Flag (FEF) */
#define LPI2C_SSR_AM0F                    (1 << 12) /* Bit 12: Address Match 0 Flag (AM0F) */
#define LPI2C_SSR_AM1F                    (1 << 13) /* Bit 13: Address Match 1 Flag (AM1F) */
#define LPI2C_SSR_GCF                     (1 << 14) /* Bit 14: General Call Flag (GCF) */
#define LPI2C_SSR_SARF                    (1 << 15) /* Bit 15: SMBus Alert Response Flag (SARF) */
                                                    /* Bits 16-23: Reserved */
#define LPI2C_MSR_SBF                     (1 << 24) /* Bit 24: Slave Busy Flag (SBF) */
#define LPI2C_MSR_BBF                     (1 << 25) /* Bit 25: Bus Busy Flag (BBF) */
                                                    /* Bits 26-31: Reserved */

/* Slave Interrupt Enable Register (SIER) */

#define LPI2C_SIER_TDIE                   (1 << 0)  /* Bit 0: Transmit Data Interrupt Enable (TDIE) */
#define LPI2C_SIER_RDIE                   (1 << 1)  /* Bit 1: Receive Data Interrupt Enable (RDIE) */
#define LPI2C_SIER_AVIE                   (1 << 2)  /* Bit 2: Address Valid Interrupt Enable (AVIE) */
#define LPI2C_SIER_TAIE                   (1 << 3)  /* Bit 3: Transmit ACK Interrupt Enable (TAIE) */
                                                    /* Bits 4-7: Reserved */
#define LPI2C_SIER_RSIE                   (1 << 8)  /* Bit 8: Repeated Start Interrupt Enable (RSIE) */
#define LPI2C_SIER_SDIE                   (1 << 9)  /* Bit 9: STOP Detect Interrupt Enable (SDIE) */
#define LPI2C_SIER_BEIE                   (1 << 10) /* Bit 10: Bit Error Interrupt Enable (BEIE) */
#define LPI2C_SIER_FEIE                   (1 << 11) /* Bit 11: FIFO Error Interrupt Enable (FEIE) */
#define LPI2C_SIER_AM0IE                  (1 << 12) /* Bit 12: Address Match 0 Interrupt Enable (AM0IE) */
#define LPI2C_SIER_AM1IE                  (1 << 13) /* Bit 13: Address Match 1 Interrupt Enable (AM1IE) */
#define LPI2C_SIER_GCIE                   (1 << 14) /* Bit 14: General Call Interrupt Enable (GCIE) */
#define LPI2C_SIER_SARIE                  (1 << 15) /* Bit 15: SMBus Alert Response Interrupt Enable (SARIE) */
                                                    /* Bits 16-31: Reserved */

/* Slave DMA Enable Register (SDER) */

#define LPI2C_SDER_TDDE                   (1 << 0)  /* Bit 0: Transmit Data DMA Enable (TDDE) */
#define LPI2C_SDER_RDDE                   (1 << 1)  /* Bit 1: Receive Data DMA Enable (RDDE) */
#define LPI2C_SDER_AVDE                   (1 << 2)  /* Bit 2: Address Valid DMA Enable (AVDE) */
                                                    /* Bits 3-31: Reserved */

/* Slave Configuration Register 1 (SCFGR1) */

#define LPI2C_SCFGR1_ADRSTALL             (1 << 0)  /* Bit 0: Address SCL Stall (ADRSTALL) */
#define LPI2C_SCFGR1_RXSTALL              (1 << 1)  /* Bit 1: RX SCL Stall (RXSTALL) */
#define LPI2C_SCFGR1_TXSTALL              (1 << 2)  /* Bit 2: TX Data SCL Stall (TXDSTALL) */
#define LPI2C_SCFGR1_ACKSTALL             (1 << 3)  /* Bit 3: ACK SCL Stall (ACKSTALL) */
                                                    /* Bits 4-7: Reserved */
#define LPI2C_SCFGR1_GCEN                 (1 << 8)  /* Bit 8: General Call Enable (GCEN) */
#define LPI2C_SCFGR1_SAEN                 (1 << 9)  /* Bit 9: SMBus Alert Enable (SAEN) */
#define LPI2C_SCFGR1_TXCFG                (1 << 10) /* Bit 10: Transmit Flag Configuration (TXCFG) */
#define LPI2C_SCFGR1_RXCFG                (1 << 11) /* Bit 11: Receive Data Configuration (RXCFG) */
#define LPI2C_SCFGR1_IGNACK               (1 << 12) /* Bit 12: Ignore NACK (IGNACK) */
#define LPI2C_SCFGR1_HSMEN                (1 << 13) /* Bit 13: High Speed Mode Enable (HSMEN) */
                                                    /* Bits 14-15: Reserved */
#define LPI2C_SCFG1_ADDRCFG_SHIFT         (16)      /* Bits 16-18: Address Configuration (ADDRCFG) */
#define LPI2C_SCFG1_ADDRCFG_MASK          (0x07 << LPI2C_SCFG1_ADDRCFG_SHIFT)
#  define LPI2C_SCFG1_ADDRCFG(n)          (((n) << LPI2C_SCFG1_ADDRCFG_SHIFT) & LPI2C_SCFG1_ADDRCFG_MASK)
#  define LPI2C_SCFG1_ADDRCFG0            (0x00 << LPI2C_SCFG1_ADDRCFG_SHIFT) /* Address match 0 (7-bit) */
#  define LPI2C_SCFG1_ADDRCFG1            (0x01 << LPI2C_SCFG1_ADDRCFG_SHIFT) /* Address match 0 (10-bit) */
#  define LPI2C_SCFG1_ADDRCFG2            (0x02 << LPI2C_SCFG1_ADDRCFG_SHIFT) /* Address match 0 (7-bit) or Address match 1 (7-bit) */
#  define LPI2C_SCFG1_ADDRCFG3            (0x03 << LPI2C_SCFG1_ADDRCFG_SHIFT) /* Address match 0 (10-bit) or Address match 1 (10-bit) */
#  define LPI2C_SCFG1_ADDRCFG4            (0x04 << LPI2C_SCFG1_ADDRCFG_SHIFT) /* Address match 0 (7-bit) or Address match 1 (10-bit) */
#  define LPI2C_SCFG1_ADDRCFG5            (0x05 << LPI2C_SCFG1_ADDRCFG_SHIFT) /* Address match 0 (10-bit) or Address match 1 (7-bit) */
#  define LPI2C_SCFG1_ADDRCFG6            (0x06 << LPI2C_SCFG1_ADDRCFG_SHIFT) /* From Address match 0 (7-bit) to Address match 1 (7-bit) */
#  define LPI2C_SCFG1_ADDRCFG7            (0x07 << LPI2C_SCFG1_ADDRCFG_SHIFT) /* From Address match 0 (10-bit) to Address match 1 (10-bit) */

                                                    /* Bits 19-31: Reserved */

/* Slave Configuration Register 2 (SCFGR2) */

#define LPI2C_SCFGR2_CLKHOLD_SHIFT        (0)       /* Bits 0-3: Clock Hold Time (CLKHOLD) */
#define LPI2C_SCFGR2_CLKHOLD_MASK         (0xff << LPI2C_SCFGR2_CLKHOLD_SHIFT)
#  define LPI2C_SCFGR2_CLKHOLD(n)         (((n) & LPI2C_SCFGR2_CLKHOLD_MASK) << LPI2C_SCFGR2_CLKHOLD_SHIFT)
                                                    /* Bits 4-7: Reserved */
#define LPI2C_SCFGR2_DATAVD_SHIFT         (8)       /* Bits 8-13: Data Valid Delay (DATAVD) */
#define LPI2C_SCFGR2_DATAVD_MASK          (0x3f << LPI2C_SCFGR2_DATAVD_SHIFT)
#  define LPI2C_SCFGR2_DATAVD(n)          (((n) << LPI2C_SCFGR2_DATAVD_SHIFT) & LPI2C_SCFGR2_DATAVD_MASK)
                                                    /* Bits 14-15: Reserved */
#define LPI2C_SCFGR2_FILTSCL_SHIFT        (16)      /* Bits 16-19: Glitch Filter SCL (FILTSCL) */
#define LPI2C_SCFGR2_FILTSCL_MASK         (0xff << LPI2C_SCFGR2_FILTSCL_SHIFT)
#define LPI2C_SCFGR2_FILTSCL_DISABLE      (0x00 << LPI2C_SCFGR2_FILTSCL_SHIFT)
#  define LPI2C_SCFGR2_FILTSCL_CYCLES(n)  (((n) << LPI2C_SCFGR2_FILTSCL_SHIFT) & LPI2C_SCFGR2_FILTSCL_MASK)
                                                    /* Bits 20-23: Reserved */
#define LPI2C_SCFGR2_FILTSDA_SHIFT        (24)      /* Bits 24-27: Glitch Filter SDA (FILTSDA) */
#define LPI2C_SCFGR2_FILTSDA_MASK         (0xff << LPI2C_SCFGR2_FILTSDA_SHIFT)
#define LPI2C_SCFGR2_FILTSDA_DISABLE      (0x00 << LPI2C_SCFGR2_FILTSDA_SHIFT)
#  define LPI2C_SCFGR2_FILTSDA_CYCLES(n)  (((n) << LPI2C_SCFGR2_FILTSDA_SHIFT) & LPI2C_SCFGR2_FILTSDA_MASK)
                                                    /* Bits 28-31: Reserved */

/* Slave Address Match Register (SAMR) */

                                                    /* Bit 0: Reserved */
#define LPI2C_SAMR_ADDR0_SHIFT            (1)       /* Bits 1-10: Address 0 Value (ADDR0) */
#define LPI2C_SAMR_ADDR0_MASK             (0x03ff << LPI2C_SAMR_ADDR0_SHIFT)
#  define LPI2C_SAMR_ADDR0(n)             (((n) << LPI2C_SAMR_ADDR0_SHIFT) & LPI2C_SAMR_ADDR0_MASK)
                                                    /* Bits 11-16: Reserved */
#define LPI2C_SAMR_ADDR1_SHIFT            (17)      /* Bits 17-26: Address 1 Value (ADDR1) */
#define LPI2C_SAMR_ADDR1_MASK             (0x03ff << LPI2C_SAMR_ADDR1_SHIFT)
#  define LPI2C_SAMR_ADDR1(n)             (((n) << LPI2C_SAMR_ADDR1_SHIFT) & LPI2C_SAMR_ADDR1_MASK)
                                                    /* Bits 27-31: Reserved */

/* Slave Address Status Register (SASR) */

#define LPI2C_SASR_RADDR_SHIFT            (0)       /* Bits 0-10: Received Address (RADDR) */
#define LPI2C_SASR_RADDR_MASK             (0x07ff << LPI2C_SASR_RADDR_SHIFT)
                                                    /* Bits 11-13: Reserved */
#define LPI2C_SASR_ANV                    (1 << 14) /* Bit 14: Address Not Valid (ANV) */
                                                    /* Bits 15-31: Reserved */

/* Slave Transmit ACK Register (STAR) */

#define LPI2C_STAR_TXNACK                 (1 << 0)  /* Bit 0: Transmit NACK (TXNACK) */
                                                    /* Bits 1-31: Reserved */

/* Slave Transmit Data Register (STDR) */

#define LPI2C_STDR_DATA_SHIFT             (0)       /* Bits 0-7: Transmit Data (DATA) */
#define LPI2C_STDR_DATA_MASK              (0xff << LPI2C_STDR_DATA_SHIFT)
#  define LPI2C_STDR_DATA(n)              (((n) << LPI2C_STDR_DATA_SHIFT) & LPI2C_STDR_DATA_MASK)
                                                    /* Bits 8-31: Reserved */

/* Slave Receive Data Register (SRDR) */

#define LPI2C_SRDR_DATA_SHIFT             (0)       /* Bits 0-7: Receive Data (DATA) */
#define LPI2C_SRDR_DATA_MASK              (0xff << LPI2C_SRDR_DATA_SHIFT)
#  define LPI2C_SRDR_DATA(n)              (((n) << LPI2C_SRDR_DATA_SHIFT) & LPI2C_SRDR_DATA_MASK)
                                                    /* Bits 8-13: Reserved */
#define LPI2C_SRDR_RXEMPTY                (1 << 14) /* Bit 14: RX Empty (RXEMPTY) */
#define LPI2C_SRDR_SOF                    (1 << 15) /* Bit 15: Start Of Frame (SOF) */
                                                    /* Bits 16-31: Reserved */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_LPI2C_H */
