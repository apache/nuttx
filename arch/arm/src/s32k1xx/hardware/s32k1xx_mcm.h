/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_mcm.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_MCM_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_MCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

#ifdef CONFIG_ARCH_CHIP_S32K14X

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* CMU Register Offsets *****************************************************/

#define S32K1XX_MCM_PLASC_BASE     0x0008  /* Crossbar Switch (AXBS) Slave Configuration */
#define S32K1XX_MCM_PLAMC_BASE     0x000a  /* Crossbar Switch (AXBS) Master Configuration */
#define S32K1XX_MCM_CPCR_BASE      0x000c  /* Core Platform Control Register */
#define S32K1XX_MCM_ISCR_BASE      0x0010  /* Interrupt Status and Control Register */
#define S32K1XX_MCM_PID_BASE       0x0030  /* Process ID Register */
#define S32K1XX_MCM_CPO_BASE       0x0040  /* Compute Operation Control Register */
#define S32K1XX_MCM_LMDR0_BASE     0x0400  /* Local Memory Descriptor Register 0 */
#define S32K1XX_MCM_LMDR1_BASE     0x0404  /* Local Memory Descriptor Register 1 */
#define S32K1XX_MCM_LMDR2_BASE     0x0408  /* Local Memory Descriptor Register 2 */
#define S32K1XX_MCM_LMPECR_BASE    0x0480  /* LMEM Parity and ECC Control Register */
#define S32K1XX_MCM_LMPEIR_BASE    0x0488  /* LMEM Parity and ECC Interrupt Register */
#define S32K1XX_MCM_LMFAR_BASE     0x0490  /* LMEM Fault Address Register */
#define S32K1XX_MCM_LMFATR_BASE    0x0494  /* LMEM Fault Attribute Register */
#define S32K1XX_MCM_LMFDHR_BASE    0x04a0  /* LMEM Fault Data High Register */
#define S32K1XX_MCM_LMFDLR_BASE    0x04a4  /* LMEM Fault Data Low Register */

/* CMU Register Addresses ***************************************************/

#define S32K1XX_MCM_PLASC          (S32K1XX_MCM_BASE + S32K1XX_MCM_PLASC_BASE)
#define S32K1XX_MCM_PLAMC          (S32K1XX_MCM_BASE + S32K1XX_MCM_PLAMC_BASE)
#define S32K1XX_MCM_CPCR           (S32K1XX_MCM_BASE + S32K1XX_MCM_CPCR_BASE)
#define S32K1XX_MCM_ISCR           (S32K1XX_MCM_BASE + S32K1XX_MCM_ISCR_BASE)
#define S32K1XX_MCM_PID            (S32K1XX_MCM_BASE + S32K1XX_MCM_PID_BASE)
#define S32K1XX_MCM_CPO            (S32K1XX_MCM_BASE + S32K1XX_MCM_CPO_BASE)
#define S32K1XX_MCM_LMDR0          (S32K1XX_MCM_BASE + S32K1XX_MCM_LMDR0_BASE)
#define S32K1XX_MCM_LMDR1          (S32K1XX_MCM_BASE + S32K1XX_MCM_LMDR1_BASE)
#define S32K1XX_MCM_LMDR2          (S32K1XX_MCM_BASE + S32K1XX_MCM_LMDR2_BASE)
#define S32K1XX_MCM_LMPECR         (S32K1XX_MCM_BASE + S32K1XX_MCM_LMPECR_BASE)
#define S32K1XX_MCM_LMPEIR         (S32K1XX_MCM_BASE + S32K1XX_MCM_LMPEIR_BASE)
#define S32K1XX_MCM_LMFAR          (S32K1XX_MCM_BASE + S32K1XX_MCM_LMFAR_BASE)
#define S32K1XX_MCM_LMFATR         (S32K1XX_MCM_BASE + S32K1XX_MCM_LMFATR_BASE)
#define S32K1XX_MCM_LMFDHR         (S32K1XX_MCM_BASE + S32K1XX_MCM_LMFDHR_BASE)
#define S32K1XX_MCM_LMFDLR         (S32K1XX_MCM_BASE + S32K1XX_MCM_LMFDLR_BASE)

/* CMU Register Bitfield Definitions ****************************************/

/* Crossbar Switch (AXBS) Slave Configuration (16-bit) */

#define MCM_PLASC_ASC(n)           (1 << (n)) /* Bus slave connection to AXBS port n present */

/* Crossbar Switch (AXBS) Master Configuration */

#define MCM_PLAMC_AMC(n)           (1 << (n)) /* Bus master connection to AXBS port n present */

/* Core Platform Control Register */

#define MCM_CPCR_HLT_FSM_ST_SHIFT  (0)        /* Bits 0-1: AXBS Halt State Machine Status */
#define MCM_CPCR_HLT_FSM_ST_MASK   (3 << MCM_CPCR_HLT_FSM_ST_SHIFT)
#  define MCM_CPCR_HLT_FSM_WTREQ   (0 << MCM_CPCR_HLT_FSM_ST_SHIFT) /* Waiting for request */
#  define MCM_CPCR_HLT_FSM_WTIDLE  (1 << MCM_CPCR_HLT_FSM_ST_SHIFT) /* Waiting for platform idle */
#  define MCM_CPCR_HLT_FSM_STALLED (2 << MCM_CPCR_HLT_FSM_ST_SHIFT) /* Platform stalled */

#define MCM_CPCR_AXBS_HLT_REQ      (1 << 2)  /* Bit 2:  AXBS Halt Request */
#define MCM_CPCR_AXBS_HLTD         (1 << 3)  /* Bit 3:  AXBS Halted */
#define MCM_CPCR_FMC_PF_IDLE       (1 << 4)  /* Bit 4:  Flash Memory Controller Program Flash Idle */
#define MCM_CPCR_PBRIDGE_IDLE      (1 << 6)  /* Bit 6:  Peripheral Bridge Idle */
#define MCM_CPCR_CBRR              (1 << 9)  /* Bit 8:  Crossbar Round-robin Arbitration Enable */
#define MCM_CPCR_SRAMUAP_SHIFT     (24)      /* Bits 24-25:  SRAM_U Arbitration Priority */
#define MCM_CPCR_SRAMUAP_MASK      (3 << MCM_CPCR_SRAMUAP_SHIFT)
#  define MCM_CPCR_SRAMUAP_RR      (0 << MCM_CPCR_SRAMUAP_SHIFT) /* Round robin */
#  define MCM_CPCR_SRAMUAP_SRR     (1 << MCM_CPCR_SRAMUAP_SHIFT) /* Special round robin (favors SRAM backdoor) */#  define MCM_CPCR_SRAMUAP_FIXEDP  (2 << MCM_CPCR_SRAMUAP_SHIFT) /* Fixed priority. Processor has highest */
#  define MCM_CPCR_SRAMUAP_FIXEDB  (3 << MCM_CPCR_SRAMUAP_SHIFT) /* Fixed priority. Backdoor has highest */

#define MCM_CPCR_SRAMUWP           (1 << 26) /* Bit 26: SRAM_U Write Protect */
#define MCM_CPCR_SRAMLAP_SHIFT     (28)      /* Bits 28-29:  SRAM_L Arbitration Priority */
#define MCM_CPCR_SRAMLAP_MASK      (3 << MCM_CPCR_SRAMLAP_SHIFT)
#  define MCM_CPCR_SRAMLAP_RR      (0 << MCM_CPCR_SRAMLAP_SHIFT) /* Round robin */
#  define MCM_CPCR_SRAMLAP_SRR     (1 << MCM_CPCR_SRAMLAP_SHIFT) /* Special round robin (favors SRAM backdoor) */#  define MCM_CPCR_SRAMLAP_FIXEDP  (2 << MCM_CPCR_SRAMLAP_SHIFT) /* Fixed priority. Processor has highest */
#  define MCM_CPCR_SRAMLAP_FIXEDB  (3 << MCM_CPCR_SRAMLAP_SHIFT) /* Fixed priority. Backdoor has highest */

#define MCM_CPCR_SRAMLWP           (1 << 30) /* Bit 30: SRAM_L Write Protect */

/* Interrupt Status and Control Register */

#define MCM_ISCR_IOC               (1 << 8)  /* Bit 8:  FPU Invalid Operation Interrupt Status */
#define MCM_ISCR_FDZC              (1 << 9)  /* Bit 9:  FPU Divide-by-Zero Interrupt Status */
#define MCM_ISCR_FOFC              (1 << 10) /* Bit 10: FPU Overflow Interrupt Status */
#define MCM_ISCR_FUFC              (1 << 11) /* Bit 11: FPU Underflow Interrupt Status */
#define MCM_ISCR_FIXC              (1 << 12) /* Bit 12: FPU Inexact Interrupt Status */
#define MCM_ISCR_FIDC              (1 << 15) /* Bit 15: FPU Input Denormal Interrupt Status */
#define MCM_ISCR_FIOCE             (1 << 24) /* Bit 24: FPU Invalid Operation Interrupt Enable */
#define MCM_ISCR_FDZCE             (1 << 25) /* Bit 25: FPU Divide-by-Zero Interrupt Enable */
#define MCM_ISCR_FOFCE             (1 << 26) /* Bit 26: FPU Overflow Interrupt Enable */
#define MCM_ISCR_FUFCE             (1 << 27) /* Bit 27: FPU Underflow Interrupt Enable */
#define MCM_ISCR_FIXCE             (1 << 28) /* Bit 28: FPU Inexact Interrupt Enable */
#define MCM_ISCR_FIDCE             (1 << 31) /* Bit 31: FPU Input Denormal Interrupt Enable */

/* Process ID Register */

#define MCM_PID_SHIFT              (0)       /* Bits 0-7:  M0_PID and M1_PID for MPU */
#define MCM_PID_MASK               (0xff << MCM_PID_SHIFT)
#  define MCM_PID(n)               ((uint32_t)(n) << MCM_PID_SHIFT)

/* Compute Operation Control Register */

#define MCM_CPO_CPOREQ             (1 << 0)  /* Bit 0:  Compute Operation Request */
#define MCM_CPO_CPOACK             (1 << 1)  /* Bit 1:  Compute Operation Acknowledge */
#define MCM_CPO_CPOWOI             (1 << 2)  /* Bit 2:  Compute Operation Wakeup On Interrupt */

/* Local Memory Descriptor Registers */

#define MCM_LMDR_CF0_SHIFT         (0)       /* Bits 0-3: Control Field 0 (LMDR0 and LMDR1) */
#define MCM_LMDR_CF0_MASK          (15 << MCM_LMDR_CF0_SHIFT)
#  define MCM_LMDR_CF0_EEWG        (0 << MCM_LMDR_CF0_SHIFT) /* CF0[0]: ECC Enable Write Generation */
#  define MCM_LMDR_CF0_EERC        (1 << MCM_LMDR_CF0_SHIFT) /* CF0[1]: ECC Enable Read Check */

#define MCM_LMDR2_CF1_SHIFT        (4)       /* Bits 4-7: Control Field 1 (LMDR2) */
#define MCM_LMDR2_CF1_MASK         (15 << MCM_LMDR2_CF1_SHIFT)
#  define MCM_LMDR2_CF1_PCPME      (2 << MCM_LMDR2_CF1_SHIFT) /* CF1[1]: PC Parity Miss Enable */
#  define MCM_LMDR2_CF1_PCPFE      (8 << MCM_LMDR2_CF1_SHIFT) /* CF1[3]: PC Parity Fault Enable */

#define MCM_LMDR_MT_SHIFT          (13)      /* Bits 13-15: Memory Type */
#define MCM_LMDR_MT_MASK           (7 << MCM_LMDR_MT_SHIFT)
#  define MCM_LMDR_MT_SRAML        (0 << MCM_LMDR_MT_SHIFT) /* SRAM_L (LMDR0 and LMDR1) */
#  define MCM_LMDR_MT_SRAMU        (1 << MCM_LMDR_MT_SHIFT) /* SRAM_U (LMDR0 and LMDR1) */
#  define MCM_LMDR2_MT_PCCACHE     (2 << MCM_LMDR_MT_SHIFT) /* PC Cache (LMDR2) */

#define MCM_LMDR_LOCK              (1 << 16) /* Bit 16: Lock */
#define MCM_LMDR_DPW_SHIFT         (17)      /* Bits 17-19: LMEM Data Path Width */
#define MCM_LMDR_DPW_MASK          (7 << MCM_LMDR_DPW_SHIFT)
#  define MCM_LMDR_DPW_ 2BITS      (2 << MCM_LMDR_DPW_SHIFT) /* LMEMn 32-bits wide */
#  define MCM_LMDR_DPW_64BITS      (3 << MCM_LMDR_DPW_SHIFT) /* LMEMn 64-bits wide */

#define MCM_LMDR_WY_SHIFT          (20)      /* Bits 20-23: Level 1 Cache Ways */
#define MCM_LMDR_WY_MASK           (15 << MCM_LMDR_WY_SHIFT)
#  define MCM_LMDR_WY_NOCACHE      (0 << MCM_LMDR_WY_SHIFT) /* No Cache */
#  define MCM_LMDR_WY_2WAY         (2 << MCM_LMDR_WY_SHIFT) /* 2-Way Set Associative */
#  define MCM_LMDR_WY_4WAY         (4 << MCM_LMDR_WY_SHIFT) /* 4-Way Set Associative */

#define MCM_LMDR_LMSZ_SHIFT        (24)      /* Bits 24-27: LMEM Size */
#define MCM_LMDR_LMSZ_MASK         (15 << MCM_LMDR_LMSZ_SHIFT)
#  define MCM_LMDR_LMSZ_ MASK      (0 << MCM_LMDR_LMSZ_SHIFT)  /* No LMEMn (0 KB) */
#  define MCM_LMDR_LMSZ_ MASK      (1 << MCM_LMDR_LMSZ_SHIFT)  /*     1 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR_LMSZ_ MASK      (2 << MCM_LMDR_LMSZ_SHIFT)  /*     2 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR_LMSZ_ MASK      (3 << MCM_LMDR_LMSZ_SHIFT)  /*     4 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR_LMSZ_ MASK      (4 << MCM_LMDR_LMSZ_SHIFT)  /*     8 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR_LMSZ_ MASK      (5 << MCM_LMDR_LMSZ_SHIFT)  /*    16 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR_LMSZ_ MASK      (6 << MCM_LMDR_LMSZ_SHIFT)  /*    32 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR_LMSZ_ MASK      (7 << MCM_LMDR_LMSZ_SHIFT)  /*    64 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR_LMSZ_ MASK      (8 << MCM_LMDR_LMSZ_SHIFT)  /*   128 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR_LMSZ_ MASK      (9 << MCM_LMDR_LMSZ_SHIFT)  /*   256 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR_LMSZ_ MASK      (10 << MCM_LMDR_LMSZ_SHIFT) /*   512 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR_LMSZ_ MASK      (11 << MCM_LMDR_LMSZ_SHIFT) /*  1024 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR_LMSZ_ MASK      (12 << MCM_LMDR_LMSZ_SHIFT) /*  2048 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR_LMSZ_ MASK      (13 << MCM_LMDR_LMSZ_SHIFT) /*  4096 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR_LMSZ_ MASK      (14 << MCM_LMDR_LMSZ_SHIFT) /*  8192 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR_LMSZ_ MASK      (15 << MCM_LMDR_LMSZ_SHIFT) /* 16384 KB LMEMn (LMDR0, LMDR1) */
#  define MCM_LMDR2_LMSZ_ MASK     (4 << MCM_LMDR_LMSZ_SHIFT)  /*     4 KB LMEMn (LMDR2) */

#define MCM_LMDR_LMSZH             (1 << 28) /* Bit 28: LMEM Size Hole */
#define MCM_LMDR_V                 (1 << 31) /* Bit 31: Local Memory Valid */

/* LMEM Parity and ECC Control Register */

#define MCM_LMPECR_ERNCR           (1 << 0)  /* Bit 0:  Enable RAM ECC Noncorrectable Reporting */
#define MCM_LMPECR_ER1BR           (1 << 8)  /* Bit 8:  Enable RAM ECC 1 Bit Reporting */
#define MCM_LMPECR_ECPR            (1 << 20) /* Bit 20: Enable Cache Parity Reporting */

/* LMEM Parity and ECC Interrupt Register */

#define MCM_LMPEIR_ENC_SHIFT       (0)       /* Bits 0-7: ENCn = ECC Noncorrectable Error n */
#define MCM_LMPEIR_ENC_MASK        (0xff << MCM_LMPEIR_ENC_SHIFT)
#  define MCM_LMPEIR_ENC_SRAML     (1 << 0)  /* PEIR[0]: Noncorrectable SRAM_L Error detected */
#  define MCM_LMPEIR_ENC_SRAMU     (2 << 1)  /* PEIR[1]: Noncorrectable SRAM_U Error detected */
#define MCM_LMPEIR_E1B_SHIFT       (8)       /* Bits 8-15: E1Bn = ECC 1-bit Error n */
#define MCM_LMPEIR_E1B_MASK        (0xff << MCM_LMPEIR_E1B_SHIFT)
#  define MCM_LMPEIR_E1B_SRAML     (1 << 8)  /* PEIR[8]: 1-bit Error detected on SRAM_L */
#  define MCM_LMPEIR_E1B_SRAMU     (2 << 9)  /* PEIR[9]: 1-bit Error detected on SRAM_U */
#define MCM_LMPEIR_PE_SHIFT        (16)      /* Bits 16-23: Cache Parity Error */
#define MCM_LMPEIR_PE_MASK         (0xff << MCM_LMPEIR_PE_SHIFT)
#  define MCM_LMPEIR_PE_TAG        (1 << 20) /* PE[20]: PC Tag Parity Error */
#  define MCM_LMPEIR_PE_DATA       (1 << 21) /* PE[21]: PC Data Parity Error */
#define MCM_LMPEIR_PEELOC_SHIFT    (24)      /* Bits 24-28: Parity or ECC Error Location */
#define MCM_LMPEIR_PEELOC_MASK     (0x1f << MCM_LMPEIR_PEELOC_SHIFT)
#  define MCM_LMPEIR_PEELOC_NCSRAML  (0 << MCM_LMPEIR_PEELOC_SHIFT)  /* Non-correctable ECC event from SRAM_L */
#  define MCM_LMPEIR_PEELOC_NCSRAMU  (1 << MCM_LMPEIR_PEELOC_SHIFT)  /* Non-correctable ECC event from SRAM_U */
#  define MCM_LMPEIR_PEELOC_CSRAML   (8 << MCM_LMPEIR_PEELOC_SHIFT)  /* 1-bit correctable ECC event from SRAM_L */
#  define MCM_LMPEIR_PEELOC_CSRAMU   (9 << MCM_LMPEIR_PEELOC_SHIFT)  /* 1-bit correctable ECC event from SRAM_U */
#  define MCM_LMPEIR_PEELOC_PCTAG    (14 << MCM_LMPEIR_PEELOC_SHIFT) /* PC tag parity error */
#  define MCM_LMPEIR_PEELOC_PCDATA   (15 << MCM_LMPEIR_PEELOC_SHIFT) /* PC data parity error */

#define MCM_LMPEIR_V               (1 << 31) /* Bit 31: Valid Bit */

/* LMEM Fault Address Register */

#define MCM_LMFAR_PEFPRT_SHIFT     (0)       /* Bits 0-3:  Parity/ECC Fault Protection */
#define MCM_LMFAR_PEFPRT_MASK      (15 << MCM_LMFAR_PEFPRT_SHIFT)
#  define MCM_LMFAR_PEFPRT_DATA       (1 << MCM_LMFAR_PEFPRT_SHIFT) /* FATR[0]: Type: 0=I-Fetch, 1=Data */
#  define MCM_LMFAR_PEFPRT_SUPERVISOR (2 << MCM_LMFAR_PEFPRT_SHIFT) /* FATR[1]: Mode: 0=User mode, 1=Supervisor mode */
#  define MCM_LMFAR_PEFPRT_BUFFERABLE (4 << MCM_LMFAR_PEFPRT_SHIFT) /* FATR[2]: Bufferable: 0=Non-bufferable, 1=Bufferable */
#  define MCM_LMFAR_PEFPRT_CACHEABLE  (8 << MCM_LMFAR_PEFPRT_SHIFT) /* FATR[3]: Cacheable: 0=Non-cacheable, 1=Cacheable */

#define MCM_LMFAR_PEFSIZE_SHIFT    (4)       /* Bits 4-6:  Parity/ECC Fault Master Size */
#define MCM_LMFAR_PEFSIZE_MASK     (7 << MCM_LMFAR_PEFSIZE_SHIFT)
#  define MCM_LMFAR_PEFSIZE_8BIT   (0 << MCM_LMFAR_PEFSIZE_SHIFT) /* 8-bit access */
#  define MCM_LMFAR_PEFSIZE_16BIT  (1 << MCM_LMFAR_PEFSIZE_SHIFT) /* 16-bit access */
#  define MCM_LMFAR_PEFSIZE_32BIT  (2 << MCM_LMFAR_PEFSIZE_SHIFT) /* 32-bit access */
#  define MCM_LMFAR_PEFSIZE_64BIT  (3 << MCM_LMFAR_PEFSIZE_SHIFT) /* 64-bit access */

#define MCM_LMFAR_PEFW             (1 << 7)  /* Bit 7:  Parity/ECC Fault Write */
#define MCM_LMFAR_PEFMST_SHIFT     (8)       /* Bits 8-15: Parity/ECC Fault Master Number */
#define MCM_LMFAR_PEFMST_MASK      (0xff << MCM_LMFAR_PEFMST_SHIFT)
#  define MCM_LMFAR_PEFMST(n)      ((uint32_t)(n) << MCM_LMFAR_PEFMST_SHIFT)
#define MCM_LMFAR_OVR              (1 << 31) /* Bit 31: Overrun */

/* LMEM Fault Attribute Register */

#define MCM_LMFATR_PEFPRT_SHIFT     (0)       /* Bits 0-3:  Parity/ECC Fault Protection */
#define MCM_LMFATR_PEFPRT_MASK      (15 << MCM_LMFATR_PEFPRT_SHIFT)
#  define MCM_LMFATR_PEFPRT_DATA       (1 << MCM_LMFATR_PEFPRT_SHIFT) /* FATR[0]: Type: 0=I-Fetch, 1=Data */
#  define MCM_LMFATR_PEFPRT_SUPERVISOR (2 << MCM_LMFATR_PEFPRT_SHIFT) /* FATR[1]: Mode: 0=User mode, 1=Supervisor mode */
#  define MCM_LMFATR_PEFPRT_BUFFERABLE (4 << MCM_LMFATR_PEFPRT_SHIFT) /* FATR[2]: Bufferable: 0=Non-bufferable, 1=Bufferable */
#  define MCM_LMFATR_PEFPRT_CACHEABLE  (8 << MCM_LMFATR_PEFPRT_SHIFT) /* FATR[3]: Cacheable: 0=Non-cacheable, 1=Cacheable */

#define MCM_LMFATR_PEFSIZE_SHIFT    (4)       /* Bits 4-6:  Parity/ECC Fault Master Size */
#define MCM_LMFATR_PEFSIZE_MASK     (7 << MCM_LMFATR_PEFSIZE_SHIFT)
#  define MCM_LMFATR_PEFSIZE_8BIT   (0 << MCM_LMFATR_PEFSIZE_SHIFT) /* 8-bit access */
#  define MCM_LMFATR_PEFSIZE_16BIT  (1 << MCM_LMFATR_PEFSIZE_SHIFT) /* 16-bit access */
#  define MCM_LMFATR_PEFSIZE_32BIT  (2 << MCM_LMFATR_PEFSIZE_SHIFT) /* 32-bit access */
#  define MCM_LMFATR_PEFSIZE_64BIT  (3 << MCM_LMFATR_PEFSIZE_SHIFT) /* 64-bit access */

#define MCM_LMFATR_PEFW             (1 << 7)  /* Bit 7:  Parity/ECC Fault Write */
#define MCM_LMFATR_PEFMST_SHIFT     (8)       /* Bits 8-15: Parity/ECC Fault Master Number */
#define MCM_LMFATR_PEFMST_MASK      (0xff << MCM_LMFATR_PEFMST_SHIFT)
#  define MCM_LMFATR_PEFMST(n)      ((uint32_t)(n) << MCM_LMFATR_PEFMST_SHIFT)
#define MCM_LMFATR_OVR              (1 << 31) /* Bit 31: Overrun */

/* LMEM Fault Data High Register (32-bit address data) */

/* LMEM Fault Data Low Register (32-bit address data) */

#endif /* CONFIG_ARCH_CHIP_S32K14X */
#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_MCM_H */
