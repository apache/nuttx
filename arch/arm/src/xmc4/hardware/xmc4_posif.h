/****************************************************************************
 * arch/arm/src/xmc4/hardware/xmc4_posif.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * May include some logic from sample code provided by Infineon:
 *
 *   Copyright (C) 2011-2015 Infineon Technologies AG. All rights reserved.
 *
 * Infineon Technologies AG (Infineon) is supplying this software for use
 * with Infineon's microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such microcontrollers.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS
 * SOFTWARE. INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
 * INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ****************************************************************************/

/* Reference: XMC[8-7]00 Reference Manual. */

#ifndef __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_POSIF_H
#define __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_POSIF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/xmc4_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* POSIF Kernel Registers */

#define XMC4_POSIF_PCONF_OFFSET 0x0000  /* Global control register */
#define XMC4_POSIF_PSUS_OFFSET  0x0004  /* Suspend configuration */
#define XMC4_POSIF_PRUNS_OFFSET 0x0008  /* POSIF Run Bit set */
#define XMC4_POSIF_PRUNC_OFFSET 0x000C  /* POSIF Run Bit clear */
#define XMC4_POSIF_PRUN_OFFSET  0x0010  /* POSIF Run Bit status */
#define XMC4_POSIF_PDBG_OFFSET  0x0100  /* Debug design register */
#define XMC4_POSIF_MIDR_OFFSET  0x0020  /* Module identification register */

/* Hall Sensor Mode Registers */

#define XMC4_POSIF_HALP_OFFSET  0x0030  /* Hall Current and Expected patterns */
#define XMC4_POSIF_HALPS_OFFSET 0x0034  /* Hall Current and Expected Shadow paterns */

/* Multi-Channel Mode Registers */

#define XMC4_POSIF_MCM_OFFSET  0x0040  /* Multi-Channel Mode Patern */
#define XMC4_POSIF_MCSM_OFFSET 0x0044  /* Multi-Channel Mode Shadow Patern */
#define XMC4_POSIF_MCMS_OFFSET 0x0048  /* Multi-Channel Mode Control Set */
#define XMC4_POSIF_MCMC_OFFSET 0x004C  /* Multi-Channel Mode Control Clear */
#define XMC4_POSIF_MCMF_OFFSET 0x0050  /* Multi-Channel Mode Flag Status */

/* Quadrature Decoder Mode Register */

#define XMC4_POSIF_QDC_OFFSET 0x0060 /* Quadrature Decoder Configuration */

/* Interrupt Registers */

#define XMC4_POSIF_PFLG_OFFSET  0x0070  /* POSIF Interrupt status */
#define XMC4_POSIF_PFLGE_OFFSET 0x0074  /* POSIF interrupt enable */
#define XMC4_POSIF_SPFLG_OFFSET 0x0078  /* Interrupt set register */
#define XMC4_POSIF_RPFLG_OFFSET 0x007C  /* Interrupt clear register */

/* POSIF0 Registers *********************************************************/

#define XMC4_POSIF0_PCONF (XMC4_POSIF0_BASE + XMC4_POSIF_PCONF_OFFSET)  /* Global control register */
#define XMC4_POSIF0_PSUS  (XMC4_POSIF0_BASE + XMC4_POSIF_PSUS_OFFSET)   /* Suspend configuration */
#define XMC4_POSIF0_PRUNS (XMC4_POSIF0_BASE + XMC4_POSIF_PRUNS_OFFSET)  /* POSIF Run Bit set */
#define XMC4_POSIF0_PRUNC (XMC4_POSIF0_BASE + XMC4_POSIF_PRUNC_OFFSET)  /* POSIF Run Bit clear */
#define XMC4_POSIF0_PRUN  (XMC4_POSIF0_BASE + XMC4_POSIF_PRUN_OFFSET)   /* POSIF Run Bit status */
#define XMC4_POSIF0_PDBG  (XMC4_POSIF0_BASE + XMC4_POSIF_PDBG_OFFSET)   /* Debug design register */
#define XMC4_POSIF0_MIDR  (XMC4_POSIF0_BASE + XMC4_POSIF_MIDR_OFFSET)   /* Module identification register */

#define XMC4_POSIF0_HALP  (XMC4_POSIF0_BASE + XMC4_POSIF_HALP_OFFSET)   /* Hall Current and Expected patterns */
#define XMC4_POSIF0_HALPS (XMC4_POSIF0_BASE + XMC4_POSIF_HALPS_OFFSET)  /* Hall Current and Expected Shadow paterns */

#define XMC4_POSIF0_MCM  (XMC4_POSIF0_BASE + XMC4_POSIF_MCM_OFFSET)   /* Multi-Channel Mode Patern */
#define XMC4_POSIF0_MCSM (XMC4_POSIF0_BASE + XMC4_POSIF_MCSM_OFFSET)  /* Multi-Channel Mode Shadow Patern */
#define XMC4_POSIF0_MCMS (XMC4_POSIF0_BASE + XMC4_POSIF_MCMS_OFFSET)  /* Multi-Channel Mode Control Set */
#define XMC4_POSIF0_MCMC (XMC4_POSIF0_BASE + XMC4_POSIF_MCMC_OFFSET)  /* Multi-Channel Mode Control Clear */
#define XMC4_POSIF0_MCMF (XMC4_POSIF0_BASE + XMC4_POSIF_MCMF_OFFSET)  /* Multi-Channel Mode Flag Status */

#define XMC4_POSIF0_QDC (XMC4_POSIF0_BASE + XMC4_POSIF_QDC_OFFSET)  /* Quadrature Decoder Configuration */

#define XMC4_POSIF0_PFLG  (XMC4_POSIF0_BASE + XMC4_POSIF_PFLG_OFFSET)   /* POSIF Interrupt status */
#define XMC4_POSIF0_PFLGE (XMC4_POSIF0_BASE + XMC4_POSIF_PFLGE_OFFSET)  /* POSIF interrupt enable */
#define XMC4_POSIF0_SPFLG (XMC4_POSIF0_BASE + XMC4_POSIF_SPFLG_OFFSET)  /* Interrupt set register */
#define XMC4_POSIF0_RPFLG (XMC4_POSIF0_BASE + XMC4_POSIF_RPFLG_OFFSET)  /* Interrupt clear register */

/* POSIF1 Registers *********************************************************/

#define XMC4_POSIF1_PCONF (XMC4_POSIF1_BASE + XMC4_POSIF_PCONF_OFFSET)  /* Global control register */
#define XMC4_POSIF1_PSUS  (XMC4_POSIF1_BASE + XMC4_POSIF_PSUS_OFFSET)   /* Suspend configuration */
#define XMC4_POSIF1_PRUNS (XMC4_POSIF1_BASE + XMC4_POSIF_PRUNS_OFFSET)  /* POSIF Run Bit set */
#define XMC4_POSIF1_PRUNC (XMC4_POSIF1_BASE + XMC4_POSIF_PRUNC_OFFSET)  /* POSIF Run Bit clear */
#define XMC4_POSIF1_PRUN  (XMC4_POSIF1_BASE + XMC4_POSIF_PRUN_OFFSET)   /* POSIF Run Bit status */
#define XMC4_POSIF1_PDBG  (XMC4_POSIF1_BASE + XMC4_POSIF_PDBG_OFFSET)   /* Debug design register */
#define XMC4_POSIF1_MIDR  (XMC4_POSIF1_BASE + XMC4_POSIF_MIDR_OFFSET)   /* Module identification register */

#define XMC4_POSIF1_HALP  (XMC4_POSIF1_BASE + XMC4_POSIF_HALP_OFFSET)   /* Hall Current and Expected patterns */
#define XMC4_POSIF1_HALPS (XMC4_POSIF1_BASE + XMC4_POSIF_HALPS_OFFSET)  /* Hall Current and Expected Shadow paterns */

#define XMC4_POSIF1_MCM  (XMC4_POSIF1_BASE + XMC4_POSIF_MCM_OFFSET)   /* Multi-Channel Mode Patern */
#define XMC4_POSIF1_MCSM (XMC4_POSIF1_BASE + XMC4_POSIF_MCSM_OFFSET)  /* Multi-Channel Mode Shadow Patern */
#define XMC4_POSIF1_MCMS (XMC4_POSIF1_BASE + XMC4_POSIF_MCMS_OFFSET)  /* Multi-Channel Mode Control Set */
#define XMC4_POSIF1_MCMC (XMC4_POSIF1_BASE + XMC4_POSIF_MCMC_OFFSET)  /* Multi-Channel Mode Control Clear */
#define XMC4_POSIF1_MCMF (XMC4_POSIF1_BASE + XMC4_POSIF_MCMF_OFFSET)  /* Multi-Channel Mode Flag Status */

#define XMC4_POSIF1_QDC (XMC4_POSIF1_BASE + XMC4_POSIF_QDC_OFFSET)  /* Quadrature Decoder Configuration */

#define XMC4_POSIF1_PFLG  (XMC4_POSIF1_BASE + XMC4_POSIF_PFLG_OFFSET)   /* POSIF Interrupt status */
#define XMC4_POSIF1_PFLGE (XMC4_POSIF1_BASE + XMC4_POSIF_PFLGE_OFFSET)  /* POSIF interrupt enable */
#define XMC4_POSIF1_SPFLG (XMC4_POSIF1_BASE + XMC4_POSIF_SPFLG_OFFSET)  /* Interrupt set register */
#define XMC4_POSIF1_RPFLG (XMC4_POSIF1_BASE + XMC4_POSIF_RPFLG_OFFSET)  /* Interrupt clear register */

/* POSIF Mask and SHIFTitions ***********************************************/

/* PCONF - POSIF Configuration */
#define POSIF_PCONF_FSEL_SHIFT    (0)                                           /* Bits 0-1 : Function Selector */
#define POSIF_PCONF_FSEL_MASK     (3 << POSIF_PCONF_FSEL_SHIFT)
#define POSIF_PCONF_QDCM_SHIFT    (2)                                           /* Bits 2 : Position Decoder Mode Selection */
#define POSIF_PCONF_QDCM_MASK     (1 << POSIF_PCONF_QDCM_SHIFT)
#define POSIF_PCONF_HIDG_SHIFT    (4)                                           /* Bit 4 : Idle generation enable */
#define POSIF_PCONF_HIDG_MASK     (1 << POSIF_PCONF_HIDG_SHIFT)
#define POSIF_PCONF_MCUE_SHIFT    (5)                                           /* Bit 5 : Multi-Channel pattern SW update enable */
#define POSIF_PCONF_MCUE_MASK     (1 << POSIF_PCONF_MCUE_SHIFT)
#define POSIF_PCONF_INSEL0_SHIFT  (8)                                           /* Bits 8-9 : PhaseA/Hal input 1 selector */
#define POSIF_PCONF_INSEL0_MASK   (3 << POSIF_PCONF_INSEL0_SHIFT)
#define POSIF_PCONF_INSEL1_SHIFT  (10)                                          /* Bits 10-11 : PhaseB/Hal input2 selector */
#define POSIF_PCONF_INSEL1_MASK   (3 << POSIF_PCONF_INSEL1_SHIFT)
#define POSIF_PCONF_INSEL2_SHIFT  (12)                                          /* Bits 12-13 : Index/Hall input 3 selector */
#define POSIF_PCONF_INSEL2_MASK   (3 << POSIF_PCONF_INSEL2_SHIFT)
#define POSIF_PCONF_DSEL_SHIFT    (16)                                          /* Bit 16 : Delay Pin selector */
#define POSIF_PCONF_DSEL_MASK     (1 << POSIF_PCONF_DSEL_SHIFT)
#define POSIF_PCONF_SPES_SHIFT    (17)                                          /* Bit 17 : Edge selector for the sampling trigger */
#define POSIF_PCONF_SPES_MASK     (1 << POSIF_PCONF_SPES_SHIFT)
#define POSIF_PCONF_MSETS_SHIFT   (18)                                          /* Bits 18-20 : Pattern update signal select */
#define POSIF_PCONF_MSETS_MASK    (7 << POSIF_PCONF_MSETS_SHIFT)
#define POSIF_PCONF_MSES_SHIFT    (21)                                          /* Bit 21 : Multi-Channel pattern update trigger edge */
#define POSIF_PCONF_MSES_MASK     (1 << POSIF_PCONF_MSES_SHIFT)
#define POSIF_PCONF_MSYNS_SHIFT   (22)                                          /* Bits 22-23 : PWM Synchronization signal selector */
#define POSIF_PCONF_MSYNS_MASK    (3 << POSIF_PCONF_MSYNS_SHIFT)
#define POSIF_PCONF_EWIS_SHIFT    (24)                                          /* Bits 24-25 : Wrong Hall Event selection */
#define POSIF_PCONF_EWIS_MASK     (3 << POSIF_PCONF_EWIS_SHIFT)
#define POSIF_PCONF_EWIE_SHIFT    (26)                                          /* Bit 26 : External Wrong Hall Event enable */
#define POSIF_PCONF_EWIE_MASK     (1 << POSIF_PCONF_EWIE_SHIFT)
#define POSIF_PCONF_EWIL_SHIFT    (27)                                          /* Bit 27 : External Wrong Hall Event active level */
#define POSIF_PCONF_EWIL_MASK     (1 << POSIF_PCONF_EWIL_SHIFT)
#define POSIF_PCONF_LPC_SHIFT     (28)                                          /* Bits 28-30 : Low Pass Filter Configuration */
#define POSIF_PCONF_LPC_MASK      (7 << POSIF_PCONF_LPC_SHIFT)

/* PSUS - Suspend Configuration */
#define POSIF_PSUS_QSUS_SHIFT     (0)                                           /* Bits 0-1 : Quadrature Mode Suspend Config */
#define POSIF_PSUS_QSUS_MASK      (3 << POSIF_PSUS_QSUS_SHIFT)
#define POSIF_PSUS_MSUS_SHIFT     (2)                                           /* Bits 2-3 : Multi-Channel Mode Suspend Config */
#define POSIF_PSUS_MSUS_MASK      (3 << POSIF_PSUS_MSUS_SHIFT)

/* PRUNS - POSIF run bit set */
#define POSIF_PRUNS_SRB_SHIFT     (0)                                           /* Bit 0 : Set Run Bit */
#define POSIF_PRUNS_SRB_MASK      (1 << POSIF_PRUNS_SRB_SHIFT)

/* PRUNC - POSIF run bit clear */
#define POSIF_PRUNC_CRB_SHIFT     (0)                                           /* Bit 0 : Clear Run Bit*/
#define POSIF_PRUNC_CRB_MASK      (1 << POSIF_PRUNC_CRB_SHIFT)
#define POSIF_PRUNC_CSM_SHIFT     (1)                                           /* Bit 1 : Clear Current internal status */
#define POSIF_PRUNC_CSM_MASK      (1 << POSIF_PRUNC_CSM_SHIFT)

/* PRUN - POSIF run bit status */
#define POSIF_PRUN_RB_SHIFT       (0)                                           /* Bit 0 : Run Bit */
#define POSIF_PRUN_RB_MASK        (1 << POSIF_PRUN_RB_SHIFT)

/* MIDR - Module Identification register */
#define POSIF_MIDR_MODR_SHIFT     (0)                                           /* Bits 0-7 : Module Revision */
#define POSIF_MIDR_MODR_MASK      (0xff << POSIF_MIDR_MODR_SHIFT)
#define POSIF_MIDR_MODT_SHIFT     (8)                                           /*  Bits 8-15 : Module Type */
#define POSIF_MIDR_MODT_MASK      (0xff << POSIF_MIDR_MODT_SHIFT)
#define POSIF_MIDR_MODN_SHIFT     (16)                                          /* Bits 16-31 : Module Number */
#define POSIF_MIDR_MODN_MASK      (0xffff << POSIF_MIDR_MODN_SHIFT)

/* HALP - Hall Current and Expected patterns */
#define POSIF_HALP_HCP_SHIFT      (0)                                           /* Bits 0-2 : Hall Current Pattern */
#define POSIF_HALP_HCP_MASK       (7 << POSIF_HALP_HCP_SHIFT)
#define POSIF_HALP_HEP_SHIFT      (3)                                           /* Bits 3-5 : Hall Expected Pattern */
#define POSIF_HALP_HEP_MASK       (7 << POSIF_HALP_HEP_SHIFT)

/* HALPS - Hall Current and Expected shadow patterns */
#define POSIF_HALPS_HCPS_SHIFT    (0)                                           /* Bits 0-2 : Shadow Hall Current Pattern */
#define POSIF_HALPS_HCPS_MASK     (7 << POSIF_HALPS_HCPS_SHIFT)
#define POSIF_HALPS_HEPS_SHIFT    (3)                                           /* Bits 3-5 : Shadow Hall Expected Pattern */
#define POSIF_HALPS_HEPS_MASK     (7 << POSIF_HALPS_HEPS_SHIFT)

/* MCM - Multi-Channel Mode Pattern */
#define POSIF_MCM_MCMP_SHIFT      (0)                                           /* Bits 0-15 : Multi-Channel Pattern */
#define POSIF_MCM_MCMP_MASK       (0xffff << POSIF_MCM_MCMP_SHIFT)

/* MCSM - Multi-Channel Mode shadow Pattern */
#define POSIF_MCSM_MCMPS_SHIFT    (0)                                           /* Bits 0-15 : Shadow Multi-Channel Pattern */
#define POSIF_MCSM_MCMPS_MASK     (0xffff << POSIF_MCSM_MCMPS_SHIFT)

/* MCMS - Multi-Channel Mode Control set */
#define POSIF_MCMS_MNPS_SHIFT     (0)                                           /* Bit 0 : Multi-Channel Pattern Update Enable Set */
#define POSIF_MCMS_MNPS_MASK      (1 << POSIF_MCMS_MNPS_SHIFT)
#define POSIF_MCMS_STHR_SHIFT     (1)                                           /* Bit 1 : Hall Pattern Shadow Transfer Request */
#define POSIF_MCMS_STHR_MASK      (1 << POSIF_MCMS_STHR_SHIFT)
#define POSIF_MCMS_STMR_SHIFT     (2)                                           /* Bit 2 : Multi-Channel Shadow Transfer Request */
#define POSIF_MCMS_STMR_MASK      (1 << POSIF_MCMS_STMR_SHIFT)

/* MCMC - Multi-Channel Mode Control clear */
#define POSIF_MCMC_MNPC_SHIFT     (0)                                           /* Bit 0 : Multi-Channel Pattern Update Enable Clear */
#define POSIF_MCMC_MNPC_MASK      (1 << POSIF_MCMC_MNPC_SHIFT)
#define POSIF_MCMC_MPC_SHIFT      (1)                                           /* Bit 1 : Multi-Channel Pattern clear */
#define POSIF_MCMC_MPC_MASK       (1 << POSIF_MCMC_MPC_SHIFT)

/* MCMF - Multi-Channel Mode flag status */
#define POSIF_MCMF_MSS_SHIFT      (0)                                           /* Bit 0 : Multi-Channel Pattern update status */
#define POSIF_MCMF_MSS_MASK       (1 << POSIF_MCMF_MSS_SHIFT)

/* QDC - Quadrature Decoder Configuration */
#define POSIF_QDC_PALS_SHIFT      (0)                                           /* Bit 0 : Phase A Level selector */
#define POSIF_QDC_PALS_MASK       (1 << POSIF_QDC_PALS_SHIFT)
#define POSIF_QDC_PBLS_SHIFT      (1)                                           /* Bit 1 : Phase B Level selector */
#define POSIF_QDC_PBLS_MASK       (1 << POSIF_QDC_PBLS_SHIFT)
#define POSIF_QDC_PHS_SHIFT       (2)                                           /* Bit 2 : Phase signals swap */
#define POSIF_QDC_PHS_MASK        (1 << POSIF_QDC_PHS_SHIFT)
#define POSIF_QDC_ICM_SHIFT       (4)                                           /* Bits 4-5 : Index Marker generations control */
#define POSIF_QDC_ICM_MASK        (3 << POSIF_QDC_ICM_SHIFT)
#define POSIF_QDC_DVAL_SHIFT      (8)                                           /* Bit 8 : Current rotation direction */
#define POSIF_QDC_DVAL_MASK       (1 << POSIF_QDC_DVAL_SHIFT)

/* PFLG - POSIF interrupt status */
#define POSIF_PFLG_CHES_SHIFT     (0)                                           /* Bit 0 : Correct Hall Event Status */
#define POSIF_PFLG_CHES_MASK      (1 << POSIF_PFLG_CHES_SHIFT)
#define POSIF_PFLG_WHES_SHIFT     (1)                                           /* Bit 1 : Wrong Hall Event Status */
#define POSIF_PFLG_WHES_MASK      (1 << POSIF_PFLG_WHES_SHIFT)
#define POSIF_PFLG_HIES_SHIFT     (2)                                           /* Bit 2 : Hall Inputs Update Status */
#define POSIF_PFLG_HIES_MASK      (1 << POSIF_PFLG_HIES_SHIFT)
#define POSIF_PFLG_MSTS_SHIFT     (4)                                           /* Bit 4 : Multi-Channel pattern shadow transfer status */
#define POSIF_PFLG_MSTS_MASK      (1 << POSIF_PFLG_MSTS_SHIFT)
#define POSIF_PFLG_INDXS_SHIFT    (8)                                           /* Bit 8 : Quadrature Index Status */
#define POSIF_PFLG_INDXS_MASK     (1 << POSIF_PFLG_INDXS_SHIFT)
#define POSIF_PFLG_ERRS_SHIFT     (9)                                           /* Bit 9 : Quadrature phase Error Status */
#define POSIF_PFLG_ERRS_MASK      (1 << POSIF_PFLG_ERRS_SHIFT)
#define POSIF_PFLG_CNTS_SHIFT     (10)                                          /* Bit 10 : Quadrature CLK Status*/
#define POSIF_PFLG_CNTS_MASK      (1 << POSIF_PFLG_CNTS_SHIFT)
#define POSIF_PFLG_DIRS_SHIFT     (11)                                          /* Bit 11 : Quadrature Direction Change*/
#define POSIF_PFLG_DIRS_MASK      (1 << POSIF_PFLG_DIRS_SHIFT)
#define POSIF_PFLG_PCLKS_SHIFT    (12)                                          /* Bit 12 : Quadrature Period CLK Status*/
#define POSIF_PFLG_PCLKS_MASK     (1 << POSIF_PFLG_PCLKS_SHIFT)

/* PFLGE - POSIF interrupt enable */
#define POSIF_PFLGE_ECHE_SHIFT    (0)                                           /* Bit 0 : Correct Hall Event Enable */
#define POSIF_PFLGE_ECHE_MASK     (1 << POSIF_PFLGE_ECHE_SHIFT)
#define POSIF_PFLGE_EWHE_SHIFT    (1)                                           /* Bit 1 : Wrong Hall Event Enable */
#define POSIF_PFLGE_EWHE_MASK     (1 << POSIF_PFLGE_EWHE_SHIFT)
#define POSIF_PFLGE_EHIE_SHIFT    (2)                                           /* Bit 2 : Hall Input Update Enable */
#define POSIF_PFLGE_EHIE_MASK     (1 << POSIF_PFLGE_EHIE_SHIFT)
#define POSIF_PFLGE_EMST_SHIFT    (4)                                           /* Bit 4 : Multi-Channel pattern shadow transfer enable */
#define POSIF_PFLGE_EMST_MASK     (1 << POSIF_PFLGE_EMST_SHIFT)
#define POSIF_PFLGE_EINDX_SHIFT   (8)                                           /* Bit 8 : Quadrature Index Event Enable */
#define POSIF_PFLGE_EINDX_MASK    (1 << POSIF_PFLGE_EINDX_SHIFT)
#define POSIF_PFLGE_EERR_SHIFT    (9)                                           /* Bit 9 : Quadrature Phase Error Enable */
#define POSIF_PFLGE_EERR_MASK     (1 << POSIF_PFLGE_EERR_SHIFT)
#define POSIF_PFLGE_ECNT_SHIFT    (10)                                          /* Bit 10 : Quadrature CLK interrupt Enable */
#define POSIF_PFLGE_ECNT_MASK     (1 << POSIF_PFLGE_ECNT_SHIFT)
#define POSIF_PFLGE_EDIR_SHIFT    (11)                                          /* Bit 11 : Quadrature direction change interrupt Enable */
#define POSIF_PFLGE_EDIR_MASK     (1 << POSIF_PFLGE_EDIR_SHIFT)
#define POSIF_PFLGE_EPCLK_SHIFT   (12)                                          /* Bit 12 : Quadrature Period CLK interrupt Enable */
#define POSIF_PFLGE_EPCLK_MASK    (1 << POSIF_PFLGE_EPCLK_SHIFT)
#define POSIF_PFLGE_CHESEL_SHIFT  (16)                                          /* Bit 16 : Correct Hall Event Service Request Selector */
#define POSIF_PFLGE_CHESEL_MASK   (1 << POSIF_PFLGE_CHESEL_SHIFT)
#define POSIF_PFLGE_WHESEL_SHIFT  (17)                                          /* Bit 17 : Wrong Hall Event Service Request Selector */
#define POSIF_PFLGE_WHESEL_MASK   (1 << POSIF_PFLGE_WHESEL_SHIFT)
#define POSIF_PFLGE_HIESEL_SHIFT  (18)                                          /* Bit 18 : Hall Inputs Update Event Service Request Selector */
#define POSIF_PFLGE_HIESEL_MASK   (1 << POSIF_PFLGE_HIESEL_SHIFT)
#define POSIF_PFLGE_MSTSEL_SHIFT  (20)                                          /* Bit 20 : Multi-Channel pattern Update Event Service Request Selector */
#define POSIF_PFLGE_MSTSEL_MASK   (1 << POSIF_PFLGE_MSTSEL_SHIFT)
#define POSIF_PFLGE_INDSEL_SHIFT  (24)                                          /* Bit 24 : Quadrature Index Event Service Request Selector */
#define POSIF_PFLGE_INDSEL_MASK   (1 << POSIF_PFLGE_INDSEL_SHIFT)
#define POSIF_PFLGE_ERRSEL_SHIFT  (25)                                          /* Bit 25 : Quadrature Phase Error Event Service Request Selector */
#define POSIF_PFLGE_ERRSEL_MASK   (1 << POSIF_PFLGE_ERRSEL_SHIFT)
#define POSIF_PFLGE_CNTSEL_SHIFT  (26)                                          /* Bit 26 : Quadrature Clock Event Service Request Selector */
#define POSIF_PFLGE_CNTSEL_MASK   (1 << POSIF_PFLGE_CNTSEL_SHIFT)
#define POSIF_PFLGE_DIRSEL_SHIFT  (27)                                          /* Bit 27 : Quadrature Direction Update Event Service Request Selector */
#define POSIF_PFLGE_DIRSEL_MASK   (1 << POSIF_PFLGE_DIRSEL_SHIFT)
#define POSIF_PFLGE_PCLSEL_SHIFT  (28)                                          /* Bit 28 : Quadrature Period clock Event Service Request Selector */
#define POSIF_PFLGE_PCLSEL_MASK   (1 << POSIF_PFLGE_PCLSEL_SHIFT)

/* SPFLG - Interrupt set register */
#define POSIF_SPFLG_SCHE_SHIFT    (0)                                           /* Bit 0 : Correct Hall Event flag set */
#define POSIF_SPFLG_SCHE_MASK     (1 << POSIF_SPFLG_SCHE_SHIFT)
#define POSIF_SPFLG_SWHE_SHIFT    (1)                                           /* Bit 1 : Wrong Hall Event flag set */
#define POSIF_SPFLG_SWHE_MASK     (1 << POSIF_SPFLG_SWHE_SHIFT)
#define POSIF_SPFLG_SHIE_SHIFT    (2)                                           /* Bit 2 : Hall Inputs Update Event flag set */
#define POSIF_SPFLG_SHIE_MASK     (1 << POSIF_SPFLG_SHIE_SHIFT)
#define POSIF_SPFLG_SMST_SHIFT    (4)                                           /* Bit 4 : Multi-Channel Pattern shadow transfer flag set */
#define POSIF_SPFLG_SMST_MASK     (1 << POSIF_SPFLG_SMST_SHIFT)
#define POSIF_SPFLG_SINDX_SHIFT   (8)                                           /* Bit 8 : Quadrature Index flag set */
#define POSIF_SPFLG_SINDX_MASK    (1 << POSIF_SPFLG_SINDX_SHIFT)
#define POSIF_SPFLG_SERR_SHIFT    (9)                                           /* Bit 9 : Quadrature Phase Error flag set */
#define POSIF_SPFLG_SERR_MASK     (1 << POSIF_SPFLG_SERR_SHIFT)
#define POSIF_SPFLG_SCNT_SHIFT    (10)                                          /* Bit 10 : Quadrature CLK flag set */
#define POSIF_SPFLG_SCNT_MASK     (1 << POSIF_SPFLG_SCNT_SHIFT)
#define POSIF_SPFLG_SDIR_SHIFT    (11)                                          /* Bit 11 : Quadrature Direction flag set */
#define POSIF_SPFLG_SDIR_MASK     (1 << POSIF_SPFLG_SDIR_SHIFT)
#define POSIF_SPFLG_SPCLK_SHIFT   (12)                                          /* Bit 12 : Quadrature period clock flag set */
#define POSIF_SPFLG_SPCLK_MASK    (1 << POSIF_SPFLG_SPCLK_SHIFT)

/* RPFLG - Interrupt clear register */
#define POSIF_RPFLG_RCHE_SHIFT    (0)                                           /* Bit 0 : Correct Hall Event flag clear */
#define POSIF_RPFLG_RCHE_MASK     (1 << POSIF_RPFLG_RCHE_SHIFT)
#define POSIF_RPFLG_RWHE_SHIFT    (1)                                           /* Bit 1 : Wrong Hall Event flag clear */
#define POSIF_RPFLG_RWHE_MASK     (1 << POSIF_RPFLG_RWHE_SHIFT)
#define POSIF_RPFLG_RHIE_SHIFT    (2)                                           /* Bit 2 : Hall Inputs Update Event flag clear */
#define POSIF_RPFLG_RHIE_MASK     (1 << POSIF_RPFLG_RHIE_SHIFT)
#define POSIF_RPFLG_RMST_SHIFT    (4)                                           /* Bit 4 : Multi-Channel Pattern shadow transfer flag clear */
#define POSIF_RPFLG_RMST_MASK     (1 << POSIF_RPFLG_RMST_SHIFT)
#define POSIF_RPFLG_RINDX_SHIFT   (8)                                           /* Bit 8 : Quadrature Index flag clear */
#define POSIF_RPFLG_RINDX_MASK    (1 << POSIF_RPFLG_RINDX_SHIFT)
#define POSIF_RPFLG_RERR_SHIFT    (9)                                           /* Bit 9 : Quadrature Phase Error flag clear */
#define POSIF_RPFLG_RERR_MASK     (1 << POSIF_RPFLG_RERR_SHIFT)
#define POSIF_RPFLG_RCNT_SHIFT    (10)                                          /* Bit 10 : Quadrature CLK flag clear */
#define POSIF_RPFLG_RCNT_MASK     (1 << POSIF_RPFLG_RCNT_SHIFT)
#define POSIF_RPFLG_RDIR_SHIFT    (11)                                          /* Bit 11 : Quadrature Direction flag clear */
#define POSIF_RPFLG_RDIR_MASK     (1 << POSIF_RPFLG_RDIR_SHIFT)
#define POSIF_RPFLG_RPCLK_SHIFT   (12)                                          /* Bit 12 : Quadrature period clock flag clear */
#define POSIF_RPFLG_RPCLK_MASK    (1 << POSIF_RPFLG_RPCLK_SHIFT)

/* PDBG    */
#define POSIF_PDBG_QCSV_SHIFT     (0)                                           /* Quadrature Decoder Current state */
#define POSIF_PDBG_QCSV_MASK      (3 << POSIF_PDBG_QCSV_SHIFT)
#define POSIF_PDBG_QPSV_SHIFT     (2)                                           /* Quadrature Decoder Previous state */
#define POSIF_PDBG_QPSV_MASK      (3 << POSIF_PDBG_QPSV_SHIFT)
#define POSIF_PDBG_IVAL_SHIFT     (4)                                           /* Current Index Value */
#define POSIF_PDBG_IVAL_MASK      (1 << POSIF_PDBG_IVAL_SHIFT)
#define POSIF_PDBG_HSP_SHIFT      (5)                                           /* Hall Current Sampled Pattern */
#define POSIF_PDBG_HSP_MASK       (7 << POSIF_PDBG_HSP_SHIFT)
#define POSIF_PDBG_LPP0_SHIFT     (8)                                           /* Actual count of the Low Pass Filter for POSI0 */
#define POSIF_PDBG_LPP0_MASK      (0x3f << POSIF_PDBG_LPP0_SHIFT)
#define POSIF_PDBG_LPP1_SHIFT     (16)                                          /* Actual count of the Low Pass Filter for POSI1 */
#define POSIF_PDBG_LPP1_MASK      (0x3f << POSIF_PDBG_LPP1_SHIFT)
#define POSIF_PDBG_LPP2_SHIFT     (22)                                          /* Actual count of the Low Pass Filter for POSI2 */
#define POSIF_PDBG_LPP2_MASK      (0x3f << POSIF_PDBG_LPP2_SHIFT)