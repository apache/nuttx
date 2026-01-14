/****************************************************************************
 * arch/arm/src/xmc4/hardware/xmc4_eru.h
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

#ifndef __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_ERU_H
#define __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_ERU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/xmc4_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* ERU Kernel Registers */

#define XMC4_ERU_EXISEL_OFFSET  0x0000  /* ERU External Input Control Selection */
#define XMC4_ERU_EXICON0_OFFSET 0x0010  /* ERU External Input Control Selection */
#define XMC4_ERU_EXICON1_OFFSET 0x0014  /* ERU External Input Control Selection */
#define XMC4_ERU_EXICON2_OFFSET 0x0018  /* ERU External Input Control Selection */
#define XMC4_ERU_EXICON3_OFFSET 0x001C  /* ERU External Input Control Selection */
#define XMC4_ERU_EXOCON0_OFFSET 0x0020  /* ERU Output Control Register */
#define XMC4_ERU_EXOCON1_OFFSET 0x0024  /* ERU Output Control Register */
#define XMC4_ERU_EXOCON2_OFFSET 0x0028  /* ERU Output Control Register */
#define XMC4_ERU_EXOCON3_OFFSET 0x002C  /* ERU Output Control Register */

/* ERU0 Registers ***********************************************************/

#define XMC4_ERU0_EXISEL  (XMC4_ERU0_BASE + XMC4_ERU_EXISEL_OFFSET)
#define XMC4_ERU0_EXICON0 (XMC4_ERU0_BASE + XMC4_ERU_EXICON0_OFFSET)
#define XMC4_ERU0_EXICON1 (XMC4_ERU0_BASE + XMC4_ERU_EXICON1_OFFSET)
#define XMC4_ERU0_EXICON2 (XMC4_ERU0_BASE + XMC4_ERU_EXICON2_OFFSET)
#define XMC4_ERU0_EXICON3 (XMC4_ERU0_BASE + XMC4_ERU_EXICON3_OFFSET)
#define XMC4_ERU0_EXOCON0 (XMC4_ERU0_BASE + XMC4_ERU_EXOCON0_OFFSET)
#define XMC4_ERU0_EXOCON1 (XMC4_ERU0_BASE + XMC4_ERU_EXOCON1_OFFSET)
#define XMC4_ERU0_EXOCON2 (XMC4_ERU0_BASE + XMC4_ERU_EXOCON2_OFFSET)
#define XMC4_ERU0_EXOCON3 (XMC4_ERU0_BASE + XMC4_ERU_EXOCON3_OFFSET)

/* ERU1 Registers ***********************************************************/

#define XMC4_ERU1_EXISEL  (XMC4_ERU1_BASE + XMC4_ERU_EXISEL_OFFSET)
#define XMC4_ERU1_EXICON0 (XMC4_ERU1_BASE + XMC4_ERU_EXICON0_OFFSET)
#define XMC4_ERU1_EXICON1 (XMC4_ERU1_BASE + XMC4_ERU_EXICON1_OFFSET)
#define XMC4_ERU1_EXICON2 (XMC4_ERU1_BASE + XMC4_ERU_EXICON2_OFFSET)
#define XMC4_ERU1_EXICON3 (XMC4_ERU1_BASE + XMC4_ERU_EXICON3_OFFSET)
#define XMC4_ERU1_EXOCON0 (XMC4_ERU1_BASE + XMC4_ERU_EXOCON0_OFFSET)
#define XMC4_ERU1_EXOCON1 (XMC4_ERU1_BASE + XMC4_ERU_EXOCON1_OFFSET)
#define XMC4_ERU1_EXOCON2 (XMC4_ERU1_BASE + XMC4_ERU_EXOCON2_OFFSET)
#define XMC4_ERU1_EXOCON3 (XMC4_ERU1_BASE + XMC4_ERU_EXOCON3_OFFSET)

/* ERU Masks and Positions **************************************************/

/* ERU_EXISEL */

#define ERU_EXISEL_EXS0A_SHIFT     (0UL)          /* (Bit 0) Event Source Select for A0 (ERS0) */
#define ERU_EXISEL_EXS0A_MASK      (0x3UL)
#define ERU_EXISEL_EXS0B_SHIFT     (2UL)          /* (Bit 2) Event Source Select for B0 (ERS0) */
#define ERU_EXISEL_EXS0B_MASK      (0xcUL)
#define ERU_EXISEL_EXS1A_SHIFT     (4UL)          /* (Bit 4) Event Source Select for A1 (ERS1) */
#define ERU_EXISEL_EXS1A_MASK      (0x30UL)
#define ERU_EXISEL_EXS1B_SHIFT     (6UL)          /* (Bit 6) Event Source Select for B1 (ERS1) */
#define ERU_EXISEL_EXS1B_MASK      (0xc0UL)
#define ERU_EXISEL_EXS2A_SHIFT     (8UL)          /* (Bit 8) Event Source Select for A2 (ERS2) */
#define ERU_EXISEL_EXS2A_MASK      (0x300UL)
#define ERU_EXISEL_EXS2B_SHIFT     (10UL)         /* (Bit 10) Event Source Select for B2 (ERS2) */
#define ERU_EXISEL_EXS2B_MASK      (0xc00UL)
#define ERU_EXISEL_EXS3A_SHIFT     (12UL)         /* (Bit 12) Event Source Select for A3 (ERS3) */
#define ERU_EXISEL_EXS3A_MASK      (0x3000UL)
#define ERU_EXISEL_EXS3B_SHIFT     (14UL)         /* (Bit 14) Event Source Select for B3 (ERS3) */
#define ERU_EXISEL_EXS3B_MASK      (0xc000UL)

/* ERU_EXICON */

#define ERU_EXICON_PE_SHIFT        (0UL)          /* (Bit 0) Output Trigger Pulse Enable for ETLx */
#define ERU_EXICON_PE_MASK         (0x1UL)
#define ERU_EXICON_LD_SHIFT        (1UL)          /* (Bit 1) Rebuild Level Detection for Status Flag for ETLx */
#define ERU_EXICON_LD_MASK         (0x2UL)
#define ERU_EXICON_RE_SHIFT        (2UL)          /* (Bit 2) Rising Edge Detection Enable ETLx */
#define ERU_EXICON_RE_MASK         (0x4UL)
#define ERU_EXICON_FE_SHIFT        (3UL)          /* (Bit 3) Falling Edge Detection Enable ETLx */
#define ERU_EXICON_FE_MASK         (0x8UL)
#define ERU_EXICON_OCS_SHIFT       (4UL)          /* (Bit 4) Output Channel Select for ETLx Output Trigger Pulse */
#define ERU_EXICON_OCS_MASK        (0x70UL)
#define ERU_EXICON_FL_SHIFT        (7UL)          /* (Bit 7) Status Flag for ETLx */
#define ERU_EXICON_FL_MASK         (0x80UL)
#define ERU_EXICON_SS_SHIFT        (8UL)          /* (Bit 8) Input Source Select for ERSx */
#define ERU_EXICON_SS_MASK         (0x300UL)
#define ERU_EXICON_NA_SHIFT        (10UL)         /* (Bit 10) Input A Negation Select for ERSx */
#define ERU_EXICON_NA_MASK         (0x400UL)
#define ERU_EXICON_NB_SHIFT        (11UL)         /* (Bit 11) Input B Negation Select for ERSx */
#define ERU_EXICON_NB_MASK         (0x800UL)

/* ERU_EXOCON */

#define ERU_EXOCON_ISS_SHIFT       (0UL)          /* (Bit 0) Internal Trigger Source Selection */
#define ERU_EXOCON_ISS_MASK        (0x3UL)
#define ERU_EXOCON_GEEN_SHIFT      (2UL)          /* (Bit 2) Gating Event Enable */
#define ERU_EXOCON_GEEN_MASK       (0x4UL)
#define ERU_EXOCON_PDR_SHIFT       (3UL)          /* (Bit 3) Pattern Detection Result Flag */
#define ERU_EXOCON_PDR_MASK        (0x8UL)
#define ERU_EXOCON_GP_SHIFT        (4UL)          /* (Bit 4) Gating Selection for Pattern Detection Result */
#define ERU_EXOCON_GP_MASK         (0x30UL)
#define ERU_EXOCON_IPEN0_SHIFT     (12UL)         /* (Bit 12) Pattern Detection Enable for ETL0 */
#define ERU_EXOCON_IPEN0_MASK      (0x1000UL)
#define ERU_EXOCON_IPEN1_SHIFT     (13UL)         /* (Bit 13) Pattern Detection Enable for ETL1 */
#define ERU_EXOCON_IPEN1_MASK      (0x2000UL)
#define ERU_EXOCON_IPEN2_SHIFT     (14UL)         /* (Bit 14) Pattern Detection Enable for ETL2 */
#define ERU_EXOCON_IPEN2_MASK      (0x4000UL)
#define ERU_EXOCON_IPEN3_SHIFT     (15UL)         /* (Bit 15) Pattern Detection Enable for ETL3 */
#define ERU_EXOCON_IPEN3_MASK      (0x8000UL)

#endif /* __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_ERU_H */
