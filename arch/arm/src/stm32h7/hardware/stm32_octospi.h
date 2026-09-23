/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_octospi.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_OCTOSPI_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_OCTOSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* OCTOSPI register offsets (RM0468) */

#define STM32_OCTOSPI_CR_OFFSET     0x0000
#define STM32_OCTOSPI_DCR1_OFFSET   0x0008
#define STM32_OCTOSPI_DCR2_OFFSET   0x000c
#define STM32_OCTOSPI_DCR3_OFFSET   0x0010
#define STM32_OCTOSPI_DCR4_OFFSET   0x0014
#define STM32_OCTOSPI_SR_OFFSET     0x0020
#define STM32_OCTOSPI_FCR_OFFSET    0x0024
#define STM32_OCTOSPI_DLR_OFFSET    0x0040
#define STM32_OCTOSPI_AR_OFFSET     0x0048
#define STM32_OCTOSPI_DR_OFFSET     0x0050
#define STM32_OCTOSPI_CCR_OFFSET    0x0100
#define STM32_OCTOSPI_TCR_OFFSET    0x0108
#define STM32_OCTOSPI_WCCR_OFFSET   0x0180
#define STM32_OCTOSPI_HLCR_OFFSET   0x0200

/* OCTOSPI2 register addresses */

#define STM32_OCTOSPI2_CR    (STM32_OCTOSPI2_BASE + STM32_OCTOSPI_CR_OFFSET)
#define STM32_OCTOSPI2_DCR1  (STM32_OCTOSPI2_BASE + STM32_OCTOSPI_DCR1_OFFSET)
#define STM32_OCTOSPI2_DCR2  (STM32_OCTOSPI2_BASE + STM32_OCTOSPI_DCR2_OFFSET)
#define STM32_OCTOSPI2_DCR3  (STM32_OCTOSPI2_BASE + STM32_OCTOSPI_DCR3_OFFSET)
#define STM32_OCTOSPI2_DCR4  (STM32_OCTOSPI2_BASE + STM32_OCTOSPI_DCR4_OFFSET)
#define STM32_OCTOSPI2_SR    (STM32_OCTOSPI2_BASE + STM32_OCTOSPI_SR_OFFSET)
#define STM32_OCTOSPI2_FCR   (STM32_OCTOSPI2_BASE + STM32_OCTOSPI_FCR_OFFSET)
#define STM32_OCTOSPI2_DLR   (STM32_OCTOSPI2_BASE + STM32_OCTOSPI_DLR_OFFSET)
#define STM32_OCTOSPI2_AR    (STM32_OCTOSPI2_BASE + STM32_OCTOSPI_AR_OFFSET)
#define STM32_OCTOSPI2_DR    (STM32_OCTOSPI2_BASE + STM32_OCTOSPI_DR_OFFSET)
#define STM32_OCTOSPI2_CCR   (STM32_OCTOSPI2_BASE + STM32_OCTOSPI_CCR_OFFSET)
#define STM32_OCTOSPI2_TCR   (STM32_OCTOSPI2_BASE + STM32_OCTOSPI_TCR_OFFSET)
#define STM32_OCTOSPI2_WCCR  (STM32_OCTOSPI2_BASE + STM32_OCTOSPI_WCCR_OFFSET)
#define STM32_OCTOSPI2_HLCR  (STM32_OCTOSPI2_BASE + STM32_OCTOSPI_HLCR_OFFSET)

/* Control and status */

#define OCTOSPI_CR_EN                (1 << 0)
#define OCTOSPI_CR_FTHRES(n)         (((n) - 1) << 8)
#define OCTOSPI_CR_FMODE_MEMORYMAP   (3 << 28)
#define OCTOSPI_SR_BUSY              (1 << 5)

/* Device configuration */

#define OCTOSPI_DCR1_DLYBYP          (1 << 3)
#define OCTOSPI_DCR1_CSHT(n)         (((n) - 1) << 8)
#define OCTOSPI_DCR1_DEVSIZE(n)      (((n) - 1) << 16)
#define OCTOSPI_DCR1_MTYP_HYPERBUS   (4 << 24)
#define OCTOSPI_DCR2_PRESCALER(n)    ((n) - 1)
#define OCTOSPI_DCR3_CSBOUND(n)      ((n) << 16)

/* HyperBus uses eight data/address lines, double data rate and RWDS. */

#define OCTOSPI_CCR_ADMODE_8LINES    (4 << 8)
#define OCTOSPI_CCR_ADDTR            (1 << 11)
#define OCTOSPI_CCR_ADSIZE_32BITS    (3 << 12)
#define OCTOSPI_CCR_DMODE_8LINES     (4 << 24)
#define OCTOSPI_CCR_DDTR             (1 << 27)
#define OCTOSPI_CCR_DQSE             (1 << 29)
#define OCTOSPI_TCR_DHQC             (1 << 28)
#define OCTOSPI_HLCR_LM              (1 << 0)
#define OCTOSPI_HLCR_TACC(n)         ((n) << 8)
#define OCTOSPI_HLCR_TRWR(n)         ((n) << 16)

/* I/O manager: route port 2 to OCTOSPI2. */

#define STM32_OCTOSPIM_P2CR          (STM32_OCTOSPIM_BASE + 0x0008)
#define OCTOSPIM_PCR_CLKEN          (1 << 0)
#define OCTOSPIM_PCR_CLKSRC_OSPI2   (1 << 1)
#define OCTOSPIM_PCR_DQSEN          (1 << 4)
#define OCTOSPIM_PCR_DQSSRC_OSPI2   (1 << 5)
#define OCTOSPIM_PCR_NCSEN          (1 << 8)
#define OCTOSPIM_PCR_NCSSRC_OSPI2   (1 << 9)
#define OCTOSPIM_PCR_IOLEN          (1 << 16)
#define OCTOSPIM_PCR_IOLSRC_OSPI2   (2 << 17)
#define OCTOSPIM_PCR_IOHEN          (1 << 24)
#define OCTOSPIM_PCR_IOHSRC_OSPI2   (3 << 25)

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_OCTOSPI_H */
