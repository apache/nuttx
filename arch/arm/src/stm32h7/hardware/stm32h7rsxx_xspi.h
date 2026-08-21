/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32h7rsxx_xspi.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_XSPI_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_XSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_XSPI_CR_OFFSET         0x0000 /* Control register */
#define STM32_XSPI_DCR1_OFFSET       0x0008 /* Device configuration 1 */
#define STM32_XSPI_DCR2_OFFSET       0x000c /* Device configuration 2 */
#define STM32_XSPI_DCR3_OFFSET       0x0010 /* Device configuration 3 */
#define STM32_XSPI_DCR4_OFFSET       0x0014 /* Device configuration 4 */
#define STM32_XSPI_SR_OFFSET         0x0020 /* Status register */
#define STM32_XSPI_FCR_OFFSET        0x0024 /* Flag clear register */
#define STM32_XSPI_DLR_OFFSET        0x0040 /* Data length register */
#define STM32_XSPI_AR_OFFSET         0x0048 /* Address register */
#define STM32_XSPI_DR_OFFSET         0x0050 /* Data register */
#define STM32_XSPI_PSMKR_OFFSET      0x0080 /* Polling status mask */
#define STM32_XSPI_PSMAR_OFFSET      0x0088 /* Polling status match */
#define STM32_XSPI_PIR_OFFSET        0x0090 /* Polling interval */
#define STM32_XSPI_CCR_OFFSET        0x0100 /* Communication configuration */
#define STM32_XSPI_TCR_OFFSET        0x0108 /* Timing configuration */
#define STM32_XSPI_IR_OFFSET         0x0110 /* Instruction register */
#define STM32_XSPI_ABR_OFFSET        0x0120 /* Alternate bytes register */
#define STM32_XSPI_LPTR_OFFSET       0x0130 /* Low-power timeout */
#define STM32_XSPI_WPCCR_OFFSET      0x0140 /* Wrap communication config */
#define STM32_XSPI_WPTCR_OFFSET      0x0148 /* Wrap timing config */
#define STM32_XSPI_WPIR_OFFSET       0x0150 /* Wrap instruction */
#define STM32_XSPI_WPABR_OFFSET      0x0160 /* Wrap alternate bytes */
#define STM32_XSPI_WCCR_OFFSET       0x0180 /* Write communication config */
#define STM32_XSPI_WTCR_OFFSET       0x0188 /* Write timing config */
#define STM32_XSPI_WIR_OFFSET        0x0190 /* Write instruction */
#define STM32_XSPI_WABR_OFFSET       0x01a0 /* Write alternate bytes */
#define STM32_XSPI_HLCR_OFFSET       0x0200 /* HyperBus latency config */
#define STM32_XSPI_CALFCR_OFFSET     0x0210 /* Full-cycle calibration */
#define STM32_XSPI_CALMR_OFFSET      0x0218 /* Master calibration */
#define STM32_XSPI_CALSOR_OFFSET     0x0220 /* Output calibration */
#define STM32_XSPI_CALSIR_OFFSET     0x0228 /* Input calibration */

#define STM32_XSPIM_CR_OFFSET        0x0000 /* XSPIM control */

/* Register Addresses *******************************************************/

#define STM32_XSPI1_CR               (STM32_XSPI1_R_BASE + STM32_XSPI_CR_OFFSET)
#define STM32_XSPI1_DCR1             (STM32_XSPI1_R_BASE + STM32_XSPI_DCR1_OFFSET)
#define STM32_XSPI1_DCR2             (STM32_XSPI1_R_BASE + STM32_XSPI_DCR2_OFFSET)
#define STM32_XSPI1_DCR3             (STM32_XSPI1_R_BASE + STM32_XSPI_DCR3_OFFSET)
#define STM32_XSPI1_DCR4             (STM32_XSPI1_R_BASE + STM32_XSPI_DCR4_OFFSET)
#define STM32_XSPI1_SR               (STM32_XSPI1_R_BASE + STM32_XSPI_SR_OFFSET)
#define STM32_XSPI1_FCR              (STM32_XSPI1_R_BASE + STM32_XSPI_FCR_OFFSET)
#define STM32_XSPI1_DLR              (STM32_XSPI1_R_BASE + STM32_XSPI_DLR_OFFSET)
#define STM32_XSPI1_AR               (STM32_XSPI1_R_BASE + STM32_XSPI_AR_OFFSET)
#define STM32_XSPI1_DR               (STM32_XSPI1_R_BASE + STM32_XSPI_DR_OFFSET)
#define STM32_XSPI1_PSMKR            (STM32_XSPI1_R_BASE + STM32_XSPI_PSMKR_OFFSET)
#define STM32_XSPI1_PSMAR            (STM32_XSPI1_R_BASE + STM32_XSPI_PSMAR_OFFSET)
#define STM32_XSPI1_PIR              (STM32_XSPI1_R_BASE + STM32_XSPI_PIR_OFFSET)
#define STM32_XSPI1_CCR              (STM32_XSPI1_R_BASE + STM32_XSPI_CCR_OFFSET)
#define STM32_XSPI1_TCR              (STM32_XSPI1_R_BASE + STM32_XSPI_TCR_OFFSET)
#define STM32_XSPI1_IR               (STM32_XSPI1_R_BASE + STM32_XSPI_IR_OFFSET)
#define STM32_XSPI1_ABR              (STM32_XSPI1_R_BASE + STM32_XSPI_ABR_OFFSET)
#define STM32_XSPI1_LPTR             (STM32_XSPI1_R_BASE + STM32_XSPI_LPTR_OFFSET)
#define STM32_XSPI1_WPCCR            (STM32_XSPI1_R_BASE + STM32_XSPI_WPCCR_OFFSET)
#define STM32_XSPI1_WPTCR            (STM32_XSPI1_R_BASE + STM32_XSPI_WPTCR_OFFSET)
#define STM32_XSPI1_WPIR             (STM32_XSPI1_R_BASE + STM32_XSPI_WPIR_OFFSET)
#define STM32_XSPI1_WPABR            (STM32_XSPI1_R_BASE + STM32_XSPI_WPABR_OFFSET)
#define STM32_XSPI1_WCCR             (STM32_XSPI1_R_BASE + STM32_XSPI_WCCR_OFFSET)
#define STM32_XSPI1_WTCR             (STM32_XSPI1_R_BASE + STM32_XSPI_WTCR_OFFSET)
#define STM32_XSPI1_WIR              (STM32_XSPI1_R_BASE + STM32_XSPI_WIR_OFFSET)
#define STM32_XSPI1_WABR             (STM32_XSPI1_R_BASE + STM32_XSPI_WABR_OFFSET)
#define STM32_XSPI1_HLCR             (STM32_XSPI1_R_BASE + STM32_XSPI_HLCR_OFFSET)
#define STM32_XSPI1_CALFCR           (STM32_XSPI1_R_BASE + STM32_XSPI_CALFCR_OFFSET)
#define STM32_XSPI1_CALMR            (STM32_XSPI1_R_BASE + STM32_XSPI_CALMR_OFFSET)
#define STM32_XSPI1_CALSOR           (STM32_XSPI1_R_BASE + STM32_XSPI_CALSOR_OFFSET)
#define STM32_XSPI1_CALSIR           (STM32_XSPI1_R_BASE + STM32_XSPI_CALSIR_OFFSET)

#define STM32_XSPI2_CR               (STM32_XSPI2_R_BASE + STM32_XSPI_CR_OFFSET)
#define STM32_XSPI2_DCR1             (STM32_XSPI2_R_BASE + STM32_XSPI_DCR1_OFFSET)
#define STM32_XSPI2_DCR2             (STM32_XSPI2_R_BASE + STM32_XSPI_DCR2_OFFSET)
#define STM32_XSPI2_DCR3             (STM32_XSPI2_R_BASE + STM32_XSPI_DCR3_OFFSET)
#define STM32_XSPI2_DCR4             (STM32_XSPI2_R_BASE + STM32_XSPI_DCR4_OFFSET)
#define STM32_XSPI2_SR               (STM32_XSPI2_R_BASE + STM32_XSPI_SR_OFFSET)
#define STM32_XSPI2_FCR              (STM32_XSPI2_R_BASE + STM32_XSPI_FCR_OFFSET)
#define STM32_XSPI2_DLR              (STM32_XSPI2_R_BASE + STM32_XSPI_DLR_OFFSET)
#define STM32_XSPI2_AR               (STM32_XSPI2_R_BASE + STM32_XSPI_AR_OFFSET)
#define STM32_XSPI2_DR               (STM32_XSPI2_R_BASE + STM32_XSPI_DR_OFFSET)
#define STM32_XSPI2_PSMKR            (STM32_XSPI2_R_BASE + STM32_XSPI_PSMKR_OFFSET)
#define STM32_XSPI2_PSMAR            (STM32_XSPI2_R_BASE + STM32_XSPI_PSMAR_OFFSET)
#define STM32_XSPI2_PIR              (STM32_XSPI2_R_BASE + STM32_XSPI_PIR_OFFSET)
#define STM32_XSPI2_CCR              (STM32_XSPI2_R_BASE + STM32_XSPI_CCR_OFFSET)
#define STM32_XSPI2_TCR              (STM32_XSPI2_R_BASE + STM32_XSPI_TCR_OFFSET)
#define STM32_XSPI2_IR               (STM32_XSPI2_R_BASE + STM32_XSPI_IR_OFFSET)
#define STM32_XSPI2_ABR              (STM32_XSPI2_R_BASE + STM32_XSPI_ABR_OFFSET)
#define STM32_XSPI2_LPTR             (STM32_XSPI2_R_BASE + STM32_XSPI_LPTR_OFFSET)
#define STM32_XSPI2_WPCCR            (STM32_XSPI2_R_BASE + STM32_XSPI_WPCCR_OFFSET)
#define STM32_XSPI2_WPTCR            (STM32_XSPI2_R_BASE + STM32_XSPI_WPTCR_OFFSET)
#define STM32_XSPI2_WPIR             (STM32_XSPI2_R_BASE + STM32_XSPI_WPIR_OFFSET)
#define STM32_XSPI2_WPABR            (STM32_XSPI2_R_BASE + STM32_XSPI_WPABR_OFFSET)
#define STM32_XSPI2_WCCR             (STM32_XSPI2_R_BASE + STM32_XSPI_WCCR_OFFSET)
#define STM32_XSPI2_WTCR             (STM32_XSPI2_R_BASE + STM32_XSPI_WTCR_OFFSET)
#define STM32_XSPI2_WIR              (STM32_XSPI2_R_BASE + STM32_XSPI_WIR_OFFSET)
#define STM32_XSPI2_WABR             (STM32_XSPI2_R_BASE + STM32_XSPI_WABR_OFFSET)
#define STM32_XSPI2_HLCR             (STM32_XSPI2_R_BASE + STM32_XSPI_HLCR_OFFSET)
#define STM32_XSPI2_CALFCR           (STM32_XSPI2_R_BASE + STM32_XSPI_CALFCR_OFFSET)
#define STM32_XSPI2_CALMR            (STM32_XSPI2_R_BASE + STM32_XSPI_CALMR_OFFSET)
#define STM32_XSPI2_CALSOR           (STM32_XSPI2_R_BASE + STM32_XSPI_CALSOR_OFFSET)
#define STM32_XSPI2_CALSIR           (STM32_XSPI2_R_BASE + STM32_XSPI_CALSIR_OFFSET)

#define STM32_XSPIM_CR               (STM32_XSPIM_BASE + STM32_XSPIM_CR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Control register *********************************************************/

#define XSPI_CR_EN                   (1 << 0)  /* XSPI enable */
#define XSPI_CR_ABORT                (1 << 1)  /* Abort request */
#define XSPI_CR_DMAEN                (1 << 2)  /* DMA enable */
#define XSPI_CR_TCEN                 (1 << 3)  /* Timeout enable */
#define XSPI_CR_DMM                  (1 << 6)  /* Dual-memory mode */
#define XSPI_CR_FTHRES_SHIFT         (8)       /* FIFO threshold level */
#define XSPI_CR_FTHRES_MASK          (31 << XSPI_CR_FTHRES_SHIFT)
#  define XSPI_CR_FTHRES(n)          (((n) - 1) << XSPI_CR_FTHRES_SHIFT)
#define XSPI_CR_TEIE                 (1 << 16) /* Transfer-error interrupt */
#define XSPI_CR_TCIE                 (1 << 17) /* Transfer-complete interrupt */
#define XSPI_CR_FTIE                 (1 << 18) /* FIFO-threshold interrupt */
#define XSPI_CR_SMIE                 (1 << 19) /* Status-match interrupt */
#define XSPI_CR_TOIE                 (1 << 20) /* Timeout interrupt */
#define XSPI_CR_APMS                 (1 << 22) /* Automatic-polling stop */
#define XSPI_CR_PMM                  (1 << 23) /* Polling match mode */
#define XSPI_CR_CSSEL                (1 << 24) /* Chip-select selection */
#define XSPI_CR_FMODE_SHIFT          (28)      /* Functional mode */
#define XSPI_CR_FMODE_MASK           (3 << XSPI_CR_FMODE_SHIFT)
#  define XSPI_CR_FMODE_WRITE        (0 << XSPI_CR_FMODE_SHIFT)
#  define XSPI_CR_FMODE_READ         (1 << XSPI_CR_FMODE_SHIFT)
#  define XSPI_CR_FMODE_POLLING      (2 << XSPI_CR_FMODE_SHIFT)
#  define XSPI_CR_FMODE_MEMORY       (3 << XSPI_CR_FMODE_SHIFT)
#define XSPI_CR_MSEL_SHIFT           (30)      /* Memory select */
#define XSPI_CR_MSEL_MASK            (3 << XSPI_CR_MSEL_SHIFT)

/* Device configuration registers *******************************************/

#define XSPI_DCR1_CKMODE             (1 << 0) /* Clock mode 3 */
#define XSPI_DCR1_FRCK               (1 << 1) /* Free-running clock */
#define XSPI_DCR1_CSHT_SHIFT         (8)      /* Chip-select high time */
#define XSPI_DCR1_CSHT_MASK          (0x3f << XSPI_DCR1_CSHT_SHIFT)
#  define XSPI_DCR1_CSHT(n)          (((n) - 1) << XSPI_DCR1_CSHT_SHIFT)
#define XSPI_DCR1_DEVSIZE_SHIFT      (16)     /* Device size */
#define XSPI_DCR1_DEVSIZE_MASK       (31 << XSPI_DCR1_DEVSIZE_SHIFT)
#  define XSPI_DCR1_DEVSIZE(n)       ((n) << XSPI_DCR1_DEVSIZE_SHIFT)
#define XSPI_DCR1_MTYP_SHIFT         (24)     /* Memory type */
#define XSPI_DCR1_MTYP_MASK          (7 << XSPI_DCR1_MTYP_SHIFT)
#  define XSPI_DCR1_MTYP(n)          ((n) << XSPI_DCR1_MTYP_SHIFT)

#define XSPI_DCR2_PRESCALER_SHIFT    (0)  /* Clock prescaler */
#define XSPI_DCR2_PRESCALER_MASK     (0xff << XSPI_DCR2_PRESCALER_SHIFT)
#  define XSPI_DCR2_PRESCALER(n)     ((n) << XSPI_DCR2_PRESCALER_SHIFT)
#define XSPI_DCR2_WRAPSIZE_SHIFT     (16) /* Wrap size */
#define XSPI_DCR2_WRAPSIZE_MASK      (7 << XSPI_DCR2_WRAPSIZE_SHIFT)
#  define XSPI_DCR2_WRAPSIZE(n)      ((n) << XSPI_DCR2_WRAPSIZE_SHIFT)

#define XSPI_DCR3_MAXTRAN_SHIFT      (0)  /* Maximum transfer */
#define XSPI_DCR3_MAXTRAN_MASK       (0xff << XSPI_DCR3_MAXTRAN_SHIFT)
#  define XSPI_DCR3_MAXTRAN(n)       ((n) << XSPI_DCR3_MAXTRAN_SHIFT)
#define XSPI_DCR3_CSBOUND_SHIFT      (16) /* CS boundary */
#define XSPI_DCR3_CSBOUND_MASK       (31 << XSPI_DCR3_CSBOUND_SHIFT)
#  define XSPI_DCR3_CSBOUND(n)       ((n) << XSPI_DCR3_CSBOUND_SHIFT)

#define XSPI_DCR4_REFRESH_SHIFT      (0) /* Refresh rate */
#define XSPI_DCR4_REFRESH_MASK       (0xffffffff << XSPI_DCR4_REFRESH_SHIFT)
#  define XSPI_DCR4_REFRESH(n)       ((n) << XSPI_DCR4_REFRESH_SHIFT)

/* Status and flag clear registers ******************************************/

#define XSPI_SR_TEF                  (1 << 0) /* Transfer error */
#define XSPI_SR_TCF                  (1 << 1) /* Transfer complete */
#define XSPI_SR_FTF                  (1 << 2) /* FIFO threshold */
#define XSPI_SR_SMF                  (1 << 3) /* Status match */
#define XSPI_SR_TOF                  (1 << 4) /* Timeout */
#define XSPI_SR_BUSY                 (1 << 5) /* Busy */
#define XSPI_SR_FLEVEL_SHIFT         (8)      /* FIFO level */
#define XSPI_SR_FLEVEL_MASK          (0x3f << XSPI_SR_FLEVEL_SHIFT)

#define XSPI_FCR_CTEF                (1 << 0) /* Clear transfer error */
#define XSPI_FCR_CTCF                (1 << 1) /* Clear transfer complete */
#define XSPI_FCR_CSMF                (1 << 3) /* Clear status match */
#define XSPI_FCR_CTOF                (1 << 4) /* Clear timeout */
#define XSPI_FCR_ALL                 (XSPI_FCR_CTEF | XSPI_FCR_CTCF | XSPI_FCR_CSMF | XSPI_FCR_CTOF)

/* Data, address and polling registers **************************************/

#define XSPI_DLR_DL_SHIFT            (0) /* Data length */
#define XSPI_DLR_DL_MASK             (0xffffffff << XSPI_DLR_DL_SHIFT)
#define XSPI_AR_ADDRESS_SHIFT        (0) /* Address */
#define XSPI_AR_ADDRESS_MASK         (0xffffffff << XSPI_AR_ADDRESS_SHIFT)
#define XSPI_DR_DATA_SHIFT           (0) /* Data */
#define XSPI_DR_DATA_MASK            (0xffffffff << XSPI_DR_DATA_SHIFT)
#define XSPI_PSMKR_MASK_SHIFT        (0) /* Status mask */
#define XSPI_PSMKR_MASK_MASK         (0xffffffff << XSPI_PSMKR_MASK_SHIFT)
#define XSPI_PSMAR_MATCH_SHIFT       (0) /* Status match */
#define XSPI_PSMAR_MATCH_MASK        (0xffffffff << XSPI_PSMAR_MATCH_SHIFT)
#define XSPI_PIR_INTERVAL_SHIFT      (0) /* Polling interval */
#define XSPI_PIR_INTERVAL_MASK       (0xffff << XSPI_PIR_INTERVAL_SHIFT)
#  define XSPI_PIR_INTERVAL(n)       ((n) << XSPI_PIR_INTERVAL_SHIFT)

/* Communication and timing registers ***************************************/

#define XSPI_CCR_IMODE_SHIFT         (0)       /* Instruction mode */
#define XSPI_CCR_IMODE_MASK          (7 << XSPI_CCR_IMODE_SHIFT)
#  define XSPI_CCR_IMODE(n)          ((n) << XSPI_CCR_IMODE_SHIFT)
#define XSPI_CCR_IDTR                (1 << 3)  /* Instruction DTR */
#define XSPI_CCR_ISIZE_SHIFT         (4)       /* Instruction size */
#define XSPI_CCR_ISIZE_MASK          (3 << XSPI_CCR_ISIZE_SHIFT)
#  define XSPI_CCR_ISIZE(n)          ((n) << XSPI_CCR_ISIZE_SHIFT)
#define XSPI_CCR_ADMODE_SHIFT        (8)       /* Address mode */
#define XSPI_CCR_ADMODE_MASK         (7 << XSPI_CCR_ADMODE_SHIFT)
#  define XSPI_CCR_ADMODE(n)         ((n) << XSPI_CCR_ADMODE_SHIFT)
#define XSPI_CCR_ADDTR               (1 << 11) /* Address DTR */
#define XSPI_CCR_ADSIZE_SHIFT        (12)      /* Address size */
#define XSPI_CCR_ADSIZE_MASK         (3 << XSPI_CCR_ADSIZE_SHIFT)
#  define XSPI_CCR_ADSIZE(n)         ((n) << XSPI_CCR_ADSIZE_SHIFT)
#define XSPI_CCR_ABMODE_SHIFT        (16)      /* Alternate bytes mode */
#define XSPI_CCR_ABMODE_MASK         (7 << XSPI_CCR_ABMODE_SHIFT)
#  define XSPI_CCR_ABMODE(n)         ((n) << XSPI_CCR_ABMODE_SHIFT)
#define XSPI_CCR_ABDTR               (1 << 19) /* Alternate bytes DTR */
#define XSPI_CCR_ABSIZE_SHIFT        (20)      /* Alternate bytes size */
#define XSPI_CCR_ABSIZE_MASK         (3 << XSPI_CCR_ABSIZE_SHIFT)
#  define XSPI_CCR_ABSIZE(n)         ((n) << XSPI_CCR_ABSIZE_SHIFT)
#define XSPI_CCR_DMODE_SHIFT         (24)      /* Data mode */
#define XSPI_CCR_DMODE_MASK          (7 << XSPI_CCR_DMODE_SHIFT)
#  define XSPI_CCR_DMODE(n)          ((n) << XSPI_CCR_DMODE_SHIFT)
#define XSPI_CCR_DDTR                (1 << 27) /* Data DTR */
#define XSPI_CCR_DQSE                (1 << 29) /* DQS enable */

#define XSPI_TCR_DCYC_SHIFT          (0)       /* Dummy cycles */
#define XSPI_TCR_DCYC_MASK           (31 << XSPI_TCR_DCYC_SHIFT)
#  define XSPI_TCR_DCYC(n)           ((n) << XSPI_TCR_DCYC_SHIFT)
#define XSPI_TCR_DHQC                (1 << 28) /* Delay hold quarter cycle */
#define XSPI_TCR_SSHIFT              (1 << 30) /* Sample shift */

#define XSPI_IR_INSTRUCTION_SHIFT    (0) /* Instruction */
#define XSPI_IR_INSTRUCTION_MASK     (0xffffffff << XSPI_IR_INSTRUCTION_SHIFT)
#define XSPI_ABR_ALTERNATE_SHIFT     (0) /* Alternate bytes */
#define XSPI_ABR_ALTERNATE_MASK      (0xffffffff << XSPI_ABR_ALTERNATE_SHIFT)
#define XSPI_LPTR_TIMEOUT_SHIFT      (0) /* Timeout period */
#define XSPI_LPTR_TIMEOUT_MASK       (0xffff << XSPI_LPTR_TIMEOUT_SHIFT)
#  define XSPI_LPTR_TIMEOUT(n)       ((n) << XSPI_LPTR_TIMEOUT_SHIFT)

/* Wrap communication and timing registers **********************************/

#define XSPI_WPCCR_IMODE_SHIFT       (0)       /* Instruction mode */
#define XSPI_WPCCR_IMODE_MASK        (7 << XSPI_WPCCR_IMODE_SHIFT)
#  define XSPI_WPCCR_IMODE(n)        ((n) << XSPI_WPCCR_IMODE_SHIFT)
#define XSPI_WPCCR_IDTR              (1 << 3)  /* Instruction DTR */
#define XSPI_WPCCR_ISIZE_SHIFT       (4)       /* Instruction size */
#define XSPI_WPCCR_ISIZE_MASK        (3 << XSPI_WPCCR_ISIZE_SHIFT)
#  define XSPI_WPCCR_ISIZE(n)        ((n) << XSPI_WPCCR_ISIZE_SHIFT)
#define XSPI_WPCCR_ADMODE_SHIFT      (8)       /* Address mode */
#define XSPI_WPCCR_ADMODE_MASK       (7 << XSPI_WPCCR_ADMODE_SHIFT)
#  define XSPI_WPCCR_ADMODE(n)       ((n) << XSPI_WPCCR_ADMODE_SHIFT)
#define XSPI_WPCCR_ADDTR             (1 << 11) /* Address DTR */
#define XSPI_WPCCR_ADSIZE_SHIFT      (12)      /* Address size */
#define XSPI_WPCCR_ADSIZE_MASK       (3 << XSPI_WPCCR_ADSIZE_SHIFT)
#  define XSPI_WPCCR_ADSIZE(n)       ((n) << XSPI_WPCCR_ADSIZE_SHIFT)
#define XSPI_WPCCR_ABMODE_SHIFT      (16)      /* Alternate bytes mode */
#define XSPI_WPCCR_ABMODE_MASK       (7 << XSPI_WPCCR_ABMODE_SHIFT)
#  define XSPI_WPCCR_ABMODE(n)       ((n) << XSPI_WPCCR_ABMODE_SHIFT)
#define XSPI_WPCCR_ABDTR             (1 << 19) /* Alternate bytes DTR */
#define XSPI_WPCCR_ABSIZE_SHIFT      (20)      /* Alternate bytes size */
#define XSPI_WPCCR_ABSIZE_MASK       (3 << XSPI_WPCCR_ABSIZE_SHIFT)
#  define XSPI_WPCCR_ABSIZE(n)       ((n) << XSPI_WPCCR_ABSIZE_SHIFT)
#define XSPI_WPCCR_DMODE_SHIFT       (24)      /* Data mode */
#define XSPI_WPCCR_DMODE_MASK        (7 << XSPI_WPCCR_DMODE_SHIFT)
#  define XSPI_WPCCR_DMODE(n)        ((n) << XSPI_WPCCR_DMODE_SHIFT)
#define XSPI_WPCCR_DDTR              (1 << 27) /* Data DTR */
#define XSPI_WPCCR_DQSE              (1 << 29) /* DQS enable */

#define XSPI_WPTCR_DCYC_SHIFT        (0)       /* Dummy cycles */
#define XSPI_WPTCR_DCYC_MASK         (31 << XSPI_WPTCR_DCYC_SHIFT)
#  define XSPI_WPTCR_DCYC(n)         ((n) << XSPI_WPTCR_DCYC_SHIFT)
#define XSPI_WPTCR_DHQC              (1 << 28) /* Delay hold quarter cycle */
#define XSPI_WPTCR_SSHIFT            (1 << 30) /* Sample shift */

#define XSPI_WPIR_INSTRUCTION_SHIFT  (0) /* Instruction */
#define XSPI_WPIR_INSTRUCTION_MASK   (0xffffffff << XSPI_WPIR_INSTRUCTION_SHIFT)
#define XSPI_WPABR_ALTERNATE_SHIFT   (0) /* Alternate bytes */
#define XSPI_WPABR_ALTERNATE_MASK    (0xffffffff << XSPI_WPABR_ALTERNATE_SHIFT)

/* Write communication and timing registers *********************************/

#define XSPI_WCCR_IMODE_SHIFT        (0)       /* Instruction mode */
#define XSPI_WCCR_IMODE_MASK         (7 << XSPI_WCCR_IMODE_SHIFT)
#  define XSPI_WCCR_IMODE(n)         ((n) << XSPI_WCCR_IMODE_SHIFT)
#define XSPI_WCCR_IDTR               (1 << 3)  /* Instruction DTR */
#define XSPI_WCCR_ISIZE_SHIFT        (4)       /* Instruction size */
#define XSPI_WCCR_ISIZE_MASK         (3 << XSPI_WCCR_ISIZE_SHIFT)
#  define XSPI_WCCR_ISIZE(n)         ((n) << XSPI_WCCR_ISIZE_SHIFT)
#define XSPI_WCCR_ADMODE_SHIFT       (8)       /* Address mode */
#define XSPI_WCCR_ADMODE_MASK        (7 << XSPI_WCCR_ADMODE_SHIFT)
#  define XSPI_WCCR_ADMODE(n)        ((n) << XSPI_WCCR_ADMODE_SHIFT)
#define XSPI_WCCR_ADDTR              (1 << 11) /* Address DTR */
#define XSPI_WCCR_ADSIZE_SHIFT       (12)      /* Address size */
#define XSPI_WCCR_ADSIZE_MASK        (3 << XSPI_WCCR_ADSIZE_SHIFT)
#  define XSPI_WCCR_ADSIZE(n)        ((n) << XSPI_WCCR_ADSIZE_SHIFT)
#define XSPI_WCCR_ABMODE_SHIFT       (16)      /* Alternate bytes mode */
#define XSPI_WCCR_ABMODE_MASK        (7 << XSPI_WCCR_ABMODE_SHIFT)
#  define XSPI_WCCR_ABMODE(n)        ((n) << XSPI_WCCR_ABMODE_SHIFT)
#define XSPI_WCCR_ABDTR              (1 << 19) /* Alternate bytes DTR */
#define XSPI_WCCR_ABSIZE_SHIFT       (20)      /* Alternate bytes size */
#define XSPI_WCCR_ABSIZE_MASK        (3 << XSPI_WCCR_ABSIZE_SHIFT)
#  define XSPI_WCCR_ABSIZE(n)        ((n) << XSPI_WCCR_ABSIZE_SHIFT)
#define XSPI_WCCR_DMODE_SHIFT        (24)      /* Data mode */
#define XSPI_WCCR_DMODE_MASK         (7 << XSPI_WCCR_DMODE_SHIFT)
#  define XSPI_WCCR_DMODE(n)         ((n) << XSPI_WCCR_DMODE_SHIFT)
#define XSPI_WCCR_DDTR               (1 << 27) /* Data DTR */
#define XSPI_WCCR_DQSE               (1 << 29) /* DQS enable */

#define XSPI_WTCR_DCYC_SHIFT         (0) /* Dummy cycles */
#define XSPI_WTCR_DCYC_MASK          (31 << XSPI_WTCR_DCYC_SHIFT)
#  define XSPI_WTCR_DCYC(n)          ((n) << XSPI_WTCR_DCYC_SHIFT)

#define XSPI_WIR_INSTRUCTION_SHIFT   (0) /* Instruction */
#define XSPI_WIR_INSTRUCTION_MASK    (0xffffffff << XSPI_WIR_INSTRUCTION_SHIFT)
#define XSPI_WABR_ALTERNATE_SHIFT    (0) /* Alternate bytes */
#define XSPI_WABR_ALTERNATE_MASK     (0xffffffff << XSPI_WABR_ALTERNATE_SHIFT)

/* HyperBus latency register ************************************************/

#define XSPI_HLCR_LM                 (1 << 0) /* Latency mode */
#define XSPI_HLCR_WZL                (1 << 1) /* Write zero latency */
#define XSPI_HLCR_TACC_SHIFT         (8)      /* Access time */
#define XSPI_HLCR_TACC_MASK          (0xff << XSPI_HLCR_TACC_SHIFT)
#define XSPI_HLCR_TRWR_SHIFT         (16)     /* Read write recovery time */
#define XSPI_HLCR_TRWR_MASK          (0xff << XSPI_HLCR_TRWR_SHIFT)

/* Calibration registers ****************************************************/

#define XSPI_CALFCR_FINE_SHIFT       (0)       /* Fine calibration */
#define XSPI_CALFCR_FINE_MASK        (0x7f << XSPI_CALFCR_FINE_SHIFT)
#  define XSPI_CALFCR_FINE(n)        ((n) << XSPI_CALFCR_FINE_SHIFT)
#define XSPI_CALFCR_COARSE_SHIFT     (16)      /* Coarse calibration */
#define XSPI_CALFCR_COARSE_MASK      (31 << XSPI_CALFCR_COARSE_SHIFT)
#  define XSPI_CALFCR_COARSE(n)      ((n) << XSPI_CALFCR_COARSE_SHIFT)
#define XSPI_CALFCR_CALMAX           (1 << 31) /* Calibration max value */

#define XSPI_CALMR_FINE_SHIFT        (0)  /* Fine calibration */
#define XSPI_CALMR_FINE_MASK         (0x7f << XSPI_CALMR_FINE_SHIFT)
#  define XSPI_CALMR_FINE(n)         ((n) << XSPI_CALMR_FINE_SHIFT)
#define XSPI_CALMR_COARSE_SHIFT      (16) /* Coarse calibration */
#define XSPI_CALMR_COARSE_MASK       (31 << XSPI_CALMR_COARSE_SHIFT)
#  define XSPI_CALMR_COARSE(n)       ((n) << XSPI_CALMR_COARSE_SHIFT)

#define XSPI_CALSOR_FINE_SHIFT       (0)  /* Fine calibration */
#define XSPI_CALSOR_FINE_MASK        (0x7f << XSPI_CALSOR_FINE_SHIFT)
#  define XSPI_CALSOR_FINE(n)        ((n) << XSPI_CALSOR_FINE_SHIFT)
#define XSPI_CALSOR_COARSE_SHIFT     (16) /* Coarse calibration */
#define XSPI_CALSOR_COARSE_MASK      (31 << XSPI_CALSOR_COARSE_SHIFT)
#  define XSPI_CALSOR_COARSE(n)      ((n) << XSPI_CALSOR_COARSE_SHIFT)

#define XSPI_CALSIR_FINE_SHIFT       (0)  /* Fine calibration */
#define XSPI_CALSIR_FINE_MASK        (0x7f << XSPI_CALSIR_FINE_SHIFT)
#  define XSPI_CALSIR_FINE(n)        ((n) << XSPI_CALSIR_FINE_SHIFT)
#define XSPI_CALSIR_COARSE_SHIFT     (16) /* Coarse calibration */
#define XSPI_CALSIR_COARSE_MASK      (31 << XSPI_CALSIR_COARSE_SHIFT)
#  define XSPI_CALSIR_COARSE(n)      ((n) << XSPI_CALSIR_COARSE_SHIFT)

/* XSPI I/O manager control register ****************************************/

#define XSPIM_CR_MUXEN               (1 << 0) /* Multiplexed mode */
#define XSPIM_CR_MODE                (1 << 1) /* Multiplexing mode */
#define XSPIM_CR_CSSEL_OVR_EN        (1 << 4) /* Chip-select override */
#define XSPIM_CR_CSSEL_OVR_O1        (1 << 5) /* XSPI1 uses nCS2 */
#define XSPIM_CR_CSSEL_OVR_O2        (1 << 6) /* XSPI2 uses nCS2 */
#define XSPIM_CR_REQ2ACK_SHIFT       (16)
#define XSPIM_CR_REQ2ACK_MASK        (0xff << XSPIM_CR_REQ2ACK_SHIFT)
#  define XSPIM_CR_REQ2ACK(n)        (((n) - 1) << XSPIM_CR_REQ2ACK_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_XSPI_H */
