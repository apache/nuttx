/****************************************************************************
 * arch/arm/src/common/stm32/hardware/stm32_tamp.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_TAMP_H
#define __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_TAMP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_TAMP_CR1_OFFSET     0x0000    /* TAMP control register 1 */
#define STM32_TAMP_CR2_OFFSET     0x0004    /* TAMP control register 2 */
#define STM32_TAMP_CR3_OFFSET     0x0008    /* TAMP control register 3 */
#define STM32_TAMP_FLTCR_OFFSET   0x000c    /* TAMP filter control register */
#define STM32_TAMP_ATCR1_OFFSET   0x0010    /* TAMP active tamper control register 1 */
#define STM32_TAMP_ATSEEDR_OFFSET 0x0014    /* TAMP active tamper seed register */
#define STM32_TAMP_ATOR_OFFSET    0x0018    /* TAMP active tamper output register */
#define STM32_TAMP_ATCR2_OFFSET   0x001c    /* TAMP active tamper control register 2 */
#define STM32_TAMP_SECCFGR_OFFSET 0x0020    /* TAMP secure mode control register */
#define STM32_TAMP_PRIVCFGR_OFFSET 0x0024   /* TAMP privilege mode control register */
#define STM32_TAMP_IER_OFFSET     0x002c    /* TAMP interrupt enable register */
#define STM32_TAMP_SR_OFFSET      0x0030    /* TAMP status register */
#define STM32_TAMP_MISR_OFFSET    0x0034    /* TAMP masked interrupt status register */
#define STM32_TAMP_SMISR_OFFSET   0x0038    /* TAMP secure masked int. status register */
#define STM32_TAMP_SCR_OFFSET     0x003c    /* TAMP status clear register */
#define STM32_TAMP_COUNT1R_OFFSET 0x0040    /* TAMP monotonic counter 1 register */
#define STM32_TAMP_OR_OFFSET      0x0050    /* TAMP option register */
#define STM32_TAMP_RPCFGR_OFFSET  0x0054    /* TAMP resource protection config. register */

#define STM32_TAMP_BKPR_OFFSET(n) (0x0100 + ((n) << 2)) /* TAMP backup registers 0-31 */

/* Register Addresses *******************************************************/

#define STM32_TAMP_CR1            (STM32_TAMP_BASE + STM32_TAMP_CR1_OFFSET)
#define STM32_TAMP_CR2            (STM32_TAMP_BASE + STM32_TAMP_CR2_OFFSET)
#define STM32_TAMP_CR3            (STM32_TAMP_BASE + STM32_TAMP_CR3_OFFSET)
#define STM32_TAMP_FLTCR          (STM32_TAMP_BASE + STM32_TAMP_FLTCR_OFFSET)
#define STM32_TAMP_ATCR1          (STM32_TAMP_BASE + STM32_TAMP_ATCR1_OFFSET)
#define STM32_TAMP_ATSEEDR        (STM32_TAMP_BASE + STM32_TAMP_ATSEEDR_OFFSET)
#define STM32_TAMP_ATOR           (STM32_TAMP_BASE + STM32_TAMP_ATOR_OFFSET)
#define STM32_TAMP_ATCR2          (STM32_TAMP_BASE + STM32_TAMP_ATCR2_OFFSET)
#define STM32_TAMP_SECCFGR        (STM32_TAMP_BASE + STM32_TAMP_SECCFGR_OFFSET)
#define STM32_TAMP_PRIVCFGR       (STM32_TAMP_BASE + STM32_TAMP_PRIVCFGR_OFFSET)
#define STM32_TAMP_IER            (STM32_TAMP_BASE + STM32_TAMP_IER_OFFSET)
#define STM32_TAMP_SR             (STM32_TAMP_BASE + STM32_TAMP_SR_OFFSET)
#define STM32_TAMP_MISR           (STM32_TAMP_BASE + STM32_TAMP_MISR_OFFSET)
#define STM32_TAMP_SMISR          (STM32_TAMP_BASE + STM32_TAMP_SMISR_OFFSET)
#define STM32_TAMP_SCR            (STM32_TAMP_BASE + STM32_TAMP_SCR_OFFSET)
#define STM32_TAMP_COUNT1R        (STM32_TAMP_BASE + STM32_TAMP_COUNT1R_OFFSET)
#define STM32_TAMP_OR             (STM32_TAMP_BASE + STM32_TAMP_OR_OFFSET)
#define STM32_TAMP_RPCFGR         (STM32_TAMP_BASE + STM32_TAMP_RPCFGR_OFFSET)
#define STM32_TAMP_BKPR(n)        (STM32_TAMP_BASE + STM32_TAMP_BKPR_OFFSET(n))

/* Register Bitfield Definitions ********************************************/

/* TAMP control register 1 */

#define TAMP_CR1_TAMP1E           (1 << 0)  /* Bit 0: Tamper 1 detection enable */
#define TAMP_CR1_TAMP2E           (1 << 1)  /* Bit 1: Tamper 2 detection enable */
#define TAMP_CR1_TAMP3E           (1 << 2)  /* Bit 2: Tamper 3 detection enable */
#define TAMP_CR1_TAMP4E           (1 << 3)  /* Bit 3: Tamper 4 detection enable */
#define TAMP_CR1_TAMP5E           (1 << 4)  /* Bit 4: Tamper 5 detection enable */
#define TAMP_CR1_TAMP6E           (1 << 5)  /* Bit 5: Tamper 6 detection enable */
#define TAMP_CR1_TAMP7E           (1 << 6)  /* Bit 6: Tamper 7 detection enable */
#define TAMP_CR1_ITAMP1E          (1 << 16) /* Bit 16: Internal tamper 1 enable */
#define TAMP_CR1_ITAMP2E          (1 << 17) /* Bit 17: Internal tamper 2 enable */
#define TAMP_CR1_ITAMP3E          (1 << 18) /* Bit 18: Internal tamper 3 enable */
#define TAMP_CR1_ITAMP4E          (1 << 19) /* Bit 19: Internal tamper 4 enable */
#define TAMP_CR1_ITAMP5E          (1 << 20) /* Bit 20: Internal tamper 5 enable */
#define TAMP_CR1_ITAMP6E          (1 << 21) /* Bit 21: Internal tamper 6 enable */
#define TAMP_CR1_ITAMP7E          (1 << 22) /* Bit 22: Internal tamper 7 enable */
#define TAMP_CR1_ITAMP8E          (1 << 23) /* Bit 23: Internal tamper 8 enable */
#define TAMP_CR1_ITAMP9E          (1 << 24) /* Bit 24: Internal tamper 9 enable */
#define TAMP_CR1_ITAMP11E         (1 << 26) /* Bit 26: Internal tamper 11 enable */

/* TAMP control register 2 */

#define TAMP_CR2_TAMP1POM         (1 << 0)  /* Bit 0: Tamper 1 potential mode */
#define TAMP_CR2_TAMP2POM         (1 << 1)  /* Bit 1: Tamper 2 potential mode */
#define TAMP_CR2_TAMP3POM         (1 << 2)  /* Bit 2: Tamper 3 potential mode */
#define TAMP_CR2_TAMP4POM         (1 << 3)  /* Bit 3: Tamper 4 potential mode */
#define TAMP_CR2_TAMP5POM         (1 << 4)  /* Bit 4: Tamper 5 potential mode */
#define TAMP_CR2_TAMP6POM         (1 << 5)  /* Bit 5: Tamper 6 potential mode */
#define TAMP_CR2_TAMP7POM         (1 << 6)  /* Bit 6: Tamper 7 potential mode */
#define TAMP_CR2_TAMP1MSK         (1 << 16) /* Bit 16: Tamper 1 mask */
#define TAMP_CR2_TAMP2MSK         (1 << 17) /* Bit 17: Tamper 2 mask */
#define TAMP_CR2_TAMP3MSK         (1 << 18) /* Bit 18: Tamper 3 mask */
#define TAMP_CR2_BKBLOCK          (1 << 22) /* Bit 22: Backup registers and SRAMs access block */
#define TAMP_CR2_BKERASE          (1 << 23) /* Bit 23: Backup registers and SRAMs erase */
#define TAMP_CR2_TAMP1TRG         (1 << 24) /* Bit 24: Tamper 1 active level */
#define TAMP_CR2_TAMP2TRG         (1 << 25) /* Bit 25: Tamper 2 active level */
#define TAMP_CR2_TAMP3TRG         (1 << 26) /* Bit 26: Tamper 3 active level */
#define TAMP_CR2_TAMP4TRG         (1 << 27) /* Bit 27: Tamper 4 active level */
#define TAMP_CR2_TAMP5TRG         (1 << 28) /* Bit 28: Tamper 5 active level */
#define TAMP_CR2_TAMP6TRG         (1 << 29) /* Bit 29: Tamper 6 active level */
#define TAMP_CR2_TAMP7TRG         (1 << 30) /* Bit 30: Tamper 7 active level */

/* TAMP control register 3 */

#define TAMP_CR3_ITAMP1POM        (1 << 0)  /* Bit 0: Internal tamper 1 potential mode */
#define TAMP_CR3_ITAMP2POM        (1 << 1)  /* Bit 1: Internal tamper 2 potential mode */
#define TAMP_CR3_ITAMP3POM        (1 << 2)  /* Bit 2: Internal tamper 3 potential mode */
#define TAMP_CR3_ITAMP4POM        (1 << 3)  /* Bit 3: Internal tamper 4 potential mode */
#define TAMP_CR3_ITAMP5POM        (1 << 4)  /* Bit 4: Internal tamper 5 potential mode */
#define TAMP_CR3_ITAMP6POM        (1 << 5)  /* Bit 5: Internal tamper 6 potential mode */
#define TAMP_CR3_ITAMP7POM        (1 << 6)  /* Bit 6: Internal tamper 7 potential mode */
#define TAMP_CR3_ITAMP8POM        (1 << 7)  /* Bit 7: Internal tamper 8 potential mode */
#define TAMP_CR3_ITAMP9POM        (1 << 8)  /* Bit 8: Internal tamper 9 potential mode */
#define TAMP_CR3_ITAMP11POM       (1 << 10) /* Bit 10: Internal tamper 11 potential mode */

/* TAMP filter control register */

#define TAMP_FLTCR_TAMPFREQ_SHIFT (0)       /* Bits 0-2: Tamper sampling frequency */
#define TAMP_FLTCR_TAMPFREQ_MASK  (7 << TAMP_FLTCR_TAMPFREQ_SHIFT)
#define TAMP_FLTCR_TAMPFLT_SHIFT  (3)       /* Bits 3-4: Tamper filter count */
#define TAMP_FLTCR_TAMPFLT_MASK   (3 << TAMP_FLTCR_TAMPFLT_SHIFT)
#define TAMP_FLTCR_TAMPPRCH_SHIFT (5)       /* Bits 5-6: Tamper precharge duration */
#define TAMP_FLTCR_TAMPPRCH_MASK  (3 << TAMP_FLTCR_TAMPPRCH_SHIFT)
#define TAMP_FLTCR_TAMPPUDIS      (1 << 7)  /* Bit 7: Tamper pull-up disable */

/* TAMP active tamper control register 1 */

#define TAMP_ATCR1_TAMP1AM        (1 << 0)  /* Bit 0: Tamper 1 active mode */
#define TAMP_ATCR1_TAMP2AM        (1 << 1)  /* Bit 1: Tamper 2 active mode */
#define TAMP_ATCR1_TAMP3AM        (1 << 2)  /* Bit 2: Tamper 3 active mode */
#define TAMP_ATCR1_TAMP4AM        (1 << 3)  /* Bit 3: Tamper 4 active mode */
#define TAMP_ATCR1_TAMP5AM        (1 << 4)  /* Bit 4: Tamper 5 active mode */
#define TAMP_ATCR1_TAMP6AM        (1 << 5)  /* Bit 5: Tamper 6 active mode */
#define TAMP_ATCR1_TAMP7AM        (1 << 6)  /* Bit 6: Tamper 7 active mode */
#define TAMP_ATCR1_ATOSEL1_SHIFT  (8)       /* Bits 8-9: Active tamper shared output 1 selection */
#define TAMP_ATCR1_ATOSEL1_MASK   (3 << TAMP_ATCR1_ATOSEL1_SHIFT)
#define TAMP_ATCR1_ATOSEL2_SHIFT  (10)      /* Bits 10-11: Active tamper shared output 2 selection */
#define TAMP_ATCR1_ATOSEL2_MASK   (3 << TAMP_ATCR1_ATOSEL2_SHIFT)
#define TAMP_ATCR1_ATOSEL3_SHIFT  (12)      /* Bits 12-13: Active tamper shared output 3 selection */
#define TAMP_ATCR1_ATOSEL3_MASK   (3 << TAMP_ATCR1_ATOSEL3_SHIFT)
#define TAMP_ATCR1_ATOSEL4_SHIFT  (14)      /* Bits 14-15: Active tamper shared output 4 selection */
#define TAMP_ATCR1_ATOSEL4_MASK   (3 << TAMP_ATCR1_ATOSEL4_SHIFT)
#define TAMP_ATCR1_ATCKSEL_SHIFT  (16)      /* Bits 16-19: Active tamper RTC clock selection */
#define TAMP_ATCR1_ATCKSEL_MASK   (15 << TAMP_ATCR1_ATCKSEL_SHIFT)
#define TAMP_ATCR1_ATPER_SHIFT    (24)      /* Bits 24-26: Active tamper output change period */
#define TAMP_ATCR1_ATPER_MASK     (7 << TAMP_ATCR1_ATPER_SHIFT)
#define TAMP_ATCR1_ATOSHARE       (1 << 30) /* Bit 30: Active tamper output sharing */
#define TAMP_ATCR1_FLTEN          (1 << 31) /* Bit 31: Active tamper filter enable */

/* TAMP active tamper seed register */

#define TAMP_ATSEEDR_SEED_SHIFT   (0)       /* Bits 0-31: Pseudo-random generator seed value */
#define TAMP_ATSEEDR_SEED_MASK    (0xffffffff << TAMP_ATSEEDR_SEED_SHIFT)

/* TAMP active tamper output register */

#define TAMP_ATOR_PRNG_SHIFT      (0)       /* Bits 0-7: Pseudo-random generator value */
#define TAMP_ATOR_PRNG_MASK       (0xff << TAMP_ATOR_PRNG_SHIFT)

/* TAMP active tamper control register 2 */

#define TAMP_ATCR2_ATOSEL1_SHIFT  (8)       /* Bits 8-10: Active tamper shared output 1 selection */
#define TAMP_ATCR2_ATOSEL1_MASK   (7 << TAMP_ATCR2_ATOSEL1_SHIFT)
#define TAMP_ATCR2_ATOSEL2_SHIFT  (11)      /* Bits 11-13: Active tamper shared output 2 selection */
#define TAMP_ATCR2_ATOSEL2_MASK   (7 << TAMP_ATCR2_ATOSEL2_SHIFT)
#define TAMP_ATCR2_ATOSEL3_SHIFT  (14)      /* Bits 14-16: Active tamper shared output 3 selection */
#define TAMP_ATCR2_ATOSEL3_MASK   (7 << TAMP_ATCR2_ATOSEL3_SHIFT)
#define TAMP_ATCR2_ATOSEL4_SHIFT  (17)      /* Bits 17-19: Active tamper shared output 4 selection */
#define TAMP_ATCR2_ATOSEL4_MASK   (7 << TAMP_ATCR2_ATOSEL4_SHIFT)
#define TAMP_ATCR2_ATOSEL5_SHIFT  (20)      /* Bits 20-22: Active tamper shared output 5 selection */
#define TAMP_ATCR2_ATOSEL5_MASK   (7 << TAMP_ATCR2_ATOSEL5_SHIFT)
#define TAMP_ATCR2_ATOSEL6_SHIFT  (23)      /* Bits 23-25: Active tamper shared output 6 selection */
#define TAMP_ATCR2_ATOSEL6_MASK   (7 << TAMP_ATCR2_ATOSEL6_SHIFT)
#define TAMP_ATCR2_ATOSEL7_SHIFT  (26)      /* Bits 26-28: Active tamper shared output 7 selection */
#define TAMP_ATCR2_ATOSEL7_MASK   (7 << TAMP_ATCR2_ATOSEL7_SHIFT)

/* TAMP secure mode control register */

#define TAMP_SECCFGR_BKPRWSEC_SHIFT (0)     /* Bits 0-7: Backup registers read/write protection */
#define TAMP_SECCFGR_BKPRWSEC_MASK (0xff << TAMP_SECCFGR_BKPRWSEC_SHIFT)
#define TAMP_SECCFGR_CNT1SEC      (1 << 15) /* Bit 15: Monotonic counter 1 secure protection */
#define TAMP_SECCFGR_BKPWSEC_SHIFT (16)     /* Bits 16-23: Backup registers write protection */
#define TAMP_SECCFGR_BKPWSEC_MASK (0xff << TAMP_SECCFGR_BKPWSEC_SHIFT)
#define TAMP_SECCFGR_BHKLOCK      (1 << 30) /* Bit 30: Boot hardware key lock */
#define TAMP_SECCFGR_TAMPSEC      (1 << 31) /* Bit 31: Tamper protection */

/* TAMP privilege mode control register */

#define TAMP_PRIVCFGR_CNT1PRIV    (1 << 15) /* Bit 15: Monotonic counter 1 privilege protection */
#define TAMP_PRIVCFGR_BKPRWPRIV   (1 << 29) /* Bit 29: Backup registers zone 1 privilege */
#define TAMP_PRIVCFGR_BKPWPRIV    (1 << 30) /* Bit 30: Backup registers zone 2 privilege */
#define TAMP_PRIVCFGR_TAMPPRIV    (1 << 31) /* Bit 31: Tamper privilege protection */

/* TAMP interrupt enable register */

#define TAMP_IER_TAMP1IE          (1 << 0)  /* Bit 0: Tamper 1 interrupt enable */
#define TAMP_IER_TAMP2IE          (1 << 1)  /* Bit 1: Tamper 2 interrupt enable */
#define TAMP_IER_TAMP3IE          (1 << 2)  /* Bit 2: Tamper 3 interrupt enable */
#define TAMP_IER_TAMP4IE          (1 << 3)  /* Bit 3: Tamper 4 interrupt enable */
#define TAMP_IER_TAMP5IE          (1 << 4)  /* Bit 4: Tamper 5 interrupt enable */
#define TAMP_IER_TAMP6IE          (1 << 5)  /* Bit 5: Tamper 6 interrupt enable */
#define TAMP_IER_TAMP7IE          (1 << 6)  /* Bit 6: Tamper 7 interrupt enable */
#define TAMP_IER_ITAMP1IE         (1 << 16) /* Bit 16: Internal tamper 1 interrupt enable */
#define TAMP_IER_ITAMP2IE         (1 << 17) /* Bit 17: Internal tamper 2 interrupt enable */
#define TAMP_IER_ITAMP3IE         (1 << 18) /* Bit 18: Internal tamper 3 interrupt enable */
#define TAMP_IER_ITAMP4IE         (1 << 19) /* Bit 19: Internal tamper 4 interrupt enable */
#define TAMP_IER_ITAMP5IE         (1 << 20) /* Bit 20: Internal tamper 5 interrupt enable */
#define TAMP_IER_ITAMP6IE         (1 << 21) /* Bit 21: Internal tamper 6 interrupt enable */
#define TAMP_IER_ITAMP7IE         (1 << 22) /* Bit 22: Internal tamper 7 interrupt enable */
#define TAMP_IER_ITAMP8IE         (1 << 23) /* Bit 23: Internal tamper 8 interrupt enable */
#define TAMP_IER_ITAMP9IE         (1 << 24) /* Bit 24: Internal tamper 9 interrupt enable */
#define TAMP_IER_ITAMP11IE        (1 << 26) /* Bit 26: Internal tamper 11 interrupt enable */

/* TAMP status register */

#define TAMP_SR_TAMP1F            (1 << 0)  /* Bit 0: Tamper 1 detection flag */
#define TAMP_SR_TAMP2F            (1 << 1)  /* Bit 1: Tamper 2 detection flag */
#define TAMP_SR_TAMP3F            (1 << 2)  /* Bit 2: Tamper 3 detection flag */
#define TAMP_SR_TAMP4F            (1 << 3)  /* Bit 3: Tamper 4 detection flag */
#define TAMP_SR_TAMP5F            (1 << 4)  /* Bit 4: Tamper 5 detection flag */
#define TAMP_SR_TAMP6F            (1 << 5)  /* Bit 5: Tamper 6 detection flag */
#define TAMP_SR_TAMP7F            (1 << 6)  /* Bit 6: Tamper 7 detection flag */
#define TAMP_SR_ITAMP1F           (1 << 16) /* Bit 16: Internal tamper 1 flag */
#define TAMP_SR_ITAMP2F           (1 << 17) /* Bit 17: Internal tamper 2 flag */
#define TAMP_SR_ITAMP3F           (1 << 18) /* Bit 18: Internal tamper 3 flag */
#define TAMP_SR_ITAMP4F           (1 << 19) /* Bit 19: Internal tamper 4 flag */
#define TAMP_SR_ITAMP5F           (1 << 20) /* Bit 20: Internal tamper 5 flag */
#define TAMP_SR_ITAMP6F           (1 << 21) /* Bit 21: Internal tamper 6 flag */
#define TAMP_SR_ITAMP7F           (1 << 22) /* Bit 22: Internal tamper 7 flag */
#define TAMP_SR_ITAMP8F           (1 << 23) /* Bit 23: Internal tamper 8 flag */
#define TAMP_SR_ITAMP9F           (1 << 24) /* Bit 24: Internal tamper 9 flag */
#define TAMP_SR_ITAMP11F          (1 << 26) /* Bit 26: Internal tamper 11 flag */

/* TAMP masked interrupt status register */

#define TAMP_MISR_TAMP1MF         (1 << 0)  /* Bit 0: Tamper 1 masked flag */
#define TAMP_MISR_TAMP2MF         (1 << 1)  /* Bit 1: Tamper 2 masked flag */
#define TAMP_MISR_TAMP3MF         (1 << 2)  /* Bit 2: Tamper 3 masked flag */
#define TAMP_MISR_TAMP4MF         (1 << 3)  /* Bit 3: Tamper 4 masked flag */
#define TAMP_MISR_TAMP5MF         (1 << 4)  /* Bit 4: Tamper 5 masked flag */
#define TAMP_MISR_TAMP6MF         (1 << 5)  /* Bit 5: Tamper 6 masked flag */
#define TAMP_MISR_TAMP7MF         (1 << 6)  /* Bit 6: Tamper 7 masked flag */
#define TAMP_MISR_ITAMP1MF        (1 << 16) /* Bit 16: Internal tamper 1 masked flag */
#define TAMP_MISR_ITAMP2MF        (1 << 17) /* Bit 17: Internal tamper 2 masked flag */
#define TAMP_MISR_ITAMP3MF        (1 << 18) /* Bit 18: Internal tamper 3 masked flag */
#define TAMP_MISR_ITAMP4MF        (1 << 19) /* Bit 19: Internal tamper 4 masked flag */
#define TAMP_MISR_ITAMP5MF        (1 << 20) /* Bit 20: Internal tamper 5 masked flag */
#define TAMP_MISR_ITAMP6MF        (1 << 21) /* Bit 21: Internal tamper 6 masked flag */
#define TAMP_MISR_ITAMP7MF        (1 << 22) /* Bit 22: Internal tamper 7 masked flag */
#define TAMP_MISR_ITAMP8MF        (1 << 23) /* Bit 23: Internal tamper 8 masked flag */
#define TAMP_MISR_ITAMP9MF        (1 << 24) /* Bit 24: Internal tamper 9 masked flag */
#define TAMP_MISR_ITAMP11MF       (1 << 26) /* Bit 26: Internal tamper 11 masked flag */

/* TAMP secure masked interrupt status register */

#define TAMP_SMISR_TAMP1MF        (1 << 0)  /* Bit 0: Tamper 1 secure masked flag */
#define TAMP_SMISR_TAMP2MF        (1 << 1)  /* Bit 1: Tamper 2 secure masked flag */
#define TAMP_SMISR_TAMP3MF        (1 << 2)  /* Bit 2: Tamper 3 secure masked flag */
#define TAMP_SMISR_TAMP4MF        (1 << 3)  /* Bit 3: Tamper 4 secure masked flag */
#define TAMP_SMISR_TAMP5MF        (1 << 4)  /* Bit 4: Tamper 5 secure masked flag */
#define TAMP_SMISR_TAMP6MF        (1 << 5)  /* Bit 5: Tamper 6 secure masked flag */
#define TAMP_SMISR_TAMP7MF        (1 << 6)  /* Bit 6: Tamper 7 secure masked flag */
#define TAMP_SMISR_ITAMP1MF       (1 << 16) /* Bit 16: Int. tamper 1 secure masked flag */
#define TAMP_SMISR_ITAMP2MF       (1 << 17) /* Bit 17: Int. tamper 2 secure masked flag */
#define TAMP_SMISR_ITAMP3MF       (1 << 18) /* Bit 18: Int. tamper 3 secure masked flag */
#define TAMP_SMISR_ITAMP4MF       (1 << 19) /* Bit 19: Int. tamper 4 secure masked flag */
#define TAMP_SMISR_ITAMP5MF       (1 << 20) /* Bit 20: Int. tamper 5 secure masked flag */
#define TAMP_SMISR_ITAMP6MF       (1 << 21) /* Bit 21: Int. tamper 6 secure masked flag */
#define TAMP_SMISR_ITAMP7MF       (1 << 22) /* Bit 22: Int. tamper 7 secure masked flag */
#define TAMP_SMISR_ITAMP8MF       (1 << 23) /* Bit 23: Int. tamper 8 secure masked flag */
#define TAMP_SMISR_ITAMP9MF       (1 << 24) /* Bit 24: Int. tamper 9 secure masked flag */
#define TAMP_SMISR_ITAMP11MF      (1 << 26) /* Bit 26: Int. tamper 11 secure masked flag */

/* TAMP status clear register */

#define TAMP_SCR_CTAMP1F          (1 << 0)  /* Bit 0: Clear tamper 1 detection flag */
#define TAMP_SCR_CTAMP2F          (1 << 1)  /* Bit 1: Clear tamper 2 detection flag */
#define TAMP_SCR_CTAMP3F          (1 << 2)  /* Bit 2: Clear tamper 3 detection flag */
#define TAMP_SCR_CTAMP4F          (1 << 3)  /* Bit 3: Clear tamper 4 detection flag */
#define TAMP_SCR_CTAMP5F          (1 << 4)  /* Bit 4: Clear tamper 5 detection flag */
#define TAMP_SCR_CTAMP6F          (1 << 5)  /* Bit 5: Clear tamper 6 detection flag */
#define TAMP_SCR_CTAMP7F          (1 << 6)  /* Bit 6: Clear tamper 7 detection flag */
#define TAMP_SCR_CTAMP8F          (1 << 7)  /* Bit 7: Clear tamper 8 detection flag */
#define TAMP_SCR_CITAMP1F         (1 << 16) /* Bit 16: Clear internal tamper 1 flag */
#define TAMP_SCR_CITAMP2F         (1 << 17) /* Bit 17: Clear internal tamper 2 flag */
#define TAMP_SCR_CITAMP3F         (1 << 18) /* Bit 18: Clear internal tamper 3 flag */
#define TAMP_SCR_CITAMP4F         (1 << 19) /* Bit 19: Clear internal tamper 4 flag */
#define TAMP_SCR_CITAMP5F         (1 << 20) /* Bit 20: Clear internal tamper 5 flag */
#define TAMP_SCR_CITAMP6F         (1 << 21) /* Bit 21: Clear internal tamper 6 flag */
#define TAMP_SCR_CITAMP7F         (1 << 22) /* Bit 22: Clear internal tamper 7 flag */
#define TAMP_SCR_CITAMP8F         (1 << 23) /* Bit 23: Clear internal tamper 8 flag */
#define TAMP_SCR_CITAMP9F         (1 << 24) /* Bit 24: Clear internal tamper 9 flag */
#define TAMP_SCR_CITAMP11F        (1 << 26) /* Bit 26: Clear internal tamper 11 flag */

/* TAMP monotonic counter 1 register */

#define TAMP_COUNT1R_COUNT_SHIFT  (0)       /* Bits 0-31: Monotonic counter value */
#define TAMP_COUNT1R_COUNT_MASK   (0xffffffff << TAMP_COUNT1R_COUNT_SHIFT)

/* TAMP option register */

#define TAMP_OR_VCOREMEN          (1 << 0)  /* Bit 0: VCORE monitoring enable */

/* TAMP resource protection configuration register */

#define TAMP_RPCFGR_RPCFG0        (1 << 0)  /* Bit 0: Configurable resource 0 protection */

#endif /* __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_TAMP_H */
