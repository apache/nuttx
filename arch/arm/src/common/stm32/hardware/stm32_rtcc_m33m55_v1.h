/****************************************************************************
 * arch/arm/src/common/stm32/hardware/stm32_rtcc_m33m55_v1.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_RTCC_M33M55_V1_H
#define __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_RTCC_M33M55_V1_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_RTC_TR_OFFSET       0x0000    /* RTC time register */
#define STM32_RTC_DR_OFFSET       0x0004    /* RTC date register */
#define STM32_RTC_SSR_OFFSET      0x0008    /* RTC sub second register */
#define STM32_RTC_ICSR_OFFSET     0x000c    /* RTC init control and status register */
#define STM32_RTC_PRER_OFFSET     0x0010    /* RTC prescaler register */
#define STM32_RTC_WUTR_OFFSET     0x0014    /* RTC wakeup timer register */
#define STM32_RTC_CR_OFFSET       0x0018    /* RTC control register */
#define STM32_RTC_PRIVCFGR_OFFSET 0x001c    /* RTC privilege mode control register */
#define STM32_RTC_SECCFGR_OFFSET  0x0020    /* RTC secure mode control register */
#define STM32_RTC_WPR_OFFSET      0x0024    /* RTC write protection register */
#define STM32_RTC_CALR_OFFSET     0x0028    /* RTC calibration register */
#define STM32_RTC_SHIFTR_OFFSET   0x002c    /* RTC shift control register */
#define STM32_RTC_TSTR_OFFSET     0x0030    /* RTC timestamp time register */
#define STM32_RTC_TSDR_OFFSET     0x0034    /* RTC timestamp date register */
#define STM32_RTC_TSSSR_OFFSET    0x0038    /* RTC timestamp sub second register */
#define STM32_RTC_ALRMAR_OFFSET   0x0040    /* RTC alarm A register */
#define STM32_RTC_ALRMASSR_OFFSET 0x0044    /* RTC alarm A sub second register */
#define STM32_RTC_ALRMBR_OFFSET   0x0048    /* RTC alarm B register */
#define STM32_RTC_ALRMBSSR_OFFSET 0x004c    /* RTC alarm B sub second register */
#define STM32_RTC_SR_OFFSET       0x0050    /* RTC status register */
#define STM32_RTC_MISR_OFFSET     0x0054    /* RTC masked interrupt status register */
#define STM32_RTC_SMISR_OFFSET    0x0058    /* RTC secure masked interrupt status reg */
#define STM32_RTC_SCR_OFFSET      0x005c    /* RTC status clear register */
#define STM32_RTC_ALRABINR_OFFSET 0x0070    /* RTC alarm A binary mode register */
#define STM32_RTC_ALRBBINR_OFFSET 0x0074    /* RTC alarm B binary mode register */

/* Register Addresses *******************************************************/

#define STM32_RTC_TR              (STM32_RTC_BASE + STM32_RTC_TR_OFFSET)
#define STM32_RTC_DR              (STM32_RTC_BASE + STM32_RTC_DR_OFFSET)
#define STM32_RTC_SSR             (STM32_RTC_BASE + STM32_RTC_SSR_OFFSET)
#define STM32_RTC_ICSR            (STM32_RTC_BASE + STM32_RTC_ICSR_OFFSET)
#define STM32_RTC_PRER            (STM32_RTC_BASE + STM32_RTC_PRER_OFFSET)
#define STM32_RTC_WUTR            (STM32_RTC_BASE + STM32_RTC_WUTR_OFFSET)
#define STM32_RTC_CR              (STM32_RTC_BASE + STM32_RTC_CR_OFFSET)
#define STM32_RTC_PRIVCFGR        (STM32_RTC_BASE + STM32_RTC_PRIVCFGR_OFFSET)
#define STM32_RTC_SECCFGR         (STM32_RTC_BASE + STM32_RTC_SECCFGR_OFFSET)
#define STM32_RTC_WPR             (STM32_RTC_BASE + STM32_RTC_WPR_OFFSET)
#define STM32_RTC_CALR            (STM32_RTC_BASE + STM32_RTC_CALR_OFFSET)
#define STM32_RTC_SHIFTR          (STM32_RTC_BASE + STM32_RTC_SHIFTR_OFFSET)
#define STM32_RTC_TSTR            (STM32_RTC_BASE + STM32_RTC_TSTR_OFFSET)
#define STM32_RTC_TSDR            (STM32_RTC_BASE + STM32_RTC_TSDR_OFFSET)
#define STM32_RTC_TSSSR           (STM32_RTC_BASE + STM32_RTC_TSSSR_OFFSET)
#define STM32_RTC_ALRMAR          (STM32_RTC_BASE + STM32_RTC_ALRMAR_OFFSET)
#define STM32_RTC_ALRMASSR        (STM32_RTC_BASE + STM32_RTC_ALRMASSR_OFFSET)
#define STM32_RTC_ALRMBR          (STM32_RTC_BASE + STM32_RTC_ALRMBR_OFFSET)
#define STM32_RTC_ALRMBSSR        (STM32_RTC_BASE + STM32_RTC_ALRMBSSR_OFFSET)
#define STM32_RTC_SR              (STM32_RTC_BASE + STM32_RTC_SR_OFFSET)
#define STM32_RTC_MISR            (STM32_RTC_BASE + STM32_RTC_MISR_OFFSET)
#define STM32_RTC_SMISR           (STM32_RTC_BASE + STM32_RTC_SMISR_OFFSET)
#define STM32_RTC_SCR             (STM32_RTC_BASE + STM32_RTC_SCR_OFFSET)
#define STM32_RTC_ALRABINR        (STM32_RTC_BASE + STM32_RTC_ALRABINR_OFFSET)
#define STM32_RTC_ALRBBINR        (STM32_RTC_BASE + STM32_RTC_ALRBBINR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* RTC time register */

#define RTC_TR_SU_SHIFT           (0)       /* Bits 0-3: Second units in BCD format */
#define RTC_TR_SU_MASK            (15 << RTC_TR_SU_SHIFT)
#define RTC_TR_ST_SHIFT           (4)       /* Bits 4-6: Second tens in BCD format */
#define RTC_TR_ST_MASK            (7 << RTC_TR_ST_SHIFT)
#define RTC_TR_MNU_SHIFT          (8)       /* Bits 8-11: Minute units in BCD format */
#define RTC_TR_MNU_MASK           (15 << RTC_TR_MNU_SHIFT)
#define RTC_TR_MNT_SHIFT          (12)      /* Bits 12-14: Minute tens in BCD format */
#define RTC_TR_MNT_MASK           (7 << RTC_TR_MNT_SHIFT)
#define RTC_TR_HU_SHIFT           (16)      /* Bits 16-19: Hour units in BCD format */
#define RTC_TR_HU_MASK            (15 << RTC_TR_HU_SHIFT)
#define RTC_TR_HT_SHIFT           (20)      /* Bits 20-21: Hour tens in BCD format */
#define RTC_TR_HT_MASK            (3 << RTC_TR_HT_SHIFT)
#define RTC_TR_PM                 (1 << 22) /* Bit 22: AM/PM notation */
#define RTC_TR_RESERVED_BITS      (0xff808080)

/* RTC date register */

#define RTC_DR_DU_SHIFT           (0)       /* Bits 0-3: Date units in BCD format */
#define RTC_DR_DU_MASK            (15 << RTC_DR_DU_SHIFT)
#define RTC_DR_DT_SHIFT           (4)       /* Bits 4-5: Date tens in BCD format */
#define RTC_DR_DT_MASK            (3 << RTC_DR_DT_SHIFT)
#define RTC_DR_MU_SHIFT           (8)       /* Bits 8-11: Month units in BCD format */
#define RTC_DR_MU_MASK            (15 << RTC_DR_MU_SHIFT)
#define RTC_DR_MT                 (1 << 12) /* Bit 12: Month tens in BCD format */
#define RTC_DR_WDU_SHIFT          (13)      /* Bits 13-15: Week day units */
#define RTC_DR_WDU_MASK           (7 << RTC_DR_WDU_SHIFT)
#define RTC_DR_YU_SHIFT           (16)      /* Bits 16-19: Year units in BCD format */
#define RTC_DR_YU_MASK            (15 << RTC_DR_YU_SHIFT)
#define RTC_DR_YT_SHIFT           (20)      /* Bits 20-23: Year tens in BCD format */
#define RTC_DR_YT_MASK            (15 << RTC_DR_YT_SHIFT)
#define RTC_DR_RESERVED_BITS      (0xff0000c0)

/* RTC sub second register */

#define RTC_SSR_SS_SHIFT          (0)       /* Bits 0-31: Sub second value */
#define RTC_SSR_SS_MASK           (0xffffffff << RTC_SSR_SS_SHIFT)

/* RTC init control and status register */

#define RTC_ICSR_WUTWF            (1 << 2)  /* Bit 2: Wakeup timer write flag */
#define RTC_ICSR_SHPF             (1 << 3)  /* Bit 3: Shift operation pending */
#define RTC_ICSR_INITS            (1 << 4)  /* Bit 4: Initialization status flag */
#define RTC_ICSR_RSF              (1 << 5)  /* Bit 5: Registers synchronization flag */
#define RTC_ICSR_INITF            (1 << 6)  /* Bit 6: Initialization flag */
#define RTC_ICSR_INIT             (1 << 7)  /* Bit 7: Initialization mode */
#define RTC_ICSR_BIN_SHIFT        (8)       /* Bits 8-9: Binary mode */
#define RTC_ICSR_BIN_MASK         (3 << RTC_ICSR_BIN_SHIFT)
#  define RTC_ICSR_BIN_BCD        (0 << RTC_ICSR_BIN_SHIFT) /* Free running BCD calendar mode */
#  define RTC_ICSR_BIN_BINARY     (1 << RTC_ICSR_BIN_SHIFT) /* Free running binary mode */
#  define RTC_ICSR_BIN_MIXED      (2 << RTC_ICSR_BIN_SHIFT) /* BCD calendar and binary mode */

#define RTC_ICSR_BCDU_SHIFT       (10)      /* Bits 10-12: BCD update, 1 s increment period */
#define RTC_ICSR_BCDU_MASK        (7 << RTC_ICSR_BCDU_SHIFT)
#define RTC_ICSR_RECALPF          (1 << 16) /* Bit 16: Recalibration pending flag */

/* RTC prescaler register */

#define RTC_PRER_PREDIV_S_SHIFT   (0)       /* Bits 0-14: Synchronous prescaler factor */
#define RTC_PRER_PREDIV_S_MASK    (0x7fff << RTC_PRER_PREDIV_S_SHIFT)
#define RTC_PRER_PREDIV_A_SHIFT   (16)      /* Bits 16-22: Asynchronous prescaler factor */
#define RTC_PRER_PREDIV_A_MASK    (0x7f << RTC_PRER_PREDIV_A_SHIFT)

/* RTC wakeup timer register */

#define RTC_WUTR_WUT_SHIFT        (0)       /* Bits 0-15: Wakeup auto-reload value */
#define RTC_WUTR_WUT_MASK         (0xffff << RTC_WUTR_WUT_SHIFT)
#define RTC_WUTR_WUTOCLR_SHIFT    (16)      /* Bits 16-31: Wakeup auto-reload output clear value */
#define RTC_WUTR_WUTOCLR_MASK     (0xffff << RTC_WUTR_WUTOCLR_SHIFT)

/* RTC control register */

#define RTC_CR_WUCKSEL_SHIFT      (0)       /* Bits 0-2: Wakeup clock selection */
#define RTC_CR_WUCKSEL_MASK       (7 << RTC_CR_WUCKSEL_SHIFT)
#  define RTC_CR_WUCKSEL_RTCDIV16  (0 << RTC_CR_WUCKSEL_SHIFT) /* 000: RTC/16 clock */
#  define RTC_CR_WUCKSEL_RTCDIV8   (1 << RTC_CR_WUCKSEL_SHIFT) /* 001: RTC/8 clock */
#  define RTC_CR_WUCKSEL_RTCDIV4   (2 << RTC_CR_WUCKSEL_SHIFT) /* 010: RTC/4 clock */
#  define RTC_CR_WUCKSEL_RTCDIV2   (3 << RTC_CR_WUCKSEL_SHIFT) /* 011: RTC/2 clock */
#  define RTC_CR_WUCKSEL_CKSPRE    (4 << RTC_CR_WUCKSEL_SHIFT) /* 10x: ck_spre clock */
#  define RTC_CR_WUCKSEL_CKSPREADD (6 << RTC_CR_WUCKSEL_SHIFT) /* 11x: ck_spre, 2^16 added to WUT */

#define RTC_CR_TSEDGE             (1 << 3)  /* Bit 3: Timestamp event active edge */
#define RTC_CR_REFCKON            (1 << 4)  /* Bit 4: Reference clock detection enable */
#define RTC_CR_BYPSHAD            (1 << 5)  /* Bit 5: Bypass the shadow registers */
#define RTC_CR_FMT                (1 << 6)  /* Bit 6: Hour format */
#define RTC_CR_SSRUIE             (1 << 7)  /* Bit 7: SSR underflow interrupt enable */
#define RTC_CR_ALRAE              (1 << 8)  /* Bit 8: Alarm A enable */
#define RTC_CR_ALRBE              (1 << 9)  /* Bit 9: Alarm B enable */
#define RTC_CR_WUTE               (1 << 10) /* Bit 10: Wakeup timer enable */
#define RTC_CR_TSE                (1 << 11) /* Bit 11: Timestamp enable */
#define RTC_CR_ALRAIE             (1 << 12) /* Bit 12: Alarm A interrupt enable */
#define RTC_CR_ALRBIE             (1 << 13) /* Bit 13: Alarm B interrupt enable */
#define RTC_CR_WUTIE              (1 << 14) /* Bit 14: Wakeup timer interrupt enable */
#define RTC_CR_TSIE               (1 << 15) /* Bit 15: Timestamp interrupt enable */
#define RTC_CR_ADD1H              (1 << 16) /* Bit 16: Add 1 hour (summer time change) */
#define RTC_CR_SUB1H              (1 << 17) /* Bit 17: Subtract 1 hour (winter time change) */
#define RTC_CR_BKP                (1 << 18) /* Bit 18: Backup */
#define RTC_CR_COSEL              (1 << 19) /* Bit 19: Calibration output selection */
#define RTC_CR_POL                (1 << 20) /* Bit 20: Output polarity */
#define RTC_CR_OSEL_SHIFT         (21)      /* Bits 21-22: Output selection */
#define RTC_CR_OSEL_MASK          (3 << RTC_CR_OSEL_SHIFT)
#  define RTC_CR_OSEL_DISABLED    (0 << RTC_CR_OSEL_SHIFT) /* 00: Output disabled */
#  define RTC_CR_OSEL_ALRMA       (1 << RTC_CR_OSEL_SHIFT) /* 01: Alarm A output enabled */
#  define RTC_CR_OSEL_ALRMB       (2 << RTC_CR_OSEL_SHIFT) /* 10: Alarm B output enabled */
#  define RTC_CR_OSEL_WUT         (3 << RTC_CR_OSEL_SHIFT) /* 11: Wakeup output enabled */

#define RTC_CR_COE                (1 << 23) /* Bit 23: Calibration output enable */
#define RTC_CR_ITSE               (1 << 24) /* Bit 24: Timestamp on internal event enable */
#define RTC_CR_TAMPTS             (1 << 25) /* Bit 25: Activate timestamp on tamper event */
#define RTC_CR_TAMPOE             (1 << 26) /* Bit 26: Tamper detection output enable */
#define RTC_CR_ALRAFCLR           (1 << 27) /* Bit 27: Alarm A flag automatic clear */
#define RTC_CR_ALRBFCLR           (1 << 28) /* Bit 28: Alarm B flag automatic clear */
#define RTC_CR_TAMPALRM_PU        (1 << 29) /* Bit 29: TAMPALRM pull-up enable */
#define RTC_CR_TAMPALRM_TYPE      (1 << 30) /* Bit 30: TAMPALRM output type */
#define RTC_CR_OUT2EN             (1 << 31) /* Bit 31: RTC_OUT2 output enable */

/* RTC privilege mode control register */

#define RTC_PRIVCFGR_ALRAPRIV     (1 << 0)  /* Bit 0: Alarm A privilege protection */
#define RTC_PRIVCFGR_ALRBPRIV     (1 << 1)  /* Bit 1: Alarm B privilege protection */
#define RTC_PRIVCFGR_WUTPRIV      (1 << 2)  /* Bit 2: Wakeup timer privilege protection */
#define RTC_PRIVCFGR_TSPRIV       (1 << 3)  /* Bit 3: Timestamp privilege protection */
#define RTC_PRIVCFGR_CALPRIV      (1 << 13) /* Bit 13: Shift, calibration, reference clock privilege protection */
#define RTC_PRIVCFGR_INITPRIV     (1 << 14) /* Bit 14: Initialization privilege protection */
#define RTC_PRIVCFGR_PRIV         (1 << 15) /* Bit 15: RTC global privilege protection */

/* RTC secure mode control register */

#define RTC_SECCFGR_ALRASEC       (1 << 0)  /* Bit 0: Alarm A protection */
#define RTC_SECCFGR_ALRBSEC       (1 << 1)  /* Bit 1: Alarm B protection */
#define RTC_SECCFGR_WUTSEC        (1 << 2)  /* Bit 2: Wakeup timer protection */
#define RTC_SECCFGR_TSSEC         (1 << 3)  /* Bit 3: Timestamp protection */
#define RTC_SECCFGR_CALSEC        (1 << 13) /* Bit 13: Shift, calibration, reference clock protection */
#define RTC_SECCFGR_INITSEC       (1 << 14) /* Bit 14: Initialization protection */
#define RTC_SECCFGR_SEC           (1 << 15) /* Bit 15: RTC global protection */

/* RTC write protection register */

#define RTC_WPR_KEY_SHIFT         (0)       /* Bits 0-7: Write protection key */
#define RTC_WPR_KEY_MASK          (0xff << RTC_WPR_KEY_SHIFT)
#  define RTC_WPR_KEY1            (0xca)    /* First unlock key */
#  define RTC_WPR_KEY2            (0x53)    /* Second unlock key */

/* RTC calibration register */

#define RTC_CALR_CALM_SHIFT       (0)       /* Bits 0-8: Calibration minus */
#define RTC_CALR_CALM_MASK        (0x1ff << RTC_CALR_CALM_SHIFT)
#define RTC_CALR_LPCAL            (1 << 12) /* Bit 12: Calibration low-power mode */
#define RTC_CALR_CALW16           (1 << 13) /* Bit 13: Use a 16-second calibration cycle */
#define RTC_CALR_CALW8            (1 << 14) /* Bit 14: Use an 8-second calibration cycle */
#define RTC_CALR_CALP             (1 << 15) /* Bit 15: Increase frequency of RTC by 488.5 ppm */

/* RTC shift control register */

#define RTC_SHIFTR_SUBFS_SHIFT    (0)       /* Bits 0-14: Subtract a fraction of a second */
#define RTC_SHIFTR_SUBFS_MASK     (0x7fff << RTC_SHIFTR_SUBFS_SHIFT)
#define RTC_SHIFTR_ADD1S          (1 << 31) /* Bit 31: Add one second */

/* RTC timestamp time register */

#define RTC_TSTR_SU_SHIFT         (0)       /* Bits 0-3: Second units in BCD format */
#define RTC_TSTR_SU_MASK          (15 << RTC_TSTR_SU_SHIFT)
#define RTC_TSTR_ST_SHIFT         (4)       /* Bits 4-6: Second tens in BCD format */
#define RTC_TSTR_ST_MASK          (7 << RTC_TSTR_ST_SHIFT)
#define RTC_TSTR_MNU_SHIFT        (8)       /* Bits 8-11: Minute units in BCD format */
#define RTC_TSTR_MNU_MASK         (15 << RTC_TSTR_MNU_SHIFT)
#define RTC_TSTR_MNT_SHIFT        (12)      /* Bits 12-14: Minute tens in BCD format */
#define RTC_TSTR_MNT_MASK         (7 << RTC_TSTR_MNT_SHIFT)
#define RTC_TSTR_HU_SHIFT         (16)      /* Bits 16-19: Hour units in BCD format */
#define RTC_TSTR_HU_MASK          (15 << RTC_TSTR_HU_SHIFT)
#define RTC_TSTR_HT_SHIFT         (20)      /* Bits 20-21: Hour tens in BCD format */
#define RTC_TSTR_HT_MASK          (3 << RTC_TSTR_HT_SHIFT)
#define RTC_TSTR_PM               (1 << 22) /* Bit 22: AM/PM notation */

/* RTC timestamp date register */

#define RTC_TSDR_DU_SHIFT         (0)       /* Bits 0-3: Date units in BCD format */
#define RTC_TSDR_DU_MASK          (15 << RTC_TSDR_DU_SHIFT)
#define RTC_TSDR_DT_SHIFT         (4)       /* Bits 4-5: Date tens in BCD format */
#define RTC_TSDR_DT_MASK          (3 << RTC_TSDR_DT_SHIFT)
#define RTC_TSDR_MU_SHIFT         (8)       /* Bits 8-11: Month units in BCD format */
#define RTC_TSDR_MU_MASK          (15 << RTC_TSDR_MU_SHIFT)
#define RTC_TSDR_MT               (1 << 12) /* Bit 12: Month tens in BCD format */
#define RTC_TSDR_WDU_SHIFT        (13)      /* Bits 13-15: Week day units */
#define RTC_TSDR_WDU_MASK         (7 << RTC_TSDR_WDU_SHIFT)

/* RTC timestamp sub second register */

#define RTC_TSSSR_SS_SHIFT        (0)       /* Bits 0-31: Sub second value */
#define RTC_TSSSR_SS_MASK         (0xffffffff << RTC_TSSSR_SS_SHIFT)

/* RTC alarm A/B registers */

#define RTC_ALRMR_SU_SHIFT        (0)       /* Bits 0-3: Second units in BCD format */
#define RTC_ALRMR_SU_MASK         (15 << RTC_ALRMR_SU_SHIFT)
#define RTC_ALRMR_ST_SHIFT        (4)       /* Bits 4-6: Second tens in BCD format */
#define RTC_ALRMR_ST_MASK         (7 << RTC_ALRMR_ST_SHIFT)
#define RTC_ALRMR_MSK1            (1 << 7)  /* Bit 7: Alarm seconds mask */
#define RTC_ALRMR_MNU_SHIFT       (8)       /* Bits 8-11: Minute units in BCD format */
#define RTC_ALRMR_MNU_MASK        (15 << RTC_ALRMR_MNU_SHIFT)
#define RTC_ALRMR_MNT_SHIFT       (12)      /* Bits 12-14: Minute tens in BCD format */
#define RTC_ALRMR_MNT_MASK        (7 << RTC_ALRMR_MNT_SHIFT)
#define RTC_ALRMR_MSK2            (1 << 15) /* Bit 15: Alarm minutes mask */
#define RTC_ALRMR_HU_SHIFT        (16)      /* Bits 16-19: Hour units in BCD format */
#define RTC_ALRMR_HU_MASK         (15 << RTC_ALRMR_HU_SHIFT)
#define RTC_ALRMR_HT_SHIFT        (20)      /* Bits 20-21: Hour tens in BCD format */
#define RTC_ALRMR_HT_MASK         (3 << RTC_ALRMR_HT_SHIFT)
#define RTC_ALRMR_PM              (1 << 22) /* Bit 22: AM/PM notation */
#define RTC_ALRMR_MSK3            (1 << 23) /* Bit 23: Alarm hours mask */
#define RTC_ALRMR_DU_SHIFT        (24)      /* Bits 24-27: Date units or day in BCD format */
#define RTC_ALRMR_DU_MASK         (15 << RTC_ALRMR_DU_SHIFT)
#define RTC_ALRMR_DT_SHIFT        (28)      /* Bits 28-29: Date tens in BCD format */
#define RTC_ALRMR_DT_MASK         (3 << RTC_ALRMR_DT_SHIFT)
#define RTC_ALRMR_WDSEL           (1 << 30) /* Bit 30: Week day selection */
#define RTC_ALRMR_MSK4            (1 << 31) /* Bit 31: Alarm date mask */

/* RTC alarm A/B sub second registers */

#define RTC_ALRMSSR_SS_SHIFT      (0)       /* Bits 0-14: Sub seconds value */
#define RTC_ALRMSSR_SS_MASK       (0x7fff << RTC_ALRMSSR_SS_SHIFT)
#define RTC_ALRMSSR_MASKSS_SHIFT  (24)      /* Bits 24-29: Mask the most-significant bits from this bit */
#define RTC_ALRMSSR_MASKSS_MASK   (0x3f << RTC_ALRMSSR_MASKSS_SHIFT)
#define RTC_ALRMSSR_SSCLR         (1 << 31) /* Bit 31: Clear sub-second binary register on reload */

/* RTC status register */

#define RTC_SR_ALRAF              (1 << 0)  /* Bit 0: Alarm A flag */
#define RTC_SR_ALRBF              (1 << 1)  /* Bit 1: Alarm B flag */
#define RTC_SR_WUTF               (1 << 2)  /* Bit 2: Wakeup timer flag */
#define RTC_SR_TSF                (1 << 3)  /* Bit 3: Timestamp flag */
#define RTC_SR_TSOVF              (1 << 4)  /* Bit 4: Timestamp overflow flag */
#define RTC_SR_ITSF               (1 << 5)  /* Bit 5: Internal timestamp flag */
#define RTC_SR_SSRUF              (1 << 6)  /* Bit 6: SSR underflow flag */

/* RTC masked interrupt status register */

#define RTC_MISR_ALRAMF           (1 << 0)  /* Bit 0: Alarm A masked flag */
#define RTC_MISR_ALRBMF           (1 << 1)  /* Bit 1: Alarm B masked flag */
#define RTC_MISR_WUTMF            (1 << 2)  /* Bit 2: Wakeup timer masked flag */
#define RTC_MISR_TSMF             (1 << 3)  /* Bit 3: Timestamp masked flag */
#define RTC_MISR_TSOVMF           (1 << 4)  /* Bit 4: Timestamp overflow masked flag */
#define RTC_MISR_ITSMF            (1 << 5)  /* Bit 5: Internal timestamp masked flag */
#define RTC_MISR_SSRUMF           (1 << 6)  /* Bit 6: SSR underflow masked flag */

/* RTC secure masked interrupt status register */

#define RTC_SMISR_ALRAMF          (1 << 0)  /* Bit 0: Alarm A secure masked flag */
#define RTC_SMISR_ALRBMF          (1 << 1)  /* Bit 1: Alarm B secure masked flag */
#define RTC_SMISR_WUTMF           (1 << 2)  /* Bit 2: Wakeup timer secure masked flag */
#define RTC_SMISR_TSMF            (1 << 3)  /* Bit 3: Timestamp secure masked flag */
#define RTC_SMISR_TSOVMF          (1 << 4)  /* Bit 4: Timestamp overflow secure masked flag */
#define RTC_SMISR_ITSMF           (1 << 5)  /* Bit 5: Internal timestamp secure masked flag */
#define RTC_SMISR_SSRUMF          (1 << 6)  /* Bit 6: SSR underflow secure masked flag */

/* RTC status clear register */

#define RTC_SCR_CALRAF            (1 << 0)  /* Bit 0: Alarm A flag clear */
#define RTC_SCR_CALRBF            (1 << 1)  /* Bit 1: Alarm B flag clear */
#define RTC_SCR_CWUTF             (1 << 2)  /* Bit 2: Wakeup timer flag clear */
#define RTC_SCR_CTSF              (1 << 3)  /* Bit 3: Timestamp flag clear */
#define RTC_SCR_CTSOVF            (1 << 4)  /* Bit 4: Timestamp overflow flag clear */
#define RTC_SCR_CITSF             (1 << 5)  /* Bit 5: Internal timestamp flag clear */
#define RTC_SCR_CSSRUF            (1 << 6)  /* Bit 6: SSR underflow flag clear */

/* RTC alarm A/B binary mode registers */

#define RTC_ALRBINR_SS_SHIFT      (0)       /* Bits 0-31: Synchronous counter alarm value */
#define RTC_ALRBINR_SS_MASK       (0xffffffff << RTC_ALRBINR_SS_SHIFT)

#endif /* __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_RTCC_M33M55_V1_H */
