/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32_rtcc.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_RTCC_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_RTCC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_RTC_TR_OFFSET       0x0000 /* RTC time register */
#define STM32_RTC_DR_OFFSET       0x0004 /* RTC date register */
#define STM32_RTC_CR_OFFSET       0x0008 /* RTC control register */
#define STM32_RTC_ISR_OFFSET      0x000c /* RTC initialization and status register */
#define STM32_RTC_PRER_OFFSET     0x0010 /* RTC prescaler register */
#define STM32_RTC_WUTR_OFFSET     0x0014 /* RTC wakeup timer register */
#define STM32_RTC_ALRMAR_OFFSET   0x001c /* RTC alarm A register */
#define STM32_RTC_WPR_OFFSET      0x0024 /* RTC write protection register */
#define STM32_RTC_SSR_OFFSET      0x0028 /* RTC sub second register */
#define STM32_RTC_SHIFTR_OFFSET   0x002c /* RTC shift control register */
#define STM32_RTC_TSTR_OFFSET     0x0030 /* RTC time stamp time register */
#define STM32_RTC_TSDR_OFFSET     0x0034 /* RTC time stamp date register */
#define STM32_RTC_TSSSR_OFFSET    0x0038 /* RTC timestamp sub second register */
#define STM32_RTC_CALR_OFFSET     0x003c /* RTC calibration register */
#define STM32_RTC_TAFCR_OFFSET    0x0040 /* RTC tamper and alternate function configuration register */
#define STM32_RTC_ALRMASSR_OFFSET 0x0044 /* RTC alarm A sub second register */

#define STM32_RTC_BKR_OFFSET(n)   (0x0050 + ((n) << 2))
#define STM32_RTC_BK0R_OFFSET     0x0050 /* RTC backup register 0 */
#define STM32_RTC_BK1R_OFFSET     0x0054 /* RTC backup register 1 */
#define STM32_RTC_BK2R_OFFSET     0x0058 /* RTC backup register 2 */
#define STM32_RTC_BK3R_OFFSET     0x005c /* RTC backup register 3 */
#define STM32_RTC_BK4R_OFFSET     0x0060 /* RTC backup register 4 */

/* Register Addresses *******************************************************/

#define STM32_RTC_TR              (STM32_RTC_BASE + STM32_RTC_TR_OFFSET)
#define STM32_RTC_DR              (STM32_RTC_BASE + STM32_RTC_DR_OFFSET)
#define STM32_RTC_CR              (STM32_RTC_BASE + STM32_RTC_CR_OFFSET)
#define STM32_RTC_ISR             (STM32_RTC_BASE + STM32_RTC_ISR_OFFSET)
#define STM32_RTC_PRER            (STM32_RTC_BASE + STM32_RTC_PRER_OFFSET)
#define STM32_RTC_WUTR            (STM32_RTC_BASE + STM32_RTC_WUTR_OFFSET)
#define STM32_RTC_ALRMAR          (STM32_RTC_BASE + STM32_RTC_ALRMAR_OFFSET)
#define STM32_RTC_WPR             (STM32_RTC_BASE + STM32_RTC_WPR_OFFSET)
#define STM32_RTC_SSR             (STM32_RTC_BASE + STM32_RTC_SSR_OFFSET)
#define STM32_RTC_SHIFTR          (STM32_RTC_BASE + STM32_RTC_SHIFTR_OFFSET)
#define STM32_RTC_TSTR            (STM32_RTC_BASE + STM32_RTC_TSTR_OFFSET)
#define STM32_RTC_TSDR            (STM32_RTC_BASE + STM32_RTC_TSDR_OFFSET)
#define STM32_RTC_TSSSR           (STM32_RTC_BASE + STM32_RTC_TSSSR_OFFSET)
#define STM32_RTC_CALR            (STM32_RTC_BASE + STM32_RTC_CALR_OFFSET)
#define STM32_RTC_TAFCR           (STM32_RTC_BASE + STM32_RTC_TAFCR_OFFSET)
#define STM32_RTC_ALRMASSR        (STM32_RTC_BASE + STM32_RTC_ALRMASSR_OFFSET)

#define STM32_RTC_BKR(n)          (STM32_RTC_BASE + STM32_RTC_BKR_OFFSET(n))
#define STM32_RTC_BK0R            (STM32_RTC_BASE + STM32_RTC_BK0R_OFFSET)
#define STM32_RTC_BK1R            (STM32_RTC_BASE + STM32_RTC_BK1R_OFFSET)
#define STM32_RTC_BK2R            (STM32_RTC_BASE + STM32_RTC_BK2R_OFFSET)
#define STM32_RTC_BK3R            (STM32_RTC_BASE + STM32_RTC_BK3R_OFFSET)
#define STM32_RTC_BK4R            (STM32_RTC_BASE + STM32_RTC_BK4R_OFFSET)

#define STM32_RTC_BKCOUNT         5

/* Register Bitfield Definitions ********************************************/

/* RTC time register */

#define RTC_TR_SU_SHIFT           (0)       /* Bits 0-3: Second units in BCD format */
#define RTC_TR_SU_MASK            (15 << RTC_TR_SU_SHIFT)
#define RTC_TR_ST_SHIFT           (4)       /* Bits 4-6: Second tens in BCD format */
#define RTC_TR_ST_MASK            (7 << RTC_TR_ST_SHIFT)
#define RTC_TR_MNU_SHIFT          (8)       /* Bit 8-11: Minute units in BCD format */
#define RTC_TR_MNU_MASK           (15 << RTC_TR_MNU_SHIFT)
#define RTC_TR_MNT_SHIFT          (12)      /* Bits 12-14: Minute tens in BCD format */
#define RTC_TR_MNT_MASK           (7 << RTC_TR_MNT_SHIFT)
#define RTC_TR_HU_SHIFT           (16)      /* Bit 16-19: Hour units in BCD format */
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
#define RTC_DR_MU_SHIFT           (8)      /* Bits 8-11: Month units in BCD format */
#define RTC_DR_MU_MASK            (15 << RTC_DR_MU_SHIFT)
#define RTC_DR_MT                 (1 << 12) /* Bit 12: Month tens in BCD format */
#define RTC_DR_WDU_SHIFT          (13)      /* Bits 13-15: Week day units */
#define RTC_DR_WDU_MASK           (7 << RTC_DR_WDU_SHIFT)
#  define RTC_DR_WDU_MONDAY       (1 << RTC_DR_WDU_SHIFT)
#  define RTC_DR_WDU_TUESDAY      (2 << RTC_DR_WDU_SHIFT)
#  define RTC_DR_WDU_WEDNESDAY    (3 << RTC_DR_WDU_SHIFT)
#  define RTC_DR_WDU_THURSDAY     (4 << RTC_DR_WDU_SHIFT)
#  define RTC_DR_WDU_FRIDAY       (5 << RTC_DR_WDU_SHIFT)
#  define RTC_DR_WDU_SATURDAY     (6 << RTC_DR_WDU_SHIFT)
#  define RTC_DR_WDU_SUNDAY       (7 << RTC_DR_WDU_SHIFT)
#define RTC_DR_YU_SHIFT           (16)     /* Bits 16-19: Year units in BCD format */
#define RTC_DR_YU_MASK            (15 << RTC_DR_YU_SHIFT)
#define RTC_DR_YT_SHIFT           (20)     /* Bits 20-23: Year tens in BCD format */
#define RTC_DR_YT_MASK            (15 << RTC_DR_YT_SHIFT)
#define RTC_DR_RESERVED_BITS      (0xff0000c0)

/* RTC control register */

#define RTC_CR_WUCKSEL_SHIFT      (0)      /* Bits 0-2: Wakeup clock selection */
#define RTC_CR_WUCKSEL_MASK       (7 << RTC_CR_WUCKSEL_SHIFT)
#  define RTC_CR_WUCKSEL_RTCDIV16  (0 << RTC_CR_WUCKSEL_SHIFT) /* 000: RTC/16 clock is selected */
#  define RTC_CR_WUCKSEL_RTCDIV8   (1 << RTC_CR_WUCKSEL_SHIFT) /* 001: RTC/8 clock is selected */
#  define RTC_CR_WUCKSEL_RTCDIV4   (2 << RTC_CR_WUCKSEL_SHIFT) /* 010: RTC/4 clock is selected */
#  define RTC_CR_WUCKSEL_RTCDIV2   (3 << RTC_CR_WUCKSEL_SHIFT) /* 011: RTC/2 clock is selected */
#  define RTC_CR_WUCKSEL_CKSPRE    (4 << RTC_CR_WUCKSEL_SHIFT) /* 10x: ck_spre clock is selected */
#  define RTC_CR_WUCKSEL_CKSPREADD (6 << RTC_CR_WUCKSEL_SHIFT) /* 11x: ck_spr clock and 216 added WUT counter */

#define RTC_CR_TSEDGE             (1 << 3)  /* Bit 3:  Timestamp event active edge */
#define RTC_CR_REFCKON            (1 << 4)  /* Bit 4:  Reference clock detection enable (50 or 60 Hz) */
#define RTC_CR_BYPSHAD            (1 << 5)  /* Bit 5:  Bypass the shadow registers */
#define RTC_CR_FMT                (1 << 6)  /* Bit 6:  Hour format */
#define RTC_CR_ALRAE              (1 << 8)  /* Bit 8:  Alarm A enable */
#define RTC_CR_WUTE               (1 << 10) /* Bit 10: Wakeup timer enable */
#define RTC_CR_TSE                (1 << 11) /* Bit 11: Time stamp enable */
#define RTC_CR_ALRAIE             (1 << 12) /* Bit 12: Alarm A interrupt enable */
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

/* RTC initialization and status register */

#define RTC_ISR_ALRAWF            (1 << 0)  /* Bit 0:  Alarm A write flag */
#define RTC_ISR_WUTWF             (1 << 2)  /* Bit 2:  Wakeup timer write flag */
#define RTC_ISR_SHPF              (1 << 3)  /* Bit 3:  Shift operation pending */
#define RTC_ISR_INITS             (1 << 4)  /* Bit 4:  Initialization status flag */
#define RTC_ISR_RSF               (1 << 5)  /* Bit 5:  Registers synchronization flag */
#define RTC_ISR_INITF             (1 << 6)  /* Bit 6:  Initialization flag */
#define RTC_ISR_INIT              (1 << 7)  /* Bit 7:  Initialization mode */
#define RTC_ISR_ALRAF             (1 << 8)  /* Bit 8:  Alarm A flag */
#define RTC_ISR_WUTF              (1 << 10) /* Bit 10: Wakeup timer flag */
#define RTC_ISR_TSF               (1 << 11) /* Bit 11: Timestamp flag */
#define RTC_ISR_TSOVF             (1 << 12) /* Bit 12: Timestamp overflow flag */
#define RTC_ISR_TAMP1F            (1 << 13) /* Bit 13: Tamper detection flag */
#define RTC_ISR_TAMP2F            (1 << 14) /* Bit 14: TAMPER2 detection flag */
#define RTC_ISR_TAMP3F            (1 << 15) /* Bit 15: TAMPER3 detection flag */
#define RTC_ISR_RECALPF           (1 << 16) /* Bit 16: Recalibration pending flag */
#define RTC_ISR_ALLFLAGS          (0x00017fff)

/* RTC prescaler register */

#define RTC_PRER_PREDIV_S_SHIFT   (0)       /* Bits 0-14: Synchronous prescaler factor */
#define RTC_PRER_PREDIV_S_MASK    (0x7fff << RTC_PRER_PREDIV_S_SHIFT)
#define RTC_PRER_PREDIV_A_SHIFT   (16)      /* Bits 16-22: Asynchronous prescaler factor */
#define RTC_PRER_PREDIV_A_MASK    (0x7f << RTC_PRER_PREDIV_A_SHIFT)

/* RTC wakeup timer register */

#define RTC_WUTR_MASK             (0xffff)  /* Bits 15:0  Wakeup auto-reload value bits */

/* RTC alarm A register */

#define RTC_ALRMR_SU_SHIFT        (0)       /* Bits 0-3: Second units in BCD format. */
#define RTC_ALRMR_SU_MASK         (15 << RTC_ALRMR_SU_SHIFT)
#define RTC_ALRMR_ST_SHIFT        (4)       /* Bits 4-6: Second tens in BCD format. */
#define RTC_ALRMR_ST_MASK         (7 << RTC_ALRMR_ST_SHIFT)
#define RTC_ALRMR_MSK1            (1 << 7)  /* Bit 7 : Alarm A seconds mask */
#define RTC_ALRMR_MNU_SHIFT       (8)       /* Bits 8-11: Minute units in BCD format. */
#define RTC_ALRMR_MNU_MASK        (15 << RTC_ALRMR_MNU_SHIFT)
#define RTC_ALRMR_MNT_SHIFT       (12)      /* Bits 12-14: Minute tens in BCD format. */
#define RTC_ALRMR_MNT_MASK        (7 << RTC_ALRMR_MNT_SHIFT)
#define RTC_ALRMR_MSK2            (1 << 15) /* Bit 15 : Alarm A minutes mask */
#define RTC_ALRMR_HU_SHIFT        (16)      /* Bits 16-19: Hour units in BCD format. */
#define RTC_ALRMR_HU_MASK         (15 << RTC_ALRMR_HU_SHIFT)
#define RTC_ALRMR_HT_SHIFT        (20)      /* Bits 20-21: Hour tens in BCD format. */
#define RTC_ALRMR_HT_MASK         (3 << RTC_ALRMR_HT_SHIFT)
#define RTC_ALRMR_PM              (1 << 22) /* Bit 22 : AM/PM notation */
#define RTC_ALRMR_MSK3            (1 << 23) /* Bit 23 : Alarm A hours mask */
#define RTC_ALRMR_DU_SHIFT        (24)      /* Bits 24-27: Date units or day in BCD format. */
#define RTC_ALRMR_DU_MASK         (15 << RTC_ALRMR_DU_SHIFT)
#define RTC_ALRMR_DT_SHIFT        (28)      /* Bits 28-29: Date tens in BCD format. */
#define RTC_ALRMR_DT_MASK         (3 << RTC_ALRMR_DT_SHIFT)
#define RTC_ALRMR_WDSEL           (1 << 30) /* Bit 30: Week day selection */
#define RTC_ALRMR_MSK4            (1 << 31) /* Bit 31: Alarm A date mask */

/* RTC write protection register */

#define RTC_WPR_MASK              (0xff)    /* Bits 0-7: Write protection key */

/* RTC sub second register */

#define RTC_SSR_MASK              (0xffff)  /* Bits 0-15: Sub second value */

/* RTC shift control register */

#define RTC_SHIFTR_SUBFS_SHIFT    (0)       /* Bits 0-14: Subtract a fraction of a second */
#define RTC_SHIFTR_SUBFS_MASK     (0x7fff << RTC_SHIFTR_SUBFS_SHIFT)
#define RTC_SHIFTR_ADD1S          (1 << 31) /* Bit 31: Add one second */

/* RTC time stamp time register */

#define RTC_TSTR_SU_SHIFT         (0)       /* Bits 0-3: Second units in BCD format. */
#define RTC_TSTR_SU_MASK          (15 << RTC_TSTR_SU_SHIFT)
#define RTC_TSTR_ST_SHIFT         (4)       /* Bits 4-6: Second tens in BCD format. */
#define RTC_TSTR_ST_MASK          (7 << RTC_TSTR_ST_SHIFT)
#define RTC_TSTR_MNU_SHIFT        (8)       /* Bits 8-11: Minute units in BCD format. */
#define RTC_TSTR_MNU_MASK         (15 << RTC_TSTR_MNU_SHIFT)
#define RTC_TSTR_MNT_SHIFT        (12)      /* Bits 12-14: Minute tens in BCD format. */
#define RTC_TSTR_MNT_MASK         (7 << RTC_TSTR_MNT_SHIFT)
#define RTC_TSTR_HU_SHIFT         (16)      /* Bits 16-19: Hour units in BCD format. */
#define RTC_TSTR_HU_MASK          (15 << RTC_TSTR_HU_SHIFT)
#define RTC_TSTR_HT_SHIFT         (20)      /* Bits 20-21: Hour tens in BCD format. */
#define RTC_TSTR_HT_MASK          (3 << RTC_TSTR_HT_SHIFT)
#define RTC_TSTR_PM               (1 << 22) /* Bit 22: AM/PM notation */

/* RTC time stamp date register */

#define RTC_TSDR_DU_SHIFT         (0)       /* Bit 0-3: Date units in BCD format */
#define RTC_TSDR_DU_MASK          (15 << RTC_TSDR_DU_SHIFT)
#define RTC_TSDR_DT_SHIFT         (4)       /* Bits 4-5: Date tens in BCD format */
#define RTC_TSDR_DT_MASK          (3 << RTC_TSDR_DT_SHIFT)
#define RTC_TSDR_MU_SHIFT         (8)       /* Bits 8-11: Month units in BCD format */
#define RTC_TSDR_MU_MASK          (15 << RTC_TSDR_MU_SHIFT)
#define RTC_TSDR_MT               (1 << 12) /* Bit 12: Month tens in BCD format */
#define RTC_TSDR_WDU_SHIFT        (13)      /* Bits 13-15: Week day units */
#define RTC_TSDR_WDU_MASK         (7 << RTC_TSDR_WDU_SHIFT)

/* RTC timestamp sub second register */

#define RTC_TSSSR_MASK            (0xffff)  /* Bits 0-15: Sub second value */

/* RTC calibration register */

#define RTC_CALR_CALM_SHIFT       (0)       /* Bits 0-8: Calibration minus */
#define RTC_CALR_CALM_MASK        (0x1ff << RTC_CALR_CALM_SHIFT)
#define RTC_CALR_CALW16           (1 << 13) /* Bit 13: Use a 16-second calibration cycle period */
#define RTC_CALR_CALW8            (1 << 14) /* Bit 14: Use an 8-second calibration cycle period */
#define RTC_CALR_CALP             (1 << 15) /* Bit 15: Increase frequency of RTC by 488.5 ppm */

/* RTC tamper and alternate function configuration register */

#define RTC_TAFCR_TAMP1E          (1 << 0)  /* Bit 0:  RTC_TAMP1 input detection enable */
#define RTC_TAFCR_TAMP1TRG        (1 << 1)  /* Bit 1:  Active level for RTC_TAMP1 input */
#define RTC_TAFCR_TAMPIE          (1 << 2)  /* Bit 2:  Tamper interrupt enable */
#define RTC_TAFCR_TAMP3E          (1 << 5)  /* Bit 5:  RTC_TAMP3 detection enable */
#define RTC_TAFCR_TAMP3TRG        (1 << 6)  /* Bit 6:  Active level for RTC_TAMP3 input */
#define RTC_TAFCR_TAMPTS          (1 << 7)  /* Bit 7:  Activate timestamp on tamper detection event */
#define RTC_TAFCR_TAMPFREQ_SHIFT  (8)       /* Bits 8-10: Tamper sampling frequency */
#define RTC_TAFCR_TAMPFREQ_MASK   (7 << RTC_TAFCR_TAMPFREQ_SHIFT)
#  define RTC_TAFCR_TAMPFREQ_DIV32768 (0 << RTC_TAFCR_TAMPFREQ_SHIFT) /* RTCCLK / 32768 (1 Hz) */
#  define RTC_TAFCR_TAMPFREQ_DIV16384 (1 << RTC_TAFCR_TAMPFREQ_SHIFT) /* RTCCLK / 16384 (2 Hz) */
#  define RTC_TAFCR_TAMPFREQ_DIV8192  (2 << RTC_TAFCR_TAMPFREQ_SHIFT) /* RTCCLK / 8192 (4 Hz) */
#  define RTC_TAFCR_TAMPFREQ_DIV4096  (3 << RTC_TAFCR_TAMPFREQ_SHIFT) /* RTCCLK / 4096 (8 Hz) */
#  define RTC_TAFCR_TAMPFREQ_DIV2048  (4 << RTC_TAFCR_TAMPFREQ_SHIFT) /* RTCCLK / 2048 (16 Hz) */
#  define RTC_TAFCR_TAMPFREQ_DIV1024  (5 << RTC_TAFCR_TAMPFREQ_SHIFT) /* RTCCLK / 1024 (32 Hz) */
#  define RTC_TAFCR_TAMPFREQ_DIV512   (6 << RTC_TAFCR_TAMPFREQ_SHIFT) /* RTCCLK / 512 (64 Hz) */
#  define RTC_TAFCR_TAMPFREQ_DIV256   (7 << RTC_TAFCR_TAMPFREQ_SHIFT) /* RTCCLK / 256 (128 Hz) */

#define RTC_TAFCR_TAMPFLT_SHIFT   (11)        /* Bits 11-12: RTC_TAMPx filter count */
#define RTC_TAFCR_TAMPFLT_MASK    (3 << RTC_TAFCR_TAMPFLT_SHIFT)
#define RTC_TAFCR_TAMPPRCH_SHIFT  (13)        /* Bits 13-14: RTC_TAMPx precharge duration */
#define RTC_TAFCR_TAMPPRCH_MASK   (3 << RTC_TAFCR_TAMPPRCH_SHIFT)
#  define RTC_TAFCR_TAMPPRCH_1CYCLE  (0 << RTC_TAFCR_TAMPPRCH_SHIFT) /* 1 RTCCLK cycle */
#  define RTC_TAFCR_TAMPPRCH_2CYCLES (1 << RTC_TAFCR_TAMPPRCH_SHIFT) /* 2 RTCCLK cycles */
#  define RTC_TAFCR_TAMPPRCH_4CYCLES (2 << RTC_TAFCR_TAMPPRCH_SHIFT) /* 4 RTCCLK cycles */
#  define RTC_TAFCR_TAMPPRCH_5CYCLES (3 << RTC_TAFCR_TAMPPRCH_SHIFT) /* 8 RTCCLK cycles */

#define RTC_TAFCR_TAMPPUDIS       (1 << 15) /* Bit 15: RTC_TAMPx pull-up disable */
#define RTC_TAFCR_PC13VALUE       (1 << 18) /* Bit 18: RTC_ALARM output type/PC13 value */
#define RTC_TAFCR_PC13MODE        (1 << 19) /* Bit 19: PC13 mode */
#define RTC_TAFCR_PC14VALUE       (1 << 20) /* Bit 20: PC14 value */
#define RTC_TAFCR_PC14MODE        (1 << 21) /* Bit 21: PC14 mode */
#define RTC_TAFCR_PC15VALUE       (1 << 22) /* Bit 22: PC15 value */
#define RTC_TAFCR_PC15MODE        (1 << 23) /* Bit 23: PC15 mode */

/* RTC alarm A sub second register */

#define RTC_ALRMSSR_SS_SHIFT      (0)   /* Bits 0-14: Sub second value */
#define RTC_ALRMSSR_SS_MASK       (0x7fff << RTC_ALRMSSR_SS_SHIFT)
#define RTC_ALRMSSR_MASKSS_SHIFT  (24)  /* Bits 24-27:  Mask the most-significant bits starting at this bit */
#define RTC_ALRMSSR_MASKSS_MASK   (0xf << RTC_ALRMSSR_MASKSS_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_RTCC_H */
