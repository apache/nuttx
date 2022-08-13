/****************************************************************************
 * arch/arm/src/stm32wb/hardware/stm32wb_rtc.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_RTCC_H
#define __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_RTCC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32WB_RTC_TR_OFFSET       0x0000 /* RTC time register */
#define STM32WB_RTC_DR_OFFSET       0x0004 /* RTC date register */
#define STM32WB_RTC_CR_OFFSET       0x0008 /* RTC control register */
#define STM32WB_RTC_ISR_OFFSET      0x000c /* RTC initialization and status register */
#define STM32WB_RTC_PRER_OFFSET     0x0010 /* RTC prescaler register */
#define STM32WB_RTC_WUTR_OFFSET     0x0014 /* RTC wakeup timer register */
#define STM32WB_RTC_ALRMAR_OFFSET   0x001c /* RTC alarm A register */
#define STM32WB_RTC_ALRMBR_OFFSET   0x0020 /* RTC alarm B register */
#define STM32WB_RTC_WPR_OFFSET      0x0024 /* RTC write protection register */
#define STM32WB_RTC_SSR_OFFSET      0x0028 /* RTC sub second register */
#define STM32WB_RTC_SHIFTR_OFFSET   0x002c /* RTC shift control register */
#define STM32WB_RTC_TSTR_OFFSET     0x0030 /* RTC time stamp time register */
#define STM32WB_RTC_TSDR_OFFSET     0x0034 /* RTC time stamp date register */
#define STM32WB_RTC_TSSSR_OFFSET    0x0038 /* RTC timestamp sub second register */
#define STM32WB_RTC_CALR_OFFSET     0x003c /* RTC calibration register */
#define STM32WB_RTC_TAMPCR_OFFSET   0x0040 /* RTC tamper configuration register */
#define STM32WB_RTC_ALRMASSR_OFFSET 0x0044 /* RTC alarm A sub second register */
#define STM32WB_RTC_ALRMBSSR_OFFSET 0x0048 /* RTC alarm B sub second register */
#define STM32WB_RTC_OR_OFFSET       0x004c /* RTC option register */

#define STM32WB_RTC_BKPR_OFFSET(n)  (0x0050 + ((n) << 2))
#define STM32WB_RTC_BKP0R_OFFSET    0x0050 /* RTC backup register 0 */
#define STM32WB_RTC_BKP1R_OFFSET    0x0054 /* RTC backup register 1 */
#define STM32WB_RTC_BKP2R_OFFSET    0x0058 /* RTC backup register 2 */
#define STM32WB_RTC_BKP3R_OFFSET    0x005c /* RTC backup register 3 */
#define STM32WB_RTC_BKP4R_OFFSET    0x0060 /* RTC backup register 4 */
#define STM32WB_RTC_BKP5R_OFFSET    0x0064 /* RTC backup register 5 */
#define STM32WB_RTC_BKP6R_OFFSET    0x0068 /* RTC backup register 6 */
#define STM32WB_RTC_BKP7R_OFFSET    0x006c /* RTC backup register 7 */
#define STM32WB_RTC_BKP8R_OFFSET    0x0070 /* RTC backup register 8 */
#define STM32WB_RTC_BKP9R_OFFSET    0x0074 /* RTC backup register 9 */
#define STM32WB_RTC_BKP10R_OFFSET   0x0078 /* RTC backup register 10 */
#define STM32WB_RTC_BKP11R_OFFSET   0x007c /* RTC backup register 11 */
#define STM32WB_RTC_BKP12R_OFFSET   0x0080 /* RTC backup register 12 */
#define STM32WB_RTC_BKP13R_OFFSET   0x0084 /* RTC backup register 13 */
#define STM32WB_RTC_BKP14R_OFFSET   0x0088 /* RTC backup register 14 */
#define STM32WB_RTC_BKP15R_OFFSET   0x008c /* RTC backup register 15 */
#define STM32WB_RTC_BKP16R_OFFSET   0x0090 /* RTC backup register 16 */
#define STM32WB_RTC_BKP17R_OFFSET   0x0094 /* RTC backup register 17 */
#define STM32WB_RTC_BKP18R_OFFSET   0x0098 /* RTC backup register 18 */
#define STM32WB_RTC_BKP19R_OFFSET   0x009c /* RTC backup register 19 */

/* Register Addresses *******************************************************/

#define STM32WB_RTC_TR              (STM32WB_RTC_BASE + STM32WB_RTC_TR_OFFSET)
#define STM32WB_RTC_DR              (STM32WB_RTC_BASE + STM32WB_RTC_DR_OFFSET)
#define STM32WB_RTC_CR              (STM32WB_RTC_BASE + STM32WB_RTC_CR_OFFSET)
#define STM32WB_RTC_ISR             (STM32WB_RTC_BASE + STM32WB_RTC_ISR_OFFSET)
#define STM32WB_RTC_PRER            (STM32WB_RTC_BASE + STM32WB_RTC_PRER_OFFSET)
#define STM32WB_RTC_WUTR            (STM32WB_RTC_BASE + STM32WB_RTC_WUTR_OFFSET)
#define STM32WB_RTC_ALRMAR          (STM32WB_RTC_BASE + STM32WB_RTC_ALRMAR_OFFSET)
#define STM32WB_RTC_ALRMBR          (STM32WB_RTC_BASE + STM32WB_RTC_ALRMBR_OFFSET)
#define STM32WB_RTC_WPR             (STM32WB_RTC_BASE + STM32WB_RTC_WPR_OFFSET)
#define STM32WB_RTC_SSR             (STM32WB_RTC_BASE + STM32WB_RTC_SSR_OFFSET)
#define STM32WB_RTC_SHIFTR          (STM32WB_RTC_BASE + STM32WB_RTC_SHIFTR_OFFSET)
#define STM32WB_RTC_TSTR            (STM32WB_RTC_BASE + STM32WB_RTC_TSTR_OFFSET)
#define STM32WB_RTC_TSDR            (STM32WB_RTC_BASE + STM32WB_RTC_TSDR_OFFSET)
#define STM32WB_RTC_TSSSR           (STM32WB_RTC_BASE + STM32WB_RTC_TSSSR_OFFSET)
#define STM32WB_RTC_CALR            (STM32WB_RTC_BASE + STM32WB_RTC_CALR_OFFSET)
#define STM32WB_RTC_TAMPCR          (STM32WB_RTC_BASE + STM32WB_RTC_TAMPCR_OFFSET)
#define STM32WB_RTC_ALRMASSR        (STM32WB_RTC_BASE + STM32WB_RTC_ALRMASSR_OFFSET)
#define STM32WB_RTC_ALRMBSSR        (STM32WB_RTC_BASE + STM32WB_RTC_ALRMBSSR_OFFSET)
#define STM32WB_RTC_OR              (STM32WB_RTC_BASE + STM32WB_RTC_OR_OFFSET)

#define STM32WB_RTC_BKPR(n)         (STM32WB_RTC_BASE + STM32WB_RTC_BKPR_OFFSET(n))
#define STM32WB_RTC_BKP0R           (STM32WB_RTC_BASE + STM32WB_RTC_BKP0R_OFFSET)
#define STM32WB_RTC_BKP1R           (STM32WB_RTC_BASE + STM32WB_RTC_BKP1R_OFFSET)
#define STM32WB_RTC_BKP2R           (STM32WB_RTC_BASE + STM32WB_RTC_BKP2R_OFFSET)
#define STM32WB_RTC_BKP3R           (STM32WB_RTC_BASE + STM32WB_RTC_BKP3R_OFFSET)
#define STM32WB_RTC_BKP4R           (STM32WB_RTC_BASE + STM32WB_RTC_BKP4R_OFFSET)
#define STM32WB_RTC_BKP5R           (STM32WB_RTC_BASE + STM32WB_RTC_BKP5R_OFFSET)
#define STM32WB_RTC_BKP6R           (STM32WB_RTC_BASE + STM32WB_RTC_BKP6R_OFFSET)
#define STM32WB_RTC_BKP7R           (STM32WB_RTC_BASE + STM32WB_RTC_BKP7R_OFFSET)
#define STM32WB_RTC_BKP8R           (STM32WB_RTC_BASE + STM32WB_RTC_BKP8R_OFFSET)
#define STM32WB_RTC_BKP9R           (STM32WB_RTC_BASE + STM32WB_RTC_BKP9R_OFFSET)
#define STM32WB_RTC_BKP10R          (STM32WB_RTC_BASE + STM32WB_RTC_BKP10R_OFFSET)
#define STM32WB_RTC_BKP11R          (STM32WB_RTC_BASE + STM32WB_RTC_BKP11R_OFFSET)
#define STM32WB_RTC_BKP12R          (STM32WB_RTC_BASE + STM32WB_RTC_BKP12R_OFFSET)
#define STM32WB_RTC_BKP13R          (STM32WB_RTC_BASE + STM32WB_RTC_BKP13R_OFFSET)
#define STM32WB_RTC_BKP14R          (STM32WB_RTC_BASE + STM32WB_RTC_BKP14R_OFFSET)
#define STM32WB_RTC_BKP15R          (STM32WB_RTC_BASE + STM32WB_RTC_BKP15R_OFFSET)
#define STM32WB_RTC_BKP16R          (STM32WB_RTC_BASE + STM32WB_RTC_BKP16R_OFFSET)
#define STM32WB_RTC_BKP17R          (STM32WB_RTC_BASE + STM32WB_RTC_BKP17R_OFFSET)
#define STM32WB_RTC_BKP18R          (STM32WB_RTC_BASE + STM32WB_RTC_BKP18R_OFFSET)
#define STM32WB_RTC_BKP19R          (STM32WB_RTC_BASE + STM32WB_RTC_BKP19R_OFFSET)

#  define STM32WB_RTC_BKCOUNT       20

/* Register Bitfield Definitions ********************************************/

/* RTC time register */

#define RTC_TR_SU_SHIFT             (0)       /* Bits 0-3: Second units in BCD format */
#define RTC_TR_SU_MASK              (0xf << RTC_TR_SU_SHIFT)
#define RTC_TR_ST_SHIFT             (4)       /* Bits 4-6: Second tens in BCD format */
#define RTC_TR_ST_MASK              (0x7 << RTC_TR_ST_SHIFT)
#define RTC_TR_MNU_SHIFT            (8)       /* Bit 8-11: Minute units in BCD format */
#define RTC_TR_MNU_MASK             (0xf << RTC_TR_MNU_SHIFT)
#define RTC_TR_MNT_SHIFT            (12)      /* Bits 12-14: Minute tens in BCD format */
#define RTC_TR_MNT_MASK             (0x7 << RTC_TR_MNT_SHIFT)
#define RTC_TR_HU_SHIFT             (16)      /* Bit 16-19: Hour units in BCD format */
#define RTC_TR_HU_MASK              (0xf << RTC_TR_HU_SHIFT)
#define RTC_TR_HT_SHIFT             (20)      /* Bits 20-21: Hour tens in BCD format */
#define RTC_TR_HT_MASK              (0x3 << RTC_TR_HT_SHIFT)
#define RTC_TR_PM                   (1 << 22) /* Bit 22: AM/PM notation */
#define RTC_TR_RESERVED_BITS        (0xff808080)

/* RTC date register */

#define RTC_DR_DU_SHIFT             (0)       /* Bits 0-3: Date units in BCD format */
#define RTC_DR_DU_MASK              (0xf << RTC_DR_DU_SHIFT)
#define RTC_DR_DT_SHIFT             (4)       /* Bits 4-5: Date tens in BCD format */
#define RTC_DR_DT_MASK              (0x3 << RTC_DR_DT_SHIFT)
#define RTC_DR_MU_SHIFT             (8)       /* Bits 8-11: Month units in BCD format */
#define RTC_DR_MU_MASK              (0xf << RTC_DR_MU_SHIFT)
#define RTC_DR_MT                   (1 << 12) /* Bit 12: Month tens in BCD format */
#define RTC_DR_WDU_SHIFT            (13)      /* Bits 13-15: Week day units */
#define RTC_DR_WDU_MASK             (0x7 << RTC_DR_WDU_SHIFT)
#  define RTC_DR_WDU_MONDAY         (0x1 << RTC_DR_WDU_SHIFT)
#  define RTC_DR_WDU_TUESDAY        (0x2 << RTC_DR_WDU_SHIFT)
#  define RTC_DR_WDU_WEDNESDAY      (0x3 << RTC_DR_WDU_SHIFT)
#  define RTC_DR_WDU_THURSDAY       (0x4 << RTC_DR_WDU_SHIFT)
#  define RTC_DR_WDU_FRIDAY         (0x5 << RTC_DR_WDU_SHIFT)
#  define RTC_DR_WDU_SATURDAY       (0x6 << RTC_DR_WDU_SHIFT)
#  define RTC_DR_WDU_SUNDAY         (0x7 << RTC_DR_WDU_SHIFT)
#define RTC_DR_YU_SHIFT             (16)      /* Bits 16-19: Year units in BCD format */
#define RTC_DR_YU_MASK              (0xf << RTC_DR_YU_SHIFT)
#define RTC_DR_YT_SHIFT             (20)      /* Bits 20-23: Year tens in BCD format */
#define RTC_DR_YT_MASK              (0xf << RTC_DR_YT_SHIFT)
#define RTC_DR_RESERVED_BITS        (0xff0000c0)

/* RTC control register */

#define RTC_CR_WUCKSEL_SHIFT        (0)      /* Bits 0-2: Wakeup clock selection */
#define RTC_CR_WUCKSEL_MASK         (0x7 << RTC_CR_WUCKSEL_SHIFT)
#  define RTC_CR_WUCKSEL_RTCDIV16   (0x0 << RTC_CR_WUCKSEL_SHIFT) /* 000: RTC/16 clock is selected */
#  define RTC_CR_WUCKSEL_RTCDIV8    (0x1 << RTC_CR_WUCKSEL_SHIFT) /* 001: RTC/8 clock is selected */
#  define RTC_CR_WUCKSEL_RTCDIV4    (0x2 << RTC_CR_WUCKSEL_SHIFT) /* 010: RTC/4 clock is selected */
#  define RTC_CR_WUCKSEL_RTCDIV2    (0x3 << RTC_CR_WUCKSEL_SHIFT) /* 011: RTC/2 clock is selected */
#  define RTC_CR_WUCKSEL_CKSPRE     (0x4 << RTC_CR_WUCKSEL_SHIFT) /* 10x: ck_spre clock is selected */
#  define RTC_CR_WUCKSEL_CKSPREADD  (0x6 << RTC_CR_WUCKSEL_SHIFT) /* 11x: ck_spre and 2^16 added to WUT counter */

#define RTC_CR_TSEDGE               (1 << 3)  /* Bit 3: Timestamp event active edge */
#  define RTC_CR_TSEDGE_RISING      (0 << 3)  /* 0: Rising edge generates event */
#  define RTC_CR_TSEDGE_FALLING     (1 << 3)  /* 1: Falling edge generates event */

#define RTC_CR_REFCKON              (1 << 4)  /* Bit 4: Reference clock detection enable (50 or 60 Hz) */
#define RTC_CR_BYPSHAD              (1 << 5)  /* Bit 5: Bypass the shadow registers */

#define RTC_CR_FMT                  (1 << 6)  /* Bit 6: Hour format */
#  define RTC_CR_FMT_24             (0 << 6)  /* 0: 24 hour/day format */
#  define RTC_CR_FMT_AMPM           (1 << 6)  /* 1: AM/PM hour format */

#define RTC_CR_ALRAE                (1 << 8)  /* Bit 8: Alarm A enable */
#define RTC_CR_ALRBE                (1 << 9)  /* Bit 9: Alarm B enable */
#define RTC_CR_WUTE                 (1 << 10) /* Bit 10: Wakeup timer enable */
#define RTC_CR_TSE                  (1 << 11) /* Bit 11: Time stamp enable */
#define RTC_CR_ALRAIE               (1 << 12) /* Bit 12: Alarm A interrupt enable */
#define RTC_CR_ALRBIE               (1 << 13) /* Bit 13: Alarm B interrupt enable */
#define RTC_CR_WUTIE                (1 << 14) /* Bit 14: Wakeup timer interrupt enable */
#define RTC_CR_TSIE                 (1 << 15) /* Bit 15: Timestamp interrupt enable */
#define RTC_CR_ADD1H                (1 << 16) /* Bit 16: Add 1 hour (summer time change) */
#define RTC_CR_SUB1H                (1 << 17) /* Bit 17: Subtract 1 hour (winter time change) */
#define RTC_CR_BKP                  (1 << 18) /* Bit 18: Backup */
#define RTC_CR_COSEL                (1 << 19) /* Bit 19: Calibration output selection */
#  define RTC_CR_COSEL_512HZ        (0 << 19) /* 0: Calibration output is 512 Hz */
#  define RTC_CR_COSEL_1HZ          (1 << 19) /* 1: Calibration output is 1 Hz */

#define RTC_CR_POL                  (1 << 20) /* Bit 20: Output polarity */
#  define RTC_CR_POL_HIGH           (0 << 20) /* 0: Output pin is High on ALRAF/ALRBF/WUTF */
#  define RTC_CR_POL_LOW            (1 << 20) /* 1: Output pin is Low on ALRAF/ALRBF/WUTF */

#define RTC_CR_OSEL_SHIFT           (21)      /* Bits 21-22: Output selection */
#define RTC_CR_OSEL_MASK            (0x3 << RTC_CR_OSEL_SHIFT)
#  define RTC_CR_OSEL_DISABLED      (0x0 << RTC_CR_OSEL_SHIFT) /* 00: Output disabled */
#  define RTC_CR_OSEL_ALRMA         (0x1 << RTC_CR_OSEL_SHIFT) /* 01: Alarm A output enabled */
#  define RTC_CR_OSEL_ALRMB         (0x2 << RTC_CR_OSEL_SHIFT) /* 10: Alarm B output enabled */
#  define RTC_CR_OSEL_WUT           (0x3 << RTC_CR_OSEL_SHIFT) /* 11: Wakeup output enabled */

#define RTC_CR_COE                  (1 << 23) /* Bit 23: Calibration output enable */
#define RTC_CR_ITSE                 (1 << 24) /* Bit 24: Timestamp on internal event enable */

/* RTC initialization and status register */

#define RTC_ISR_ALRAWF              (1 << 0)  /* Bit 0: Alarm A write flag */
#define RTC_ISR_ALRBWF              (1 << 1)  /* Bit 1: Alarm B write flag */
#define RTC_ISR_WUTWF               (1 << 2)  /* Bit 2: Wakeup timer write flag */
#define RTC_ISR_SHPF                (1 << 3)  /* Bit 3: Shift operation pending */
#define RTC_ISR_INITS               (1 << 4)  /* Bit 4: Initialization status flag */
#define RTC_ISR_RSF                 (1 << 5)  /* Bit 5: Registers synchronization flag */
#define RTC_ISR_INITF               (1 << 6)  /* Bit 6: Initialization flag */
#define RTC_ISR_INIT                (1 << 7)  /* Bit 7: Initialization mode */
#define RTC_ISR_ALRAF               (1 << 8)  /* Bit 8: Alarm A flag */
#define RTC_ISR_ALRBF               (1 << 9)  /* Bit 9: Alarm B flag */
#define RTC_ISR_WUTF                (1 << 10) /* Bit 10: Wakeup timer flag */
#define RTC_ISR_TSF                 (1 << 11) /* Bit 11: Time-stamp flag */
#define RTC_ISR_TSOVF               (1 << 12) /* Bit 12: Timestamp overflow flag */
#define RTC_ISR_TAMP1F              (1 << 13) /* Bit 13: Tamper 1 detection flag */
#define RTC_ISR_TAMP2F              (1 << 14) /* Bit 14: Tamper 2 detection flag */
#define RTC_ISR_TAMP3F              (1 << 15) /* Bit 15: Tamper 3 detection flag */
#define RTC_ISR_RECALPF             (1 << 16) /* Bit 16: Recalibration pending flag */
#define RTC_ISR_ITSF                (1 << 17) /* Bit 17: Internal time-stamp flag */
#define RTC_ISR_ALLFLAGS            (0x0003ffff)

/* RTC prescaler register */

#define RTC_PRER_PREDIV_S_SHIFT     (0)       /* Bits 0-14: Synchronous prescaler factor */
#define RTC_PRER_PREDIV_S_MASK      (0x7fff << RTC_PRER_PREDIV_S_SHIFT)
#define RTC_PRER_PREDIV_A_SHIFT     (16)      /* Bits 16-22: Asynchronous prescaler factor */
#define RTC_PRER_PREDIV_A_MASK      (0x7f << RTC_PRER_PREDIV_A_SHIFT)

/* RTC wakeup timer register */

#define RTC_WUTR_SHIFT              (0)       /* Bits 15:0  Wakeup auto-reload value bits */
#define RTC_WUTR_MASK               (0xffff << RTC_WUTR_SHIFT)

/* RTC alarm A/B registers */

#define RTC_ALRMR_SU_SHIFT          (0)       /* Bits 0-3: Second units in BCD format. */
#define RTC_ALRMR_SU_MASK           (0xf << RTC_ALRMR_SU_SHIFT)
#define RTC_ALRMR_ST_SHIFT          (4)       /* Bits 4-6: Second tens in BCD format. */
#define RTC_ALRMR_ST_MASK           (0x7 << RTC_ALRMR_ST_SHIFT)
#define RTC_ALRMR_MSK1              (1 << 7)  /* Bit 7: Alarm A/B seconds mask */
#define RTC_ALRMR_MNU_SHIFT         (8)       /* Bits 8-11: Minute units in BCD format. */
#define RTC_ALRMR_MNU_MASK          (0xf << RTC_ALRMR_MNU_SHIFT)
#define RTC_ALRMR_MNT_SHIFT         (12)      /* Bits 12-14: Minute tens in BCD format. */
#define RTC_ALRMR_MNT_MASK          (0x7 << RTC_ALRMR_MNT_SHIFT)
#define RTC_ALRMR_MSK2              (1 << 15) /* Bit 15: Alarm A/B minutes mask */
#define RTC_ALRMR_HU_SHIFT          (16)      /* Bits 16-19: Hour units in BCD format. */
#define RTC_ALRMR_HU_MASK           (15 << RTC_ALRMR_HU_SHIFT)
#define RTC_ALRMR_HT_SHIFT          (20)      /* Bits 20-21: Hour tens in BCD format. */
#define RTC_ALRMR_HT_MASK           (0x3 << RTC_ALRMR_HT_SHIFT)
#define RTC_ALRMR_PM                (1 << 22) /* Bit 22: AM/PM notation */
#define RTC_ALRMR_MSK3              (1 << 23) /* Bit 23: Alarm A/B hours mask */
#define RTC_ALRMR_DU_SHIFT          (24)      /* Bits 24-27: Date units or day in BCD format. */
#define RTC_ALRMR_DU_MASK           (0xf << RTC_ALRMR_DU_SHIFT)
#define RTC_ALRMR_DT_SHIFT          (28)      /* Bits 28-29: Date tens in BCD format. */
#define RTC_ALRMR_DT_MASK           (0x3 << RTC_ALRMR_DT_SHIFT)
#define RTC_ALRMR_WDSEL             (1 << 30) /* Bit 30: Week day selection */
#define RTC_ALRMR_MSK4              (1 << 31) /* Bit 31: Alarm A/B date mask */

/* RTC write protection register */

#define RTC_WPR_KEY_SHIFT           (0)       /* Bits 0-7: Write protection key */
#define RTC_WPR_KEY_MASK            (0xff << RTC_WPR_KEY_SHIFT)
#  define RTC_WPR_KEY1              (0xca << RTC_WPR_KEY_SHIFT)
#  define RTC_WPR_KEY2              (0x53 << RTC_WPR_KEY_SHIFT)

/* RTC sub second register */

#define RTC_SSR_SHIFT               (0)       /* Bits 0-15: Sub second value */
#define RTC_SSR_MASK                (0xffff << RTC_SSR_SHIFT)

/* RTC shift control register */

#define RTC_SHIFTR_SUBFS_SHIFT      (0)       /* Bits 0-14: Subtract a fraction of a second */
#define RTC_SHIFTR_SUBFS_MASK       (0x7fff << RTC_SHIFTR_SUBFS_SHIFT)
#define RTC_SHIFTR_ADD1S            (1 << 31) /* Bit 31: Add one second */

/* RTC time-stamp time register */

#define RTC_TSTR_SU_SHIFT           (0)       /* Bits 0-3: Second units in BCD format. */
#define RTC_TSTR_SU_MASK            (0xf << RTC_TSTR_SU_SHIFT)
#define RTC_TSTR_ST_SHIFT           (4)       /* Bits 4-6: Second tens in BCD format. */
#define RTC_TSTR_ST_MASK            (0x7 << RTC_TSTR_ST_SHIFT)
#define RTC_TSTR_MNU_SHIFT          (8)       /* Bits 8-11: Minute units in BCD format. */
#define RTC_TSTR_MNU_MASK           (0xf << RTC_TSTR_MNU_SHIFT)
#define RTC_TSTR_MNT_SHIFT          (12)      /* Bits 12-14: Minute tens in BCD format. */
#define RTC_TSTR_MNT_MASK           (0x7 << RTC_TSTR_MNT_SHIFT)
#define RTC_TSTR_HU_SHIFT           (16)      /* Bits 16-19: Hour units in BCD format. */
#define RTC_TSTR_HU_MASK            (0xf << RTC_TSTR_HU_SHIFT)
#define RTC_TSTR_HT_SHIFT           (20)      /* Bits 20-21: Hour tens in BCD format. */
#define RTC_TSTR_HT_MASK            (0x3 << RTC_TSTR_HT_SHIFT)
#define RTC_TSTR_PM                 (1 << 22) /* Bit 22: AM/PM notation */

/* RTC time-stamp date register */

#define RTC_TSDR_DU_SHIFT           (0)       /* Bit 0-3: Date units in BCD format */
#define RTC_TSDR_DU_MASK            (0xf << RTC_TSDR_DU_SHIFT)
#define RTC_TSDR_DT_SHIFT           (4)       /* Bits 4-5: Date tens in BCD format */
#define RTC_TSDR_DT_MASK            (0x3 << RTC_TSDR_DT_SHIFT)
#define RTC_TSDR_MU_SHIFT           (8)       /* Bits 8-11: Month units in BCD format */
#define RTC_TSDR_MU_MASK            (0xf << RTC_TSDR_MU_SHIFT)
#define RTC_TSDR_MT                 (1 << 12) /* Bit 12: Month tens in BCD format */
#define RTC_TSDR_WDU_SHIFT          (13)      /* Bits 13-15: Week day units */
#define RTC_TSDR_WDU_MASK           (0x7 << RTC_TSDR_WDU_SHIFT)

/* RTC time-stamp sub second register */

#define RTC_TSSSR_SHIFT             (0)       /* Bits 0-15: Sub second value */
#define RTC_TSSSR_MASK              (0xffff << RTC_TSSSR_SHIFT)

/* RTC calibration register */

#define RTC_CALR_CALM_SHIFT         (0)       /* Bits 0-8: Calibration minus */
#define RTC_CALR_CALM_MASK          (0x1ff << RTC_CALR_CALM_SHIFT)
#define RTC_CALR_CALW16             (1 << 13) /* Bit 13: Use a 16-second calibration cycle period */
#define RTC_CALR_CALW8              (1 << 14) /* Bit 14: Use an 8-second calibration cycle period */
#define RTC_CALR_CALP               (1 << 15) /* Bit 15: Increase frequency of RTC by 488.5 ppm */

/* RTC tamper configuration register */

#define RTC_TAMPCR_TAMP1E             (1 << 0)  /* Bit 0: Tamper 1 input detection enable */
#define RTC_TAMPCR_TAMP1TRG           (1 << 1)  /* Bit 1: Active level for Tamper 1 input */
#define RTC_TAMPCR_TAMPIE             (1 << 2)  /* Bit 2: Tamper interrupt enable */
#define RTC_TAMPCR_TAMP2E             (1 << 3)  /* Bit 3: Tamper 2 detection enable */
#define RTC_TAMPCR_TAMP2TRG           (1 << 4)  /* Bit 4: Active level for Tamper 2 input */
#define RTC_TAMPCR_TAMP3E             (1 << 5)  /* Bit 5: Tamper 3 detection enable */
#define RTC_TAMPCR_TAMP3TRG           (1 << 6)  /* Bit 6: Active level for Tamper 3 input */
#define RTC_TAMPCR_TAMPTS             (1 << 7)  /* Bit 7: Activate timestamp on tamper detection event */
#define RTC_TAMPCR_TAMPFREQ_SHIFT     (8)       /* Bits 8-10: Tamper sampling frequency */
#define RTC_TAMPCR_TAMPFREQ_MASK      (0x7 << RTC_TAMPCR_TAMPFREQ_SHIFT)
#  define RTC_TAMPCR_TAMPFREQ_d32768  (0x0 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 32768 (1 Hz) */
#  define RTC_TAMPCR_TAMPFREQ_d16384  (0x1 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 16384 (2 Hz) */
#  define RTC_TAMPCR_TAMPFREQ_d8192   (0x2 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 8192 (4 Hz) */
#  define RTC_TAMPCR_TAMPFREQ_d4096   (0x3 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 4096 (8 Hz) */
#  define RTC_TAMPCR_TAMPFREQ_d2048   (0x4 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 2048 (16 Hz) */
#  define RTC_TAMPCR_TAMPFREQ_d1024   (0x5 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 1024 (32 Hz) */
#  define RTC_TAMPCR_TAMPFREQ_d512    (0x6 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 512 (64 Hz) */
#  define RTC_TAMPCR_TAMPFREQ_d256    (0x7 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 256 (128 Hz) */

#define RTC_TAMPCR_TAMPFLT_SHIFT      (11)        /* Bits 11-12: RTC_TAMPx filter count */
#define RTC_TAMPCR_TAMPFLT_MASK       (0x3 << RTC_TAMPCR_TAMPFLT_SHIFT)

#define RTC_TAMPCR_TAMPPRCH_SHIFT     (13)        /* Bits 13-14: RTC_TAMPx precharge duration */
#define RTC_TAMPCR_TAMPPRCH_MASK      (0x3 << RTC_TAMPCR_TAMPPRCH_SHIFT)
#  define RTC_TAMPCR_TAMPPRCH_1CYCLE  (0x0 << RTC_TAMPCR_TAMPPRCH_SHIFT) /* 1 RTCCLK cycle */
#  define RTC_TAMPCR_TAMPPRCH_2CYCLES (0x1 << RTC_TAMPCR_TAMPPRCH_SHIFT) /* 2 RTCCLK cycles */
#  define RTC_TAMPCR_TAMPPRCH_4CYCLES (0x2 << RTC_TAMPCR_TAMPPRCH_SHIFT) /* 4 RTCCLK cycles */
#  define RTC_TAMPCR_TAMPPRCH_5CYCLES (0x3 << RTC_TAMPCR_TAMPPRCH_SHIFT) /* 8 RTCCLK cycles */

#define RTC_TAMPCR_TAMPPUDIS          (1 << 15) /* Bit 15: Tampers pull-up disable */
#define RTC_TAMPCR_TAMP1IE            (1 << 16) /* Bit 16: Tamper 1 interrupt enable */
#define RTC_TAMPCR_TAMP1NOERASE       (1 << 17) /* Bit 17: Tamper 1 no erase */
#define RTC_TAMPCR_TAMP1MF            (1 << 18) /* Bit 18: Tamper 1 mask flag */
#define RTC_TAMPCR_TAMP2IE            (1 << 19) /* Bit 19: Tamper 2 interrupt enable */
#define RTC_TAMPCR_TAMP2NOERASE       (1 << 20) /* Bit 20: Tamper 2 no erase */
#define RTC_TAMPCR_TAMP2MF            (1 << 21) /* Bit 21: Tamper 2 mask flag */
#define RTC_TAMPCR_TAMP3IE            (1 << 22) /* Bit 22: Tamper 3 interrupt enable */
#define RTC_TAMPCR_TAMP3NOERASE       (1 << 23) /* Bit 23: Tamper 3 no erase */
#define RTC_TAMPCR_TAMP3MF            (1 << 24) /* Bit 24: Tamper 3 mask flag */

/* RTC alarm A/B sub second register */

#define RTC_ALRMSSR_SS_SHIFT          (0)       /* Bits 0-14: Sub second value */
#define RTC_ALRMSSR_SS_MASK           (0x7fff << RTC_ALRMSSR_SS_SHIFT)
#define RTC_ALRMSSR_MASKSS_SHIFT      (24)      /* Bits 24-27: Mask the most-significant bits starting at this bit */
#define RTC_ALRMSSR_MASKSS_MASK       (0xf << RTC_ALRMSSR_MASKSS_SHIFT)

/* RTC option register */

#define RTC_OR_ALARMTYPE              (1 << 0)  /* Bit 0: RTC alarm output type on PC13 (PP/OD) */
#  define RTC_OR_ALARMTYPE_OD         (0 << 0)  /* 0: Open-drain output when mapped to PC13 */
#  define RTC_OR_ALARMTYPE_PP         (1 << 0)  /* 1: Push-pull output when mapped to PC13 */
#define RTC_OR_OUTRMP                 (1 << 1)  /* Bit 1: Output remap */
#  define RTC_OR_OUTRMP_PC13          (0 << 1)  /* 0: Alarm/calibration output on PC13 */
#  define RTC_OR_OUTRMP_PB2PC13       (1 << 1)  /* 1: Alarm/calibration output on PB2 or PC13 */

#endif /* __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_RTCC_H */
