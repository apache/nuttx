/****************************************************************************
 * arch/arm/src/stm32l4/hardware/stm32l4_rtcc.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_RTCC_H
#define __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_RTCC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32L4_RTC_TR_OFFSET       0x0000 /* RTC time register */
#define STM32L4_RTC_DR_OFFSET       0x0004 /* RTC date register */
#define STM32L4_RTC_CR_OFFSET       0x0008 /* RTC control register */
#define STM32L4_RTC_ISR_OFFSET      0x000c /* RTC initialization and status register */
#define STM32L4_RTC_PRER_OFFSET     0x0010 /* RTC prescaler register */
#define STM32L4_RTC_WUTR_OFFSET     0x0014 /* RTC wakeup timer register */
#define STM32L4_RTC_ALRMAR_OFFSET   0x001c /* RTC alarm A register */
#define STM32L4_RTC_ALRMBR_OFFSET   0x0020 /* RTC alarm B register */
#define STM32L4_RTC_WPR_OFFSET      0x0024 /* RTC write protection register */
#define STM32L4_RTC_SSR_OFFSET      0x0028 /* RTC sub second register */
#define STM32L4_RTC_SHIFTR_OFFSET   0x002c /* RTC shift control register */
#define STM32L4_RTC_TSTR_OFFSET     0x0030 /* RTC time stamp time register */
#define STM32L4_RTC_TSDR_OFFSET     0x0034 /* RTC time stamp date register */
#define STM32L4_RTC_TSSSR_OFFSET    0x0038 /* RTC timestamp sub second register */
#define STM32L4_RTC_CALR_OFFSET     0x003c /* RTC calibration register */
#define STM32L4_RTC_TAMPCR_OFFSET   0x0040 /* RTC tamper configuration register */
#define STM32L4_RTC_ALRMASSR_OFFSET 0x0044 /* RTC alarm A sub second register */
#define STM32L4_RTC_ALRMBSSR_OFFSET 0x0048 /* RTC alarm B sub second register */
#define STM32L4_RTC_OR_OFFSET       0x004c /* RTC option register */

#define STM32L4_RTC_BKR_OFFSET(n)   (0x0050+((n)<<2))
#define STM32L4_RTC_BK0R_OFFSET     0x0050 /* RTC backup register 0 */
#define STM32L4_RTC_BK1R_OFFSET     0x0054 /* RTC backup register 1 */
#define STM32L4_RTC_BK2R_OFFSET     0x0058 /* RTC backup register 2 */
#define STM32L4_RTC_BK3R_OFFSET     0x005c /* RTC backup register 3 */
#define STM32L4_RTC_BK4R_OFFSET     0x0060 /* RTC backup register 4 */
#define STM32L4_RTC_BK5R_OFFSET     0x0064 /* RTC backup register 5 */
#define STM32L4_RTC_BK6R_OFFSET     0x0068 /* RTC backup register 6 */
#define STM32L4_RTC_BK7R_OFFSET     0x006c /* RTC backup register 7 */
#define STM32L4_RTC_BK8R_OFFSET     0x0070 /* RTC backup register 8 */
#define STM32L4_RTC_BK9R_OFFSET     0x0074 /* RTC backup register 9 */
#define STM32L4_RTC_BK10R_OFFSET    0x0078 /* RTC backup register 10 */
#define STM32L4_RTC_BK11R_OFFSET    0x007c /* RTC backup register 11 */
#define STM32L4_RTC_BK12R_OFFSET    0x0080 /* RTC backup register 12 */
#define STM32L4_RTC_BK13R_OFFSET    0x0084 /* RTC backup register 13 */
#define STM32L4_RTC_BK14R_OFFSET    0x0088 /* RTC backup register 14 */
#define STM32L4_RTC_BK15R_OFFSET    0x008c /* RTC backup register 15 */
#define STM32L4_RTC_BK16R_OFFSET    0x0090 /* RTC backup register 16 */
#define STM32L4_RTC_BK17R_OFFSET    0x0094 /* RTC backup register 17 */
#define STM32L4_RTC_BK18R_OFFSET    0x0098 /* RTC backup register 18 */
#define STM32L4_RTC_BK19R_OFFSET    0x009c /* RTC backup register 19 */
#define STM32L4_RTC_BK20R_OFFSET    0x00a0 /* RTC backup register 20 */
#define STM32L4_RTC_BK21R_OFFSET    0x00a4 /* RTC backup register 21 */
#define STM32L4_RTC_BK22R_OFFSET    0x00a8 /* RTC backup register 22 */
#define STM32L4_RTC_BK23R_OFFSET    0x00ac /* RTC backup register 23 */
#define STM32L4_RTC_BK24R_OFFSET    0x00b0 /* RTC backup register 24 */
#define STM32L4_RTC_BK25R_OFFSET    0x00b4 /* RTC backup register 25 */
#define STM32L4_RTC_BK26R_OFFSET    0x00b8 /* RTC backup register 26 */
#define STM32L4_RTC_BK27R_OFFSET    0x00bc /* RTC backup register 27 */
#define STM32L4_RTC_BK28R_OFFSET    0x00c0 /* RTC backup register 28 */
#define STM32L4_RTC_BK29R_OFFSET    0x00c4 /* RTC backup register 29 */
#define STM32L4_RTC_BK30R_OFFSET    0x00c8 /* RTC backup register 30 */
#define STM32L4_RTC_BK31R_OFFSET    0x00cc /* RTC backup register 31 */

/* Register Addresses *******************************************************/

#define STM32L4_RTC_TR              (STM32L4_RTC_BASE+STM32L4_RTC_TR_OFFSET)
#define STM32L4_RTC_DR              (STM32L4_RTC_BASE+STM32L4_RTC_DR_OFFSET)
#define STM32L4_RTC_CR              (STM32L4_RTC_BASE+STM32L4_RTC_CR_OFFSET)
#define STM32L4_RTC_ISR             (STM32L4_RTC_BASE+STM32L4_RTC_ISR_OFFSET)
#define STM32L4_RTC_PRER            (STM32L4_RTC_BASE+STM32L4_RTC_PRER_OFFSET)
#define STM32L4_RTC_WUTR            (STM32L4_RTC_BASE+STM32L4_RTC_WUTR_OFFSET)
#define STM32L4_RTC_ALRMAR          (STM32L4_RTC_BASE+STM32L4_RTC_ALRMAR_OFFSET)
#define STM32L4_RTC_ALRMBR          (STM32L4_RTC_BASE+STM32L4_RTC_ALRMBR_OFFSET)
#define STM32L4_RTC_WPR             (STM32L4_RTC_BASE+STM32L4_RTC_WPR_OFFSET)
#define STM32L4_RTC_SSR             (STM32L4_RTC_BASE+STM32L4_RTC_SSR_OFFSET)
#define STM32L4_RTC_SHIFTR          (STM32L4_RTC_BASE+STM32L4_RTC_SHIFTR_OFFSET)
#define STM32L4_RTC_TSTR            (STM32L4_RTC_BASE+STM32L4_RTC_TSTR_OFFSET)
#define STM32L4_RTC_TSDR            (STM32L4_RTC_BASE+STM32L4_RTC_TSDR_OFFSET)
#define STM32L4_RTC_TSSSR           (STM32L4_RTC_BASE+STM32L4_RTC_TSSSR_OFFSET)
#define STM32L4_RTC_CALR            (STM32L4_RTC_BASE+STM32L4_RTC_CALR_OFFSET)
#define STM32L4_RTC_TAMPCR          (STM32L4_RTC_BASE+STM32L4_RTC_TAMPCR_OFFSET)
#define STM32L4_RTC_ALRMASSR        (STM32L4_RTC_BASE+STM32L4_RTC_ALRMASSR_OFFSET)
#define STM32L4_RTC_ALRMBSSR        (STM32L4_RTC_BASE+STM32L4_RTC_ALRMBSSR_OFFSET)
#define STM32L4_RTC_OR              (STM32L4_RTC_BASE+STM32L4_RTC_OR_OFFSET)

#define STM32L4_RTC_BKR(n)          (STM32L4_RTC_BASE+STM32L4_RTC_BKR_OFFSET(n))
#define STM32L4_RTC_BK0R            (STM32L4_RTC_BASE+STM32L4_RTC_BK0R_OFFSET)
#define STM32L4_RTC_BK1R            (STM32L4_RTC_BASE+STM32L4_RTC_BK1R_OFFSET)
#define STM32L4_RTC_BK2R            (STM32L4_RTC_BASE+STM32L4_RTC_BK2R_OFFSET)
#define STM32L4_RTC_BK3R            (STM32L4_RTC_BASE+STM32L4_RTC_BK3R_OFFSET)
#define STM32L4_RTC_BK4R            (STM32L4_RTC_BASE+STM32L4_RTC_BK4R_OFFSET)
#define STM32L4_RTC_BK5R            (STM32L4_RTC_BASE+STM32L4_RTC_BK5R_OFFSET)
#define STM32L4_RTC_BK6R            (STM32L4_RTC_BASE+STM32L4_RTC_BK6R_OFFSET)
#define STM32L4_RTC_BK7R            (STM32L4_RTC_BASE+STM32L4_RTC_BK7R_OFFSET)
#define STM32L4_RTC_BK8R            (STM32L4_RTC_BASE+STM32L4_RTC_BK8R_OFFSET)
#define STM32L4_RTC_BK9R            (STM32L4_RTC_BASE+STM32L4_RTC_BK9R_OFFSET)
#define STM32L4_RTC_BK10R           (STM32L4_RTC_BASE+STM32L4_RTC_BK10R_OFFSET)
#define STM32L4_RTC_BK11R           (STM32L4_RTC_BASE+STM32L4_RTC_BK11R_OFFSET)
#define STM32L4_RTC_BK12R           (STM32L4_RTC_BASE+STM32L4_RTC_BK12R_OFFSET)
#define STM32L4_RTC_BK13R           (STM32L4_RTC_BASE+STM32L4_RTC_BK13R_OFFSET)
#define STM32L4_RTC_BK14R           (STM32L4_RTC_BASE+STM32L4_RTC_BK14R_OFFSET)
#define STM32L4_RTC_BK15R           (STM32L4_RTC_BASE+STM32L4_RTC_BK15R_OFFSET)
#define STM32L4_RTC_BK16R           (STM32L4_RTC_BASE+STM32L4_RTC_BK16R_OFFSET)
#define STM32L4_RTC_BK17R           (STM32L4_RTC_BASE+STM32L4_RTC_BK17R_OFFSET)
#define STM32L4_RTC_BK18R           (STM32L4_RTC_BASE+STM32L4_RTC_BK18R_OFFSET)
#define STM32L4_RTC_BK19R           (STM32L4_RTC_BASE+STM32L4_RTC_BK19R_OFFSET)
#define STM32L4_RTC_BK20R           (STM32L4_RTC_BASE+STM32L4_RTC_BK20R_OFFSET)
#define STM32L4_RTC_BK21R           (STM32L4_RTC_BASE+STM32L4_RTC_BK21R_OFFSET)
#define STM32L4_RTC_BK22R           (STM32L4_RTC_BASE+STM32L4_RTC_BK22R_OFFSET)
#define STM32L4_RTC_BK23R           (STM32L4_RTC_BASE+STM32L4_RTC_BK23R_OFFSET)
#define STM32L4_RTC_BK24R           (STM32L4_RTC_BASE+STM32L4_RTC_BK24R_OFFSET)
#define STM32L4_RTC_BK25R           (STM32L4_RTC_BASE+STM32L4_RTC_BK25R_OFFSET)
#define STM32L4_RTC_BK26R           (STM32L4_RTC_BASE+STM32L4_RTC_BK26R_OFFSET)
#define STM32L4_RTC_BK27R           (STM32L4_RTC_BASE+STM32L4_RTC_BK27R_OFFSET)
#define STM32L4_RTC_BK28R           (STM32L4_RTC_BASE+STM32L4_RTC_BK28R_OFFSET)
#define STM32L4_RTC_BK29R           (STM32L4_RTC_BASE+STM32L4_RTC_BK29R_OFFSET)
#define STM32L4_RTC_BK30R           (STM32L4_RTC_BASE+STM32L4_RTC_BK30R_OFFSET)
#define STM32L4_RTC_BK31R           (STM32L4_RTC_BASE+STM32L4_RTC_BK31R_OFFSET)

#  define STM32L4_RTC_BKCOUNT       32

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
                                            /* Bit 7:  reserved */
#define RTC_CR_ALRAE              (1 << 8)  /* Bit 8:  Alarm A enable */
#define RTC_CR_ALRBE              (1 << 9)  /* Bit 9:  Alarm B enable */
#define RTC_CR_WUTE               (1 << 10) /* Bit 10: Wakeup timer enable */
#define RTC_CR_TSE                (1 << 11) /* Bit 11: Time stamp enable */
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

/* RTC initialization and status register */

#define RTC_ISR_ALRAWF            (1 << 0)  /* Bit 0:  Alarm A write flag */
#define RTC_ISR_ALRBWF            (1 << 1)  /* Bit 1:  Alarm B write flag */
#define RTC_ISR_WUTWF             (1 << 2)  /* Bit 2:  Wakeup timer write flag */
#define RTC_ISR_SHPF              (1 << 3)  /* Bit 3:  Shift operation pending */
#define RTC_ISR_INITS             (1 << 4)  /* Bit 4:  Initialization status flag */
#define RTC_ISR_RSF               (1 << 5)  /* Bit 5:  Registers synchronization flag */
#define RTC_ISR_INITF             (1 << 6)  /* Bit 6:  Initialization flag */
#define RTC_ISR_INIT              (1 << 7)  /* Bit 7:  Initialization mode */
#define RTC_ISR_ALRAF             (1 << 8)  /* Bit 8:  Alarm A flag */
#define RTC_ISR_ALRBF             (1 << 9)  /* Bit 9:  Alarm B flag */
#define RTC_ISR_WUTF              (1 << 10) /* Bit 10: Wakeup timer flag */
#define RTC_ISR_TSF               (1 << 11) /* Bit 11: Timestamp flag */
#define RTC_ISR_TSOVF             (1 << 12) /* Bit 12: Timestamp overflow flag */
#define RTC_ISR_TAMP1F            (1 << 13) /* Bit 13: Tamper detection flag */
#define RTC_ISR_TAMP2F            (1 << 14) /* Bit 14: TAMPER2 detection flag */
#define RTC_ISR_TAMP3F            (1 << 15) /* Bit 15: TAMPER3 detection flag */
#define RTC_ISR_RECALPF           (1 << 16) /* Bit 16: Recalibration pending flag */
#define RTC_ISR_ITSF              (1 << 17) /* Bit 17: Internal time-stamp flag */
#define RTC_ISR_ALLFLAGS          (0x0003ffff)

/* RTC prescaler register */

#define RTC_PRER_PREDIV_S_SHIFT   (0)       /* Bits 0-14: Synchronous prescaler factor */
#define RTC_PRER_PREDIV_S_MASK    (0x7fff << RTC_PRER_PREDIV_S_SHIFT)
#define RTC_PRER_PREDIV_A_SHIFT   (16)      /* Bits 16-22: Asynchronous prescaler factor */
#define RTC_PRER_PREDIV_A_MASK    (0x7f << RTC_PRER_PREDIV_A_SHIFT)

/* RTC wakeup timer register */

#define RTC_WUTR_MASK             (0xffff)  /* Bits 15:0  Wakeup auto-reload value bits */

/* RTC alarm A/B registers */

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

/* RTC tamper configuration register */

#define RTC_TAMPCR_TAMP1E         (1 << 0)  /* Bit 0:  RTC_TAMP1 input detection enable */
#define RTC_TAMPCR_TAMP1TRG       (1 << 1)  /* Bit 1:  Active level for RTC_TAMP1 input */
#define RTC_TAMPCR_TAMPIE         (1 << 2)  /* Bit 2:  Tamper interrupt enable */
#define RTC_TAMPCR_TAMP2E         (1 << 3)  /* Bit 3:  RTC_TAMP2 detection enable */
#define RTC_TAMPCR_TAMP2TRG       (1 << 4)  /* Bit 4:  Active level for RTC_TAMP2 input */
#define RTC_TAMPCR_TAMP3E         (1 << 5)  /* Bit 5:  RTC_TAMP3 detection enable */
#define RTC_TAMPCR_TAMP3TRG       (1 << 6)  /* Bit 6:  Active level for RTC_TAMP3 input */
#define RTC_TAMPCR_TAMPTS         (1 << 7)  /* Bit 7:  Activate timestamp on tamper detection event */
#define RTC_TAMPCR_TAMPFREQ_SHIFT (8)       /* Bits 8-10: Tamper sampling frequency */
#define RTC_TAMPCR_TAMPFREQ_MASK  (7 << RTC_TAMPCR_TAMPFREQ_SHIFT)
#  define RTC_TAMPCR_TAMPFREQ_DIV32768 (0 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 32768 (1 Hz) */
#  define RTC_TAMPCR_TAMPFREQ_DIV16384 (1 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 16384 (2 Hz) */
#  define RTC_TAMPCR_TAMPFREQ_DIV8192  (2 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 8192 (4 Hz) */
#  define RTC_TAMPCR_TAMPFREQ_DIV4096  (3 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 4096 (8 Hz) */
#  define RTC_TAMPCR_TAMPFREQ_DIV2048  (4 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 2048 (16 Hz) */
#  define RTC_TAMPCR_TAMPFREQ_DIV1024  (5 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 1024 (32 Hz) */
#  define RTC_TAMPCR_TAMPFREQ_DIV512   (6 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 512 (64 Hz) */
#  define RTC_TAMPCR_TAMPFREQ_DIV256   (7 << RTC_TAMPCR_TAMPFREQ_SHIFT) /* RTCCLK / 256 (128 Hz) */

#define RTC_TAMPCR_TAMPFLT_SHIFT  (11)        /* Bits 11-12: RTC_TAMPx filter count */
#define RTC_TAMPCR_TAMPFLT_MASK   (3 << RTC_TAMPCR_TAMPFLT_SHIFT)
#define RTC_TAMPCR_TAMPPRCH_SHIFT (13)        /* Bits 13-14: RTC_TAMPx precharge duration */
#define RTC_TAMPCR_TAMPPRCH_MASK  (3 << RTC_TAMPCR_TAMPPRCH_SHIFT)
#  define RTC_TAMPCR_TAMPPRCH_1CYCLE  (0 << RTC_TAMPCR_TAMPPRCH_SHIFT) /* 1 RTCCLK cycle */
#  define RTC_TAMPCR_TAMPPRCH_2CYCLES (1 << RTC_TAMPCR_TAMPPRCH_SHIFT) /* 2 RTCCLK cycles */
#  define RTC_TAMPCR_TAMPPRCH_4CYCLES (2 << RTC_TAMPCR_TAMPPRCH_SHIFT) /* 4 RTCCLK cycles */
#  define RTC_TAMPCR_TAMPPRCH_5CYCLES (3 << RTC_TAMPCR_TAMPPRCH_SHIFT) /* 8 RTCCLK cycles */

#define RTC_TAMPCR_TAMPPUDIS      (1 << 15) /* Bit 15: RTC_TAMPx pull-up disable */
#define RTC_TAMPCR_TAMP1IE        (1 << 16) /* Bit 16:  RTC_TAMP1 interrupt enable */
#define RTC_TAMPCR_TAMP1NOERASE   (1 << 17) /* Bit 17:  RTC_TAMP1 no erase */
#define RTC_TAMPCR_TAMP1F         (1 << 18) /* Bit 18:  RTC_TAMP1 mask flag */
#define RTC_TAMPCR_TAMP2IE        (1 << 19) /* Bit 19:  RTC_TAMP2 interrupt enable */
#define RTC_TAMPCR_TAMP2NOERASE   (1 << 20) /* Bit 20:  RTC_TAMP2 no erase */
#define RTC_TAMPCR_TAMP2MF        (1 << 21) /* Bit 21:  RTC_TAMP2 mask flag */
#define RTC_TAMPCR_TAMP3IE        (1 << 22) /* Bit 22:  RTC_TAMP3 interrupt enable */
#define RTC_TAMPCR_TAMP3NOERASE   (1 << 23) /* Bit 23:  RTC_TAMP3 no erase */
#define RTC_TAMPCR_TAMP3MF        (1 << 24) /* Bit 24:  RTC_TAMP3 mask flag */

/* RTC alarm A/B sub second register */

#define RTC_ALRMSSR_SS_SHIFT      (0)   /* Bits 0-14: Sub second value */
#define RTC_ALRMSSR_SS_MASK       (0x7fff << RTC_ALRMSSR_SS_SHIFT)
#define RTC_ALRMSSR_MASKSS_SHIFT  (24)  /* Bits 24-27:  Mask the most-significant bits starting at this bit */
#define RTC_ALRMSSR_MASKSS_MASK   (0xf << RTC_ALRMSSR_MASKSS_SHIFT)

/* RTC option register */

#define RTC_OR_ALARMTYPE          (1 << 0)  /* Bit 0:  RTC alarm type on PC13/PB14, pushpull/OD */
#define RTC_OR_OUTRMP             (1 << 1)  /* Bit 1:  remap output to PB14 */

#endif /* __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_RTCC_H */
