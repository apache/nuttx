/****************************************************************************
 * drivers/timers/ds3231.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __DRIVERS_TIMERS_DS3231_H
#define __DRIVERS_TIMERS_DS3231_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DSXXXX_TIME_SECR                 0x00      /* Seconds register */
#  define DSXXXX_TIME_SEC_SHIFT          0         /* Bits 0-3: Seconds, range 0-9 */
#  define DSXXXX_TIME_SEC_MASK           (15 << DSXXXX_TIME_SEC_SHIFT)
#    define DSXXXX_TIME_SEC(n)           ((uint8_t)(n) << DSXXXX_TIME_SEC_SHIFT)
#  define DSXXXX_TIME_10SEC_SHIFT        4         /* Bits 4-6: 10 seconds, range 0-5 */
#  define DSXXXX_TIME_10SEC_MASK         (7 << DSXXXX_TIME_10SEC_SHIFT)
#    define DSXXXX_TIME_10SEC(n)         ((uint8_t)(n) << DSXXXX_TIME_10SEC_SHIFT)
#  define DSXXXX_TIME_SEC_BCDMASK        (DSXXXX_TIME_SEC_MASK | DSXXXX_TIME_10SEC_MASK)
#if defined(CONFIG_RTC_DS1302) || defined(CONFIG_RTC_DS1307)
#  define DS130x_TIME_CH                 (1 << 7)  /* Bit 7: Clock halt */
#endif

#define DSXXXX_TIME_MINR                 0x01      /* Minutes register */
#  define DSXXXX_TIME_MIN_SHIFT          0         /* Bits 0-3: Minutes, range 0-9 */
#  define DSXXXX_TIME_MIN_MASK           (15 << DSXXXX_TIME_MIN_SHIFT)
#    define DSXXXX_TIME_MIN(n)           ((uint8_t)(n) << DSXXXX_TIME_MIN_SHIFT)
#  define DSXXXX_TIME_10MIN_SHIFT        4         /* Bits 4-6: 10 minutes, range 0-5 */
#  define DSXXXX_TIME_10MIN_MASK         (7 << DSXXXX_TIME_10MIN_SHIFT)
#    define DSXXXX_TIME_10MIN(n)         ((uint8_t)(n) << DSXXXX_TIME_10MIN_SHIFT)
#  define DSXXXX_TIME_MIN_BCDMASK        (DSXXXX_TIME_MIN_MASK | DSXXXX_TIME_10MIN_MASK)

#define DSXXXX_TIME_HOURR                0x02      /* Hours register */
#  define DSXXXX_TIME_HOUR_SHIFT         0         /* Bits 0-3: Hours, range 0-9 */
#  define DSXXXX_TIME_HOUR_MASK          (15 << DSXXXX_TIME_HOUR_SHIFT)
#    define DSXXXX_TIME_HOUR(n)          ((uint8_t)(n) << DSXXXX_TIME_HOUR_SHIFT)
#  define DSXXXX_TIME_10HOUR12_SHIFT     4         /* Bit 4: 10 hours, range 0-1 */
#  define DSXXXX_TIME_10HOUR12_MASK      (1 << DSXXXX_TIME_10HOUR12_SHIFT)
#    define DSXXXX_TIME_10HOUR12(n)      ((uint8_t)(n) << DSXXXX_TIME_10HOUR12_SHIFT)
#  define DSXXXX_TIME_10HOUR24_SHIFT     4         /* Bits 4-5: 10 hours, range 0-2 */
#  define DSXXXX_TIME_10HOUR24_MASK      (3 << DSXXXX_TIME_10HOUR24_SHIFT)
#    define DSXXXX_TIME_10HOUR24(n)      ((uint8_t)(n) << DSXXXX_TIME_10HOUR24_SHIFT)
#  define DSXXXX_TIME_HOUR12_BCDMASK     (DSXXXX_TIME_HOUR_MASK | DSXXXX_TIME_10HOUR12_MASK)
#  define DSXXXX_TIME_HOUR24_BCDMASK     (DSXXXX_TIME_HOUR_MASK | DSXXXX_TIME_10HOUR24_MASK)
#  define DSXXXX_TIME_AMPM_SHIFT          5         /* Bit 5: AM/PM Indication */
#  define DSXXXX_TIME_AMPM_MASK          (1 << DSXXXX_TIME_AMPM_SHIFT)
#    define DSXXXX_TIME_AM               ((uint8_t)(0) << DSXXXX_TIME_AMPM_SHIFT)
#    define DSXXXX_TIME_PM               ((uint8_t)(1) << DSXXXX_TIME_AMPM_SHIFT)
#  define DSXXXX_TIME_1224_SHIFT         6         /* Bit 6: 12/24 Indication */
#  define DSXXXX_TIME_1224_MASK          (1 << DSXXXX_TIME_1224_SHIFT)
#    define DSXXXX_TIME_24               ((uint8_t)(0) << DSXXXX_TIME_1224_SHIFT)
#    define DSXXXX_TIME_12               ((uint8_t)(1) << DSXXXX_TIME_1224_SHIFT)

#define DSXXXX_TIME_DAYR                 0x03      /* Day of the week register */
#  define DSXXXX_TIME_DAY_SHIFT          0         /* Bits 0-3: Day of the week, range 1-7 */
#  define DSXXXX_TIME_DAY_MASK           (7 << DSXXXX_TIME_DAY_SHIFT)
#    define DSXXXX_TIME_DAY(n)           ((uint8_t)(n) << DSXXXX_TIME_DAY_SHIFT)

#define DSXXXX_TIME_DATER                0x04      /* Date register */
#  define DSXXXX_TIME_DATE_SHIFT         0         /* Bits 0-3: Days, range 0-9 */
#  define DSXXXX_TIME_DATE_MASK          (15 << DSXXXX_TIME_DATE_SHIFT)
#    define DSXXXX_TIME_DATE(n)          ((uint8_t)(n) << DSXXXX_TIME_DATE_SHIFT)
#  define DSXXXX_TIME_10DATE_SHIFT       4         /* Bits 4-5: 10 days, range 0-5 */
#  define DSXXXX_TIME_10DATE_MASK        (3 << DSXXXX_TIME_10DATE_SHIFT)
#    define DSXXXX_TIME_10DATE(n)        ((uint8_t)(n) << DSXXXX_TIME_10DATE_SHIFT)
#  define DSXXXX_TIME_DATE_BCDMASK       (DSXXXX_TIME_DATE_MASK | DSXXXX_TIME_10DATE_MASK)

#define DSXXXX_TIME_MONTHR               0x05      /* Month register */
#  define DSXXXX_TIME_MONTH_SHIFT        0         /* Bits 0-3: Month, range 0-9 */
#  define DSXXXX_TIME_MONTH_MASK         (15 << DSXXXX_TIME_MONTH_SHIFT)
#    define DSXXXX_TIME_MONTH(n)         ((uint8_t)(n) << DSXXXX_TIME_MONTH_SHIFT)
#  define DSXXXX_TIME_10MONTH_SHIFT      4         /* Bit 4: 10 month, range 0-1 */
#  define DSXXXX_TIME_10MONTH_MASK       (1 << DSXXXX_TIME_10MONTH_SHIFT)
#    define DSXXXX_TIME_10MONTH(n)       ((uint8_t)(n) << DSXXXX_TIME_10MONTH_SHIFT)
#  define DSXXXX_TIME_MONTH_BCDMASK      (DSXXXX_TIME_MONTH_MASK | DSXXXX_TIME_10MONTH_MASK)
#if defined(CONFIG_RTC_DS3231) || defined(CONFIG_RTC_DS3232) || defined(CONFIG_RTC_DS3234)
#  define DS323X_TIME_CENTURY_SHIFT      7         /* Bit 7: Century Indication */
#  define DS323X_TIME_CENTURY_MASK       (1 << DS323X_TIME_CENTURY_SHIFT)
#    define DS323X_TIME_1900             ((uint8_t)(0) << DS323X_TIME_CENTURY_SHIFT)
#    define DS323X_TIME_2000             ((uint8_t)(1) << DS323X_TIME_CENTURY_SHIFT)
#endif

#define DSXXXX_TIME_YEARR                0x06      /* Date register */
#  define DSXXXX_TIME_YEAR_SHIFT         0         /* Bits 0-3: Year, range 0-9 */
#  define DSXXXX_TIME_YEAR_MASK          (15 << DSXXXX_TIME_YEAR_SHIFT)
#    define DSXXXX_TIME_YEAR(n)          ((uint8_t)(n) << DSXXXX_TIME_YEAR_SHIFT)
#  define DSXXXX_TIME_10YEAR_SHIFT       4         /* Bits 4-7: 10 year, range 0-9 */
#  define DSXXXX_TIME_10YEAR_MASK        (15 << DSXXXX_TIME_10YEAR_SHIFT)
#    define DSXXXX_TIME_10YEAR(n)        ((uint8_t)(n) << DSXXXX_TIME_10YEAR_SHIFT)
#  define DSXXXX_TIME_YEAR_BCDMASK       (DSXXXX_TIME_YEAR_MASK | DSXXXX_TIME_10YEAR_MASK)

#ifdef CONFIG_RTC_DS1302
#  define DS1302_CR                      0x07      /* Control register */
#    define DS1302_CR_WP                 (1 << 7)  /* Bit 7:  Write protect */

#  define DS1302_TCR                     0x08      /* Trickle charge register */
#    define DS1302_TCR_RS_SHIFT          (0)       /* Bits 0-1: Reistance select */
#    define DS1302_TCR_RS_MASK           (3 << DS1302_TCR_RS_SHIFT)
#      define DS1302_TCR_RS(n)           ((uint8_t)(n) << DS1302_TCR_RS_SHIFT)
#      define DS1302_TCR_RS_DISABLED     (0 << DS1302_TCR_RS_SHIFT)
#      define DS1302_TCR_RS_2OHM         (1 << DS1302_TCR_RS_SHIFT)
#      define DS1302_TCR_RS_4OHM         (2 << DS1302_TCR_RS_SHIFT)
#      define DS1302_TCR_RS_8OHM         (3 << DS1302_TCR_RS_SHIFT)
#    define DS1302_TCR_DS_SHIFT          (4)       /* Bits 2-3: Diode select */
#    define DS1302_TCR_DS_MASK           (3 << DS1302_TCR_DS_SHIFT)
#      define DS1302_TCR_DS(n)           ((uint8_t)(n) << DS1302_TCR_DS_SHIFT)
#      define DS1302_TCR_DS_DISABLED     (0 << DS1302_TCR_DS_SHIFT)
#      define DS1302_TCR_DS_1DIODE       (1 << DS1302_TCR_DS_SHIFT)
#      define DS1302_TCR_DS_2DIODE       (2 << DS1302_TCR_DS_SHIFT)
#      define DS1302_TCR_DS_DISABLED_2   (3 << DS1302_TCR_DS_SHIFT)
#    define DS1302_TCR_TCS_SHIFT         (4)       /* Bits 4-7: Trickle charge select */
#    define DS1302_TCR_TCS_MASK          (15 << DS1302_TCR_TCS_SHIFT)
#      define DS1302_TCR_TCS(n)          ((uint8_t)(n) << DS1302_TCR_TCS_SHIFT)

#  define DS1302_TCR_DISABLED            (DS1302_TCR_RS_DISABLED | DS1302_TCR_DS_DISABLED   | define DS1302_TCR_TCS(0))
#  define DS1302_TCR_1DIODE_2OHM         (DS1302_TCR_RS_2OHM     | DS1302_TCR_DS_1DIODE     | define DS1302_TCR_TCS(10))
#  define DS1302_TCR_1DIODE_4OHM         (DS1302_TCR_RS_4OHM     | DS1302_TCR_DS_1DIODE     | define DS1302_TCR_TCS(10))
#  define DS1302_TCR_1DIODE_8OHM         (DS1302_TCR_RS_8OHM     | DS1302_TCR_DS_1DIODE     | define DS1302_TCR_TCS(10))
#  define DS1302_TCR_2DIODE_2OHM         (DS1302_TCR_RS_2OHM     | DS1302_TCR_DS_2DIODE     | define DS1302_TCR_TCS(10))
#  define DS1302_TCR_2DIODE_4OHM         (DS1302_TCR_RS_4OHM     | DS1302_TCR_DS_2DIODE     | define DS1302_TCR_TCS(10))
#  define DS1302_TCR_2DIODE_8OHM         (DS1302_TCR_RS_8OHM     | DS1302_TCR_DS_2DIODE     | define DS1302_TCR_TCS(10))
#  define DS1302_TCR_INIT                (DS1302_TCR_RS_DISABLED | DS1302_TCR_DS_DISABLED_2 | define DS1302_TCR_TCS(5))
#endif

#ifdef CONFIG_RTC_DS1307
#  define DS1307_CR                      0x07      /* Control register */
#    define DS1307_CR_RS_SHIFT           (3)       /* Bits 0-1:  Rate selection */
#    define DS1307_CR_RS_MASK            (3 << DS1307_CR_RS_SHIFT)
#      define DS1307_CR_RS_1HZ           (0 << DS1307_CR_RS_SHIFT) /* 1Hz */
#      define DS1307_CR_RS_4KHZ          (1 << DS1307_CR_RS_SHIFT) /* 4.096kHz */
#      define DS1307_CR_RS_8KHZ          (2 << DS1307_CR_RS_SHIFT) /* 8.192kHz */
#      define DS1307_CR_RS_32KHZ         (3 << DS1307_CR_RS_SHIFT) /* 32.768kHz */
#    define DS1307_CR_SQWE               (1 << 4)  /* Bit 4:  Square wave enable */
#    define DS1307_CR_OUT                (1 << 7)  /* Bit 7:  Output control */
#  define DS1307_RAM_BASER               0x08      /* 0x08-0x3f: 56x8 RAM */
#endif

#if defined(CONFIG_RTC_DS3231) || defined(CONFIG_RTC_DS3232) || defined(CONFIG_RTC_DS3234)
#  define DS323X_ALARM1_SECR             0x07      /* Alarm1 seconds register */
#    define DS323X_ALARM1_SEC_SHIFT      0         /* Bits 0-3: Seconds, range 0-9 */
#    define DS323X_ALARM1_SEC_MASK       (15 << DS323X_ALARM1_SEC_SHIFT)
#      define DS323X_ALARM1_SEC(n)       ((uint8_t)(n) << DS323X_ALARM1_SEC_SHIFT)
#    define DS323X_ALARM1_10SEC_SHIFT    4         /* Bits 4-6: 10 seconds, range 0-5 */
#    define DS323X_ALARM1_10SEC_MASK     (7 << DS323X_ALARM1_10SEC_SHIFT)
#      define DS323X_ALARM1_10SEC(n)     ((uint8_t)(n) << DS323X_ALARM1_10SEC_SHIFT)
#    define DS323X_ALARM1_SEC_BCDMASK    (DS323X_ALARM1_SEC_MASK | DS323X_ALARM1_10SEC_MASK)
#    define DS323X_ALARM1_A1M1_SHIFT     7         /* Bits 7: A1M1 mask */
#    define DS323X_ALARM1_A1M1_MASK      (1 << DS323X_ALARM1_A1M1_SHIFT)
#      define DS323X_ALARM1_A1M1(n)      ((uint8_t)(n) << DS323X_ALARM1_A1M1_SHIFT)

#  define DS323X_ALARM1_MINR             0x08      /* Alarm1 minutes register */
#    define DS323X_ALARM1_MIN_SHIFT      0         /* Bits 0-3: Minutes, range 0-9 */
#    define DS323X_ALARM1_MIN_MASK       (15 << DS323X_ALARM1_MIN_SHIFT)
#      define DS323X_ALARM1_MIN(n)       ((uint8_t)(n) << DS323X_ALARM1_MIN_SHIFT)
#    define DS323X_ALARM1_10MIN_SHIFT    4         /* Bits 4-6: 10 minutes, range 0-5 */
#    define DS323X_ALARM1_10MIN_MASK     (7 << DS323X_ALARM1_10MIN_SHIFT)
#      define DS323X_ALARM1_10MIN(n)     ((uint8_t)(n) << DS323X_ALARM1_10MIN_SHIFT)
#    define DS323X_ALARM1_MIN_BCDMASK    (DS323X_ALARM1_MIN_MASK | DS323X_ALARM1_10MIN_MASK)
#    define DS323X_ALARM1_A1M2_SHIFT     7         /* Bits 7: A1M2 mask */
#    define DS323X_ALARM1_A1M2_MASK      (1 << DS323X_ALARM1_A1M2_SHIFT)
#      define DS323X_ALARM1_A1M2(n)      ((uint8_t)(n) << DS323X_ALARM1_A1M2_SHIFT)

#  define DS323X_ALARM1_HOURR            0x09      /* Alarm1 hours register */
#    define DS323X_ALARM1_HOUR_SHIFT     0         /* Bits 0-3: Hours, range 0-9 */
#    define DS323X_ALARM1_HOUR_MASK      (15 << DS323X_ALARM1_HOUR_SHIFT)
#      define DS323X_ALARM1_HOUR(n)      ((uint8_t)(n) << DS323X_ALARM1_HOUR_SHIFT)
#    define DS323X_ALARM1_10HOUR12_SHIFT 4         /* Bit 4: 10 hours, range 0-1 */
#    define DS323X_ALARM1_10HOUR12_MASK  (1 << DS323X_ALARM1_10HOUR12_SHIFT)
#      define DS323X_ALARM1_10HOUR12(n)  ((uint8_t)(n) << DS323X_ALARM1_10HOUR12_SHIFT)
#    define DS323X_ALARM1_10HOUR24_SHIFT 4         /* Bits 4-5: 10 hours, range 0-2 */
#    define DS323X_ALARM1_10HOUR24_MASK  (3 << DS323X_ALARM1_10HOUR24_SHIFT)
#      define DS323X_ALARM1_10HOUR24(n)  ((uint8_t)(n) << DS323X_ALARM1_10HOUR24_SHIFT)
#    define DS323X_ALARM1_HOUR12_BCDMASK (DS323X_ALARM1_HOUR_MASK | DS323X_ALARM1_10HOUR12_MASK)
#    define DS323X_ALARM1_HOUR24_BCDMASK (DS323X_ALARM1_HOUR_MASK | DS323X_ALARM1_10HOUR24_MASK)
#    define DS323X_ALARM1_AMPM_SHIFT     5         /* Bit 5: AM/PM Indication */
#    define DS323X_ALARM1_AMPM_MASK      (1 << DS323X_ALARM1_AMPM_SHIFT)
#      define DS323X_ALARM1_AM           ((uint8_t)(0) << DS323X_ALARM1_AMPM_SHIFT)
#      define DS323X_ALARM1_PM           ((uint8_t)(1) << DS323X_ALARM1_AMPM_SHIFT)
#    define DS323X_ALARM1_1224_SHIFT     6         /* Bit 6: 12/24 Indication */
#    define DS323X_ALARM1_1224_MASK      (1 << DS323X_ALARM1_1224_SHIFT)
#      define DS323X_ALARM1_12           ((uint8_t)(0) << DS323X_ALARM1_1224_SHIFT)
#      define DS323X_ALARM1_24           ((uint8_t)(1) << DS323X_ALARM1_1224_SHIFT)
#    define DS323X_ALARM1_A1M3_SHIFT     7         /* Bits 7: A1M3 mask */
#    define DS323X_ALARM1_A1M3_MASK      (1 << DS323X_ALARM1_A1M3_SHIFT)
#      define DS323X_ALARM1_A1M3(n)      ((uint8_t)(n) << DS323X_ALARM1_A1M3_SHIFT)

#  define DS323X_ALARM1_DAYDATER         0x0a      /* Alarm1 date / day of the week register */
#    define DS323X_ALARM1_DAY_SHIFT      0         /* Bits 0-3: Day of the week, range 1-7 */
#    define DS323X_ALARM1_DAY_MASK       (7 << DS323X_ALARM1_DAY_SHIFT)
#      define DS323X_ALARM1_DAY(n)       ((uint8_t)(n) << DS323X_ALARM1_DAY_SHIFT)
#    define DS323X_ALARM1_DATE_SHIFT     0         /* Bits 0-3: Days, range 0-9 */
#    define DS323X_ALARM1_DATE_MASK      (15 << DS323X_ALARM1_DATE_SHIFT)
#      define DS323X_ALARM1_DATE(n)      ((uint8_t)(n) << DS323X_ALARM1_DATE_SHIFT)
#    define DS323X_ALARM1_10DATE_SHIFT   4         /* Bits 4-5: 10 days, range 0-5 */
#    define DS323X_ALARM1_10DATE_MASK    (3 << DS323X_ALARM1_10DATE_SHIFT)
#      define DS323X_ALARM1_10DATE(n)    ((uint8_t)(n) << DS323X_ALARM1_10DATE_SHIFT)
#    define DS323X_ALARM1_DATE_BCDMASK   (DS323X_ALARM1_DATE_MASK | DS323X_ALARM1_10DATE_MASK)
#    define DS323X_ALARM1_DYDT_SHIFT     6         /* Bits 6: DY/DT */
#    define DS323X_ALARM1_DYDT_MASK      (1 << DS323X_ALARM1_DYDT_SHIFT)
#      define DS323X_ALARM1_DYDT_DATE    ((uint8_t)(0) << DS323X_ALARM1_DYDT_SHIFT)
#      define DS323X_ALARM1_DYDT_DAY     ((uint8_t)(1) << DS323X_ALARM1_DYDT_SHIFT)
#    define DS323X_ALARM1_A1M4_SHIFT     7         /* Bits 7: A1M4 mask */
#    define DS323X_ALARM1_A1M4_MASK      (1 << DS323X_ALARM1_A1M4_SHIFT)
#      define DS323X_ALARM1_A1M4(n)      ((uint8_t)(n) << DS323X_ALARM1_A1M4_SHIFT)

#  define DS323X_ALARM2_MINR             0x0b      /* Alarm2 minutes register */
#    define DS323X_ALARM2_MIN_SHIFT      0         /* Bits 0-3: Minutes, range 0-9 */
#    define DS323X_ALARM2_MIN_MASK       (15 << DS323X_ALARM2_MIN_SHIFT)
#      define DS323X_ALARM2_MIN(n)       ((uint8_t)(n) << DS323X_ALARM2_MIN_SHIFT)
#    define DS323X_ALARM2_10MIN_SHIFT    4         /* Bits 4-6: 10 minutes, range 0-5 */
#    define DS323X_ALARM2_10MIN_MASK     (7 << DS323X_ALARM2_10MIN_SHIFT)
#      define DS323X_ALARM2_10MIN(n)     ((uint8_t)(n) << DS323X_ALARM2_10MIN_SHIFT)
#    define DS323X_ALARM2_MIN_BCDMASK    (DS323X_ALARM2_MIN_MASK | DS323X_ALARM2_10MIN_MASK)
#    define DS323X_ALARM2_A2M2_SHIFT     7         /* Bits 7: A2M2 mask */
#    define DS323X_ALARM2_A2M2_MASK      (1 << DS323X_ALARM2_A2M2_SHIFT)
#      define DS323X_ALARM2_A2M2(n)      ((uint8_t)(n) << DS323X_ALARM2_A2M2_SHIFT)

#  define DS323X_ALARM2_HOURR            0x0c      /* Alarm2 hours register */
#    define DS323X_ALARM2_HOUR_SHIFT     0         /* Bits 0-3: Hours, range 0-9 */
#    define DS323X_ALARM2_HOUR_MASK      (15 << DS323X_ALARM2_HOUR_SHIFT)
#      define DS323X_ALARM2_HOUR(n)      ((uint8_t)(n) << DS323X_ALARM2_HOUR_SHIFT)
#    define DS323X_ALARM2_10HOUR12_SHIFT 4         /* Bit 4: 10 hours, range 0-1 */
#    define DS323X_ALARM2_10HOUR12_MASK  (1 << DS323X_ALARM2_10HOUR12_SHIFT)
#      define DS323X_ALARM2_10HOUR12(n)  ((uint8_t)(n) << DS323X_ALARM2_10HOUR12_SHIFT)
#    define DS323X_ALARM2_10HOUR24_SHIFT 4         /* Bits 4-5: 10 hours, range 0-2 */
#    define DS323X_ALARM2_10HOUR24_MASK  (3 << DS323X_ALARM2_10HOUR24_SHIFT)
#      define DS323X_ALARM2_10HOUR24(n)  ((uint8_t)(n) << DS323X_ALARM2_10HOUR24_SHIFT)
#    define DS323X_ALARM2_HOUR12_BCDMASK (DS323X_ALARM2_HOUR_MASK | DS323X_ALARM2_10HOUR12_MASK)
#    define DS323X_ALARM2_HOUR24_BCDMASK (DS323X_ALARM2_HOUR_MASK | DS323X_ALARM2_10HOUR24_MASK)
#    define DS323X_ALARM2_AMPM_SHIFT     5         /* Bit 5: AM/PM Indication */
#    define DS323X_ALARM2_AMPM_MASK      (1 << DS323X_ALARM2_AMPM_SHIFT)
#      define DS323X_ALARM2_AM           ((uint8_t)(0) << DS323X_ALARM2_AMPM_SHIFT)
#      define DS323X_ALARM2_PM           ((uint8_t)(1) << DS323X_ALARM2_AMPM_SHIFT)
#    define DS323X_ALARM2_1224_SHIFT     6         /* Bit 6: 12/24 Indication */
#    define DS323X_ALARM2_1224_MASK      (1 << DS323X_ALARM2_1224_SHIFT)
#      define DS323X_ALARM2_12           ((uint8_t)(0) << DS323X_ALARM2_1224_SHIFT)
#      define DS323X_ALARM2_24           ((uint8_t)(1) << DS323X_ALARM2_1224_SHIFT)
#    define DS323X_ALARM2_A2M3_SHIFT     7         /* Bits 7: A2M3 mask */
#    define DS323X_ALARM2_A2M3_MASK      (1 << DS323X_ALARM2_A2M3_SHIFT)
#      define DS323X_ALARM2_A2M3(n)      ((uint8_t)(n) << DS323X_ALARM2_A2M3_SHIFT)

#  define DS323X_ALARM2_DAYDATER         0x0d      /* Alarm2 date / day of the week register */
#    define DS323X_ALARM2_DAY_SHIFT      0         /* Bits 0-3: Day of the week, range 1-7 */
#    define DS323X_ALARM2_DAY_MASK       (7 << DS323X_ALARM2_DAY_SHIFT)
#      define DS323X_ALARM2_DAY(n)       ((uint8_t)(n) << DS323X_ALARM2_DAY_SHIFT)
#    define DS323X_ALARM2_DATE_SHIFT     0         /* Bits 0-3: Days, range 0-9 */
#    define DS323X_ALARM2_DATE_MASK      (15 << DS323X_ALARM2_DATE_SHIFT)
#      define DS323X_ALARM2_DATE(n)      ((uint8_t)(n) << DS323X_ALARM2_DATE_SHIFT)
#    define DS323X_ALARM2_10DATE_SHIFT   4         /* Bits 4-5: 10 days, range 0-5 */
#    define DS323X_ALARM2_10DATE_MASK    (3 << DS323X_ALARM2_10DATE_SHIFT)
#      define DS323X_ALARM2_10DATE(n)    ((uint8_t)(n) << DS323X_ALARM2_10DATE_SHIFT)
#    define DS323X_ALARM2_DATE_BCDMASK   (DS323X_ALARM2_DATE_MASK | DS323X_ALARM2_10DATE_MASK)
#    define DS323X_ALARM2_DYDT_SHIFT     6         /* Bits 6: DY/DT */
#    define DS323X_ALARM2_DYDT_MASK      (1 << DS323X_ALARM2_DYDT_SHIFT)
#      define DS323X_ALARM2_DYDT_DATE    ((uint8_t)(0) << DS323X_ALARM2_DYDT_SHIFT)
#      define DS323X_ALARM2_DYDT_DAY     ((uint8_t)(1) << DS323X_ALARM2_DYDT_SHIFT)
#    define DS323X_ALARM2_A2M4_SHIFT     7         /* Bits 7: A2M4 mask */
#    define DS323X_ALARM2_A2M4_MASK      (1 << DS323X_ALARM2_A2M4_SHIFT)
#      define DS323X_ALARM2_A2M4(n)      ((uint8_t)(n) << DS3231_ALARM2_A2M4_SHIFT)

#  define DS323X_CR                      0x0e      /* Control register */
#    define DS323X_CR_A1IE               (1 << 0)  /* Bit 0:  Alarm 1 interrupt enable */
#    define DS323X_CR_A2IE               (1 << 1)  /* Bit 1:  Alarm 2 interrupt enable */
#    define DS323X_CR_INTCN              (1 << 2)  /* Bit 2:  Interrupt control */
#    define DS323X_CR_RS_SHIFT           (3)       /* Bits 3-4:  Rate selection */
#    define DS323X_CR_RS_MASK            (3 << DS323X_CR_RS_SHIFT)
#      define DS323X_CR_RS_1HZ           (0 << DS323X_CR_RS_SHIFT) /* 1Hz */
#      define DS323X_CR_RS_1KHZ          (1 << DS323X_CR_RS_SHIFT) /* 1.024kHz */
#      define DS323X_CR_RS_4KHZ          (2 << DS323X_CR_RS_SHIFT) /* 4.096kHz */
#      define DS323X_CR_RS_8KHZ          (3 << DS323X_CR_RS_SHIFT) /* 8.192kHz */
#    define DS323X_CR_CONV               (1 << 5)  /* Bit 5:  Convert temperature */
#    define DS323X_CR_BBSQW              (1 << 6)  /* Bit 6:  Battery backed square wave enable */
#    define DS323X_CR_EOSC               (1 << 7)  /* Bit 7:  Enable oscillator */

#  define DS323X_CSR                     0x0f      /* Control/status register */
#    define DS323X_CSR_A1F               (1 << 0)  /* Bit 0:  Alarm 1 flag */
#    define DS323X_CSR_A2F               (1 << 1)  /* Bit 1:  Alarm 2 flag */
#    define DS323X_CSR_BSY               (1 << 2)  /* Bit 2:  Busy */
#    define DS323X_CSR_EN32kHz           (1 << 3)  /* Bit 3:  Enable 32kHz output */
#    if defined(CONFIG_RTC_DS3232) ||  defined(CONFIG_RTC_DS3234)
#      define DS323x_CSR_CRATE_SHIFT     (4)       /* Bits 4-5: Conversion rate */
#      define DS323x_CSR_CRATE_MASK      (3 << DS323x_CSR_CRATE_SHIFT)
#        define DS323x_CSR_CRATE_64SEC   (0 << DS323x_CSR_CRATE_SHIFT)
#        define DS323x_CSR_CRATE_128SEC  (1 << DS323x_CSR_CRATE_SHIFT)
#        define DS323x_CSR_CRATE_256SEC  (2 << DS323x_CSR_CRATE_SHIFT)
#        define DS323x_CSR_CRATE_512SEC  (3 << DS323x_CSR_CRATE_SHIFT)
#      define DS323x_CSR_BB32KHZ         (1 << 6)  /* Bit 6: Battery-Backed 32kHz Output */
#    endif
#    define DS323X_CSR_OSF               (1 << 7)  /* Bit 7:  Oscillator stop flag */

#  define DS323X_AGINGR                  0x10      /* Aging offset register (8-bit, 2's complement) */

#  define DS323X_TMPMR                   0x11      /* MSB of temp register (8-bit, 2's complement) */

#  define DS323X_TMPLR                   0x12      /* LSB of temp register (2-bits) */
#    define DS323X_TMPLR_MASK            0xc0      /* Bits 6-7: LSB of temp register (2-bits) */
#endif /* CONFIG_RTC_DS3231 || CONFIG_RTC_DS3232 || CONFIG_RTC_DS3234 */

#ifdef CONFIG_RTC_DS3232
#  define DS3232_SRAM_BASER              0x14      /* 0x14-0xff: SRAM */
#endif /* CONFIG_RTC_DS3232 */

#ifdef CONFIG_RTC_DS3234
#  define DS3234_SRAM_ADDRR              0x98      /* SRAM address register */
#  define DS3234_SRAM_DATAR              0x99      /* SRAM data register */
#endif /* CONFIG_RTC_DS3234 */

#endif /* __DRIVERS_TIMERS_DS3231_H */
