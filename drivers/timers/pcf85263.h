/****************************************************************************
 * drivers/timers/pcf85263.h
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

#ifndef __DRIVERS_TIMERS_PCF85263_H
#define __DRIVERS_TIMERS_PCF85263_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTC time and date registers */

#define PCF85263_RTC_100TH_SECONDS         0x00      /* RTC 100ths of seconds register */
                                                     /* Bits 0-7: 100ths of seconds register (0-99 BCD) */

#define PCF85263_RTC_SECONDS               0x01      /* RTC seconds register */
#  define PCF85263_RTC_SECONDS_MASK        (0x7f)    /* Bits 0-6: Seconds (0-59 BCD) */
#  define PCF85263_RTC_SECONDS_OS          (1 << 7)  /* Bit 7: Oscillator stop */

#define PCF85263_RTC_MINUTES               0x02      /* RTC minutes register */
#  define PCF85263_RTC_MINUTES_MASK        (0x7f)    /* Bits 0-6: Minutes (0-59 BCD) */
#  define PCF85263_RTC_MINUTES_EMON        (1 << 7)  /* Bit 7: Event monitor */

#define PCF85263_RTC_HOURS                 0x03      /* RTC hours register */
#  define PCF85263_RTC_HOURS12_MASK        0x1f      /* Bits 0-4: Hours (1-12 BCD) */
#  define PCF85263_RTC_AMPM                (1 << 5)  /* Bit 5: AM/PM */
#  define PCF85263_RTC_HOURS24_MASK        0x3f      /* Bits 0-5: Hours (0-23  BCD) */

#define PCF85263_RTC_DAYS                  0x04      /* RTC days register */
#  define PCF85263_RTC_DAYS_MASK           0x3f      /* Bit 0-5: Day of the month (1-31 BCD) */

#define PCF85263_RTC_WEEKDAYS              0x05      /* RTC day-of-week register */
#  define PCF85263_RTC_WEEKDAYS_MASK       0x07      /* Bits 0-2: Day of the week (0-6) */

#define PCF85263_RTC_MONTHS                0x06      /* RTC month register */
#  define PCF85263_RTC_MONTHS_MASK         0x1f      /* Bits 0-4: Month (1-12 BCD) */

#define PCF85263_RTC_YEARS                 0x07      /* RTC year register */
                                                     /* Bits 0-7: Year (0-99 BCD) */

/* RTC alarm1 */

#define PCF85263_RTC_SECOND_ALARM1         0x08      /* RTC alarm1 seconds register */
#  define PCF85263_RTC_SECOND_ALARM1_MASK  0x7f      /* Bits 0-6:  Seconds (0-59 BCD) */

#define PCF85263_RTC_MINUTE_ALARM1         0x09      /* RTC alarm1 minutes register */
#  define PCF85263_RTC_MINUTE_ALARM1_MASK  0x7f      /* Bits 0-6:  Minutes (0-59 BCD) */

#define PCF85263_RTC_HOUR_ALARM1           0x0a      /* RTC alarm1 hours register */
#  define PCF85263_RTC_HOURS12_ALARM1_MASK 0x1f      /* Bits 0-4: Hours (1-12 BCD) */
#  define PCF85263_RTC_AMPM_ALARM1         (1 << 5)  /* Bit 5: AM/PM */
#  define PCF85263_RTC_HOURS24_ALARM1_MASK 0x3f      /* Bits 0-5: Hours (0-23  BCD) */

#define PCF85263_RTC_DAY_ALARM1            0x0b      /* RTC alarm1 days register */
#  define PCF85263_RTC_DAY_ALARM1_MASK     0x3f      /* Bits 0-5: Days (1-31 BCD) */

#define PCF85263_RTC_MONTH_ALARM1          0x0c      /* RTC alarm1 month register */
#  define PCF85263_RTC_MONTH_ALARM1_MASK   0x1f      /* Bits 0-4: Month (1-12 BCD) */

/* RTC alarm2 */

#define PCF85263_RTC_MINUTE_ALARM2         0x0d      /* RTC alarm2 seconds register */
#  define PCF85263_RTC_MINUTE_ALARM2_MASK  0x7f      /* Bits 0-6:  Minutes (0-59 BCD) */

#define PCF85263_RTC_HOUR_ALARM2           0x0e      /* RTC alarm1 minutes register */
#  define PCF85263_RTC_HOURS12_ALARM2_MASK 0x1f      /* Bits 0-4: Hours (1-12 BCD) */
#  define PCF85263_RTC_AMPM_ALARM2         (1 << 5)  /* Bit 5: AM/PM */
#  define PCF85263_RTC_HOURS24_ALARM2_MASK 0x3f      /* Bits 0-5: Hours (0-23  BCD) */

#define PCF85263_RTC_WEEKDAY_ALARM         0x0f      /* RTC alarm1 day-of-week register */
#  define PCF85263_RTC_WEEKDAY_ALARM_MASK  0x07      /* Bits 0-2: Day-of-week (0-6) */

/* RTC alarm enables */

#define PCF85263_RTC_ALARM_ENABLES         0x10      /* RTC alarem enables */
#  define PCF85263_RTC_ALARM_SEC_A1E       (1 << 0)  /* Second alarm1 enable */
#  define PCF85263_RTC_ALARM_MIN_A1E       (1 << 1)  /* Minute alarm1 enable */
#  define PCF85263_RTC_ALARM_HR_A1E        (1 << 2)  /* Hour alarm1 enable */
#  define PCF85263_RTC_ALARM_DAY_A1E       (1 << 3)  /* Day alarm1 enable */
#  define PCF85263_RTC_ALARM_MON_A1E       (1 << 4)  /* Month alarm1 enable */
#  define PCF85263_RTC_ALARM_MIN_A2E       (1 << 5)  /* Minute alarm2 enable */
#  define PCF85263_RTC_ALARM_HR_A2E        (1 << 6)  /* Hour alarm2 enable */
#  define PCF85263_RTC_ALARM_WDAY_A2E      (1 << 7)  /* Day-of-week alarm2 enable */

/* RTC timestamp1 (TSR1) */

#define PCF85263_RTC_TSR1_SECONDS          0x11      /* TSR1 seconds register */
#  define PCF85263_RTC_TSR1_SECONDS_MASK   (0x7f)    /* Bits 0-6: Seconds (0-59 BCD) */
#  define PCF85263_RTC_SECONDS_OS          (1 << 7)  /* Bit 7: Oscillator stop */

#define PCF85263_RTC_TSR1_MINUTES          0x12      /* TSR1 minutes register */
#  define PCF85263_RTC_TSR1_MINUTES_MASK   (0x7f)    /* Bits 0-6: Minutes (0-59 BCD) */

#define PCF85263_RTC_TSR1_HOURS            0x13      /* TSR1 hours register */
#  define PCF85263_RTC_TSR1_HOURS12_MASK   0x1f      /* Bits 0-4: Hours (1-12 BCD) */
#  define PCF85263_RTC_TSR1_AMPM           (1 << 5)  /* Bit 5: AM/PM */
#  define PCF85263_RTC_TSR1_HOURS24_MASK   0x3f      /* Bits 0-5: Hours (0-23  BCD) */

#define PCF85263_RTC_TSR1_DAYS             0x14      /* TSR1 days register */
#  define PCF85263_RTC_TSR1_DAYS_MASK      0x3f      /* Bits 0-5: Day of the month (1-31 BCD) */

#define PCF85263_RTC_TSR1_MONTHS           0x15      /* TSR1 month register */
#  define PCF85263_RTC_TSR1_MONTHS_MASK    0x1f      /* Bits 0-4: Month (1-12 BCD) */

#define PCF85263_RTC_TSR1_YEARS            0x16      /* TSR1 year register */
                                                     /* Bits 0-7: Year (0-99 BCD) */

/* RTC timestamp2 (TSR2) */

#define PCF85263_RTC_TSR2_SECONDS          0x17      /* TSR2 seconds register */
#  define PCF85263_RTC_TSR2_SECONDS_MASK   (0x7f)    /* Bits 0-6: Seconds (0-59 BCD) */

#define PCF85263_RTC_TSR2_MINUTES          0x18      /* TSR2 minutes register */
#  define PCF85263_RTC_TSR2_MINUTES_MASK   (0x7f)    /* Bits 0-6: Minutes (0-59 BCD) */

#define PCF85263_RTC_TSR2_HOURS            0x19      /* TSR2 hours register */
#  define PCF85263_RTC_TSR2_HOURS12_MASK   0x1f      /* Bits 0-4: Hours (1-12 BCD) */
#  define PCF85263_RTC_TSR2_AMPM           (1 << 5)  /* Bit 5: AM/PM */
#  define PCF85263_RTC_TSR2_HOURS24_MASK   0x3f      /* Bits 0-5: Hours (0-23  BCD) */

#define PCF85263_RTC_TSR2_DAYS             0x1a      /* TSR2 days register */
#  define PCF85263_RTC_TSR2_DAYS_MASK      0x3f      /* Bits 0-5: Day of the month (1-31 BCD) */

#define PCF85263_RTC_TSR2_MONTHS           0x1b      /* TSR2 month register */
#  define PCF85263_RTC_TSR2_MONTHS_MASK    0x1f      /* Bits 0-4: Month (1-12 BCD) */

#define PCF85263_RTC_TSR2_YEARS            0x1c      /* TSR2 year register */
                                                     /* Bits 0-7: Year (0-99 BCD) */

/* RTC timestamp3 (TSR3) */

#define PCF85263_RTC_TSR3_SECONDS          0x1d      /* TSR3 seconds register */
#  define PCF85263_RTC_TSR3_SECONDS_MASK   (0x7f)    /* Bits 0-6: Seconds (0-59 BCD) */

#define PCF85263_RTC_TSR3_MINUTES          0x1e      /* TSR3 minutes register */
#  define PCF85263_RTC_TSR3_MINUTES_MASK   (0x7f)    /* Bits 0-6: Minutes (0-59 BCD) */

#define PCF85263_RTC_TSR3_HOURS            0x1f      /* TSR3 hours register */
#  define PCF85263_RTC_TSR3_HOURS12_MASK   0x1f      /* Bits 0-4: Hours (1-12 BCD) */
#  define PCF85263_RTC_TSR3_AMPM           (1 << 5)  /* Bit 5: AM/PM */
#  define PCF85263_RTC_TSR3_HOURS24_MASK   0x3f      /* Bits 0-5: Hours (0-23  BCD) */

#define PCF85263_RTC_TSR3_DAYS             0x20      /* TSR3 days register */
#  define PCF85263_RTC_TSR3_DAYS_MASK      0x3f      /* Bits 0-5: Day of the month (1-31 BCD) */

#define PCF85263_RTC_TSR3_MONTHS           0x21      /* TSR3 month register */
#  define PCF85263_RTC_TSR3_MONTHS_MASK    0x1f      /* Bits 0-4: Month (1-12 BCD) */

#define PCF85263_RTC_TSR3_YEARS            0x22      /* TSR3 year register */
                                                     /* Bits 0-7: Year (0-99 BCD) */

/* RTC timestamp mode control */

#define PCF85263_RTC_TSR_MODE              0x23      /* Timestamp mode control register */
#  define PCF85263_RTC_TSR_TSR1M_SHIFT     (0)       /* Bit 0-1: Timestamp register 1 mode */
#  define PCF85263_RTC_TSR_TSR1M_MASK      (3 << PCF85263_RTC_TSR_TSR1M_SHIFT)
#    define PCF85263_RTC_TSR_TSR1M_NONE    (0 << PCF85263_RTC_TSR_TSR1M_SHIFT) /* No timestamp */
#    define PCF85263_RTC_TSR_TSR1M_FE      (1 << PCF85263_RTC_TSR_TSR1M_SHIFT) /* Record First TS pin Event */
#    define PCF85263_RTC_TSR_TSR1M_LE      (2 << PCF85263_RTC_TSR_TSR1M_SHIFT) /* Record Last TS pin Event */
#  define PCF85263_RTC_TSR_TSR2M_SHIFT     (2)       /* Bit 2-4: Timestamp register 2 mode */
#  define PCF85263_RTC_TSR_TSR2M_MASK      (7 << PCF85263_RTC_TSR_TSR2M_SHIFT)
#    define PCF85263_RTC_TSR_TSR2M_NONE    (0 << PCF85263_RTC_TSR_TSR2M_SHIFT) /* No timestamp */
#    define PCF85263_RTC_TSR_TSR2M_FB      (1 << PCF85263_RTC_TSR_TSR2M_SHIFT) /* Record First time switch to Battery event */
#    define PCF85263_RTC_TSR_TSR2M_LB      (2 << PCF85263_RTC_TSR_TSR2M_SHIFT) /* Record Last time switch to Battery event */
#    define PCF85263_RTC_TSR_TSR2M_LV      (3 << PCF85263_RTC_TSR_TSR2M_SHIFT) /* Record Last time switch to VDD event */
#    define PCF85263_RTC_TSR_TSR2M_FE      (4 << PCF85263_RTC_TSR_TSR2M_SHIFT) /* Record First TS pin Event */
#    define PCF85263_RTC_TSR_TSR2M_LE      (5 << PCF85263_RTC_TSR_TSR2M_SHIFT) /* Record Last TS pin Event */
#  define PCF85263_RTC_TSR_TSR3M_SHIFT     (6)       /* Bit 6-7: Timestamp register 3 mode */
#  define PCF85263_RTC_TSR_TSR3M_MASK      (3 << PCF85263_RTC_TSR_TSR3M_SHIFT)
#    define PCF85263_RTC_TSR_TSR3M_NONE    (0 << PCF85263_RTC_TSR_TSR3M_SHIFT) /* No timestamp */
#    define PCF85263_RTC_TSR_TSR3M_FB      (1 << PCF85263_RTC_TSR_TSR3M_SHIFT) /* Record First time switch to Battery event */
#    define PCF85263_RTC_TSR_TSR3M_LB      (2 << PCF85263_RTC_TSR_TSR3M_SHIFT) /* Record Last time switch to Battery event */
#    define PCF85263_RTC_TSR_TSR3M_LV      (3 << PCF85263_RTC_TSR_TSR3M_SHIFT) /* Record Last time switch to VDD event */

/* Stop-watch time registers */

#define PCF85263_STW_100TH_SECONDS         0x00      /* Stopwatch 100ths of seconds register */
                                                     /* Bits 0-7: 100ths of seconds register (0-99 BCD) */
#define PCF85263_STW_SECONDS               0x01      /* Stopwatch seconds register */
#  define PCF85263_STW_SECONDS_MASK        (0x7f)    /* Bits 0-6: Seconds (0-59 BCD) */
#  define PCF85263_STW_SECONDS_OS          (1 << 7)  /* Bit 7: Oscillator stop */

#define PCF85263_STW_MINUTES               0x02      /* Stopwatch minutes register */
#  define PCF85263_STW_MINUTES_MASK        (0x7f)    /* Bits 0-6: Minutes (0-59 BCD) */

#define PCF85263_STW_HOURS_XX_XX_00        0x03      /* Stopwatch hours register xx_xx_00 */
                                                     /* Bits 0-7: hours (0-99 BCD) */
#define PCF85263_STW_HOURS_XX_00_XX        0x04      /* Stopwatch hours register xx_00_xx */
                                                     /* Bits 0-7: hours (0-99 BCD) */
#define PCF85263_STW_HOURS_00_XX_XX        0x05      /* Stopwatch hours register 00_xx_xx */
                                                     /* Bits 0-7: hours (0-99 BCD) */

/* Stop-watch alarm1 */

#define PCF85263_STW_SECOND_ALM1           0x08      /* Stopwatch alarm1 seconds register */
#  define PCF85263_STW_SECOND_ALM1_MASK    (0x7f)    /* Bits 0-6: Seconds (0-59 BCD) */

#define PCF85263_STW_MINUTE_ALM1           0x09      /* Stopwatch alarm1 minutes register */
#  define PCF85263_STW_MINUTE_ALM1_MASK    (0x7f)    /* Bits 0-6: Minutes (0-59 BCD) */

#define PCF85263_STW_HR_XX_XX_00_ALM1      0x0a      /* Stopwatch alarm1 hours register xx_xx_00 */
                                                     /* Bits 0-7: hours (0-99 BCD) */
#define PCF85263_STW_HR_XX_00_XX_ALM1      0x0b      /* Stopwatch alarm1 hours register xx_00_xx */
                                                     /* Bits 0-7: hours (0-99 BCD) */
#define PCF85263_STW_HR_00_XX_XX_ALM1      0x0c      /* Stopwatch alarm1 hours register 00_xx_xx */
                                                     /* Bits 0-7: hours (0-99 BCD) */

/* Stop-watch alarm2 */

#define PCF85263_STW_MINUTE_ALM2           0x0d      /* Stopwatch alarm2 minutes register */
#  define PCF85263_STW_MINUTE_ALM2_MASK    (0x7f)    /* Bits 0-6: Minutes (0-59 BCD) */

#define PCF85263_STW_HR_XX_00_ALM2         0x0e      /* Stopwatch alarm2 hours register xx_00_xx */
                                                     /* Bits 0-7: hours (0-99 BCD) */
#define PCF85263_STW_HR_00_XX_ALM2         0x0f      /* Stopwatch alarm2 hours register 00_xx_xx */
                                                     /* Bits 0-7: hours (0-99 BCD) */

/* Stop-watch alarm enables */

#define PCF85263_STW_ALARM_ENABLES         0x10      /* Alarm enable control register */
#  define PCF85263_STW_SEC_A1E             (1 << 0)  /* Bit 0: Second alarm1 enable */
#  define PCF85263_STW_MIN_A1E             (1 << 1)  /* Bit 1: Minute alarm1 enable */
#  define PCF85263_STW_HR_XX_XX_00_A1E     (1 << 2)  /* Bit 2: Tens of hour alarm1 enable */
#  define PCF85263_STW_HR_XX_00_XX_A1E     (1 << 3)  /* Bit 3: Thousands of hours alarm1 enable */
#  define PCF85263_STW_HR_00_XX_XX_A1E     (1 << 4)  /* Bit 4: 100 thousands of hours alarm1 enable */
#  define PCF85263_STW_MIN_A2E             (1 << 5)  /* Bit 5: Minute alarm2 enable */
#  define PCF85263_STW_HR_XX_00_A2E        (1 << 6)  /* Bit 6: Tens of hours alarm2 enable */
#  define PCF85263_STW_HR_00_XX_A2E        (1 << 7)  /* Bit 7: Thousands of hours alarm2 enable */

/* Stop-watch timestamp1 (TSR1) */

#define PCF85263_STW_TSR1_SECONDS          0x11      /* TSR1 seconds register */
#  define PCF85263_STW_TSR1_SECONDS_MASK   (0x7f)    /* Bits 0-6: Seconds (0-59 BCD) */

#define PCF85263_STW_TSR1_MINUTES          0x12      /* TSR1 minutes register */
#  define PCF85263_STW_TSR1_MINUTES_MASK   (0x7f)    /* Bits 0-6: Minutes (0-59 BCD) */

#define PCF85263_STW_TSR1_HR_XX_XX_00      0x13      /* Stopwatch TSR1 hours register xx_xx_00 */
                                                     /* Bits 0-7: hours (0-99 BCD) */
#define PCF85263_STW_TSR1_HR_XX_00_XX      0x14      /* Stopwatch TSR1 hours register xx_00_xx */
                                                     /* Bits 0-7: hours (0-99 BCD) */
#define PCF85263_STW_TSR1_HR_00_XX_XX      0x15      /* Stopwatch TSR1 hours register 00_xx_xx */
                                                     /* Bits 0-7: hours (0-99 BCD) */

/* Stop-watch timestamp2 (TSR2) */

#define PCF85263_STW_TSR2_SECONDS          0x17      /* TSR2 seconds register */
#  define PCF85263_STW_TSR2_SECONDS_MASK   (0x7f)    /* Bits 0-6: Seconds (0-59 BCD) */

#define PCF85263_STW_TSR2_MINUTES          0x18      /* TSR2 minutes register */
#  define PCF85263_RTC_TSR2_MINUTES_MASK   (0x7f)    /* Bits 0-6: Minutes (0-59 BCD) */

#define PCF85263_STW_TSR2_HR_XX_XX_00      0x19      /* Stopwatch TSR2 hours register xx_xx_00 */
                                                     /* Bits 0-7: hours (0-99 BCD) */
#define PCF85263_STW_TSR2_HR_XX_00_XX      0x1a      /* Stopwatch TSR2 hours register xx_00_xx */
                                                     /* Bits 0-7: hours (0-99 BCD) */
#define PCF85263_STW_TSR2_HR_00_XX_XX      0x1b      /* Stopwatch TSR2 hours register 00_xx_xx */
                                                     /* Bits 0-7: hours (0-99 BCD) */

/* Stop-watch timestamp3 (TSR3) */

#define PCF85263_STW_TSR3_SECONDS          0x1d      /* TSR3 seconds register */
#  define PCF85263_STW_TSR3_SECONDS_MASK   (0x7f)    /* Bits 0-6: Seconds (0-59 BCD) */

#define PCF85263_STW_TSR3_MINUTES          0x1e      /* TSR3 minutes register */
#  define PCF85263_RTC_TSR3_MINUTES_MASK   (0x7f)    /* Bits 0-6: Minutes (0-59 BCD) */

#define PCF85263_STW_TSR3_HR_XX_XX_00      0x1f      /* Stopwatch TSR3 hours register xx_xx_00 */
                                                     /* Bits 0-7: hours (0-99 BCD) */
#define PCF85263_STW_TSR3_HR_XX_00_XX      0x20      /* Stopwatch TSR3 hours register xx_00_xx */
                                                     /* Bits 0-7: hours (0-99 BCD) */
#define PCF85263_STW_TSR3_HR_00_XX_XX      0x21      /* Stopwatch TSR3 hours register 00_xx_xx */
                                                     /* Bits 0-7: hours (0-99 BCD) */

/* Stop-watch timestamp mode control */

#define PCF85263_STW_TSR_MODE              0x23      /* Timestamp mode control register */
#  define PCF85263_STW_TSR_TSR1M_SHIFT     (0)       /* Bit 0-1: Timestamp register 1 mode */
#  define PCF85263_STW_TSR_TSR1M_MASK      (3 << PCF85263_STW_TSR_TSR1M_SHIFT)
#    define PCF85263_STW_TSR_TSR1M_NONE    (0 << PCF85263_STW_TSR_TSR1M_SHIFT) /* No timestamp */
#    define PCF85263_STW_TSR_TSR1M_FE      (1 << PCF85263_STW_TSR_TSR1M_SHIFT) /* Record First TS pin Event */
#    define PCF85263_STW_TSR_TSR1M_LE      (2 << PCF85263_STW_TSR_TSR1M_SHIFT) /* Record Last TS pin Event */
#  define PCF85263_STW_TSR_TSR2M_SHIFT     (2)       /* Bit 2-4: Timestamp register 2 mode */
#  define PCF85263_STW_TSR_TSR2M_MASK      (7 << PCF85263_STW_TSR_TSR2M_SHIFT)
#    define PCF85263_STW_TSR_TSR2M_NONE    (0 << PCF85263_STW_TSR_TSR2M_SHIFT) /* No timestamp */
#    define PCF85263_STW_TSR_TSR2M_FB      (1 << PCF85263_STW_TSR_TSR2M_SHIFT) /* Record First time switch to Battery event */
#    define PCF85263_STW_TSR_TSR2M_LB      (2 << PCF85263_STW_TSR_TSR2M_SHIFT) /* Record Last time switch to Battery event */
#    define PCF85263_STW_TSR_TSR2M_LV      (3 << PCF85263_STW_TSR_TSR2M_SHIFT) /* Record Last time switch to VDD event */
#    define PCF85263_STW_TSR_TSR2M_FE      (4 << PCF85263_STW_TSR_TSR2M_SHIFT) /* Record First TS pin Event */
#    define PCF85263_STW_TSR_TSR2M_LE      (5 << PCF85263_STW_TSR_TSR2M_SHIFT) /* Record Last TS pin Event */
#  define PCF85263_STW_TSR_TSR3M_SHIFT     (6)       /* Bit 6-7: Timestamp register 3 mode */
#  define PCF85263_STW_TSR_TSR3M_MASK      (3 << PCF85263_STW_TSR_TSR3M_SHIFT)
#    define PCF85263_STW_TSR_TSR3M_NONE    (0 << PCF85263_STW_TSR_TSR3M_SHIFT) /* No timestamp */
#    define PCF85263_STW_TSR_TSR3M_FB      (1 << PCF85263_STW_TSR_TSR3M_SHIFT) /* Record First time switch to Battery event */
#    define PCF85263_STW_TSR_TSR3M_LB      (2 << PCF85263_STW_TSR_TSR3M_SHIFT) /* Record Last time switch to Battery event */
#    define PCF85263_STW_TSR_TSR3M_LV      (3 << PCF85263_STW_TSR_TSR3M_SHIFT) /* Record Last time switch to VDD event */

/* Offset register */

#define PCF85263_CTL_OFFSET                0x24      /* Offset regsiter */
                                                     /* Bits 0-7: Offset value */

/* Control registers */

#define PCF85263_CTL_OSCILLATOR            0x25      /* Oscillator control register */
#  define PCF85263_CTL_OSC_CL_SHIFT        (0)       /* Bits 0-1: Quartz oscillator load capacitance */
#  define PCF85263_CTL_OSC_CL_MASK         (3 << PCF85263_CTL_OSC_CL_SHIFT)
#    define PCF85263_CTL_OSC_CL_7PF        (0 << PCF85263_CTL_OSC_CL_SHIFT) /* 7.0 pF */
#    define PCF85263_CTL_OSC_CL_6PF        (1 << PCF85263_CTL_OSC_CL_SHIFT) /* 6.0 pF */
#    define PCF85263_CTL_OSC_CL_12p5PF     (2 << PCF85263_CTL_OSC_CL_SHIFT) /* 12.5 pF */
#  define PCF85263_CTL_OSC_OSCD_SHIFT      (2)       /* Bits 1-2: Oscillator driver bits */
#  define PCF85263_CTL_OSC_OSCD_MASK       (3 << PCF85263_CTL_OSC_OSCD_SHIFT)
#    define PCF85263_CTL_OSC_OSCD_NORMAL   (0 << PCF85263_CTL_OSC_OSCD_SHIFT) /* Normal drive; RS(max): 100 kohm */
#    define PCF85263_CTL_OSC_OSCD_LOW      (1 << PCF85263_CTL_OSC_OSCD_SHIFT) /* Low drive; RS(max): 60 kohm; reduced IDD */
#    define PCF85263_CTL_OSC_OSCD_HIGH     (2 << PCF85263_CTL_OSC_OSCD_SHIFT) /* High drive; RS(max): 500 kohm; increased IDD */
#  define PCF85263_CTL_OSC_LOWJ            (1 << 4)  /* Bit 4:  Low jitter mode */
#  define PCF85263_CTL_OSC_12_24           (1 << 5)  /* Bit 5:  12-/24-hour mode */
#  define PCF85263_CTL_OSC_OFFM            (1 << 6)  /* Bit 6:  Offset calibration mode */
#  define PCF85263_CTL_OSC_CLKIV           (1 << 7)  /* Bit 7:  Output clock inversion */

#define PCF85263_CTL_BATTERY_SWITCH        0x26      /* Battery switch control register */
#  define PCF85263_CTL_BATTERY_BSTH        (1 << 0)  /* Bit 0: Threshold voltage control */
#  define PCF85263_CTL_BATTERY_BSM_SHIFT   (1)       /* Bits 1-2: Battery switch mode bits */
#  define PCF85263_CTL_BATTERY_BSM_MASK    (3 << PCF85263_CTL_BATTERY_BSM_SHIFT)
#    define PCF85263_CTL_BATTERY_BSM_VTH   (0 << PCF85263_CTL_BATTERY_BSM_SHIFT) /* Switching at the Vth level */
#    define PCF85263_CTL_BATTERY_BSM_VBAT  (1 << PCF85263_CTL_BATTERY_BSM_SHIFT) /* Switching at the VBAT level */
#    define PCF85263_CTL_BATTERY_BSM_MAX   (2 << PCF85263_CTL_BATTERY_BSM_SHIFT) /* Switching at the higher level of Vth or VBAT */
#    define PCF85263_CTL_BATTERY_BSM_MIN   (3 << PCF85263_CTL_BATTERY_BSM_SHIFT) /* Switching at the lower level of Vth or VBAT */
#  define PCF85263_CTL_BATTERY_BSRR        (1 << 3)  /* Bit 3:  Battery switch refresh rate */
#  define PCF85263_CTL_BATTERY_BSOFF       (1 << 4)  /* Bit 4:  Battery switch on/off */

#define PCF85263_CTL_PIN_IO                0x27      /* Pin input/output control register */
#  define PCF85263_CTL_INTAPM_SHIFT        (0)       /* Bits 0-1: INTA pin mode */
#  define PCF85263_CTL_INTAPM_MASK         (3 << PCF85263_CTL_INTAPM_SHIFT)
#    define PCF85263_CTL_INTAPM_CLK        (0 << PCF85263_CTL_INTAPM_SHIFT) /* CLK output mode */
#    define PCF85263_CTL_INTAPM_BAT        (1 << PCF85263_CTL_INTAPM_SHIFT) /* Battery mode indication */
#    define PCF85263_CTL_INTAPM_INTA       (2 << PCF85263_CTL_INTAPM_SHIFT) /* INTA output */
#    define PCF85263_CTL_INTAPM_HIZ        (3 << PCF85263_CTL_INTAPM_SHIFT) /* Hi-Z */
#  define PCF85263_CTL_TSPM_SHIFT          (2)       /* Bits 2-3: TS pin I/O control */
#  define PCF85263_CTL_TSPM_MASK           (3 << PCF85263_CTL_TSPM_SHIFT)
#    define PCF85263_CTL_TSPM_DISABLED     (0 << PCF85263_CTL_TSPM_SHIFT) /* Disabled; input can be left floating */
#    define PCF85263_CTL_TSPM_INTB         (1 << PCF85263_CTL_TSPM_SHIFT) /* INTB output; push-pull */
#    define PCF85263_CTL_TSPM_CLK          (2 << PCF85263_CTL_TSPM_SHIFT) /* CLK output; push-pull */
#    define PCF85263_CTL_TSPM_INPUT        (3 << PCF85263_CTL_TSPM_SHIFT) /* Input mode */
#  define PCF85263_CTL_TSIM                (1 << 4)  /* Bit 4:  TS pin input mode */
#  define PCF85263_CTL_TSL                 (1 << 5)  /* Bit 5:  TS pin input sense */
#  define PCF85263_CTL_TSPULL              (1 << 6)  /* Bit 6:  TS pin pull-up resistor value */
#  define PCF85263_CTL_CLKPM               (1 << 7)  /* Bit 7:  CLK pin mode */

#define PCF85263_CTL_FUNCTION              0x28      /* Function control register */
#  define PCF85263_CTL_FUNC_COF_SHIFT      (0)       /* Bits 0-2: Clock output frequency */
#  define PCF85263_CTL_FUNC_COF_MASK       (7 << PCF85263_CTL_FUNC_COF_SHIFT) /* CLK pin    TS pin     INTA pin */
#    define PCF85263_CTL_FUNC_COF_32KHZ    (0 << PCF85263_CTL_FUNC_COF_SHIFT) /* 32768      32768      32768    */
#    define PCF85263_CTL_FUNC_COF_16KHZ    (1 << PCF85263_CTL_FUNC_COF_SHIFT) /* 16384      16384      16384    */
#    define PCF85263_CTL_FUNC_COF_8KHZ     (2 << PCF85263_CTL_FUNC_COF_SHIFT) /* 8192       8192       8192     */
#    define PCF85263_CTL_FUNC_COF_4KHZ     (3 << PCF85263_CTL_FUNC_COF_SHIFT) /* 4096       4096       4096     */
#    define PCF85263_CTL_FUNC_COF_2KHZ     (4 << PCF85263_CTL_FUNC_COF_SHIFT) /* 2048       2048       2048     */
#    define PCF85263_CTL_FUNC_COF_1KHZ     (5 << PCF85263_CTL_FUNC_COF_SHIFT) /* 1024       1024       1024     */
#    define PCF85263_CTL_FUNC_COF_1HZ      (6 << PCF85263_CTL_FUNC_COF_SHIFT) /* 1          1          1        */
#    define PCF85263_CTL_FUNC_COF_LOW      (7 << PCF85263_CTL_FUNC_COF_SHIFT) /* static LOW static LOW Hi-Z     */
#  define PCF85263_CTL_FUNC_STOPM          (1 << 3)  /* Bit 3:  STOP mode */
#  define PCF85263_CTL_FUNC_RTCM           (1 << 4)  /* Bit 4:  RTC mode */
#  define PCF85263_CTL_FUNC_PI_SHIFT       (5)       /* Bits 5-6: Periodic interrupt */
#  define PCF85263_CTL_FUNC_PI_MASK        (3 << PCF85263_CTL_FUNC_PI_SHIFT)
#    define PCF85263_CTL_FUNC_PI_NONE      (0 << PCF85263_CTL_FUNC_PI_SHIFT) /* No periodic interrupt */
#    define PCF85263_CTL_FUNC_PI_SEC       (1 << PCF85263_CTL_FUNC_PI_SHIFT) /* Once per second */
#    define PCF85263_CTL_FUNC_PI_MIN       (2 << PCF85263_CTL_FUNC_PI_SHIFT) /* Once per minute */
#    define PCF85263_CTL_FUNC_PI_HOUR      (3 << PCF85263_CTL_FUNC_PI_SHIFT) /* Once per hour */
#  define PCF85263_CTL_FUNC_100TH          (1 << 7)  /* Bit 7:  100th seconds mode */

#define PCF85263_CTL_INTA_ENABLE           0x29      /* Interrupt A control bits */
#  define PCF85263_CTL_INTA_WDIEA          (1 << 0)  /* Bit 0:  Watchdog interrupt enable */
#  define PCF85263_CTL_INTA_BSIEA          (1 << 1)  /* Bit 1:  Battery switch interrupt enable */
#  define PCF85263_CTL_INTA_TSRIEA         (1 << 2)  /* Bit 2:  Timestamp register interrupt enable */
#  define PCF85263_CTL_INTA_A2IEA          (1 << 3)  /* Bit 3:  Alarm2 interrupt enable */
#  define PCF85263_CTL_INTA_A1IEA          (1 << 4)  /* Bit 4:  Alarm1 interrupt enable */
#  define PCF85263_CTL_INTA_OIEA           (1 << 5)  /* Bit 5:  Offset correction interrupt enable */
#  define PCF85263_CTL_INTA_PIEA           (1 << 6)  /* Bit 6:  Periodic interrupt enable */
#  define PCF85263_CTL_INTA_ILPA           (1 << 7)  /* Bit 7:  Interrupt generates a pulse */

#define PCF85263_CTL_INTB_ENABLE           0x2a      /* Interrupt B control bits */
#  define PCF85263_CTL_INTB_WDIEB          (1 << 0)  /* Bit 0:  Watchdog interrupt enable */
#  define PCF85263_CTL_INTB_BSIEB          (1 << 1)  /* Bit 1:  Battery switch interrupt enable */
#  define PCF85263_CTL_INTB_TSRIEB         (1 << 2)  /* Bit 2:  Timestamp register interrupt enable */
#  define PCF85263_CTL_INTB_A2IEB          (1 << 3)  /* Bit 3:  Alarm2 interrupt enable */
#  define PCF85263_CTL_INTB_A1IEB          (1 << 4)  /* Bit 4:  Alarm1 interrupt enable */
#  define PCF85263_CTL_INTB_OIEB           (1 << 5)  /* Bit 5:  Offset correction interrupt enable */
#  define PCF85263_CTL_INTB_PIEB           (1 << 6)  /* Bit 6:  Periodic interrupt enable */
#  define PCF85263_CTL_INTB_ILPB           (1 << 7)  /* Bit 7:  Interrupt generates a pulse */

#define PCF85263_CTL_FLAGS                 0x2b      /* Flag status register */
#  define PCF85263_CTL_FLAGS_TSR1F         (1 << 0)  /* Bit 0:  Timestamp register 1 event flag */
#  define PCF85263_CTL_FLAGS_TSR2F         (1 << 1)  /* Bit 1:  Timestamp register 2 event flag */
#  define PCF85263_CTL_FLAGS_TSR3F         (1 << 2)  /* Bit 2:  Timestamp register 3 event flag */
#  define PCF85263_CTL_FLAGS_BSF           (1 << 3)  /* Bit 3:  Battery switch flag */
#  define PCF85263_CTL_FLAGS_WDF           (1 << 4)  /* Bit 4:  Watchdog flag */
#  define PCF85263_CTL_FLAGS_A1F           (1 << 5)  /* Bit 5:  Alarm1 flag */
#  define PCF85263_CTL_FLAGS_A2F           (1 << 6)  /* Bit 6:  Alarm2 flag */
#  define PCF85263_CTL_FLAGS_PIF           (1 << 7)  /* Bit 7:  Periodic interrupt flag */

/* RAM byte */

#define PCF85263_CTL_RAM_BYTE              0x2c      /* RAM byte register */
                                                     /* Bits 0-7: RAM data */

/* Watchdog registers */

#define PCF85263_CTL_WATCHDOG              0x2d      /* Watchdog control and status register */
#  define PCF85263_CTL_WDS_SHIFT           (0)       /* Bits 0-1: Watchdog step size (source clock) */
#  define PCF85263_CTL_WDS_MASK            (3 << PCF85263_CTL_WDS_SHIFT)
#    define PCF85263_CTL_WDS_4SEC          (0 << PCF85263_CTL_WDS_SHIFT) /* 4 seconds (0.25 Hz) */
#    define PCF85263_CTL_WDS_1SEC          (1 << PCF85263_CTL_WDS_SHIFT) /* 1 second (1 Hz) */
#    define PCF85263_CTL_WDS_250MSEC       (2 << PCF85263_CTL_WDS_SHIFT) /* 1⁄4 second (4 Hz) */
#    define PCF85263_CTL_WDS_67MSEC        (3 << PCF85263_CTL_WDS_SHIFT) /* 1⁄16 second (16 Hz) */
#  define PCF85263_CTL_WDR_SHIFT           (2)       /* Bits 2-6: Watchdog register bits */
#  define PCF85263_CTL_WDR_MASK            (31 << PCF85263_CTL_WDR_SHIFT)
#    define PCF85263_CTL_WDR(n)            ((uint9_t)(n) << PCF85263_CTL_WDR_SHIFT)
#  define PCF85263_CTL_WDM                 (1 << 7)  /* Bit 7:  Watchdog mode */

/* Stop */

#define PCF85263_CTL_STOP_ENABLE           0x2e      /* Stop enable register */
#  define PCF85263_CTL_STOP                (1 << 0)  /* Bit 0:  Stop bit */

/* Reset */

#define PCF85263_CTL_RESETS                0x2f      /* Software reset control register */
#  define PCF85263_CTL_CTS                 (1 << 0)  /* Bit 0:  Clear timestamp */
#  define PCF85263_CTL_SR                  (1 << 3)  /* Bit 3:  Software reset */
#  define PCF85263_CTL_CPR                 (1 << 7)  /* Bit 7:  Clear prescaler */
#  define PCF85263_CTL_RESETS_BITS         0x24      /* Fixed register bits */

#endif /* __DRIVERS_TIMERS_PCF85263_H */
