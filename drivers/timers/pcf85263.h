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

#define PCF85263_STW_ALARM_ENABLES    0x10      /* 
HR_00_XX
_A2E
HR_XX_00
_A2E
MIN_A2E HR_00_XX
_XX_A1E
HR_XX_00
_XX_A1E
HR_XX_XX
_00_A1E
MIN_A1E SEC_A1E

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

#define PCF85263_STW_TSR_MODE         0x23      /* 
TSR3M[1:0] - TSR2M[2:0] TSR1M[1:0]

/* Offset register */

#define PCF85263_STW_OFFSET           0x24      /* 
OFFSET[7:0]

/* Control registers */

#define PCF85263_STW_OSCILLATOR       0x25      /* 
CLKIV OFFM 12_24 LOWJ OSCD[1:0] CL[1:0]
#define PCF85263_STW_BATTERY_SWITCH   0x26      /* 
BSOFF BSRR BSM[1:0] BSTH
#define PCF85263_STW_PIN_IO           0x27      /* 
CLKPM TSPULL TSL TSIM TSPM[1:0] INTAPM[1:0]
#define PCF85263_STW_FUNCTION         0x28      /* 
100TH PI[1:0] RTCM STOPM COF[2:0]
#define PCF85263_STW_INTA_ENABLE      0x29      /* 
ILPA PIEA OIEA A1IEA A2IEA TSRIEA BSIEA WDIEA
#define PCF85263_STW_INTB_ENABLE      0x2a      /* 
ILPB PIEB OIEB A1IEB A2IEB TSRIEB BSIEB WDIEB
#define PCF85263_STW_FLAGS            0x2b      /* 
PIF A2F A1F WDF BSF TSR3F TSR2F TSR1F

/* RAM byte */

#define PCF85263_STW_RAM_BYTE         0x2c      /* 
                                                /* Bits 0-7: RAM data */

/* WatchDog registers */

#define PCF85263_STW_WATCHDOG         0x2d      /* 
WDM WDR[4:0] WDS[1:0]

/* Stop */

#define PCF85263_STW_STOP_ENABLE      0x2e      /* 
STOP

/* Reset */

#define PCF85263_STW_RESETS           0x2f      /* 
CPR 0 1 0 SR 1 0 CTS

#endif /* __DRIVERS_TIMERS_PCF85263_H */
