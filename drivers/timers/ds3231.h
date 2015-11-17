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

#define DS3231_TIME_SECR               0x00      /* Seconds register */
#  define DS3231_TIME_SEC_SHIFT        0         /* Bits 0-3: Seconds, range 0-9 */
#  define DS3231_TIME_SEC_MASK         (15 << DS3231_TIME_SEC_SHIFT)
#    define DS3231_TIME_SEC(n)         ((uint8_t)(n) << DS3231_TIME_SEC_SHIFT)
#  define DS3231_TIME_10SEC_SHIFT      4         /* Bits 4-6: 10 seconds, range 0-5 */
#  define DS3231_TIME_10SEC_MASK       (7 << DS3231_TIME_10SEC_SHIFT)
#    define DS3231_TIME_10SEC(n)       ((uint8_t)(n) << DS3231_TIME_10SEC_SHIFT)
#  define DS3231_TIME_SEC_BCDMASK      (DS3231_TIME_SEC_MASK | DS3231_TIME_10SEC_MASK)

#define DS3231_TIME_MINR               0x01      /* Minutes register */
#  define DS3231_TIME_MIN_SHIFT        0         /* Bits 0-3: Minutes, range 0-9 */
#  define DS3231_TIME_MIN_MASK         (15 << DS3231_TIME_MIN_SHIFT)
#    define DS3231_TIME_MIN(n)         ((uint8_t)(n) << DS3231_TIME_MIN_SHIFT)
#  define DS3231_TIME_10MIN_SHIFT      4         /* Bits 4-6: 10 minutes, range 0-5 */
#  define DS3231_TIME_10MIN_MASK       (7 << DS3231_TIME_10MIN_SHIFT)
#    define DS3231_TIME_10MIN(n)       ((uint8_t)(n) << DS3231_TIME_10MIN_SHIFT)
#  define DS3231_TIME_MIN_BCDMASK      (DS3231_TIME_MIN_MASK | DS3231_TIME_10MIN_MASK)

#define DS3231_TIME_HOURR              0x02      /* Hours register */
#  define DS3231_TIME_HOUR_SHIFT       0         /* Bits 0-3: Hours, range 0-9 */
#  define DS3231_TIME_HOUR_MASK        (15 << DS3231_TIME_HOUR_SHIFT)
#    define DS3231_TIME_HOUR(n)        ((uint8_t)(n) << DS3231_TIME_HOUR_SHIFT)
#  define DS3231_TIME_10HOUR12_SHIFT   4         /* Bit 4: 10 hours, range 0-1 */
#  define DS3231_TIME_10HOUR12_MASK    (1 << DS3231_TIME_10HOUR12_SHIFT)
#    define DS3231_TIME_10HOUR12(n)    ((uint8_t)(n) << DS3231_TIME_10HOUR12_SHIFT)
#  define DS3231_TIME_10HOUR24_SHIFT   4         /* Bits 4-5: 10 hours, range 0-2 */
#  define DS3231_TIME_10HOUR24_MASK    (3 << DS3231_TIME_10HOUR24_SHIFT)
#    define DS3231_TIME_10HOUR24(n)    ((uint8_t)(n) << DS3231_TIME_10HOUR24_SHIFT)
#  define DS3231_TIME_HOUR12_BCDMASK   (DS3231_TIME_HOUR_MASK | DS3231_TIME_10HOUR12_MASK)
#  define DS3231_TIME_HOUR24_BCDMASK   (DS3231_TIME_HOUR_MASK | DS3231_TIME_10HOUR24_MASK)
#  define DS3231_TIME_AMPM_SHIFT       5         /* Bit 5: AM/PM Indication */
#  define DS3231_TIME_AMPM_MASK        (1 << DS3231_TIME_AMPM_SHIFT)
#    define DS3231_TIME_AM             ((uint8_t)(0) << DS3231_TIME_AMPM_SHIFT)
#    define DS3231_TIME_PM             ((uint8_t)(1) << DS3231_TIME_AMPM_SHIFT)
#  define DS3231_TIME_1224_SHIFT       6         /* Bit 6: 12/24 Indication */
#  define DS3231_TIME_1224_MASK        (1 << DS3231_TIME_1224_SHIFT)
#    define DS3231_TIME_24             ((uint8_t)(0) << DS3231_TIME_1224_SHIFT)
#    define DS3231_TIME_12             ((uint8_t)(1) << DS3231_TIME_1224_SHIFT)

#define DS3231_TIME_DAYR               0x03      /* Day of the week register */
#  define DS3231_TIME_DAY_SHIFT        0         /* Bits 0-3: Day of the week, range 1-7 */
#  define DS3231_TIME_DAY_MASK         (7 << DS3231_TIME_DAY_SHIFT)
#    define DS3231_TIME_DAY(n)         ((uint8_t)(n) << DS3231_TIME_DAY_SHIFT)

#define DS3231_TIME_DATER              0x04      /* Date register */
#  define DS3231_TIME_DATE_SHIFT       0         /* Bits 0-3: Days, range 0-9 */
#  define DS3231_TIME_DATE_MASK        (15 << DS3231_TIME_DATE_SHIFT)
#    define DS3231_TIME_DATE(n)        ((uint8_t)(n) << DS3231_TIME_DATE_SHIFT)
#  define DS3231_TIME_10DATE_SHIFT     4         /* Bits 4-5: 10 days, range 0-5 */
#  define DS3231_TIME_10DATE_MASK      (3 << DS3231_TIME_10DATE_SHIFT)
#    define DS3231_TIME_10DATE(n)      ((uint8_t)(n) << DS3231_TIME_10DATE_SHIFT)
#  define DS3231_TIME_DATE_BCDMASK     (DS3231_TIME_DATE_MASK | DS3231_TIME_10DATE_MASK)

#define DS3231_TIME_MONTHR             0x05      /* Month register */
#  define DS3231_TIME_MONTH_SHIFT      0         /* Bits 0-3: Month, range 0-9 */
#  define DS3231_TIME_MONTH_MASK       (15 << DS3231_TIME_MONTH_SHIFT)
#    define DS3231_TIME_MONTH(n)       ((uint8_t)(n) << DS3231_TIME_MONTH_SHIFT)
#  define DS3231_TIME_10MONTH_SHIFT    4         /* Bit 4: 10 month, range 0-1 */
#  define DS3231_TIME_10MONTH_MASK     (1 << DS3231_TIME_10MONTH_SHIFT)
#    define DS3231_TIME_10MONTH(n)     ((uint8_t)(n) << DS3231_TIME_10MONTH_SHIFT)
#  define DS3231_TIME_MONTH_BCDMASK    (DS3231_TIME_MONTH_MASK | DS3231_TIME_10MONTH_MASK)
#  define DS3231_TIME_CENTURY_SHIFT    7         /* Bit 7: AM/PM Indication */
#  define DS3231_TIME_CENTURY_MASK     (1 << DS3231_TIME_CENTURY_SHIFT)
#    define DS3231_TIME_1900           ((uint8_t)(0) << DS3231_TIME_CENTURY_SHIFT)
#    define DS3231_TIME_2000           ((uint8_t)(1) << DS3231_TIME_CENTURY_SHIFT)

#define DS3231_TIME_YEARR              0x06      /* Date register */
#  define DS3231_TIME_YEAR_SHIFT       0         /* Bits 0-3: Year, range 0-9 */
#  define DS3231_TIME_YEAR_MASK        (15 << DS3231_TIME_YEAR_SHIFT)
#    define DS3231_TIME_YEAR(n)        ((uint8_t)(n) << DS3231_TIME_YEAR_SHIFT)
#  define DS3231_TIME_10YEAR_SHIFT     4         /* Bits 4-7: 10 year, range 0-9 */
#  define DS3231_TIME_10YEAR_MASK      (15 << DS3231_TIME_10YEAR_SHIFT)
#    define DS3231_TIME_10YEAR(n)      ((uint8_t)(n) << DS3231_TIME_10YEAR_SHIFT)
#  define DS3231_TIME_YEAR_BCDMASK     (DS3231_TIME_YEAR_MASK | DS3231_TIME_10YEAR_MASK)

#define DS3231_ALARM1_SECR             0x07      /* Alarm1 seconds register */
#  define DS3231_ALARM1_SEC_SHIFT      0         /* Bits 0-3: Seconds, range 0-9 */
#  define DS3231_ALARM1_SEC_MASK       (15 << DS3231_ALARM1_SEC_SHIFT)
#    define DS3231_ALARM1_SEC(n)       ((uint8_t)(n) << DS3231_ALARM1_SEC_SHIFT)
#  define DS3231_ALARM1_10SEC_SHIFT    4         /* Bits 4-6: 10 seconds, range 0-5 */
#  define DS3231_ALARM1_10SEC_MASK     (7 << DS3231_ALARM1_10SEC_SHIFT)
#    define DS3231_ALARM1_10SEC(n)     ((uint8_t)(n) << DS3231_ALARM1_10SEC_SHIFT)
#  define DS3231_ALARM1_SEC_BCDMASK    (DS3231_ALARM1_SEC_MASK | DS3231_ALARM1_10SEC_MASK)
#  define DS3231_ALARM1_A1M1_SHIFT     7         /* Bits 7: A1M1 mask */
#  define DS3231_ALARM1_A1M1_MASK      (1 << DS3231_ALARM1_A1M1_SHIFT)
#    define DS3231_ALARM1_A1M1(n)      ((uint8_t)(n) << DS3231_ALARM1_A1M1_SHIFT)

#define DS3231_ALARM1_MINR             0x08      /* Alarm1 minutes register */
#  define DS3231_ALARM1_MIN_SHIFT      0         /* Bits 0-3: Minutes, range 0-9 */
#  define DS3231_ALARM1_MIN_MASK       (15 << DS3231_ALARM1_MIN_SHIFT)
#    define DS3231_ALARM1_MIN(n)       ((uint8_t)(n) << DS3231_ALARM1_MIN_SHIFT)
#  define DS3231_ALARM1_10MIN_SHIFT    4         /* Bits 4-6: 10 minutes, range 0-5 */
#  define DS3231_ALARM1_10MIN_MASK     (7 << DS3231_ALARM1_10MIN_SHIFT)
#    define DS3231_ALARM1_10MIN(n)     ((uint8_t)(n) << DS3231_ALARM1_10MIN_SHIFT)
#  define DS3231_ALARM1_MIN_BCDMASK    (DS3231_ALARM1_MIN_MASK | DS3231_ALARM1_10MIN_MASK)
#  define DS3231_ALARM1_A1M2_SHIFT     7         /* Bits 7: A1M2 mask */
#  define DS3231_ALARM1_A1M2_MASK      (1 << DS3231_ALARM1_A1M2_SHIFT)
#    define DS3231_ALARM1_A1M2(n)      ((uint8_t)(n) << DS3231_ALARM1_A1M2_SHIFT)

#define DS3231_ALARM1_HOURR            0x09      /* Alarm1 hours register */
#  define DS3231_ALARM1_HOUR_SHIFT     0         /* Bits 0-3: Hours, range 0-9 */
#  define DS3231_ALARM1_HOUR_MASK      (15 << DS3231_ALARM1_HOUR_SHIFT)
#    define DS3231_ALARM1_HOUR(n)      ((uint8_t)(n) << DS3231_ALARM1_HOUR_SHIFT)
#  define DS3231_ALARM1_10HOUR12_SHIFT 4         /* Bit 4: 10 hours, range 0-1 */
#  define DS3231_ALARM1_10HOUR12_MASK  (1 << DS3231_ALARM1_10HOUR12_SHIFT)
#    define DS3231_ALARM1_10HOUR12(n)  ((uint8_t)(n) << DS3231_ALARM1_10HOUR12_SHIFT)
#  define DS3231_ALARM1_10HOUR24_SHIFT 4         /* Bits 4-5: 10 hours, range 0-2 */
#  define DS3231_ALARM1_10HOUR24_MASK  (3 << DS3231_ALARM1_10HOUR24_SHIFT)
#    define DS3231_ALARM1_10HOUR24(n)  ((uint8_t)(n) << DS3231_ALARM1_10HOUR24_SHIFT)
#  define DS3231_ALARM1_HOUR12_BCDMASK (DS3231_ALARM1_HOUR_MASK | DS3231_ALARM1_10HOUR12_MASK)
#  define DS3231_ALARM1_HOUR24_BCDMASK (DS3231_ALARM1_HOUR_MASK | DS3231_ALARM1_10HOUR24_MASK)
#  define DS3231_ALARM1_AMPM_SHIFT     5         /* Bit 5: AM/PM Indication */
#  define DS3231_ALARM1_AMPM_MASK      (1 << DS3231_ALARM1_AMPM_SHIFT)
#    define DS3231_ALARM1_AM           ((uint8_t)(0) << DS3231_ALARM1_AMPM_SHIFT)
#    define DS3231_ALARM1_PM           ((uint8_t)(1) << DS3231_ALARM1_AMPM_SHIFT)
#  define DS3231_ALARM1_1224_SHIFT     6         /* Bit 6: 12/24 Indication */
#  define DS3231_ALARM1_1224_MASK      (1 << DS3231_ALARM1_1224_SHIFT)
#    define DS3231_ALARM1_12           ((uint8_t)(0) << DS3231_ALARM1_1224_SHIFT)
#    define DS3231_ALARM1_24           ((uint8_t)(1) << DS3231_ALARM1_1224_SHIFT)
#  define DS3231_ALARM1_A1M3_SHIFT     7         /* Bits 7: A1M3 mask */
#  define DS3231_ALARM1_A1M3_MASK      (1 << DS3231_ALARM1_A1M3_SHIFT)
#    define DS3231_ALARM1_A1M3(n)      ((uint8_t)(n) << DS3231_ALARM1_A1M3_SHIFT)

#define DS3231_ALARM1_DAYDATER         0x0a      /* Alarm1 date / day of the week register */
#  define DS3231_ALARM1_DAY_SHIFT      0         /* Bits 0-3: Day of the week, range 1-7 */
#  define DS3231_ALARM1_DAY_MASK       (7 << DS3231_ALARM1_DAY_SHIFT)
#    define DS3231_ALARM1_DAY(n)       ((uint8_t)(n) << DS3231_ALARM1_DAY_SHIFT)
#  define DS3231_ALARM1_DATE_SHIFT     0         /* Bits 0-3: Days, range 0-9 */
#  define DS3231_ALARM1_DATE_MASK      (15 << DS3231_ALARM1_DATE_SHIFT)
#    define DS3231_ALARM1_DATE(n)      ((uint8_t)(n) << DS3231_ALARM1_DATE_SHIFT)
#  define DS3231_ALARM1_10DATE_SHIFT   4         /* Bits 4-5: 10 days, range 0-5 */
#  define DS3231_ALARM1_10DATE_MASK    (3 << DS3231_ALARM1_10DATE_SHIFT)
#    define DS3231_ALARM1_10DATE(n)    ((uint8_t)(n) << DS3231_ALARM1_10DATE_SHIFT)
#  define DS3231_ALARM1_DATE_BCDMASK   (DS3231_ALARM1_DATE_MASK | DS3231_ALARM1_10DATE_MASK)
#  define DS3231_ALARM1_DYDT_SHIFT     6         /* Bits 6: DY/DT */
#  define DS3231_ALARM1_DYDT_MASK      (1 << DS3231_ALARM1_DYDT_SHIFT)
#    define DS3231_ALARM1_DYDT_DATE    ((uint8_t)(0) << DS3231_ALARM1_DYDT_SHIFT)
#    define DS3231_ALARM1_DYDT_DAY     ((uint8_t)(1) << DS3231_ALARM1_DYDT_SHIFT)
#  define DS3231_ALARM1_A1M4_SHIFT     7         /* Bits 7: A1M4 mask */
#  define DS3231_ALARM1_A1M4_MASK      (1 << DS3231_ALARM1_A1M4_SHIFT)
#    define DS3231_ALARM1_A1M4(n)      ((uint8_t)(n) << DS3231_ALARM1_A1M4_SHIFT)

#define DS3231_ALARM2_MINR             0x0b      /* Alarm2 minutes register */
#  define DS3231_ALARM2_MIN_SHIFT      0         /* Bits 0-3: Minutes, range 0-9 */
#  define DS3231_ALARM2_MIN_MASK       (15 << DS3231_ALARM2_MIN_SHIFT)
#    define DS3231_ALARM2_MIN(n)       ((uint8_t)(n) << DS3231_ALARM2_MIN_SHIFT)
#  define DS3231_ALARM2_10MIN_SHIFT    4         /* Bits 4-6: 10 minutes, range 0-5 */
#  define DS3231_ALARM2_10MIN_MASK     (7 << DS3231_ALARM2_10MIN_SHIFT)
#    define DS3231_ALARM2_10MIN(n)     ((uint8_t)(n) << DS3231_ALARM2_10MIN_SHIFT)
#  define DS3231_ALARM2_MIN_BCDMASK    (DS3231_ALARM2_MIN_MASK | DS3231_ALARM2_10MIN_MASK)
#  define DS3231_ALARM2_A2M2_SHIFT     7         /* Bits 7: A2M2 mask */
#  define DS3231_ALARM2_A2M2_MASK      (1 << DS3231_ALARM2_A2M2_SHIFT)
#    define DS3231_ALARM2_A2M2(n)      ((uint8_t)(n) << DS3231_ALARM2_A2M2_SHIFT)

#define DS3231_ALARM2_HOURR            0x0c      /* Alarm2 hours register */
#  define DS3231_ALARM2_HOUR_SHIFT     0         /* Bits 0-3: Hours, range 0-9 */
#  define DS3231_ALARM2_HOUR_MASK      (15 << DS3231_ALARM2_HOUR_SHIFT)
#    define DS3231_ALARM2_HOUR(n)      ((uint8_t)(n) << DS3231_ALARM2_HOUR_SHIFT)
#  define DS3231_ALARM2_10HOUR12_SHIFT 4         /* Bit 4: 10 hours, range 0-1 */
#  define DS3231_ALARM2_10HOUR12_MASK  (1 << DS3231_ALARM2_10HOUR12_SHIFT)
#    define DS3231_ALARM2_10HOUR12(n)  ((uint8_t)(n) << DS3231_ALARM2_10HOUR12_SHIFT)
#  define DS3231_ALARM2_10HOUR24_SHIFT 4         /* Bits 4-5: 10 hours, range 0-2 */
#  define DS3231_ALARM2_10HOUR24_MASK  (3 << DS3231_ALARM2_10HOUR24_SHIFT)
#    define DS3231_ALARM2_10HOUR24(n)  ((uint8_t)(n) << DS3231_ALARM2_10HOUR24_SHIFT)
#  define DS3231_ALARM2_HOUR12_BCDMASK (DS3231_ALARM2_HOUR_MASK | DS3231_ALARM2_10HOUR12_MASK)
#  define DS3231_ALARM2_HOUR24_BCDMASK (DS3231_ALARM2_HOUR_MASK | DS3231_ALARM2_10HOUR24_MASK)
#  define DS3231_ALARM2_AMPM_SHIFT     5         /* Bit 5: AM/PM Indication */
#  define DS3231_ALARM2_AMPM_MASK      (1 << DS3231_ALARM2_AMPM_SHIFT)
#    define DS3231_ALARM2_AM           ((uint8_t)(0) << DS3231_ALARM2_AMPM_SHIFT)
#    define DS3231_ALARM2_PM           ((uint8_t)(1) << DS3231_ALARM2_AMPM_SHIFT)
#  define DS3231_ALARM2_1224_SHIFT     6         /* Bit 6: 12/24 Indication */
#  define DS3231_ALARM2_1224_MASK      (1 << DS3231_ALARM2_1224_SHIFT)
#    define DS3231_ALARM2_12           ((uint8_t)(0) << DS3231_ALARM2_1224_SHIFT)
#    define DS3231_ALARM2_24           ((uint8_t)(1) << DS3231_ALARM2_1224_SHIFT)
#  define DS3231_ALARM2_A2M3_SHIFT     7         /* Bits 7: A2M3 mask */
#  define DS3231_ALARM2_A2M3_MASK      (1 << DS3231_ALARM2_A2M3_SHIFT)
#    define DS3231_ALARM2_A2M3(n)      ((uint8_t)(n) << DS3231_ALARM2_A2M3_SHIFT)

#define DS3231_ALARM2_DAYDATER         0x0d      /* Alarm2 date / day of the week register */
#  define DS3231_ALARM2_DAY_SHIFT      0         /* Bits 0-3: Day of the week, range 1-7 */
#  define DS3231_ALARM2_DAY_MASK       (7 << DS3231_ALARM2_DAY_SHIFT)
#    define DS3231_ALARM2_DAY(n)       ((uint8_t)(n) << DS3231_ALARM2_DAY_SHIFT)
#  define DS3231_ALARM2_DATE_SHIFT     0         /* Bits 0-3: Days, range 0-9 */
#  define DS3231_ALARM2_DATE_MASK      (15 << DS3231_ALARM2_DATE_SHIFT)
#    define DS3231_ALARM2_DATE(n)      ((uint8_t)(n) << DS3231_ALARM2_DATE_SHIFT)
#  define DS3231_ALARM2_10DATE_SHIFT   4         /* Bits 4-5: 10 days, range 0-5 */
#  define DS3231_ALARM2_10DATE_MASK    (3 << DS3231_ALARM2_10DATE_SHIFT)
#    define DS3231_ALARM2_10DATE(n)    ((uint8_t)(n) << DS3231_ALARM2_10DATE_SHIFT)
#  define DS3231_ALARM2_DATE_BCDMASK   (DS3231_ALARM2_DATE_MASK | DS3231_ALARM2_10DATE_MASK)
#  define DS3231_ALARM2_DYDT_SHIFT     6         /* Bits 6: DY/DT */
#  define DS3231_ALARM2_DYDT_MASK      (1 << DS3231_ALARM2_DYDT_SHIFT)
#    define DS3231_ALARM2_DYDT_DATE    ((uint8_t)(0) << DS3231_ALARM2_DYDT_SHIFT)
#    define DS3231_ALARM2_DYDT_DAY     ((uint8_t)(1) << DS3231_ALARM2_DYDT_SHIFT)
#  define DS3231_ALARM2_A2M4_SHIFT     7         /* Bits 7: A2M4 mask */
#  define DS3231_ALARM2_A2M4_MASK      (1 << DS3231_ALARM2_A2M4_SHIFT)
#    define DS3231_ALARM2_A2M4(n)      ((uint8_t)(n) << DS3231_ALARM2_A2M4_SHIFT)

#define DS3231_CR                      0x0e      /* Control register */
#  define DS3231_CR_A1IE               (1 << 0)  /* Bit 0:  Alarm 1 interrupt enable */
#  define DS3231_CR_A2IE               (1 << 1)  /* Bit 1:  Alarm 2 interrupt enable */
#  define DS3231_CR_INTCN              (1 << 2)  /* Bit 2:  Interrupt control */
#  define DS3231_CR_RS_SHIFT           (3)       /* Bit 3-4:  Rate selection */
#  define DS3231_CR_RS_MASK            (3 << DS3231_CR_RS_SHIFT)
#    define DS3231_CR_RS_1HZ           (0 << DS3231_CR_RS_SHIFT) /* 1Hz */
#    define DS3231_CR_RS_1KHZ          (1 << DS3231_CR_RS_SHIFT) /* 1.024kHz */
#    define DS3231_CR_RS_4KHZ          (2 << DS3231_CR_RS_SHIFT) /* 4.096kHz */
#    define DS3231_CR_RS_8KHZ          (3 << DS3231_CR_RS_SHIFT) /* 8.192kHz */
#  define DS3231_CR_CONV               (1 << 5)  /* Bit 5:  Convert temperature */
#  define DS3231_CR_BBSQW              (1 << 6)  /* Bit 6:  Battery backed square wave enable */
#  define DS3231_CR_EOSC               (1 << 7)  /* Bit 7:  Enable oscillator */

#define DS3231_CSR                     0x0f      /* Control/status register */
#  define DS3231_CSR_A1F               (1 << 0)  /* Bit 0:  Alarm 1 flag */
#  define DS3231_CSR_A2F               (1 << 1)  /* Bit 1:  Alarm 2 flag */
#  define DS3231_CSR_BSY               (1 << 2)  /* Bit 2:  Busy */
#  define DS3231_CSR_EN32kHz           (1 << 3)  /* Bit 3:  Enable 32kHz output */
#  define DS3231_CSR_OSF               (1 << 7)  /* Bit 7:  Oscillator stop flag */

#define DS3231_AGINGR                  0x10      /* Aging offset register (8-bit, 2's complement) */

#define DS3231_TMPMR                   0x11      /* MSB of temp register (8-bit, 2's complement) */

#define DS3231_TMPLR                   0x12      /* LSB of temp register (2-bits) */
#  define DS3231_TMPLR_MASK            0xc0      /* Bits 6-7: LSB of temp register (2-bits) */

#endif /* __DRIVERS_TIMERS_DS3231_H */
