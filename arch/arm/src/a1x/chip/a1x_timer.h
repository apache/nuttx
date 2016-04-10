/************************************************************************************
 * arch/arm/src/a1x/chip/a1x_timer.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_A1X_CHIP_A1X_TIMER_H
#define __ARCH_ARM_SRC_A1X_CHIP_A1X_TIMER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/a1x_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define A1X_TMR_IRQ_EN_OFFSET      0x0000 /* Timer IRQ Enable */
#define A1X_TMR_IRQ_STA_OFFSET     0x0004 /* Timer Status */

#define A1X_TMR_OFFSET(n)          (0x0010 + ((n) << 4))
#define A1X_TMR_CTRL_OFFSET        0x0000 /* Timer Control */
#define A1X_TMR_INTV_VALUE_OFFSET  0x0004 /* Timer Interval Value */
#define A1X_TMR_CUR_VALUE_OFFSET   0x0008 /* Timer Current Value */

#define A1X_TMR0_CTRL_OFFSET       0x0010 /* Timer 0 Control */
#define A1X_TMR0_INTV_VALUE_OFFSET 0x0014 /* Timer 0 Interval Value */
#define A1X_TMR0_CUR_VALUE_OFFSET  0x0018 /* Timer 0 Current Value */

#define A1X_TMR1_CTRL_OFFSET       0x0020 /* Timer 1 Control */
#define A1X_TMR1_INTV_VALUE_OFFSET 0x0024 /* Timer 1 Interval Value */
#define A1X_TMR1_CUR_VALUE_OFFSET  0x0028 /* Timer 1 Current Value */

#define A1X_TMR2_CTRL_OFFSET       0x0030 /* Timer 2 Control */
#define A1X_TMR2_INTV_VALUE_OFFSET 0x0034 /* Timer 2 Interval Value */
#define A1X_TMR2_CUR_VALUE_OFFSET  0x0038 /* Timer 2 Current Value */

#define A1X_TMR3_CTRL_OFFSET       0x0040 /* Timer 3 Control */
#define A1X_TMR3_INTV_VALUE_OFFSET 0x0044 /* Timer 3 Interval Value */

#define A1X_TMR4_CTRL_OFFSET       0x0050 /* Timer 4 Control */
#define A1X_TMR4_INTV_VALUE_OFFSET 0x0054 /* Timer 4 Interval Value */
#define A1X_TMR4_CUR_VALUE_OFFSET  0x0058 /* Timer 4 Current Value */

#define A1X_TMR5_CTRL_OFFSET       0x0060 /* Timer 5 Control */
#define A1X_TMR5_INTV_VALUE_OFFSET 0x0064 /* Timer 5 Interval Value */
#define A1X_TMR5_CUR_VALUE_OFFSET  0x0068 /* Timer 5 Current Value */

#define A1X_AVS_CNT_CTL_OFFSET     0x0080 /* AVS Control Register */
#define A1X_AVS_CNT0_OFFSET        0x0084 /* AVS Counter 0 Register */
#define A1X_AVS_CNT1_OFFSET        0x0088 /* AVS Counter 1 Register */
#define A1X_AVS_CNT_DIV_OFFSET     0x008c /* AVS Divisor */

#define A1X_WDOG_CTRL_OFFSET       0x0090 /* Watchdog Control */
#define A1X_WDOG_MODE_OFFSET       0x0094 /* Watchdog Mode */

#define A1X_CNT64_CTRL_OFFSET      0x00a0 /* 64-bit Counter control */
#define A1X_CNT64_LO_OFFSET        0x00a4 /* 64-bit Counter low */
#define A1X_CNT64_HI_OFFSET        0x00a8 /* 64-bit Counter high */

#define A1X_LOSC_CTRL_OFFSET       0x0100 /* Low Oscillator Control */

#define A1X_RTC_YMD_OFFSET         0x0104 /* RTC Year-Month-Day */
#define A1X_RTC_HMS_OFFSET         0x0108 /* RTC Hour-Minute-Second */
#define A1X_ALRAM_DD_HMS_OFFSET    0x010c /* Alarm Day-Hour-Minute-Second */
#define A1X_ALARM_WK_HMS_OFFSET    0x0110 /* Alarm Week HMS */
#define A1X_ALARM_EN_OFFSET        0x0114 /* Alarm Enable */
#define A1X_ALARM_IRQ_EN_OFFSET    0x0118 /* Alarm IRQ Enable */
#define A1X_ALARM_IRQ_STA_OFFSET   0x011c /* Alarm IRQ Status */

#define A1X_TMR_GP_DATA0_OFFSET    0x0120 /* Timer general purpose register 0 */
#define A1X_TMR_GP_DATA1_OFFSET    0x0124 /* Timer general purpose register 1 */
#define A1X_TMR_GP_DATA2_OFFSET    0x0128 /* Timer general purpose register 2 */
#define A1X_TMR_GP_DATA3_OFFSET    0x012c /* Timer general purpose register 3 */

#define A1X_CPU_CFG_OFFSET         0x0140 /* CPU configuration register */

/* Register virtual addresses *******************************************************/

#define A1X_TMR_IRQ_EN             (A1X_TIMER_VADDR+A1X_TMR_IRQ_EN_OFFSET)
#define A1X_TMR_IRQ_STA            (A1X_TIMER_VADDR+A1X_TMR_IRQ_STA_OFFSET)

#define A1X_TMR(n)                 (A1X_TIMER_VADDR+A1X_TMR_OFFSET(n))
#define A1X_TMR_CTRL               (A1X_TIMER_VADDR+A1X_TMR_CTRL_OFFSET)
#define A1X_TMR_INTV_VALUE         (A1X_TIMER_VADDR+A1X_TMR_INTV_VALUE_OFFSET)
#define A1X_TMR_CUR_VALUE          (A1X_TIMER_VADDR+A1X_TMR_CUR_VALUE_OFFSET)

#define A1X_TMR0_CTRL              (A1X_TIMER_VADDR+A1X_TMR0_CTRL_OFFSET)
#define A1X_TMR0_INTV_VALUE        (A1X_TIMER_VADDR+A1X_TMR0_INTV_VALUE_OFFSET)
#define A1X_TMR0_CUR_VALUE         (A1X_TIMER_VADDR+A1X_TMR0_CUR_VALUE_OFFSET)

#define A1X_TMR1_CTRL              (A1X_TIMER_VADDR+A1X_TMR1_CTRL_OFFSET)
#define A1X_TMR1_INTV_VALUE        (A1X_TIMER_VADDR+A1X_TMR1_INTV_VALUE_OFFSET)
#define A1X_TMR1_CUR_VALUE         (A1X_TIMER_VADDR+A1X_TMR1_CUR_VALUE_OFFSET)

#define A1X_TMR2_CTRL              (A1X_TIMER_VADDR+A1X_TMR2_CTRL_OFFSET)
#define A1X_TMR2_INTV_VALUE        (A1X_TIMER_VADDR+A1X_TMR2_INTV_VALUE_OFFSET)
#define A1X_TMR2_CUR_VALUE         (A1X_TIMER_VADDR+A1X_TMR2_CUR_VALUE_OFFSET)

#define A1X_TMR3_CTRL              (A1X_TIMER_VADDR+A1X_TMR3_CTRL_OFFSET)
#define A1X_TMR3_INTV_VALUE        (A1X_TIMER_VADDR+A1X_TMR3_INTV_VALUE_OFFSET)

#define A1X_TMR4_CTRL              (A1X_TIMER_VADDR+A1X_TMR4_CTRL_OFFSET)
#define A1X_TMR4_INTV_VALUE        (A1X_TIMER_VADDR+A1X_TMR4_INTV_VALUE_OFFSET)
#define A1X_TMR4_CUR_VALUE         (A1X_TIMER_VADDR+A1X_TMR4_CUR_VALUE_OFFSET)

#define A1X_TMR5_CTRL              (A1X_TIMER_VADDR+A1X_TMR5_CTRL_OFFSET)
#define A1X_TMR5_INTV_VALUE        (A1X_TIMER_VADDR+A1X_TMR5_INTV_VALUE_OFFSET)
#define A1X_TMR5_CUR_VALUE         (A1X_TIMER_VADDR+A1X_TMR5_CUR_VALUE_OFFSET)

#define A1X_AVS_CNT_CTL            (A1X_TIMER_VADDR+A1X_AVS_CNT_CTL_OFFSET)
#define A1X_AVS_CNT0               (A1X_TIMER_VADDR+A1X_AVS_CNT0_OFFSET)
#define A1X_AVS_CNT1               (A1X_TIMER_VADDR+A1X_AVS_CNT1_OFFSET)
#define A1X_AVS_CNT_DIV            (A1X_TIMER_VADDR+A1X_AVS_CNT_DIV_OFFSET)

#define A1X_WDOG_CTRL              (A1X_TIMER_VADDR+A1X_WDOG_CTRL_OFFSET)
#define A1X_WDOG_MODE              (A1X_TIMER_VADDR+A1X_WDOG_MODE_OFFSET)

#define A1X_CNT64_CTRL             (A1X_TIMER_VADDR+A1X_CNT64_CTRL_OFFSET)
#define A1X_CNT64_LO               (A1X_TIMER_VADDR+A1X_CNT64_LO_OFFSET)
#define A1X_CNT64_HI               (A1X_TIMER_VADDR+A1X_CNT64_HI_OFFSET)

#define A1X_LOSC_CTRL              (A1X_TIMER_VADDR+A1X_LOSC_CTRL_OFFSET)

#define A1X_RTC_YMD                (A1X_TIMER_VADDR+A1X_RTC_YMD_OFFSET)
#define A1X_RTC_HMS                (A1X_TIMER_VADDR+A1X_RTC_HMS_OFFSET)
#define A1X_ALRAM_DD_HMS           (A1X_TIMER_VADDR+A1X_ALRAM_DD_HMS_OFFSET)
#define A1X_ALARM_WK_HMS           (A1X_TIMER_VADDR+A1X_ALARM_WK_HMS_OFFSET)
#define A1X_ALARM_EN               (A1X_TIMER_VADDR+A1X_ALARM_EN_OFFSET)
#define A1X_ALARM_IRQ_EN           (A1X_TIMER_VADDR+A1X_ALARM_IRQ_EN_OFFSET)
#define A1X_ALARM_IRQ_STA          (A1X_TIMER_VADDR+A1X_ALARM_IRQ_STA_OFFSET)

#define A1X_TMR_GP_DATA0           (A1X_TIMER_VADDR+A1X_TMR_GP_DATA0_OFFSET)
#define A1X_TMR_GP_DATA1           (A1X_TIMER_VADDR+A1X_TMR_GP_DATA1_OFFSET)
#define A1X_TMR_GP_DATA2           (A1X_TIMER_VADDR+A1X_TMR_GP_DATA2_OFFSET)
#define A1X_TMR_GP_DATA3           (A1X_TIMER_VADDR+A1X_TMR_GP_DATA3_OFFSET)

#define A1X_CPU_CFG                (A1X_TIMER_VADDR+A1X_CPU_CFG_OFFSET)

/* Register bit field definitions ***************************************************/

/* Timer IRQ Enable and Timer Status */

#define TMR_IRQ_TMR0               (1 << 0)  /* Bit 0:  Timer 0 Interrupt */
#define TMR_IRQ_TMR1               (1 << 1)  /* Bit 1:  Timer 1 Interrupt */
#define TMR_IRQ_TMR2               (1 << 2)  /* Bit 2:  Timer 2 Interrupt */
#define TMR_IRQ_TMR3               (1 << 3)  /* Bit 3:  Timer 3 Interrupt */
#define TMR_IRQ_TMR4               (1 << 4)  /* Bit 4:  Timer 4 Interrupt */
#define TMR_IRQ_TMR5               (1 << 5)  /* Bit 5:  Timer 5 Interrupt */
#define TMR_IRQ_WDOG               (1 << 8)  /* Bit 8:  Watchdog Interrupt */

/* Timer 0-2,/4-5 Control */

#define TMR_CTRL_EN                (1 << 0)  /* Bit 0:  Timer n Enable, n={0,1,2,3,4,5} */
#define TMR_CTRL_RELOAD            (1 << 1)  /* Bit 1:  Timer n Reload, n={0,1,2,4,5} */
#define TMR_CTRL_SRC_SHIFT         (2)       /* Bits 2-3: Timer n Clock Source, n={0,1,2,4,5} */
#define TMR_CTRL_SRC_MASK          (3 << TMR_CTRL_SRC_SHIFT)
#  define TMR_CTRL_SRC_LOSC        (0 << TMR_CTRL_SRC_SHIFT) /* Low speed OSC */
#  define TMR_CTRL_SRC_OSC24M      (1 << TMR_CTRL_SRC_SHIFT) /* OSC24M */
#  define TMR01_CTRL_SRC_PLL6DIV6  (2 << TMR_CTRL_SRC_SHIFT) /* PLL6/6 (Timers 0 and 1 only) */
#  define TMR4_CTRL_SRC_CLKIN0     (2 << TMR_CTRL_SRC_SHIFT) /* External CLKIN0 (Timer 4 only) */
#  define TMR5_CTRL_SRC_CLKIN1     (2 << TMR_CTRL_SRC_SHIFT) /* External CLKIN1 (Timer 5 only) */
#define TMR_CTRL_CLK_PRES_SHIFT    (4) /* Bits 4-6: Select the pre-scale of timer n clock source */
#define TMR_CTRL_CLK_PRES_MASK     (7 << TMR_CTRL_CLK_PRES_SHIFT)
#  define TMR_CTRL_CLK_PRES_DIV1   (0 << TMR_CTRL_CLK_PRES_SHIFT) /* /1 */
#  define TMR_CTRL_CLK_PRES_DIV2   (1 << TMR_CTRL_CLK_PRES_SHIFT) /* /2 */
#  define TMR_CTRL_CLK_PRES_DIV4   (2 << TMR_CTRL_CLK_PRES_SHIFT) /* /4 */
#  define TMR_CTRL_CLK_PRES_DIV8   (3 << TMR_CTRL_CLK_PRES_SHIFT) /* /8 */
#  define TMR_CTRL_CLK_PRES_DIV16  (4 << TMR_CTRL_CLK_PRES_SHIFT) /* /16 (Not Timer 0) */
#  define TMR_CTRL_CLK_PRES_DIV32  (5 << TMR_CTRL_CLK_PRES_SHIFT) /* /32 (Not Timer 0) */
#  define TMR_CTRL_CLK_PRES_DIV64  (6 << TMR_CTRL_CLK_PRES_SHIFT) /* /64 (Not Timer 0) */
#  define TMR_CTRL_CLK_PRES_DIV128 (7 << TMR_CTRL_CLK_PRES_SHIFT) /* /128 (Not Timer 0) */
#define TMR_CTRL_MODE              (1 << 7)  /* Bit 7:  Timer n mode, n={0,1,2,4,5} */
#  define TMR_CTRL_MODE_SINGLE     (1 << 7)  /*         1=single mode */
#  define TMR_CTRL_MODE_CONTINUOUS (0 << 7)  /*         0=continuous mode */

/* Timer 3 Control */

#define TMR3_CTRL_EN               (1 << 0)  /* Bit 0:  Timer 3 Enable*/
#define TMR3_CTRL_CLK_PRES_SHIFT   (2) /* Bits 2-3: Select the pre-scale of timer 3 clock source (LOSC) */
#define TMR3_CTRL_CLK_PRES_MASK    (7 << TMR3_CTRL_CLK_PRES_SHIFT)
#  define TMR3_CTRL_CLK_PRES_DIV16 (0 << TMR3_CTRL_CLK_PRES_SHIFT) /* /16 */
#  define TMR3_CTRL_CLK_PRES_DIV32 (1 << TMR3_CTRL_CLK_PRES_SHIFT) /* /32 */
#  define TMR3_CTRL_CLK_PRES_DIV64 (2 << TMR3_CTRL_CLK_PRES_SHIFT) /* /64 */
#define TMR3_CTRL_MODE             (1 << 4)  /* Bit 4:  Timer3 mode */

/* Timer Interval Value (32-bit value) */
/* Timer Current Value (32-bit value) */

/* AVS Control Register */

#define AVS_CNT0_EN                (1 << 0)  /* Bit 0:  Audio/Video Sync Counter 1 Enable/ Disable (OSC24M) */
#define AVS_CNT1_EN                (1 << 1)  /* Bit 1:  Audio/Video Sync Counter 1 Enable/ Disable (OSC24M) */
#define AVS_CNT0_PS                (1 << 8)  /* Bit 8:  Audio/Video Sync Counter 0 Pause Control */
#define AVS_CNT1_PS                (1 << 9)  /* Bit 9:  Audio/Video Sync Counter 1 Pause Control */

/* AVS Counter 0 Register (32-bit value) */
/* AVS Counter 1 Register (32-bit value) */

/* AVS Divisor */

#define AVS_CNT0_D_SHIFT           (0)       /* Bits 0-11: Divisor N for AVS Counter 0 */
#define AVS_CNT0_D_MASK            (0xfff << AVS_CNT0_D_SHIFT)
#  define AVS_CNT0_D(n)            ((uint32_t)(n) << AVS_CNT0_D_SHIFT)
#define AVS_CNT1_D_SHIFT           (16)      /* Bits 16-27: Divisor N for AVS Counter 1 */
#define AVS_CNT1_D_MASK            (0xfff << AVS_CNT1_D_SHIFT)
#  define AVS_CNT1_D(n)            ((uint32_t)(n) << AVS_CNT1_D_SHIFT)

/* Watchdog Control */

#define WDOG_CTRL_RSTART           (1 << 0)  /* Bit 0:  Watch-Dog Restart */

/* Watchdog Mode */

#define WDOG_MODE_EN               (1 << 0)  /* Bit 0:  Watch-Dog Enable */
#define WDOG_MODE_RSTEN            (1 << 1)  /* Bit 1:  Watch-Dog Reset Enable */
#define WDOG_MODE_INTV_SHIFT       (3)       /* Bits 3-6: Watch-Dog Interval Value */
#define WDOG_MODE_INTV_MASK        (15 << WDOG_MODE_INTV_SHIFT)
#  define WDOG_MODE_INTV_0p5SEC    (0 << WDOG_MODE_INTV_SHIFT) /* 0.5 sec */
#  define WDOG_MODE_INTV_1SEc      (1 << WDOG_MODE_INTV_SHIFT) /* 1 sec */
#  define WDOG_MODE_INTV_2SEC      (2 << WDOG_MODE_INTV_SHIFT) /* 2 sec */
#  define WDOG_MODE_INTV_3SEC      (3 << WDOG_MODE_INTV_SHIFT) /* 3 sec */
#  define WDOG_MODE_INTV_4SEC      (4 << WDOG_MODE_INTV_SHIFT) /* 4 sec */
#  define WDOG_MODE_INTV_5SEC      (5 << WDOG_MODE_INTV_SHIFT) /* 5 sec */
#  define WDOG_MODE_INTV_6SEC      (6 << WDOG_MODE_INTV_SHIFT) /* 6 sec */
#  define WDOG_MODE_INTV_8SEC      (7 << WDOG_MODE_INTV_SHIFT) /* 8 sec */
#  define WDOG_MODE_INTV_10SEC     (8 << WDOG_MODE_INTV_SHIFT) /* 10 sec */
#  define WDOG_MODE_INTV_12SEC     (9 << WDOG_MODE_INTV_SHIFT) /* 12 sec */
#  define WDOG_MODE_INTV_14SEC     (10 << WDOG_MODE_INTV_SHIFT) /* 14 sec */
#  define WDOG_MODE_INTV_16SEC     (11 << WDOG_MODE_INTV_SHIFT) /* 16 sec */

/* 64-bit Counter control */

#define CNT64_CTRL_CLREN           (1 << 0)  /* Bit 0:  64-bit Counter Clear Enable */
#define CNT64_CTRL_RLEN            (1 << 1)  /* Bit 1:  64-bit Counter Read Latch Enable */
#define CNT64_CTRL_SRCSEL_MASK     (1 << 2)  /* Bit 2:  64-bit Counter Clock Source Select */
#  define CNT64_CTRL_SRC_OSC24M    (0 << 2)  /*         0=OSC24M */
#  define CNT64_CTRL_SRC_PLL6DIV6  (1 << 2)  /*         1=PLL6/6 */

/* 64-bit Counter low (32-bit value) */
/* 64-bit Counter high (32-bit value) */

/* Low Oscillator Control */

#define LOSC_CTRL_OSC32K_SRCSEL    (1 << 0)  /* Bit 0:  OSC32KHz Clock source Select */
#define LOSC_CTRL_EXT_GSM_SHIFT    (2)  /* Bits 2-3: External 32768Hz Crystal GSM */
#define LOSC_CTRL_EXT_GSM_MASK     (3 << LOSC_CTRL_EXT_GSM_SHIFT)
#  define LOSC_CTRL_EXT_GSM_LOW    (0 << LOSC_CTRL_EXT_GSM_SHIFT)
#  define LOSC_CTRL_EXT_GSM_MEDLOW (1 << LOSC_CTRL_EXT_GSM_SHIFT)
#  define LOSC_CTRL_EXT_GSM_MEDHI  (2 << LOSC_CTRL_EXT_GSM_SHIFT)
#  define LOSC_CTRL_EXT_GSM_HIGH   (3 << LOSC_CTRL_EXT_GSM_SHIFT)
#define LOSC_CTRL_RTC_YMD_ACCE     (1 << 7)  /* Bit 7:  RTC YY-MM-DD access */
#define LOSC_CTRL_RTC_HMS_ACCE     (1 << 8)  /* Bit 8:  RTC HH-MM-SS access */
#define LOSC_CTRL_ALM_DHMS_ACCE    (1 << 9)  /* Bit 9:  ALARM DD-HH-MM-SS access */
#define LOSC_CTRL_CLK32K_AUTOSWE   (1 << 14) /* Bit 14: CLK32K auto switch enable */
#define LOSC_CTRL_CLK32K_AUTOSWP   (1 << 15) /* Bit 15: CLK32K auto switch pending */

/* RTC Year-Month-Day */

#define RTC_YMD_DAY_SHIFT          (0) /* Bits 0-4: Day (1-31) */
#define RTC_YMD_DAY_MASK           (0x1f << RTC_YMD_DAY_SHIFT)
#  define RTC_YMD_DAY(n)           ((uint32_t)(n) << RTC_YMD_DAY_SHIFT)
#define RTC_YMD_MONTH_SHIFT        (8) /* Bit 8-11: Month (1-12) */
#define RTC_YMD_MONTH_MASK         (15 << RTC_YMD_MONTH_SHIFT)
#  define RTC_YMD_MONTH(n)         ((uint32_t)(n) << RTC_YMD_MONTH_SHIFT)
#define RTC_YMD_YEAR_SHIFT         (16) /* Bit 16-21: Year (0-63) */
#define RTC_YMD_YEAR_MASK          (0x3f << RTC_YMD_YEAR_SHIFT)
#  define RTC_YMD_YEAR(n)          ((uint32_t)(n) << RTC_YMD_YEAR_SHIFT)
#define RTC_YMD_LEAP               (1 << 22) /* Bit 22: Leap Year */
#define RTC_YMD_SIMCTRL            (1 << 30) /* Bit 30: RTC Simulation Control bit */
#define RTC_YMD_TESTMODECTRL       (1 << 31) /* Bit 31: RTC TEST Mode Control bit */

/* RTC Hour-Minute-Second */

#define RTC_HMS_SECOND_SHIFT       (0)       /* Bits 0-5: Second (0-59) */
#define RTC_HMS_SECOND_MASK        (0x3f << RTC_HMS_SECOND_SHIFT)
#  define RTC_HMS_SECOND(n)        ((uint32_t)(n) << RTC_HMS_SECOND_SHIFT)
#define RTC_HMS_MINUTE_SHIFT       (8)       /* Bits 8-13: Minute (0-59) */
#define RTC_HMS_MINUTE_MASK        (0x3f << RTC_HMS_MINUTE_SHIFT)
#  define RTC_HMS_MINUTE(n)        ((uint32_t)(n) << RTC_HMS_MINUTE_SHIFT)
#define RTC_HMS_HOUR_SHIFT         (16)      /* Bits 16-20: Hour (0-23) */
#define RTC_HMS_HOUR_MASK          (0x1f << RTC_HMS_HOUR_SHIFT)
#  define RTC_HMS_HOUR(n)          ((uint32_t)(n) << RTC_HMS_HOUR_SHIFT)
#define RTC_HMS_WKNO_SHIFT         (29)      /* Bits 29-31: Week number */
#define RTC_HMS_WKNO_MASK          (7 << RTC_HMS_WKNO_SHIFT)
#  define RTC_HMS_WKNO(n)          ((uint32_t)(n) << RTC_HMS_WKNO_SHIFT)
#  define RTC_HMS_WKNO_MONDAY      (0 << RTC_HMS_WKNO_SHIFT)
#  define RTC_HMS_WKNO_TUESDAY     (1 << RTC_HMS_WKNO_SHIFT)
#  define RTC_HMS_WKNO_WEDNESDAY   (2 << RTC_HMS_WKNO_SHIFT)
#  define RTC_HMS_WKNO_THURSDAY    (3 << RTC_HMS_WKNO_SHIFT)
#  define RTC_HMS_WKNO_FRIDAY      (4 << RTC_HMS_WKNO_SHIFT)
#  define RTC_HMS_WKNO_SATURDAY    (5 << RTC_HMS_WKNO_SHIFT)
#  define RTC_HMS_WKNO_SUNDAY      (6 << RTC_HMS_WKNO_SHIFT)

/* Alarm Day-Hour-Minute-Second */

#define ALRAM_DD_HMS_SECOND_SHIFT  (0)       /* Bits 8-13: Second (0-59) */
#define ALRAM_DD_HMS_SECOND_MASK   (0x3f << ALRAM_DD_HMS_SECOND_SHIFT)
#  define ALRAM_DD_HMS_SECOND(n)   ((uint32_t)(n) << ALRAM_DD_HMS_SECOND_SHIFT)
#define ALRAM_DD_HMS_MINUTE_SHIFT  (8)       /* Bits 8-13: Minute (0-59) */
#define ALRAM_DD_HMS_MINUTE_MASK   (0x3f << ALRAM_DD_HMS_MINUTE_SHIFT)
#  define ALRAM_DD_HMS_MINUTE(n)   ((uint32_t)(n) << ALRAM_DD_HMS_MINUTE_SHIFT)
#define ALRAM_DD_HMS_HOUR_SHIFT    (16)      /* Bits 16-20: Hour (0-23) */
#define ALRAM_DD_HMS_HOUR_MASK     (0x1f << ALRAM_DD_HMS_HOUR_SHIFT)
#  define ALRAM_DD_HMS_HOUR(n)     ((uint32_t)(n) << ALRAM_DD_HMS_HOUR_SHIFT)
#define ALRAM_DD_HMS_DAY_SHIFT     (24)      /* Bits 24-31: Day (0-255) */
#define ALRAM_DD_HMS_DAY_MASK      (0xff << ALRAM_DD_HMS_DAY_SHIFT)
#  define ALRAM_DD_HMS_DAY(n)      ((uint32_t)(n) << ALRAM_DD_HMS_DAY_SHIFT)

/* Alarm Week HMS */

#define ALARM_WK_HMS_SECOND_SHIFT  (0)   /* Bits 0-5: Seconds (0-59) */
#define ALARM_WK_HMS_SECOND_MASK   (0x3f << ALARM_WK_HMS_SECOND_SHIFT)
#  define ALARM_WK_HMS_SECOND(n)   ((uint32_t)(n) << ALARM_WK_HMS_SECOND_SHIFT)
#define ALARM_WK_HMS_MINUTE_SHIFT  (8)   /* Bits 8-13:  Minutes (0-59) */
#define ALARM_WK_HMS_MINUTE_MASK   (0x3f << ALARM_WK_HMS_MINUTE_SHIFT)
#  define ALARM_WK_HMS_MINUTE(n)   ((uint32_t)(n) << ALARM_WK_HMS_MINUTE_SHIFT)
#define ALARM_WK_HMS_HOUR_SHIFT    (16)  /* Bits 16-20: Hours (0-23) */
#define ALARM_WK_HMS_HOUR_MASK     (0x1f << ALARM_WK_HMS_HOUR_SHIFT)
#  define ALARM_WK_HMS_HOUR(n)     ((uint32_t)(n) << ALARM_WK_HMS_HOUR_SHIFT)

/* Alarm Enable */

#define ALARM_EN_WK0EN             (1 << 0)  /* Bit 0:  Week 0(Monday) Alarm Enable */
#define ALARM_EN_WK1EN             (1 << 1)  /* Bit 1:  Week 1(Tuesday) Alarm Enable */
#define ALARM_EN_WK2EN             (1 << 2)  /* Bit 2:  Week 2(Wednesday) Alarm Enable */
#define ALARM_EN_WK3EN             (1 << 3)  /* Bit 3:  Week 3(Thursday) Alarm Enable */
#define ALARM_EN_WK4EN             (1 << 4)  /* Bit 4:  Week 4(Friday) Alarm Enable */
#define ALARM_EN_WK5EN             (1 << 5)  /* Bit 5:  Week 5(Saturday) Alarm Enable */
#define ALARM_EN_WK6EN             (1 << 6)  /* Bit 6:  Week 6(Sunday) Alarm Enable */
#define ALARM_EN_CNTEN             (1 << 8)  /* Bit 8:  Alarm Counter Enable */

/* Alarm IRQ Enable and Alarm IRQ Status*/

#define ALARM_IRQ_CNT              (1 << 0)  /* Bit 0:  Alarm Counter IRQ */
#define ALARM_IRQ_WK               (1 << 1)  /* Bit 1:  Alarm Week IRQ */

/* Timer general purpose register 0-3 (32-bit values) */

/* CPU configuration register */

#define CPU_CFG_L2DCACHE_INVEN     (1 << 0)  /* Bit 0:  Enable L2 data cache invalidation at reset */
#define CPU_CFG_L1DCACHE_INVAEN    (1 << 1)  /* Bit 1:  Enable L1 data cache invalidation at reset */

#endif /* __ARCH_ARM_SRC_A1X_CHIP_A1X_TIMER_H */
