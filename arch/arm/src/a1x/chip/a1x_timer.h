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

#define A1X_RTC_YYMMDD_OFFSET      0x0104 /* RTC Year-Month-Day */
#define A1X_RTC_HHMMSS_OFFSET      0x0108 /* RTC Hour-Minute-Second */
#define A1X_ALRAM_DD_HHMMSS_OFFSET 0x010c /* Alarm Day-Hour-Minute-Second */
#define A1X_ALARM_WK_HHMMSS_OFFSET 0x0110 /* Alarm Week HMS */
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

#define A1X_RTC_YYMMDD             (A1X_TIMER_VADDR+A1X_RTC_YYMMDD_OFFSET)
#define A1X_RTC_HHMMSS             (A1X_TIMER_VADDR+A1X_RTC_HHMMSS_OFFSET)
#define A1X_ALRAM_DD_HHMMSS        (A1X_TIMER_VADDR+A1X_ALRAM_DD_HHMMSS_OFFSET)
#define A1X_ALARM_WK_HHMMSS        (A1X_TIMER_VADDR+A1X_ALARM_WK_HHMMSS_OFFSET)
#define A1X_ALARM_EN               (A1X_TIMER_VADDR+A1X_ALARM_EN_OFFSET)
#define A1X_ALARM_IRQ_EN           (A1X_TIMER_VADDR+A1X_ALARM_IRQ_EN_OFFSET)
#define A1X_ALARM_IRQ_STA          (A1X_TIMER_VADDR+A1X_ALARM_IRQ_STA_OFFSET)

#define A1X_TMR_GP_DATA0           (A1X_TIMER_VADDR+A1X_TMR_GP_DATA0_OFFSET)
#define A1X_TMR_GP_DATA1           (A1X_TIMER_VADDR+A1X_TMR_GP_DATA1_OFFSET)
#define A1X_TMR_GP_DATA2           (A1X_TIMER_VADDR+A1X_TMR_GP_DATA2_OFFSET)
#define A1X_TMR_GP_DATA3           (A1X_TIMER_VADDR+A1X_TMR_GP_DATA3_OFFSET)

#define A1X_CPU_CFG                (A1X_TIMER_VADDR+A1X_CPU_CFG_OFFSET)

/* Register bit field definitions ***************************************************/

/* Timer IRQ Enable */
#define TMR_IRQ_EN_
/* Timer Status */
#define TMR_IRQ_STA_

/* Timer Control */
#define TMR_CTRL_
/* Timer Interval Value */
#define TMR_INTV_VALUE_
/* Timer Current Value */
#define TMR_CUR_VALUE_

/* Timer 0 Control */
#define TMR0_CTRL_
/* Timer 0 Interval Value */
#define TMR0_INTV_VALUE_
/* Timer 0 Current Value */
#define TMR0_CUR_VALUE_

/* Timer 1 Control */
#define TMR1_CTRL_
/* Timer 1 Interval Value */
#define TMR1_INTV_VALUE_
/* Timer 1 Current Value */
#define TMR1_CUR_VALUE_

/* Timer 2 Control */
#define TMR2_CTRL_
/* Timer 2 Interval Value */
#define TMR2_INTV_VALUE_
/* Timer 2 Current Value */
#define TMR2_CUR_VALUE_

/* Timer 3 Control */
#define TMR3_CTRL_
/* Timer 3 Interval Value */
#define TMR3_INTV_VALUE_

/* Timer 4 Control */
#define TMR4_CTRL_
/* Timer 4 Interval Value */
#define TMR4_INTV_VALUE_
/* Timer 4 Current Value */
#define TMR4_CUR_VALUE_

/* Timer 5 Control */
#define TMR5_CTRL_
/* Timer 5 Interval Value */
#define TMR5_INTV_VALUE_
/* Timer 5 Current Value */
#define TMR5_CUR_VALUE_

/* AVS Control Register */
#define AVS_CNT_CTL_
/* AVS Counter 0 Register */
#define AVS_CNT0_
/* AVS Counter 1 Register */
#define AVS_CNT1_
/* AVS Divisor */
#define AVS_CNT_DIV_

/* Watchdog Control */
#define WDOG_CTRL_
/* Watchdog Mode */
#define WDOG_MODE_

/* 64-bit Counter control */
#define CNT64_CTRL_
/* 64-bit Counter low */
#define CNT64_LO_
/* 64-bit Counter high */
#define CNT64_HI_

/* Low Oscillator Control */
#define LOSC_CTRL_

/* RTC Year-Month-Day */
#define RTC_YYMMDD_
/* RTC Hour-Minute-Second */
#define RTC_HHMMSS_
/* Alarm Day-Hour-Minute-Second */
#define ALRAM_DD_HHMMSS_
/* Alarm Week HMS */
#define ALARM_WK_HHMMSS_
/* Alarm Enable */
#define ALARM_EN_
/* Alarm IRQ Enable */
#define ALARM_IRQ_
/* Alarm IRQ Status */
#define ALARM_IRQ_STA_

/* Timer general purpose register 0 */
#define TMR_GP_DATA0_
/* Timer general purpose register 1 */
#define TMR_GP_DATA1_
/* Timer general purpose register 2 */
#define TMR_GP_DATA2_
/* Timer general purpose register 3 */
#define TMR_GP_DATA3_

/* CPU configuration register */
#define CPU_CFG_

#endif /* __ARCH_ARM_SRC_A1X_CHIP_A1X_TIMER_H */
