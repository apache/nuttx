/************************************************************************************
 * arch/arm/src/lpc17xx_40xx/hardware/lpc17_40_rtcevmr.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_RTCEVMR_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_RTCEVMR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/lpc17_40_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC17_40_RTCEV_ERCONTROL_OFFSET         0x0084 /* Monitor/Recorder Control register */
#define LPC17_40_RTCEV_ERSTATUS_OFFSET          0x0080 /* Status register */
#define LPC17_40_RTCEV_ERCOUNTERS_OFFSET        0x0088 /* Counters register */
#define LPC17_40_RTCEV_ERFIRSTSTAMP0_OFFSET     0x0090 /* Channel 0 first Stamp register */
#define LPC17_40_RTCEV_ERFIRSTSTAMP1_OFFSET     0x0090 /* Channel 1 first Stamp register */
#define LPC17_40_RTCEV_ERFIRSTSTAMP2_OFFSET     0x0090 /* Channel 2 first Stamp register */
#define LPC17_40_RTCEV_ERLASTSTAMP0_OFFSET      0x0098 /* Channel 0 last stamp register */
#define LPC17_40_RTCEV_ERLASTSTAMP1_OFFSET      0x00a0 /* Channel 1 last stamp register */
#define LPC17_40_RTCEV_ERLASTSTAMP2_OFFSET      0x00a8 /* Channel 2 last stamp register */

#define LPC17_40_RTCEV_ERCONTROL                (LPC17_40_RTC_BASE+LPC17_40_RTCEV_ERCONTROL_OFFSET)
#define LPC17_40_RTCEV_ERSTATUS                 (LPC17_40_RTC_BASE+LPC17_40_RTCEV_ERSTATUS_OFFSET)
#define LPC17_40_RTCEV_ERCOUNTERS               (LPC17_40_RTC_BASE+LPC17_40_RTCEV_ERCOUNTERS_OFFSET)
#define LPC17_40_RTCEV_ERFIRSTSTAMP0            (LPC17_40_RTC_BASE+LPC17_40_RTCEV_ERFIRSTSTAMP0_OFFSET)
#define LPC17_40_RTCEV_ERFIRSTSTAMP1            (LPC17_40_RTC_BASE+LPC17_40_RTCEV_ERFIRSTSTAMP1_OFFSET)
#define LPC17_40_RTCEV_ERFIRSTSTAMP2            (LPC17_40_RTC_BASE+LPC17_40_RTCEV_ERFIRSTSTAMP2_OFFSET)
#define LPC17_40_RTCEV_ERLASTSTAMP0             (LPC17_40_RTC_BASE+LPC17_40_RTCEV_ERLASTSTAMP0_OFFSET)
#define LPC17_40_RTCEV_ERLASTSTAMP1             (LPC17_40_RTC_BASE+LPC17_40_RTCEV_ERLASTSTAMP1_OFFSET)
#define LPC17_40_RTCEV_ERLASTSTAMP2             (LPC17_40_RTC_BASE+LPC17_40_RTCEV_ERLASTSTAMP2_OFFSET)

/* RTCEV ERCONTROL Event Monitor/Recorder Control Register */

#define RTCEV_ERCONTROL_INTWAKE_EN0             (1)       /* Bit 0: Interrupt/wakeup enable channel 0 */
#define RTCEV_ERCONTROL_GPCLEAR_EN0             (1 << 1)  /* Bit 1: Automatic clearing of RTC - channel 0 */
#define RTCEV_ERCONTROL_POL0                    (1 << 2)  /* Bit 2: Edge polarity on RTC_EV0 pins */
#define RTCEV_ERCONTROL_EV0_INPUT_EN            (1 << 3)  /* Bit 3: Event enable for channel 0 */
                                                          /* Bits 4-9: Reserved */
#define RTCEV_ERCONTROL_INTWAKE_EN1             (1 << 10) /* Bit 10: Interrupt/wakeup enable - channel 1 */
#define RTCEV_ERCONTROL_GPCLEAR_EN1             (1 << 11) /* Bit 11: Automatic clearing of RTC - channel 1 */
#define RTCEV_ERCONTROL_POL1                    (1 << 12) /* Bit 12: Edge polarity on RTC_EV1 pins */
#define RTCEV_ERCONTROL_EV1_INPUT_EN            (1 << 13) /* Bit 13: Event enable for channel 1 */
                                                          /* Bits 14-19: Reserved */
#define RTCEV_ERCONTROL_INTWAKE_EN2             (1 << 20) /* Bit 20: Interrupt/wakeup enable - channel 2 */
#define RTCEV_ERCONTROL_GPCLEAR_EN2             (1 << 21) /* Bit 21: Automatic clearing of RTC - channel 2 */
#define RTCEV_ERCONTROL_POL2                    (1 << 22) /* Bit 22: Edge polarity on RTC_EV2 pins */
#define RTCEV_ERCONTROL_EV2_INPUT_EN            (1 << 23) /* Bit 23: Event enable for channel 1 */
                                                          /* Bits 24-29: Reserved */
#define RTCEV_ERCONTROL_ERMODE_SHIFT            (30)      /* Bits 30-31:  Event monitoring mode */
#define RTCEV_ERCONTROL_ERMODE_MASK             (3 << RTCEV_ERCONTROL_ERMODE_SHIFT)
#  define ERMODE0                               (0)       /* monitor/clocks disabled */
#  define ERMODE1                               (1)       /* 16Hz sample clock */
#  define ERMODE2                               (2)       /* 64Hz sample clock */
#  define ERMODE3                               (3)       /* 1000Hz sample clock */

/* RTCEV ERSTATUS - Monitor/Recorder Status Register */

#define RTCEV_ERSTATUS_EV0                      (1)       /* Bit 0: Event flag - channel 0 */
#define RTCEV_ERSTATUS_EV1                      (1 << 1)  /* Bit 1: Event flag - channel 1 */
#define RTCEV_ERSTATUS_EV2                      (1 << 2)  /* Bit 2: Event flag - channel 2 */
#define RTCEV_ERSTATUS_EV2                      (1 << 3)  /* Bit 3: GPReg async clear flag */
                                                          /* Bits 4-30: Reserved */
#define RTCEV_ERSTATUS_WAKEUP                   (1 << 31) /* Bit 31: Interrupt/Wakeup request flag */

/* RTCEV ERCOUNTERS - Monitor/Recorder Counters Register */

#define RTCEV_ERCOUNTER_COUNTER0_SHIFT          (0)       /* Bits 0-2: Value for event 0 */
#define RTCEV_ERCOUNTER_COUNTER0_MASK           (7 << RTCEV_ERCOUNTER_COUNTER0_SHIFT)
                                                          /* Bits 3-7: Reserved */
#define RTCEV_ERCOUNTER_COUNTER1_SHIFT          (8) i     /* Bits 8-10: Value for event 1 */
#define RTCEV_ERCOUNTER_COUNTER1_MASK           (7 << RTCEV_ERCOUNTER_COUNTER1_SHIFT)
                                                          /* Bits 11-15: Reserved */
#define RTCEV_ERCOUNTER_COUNTER2_SHIFT          (16)      /* Bits 16-18: Value for event 2 */
#define RTCEV_ERCOUNTER_COUNTER2_MASK           (7 << RTCEV_ERCOUNTER_COUNTER2_SHIFT)
                                                          /* Bits 19-31: Reserved */

/* RTCEV ERFIRSTSTAMP[0-2] - Monitor/Recorder First Stamp Registers */
/* RTCEV ERLASTSTAMP[0-2] - Monitor/Recorder Last Stamp Registers */

#define RTCEV_TIMESTAMP_SEC_SHIFT               (0)       /* Bits 0-5: Seconds value 0-59 */
#define RTCEV_TIMESTAMP_SEC_MASK                (0x3f << RTCEV_TIMESTAMP_SEC_SHIFT)
#define RTCEV_TIMESTAMP_MIN_SHIFT               (6)       /* Bits 6-11: Minutes value 0-59 */
#define RTCEV_TIMESTAMP_MIN_MASK                (0x3f << RTCEV_TIMESTAMP_MIN_SHIFT)
#define RTCEV_TIMESTAMP_HOUR_SHIFT              (12)      /* Bits 12-16: Hours value 0-23 */
#define RTCEV_TIMESTAMP_HOUR_MASK               (0x1f << RTCEV_TIMESTAMP_HOUR_SHIFT)
#define RTCEV_TIMESTAMP_DOY_SHIFT               (17)      /* Bits 17-25: Day of the year value 1-366 */
#define RTCEV_TIMESTAMP_DOY_MASK                (0x1ff << RTCEV_TIMESTAMP_DOY_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_RTCEVMR_H */
