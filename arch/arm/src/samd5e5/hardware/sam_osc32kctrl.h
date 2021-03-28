/****************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_osc32kctrl.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_OSC32KCTRL_H
#define __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_OSC32KCTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* OSC32KCTRL register offsets **********************************************/

#define SAM_OSC32KCTRL_INTENCLR_OFFSET   0x0000  /* Interrupt enable clear */
#define SAM_OSC32KCTRL_INTENSET_OFFSET   0x0004  /* Interrupt enable set */
#define SAM_OSC32KCTRL_INTFLAG_OFFSET    0x0008  /* Interrupt flag status and clear */
#define SAM_OSC32KCTRL_STATUS_OFFSET     0x000c  /* Status */
#define SAM_OSC32KCTRL_RTCCTRL_OFFSET    0x0010  /* RTC clock selection */
#define SAM_OSC32KCTRL_XOSC32K_OFFSET    0x0014  /* 32kHz external crystal oscillator control */
#define SAM_OSC32KCTRL_CFDCTRL_OFFSET    0x0016  /* Clock Failure Detector Control */
#define SAM_OSC32KCTRL_EVCTRL_OFFSET     0x0017  /* Event Control */
#define SAM_OSC32KCTRL_OSCULP32K_OFFSET  0x001c  /* 32kHz ultra low power internal oscillator control */

/* OSC32KCTRL register addresses ********************************************/

#define SAM_OSC32KCTRL_INTENCLR          (SAM_OSC32KCTRL_BASE + SAM_OSC32KCTRL_INTENCLR_OFFSET)
#define SAM_OSC32KCTRL_INTENSET          (SAM_OSC32KCTRL_BASE + SAM_OSC32KCTRL_INTENSET_OFFSET)
#define SAM_OSC32KCTRL_INTFLAG           (SAM_OSC32KCTRL_BASE + SAM_OSC32KCTRL_INTFLAG_OFFSET)
#define SAM_OSC32KCTRL_STATUS            (SAM_OSC32KCTRL_BASE + SAM_OSC32KCTRL_STATUS_OFFSET)
#define SAM_OSC32KCTRL_RTCCTRL           (SAM_OSC32KCTRL_BASE + SAM_OSC32KCTRL_RTCCTRL_OFFSET)
#define SAM_OSC32KCTRL_XOSC32K           (SAM_OSC32KCTRL_BASE + SAM_OSC32KCTRL_XOSC32K_OFFSET)
#define SAM_OSC32KCTRL_CFDCTRL           (SAM_OSC32KCTRL_BASE + SAM_OSC32KCTRL_CFDCTRL_OFFSET )
#define SAM_OSC32KCTRL_EVCTRL            (SAM_OSC32KCTRL_BASE + SAM_OSC32KCTRL_EVCTRL_OFFSET )
#define SAM_OSC32KCTRL_OSCULP32K         (SAM_OSC32KCTRL_BASE + SAM_OSC32KCTRL_OSCULP32K_OFFSET)

/* OSC32KCTRL register bit definitions **************************************/

/* Interrupt enable clear, Interrupt enable set, Interrupt flag status and
 * clear, and status registers.
 */

#define OSC32KCTRL_INT_XOSC32KRDY        (1 << 0)  /* Bit 0:  XOSC32K ready interrupt */
#define OSC32KCTRL_INT_XOSC32KFAIL       (1 << 2)  /* Bit 2:  Clock failure detector interrupt */
#define OSC32KCTRL_STATUS_XOSC32KSW      (1 << 3)  /* Bit 3:  XOSC32K Clock Switch */

#define OSC32KCTRL_INT_ALL               (0x00000005)

/* RTC clock selection */

#define OSC32KCTRL_RTCCTRL_RTCSEL_SHIFT  (0)       /* Bits 0-2: RTC clock source selection */
#define OSC32KCTRL_RTCCTRL_RTCSEL_MASK   (7 << OSC32KCTRL_RTCCTRL_RTCSEL_SHIFT)
#  define OSC32KCTRL_RTCCTRL_RTCSEL(n)   ((uint8_t)(n) << OSC32KCTRL_RTCCTRL_RTCSEL_SHIFT)
#  define OSC32KCTRL_RTCCTRL_RTCSEL_ULP1K    (0 << OSC32KCTRL_RTCCTRL_RTCSEL_SHIFT) /* 1.024KHz from 32HKz internal ULP oscillator */
#  define OSC32KCTRL_RTCCTRL_RTCSEL_ULP32K   (1 << OSC32KCTRL_RTCCTRL_RTCSEL_SHIFT) /* 32.768KHz from 32KHz internal ULP oscillator */
#  define OSC32KCTRL_RTCCTRL_RTCSEL_XOSC1K   (4 << OSC32KCTRL_RTCCTRL_RTCSEL_SHIFT) /* 1.024kHz from 32KHz external oscillator */
#  define OSC32KCTRL_RTCCTRL_RTCSEL_XOSC312K (5 << OSC32KCTRL_RTCCTRL_RTCSEL_SHIFT) /* 32.768KHz from 32KHz external crystal oscillator */

/* 32kHz external crystal oscillator control register */

#define OSC32KCTRL_XOSC32K_ENABLE        (1 << 1)  /* Bit 1:  Oscillator enable */
#define OSC32KCTRL_XOSC32K_XTALEN        (1 << 2)  /* Bit 2:  Crystal oscillator enable */
#define OSC32KCTRL_XOSC32K_EN32K         (1 << 3)  /* Bit 3:  32kHz Output enable */
#define OSC32KCTRL_XOSC32K_EN1K          (1 << 4)  /* Bit 4:  1kHz Output enable */
#define OSC32KCTRL_XOSC32K_RUNSTDBY      (1 << 6)  /* Bit 6:  Run in standby */
#define OSC32KCTRL_XOSC32K_ONDEMAND      (1 << 7)  /* Bit 7:  On demand control */
#define OSC32KCTRL_XOSC32K_STARTUP_SHIFT    (8)    /* Bits 8-10: Oscillator start-up time */
#define OSC32KCTRL_XOSC32K_STARTUP_MASK     (7 << OSC32KCTRL_XOSC32K_STARTUP_SHIFT)
#  define OSC32KCTRL_XOSC32K_STARTUP(n)     ((n) << OSC32KCTRL_XOSC32K_STARTUP_SHIFT)
#  define OSC32KCTRL_XOSC32K_STARTUP_63MS   (0 << OSC32KCTRL_XOSC32K_STARTUP_SHIFT) /* 62.592 msec */
#  define OSC32KCTRL_XOSC32K_STARTUP_125MS  (1 << OSC32KCTRL_XOSC32K_STARTUP_SHIFT) /* 125.092 msec */
#  define OSC32KCTRL_XOSC32K_STARTUP_500MS  (2 << OSC32KCTRL_XOSC32K_STARTUP_SHIFT) /* 500.092 msec */
#  define OSC32KCTRL_XOSC32K_STARTUP_1S     (3 << OSC32KCTRL_XOSC32K_STARTUP_SHIFT) /* 1000.0092 msec */
#  define OSC32KCTRL_XOSC32K_STARTUP_2S     (4 << OSC32KCTRL_XOSC32K_STARTUP_SHIFT) /* 2000.0092 msec */
#  define OSC32KCTRL_XOSC32K_STARTUP_4S     (5 << OSC32KCTRL_XOSC32K_STARTUP_SHIFT) /* 4000.092 msec */
#  define OSC32KCTRL_XOSC32K_STARTUP_8S     (6 << OSC32KCTRL_XOSC32K_STARTUP_SHIFT) /* 8000.0092 msec */

#define OSC32KCTRL_XOSC32K_WRTLOCK       (1 << 12) /* Bit 12: Write lock */
#define OSC32KCTRL_XOSC32K_GCM_SHIFT     (13)      /* Bits 13-14: Control Gain Mode */
#define OSC32KCTRL_XOSC32K_GCM_MASK      (3 << OSC32KCTRL_XOSC32K_GCM_SHIFT)
#  define OSC32KCTRL_XOSC32K_GCM_XT      (1 << OSC32KCTRL_XOSC32K_GCM_SHIFT) /* Standard mode */
#  define OSC32KCTRL_XOSC32K_GCM_HS      (2 << OSC32KCTRL_XOSC32K_GCM_SHIFT) /* High Speed mode */

/* Clock Failure Detector Control */

#define OSC32KCTRL_CFDCTRL_CFDEN         (1 << 0)  /* Bit 0:  Clock Failure Detector Enable */
#define OSC32KCTRL_CFDCTRL_SWBACK        (1 << 1)  /* Bit 1:  Clock Switch Back */
#define OSC32KCTRL_CFDCTRL_CFDPRESC      (1 << 2)  /* Bit 2:  Clock Failure Detector Prescaler */

/* Event Control */

#define OSC32KCTRL_EVCTRL_CFDEO          (1 << 0)  /* Bit 0:  Clock Failure Detector Event Out Enable */

/* 32kHz ultra low power internal oscillator control register */

#define OSC32KCTRL_OSCULP32K_EN32K       (1 << 0)  /* Bit 0:  32kHz Output Enable */
#define OSC32KCTRL_OSCULP32K_EN1K        (1 << 1)  /* Bit 1:  1kHz Output Enable */
#define OSC32KCTRL_OSCULP32K_CALIB_SHIFT (8)       /* Bits 8-13: Oscillator Calibration */
#define OSC32KCTRL_OSCULP32K_CALIB_MASK  (0x3f << OSC32KCTRL_OSCULP32K_CALIB_SHIFT)
#  define OSC32KCTRL_OSCULP32K_CALIB(n)  ((uint16_t)(n) << OSC32KCTRL_OSCULP32K_CALIB_SHIFT)
#define OSC32KCTRL_OSCULP32K_WRTLOCK     (1 << 15) /* Bit 15: Write Lock */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_OSC32KCTRL_H */
