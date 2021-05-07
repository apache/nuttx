/****************************************************************************
 * arch/arm/src/lpc43xx/hardware/lpc43_evntmntr.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_EVNTMNTR_H
#define __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_EVNTMNTR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define LPC43_EMR_CONTROL_OFFSET       0x0004 /* Event Monitor/Recorder Control register */
#define LPC43_EMR_STATUS_OFFSET        0x0000 /* Event Monitor/Recorder Status register */
#define LPC43_EMR_COUNTERS_OFFSET      0x0008 /* Event Monitor/Recorder Counters register */

#define LPC43_EMR_FIRSTSTAMP_OFFSET(n) (0x0010 + ((n) << 2))
#define LPC43_EMR_FIRSTSTAMP0_OFFSET   0x0010 /* Event Monitor/Recorder First Stamp register Ch0 */
#define LPC43_EMR_FIRSTSTAMP1_OFFSET   0x0014 /* Event Monitor/Recorder First Stamp register Ch1 */
#define LPC43_EMR_FIRSTSTAMP2_OFFSET   0x0018 /* Event Monitor/Recorder First Stamp register Ch2 */

#define LPC43_EMR_LASTSTAMP_OFFSET(n)  (0x0020 + ((n) << 2))
#define LPC43_EMR_LASTSTAMP0_OFFSET    0x0020 /* Event Monitor/Recorder Last Stamp register Ch0 */
#define LPC43_EMR_LASTSTAMP1_OFFSET    0x0024 /* Event Monitor/Recorder Last Stamp register Ch1 */
#define LPC43_EMR_LASTSTAMP2_OFFSET    0x0028 /* Event Monitor/Recorder Last Stamp register Ch2 */

/* Register Addresses *******************************************************/

#define LPC43_EMR_CONTROL              (LPC43_EVNTMNTR_BASE+LPC43_EMR_CONTROL_OFFSET)
#define LPC43_EMR_STATUS               (LPC43_EVNTMNTR_BASE+LPC43_EMR_STATUS_OFFSET)
#define LPC43_EMR_COUNTERS             (LPC43_EVNTMNTR_BASE+LPC43_EMR_COUNTERS_OFFSET)

#define LPC43_EMR_FIRSTSTAMP(n)        (LPC43_EVNTMNTR_BASE+LPC43_EMR_FIRSTSTAMP_OFFSET(n))
#define LPC43_EMR_FIRSTSTAMP0          (LPC43_EVNTMNTR_BASE+LPC43_EMR_FIRSTSTAMP0_OFFSET)
#define LPC43_EMR_FIRSTSTAMP1          (LPC43_EVNTMNTR_BASE+LPC43_EMR_FIRSTSTAMP1_OFFSET)
#define LPC43_EMR_FIRSTSTAMP2          (LPC43_EVNTMNTR_BASE+LPC43_EMR_FIRSTSTAMP2_OFFSET)

#define LPC43_EMR_LASTSTAMP(n)         (LPC43_EVNTMNTR_BASE+LPC43_EMR_LASTSTAMP_OFFSET(n))
#define LPC43_EMR_LASTSTAMP0           (LPC43_EVNTMNTR_BASE+LPC43_EMR_LASTSTAMP0_OFFSET)
#define LPC43_EMR_LASTSTAMP1           (LPC43_EVNTMNTR_BASE+LPC43_EMR_LASTSTAMP1_OFFSET)
#define LPC43_EMR_LASTSTAMP2           (LPC43_EVNTMNTR_BASE+LPC43_EMR_LASTSTAMP2_OFFSET)

/* Register Bit Definitions *************************************************/

/* Event Monitor/Recorder Control register */

#define EMR_CONTROL_INTWAKE_EN0        (1 << 0)  /* Bit 0:  Interrupt and wakeup enable Ch0 */
#define EMR_CONTROL_GPCLEAR_EN0        (1 << 1)  /* Bit 1:  Enables auto clearing of RTC GP regs Ch0 */
#define EMR_CONTROL_POL0               (1 << 2)  /* Bit 2:  Selects polarity of input pin WAKEUP0 */
#define EMR_CONTROL_EV0_INPUT_EN       (1 << 3)  /* Bit 3:  Event enable control Ch0 */
                                                 /* Bits 4-9: Reserved */
#define EMR_CONTROL_INTWAKE_EN1        (1 << 10) /* Bit 10: Interrupt and wakeup enable Ch1 */
#define EMR_CONTROL_GPCLEAR_EN1        (1 << 11) /* Bit 11: Enables auto clearing the RTC GP regs Ch1 */
#define EMR_CONTROL_POL1               (1 << 12) /* Bit 12: Selects polarity of input pin WAKEUP1 */
#define EMR_CONTROL_EV1_INPUT_EN       (1 << 13) /* Bit 13: Event enable control Ch1 */
                                                 /* Bits 14-19: Reserved */
#define EMR_CONTROL_INTWAKE_EN2        (1 << 20) /* Bit 20: Interrupt and wakeup enable Ch2 */
#define EMR_CONTROL_GPCLEAR_EN2        (1 << 21) /* Bit 21: Enables auto clearing of RTC GP regs Ch2 */
#define EMR_CONTROL_POL2               (1 << 22) /* Bit 22: Selects polarity of input pin WAKEUP2 */
#define EMR_CONTROL_EV2_INPUT_EN       (1 << 23) /* Bit 23: Event enable control Ch2 */
                                                 /* Bits 24-29: Reserved */
#define EMR_CONTROL_ERMODE_SHIFT       (30)      /* Bits 30-31: Enable Event Monitor/Recorder */
#define EMR_CONTROL_ERMODE_MASK        (3 << EMR_CONTROL_ERMODE_SHIFT)
#  define EMR_CONTROL_ERMODE_DISABLE   (0 << EMR_CONTROL_ERMODE_SHIFT) /* Disable Event Monitor/Recorder clocks */
#  define EMR_CONTROL_ERMODE_16Hz      (1 << EMR_CONTROL_ERMODE_SHIFT) /* 16 Hz sample clock */
#  define EMR_CONTROL_ERMODE_64Hz      (2 << EMR_CONTROL_ERMODE_SHIFT) /* 64 Hz sample clock */
#  define EMR_CONTROL_ERMODE_1KHz      (3 << EMR_CONTROL_ERMODE_SHIFT) /* 1 kHz sample clock */

/* Event Monitor/Recorder Status register */

#define EMR_STATUS_EV0                 (1 << 0)  /* Bit 0:  Channel0 event flag (WAKEUP0 pin) */
#define EMR_STATUS_EV1                 (1 << 1)  /* Bit 1:  Channel1 Event flag (WAKEUP1 pin) */
#define EMR_STATUS_EV2                 (1 << 2)  /* Bit 2:  Channel2 Event flag (WAKEUP2 pin) */
#define EMR_STATUS_GPCLR               (1 << 3)  /* Bit 3:  General purpose register asynchronous clear flag */
                                                 /* Bits 4-30: Reserved */
#define EMR_STATUS_WAKEUP              (1 << 31) /* Bit 31:  WAKEUP Interrupt/wakeup request flag */

/* Event Monitor/Recorder Counters register */

#define EMR_COUNTERS_COUNTER0_SHIFT    (0)       /* Bits 0-2: Value of the counter for Event 0 */
#define EMR_COUNTERS_COUNTER0_MASK     (7 << EMR_COUNTERS_COUNTER0_SHIFT)
                                                 /* Bits 3-7: Reserved */
#define EMR_COUNTERS_COUNTER1_SHIFT    (8)       /* Bits 8-10: Value of the counter for event 1 */
#define EMR_COUNTERS_COUNTER1_MASK     (8 << EMR_COUNTERS_COUNTER1_SHIFT)
                                                 /* Bits 11-15: Reserved */
#define EMR_COUNTERS_COUNTER2_SHIFT    (16)      /* Bits 16-18: Value of the counter for event 2 */
#define EMR_COUNTERS_COUNTER2_MASK     (7 << EMR_COUNTERS_COUNTER2_SHIFT)
                                                 /* Bits 19-31: Reserved */

/* Event Monitor/Recorder First/Last Stamp registers */

#define EMR_STAMP_SEC_SHIFT            (0)       /* Bits 0-5: Seconds value 0-59 */
#define EMR_STAMP_SEC_MASK             (63 << EMR_STAMP_SEC_SHIFT)
#define EMR_STAMP_MIN_SHIFT            (6)       /* Bits 6-11: Minutes value 0-59 */
#define EMR_STAMP_MIN_MASK             (63 << EMR_STAMP_MIN_SHIFT)
#define EMR_STAMP_HOUR_SHIFT           (12)      /* Bits 12-16: Hours value 0-23 */
#define EMR_STAMP_HOUR_MASK            (31 << EMR_STAMP_HOUR_SHIFT)
#define EMR_STAMP_DOY_SHIFT            (17)      /* Bits 17-25: Day of Year value1-366 */
#define EMR_STAMP_DOY_MASK             (511 << EMR_STAMP_DOY_SHIFT)
                                                 /* Bits 26-31: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_EVNTMNTR_H */
