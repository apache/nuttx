/****************************************************************************
 * arch/xtensa/src/esp32s3/hardware/esp32s3_rtc_io.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_RTC_IO_H
#define __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_RTC_IO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32s3_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTCIO_RTC_GPIO_OUT_REG register
 * RTC GPIO output register
 */

#define RTCIO_RTC_GPIO_OUT_REG (DR_REG_RTCIO_BASE + 0x0)

/* RTCIO_GPIO_OUT_DATA : R/W; bitpos: [31:10]; default: 0;
 * GPIO0 ~ 21 output register. Bit10 corresponds to GPIO0, bit11 corre-
 * sponds to GPIO1, etc.
 */

#define RTCIO_GPIO_OUT_DATA    0x003FFFFF
#define RTCIO_GPIO_OUT_DATA_M  (RTCIO_GPIO_OUT_DATA_V << RTCIO_GPIO_OUT_DATA_S)
#define RTCIO_GPIO_OUT_DATA_V  0x003FFFFF
#define RTCIO_GPIO_OUT_DATA_S  10

/* RTCIO_RTC_GPIO_OUT_W1TS_REG register
 * RTC GPIO output bit set register
 */

#define RTCIO_RTC_GPIO_OUT_W1TS_REG (DR_REG_RTCIO_BASE + 0x4)

/* RTCIO_GPIO_OUT_DATA_W1TS : WO; bitpos: [31:10]; default: 0;
 * GPIO0 ~ 21 output set register. If the value 1 is written to a bit here,
 * the corresponding bit in RTCIO_RTC_GPIO_OUT_REG will be set to 1.
 * Recommended operation: use this register to set RTCIO_RTC_GPIO_OUT_REG.
 */

#define RTCIO_GPIO_OUT_DATA_W1TS    0x003FFFFF
#define RTCIO_GPIO_OUT_DATA_W1TS_M  (RTCIO_GPIO_OUT_DATA_W1TS_V << RTCIO_GPIO_OUT_DATA_W1TS_S)
#define RTCIO_GPIO_OUT_DATA_W1TS_V  0x003FFFFF
#define RTCIO_GPIO_OUT_DATA_W1TS_S  10

/* RTCIO_RTC_GPIO_OUT_W1TC_REG register
 * RTC GPIO output bit clear register
 */

#define RTCIO_RTC_GPIO_OUT_W1TC_REG (DR_REG_RTCIO_BASE + 0x8)

/* RTCIO_GPIO_OUT_DATA_W1TC : WO; bitpos: [31:10]; default: 0;
 * GPIO0 ~ 21 output clear register. If the value 1 is written to a bit
 * here, the corresponding bit in RTCIO_RTC_GPIO_OUT_REG will be cleared.
 * Recommended operation: use this register to clear RTCIO_RTC_GPIO_OUT_REG.
 */

#define RTCIO_GPIO_OUT_DATA_W1TC    0x003FFFFF
#define RTCIO_GPIO_OUT_DATA_W1TC_M  (RTCIO_GPIO_OUT_DATA_W1TC_V << RTCIO_GPIO_OUT_DATA_W1TC_S)
#define RTCIO_GPIO_OUT_DATA_W1TC_V  0x003FFFFF
#define RTCIO_GPIO_OUT_DATA_W1TC_S  10

/* RTCIO_RTC_GPIO_ENABLE_REG register
 * RTC GPIO output enable register
 */

#define RTCIO_RTC_GPIO_ENABLE_REG (DR_REG_RTCIO_BASE + 0xc)

/* RTCIO_REG_RTCIO_REG_GPIO_ENABLE : R/W; bitpos: [31:10]; default: 0;
 * GPIO0 ~ 21 output enable. Bit10 corresponds to GPIO0, bit11 corresponds
 * to GPIO1, etc. If the bit is set to 1, it means this GPIO pad is output.
 */

#define RTCIO_REG_RTCIO_REG_GPIO_ENABLE    0x003FFFFF
#define RTCIO_REG_RTCIO_REG_GPIO_ENABLE_M  (RTCIO_REG_RTCIO_REG_GPIO_ENABLE_V << RTCIO_REG_RTCIO_REG_GPIO_ENABLE_S)
#define RTCIO_REG_RTCIO_REG_GPIO_ENABLE_V  0x003FFFFF
#define RTCIO_REG_RTCIO_REG_GPIO_ENABLE_S  10

/* RTCIO_RTC_GPIO_ENABLE_W1TS_REG register
 * RTC GPIO output enable bit set register
 */

#define RTCIO_RTC_GPIO_ENABLE_W1TS_REG (DR_REG_RTCIO_BASE + 0x10)

/* RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TS : WO; bitpos: [31:10]; default: 0;
 * GPIO0 ~ 21 output enable set register. If the value 1 is written to a bit
 * here, the corresponding bit in RTCIO_RTC_GPIO_ENABLE_REG will be set to
 * 1. Recommended operation: use this register to set
 * RTCIO_RTC_GPIO_ENABLE_REG.
 */

#define RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TS    0x003FFFFF
#define RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TS_M  (RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TS_V << RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TS_S)
#define RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TS_V  0x003FFFFF
#define RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TS_S  10

/* RTCIO_RTC_GPIO_ENABLE_W1TC_REG register
 * RTC GPIO output enable bit clear register
 */

#define RTCIO_RTC_GPIO_ENABLE_W1TC_REG (DR_REG_RTCIO_BASE + 0x14)

/* RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TC : WO; bitpos: [31:10]; default: 0;
 * GPIO0 ~ 21 output enable clear register. If the value 1 is written to a
 * bit here, the corresponding bit in RTCIO_RTC_GPIO_ENABLE_REG will be
 * cleared. Recom- mended operation: use this register to clear
 * RTCIO_RTC_GPIO_ENABLE_REG.
 */

#define RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TC    0x003FFFFF
#define RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TC_M  (RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TC_V << RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TC_S)
#define RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TC_V  0x003FFFFF
#define RTCIO_REG_RTCIO_REG_GPIO_ENABLE_W1TC_S  10

/* RTCIO_RTC_GPIO_STATUS_REG register
 * RTC GPIO interrupt status register
 */

#define RTCIO_RTC_GPIO_STATUS_REG (DR_REG_RTCIO_BASE + 0x18)

/* RTCIO_GPIO_STATUS_INT : R/W; bitpos: [31:10]; default: 0;
 * GPIO0 ~ 21 interrupt status register. Bit10 corresponds to GPIO0, bit11
 * corresponds to GPIO1, etc. This register should be used together with RT-
 * CIO_RTC_GPIO_PINn_INT_TYPE in RTCIO_RTC_GPIO_PINn_REG. 0: no interrupt;
 * 1: corresponding interrupt.
 */

#define RTCIO_GPIO_STATUS_INT    0x003FFFFF
#define RTCIO_GPIO_STATUS_INT_M  (RTCIO_GPIO_STATUS_INT_V << RTCIO_GPIO_STATUS_INT_S)
#define RTCIO_GPIO_STATUS_INT_V  0x003FFFFF
#define RTCIO_GPIO_STATUS_INT_S  10

/* RTCIO_RTC_GPIO_STATUS_W1TS_REG register
 * RTC GPIO interrupt status bit set register
 */

#define RTCIO_RTC_GPIO_STATUS_W1TS_REG (DR_REG_RTCIO_BASE + 0x1c)

/* RTCIO_GPIO_STATUS_INT_W1TS : WO; bitpos: [31:10]; default: 0;
 * GPIO0 ~ 21 interrupt set register. If the value 1 is written to a bit
 * here, the corresponding bit in RTCIO_GPIO_STATUS_INT will be set to 1.
 * Recommended operation: use this register to set RTCIO_GPIO_STATUS_INT.
 */

#define RTCIO_GPIO_STATUS_INT_W1TS    0x003FFFFF
#define RTCIO_GPIO_STATUS_INT_W1TS_M  (RTCIO_GPIO_STATUS_INT_W1TS_V << RTCIO_GPIO_STATUS_INT_W1TS_S)
#define RTCIO_GPIO_STATUS_INT_W1TS_V  0x003FFFFF
#define RTCIO_GPIO_STATUS_INT_W1TS_S  10

/* RTCIO_RTC_GPIO_STATUS_W1TC_REG register
 * RTC GPIO interrupt status bit clear register
 */

#define RTCIO_RTC_GPIO_STATUS_W1TC_REG (DR_REG_RTCIO_BASE + 0x20)

/* RTCIO_GPIO_STATUS_INT_W1TC : WO; bitpos: [31:10]; default: 0;
 * GPIO0 ~ 21 interrupt clear register. If the value 1 is written to a bit
 * here, the corresponding bit in RTCIO_GPIO_STATUS_INT will be cleared.
 * Recommended operation: use this register to clear RTCIO_GPIO_STATUS_INT.
 */

#define RTCIO_GPIO_STATUS_INT_W1TC    0x003FFFFF
#define RTCIO_GPIO_STATUS_INT_W1TC_M  (RTCIO_GPIO_STATUS_INT_W1TC_V << RTCIO_GPIO_STATUS_INT_W1TC_S)
#define RTCIO_GPIO_STATUS_INT_W1TC_V  0x003FFFFF
#define RTCIO_GPIO_STATUS_INT_W1TC_S  10

/* RTCIO_RTC_GPIO_IN_REG register
 * RTC GPIO input register
 */

#define RTCIO_RTC_GPIO_IN_REG (DR_REG_RTCIO_BASE + 0x24)

/* RTCIO_GPIO_IN_NEXT : RO; bitpos: [31:10]; default: 0;
 * GPIO0 ~ 21 input value. Bit10 corresponds to GPIO0, bit11 corresponds to
 * GPIO1, etc. Each bit represents a pad input value, 1 for high level, and
 * 0 for low level.
 */

#define RTCIO_GPIO_IN_NEXT    0x003FFFFF
#define RTCIO_GPIO_IN_NEXT_M  (RTCIO_GPIO_IN_NEXT_V << RTCIO_GPIO_IN_NEXT_S)
#define RTCIO_GPIO_IN_NEXT_V  0x003FFFFF
#define RTCIO_GPIO_IN_NEXT_S  10

/* RTCIO_RTC_GPIO_PIN0_REG register
 * RTC configuration for pin 0
 */

#define RTCIO_RTC_GPIO_PIN0_REG (DR_REG_RTCIO_BASE + 0x28)

/* RTCIO_GPIO_PIN0_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN0_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN0_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN0_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN0_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN0_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN0_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN0_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN0_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN0_INT_TYPE_M  (RTCIO_GPIO_PIN0_INT_TYPE_V << RTCIO_GPIO_PIN0_INT_TYPE_S)
#define RTCIO_GPIO_PIN0_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN0_INT_TYPE_S  7

/* RTCIO_GPIO_PIN0_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN0_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN0_PAD_DRIVER_M  (RTCIO_GPIO_PIN0_PAD_DRIVER_V << RTCIO_GPIO_PIN0_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN0_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN0_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN1_REG register
 * RTC configuration for pin 1
 */

#define RTCIO_RTC_GPIO_PIN1_REG (DR_REG_RTCIO_BASE + 0x2c)

/* RTCIO_GPIO_PIN1_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN1_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN1_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN1_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN1_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN1_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN1_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN1_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN1_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN1_INT_TYPE_M  (RTCIO_GPIO_PIN1_INT_TYPE_V << RTCIO_GPIO_PIN1_INT_TYPE_S)
#define RTCIO_GPIO_PIN1_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN1_INT_TYPE_S  7

/* RTCIO_GPIO_PIN1_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN1_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN1_PAD_DRIVER_M  (RTCIO_GPIO_PIN1_PAD_DRIVER_V << RTCIO_GPIO_PIN1_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN1_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN1_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN2_REG register
 * RTC configuration for pin 2
 */

#define RTCIO_RTC_GPIO_PIN2_REG (DR_REG_RTCIO_BASE + 0x30)

/* RTCIO_GPIO_PIN2_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN2_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN2_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN2_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN2_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN2_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN2_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN2_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN2_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN2_INT_TYPE_M  (RTCIO_GPIO_PIN2_INT_TYPE_V << RTCIO_GPIO_PIN2_INT_TYPE_S)
#define RTCIO_GPIO_PIN2_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN2_INT_TYPE_S  7

/* RTCIO_GPIO_PIN2_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN2_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN2_PAD_DRIVER_M  (RTCIO_GPIO_PIN2_PAD_DRIVER_V << RTCIO_GPIO_PIN2_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN2_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN2_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN3_REG register
 * RTC configuration for pin 3
 */

#define RTCIO_RTC_GPIO_PIN3_REG (DR_REG_RTCIO_BASE + 0x34)

/* RTCIO_GPIO_PIN3_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN3_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN3_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN3_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN3_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN3_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN3_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN3_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN3_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN3_INT_TYPE_M  (RTCIO_GPIO_PIN3_INT_TYPE_V << RTCIO_GPIO_PIN3_INT_TYPE_S)
#define RTCIO_GPIO_PIN3_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN3_INT_TYPE_S  7

/* RTCIO_GPIO_PIN3_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN3_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN3_PAD_DRIVER_M  (RTCIO_GPIO_PIN3_PAD_DRIVER_V << RTCIO_GPIO_PIN3_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN3_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN3_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN4_REG register
 * RTC configuration for pin 4
 */

#define RTCIO_RTC_GPIO_PIN4_REG (DR_REG_RTCIO_BASE + 0x38)

/* RTCIO_GPIO_PIN4_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN4_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN4_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN4_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN4_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN4_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN4_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN4_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN4_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN4_INT_TYPE_M  (RTCIO_GPIO_PIN4_INT_TYPE_V << RTCIO_GPIO_PIN4_INT_TYPE_S)
#define RTCIO_GPIO_PIN4_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN4_INT_TYPE_S  7

/* RTCIO_GPIO_PIN4_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN4_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN4_PAD_DRIVER_M  (RTCIO_GPIO_PIN4_PAD_DRIVER_V << RTCIO_GPIO_PIN4_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN4_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN4_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN5_REG register
 * RTC configuration for pin 5
 */

#define RTCIO_RTC_GPIO_PIN5_REG (DR_REG_RTCIO_BASE + 0x3c)

/* RTCIO_GPIO_PIN5_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN5_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN5_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN5_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN5_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN5_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN5_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN5_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN5_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN5_INT_TYPE_M  (RTCIO_GPIO_PIN5_INT_TYPE_V << RTCIO_GPIO_PIN5_INT_TYPE_S)
#define RTCIO_GPIO_PIN5_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN5_INT_TYPE_S  7

/* RTCIO_GPIO_PIN5_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN5_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN5_PAD_DRIVER_M  (RTCIO_GPIO_PIN5_PAD_DRIVER_V << RTCIO_GPIO_PIN5_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN5_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN5_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN6_REG register
 * RTC configuration for pin 6
 */

#define RTCIO_RTC_GPIO_PIN6_REG (DR_REG_RTCIO_BASE + 0x40)

/* RTCIO_GPIO_PIN6_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN6_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN6_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN6_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN6_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN6_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN6_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN6_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN6_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN6_INT_TYPE_M  (RTCIO_GPIO_PIN6_INT_TYPE_V << RTCIO_GPIO_PIN6_INT_TYPE_S)
#define RTCIO_GPIO_PIN6_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN6_INT_TYPE_S  7

/* RTCIO_GPIO_PIN6_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN6_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN6_PAD_DRIVER_M  (RTCIO_GPIO_PIN6_PAD_DRIVER_V << RTCIO_GPIO_PIN6_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN6_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN6_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN7_REG register
 * RTC configuration for pin 7
 */

#define RTCIO_RTC_GPIO_PIN7_REG (DR_REG_RTCIO_BASE + 0x44)

/* RTCIO_GPIO_PIN7_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN7_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN7_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN7_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN7_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN7_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN7_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN7_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN7_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN7_INT_TYPE_M  (RTCIO_GPIO_PIN7_INT_TYPE_V << RTCIO_GPIO_PIN7_INT_TYPE_S)
#define RTCIO_GPIO_PIN7_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN7_INT_TYPE_S  7

/* RTCIO_GPIO_PIN7_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN7_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN7_PAD_DRIVER_M  (RTCIO_GPIO_PIN7_PAD_DRIVER_V << RTCIO_GPIO_PIN7_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN7_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN7_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN8_REG register
 * RTC configuration for pin 8
 */

#define RTCIO_RTC_GPIO_PIN8_REG (DR_REG_RTCIO_BASE + 0x48)

/* RTCIO_GPIO_PIN8_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN8_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN8_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN8_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN8_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN8_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN8_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN8_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN8_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN8_INT_TYPE_M  (RTCIO_GPIO_PIN8_INT_TYPE_V << RTCIO_GPIO_PIN8_INT_TYPE_S)
#define RTCIO_GPIO_PIN8_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN8_INT_TYPE_S  7

/* RTCIO_GPIO_PIN8_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN8_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN8_PAD_DRIVER_M  (RTCIO_GPIO_PIN8_PAD_DRIVER_V << RTCIO_GPIO_PIN8_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN8_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN8_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN9_REG register
 * RTC configuration for pin 9
 */

#define RTCIO_RTC_GPIO_PIN9_REG (DR_REG_RTCIO_BASE + 0x4c)

/* RTCIO_GPIO_PIN9_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN9_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN9_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN9_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN9_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN9_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN9_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN9_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN9_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN9_INT_TYPE_M  (RTCIO_GPIO_PIN9_INT_TYPE_V << RTCIO_GPIO_PIN9_INT_TYPE_S)
#define RTCIO_GPIO_PIN9_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN9_INT_TYPE_S  7

/* RTCIO_GPIO_PIN9_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN9_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN9_PAD_DRIVER_M  (RTCIO_GPIO_PIN9_PAD_DRIVER_V << RTCIO_GPIO_PIN9_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN9_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN9_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN10_REG register
 * RTC configuration for pin 10
 */

#define RTCIO_RTC_GPIO_PIN10_REG (DR_REG_RTCIO_BASE + 0x50)

/* RTCIO_GPIO_PIN10_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN10_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN10_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN10_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN10_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN10_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN10_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN10_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN10_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN10_INT_TYPE_M  (RTCIO_GPIO_PIN10_INT_TYPE_V << RTCIO_GPIO_PIN10_INT_TYPE_S)
#define RTCIO_GPIO_PIN10_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN10_INT_TYPE_S  7

/* RTCIO_GPIO_PIN10_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN10_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN10_PAD_DRIVER_M  (RTCIO_GPIO_PIN10_PAD_DRIVER_V << RTCIO_GPIO_PIN10_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN10_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN10_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN11_REG register
 * RTC configuration for pin 11
 */

#define RTCIO_RTC_GPIO_PIN11_REG (DR_REG_RTCIO_BASE + 0x54)

/* RTCIO_GPIO_PIN11_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN11_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN11_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN11_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN11_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN11_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN11_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN11_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN11_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN11_INT_TYPE_M  (RTCIO_GPIO_PIN11_INT_TYPE_V << RTCIO_GPIO_PIN11_INT_TYPE_S)
#define RTCIO_GPIO_PIN11_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN11_INT_TYPE_S  7

/* RTCIO_GPIO_PIN11_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN11_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN11_PAD_DRIVER_M  (RTCIO_GPIO_PIN11_PAD_DRIVER_V << RTCIO_GPIO_PIN11_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN11_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN11_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN12_REG register
 * RTC configuration for pin 12
 */

#define RTCIO_RTC_GPIO_PIN12_REG (DR_REG_RTCIO_BASE + 0x58)

/* RTCIO_GPIO_PIN12_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN12_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN12_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN12_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN12_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN12_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN12_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN12_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN12_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN12_INT_TYPE_M  (RTCIO_GPIO_PIN12_INT_TYPE_V << RTCIO_GPIO_PIN12_INT_TYPE_S)
#define RTCIO_GPIO_PIN12_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN12_INT_TYPE_S  7

/* RTCIO_GPIO_PIN12_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN12_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN12_PAD_DRIVER_M  (RTCIO_GPIO_PIN12_PAD_DRIVER_V << RTCIO_GPIO_PIN12_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN12_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN12_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN13_REG register
 * RTC configuration for pin 13
 */

#define RTCIO_RTC_GPIO_PIN13_REG (DR_REG_RTCIO_BASE + 0x5c)

/* RTCIO_GPIO_PIN13_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN13_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN13_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN13_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN13_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN13_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN13_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN13_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN13_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN13_INT_TYPE_M  (RTCIO_GPIO_PIN13_INT_TYPE_V << RTCIO_GPIO_PIN13_INT_TYPE_S)
#define RTCIO_GPIO_PIN13_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN13_INT_TYPE_S  7

/* RTCIO_GPIO_PIN13_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN13_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN13_PAD_DRIVER_M  (RTCIO_GPIO_PIN13_PAD_DRIVER_V << RTCIO_GPIO_PIN13_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN13_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN13_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN14_REG register
 * RTC configuration for pin 14
 */

#define RTCIO_RTC_GPIO_PIN14_REG (DR_REG_RTCIO_BASE + 0x60)

/* RTCIO_GPIO_PIN14_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN14_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN14_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN14_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN14_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN14_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN14_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN14_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN14_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN14_INT_TYPE_M  (RTCIO_GPIO_PIN14_INT_TYPE_V << RTCIO_GPIO_PIN14_INT_TYPE_S)
#define RTCIO_GPIO_PIN14_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN14_INT_TYPE_S  7

/* RTCIO_GPIO_PIN14_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN14_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN14_PAD_DRIVER_M  (RTCIO_GPIO_PIN14_PAD_DRIVER_V << RTCIO_GPIO_PIN14_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN14_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN14_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN15_REG register
 * RTC configuration for pin 15
 */

#define RTCIO_RTC_GPIO_PIN15_REG (DR_REG_RTCIO_BASE + 0x64)

/* RTCIO_GPIO_PIN15_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN15_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN15_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN15_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN15_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN15_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN15_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN15_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN15_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN15_INT_TYPE_M  (RTCIO_GPIO_PIN15_INT_TYPE_V << RTCIO_GPIO_PIN15_INT_TYPE_S)
#define RTCIO_GPIO_PIN15_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN15_INT_TYPE_S  7

/* RTCIO_GPIO_PIN15_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN15_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN15_PAD_DRIVER_M  (RTCIO_GPIO_PIN15_PAD_DRIVER_V << RTCIO_GPIO_PIN15_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN15_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN15_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN16_REG register
 * RTC configuration for pin 16
 */

#define RTCIO_RTC_GPIO_PIN16_REG (DR_REG_RTCIO_BASE + 0x68)

/* RTCIO_GPIO_PIN16_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN16_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN16_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN16_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN16_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN16_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN16_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN16_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN16_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN16_INT_TYPE_M  (RTCIO_GPIO_PIN16_INT_TYPE_V << RTCIO_GPIO_PIN16_INT_TYPE_S)
#define RTCIO_GPIO_PIN16_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN16_INT_TYPE_S  7

/* RTCIO_GPIO_PIN16_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN16_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN16_PAD_DRIVER_M  (RTCIO_GPIO_PIN16_PAD_DRIVER_V << RTCIO_GPIO_PIN16_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN16_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN16_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN17_REG register
 * RTC configuration for pin 17
 */

#define RTCIO_RTC_GPIO_PIN17_REG (DR_REG_RTCIO_BASE + 0x6c)

/* RTCIO_GPIO_PIN17_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN17_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN17_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN17_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN17_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN17_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN17_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN17_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN17_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN17_INT_TYPE_M  (RTCIO_GPIO_PIN17_INT_TYPE_V << RTCIO_GPIO_PIN17_INT_TYPE_S)
#define RTCIO_GPIO_PIN17_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN17_INT_TYPE_S  7

/* RTCIO_GPIO_PIN17_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN17_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN17_PAD_DRIVER_M  (RTCIO_GPIO_PIN17_PAD_DRIVER_V << RTCIO_GPIO_PIN17_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN17_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN17_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN18_REG register
 * RTC configuration for pin 18
 */

#define RTCIO_RTC_GPIO_PIN18_REG (DR_REG_RTCIO_BASE + 0x70)

/* RTCIO_GPIO_PIN18_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN18_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN18_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN18_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN18_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN18_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN18_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN18_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN18_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN18_INT_TYPE_M  (RTCIO_GPIO_PIN18_INT_TYPE_V << RTCIO_GPIO_PIN18_INT_TYPE_S)
#define RTCIO_GPIO_PIN18_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN18_INT_TYPE_S  7

/* RTCIO_GPIO_PIN18_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN18_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN18_PAD_DRIVER_M  (RTCIO_GPIO_PIN18_PAD_DRIVER_V << RTCIO_GPIO_PIN18_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN18_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN18_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN19_REG register
 * RTC configuration for pin 19
 */

#define RTCIO_RTC_GPIO_PIN19_REG (DR_REG_RTCIO_BASE + 0x74)

/* RTCIO_GPIO_PIN19_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN19_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN19_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN19_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN19_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN19_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN19_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN19_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN19_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN19_INT_TYPE_M  (RTCIO_GPIO_PIN19_INT_TYPE_V << RTCIO_GPIO_PIN19_INT_TYPE_S)
#define RTCIO_GPIO_PIN19_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN19_INT_TYPE_S  7

/* RTCIO_GPIO_PIN19_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN19_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN19_PAD_DRIVER_M  (RTCIO_GPIO_PIN19_PAD_DRIVER_V << RTCIO_GPIO_PIN19_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN19_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN19_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN20_REG register
 * RTC configuration for pin 20
 */

#define RTCIO_RTC_GPIO_PIN20_REG (DR_REG_RTCIO_BASE + 0x78)

/* RTCIO_GPIO_PIN20_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN20_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN20_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN20_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN20_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN20_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN20_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN20_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN20_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN20_INT_TYPE_M  (RTCIO_GPIO_PIN20_INT_TYPE_V << RTCIO_GPIO_PIN20_INT_TYPE_S)
#define RTCIO_GPIO_PIN20_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN20_INT_TYPE_S  7

/* RTCIO_GPIO_PIN20_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN20_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN20_PAD_DRIVER_M  (RTCIO_GPIO_PIN20_PAD_DRIVER_V << RTCIO_GPIO_PIN20_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN20_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN20_PAD_DRIVER_S  2

/* RTCIO_RTC_GPIO_PIN21_REG register
 * RTC configuration for pin 21
 */

#define RTCIO_RTC_GPIO_PIN21_REG (DR_REG_RTCIO_BASE + 0x7c)

/* RTCIO_GPIO_PIN21_WAKEUP_ENABLE : R/W; bitpos: [10]; default: 0;
 * GPIO wake-up enable. This will only wake up ESP32-S2 from Light-sleep.
 */

#define RTCIO_GPIO_PIN21_WAKEUP_ENABLE    (BIT(10))
#define RTCIO_GPIO_PIN21_WAKEUP_ENABLE_M  (RTCIO_GPIO_PIN21_WAKEUP_ENABLE_V << RTCIO_GPIO_PIN21_WAKEUP_ENABLE_S)
#define RTCIO_GPIO_PIN21_WAKEUP_ENABLE_V  0x00000001
#define RTCIO_GPIO_PIN21_WAKEUP_ENABLE_S  10

/* RTCIO_GPIO_PIN21_INT_TYPE : R/W; bitpos: [9:7]; default: 0;
 * GPIO interrupt type selection. 0: GPIO interrupt disabled; 1: rising edge
 * trigger; 2: falling edge trigger; 3: any edge trigger; 4: low level
 * trigger; 5: high level trigger.
 */

#define RTCIO_GPIO_PIN21_INT_TYPE    0x00000007
#define RTCIO_GPIO_PIN21_INT_TYPE_M  (RTCIO_GPIO_PIN21_INT_TYPE_V << RTCIO_GPIO_PIN21_INT_TYPE_S)
#define RTCIO_GPIO_PIN21_INT_TYPE_V  0x00000007
#define RTCIO_GPIO_PIN21_INT_TYPE_S  7

/* RTCIO_GPIO_PIN21_PAD_DRIVER : R/W; bitpos: [2]; default: 0;
 * Pad driver selection. 0: normal output; 1: open drain.
 */

#define RTCIO_GPIO_PIN21_PAD_DRIVER    (BIT(2))
#define RTCIO_GPIO_PIN21_PAD_DRIVER_M  (RTCIO_GPIO_PIN21_PAD_DRIVER_V << RTCIO_GPIO_PIN21_PAD_DRIVER_S)
#define RTCIO_GPIO_PIN21_PAD_DRIVER_V  0x00000001
#define RTCIO_GPIO_PIN21_PAD_DRIVER_S  2

/* RTCIO_RTC_DEBUG_SEL_REG register
 * RTC debug select register
 */

#define RTCIO_RTC_DEBUG_SEL_REG (DR_REG_RTCIO_BASE + 0x80)

/* RTCIO_RTC_DEBUG_12M_NO_GATING : R/W; bitpos: [25]; default: 0; */

#define RTCIO_RTC_DEBUG_12M_NO_GATING    (BIT(25))
#define RTCIO_RTC_DEBUG_12M_NO_GATING_M  (RTCIO_RTC_DEBUG_12M_NO_GATING_V << RTCIO_RTC_DEBUG_12M_NO_GATING_S)
#define RTCIO_RTC_DEBUG_12M_NO_GATING_V  0x00000001
#define RTCIO_RTC_DEBUG_12M_NO_GATING_S  25

/* RTCIO_RTC_DEBUG_SEL4 : R/W; bitpos: [24:20]; default: 0; */

#define RTCIO_RTC_DEBUG_SEL4    0x0000001F
#define RTCIO_RTC_DEBUG_SEL4_M  (RTCIO_RTC_DEBUG_SEL4_V << RTCIO_RTC_DEBUG_SEL4_S)
#define RTCIO_RTC_DEBUG_SEL4_V  0x0000001F
#define RTCIO_RTC_DEBUG_SEL4_S  20

/* RTCIO_RTC_DEBUG_SEL3 : R/W; bitpos: [19:15]; default: 0; */

#define RTCIO_RTC_DEBUG_SEL3    0x0000001F
#define RTCIO_RTC_DEBUG_SEL3_M  (RTCIO_RTC_DEBUG_SEL3_V << RTCIO_RTC_DEBUG_SEL3_S)
#define RTCIO_RTC_DEBUG_SEL3_V  0x0000001F
#define RTCIO_RTC_DEBUG_SEL3_S  15

/* RTCIO_RTC_DEBUG_SEL2 : R/W; bitpos: [14:10]; default: 0; */

#define RTCIO_RTC_DEBUG_SEL2    0x0000001F
#define RTCIO_RTC_DEBUG_SEL2_M  (RTCIO_RTC_DEBUG_SEL2_V << RTCIO_RTC_DEBUG_SEL2_S)
#define RTCIO_RTC_DEBUG_SEL2_V  0x0000001F
#define RTCIO_RTC_DEBUG_SEL2_S  10

/* RTCIO_RTC_DEBUG_SEL1 : R/W; bitpos: [9:5]; default: 0; */

#define RTCIO_RTC_DEBUG_SEL1    0x0000001F
#define RTCIO_RTC_DEBUG_SEL1_M  (RTCIO_RTC_DEBUG_SEL1_V << RTCIO_RTC_DEBUG_SEL1_S)
#define RTCIO_RTC_DEBUG_SEL1_V  0x0000001F
#define RTCIO_RTC_DEBUG_SEL1_S  5

/* RTCIO_RTC_DEBUG_SEL0 : R/W; bitpos: [4:0]; default: 0; */

#define RTCIO_RTC_DEBUG_SEL0    0x0000001F
#define RTCIO_RTC_DEBUG_SEL0_M  (RTCIO_RTC_DEBUG_SEL0_V << RTCIO_RTC_DEBUG_SEL0_S)
#define RTCIO_RTC_DEBUG_SEL0_V  0x0000001F
#define RTCIO_RTC_DEBUG_SEL0_S  0

/* RTCIO_TOUCH_PAD0_REG register
 * Touch pad 0 configuration register
 */

#define RTCIO_TOUCH_PAD0_REG (DR_REG_RTCIO_BASE + 0x84)

/* RTCIO_TOUCH_PAD0_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD0_DRV    0x00000003
#define RTCIO_TOUCH_PAD0_DRV_M  (RTCIO_TOUCH_PAD0_DRV_V << RTCIO_TOUCH_PAD0_DRV_S)
#define RTCIO_TOUCH_PAD0_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD0_DRV_S  29

/* RTCIO_TOUCH_PAD0_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD0_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD0_RDE_M  (RTCIO_TOUCH_PAD0_RDE_V << RTCIO_TOUCH_PAD0_RDE_S)
#define RTCIO_TOUCH_PAD0_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD0_RDE_S  28

/* RTCIO_TOUCH_PAD0_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD0_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD0_RUE_M  (RTCIO_TOUCH_PAD0_RUE_V << RTCIO_TOUCH_PAD0_RUE_S)
#define RTCIO_TOUCH_PAD0_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD0_RUE_S  27

/* RTCIO_TOUCH_PAD0_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD0_START    (BIT(22))
#define RTCIO_TOUCH_PAD0_START_M  (RTCIO_TOUCH_PAD0_START_V << RTCIO_TOUCH_PAD0_START_S)
#define RTCIO_TOUCH_PAD0_START_V  0x00000001
#define RTCIO_TOUCH_PAD0_START_S  22

/* RTCIO_TOUCH_PAD0_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD0_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD0_TIE_OPT_M  (RTCIO_TOUCH_PAD0_TIE_OPT_V << RTCIO_TOUCH_PAD0_TIE_OPT_S)
#define RTCIO_TOUCH_PAD0_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD0_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD0_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD0_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD0_XPD_M  (RTCIO_TOUCH_PAD0_XPD_V << RTCIO_TOUCH_PAD0_XPD_S)
#define RTCIO_TOUCH_PAD0_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD0_XPD_S  20

/* RTCIO_TOUCH_PAD0_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD0_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD0_MUX_SEL_M  (RTCIO_TOUCH_PAD0_MUX_SEL_V << RTCIO_TOUCH_PAD0_MUX_SEL_S)
#define RTCIO_TOUCH_PAD0_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD0_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD0_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD0_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD0_FUN_SEL_M  (RTCIO_TOUCH_PAD0_FUN_SEL_V << RTCIO_TOUCH_PAD0_FUN_SEL_S)
#define RTCIO_TOUCH_PAD0_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD0_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD0_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD0_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD0_SLP_SEL_M  (RTCIO_TOUCH_PAD0_SLP_SEL_V << RTCIO_TOUCH_PAD0_SLP_SEL_S)
#define RTCIO_TOUCH_PAD0_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD0_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD0_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD0_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD0_SLP_IE_M  (RTCIO_TOUCH_PAD0_SLP_IE_V << RTCIO_TOUCH_PAD0_SLP_IE_S)
#define RTCIO_TOUCH_PAD0_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD0_SLP_IE_S  15

/* RTCIO_TOUCH_PAD0_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD0_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD0_SLP_OE_M  (RTCIO_TOUCH_PAD0_SLP_OE_V << RTCIO_TOUCH_PAD0_SLP_OE_S)
#define RTCIO_TOUCH_PAD0_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD0_SLP_OE_S  14

/* RTCIO_TOUCH_PAD0_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD0_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD0_FUN_IE_M  (RTCIO_TOUCH_PAD0_FUN_IE_V << RTCIO_TOUCH_PAD0_FUN_IE_S)
#define RTCIO_TOUCH_PAD0_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD0_FUN_IE_S  13

/* RTCIO_TOUCH_PAD1_REG register
 * Touch pad 1 configuration register
 */

#define RTCIO_TOUCH_PAD1_REG (DR_REG_RTCIO_BASE + 0x88)

/* RTCIO_TOUCH_PAD1_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD1_DRV    0x00000003
#define RTCIO_TOUCH_PAD1_DRV_M  (RTCIO_TOUCH_PAD1_DRV_V << RTCIO_TOUCH_PAD1_DRV_S)
#define RTCIO_TOUCH_PAD1_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD1_DRV_S  29

/* RTCIO_TOUCH_PAD1_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD1_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD1_RDE_M  (RTCIO_TOUCH_PAD1_RDE_V << RTCIO_TOUCH_PAD1_RDE_S)
#define RTCIO_TOUCH_PAD1_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD1_RDE_S  28

/* RTCIO_TOUCH_PAD1_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD1_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD1_RUE_M  (RTCIO_TOUCH_PAD1_RUE_V << RTCIO_TOUCH_PAD1_RUE_S)
#define RTCIO_TOUCH_PAD1_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD1_RUE_S  27

/* RTCIO_TOUCH_PAD1_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD1_START    (BIT(22))
#define RTCIO_TOUCH_PAD1_START_M  (RTCIO_TOUCH_PAD1_START_V << RTCIO_TOUCH_PAD1_START_S)
#define RTCIO_TOUCH_PAD1_START_V  0x00000001
#define RTCIO_TOUCH_PAD1_START_S  22

/* RTCIO_TOUCH_PAD1_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD1_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD1_TIE_OPT_M  (RTCIO_TOUCH_PAD1_TIE_OPT_V << RTCIO_TOUCH_PAD1_TIE_OPT_S)
#define RTCIO_TOUCH_PAD1_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD1_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD1_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD1_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD1_XPD_M  (RTCIO_TOUCH_PAD1_XPD_V << RTCIO_TOUCH_PAD1_XPD_S)
#define RTCIO_TOUCH_PAD1_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD1_XPD_S  20

/* RTCIO_TOUCH_PAD1_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD1_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD1_MUX_SEL_M  (RTCIO_TOUCH_PAD1_MUX_SEL_V << RTCIO_TOUCH_PAD1_MUX_SEL_S)
#define RTCIO_TOUCH_PAD1_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD1_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD1_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD1_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD1_FUN_SEL_M  (RTCIO_TOUCH_PAD1_FUN_SEL_V << RTCIO_TOUCH_PAD1_FUN_SEL_S)
#define RTCIO_TOUCH_PAD1_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD1_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD1_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD1_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD1_SLP_SEL_M  (RTCIO_TOUCH_PAD1_SLP_SEL_V << RTCIO_TOUCH_PAD1_SLP_SEL_S)
#define RTCIO_TOUCH_PAD1_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD1_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD1_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD1_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD1_SLP_IE_M  (RTCIO_TOUCH_PAD1_SLP_IE_V << RTCIO_TOUCH_PAD1_SLP_IE_S)
#define RTCIO_TOUCH_PAD1_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD1_SLP_IE_S  15

/* RTCIO_TOUCH_PAD1_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD1_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD1_SLP_OE_M  (RTCIO_TOUCH_PAD1_SLP_OE_V << RTCIO_TOUCH_PAD1_SLP_OE_S)
#define RTCIO_TOUCH_PAD1_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD1_SLP_OE_S  14

/* RTCIO_TOUCH_PAD1_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD1_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD1_FUN_IE_M  (RTCIO_TOUCH_PAD1_FUN_IE_V << RTCIO_TOUCH_PAD1_FUN_IE_S)
#define RTCIO_TOUCH_PAD1_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD1_FUN_IE_S  13

/* RTCIO_TOUCH_PAD2_REG register
 * Touch pad 2 configuration register
 */

#define RTCIO_TOUCH_PAD2_REG (DR_REG_RTCIO_BASE + 0x8c)

/* RTCIO_TOUCH_PAD2_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD2_DRV    0x00000003
#define RTCIO_TOUCH_PAD2_DRV_M  (RTCIO_TOUCH_PAD2_DRV_V << RTCIO_TOUCH_PAD2_DRV_S)
#define RTCIO_TOUCH_PAD2_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD2_DRV_S  29

/* RTCIO_TOUCH_PAD2_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD2_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD2_RDE_M  (RTCIO_TOUCH_PAD2_RDE_V << RTCIO_TOUCH_PAD2_RDE_S)
#define RTCIO_TOUCH_PAD2_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD2_RDE_S  28

/* RTCIO_TOUCH_PAD2_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD2_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD2_RUE_M  (RTCIO_TOUCH_PAD2_RUE_V << RTCIO_TOUCH_PAD2_RUE_S)
#define RTCIO_TOUCH_PAD2_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD2_RUE_S  27

/* RTCIO_TOUCH_PAD2_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD2_START    (BIT(22))
#define RTCIO_TOUCH_PAD2_START_M  (RTCIO_TOUCH_PAD2_START_V << RTCIO_TOUCH_PAD2_START_S)
#define RTCIO_TOUCH_PAD2_START_V  0x00000001
#define RTCIO_TOUCH_PAD2_START_S  22

/* RTCIO_TOUCH_PAD2_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD2_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD2_TIE_OPT_M  (RTCIO_TOUCH_PAD2_TIE_OPT_V << RTCIO_TOUCH_PAD2_TIE_OPT_S)
#define RTCIO_TOUCH_PAD2_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD2_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD2_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD2_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD2_XPD_M  (RTCIO_TOUCH_PAD2_XPD_V << RTCIO_TOUCH_PAD2_XPD_S)
#define RTCIO_TOUCH_PAD2_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD2_XPD_S  20

/* RTCIO_TOUCH_PAD2_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD2_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD2_MUX_SEL_M  (RTCIO_TOUCH_PAD2_MUX_SEL_V << RTCIO_TOUCH_PAD2_MUX_SEL_S)
#define RTCIO_TOUCH_PAD2_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD2_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD2_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD2_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD2_FUN_SEL_M  (RTCIO_TOUCH_PAD2_FUN_SEL_V << RTCIO_TOUCH_PAD2_FUN_SEL_S)
#define RTCIO_TOUCH_PAD2_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD2_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD2_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD2_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD2_SLP_SEL_M  (RTCIO_TOUCH_PAD2_SLP_SEL_V << RTCIO_TOUCH_PAD2_SLP_SEL_S)
#define RTCIO_TOUCH_PAD2_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD2_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD2_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD2_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD2_SLP_IE_M  (RTCIO_TOUCH_PAD2_SLP_IE_V << RTCIO_TOUCH_PAD2_SLP_IE_S)
#define RTCIO_TOUCH_PAD2_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD2_SLP_IE_S  15

/* RTCIO_TOUCH_PAD2_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD2_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD2_SLP_OE_M  (RTCIO_TOUCH_PAD2_SLP_OE_V << RTCIO_TOUCH_PAD2_SLP_OE_S)
#define RTCIO_TOUCH_PAD2_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD2_SLP_OE_S  14

/* RTCIO_TOUCH_PAD2_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD2_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD2_FUN_IE_M  (RTCIO_TOUCH_PAD2_FUN_IE_V << RTCIO_TOUCH_PAD2_FUN_IE_S)
#define RTCIO_TOUCH_PAD2_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD2_FUN_IE_S  13

/* RTCIO_TOUCH_PAD3_REG register
 * Touch pad 3 configuration register
 */

#define RTCIO_TOUCH_PAD3_REG (DR_REG_RTCIO_BASE + 0x90)

/* RTCIO_TOUCH_PAD3_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD3_DRV    0x00000003
#define RTCIO_TOUCH_PAD3_DRV_M  (RTCIO_TOUCH_PAD3_DRV_V << RTCIO_TOUCH_PAD3_DRV_S)
#define RTCIO_TOUCH_PAD3_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD3_DRV_S  29

/* RTCIO_TOUCH_PAD3_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD3_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD3_RDE_M  (RTCIO_TOUCH_PAD3_RDE_V << RTCIO_TOUCH_PAD3_RDE_S)
#define RTCIO_TOUCH_PAD3_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD3_RDE_S  28

/* RTCIO_TOUCH_PAD3_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD3_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD3_RUE_M  (RTCIO_TOUCH_PAD3_RUE_V << RTCIO_TOUCH_PAD3_RUE_S)
#define RTCIO_TOUCH_PAD3_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD3_RUE_S  27

/* RTCIO_TOUCH_PAD3_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD3_START    (BIT(22))
#define RTCIO_TOUCH_PAD3_START_M  (RTCIO_TOUCH_PAD3_START_V << RTCIO_TOUCH_PAD3_START_S)
#define RTCIO_TOUCH_PAD3_START_V  0x00000001
#define RTCIO_TOUCH_PAD3_START_S  22

/* RTCIO_TOUCH_PAD3_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD3_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD3_TIE_OPT_M  (RTCIO_TOUCH_PAD3_TIE_OPT_V << RTCIO_TOUCH_PAD3_TIE_OPT_S)
#define RTCIO_TOUCH_PAD3_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD3_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD3_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD3_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD3_XPD_M  (RTCIO_TOUCH_PAD3_XPD_V << RTCIO_TOUCH_PAD3_XPD_S)
#define RTCIO_TOUCH_PAD3_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD3_XPD_S  20

/* RTCIO_TOUCH_PAD3_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD3_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD3_MUX_SEL_M  (RTCIO_TOUCH_PAD3_MUX_SEL_V << RTCIO_TOUCH_PAD3_MUX_SEL_S)
#define RTCIO_TOUCH_PAD3_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD3_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD3_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD3_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD3_FUN_SEL_M  (RTCIO_TOUCH_PAD3_FUN_SEL_V << RTCIO_TOUCH_PAD3_FUN_SEL_S)
#define RTCIO_TOUCH_PAD3_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD3_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD3_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD3_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD3_SLP_SEL_M  (RTCIO_TOUCH_PAD3_SLP_SEL_V << RTCIO_TOUCH_PAD3_SLP_SEL_S)
#define RTCIO_TOUCH_PAD3_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD3_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD3_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD3_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD3_SLP_IE_M  (RTCIO_TOUCH_PAD3_SLP_IE_V << RTCIO_TOUCH_PAD3_SLP_IE_S)
#define RTCIO_TOUCH_PAD3_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD3_SLP_IE_S  15

/* RTCIO_TOUCH_PAD3_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD3_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD3_SLP_OE_M  (RTCIO_TOUCH_PAD3_SLP_OE_V << RTCIO_TOUCH_PAD3_SLP_OE_S)
#define RTCIO_TOUCH_PAD3_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD3_SLP_OE_S  14

/* RTCIO_TOUCH_PAD3_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD3_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD3_FUN_IE_M  (RTCIO_TOUCH_PAD3_FUN_IE_V << RTCIO_TOUCH_PAD3_FUN_IE_S)
#define RTCIO_TOUCH_PAD3_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD3_FUN_IE_S  13

/* RTCIO_TOUCH_PAD4_REG register
 * Touch pad 4 configuration register
 */

#define RTCIO_TOUCH_PAD4_REG (DR_REG_RTCIO_BASE + 0x94)

/* RTCIO_TOUCH_PAD4_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD4_DRV    0x00000003
#define RTCIO_TOUCH_PAD4_DRV_M  (RTCIO_TOUCH_PAD4_DRV_V << RTCIO_TOUCH_PAD4_DRV_S)
#define RTCIO_TOUCH_PAD4_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD4_DRV_S  29

/* RTCIO_TOUCH_PAD4_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD4_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD4_RDE_M  (RTCIO_TOUCH_PAD4_RDE_V << RTCIO_TOUCH_PAD4_RDE_S)
#define RTCIO_TOUCH_PAD4_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD4_RDE_S  28

/* RTCIO_TOUCH_PAD4_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD4_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD4_RUE_M  (RTCIO_TOUCH_PAD4_RUE_V << RTCIO_TOUCH_PAD4_RUE_S)
#define RTCIO_TOUCH_PAD4_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD4_RUE_S  27

/* RTCIO_TOUCH_PAD4_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD4_START    (BIT(22))
#define RTCIO_TOUCH_PAD4_START_M  (RTCIO_TOUCH_PAD4_START_V << RTCIO_TOUCH_PAD4_START_S)
#define RTCIO_TOUCH_PAD4_START_V  0x00000001
#define RTCIO_TOUCH_PAD4_START_S  22

/* RTCIO_TOUCH_PAD4_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD4_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD4_TIE_OPT_M  (RTCIO_TOUCH_PAD4_TIE_OPT_V << RTCIO_TOUCH_PAD4_TIE_OPT_S)
#define RTCIO_TOUCH_PAD4_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD4_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD4_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD4_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD4_XPD_M  (RTCIO_TOUCH_PAD4_XPD_V << RTCIO_TOUCH_PAD4_XPD_S)
#define RTCIO_TOUCH_PAD4_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD4_XPD_S  20

/* RTCIO_TOUCH_PAD4_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD4_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD4_MUX_SEL_M  (RTCIO_TOUCH_PAD4_MUX_SEL_V << RTCIO_TOUCH_PAD4_MUX_SEL_S)
#define RTCIO_TOUCH_PAD4_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD4_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD4_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD4_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD4_FUN_SEL_M  (RTCIO_TOUCH_PAD4_FUN_SEL_V << RTCIO_TOUCH_PAD4_FUN_SEL_S)
#define RTCIO_TOUCH_PAD4_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD4_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD4_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD4_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD4_SLP_SEL_M  (RTCIO_TOUCH_PAD4_SLP_SEL_V << RTCIO_TOUCH_PAD4_SLP_SEL_S)
#define RTCIO_TOUCH_PAD4_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD4_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD4_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD4_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD4_SLP_IE_M  (RTCIO_TOUCH_PAD4_SLP_IE_V << RTCIO_TOUCH_PAD4_SLP_IE_S)
#define RTCIO_TOUCH_PAD4_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD4_SLP_IE_S  15

/* RTCIO_TOUCH_PAD4_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD4_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD4_SLP_OE_M  (RTCIO_TOUCH_PAD4_SLP_OE_V << RTCIO_TOUCH_PAD4_SLP_OE_S)
#define RTCIO_TOUCH_PAD4_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD4_SLP_OE_S  14

/* RTCIO_TOUCH_PAD4_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD4_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD4_FUN_IE_M  (RTCIO_TOUCH_PAD4_FUN_IE_V << RTCIO_TOUCH_PAD4_FUN_IE_S)
#define RTCIO_TOUCH_PAD4_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD4_FUN_IE_S  13

/* RTCIO_TOUCH_PAD5_REG register
 * Touch pad 5 configuration register
 */

#define RTCIO_TOUCH_PAD5_REG (DR_REG_RTCIO_BASE + 0x98)

/* RTCIO_TOUCH_PAD5_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD5_DRV    0x00000003
#define RTCIO_TOUCH_PAD5_DRV_M  (RTCIO_TOUCH_PAD5_DRV_V << RTCIO_TOUCH_PAD5_DRV_S)
#define RTCIO_TOUCH_PAD5_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD5_DRV_S  29

/* RTCIO_TOUCH_PAD5_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD5_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD5_RDE_M  (RTCIO_TOUCH_PAD5_RDE_V << RTCIO_TOUCH_PAD5_RDE_S)
#define RTCIO_TOUCH_PAD5_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD5_RDE_S  28

/* RTCIO_TOUCH_PAD5_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD5_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD5_RUE_M  (RTCIO_TOUCH_PAD5_RUE_V << RTCIO_TOUCH_PAD5_RUE_S)
#define RTCIO_TOUCH_PAD5_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD5_RUE_S  27

/* RTCIO_TOUCH_PAD5_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD5_START    (BIT(22))
#define RTCIO_TOUCH_PAD5_START_M  (RTCIO_TOUCH_PAD5_START_V << RTCIO_TOUCH_PAD5_START_S)
#define RTCIO_TOUCH_PAD5_START_V  0x00000001
#define RTCIO_TOUCH_PAD5_START_S  22

/* RTCIO_TOUCH_PAD5_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD5_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD5_TIE_OPT_M  (RTCIO_TOUCH_PAD5_TIE_OPT_V << RTCIO_TOUCH_PAD5_TIE_OPT_S)
#define RTCIO_TOUCH_PAD5_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD5_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD5_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD5_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD5_XPD_M  (RTCIO_TOUCH_PAD5_XPD_V << RTCIO_TOUCH_PAD5_XPD_S)
#define RTCIO_TOUCH_PAD5_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD5_XPD_S  20

/* RTCIO_TOUCH_PAD5_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD5_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD5_MUX_SEL_M  (RTCIO_TOUCH_PAD5_MUX_SEL_V << RTCIO_TOUCH_PAD5_MUX_SEL_S)
#define RTCIO_TOUCH_PAD5_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD5_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD5_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD5_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD5_FUN_SEL_M  (RTCIO_TOUCH_PAD5_FUN_SEL_V << RTCIO_TOUCH_PAD5_FUN_SEL_S)
#define RTCIO_TOUCH_PAD5_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD5_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD5_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD5_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD5_SLP_SEL_M  (RTCIO_TOUCH_PAD5_SLP_SEL_V << RTCIO_TOUCH_PAD5_SLP_SEL_S)
#define RTCIO_TOUCH_PAD5_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD5_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD5_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD5_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD5_SLP_IE_M  (RTCIO_TOUCH_PAD5_SLP_IE_V << RTCIO_TOUCH_PAD5_SLP_IE_S)
#define RTCIO_TOUCH_PAD5_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD5_SLP_IE_S  15

/* RTCIO_TOUCH_PAD5_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD5_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD5_SLP_OE_M  (RTCIO_TOUCH_PAD5_SLP_OE_V << RTCIO_TOUCH_PAD5_SLP_OE_S)
#define RTCIO_TOUCH_PAD5_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD5_SLP_OE_S  14

/* RTCIO_TOUCH_PAD5_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD5_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD5_FUN_IE_M  (RTCIO_TOUCH_PAD5_FUN_IE_V << RTCIO_TOUCH_PAD5_FUN_IE_S)
#define RTCIO_TOUCH_PAD5_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD5_FUN_IE_S  13

/* RTCIO_TOUCH_PAD6_REG register
 * Touch pad 6 configuration register
 */

#define RTCIO_TOUCH_PAD6_REG (DR_REG_RTCIO_BASE + 0x9c)

/* RTCIO_TOUCH_PAD6_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD6_DRV    0x00000003
#define RTCIO_TOUCH_PAD6_DRV_M  (RTCIO_TOUCH_PAD6_DRV_V << RTCIO_TOUCH_PAD6_DRV_S)
#define RTCIO_TOUCH_PAD6_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD6_DRV_S  29

/* RTCIO_TOUCH_PAD6_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD6_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD6_RDE_M  (RTCIO_TOUCH_PAD6_RDE_V << RTCIO_TOUCH_PAD6_RDE_S)
#define RTCIO_TOUCH_PAD6_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD6_RDE_S  28

/* RTCIO_TOUCH_PAD6_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD6_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD6_RUE_M  (RTCIO_TOUCH_PAD6_RUE_V << RTCIO_TOUCH_PAD6_RUE_S)
#define RTCIO_TOUCH_PAD6_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD6_RUE_S  27

/* RTCIO_TOUCH_PAD6_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD6_START    (BIT(22))
#define RTCIO_TOUCH_PAD6_START_M  (RTCIO_TOUCH_PAD6_START_V << RTCIO_TOUCH_PAD6_START_S)
#define RTCIO_TOUCH_PAD6_START_V  0x00000001
#define RTCIO_TOUCH_PAD6_START_S  22

/* RTCIO_TOUCH_PAD6_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD6_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD6_TIE_OPT_M  (RTCIO_TOUCH_PAD6_TIE_OPT_V << RTCIO_TOUCH_PAD6_TIE_OPT_S)
#define RTCIO_TOUCH_PAD6_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD6_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD6_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD6_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD6_XPD_M  (RTCIO_TOUCH_PAD6_XPD_V << RTCIO_TOUCH_PAD6_XPD_S)
#define RTCIO_TOUCH_PAD6_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD6_XPD_S  20

/* RTCIO_TOUCH_PAD6_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD6_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD6_MUX_SEL_M  (RTCIO_TOUCH_PAD6_MUX_SEL_V << RTCIO_TOUCH_PAD6_MUX_SEL_S)
#define RTCIO_TOUCH_PAD6_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD6_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD6_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD6_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD6_FUN_SEL_M  (RTCIO_TOUCH_PAD6_FUN_SEL_V << RTCIO_TOUCH_PAD6_FUN_SEL_S)
#define RTCIO_TOUCH_PAD6_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD6_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD6_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD6_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD6_SLP_SEL_M  (RTCIO_TOUCH_PAD6_SLP_SEL_V << RTCIO_TOUCH_PAD6_SLP_SEL_S)
#define RTCIO_TOUCH_PAD6_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD6_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD6_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD6_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD6_SLP_IE_M  (RTCIO_TOUCH_PAD6_SLP_IE_V << RTCIO_TOUCH_PAD6_SLP_IE_S)
#define RTCIO_TOUCH_PAD6_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD6_SLP_IE_S  15

/* RTCIO_TOUCH_PAD6_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD6_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD6_SLP_OE_M  (RTCIO_TOUCH_PAD6_SLP_OE_V << RTCIO_TOUCH_PAD6_SLP_OE_S)
#define RTCIO_TOUCH_PAD6_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD6_SLP_OE_S  14

/* RTCIO_TOUCH_PAD6_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD6_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD6_FUN_IE_M  (RTCIO_TOUCH_PAD6_FUN_IE_V << RTCIO_TOUCH_PAD6_FUN_IE_S)
#define RTCIO_TOUCH_PAD6_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD6_FUN_IE_S  13

/* RTCIO_TOUCH_PAD7_REG register
 * Touch pad 7 configuration register
 */

#define RTCIO_TOUCH_PAD7_REG (DR_REG_RTCIO_BASE + 0xa0)

/* RTCIO_TOUCH_PAD7_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD7_DRV    0x00000003
#define RTCIO_TOUCH_PAD7_DRV_M  (RTCIO_TOUCH_PAD7_DRV_V << RTCIO_TOUCH_PAD7_DRV_S)
#define RTCIO_TOUCH_PAD7_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD7_DRV_S  29

/* RTCIO_TOUCH_PAD7_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD7_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD7_RDE_M  (RTCIO_TOUCH_PAD7_RDE_V << RTCIO_TOUCH_PAD7_RDE_S)
#define RTCIO_TOUCH_PAD7_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD7_RDE_S  28

/* RTCIO_TOUCH_PAD7_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD7_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD7_RUE_M  (RTCIO_TOUCH_PAD7_RUE_V << RTCIO_TOUCH_PAD7_RUE_S)
#define RTCIO_TOUCH_PAD7_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD7_RUE_S  27

/* RTCIO_TOUCH_PAD7_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD7_START    (BIT(22))
#define RTCIO_TOUCH_PAD7_START_M  (RTCIO_TOUCH_PAD7_START_V << RTCIO_TOUCH_PAD7_START_S)
#define RTCIO_TOUCH_PAD7_START_V  0x00000001
#define RTCIO_TOUCH_PAD7_START_S  22

/* RTCIO_TOUCH_PAD7_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD7_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD7_TIE_OPT_M  (RTCIO_TOUCH_PAD7_TIE_OPT_V << RTCIO_TOUCH_PAD7_TIE_OPT_S)
#define RTCIO_TOUCH_PAD7_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD7_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD7_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD7_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD7_XPD_M  (RTCIO_TOUCH_PAD7_XPD_V << RTCIO_TOUCH_PAD7_XPD_S)
#define RTCIO_TOUCH_PAD7_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD7_XPD_S  20

/* RTCIO_TOUCH_PAD7_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD7_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD7_MUX_SEL_M  (RTCIO_TOUCH_PAD7_MUX_SEL_V << RTCIO_TOUCH_PAD7_MUX_SEL_S)
#define RTCIO_TOUCH_PAD7_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD7_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD7_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD7_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD7_FUN_SEL_M  (RTCIO_TOUCH_PAD7_FUN_SEL_V << RTCIO_TOUCH_PAD7_FUN_SEL_S)
#define RTCIO_TOUCH_PAD7_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD7_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD7_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD7_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD7_SLP_SEL_M  (RTCIO_TOUCH_PAD7_SLP_SEL_V << RTCIO_TOUCH_PAD7_SLP_SEL_S)
#define RTCIO_TOUCH_PAD7_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD7_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD7_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD7_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD7_SLP_IE_M  (RTCIO_TOUCH_PAD7_SLP_IE_V << RTCIO_TOUCH_PAD7_SLP_IE_S)
#define RTCIO_TOUCH_PAD7_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD7_SLP_IE_S  15

/* RTCIO_TOUCH_PAD7_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD7_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD7_SLP_OE_M  (RTCIO_TOUCH_PAD7_SLP_OE_V << RTCIO_TOUCH_PAD7_SLP_OE_S)
#define RTCIO_TOUCH_PAD7_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD7_SLP_OE_S  14

/* RTCIO_TOUCH_PAD7_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD7_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD7_FUN_IE_M  (RTCIO_TOUCH_PAD7_FUN_IE_V << RTCIO_TOUCH_PAD7_FUN_IE_S)
#define RTCIO_TOUCH_PAD7_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD7_FUN_IE_S  13

/* RTCIO_TOUCH_PAD8_REG register
 * Touch pad 8 configuration register
 */

#define RTCIO_TOUCH_PAD8_REG (DR_REG_RTCIO_BASE + 0xa4)

/* RTCIO_TOUCH_PAD8_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD8_DRV    0x00000003
#define RTCIO_TOUCH_PAD8_DRV_M  (RTCIO_TOUCH_PAD8_DRV_V << RTCIO_TOUCH_PAD8_DRV_S)
#define RTCIO_TOUCH_PAD8_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD8_DRV_S  29

/* RTCIO_TOUCH_PAD8_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD8_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD8_RDE_M  (RTCIO_TOUCH_PAD8_RDE_V << RTCIO_TOUCH_PAD8_RDE_S)
#define RTCIO_TOUCH_PAD8_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD8_RDE_S  28

/* RTCIO_TOUCH_PAD8_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD8_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD8_RUE_M  (RTCIO_TOUCH_PAD8_RUE_V << RTCIO_TOUCH_PAD8_RUE_S)
#define RTCIO_TOUCH_PAD8_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD8_RUE_S  27

/* RTCIO_TOUCH_PAD8_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD8_START    (BIT(22))
#define RTCIO_TOUCH_PAD8_START_M  (RTCIO_TOUCH_PAD8_START_V << RTCIO_TOUCH_PAD8_START_S)
#define RTCIO_TOUCH_PAD8_START_V  0x00000001
#define RTCIO_TOUCH_PAD8_START_S  22

/* RTCIO_TOUCH_PAD8_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD8_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD8_TIE_OPT_M  (RTCIO_TOUCH_PAD8_TIE_OPT_V << RTCIO_TOUCH_PAD8_TIE_OPT_S)
#define RTCIO_TOUCH_PAD8_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD8_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD8_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD8_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD8_XPD_M  (RTCIO_TOUCH_PAD8_XPD_V << RTCIO_TOUCH_PAD8_XPD_S)
#define RTCIO_TOUCH_PAD8_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD8_XPD_S  20

/* RTCIO_TOUCH_PAD8_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD8_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD8_MUX_SEL_M  (RTCIO_TOUCH_PAD8_MUX_SEL_V << RTCIO_TOUCH_PAD8_MUX_SEL_S)
#define RTCIO_TOUCH_PAD8_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD8_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD8_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD8_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD8_FUN_SEL_M  (RTCIO_TOUCH_PAD8_FUN_SEL_V << RTCIO_TOUCH_PAD8_FUN_SEL_S)
#define RTCIO_TOUCH_PAD8_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD8_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD8_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD8_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD8_SLP_SEL_M  (RTCIO_TOUCH_PAD8_SLP_SEL_V << RTCIO_TOUCH_PAD8_SLP_SEL_S)
#define RTCIO_TOUCH_PAD8_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD8_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD8_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD8_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD8_SLP_IE_M  (RTCIO_TOUCH_PAD8_SLP_IE_V << RTCIO_TOUCH_PAD8_SLP_IE_S)
#define RTCIO_TOUCH_PAD8_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD8_SLP_IE_S  15

/* RTCIO_TOUCH_PAD8_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD8_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD8_SLP_OE_M  (RTCIO_TOUCH_PAD8_SLP_OE_V << RTCIO_TOUCH_PAD8_SLP_OE_S)
#define RTCIO_TOUCH_PAD8_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD8_SLP_OE_S  14

/* RTCIO_TOUCH_PAD8_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD8_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD8_FUN_IE_M  (RTCIO_TOUCH_PAD8_FUN_IE_V << RTCIO_TOUCH_PAD8_FUN_IE_S)
#define RTCIO_TOUCH_PAD8_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD8_FUN_IE_S  13

/* RTCIO_TOUCH_PAD9_REG register
 * Touch pad 9 configuration register
 */

#define RTCIO_TOUCH_PAD9_REG (DR_REG_RTCIO_BASE + 0xa8)

/* RTCIO_TOUCH_PAD9_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD9_DRV    0x00000003
#define RTCIO_TOUCH_PAD9_DRV_M  (RTCIO_TOUCH_PAD9_DRV_V << RTCIO_TOUCH_PAD9_DRV_S)
#define RTCIO_TOUCH_PAD9_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD9_DRV_S  29

/* RTCIO_TOUCH_PAD9_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD9_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD9_RDE_M  (RTCIO_TOUCH_PAD9_RDE_V << RTCIO_TOUCH_PAD9_RDE_S)
#define RTCIO_TOUCH_PAD9_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD9_RDE_S  28

/* RTCIO_TOUCH_PAD9_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD9_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD9_RUE_M  (RTCIO_TOUCH_PAD9_RUE_V << RTCIO_TOUCH_PAD9_RUE_S)
#define RTCIO_TOUCH_PAD9_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD9_RUE_S  27

/* RTCIO_TOUCH_PAD9_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD9_START    (BIT(22))
#define RTCIO_TOUCH_PAD9_START_M  (RTCIO_TOUCH_PAD9_START_V << RTCIO_TOUCH_PAD9_START_S)
#define RTCIO_TOUCH_PAD9_START_V  0x00000001
#define RTCIO_TOUCH_PAD9_START_S  22

/* RTCIO_TOUCH_PAD9_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD9_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD9_TIE_OPT_M  (RTCIO_TOUCH_PAD9_TIE_OPT_V << RTCIO_TOUCH_PAD9_TIE_OPT_S)
#define RTCIO_TOUCH_PAD9_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD9_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD9_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD9_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD9_XPD_M  (RTCIO_TOUCH_PAD9_XPD_V << RTCIO_TOUCH_PAD9_XPD_S)
#define RTCIO_TOUCH_PAD9_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD9_XPD_S  20

/* RTCIO_TOUCH_PAD9_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD9_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD9_MUX_SEL_M  (RTCIO_TOUCH_PAD9_MUX_SEL_V << RTCIO_TOUCH_PAD9_MUX_SEL_S)
#define RTCIO_TOUCH_PAD9_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD9_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD9_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD9_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD9_FUN_SEL_M  (RTCIO_TOUCH_PAD9_FUN_SEL_V << RTCIO_TOUCH_PAD9_FUN_SEL_S)
#define RTCIO_TOUCH_PAD9_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD9_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD9_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD9_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD9_SLP_SEL_M  (RTCIO_TOUCH_PAD9_SLP_SEL_V << RTCIO_TOUCH_PAD9_SLP_SEL_S)
#define RTCIO_TOUCH_PAD9_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD9_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD9_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD9_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD9_SLP_IE_M  (RTCIO_TOUCH_PAD9_SLP_IE_V << RTCIO_TOUCH_PAD9_SLP_IE_S)
#define RTCIO_TOUCH_PAD9_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD9_SLP_IE_S  15

/* RTCIO_TOUCH_PAD9_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD9_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD9_SLP_OE_M  (RTCIO_TOUCH_PAD9_SLP_OE_V << RTCIO_TOUCH_PAD9_SLP_OE_S)
#define RTCIO_TOUCH_PAD9_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD9_SLP_OE_S  14

/* RTCIO_TOUCH_PAD9_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD9_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD9_FUN_IE_M  (RTCIO_TOUCH_PAD9_FUN_IE_V << RTCIO_TOUCH_PAD9_FUN_IE_S)
#define RTCIO_TOUCH_PAD9_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD9_FUN_IE_S  13

/* RTCIO_TOUCH_PAD10_REG register
 * Touch pad 10 configuration register
 */

#define RTCIO_TOUCH_PAD10_REG (DR_REG_RTCIO_BASE + 0xac)

/* RTCIO_TOUCH_PAD10_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD10_DRV    0x00000003
#define RTCIO_TOUCH_PAD10_DRV_M  (RTCIO_TOUCH_PAD10_DRV_V << RTCIO_TOUCH_PAD10_DRV_S)
#define RTCIO_TOUCH_PAD10_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD10_DRV_S  29

/* RTCIO_TOUCH_PAD10_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD10_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD10_RDE_M  (RTCIO_TOUCH_PAD10_RDE_V << RTCIO_TOUCH_PAD10_RDE_S)
#define RTCIO_TOUCH_PAD10_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD10_RDE_S  28

/* RTCIO_TOUCH_PAD10_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD10_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD10_RUE_M  (RTCIO_TOUCH_PAD10_RUE_V << RTCIO_TOUCH_PAD10_RUE_S)
#define RTCIO_TOUCH_PAD10_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD10_RUE_S  27

/* RTCIO_TOUCH_PAD10_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD10_START    (BIT(22))
#define RTCIO_TOUCH_PAD10_START_M  (RTCIO_TOUCH_PAD10_START_V << RTCIO_TOUCH_PAD10_START_S)
#define RTCIO_TOUCH_PAD10_START_V  0x00000001
#define RTCIO_TOUCH_PAD10_START_S  22

/* RTCIO_TOUCH_PAD10_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD10_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD10_TIE_OPT_M  (RTCIO_TOUCH_PAD10_TIE_OPT_V << RTCIO_TOUCH_PAD10_TIE_OPT_S)
#define RTCIO_TOUCH_PAD10_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD10_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD10_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD10_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD10_XPD_M  (RTCIO_TOUCH_PAD10_XPD_V << RTCIO_TOUCH_PAD10_XPD_S)
#define RTCIO_TOUCH_PAD10_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD10_XPD_S  20

/* RTCIO_TOUCH_PAD10_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD10_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD10_MUX_SEL_M  (RTCIO_TOUCH_PAD10_MUX_SEL_V << RTCIO_TOUCH_PAD10_MUX_SEL_S)
#define RTCIO_TOUCH_PAD10_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD10_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD10_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD10_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD10_FUN_SEL_M  (RTCIO_TOUCH_PAD10_FUN_SEL_V << RTCIO_TOUCH_PAD10_FUN_SEL_S)
#define RTCIO_TOUCH_PAD10_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD10_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD10_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD10_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD10_SLP_SEL_M  (RTCIO_TOUCH_PAD10_SLP_SEL_V << RTCIO_TOUCH_PAD10_SLP_SEL_S)
#define RTCIO_TOUCH_PAD10_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD10_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD10_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD10_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD10_SLP_IE_M  (RTCIO_TOUCH_PAD10_SLP_IE_V << RTCIO_TOUCH_PAD10_SLP_IE_S)
#define RTCIO_TOUCH_PAD10_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD10_SLP_IE_S  15

/* RTCIO_TOUCH_PAD10_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD10_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD10_SLP_OE_M  (RTCIO_TOUCH_PAD10_SLP_OE_V << RTCIO_TOUCH_PAD10_SLP_OE_S)
#define RTCIO_TOUCH_PAD10_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD10_SLP_OE_S  14

/* RTCIO_TOUCH_PAD10_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD10_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD10_FUN_IE_M  (RTCIO_TOUCH_PAD10_FUN_IE_V << RTCIO_TOUCH_PAD10_FUN_IE_S)
#define RTCIO_TOUCH_PAD10_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD10_FUN_IE_S  13

/* RTCIO_TOUCH_PAD11_REG register
 * Touch pad 11 configuration register
 */

#define RTCIO_TOUCH_PAD11_REG (DR_REG_RTCIO_BASE + 0xb0)

/* RTCIO_TOUCH_PAD11_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD11_DRV    0x00000003
#define RTCIO_TOUCH_PAD11_DRV_M  (RTCIO_TOUCH_PAD11_DRV_V << RTCIO_TOUCH_PAD11_DRV_S)
#define RTCIO_TOUCH_PAD11_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD11_DRV_S  29

/* RTCIO_TOUCH_PAD11_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD11_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD11_RDE_M  (RTCIO_TOUCH_PAD11_RDE_V << RTCIO_TOUCH_PAD11_RDE_S)
#define RTCIO_TOUCH_PAD11_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD11_RDE_S  28

/* RTCIO_TOUCH_PAD11_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD11_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD11_RUE_M  (RTCIO_TOUCH_PAD11_RUE_V << RTCIO_TOUCH_PAD11_RUE_S)
#define RTCIO_TOUCH_PAD11_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD11_RUE_S  27

/* RTCIO_TOUCH_PAD11_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD11_START    (BIT(22))
#define RTCIO_TOUCH_PAD11_START_M  (RTCIO_TOUCH_PAD11_START_V << RTCIO_TOUCH_PAD11_START_S)
#define RTCIO_TOUCH_PAD11_START_V  0x00000001
#define RTCIO_TOUCH_PAD11_START_S  22

/* RTCIO_TOUCH_PAD11_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD11_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD11_TIE_OPT_M  (RTCIO_TOUCH_PAD11_TIE_OPT_V << RTCIO_TOUCH_PAD11_TIE_OPT_S)
#define RTCIO_TOUCH_PAD11_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD11_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD11_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD11_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD11_XPD_M  (RTCIO_TOUCH_PAD11_XPD_V << RTCIO_TOUCH_PAD11_XPD_S)
#define RTCIO_TOUCH_PAD11_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD11_XPD_S  20

/* RTCIO_TOUCH_PAD11_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD11_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD11_MUX_SEL_M  (RTCIO_TOUCH_PAD11_MUX_SEL_V << RTCIO_TOUCH_PAD11_MUX_SEL_S)
#define RTCIO_TOUCH_PAD11_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD11_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD11_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD11_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD11_FUN_SEL_M  (RTCIO_TOUCH_PAD11_FUN_SEL_V << RTCIO_TOUCH_PAD11_FUN_SEL_S)
#define RTCIO_TOUCH_PAD11_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD11_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD11_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD11_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD11_SLP_SEL_M  (RTCIO_TOUCH_PAD11_SLP_SEL_V << RTCIO_TOUCH_PAD11_SLP_SEL_S)
#define RTCIO_TOUCH_PAD11_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD11_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD11_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD11_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD11_SLP_IE_M  (RTCIO_TOUCH_PAD11_SLP_IE_V << RTCIO_TOUCH_PAD11_SLP_IE_S)
#define RTCIO_TOUCH_PAD11_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD11_SLP_IE_S  15

/* RTCIO_TOUCH_PAD11_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD11_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD11_SLP_OE_M  (RTCIO_TOUCH_PAD11_SLP_OE_V << RTCIO_TOUCH_PAD11_SLP_OE_S)
#define RTCIO_TOUCH_PAD11_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD11_SLP_OE_S  14

/* RTCIO_TOUCH_PAD11_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD11_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD11_FUN_IE_M  (RTCIO_TOUCH_PAD11_FUN_IE_V << RTCIO_TOUCH_PAD11_FUN_IE_S)
#define RTCIO_TOUCH_PAD11_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD11_FUN_IE_S  13

/* RTCIO_TOUCH_PAD12_REG register
 * Touch pad 12 configuration register
 */

#define RTCIO_TOUCH_PAD12_REG (DR_REG_RTCIO_BASE + 0xb4)

/* RTCIO_TOUCH_PAD12_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD12_DRV    0x00000003
#define RTCIO_TOUCH_PAD12_DRV_M  (RTCIO_TOUCH_PAD12_DRV_V << RTCIO_TOUCH_PAD12_DRV_S)
#define RTCIO_TOUCH_PAD12_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD12_DRV_S  29

/* RTCIO_TOUCH_PAD12_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD12_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD12_RDE_M  (RTCIO_TOUCH_PAD12_RDE_V << RTCIO_TOUCH_PAD12_RDE_S)
#define RTCIO_TOUCH_PAD12_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD12_RDE_S  28

/* RTCIO_TOUCH_PAD12_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD12_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD12_RUE_M  (RTCIO_TOUCH_PAD12_RUE_V << RTCIO_TOUCH_PAD12_RUE_S)
#define RTCIO_TOUCH_PAD12_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD12_RUE_S  27

/* RTCIO_TOUCH_PAD12_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD12_START    (BIT(22))
#define RTCIO_TOUCH_PAD12_START_M  (RTCIO_TOUCH_PAD12_START_V << RTCIO_TOUCH_PAD12_START_S)
#define RTCIO_TOUCH_PAD12_START_V  0x00000001
#define RTCIO_TOUCH_PAD12_START_S  22

/* RTCIO_TOUCH_PAD12_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD12_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD12_TIE_OPT_M  (RTCIO_TOUCH_PAD12_TIE_OPT_V << RTCIO_TOUCH_PAD12_TIE_OPT_S)
#define RTCIO_TOUCH_PAD12_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD12_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD12_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD12_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD12_XPD_M  (RTCIO_TOUCH_PAD12_XPD_V << RTCIO_TOUCH_PAD12_XPD_S)
#define RTCIO_TOUCH_PAD12_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD12_XPD_S  20

/* RTCIO_TOUCH_PAD12_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD12_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD12_MUX_SEL_M  (RTCIO_TOUCH_PAD12_MUX_SEL_V << RTCIO_TOUCH_PAD12_MUX_SEL_S)
#define RTCIO_TOUCH_PAD12_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD12_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD12_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD12_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD12_FUN_SEL_M  (RTCIO_TOUCH_PAD12_FUN_SEL_V << RTCIO_TOUCH_PAD12_FUN_SEL_S)
#define RTCIO_TOUCH_PAD12_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD12_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD12_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD12_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD12_SLP_SEL_M  (RTCIO_TOUCH_PAD12_SLP_SEL_V << RTCIO_TOUCH_PAD12_SLP_SEL_S)
#define RTCIO_TOUCH_PAD12_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD12_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD12_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD12_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD12_SLP_IE_M  (RTCIO_TOUCH_PAD12_SLP_IE_V << RTCIO_TOUCH_PAD12_SLP_IE_S)
#define RTCIO_TOUCH_PAD12_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD12_SLP_IE_S  15

/* RTCIO_TOUCH_PAD12_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD12_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD12_SLP_OE_M  (RTCIO_TOUCH_PAD12_SLP_OE_V << RTCIO_TOUCH_PAD12_SLP_OE_S)
#define RTCIO_TOUCH_PAD12_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD12_SLP_OE_S  14

/* RTCIO_TOUCH_PAD12_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD12_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD12_FUN_IE_M  (RTCIO_TOUCH_PAD12_FUN_IE_V << RTCIO_TOUCH_PAD12_FUN_IE_S)
#define RTCIO_TOUCH_PAD12_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD12_FUN_IE_S  13

/* RTCIO_TOUCH_PAD13_REG register
 * Touch pad 13 configuration register
 */

#define RTCIO_TOUCH_PAD13_REG (DR_REG_RTCIO_BASE + 0xb8)

/* RTCIO_TOUCH_PAD13_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD13_DRV    0x00000003
#define RTCIO_TOUCH_PAD13_DRV_M  (RTCIO_TOUCH_PAD13_DRV_V << RTCIO_TOUCH_PAD13_DRV_S)
#define RTCIO_TOUCH_PAD13_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD13_DRV_S  29

/* RTCIO_TOUCH_PAD13_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD13_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD13_RDE_M  (RTCIO_TOUCH_PAD13_RDE_V << RTCIO_TOUCH_PAD13_RDE_S)
#define RTCIO_TOUCH_PAD13_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD13_RDE_S  28

/* RTCIO_TOUCH_PAD13_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD13_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD13_RUE_M  (RTCIO_TOUCH_PAD13_RUE_V << RTCIO_TOUCH_PAD13_RUE_S)
#define RTCIO_TOUCH_PAD13_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD13_RUE_S  27

/* RTCIO_TOUCH_PAD13_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD13_START    (BIT(22))
#define RTCIO_TOUCH_PAD13_START_M  (RTCIO_TOUCH_PAD13_START_V << RTCIO_TOUCH_PAD13_START_S)
#define RTCIO_TOUCH_PAD13_START_V  0x00000001
#define RTCIO_TOUCH_PAD13_START_S  22

/* RTCIO_TOUCH_PAD13_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD13_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD13_TIE_OPT_M  (RTCIO_TOUCH_PAD13_TIE_OPT_V << RTCIO_TOUCH_PAD13_TIE_OPT_S)
#define RTCIO_TOUCH_PAD13_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD13_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD13_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD13_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD13_XPD_M  (RTCIO_TOUCH_PAD13_XPD_V << RTCIO_TOUCH_PAD13_XPD_S)
#define RTCIO_TOUCH_PAD13_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD13_XPD_S  20

/* RTCIO_TOUCH_PAD13_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD13_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD13_MUX_SEL_M  (RTCIO_TOUCH_PAD13_MUX_SEL_V << RTCIO_TOUCH_PAD13_MUX_SEL_S)
#define RTCIO_TOUCH_PAD13_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD13_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD13_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD13_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD13_FUN_SEL_M  (RTCIO_TOUCH_PAD13_FUN_SEL_V << RTCIO_TOUCH_PAD13_FUN_SEL_S)
#define RTCIO_TOUCH_PAD13_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD13_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD13_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD13_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD13_SLP_SEL_M  (RTCIO_TOUCH_PAD13_SLP_SEL_V << RTCIO_TOUCH_PAD13_SLP_SEL_S)
#define RTCIO_TOUCH_PAD13_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD13_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD13_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD13_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD13_SLP_IE_M  (RTCIO_TOUCH_PAD13_SLP_IE_V << RTCIO_TOUCH_PAD13_SLP_IE_S)
#define RTCIO_TOUCH_PAD13_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD13_SLP_IE_S  15

/* RTCIO_TOUCH_PAD13_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD13_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD13_SLP_OE_M  (RTCIO_TOUCH_PAD13_SLP_OE_V << RTCIO_TOUCH_PAD13_SLP_OE_S)
#define RTCIO_TOUCH_PAD13_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD13_SLP_OE_S  14

/* RTCIO_TOUCH_PAD13_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD13_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD13_FUN_IE_M  (RTCIO_TOUCH_PAD13_FUN_IE_V << RTCIO_TOUCH_PAD13_FUN_IE_S)
#define RTCIO_TOUCH_PAD13_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD13_FUN_IE_S  13

/* RTCIO_TOUCH_PAD14_REG register
 * Touch pad 14 configuration register
 */

#define RTCIO_TOUCH_PAD14_REG (DR_REG_RTCIO_BASE + 0xbc)

/* RTCIO_TOUCH_PAD14_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_TOUCH_PAD14_DRV    0x00000003
#define RTCIO_TOUCH_PAD14_DRV_M  (RTCIO_TOUCH_PAD14_DRV_V << RTCIO_TOUCH_PAD14_DRV_S)
#define RTCIO_TOUCH_PAD14_DRV_V  0x00000003
#define RTCIO_TOUCH_PAD14_DRV_S  29

/* RTCIO_TOUCH_PAD14_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_TOUCH_PAD14_RDE    (BIT(28))
#define RTCIO_TOUCH_PAD14_RDE_M  (RTCIO_TOUCH_PAD14_RDE_V << RTCIO_TOUCH_PAD14_RDE_S)
#define RTCIO_TOUCH_PAD14_RDE_V  0x00000001
#define RTCIO_TOUCH_PAD14_RDE_S  28

/* RTCIO_TOUCH_PAD14_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_TOUCH_PAD14_RUE    (BIT(27))
#define RTCIO_TOUCH_PAD14_RUE_M  (RTCIO_TOUCH_PAD14_RUE_V << RTCIO_TOUCH_PAD14_RUE_S)
#define RTCIO_TOUCH_PAD14_RUE_V  0x00000001
#define RTCIO_TOUCH_PAD14_RUE_S  27

/* RTCIO_TOUCH_PAD14_START : R/W; bitpos: [22]; default: 0;
 * Start touch sensor.
 */

#define RTCIO_TOUCH_PAD14_START    (BIT(22))
#define RTCIO_TOUCH_PAD14_START_M  (RTCIO_TOUCH_PAD14_START_V << RTCIO_TOUCH_PAD14_START_S)
#define RTCIO_TOUCH_PAD14_START_V  0x00000001
#define RTCIO_TOUCH_PAD14_START_S  22

/* RTCIO_TOUCH_PAD14_TIE_OPT : R/W; bitpos: [21]; default: 0;
 * The tie option of touch sensor. 0: tie low; 1: tie high.
 */

#define RTCIO_TOUCH_PAD14_TIE_OPT    (BIT(21))
#define RTCIO_TOUCH_PAD14_TIE_OPT_M  (RTCIO_TOUCH_PAD14_TIE_OPT_V << RTCIO_TOUCH_PAD14_TIE_OPT_S)
#define RTCIO_TOUCH_PAD14_TIE_OPT_V  0x00000001
#define RTCIO_TOUCH_PAD14_TIE_OPT_S  21

/* RTCIO_TOUCH_PAD14_XPD : R/W; bitpos: [20]; default: 0;
 * Touch sensor power on.
 */

#define RTCIO_TOUCH_PAD14_XPD    (BIT(20))
#define RTCIO_TOUCH_PAD14_XPD_M  (RTCIO_TOUCH_PAD14_XPD_V << RTCIO_TOUCH_PAD14_XPD_S)
#define RTCIO_TOUCH_PAD14_XPD_V  0x00000001
#define RTCIO_TOUCH_PAD14_XPD_S  20

/* RTCIO_TOUCH_PAD14_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * Connect the RTC pad input to digital pad input.  0 is available.
 */

#define RTCIO_TOUCH_PAD14_MUX_SEL    (BIT(19))
#define RTCIO_TOUCH_PAD14_MUX_SEL_M  (RTCIO_TOUCH_PAD14_MUX_SEL_V << RTCIO_TOUCH_PAD14_MUX_SEL_S)
#define RTCIO_TOUCH_PAD14_MUX_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD14_MUX_SEL_S  19

/* RTCIO_TOUCH_PAD14_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_TOUCH_PAD14_FUN_SEL    0x00000003
#define RTCIO_TOUCH_PAD14_FUN_SEL_M  (RTCIO_TOUCH_PAD14_FUN_SEL_V << RTCIO_TOUCH_PAD14_FUN_SEL_S)
#define RTCIO_TOUCH_PAD14_FUN_SEL_V  0x00000003
#define RTCIO_TOUCH_PAD14_FUN_SEL_S  17

/* RTCIO_TOUCH_PAD14_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 0: no sleep mode; 1: enable sleep mode.
 */

#define RTCIO_TOUCH_PAD14_SLP_SEL    (BIT(16))
#define RTCIO_TOUCH_PAD14_SLP_SEL_M  (RTCIO_TOUCH_PAD14_SLP_SEL_V << RTCIO_TOUCH_PAD14_SLP_SEL_S)
#define RTCIO_TOUCH_PAD14_SLP_SEL_V  0x00000001
#define RTCIO_TOUCH_PAD14_SLP_SEL_S  16

/* RTCIO_TOUCH_PAD14_SLP_IE : R/W; bitpos: [15]; default: 0;
 * Input enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD14_SLP_IE    (BIT(15))
#define RTCIO_TOUCH_PAD14_SLP_IE_M  (RTCIO_TOUCH_PAD14_SLP_IE_V << RTCIO_TOUCH_PAD14_SLP_IE_S)
#define RTCIO_TOUCH_PAD14_SLP_IE_V  0x00000001
#define RTCIO_TOUCH_PAD14_SLP_IE_S  15

/* RTCIO_TOUCH_PAD14_SLP_OE : R/W; bitpos: [14]; default: 0;
 * Output enable in sleep mode.
 */

#define RTCIO_TOUCH_PAD14_SLP_OE    (BIT(14))
#define RTCIO_TOUCH_PAD14_SLP_OE_M  (RTCIO_TOUCH_PAD14_SLP_OE_V << RTCIO_TOUCH_PAD14_SLP_OE_S)
#define RTCIO_TOUCH_PAD14_SLP_OE_V  0x00000001
#define RTCIO_TOUCH_PAD14_SLP_OE_S  14

/* RTCIO_TOUCH_PAD14_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_TOUCH_PAD14_FUN_IE    (BIT(13))
#define RTCIO_TOUCH_PAD14_FUN_IE_M  (RTCIO_TOUCH_PAD14_FUN_IE_V << RTCIO_TOUCH_PAD14_FUN_IE_S)
#define RTCIO_TOUCH_PAD14_FUN_IE_V  0x00000001
#define RTCIO_TOUCH_PAD14_FUN_IE_S  13

/* RTCIO_XTAL_32P_PAD_REG register
 * 32KHz crystal P-pad configuration register
 */

#define RTCIO_XTAL_32P_PAD_REG (DR_REG_RTCIO_BASE + 0xc0)

/* RTCIO_X32P_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_X32P_DRV    0x00000003
#define RTCIO_X32P_DRV_M  (RTCIO_X32P_DRV_V << RTCIO_X32P_DRV_S)
#define RTCIO_X32P_DRV_V  0x00000003
#define RTCIO_X32P_DRV_S  29

/* RTCIO_X32P_RDE : R/W; bitpos: [28]; default: 0;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_X32P_RDE    (BIT(28))
#define RTCIO_X32P_RDE_M  (RTCIO_X32P_RDE_V << RTCIO_X32P_RDE_S)
#define RTCIO_X32P_RDE_V  0x00000001
#define RTCIO_X32P_RDE_S  28

/* RTCIO_X32P_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_X32P_RUE    (BIT(27))
#define RTCIO_X32P_RUE_M  (RTCIO_X32P_RUE_V << RTCIO_X32P_RUE_S)
#define RTCIO_X32P_RUE_V  0x00000001
#define RTCIO_X32P_RUE_S  27

/* RTCIO_X32P_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * 1: use RTC GPIO,0: use digital GPIO
 */

#define RTCIO_X32P_MUX_SEL    (BIT(19))
#define RTCIO_X32P_MUX_SEL_M  (RTCIO_X32P_MUX_SEL_V << RTCIO_X32P_MUX_SEL_S)
#define RTCIO_X32P_MUX_SEL_V  0x00000001
#define RTCIO_X32P_MUX_SEL_S  19

/* RTCIO_X32P_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_X32P_FUN_SEL    0x00000003
#define RTCIO_X32P_FUN_SEL_M  (RTCIO_X32P_FUN_SEL_V << RTCIO_X32P_FUN_SEL_S)
#define RTCIO_X32P_FUN_SEL_V  0x00000003
#define RTCIO_X32P_FUN_SEL_S  17

/* RTCIO_X32P_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 1: enable sleep mode, 0: no sleep mode
 */

#define RTCIO_X32P_SLP_SEL    (BIT(16))
#define RTCIO_X32P_SLP_SEL_M  (RTCIO_X32P_SLP_SEL_V << RTCIO_X32P_SLP_SEL_S)
#define RTCIO_X32P_SLP_SEL_V  0x00000001
#define RTCIO_X32P_SLP_SEL_S  16

/* RTCIO_X32P_SLP_IE : R/W; bitpos: [15]; default: 0;
 * input enable in sleep mode
 */

#define RTCIO_X32P_SLP_IE    (BIT(15))
#define RTCIO_X32P_SLP_IE_M  (RTCIO_X32P_SLP_IE_V << RTCIO_X32P_SLP_IE_S)
#define RTCIO_X32P_SLP_IE_V  0x00000001
#define RTCIO_X32P_SLP_IE_S  15

/* RTCIO_X32P_SLP_OE : R/W; bitpos: [14]; default: 0;
 * output enable in sleep mode
 */

#define RTCIO_X32P_SLP_OE    (BIT(14))
#define RTCIO_X32P_SLP_OE_M  (RTCIO_X32P_SLP_OE_V << RTCIO_X32P_SLP_OE_S)
#define RTCIO_X32P_SLP_OE_V  0x00000001
#define RTCIO_X32P_SLP_OE_S  14

/* RTCIO_X32P_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_X32P_FUN_IE    (BIT(13))
#define RTCIO_X32P_FUN_IE_M  (RTCIO_X32P_FUN_IE_V << RTCIO_X32P_FUN_IE_S)
#define RTCIO_X32P_FUN_IE_V  0x00000001
#define RTCIO_X32P_FUN_IE_S  13

/* RTCIO_XTAL_32N_PAD_REG register
 * 32KHz crystal N-pad configuration register
 */

#define RTCIO_XTAL_32N_PAD_REG (DR_REG_RTCIO_BASE + 0xc4)

/* RTCIO_X32N_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_X32N_DRV    0x00000003
#define RTCIO_X32N_DRV_M  (RTCIO_X32N_DRV_V << RTCIO_X32N_DRV_S)
#define RTCIO_X32N_DRV_V  0x00000003
#define RTCIO_X32N_DRV_S  29

/* RTCIO_X32N_RDE : R/W; bitpos: [28]; default: 0;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_X32N_RDE    (BIT(28))
#define RTCIO_X32N_RDE_M  (RTCIO_X32N_RDE_V << RTCIO_X32N_RDE_S)
#define RTCIO_X32N_RDE_V  0x00000001
#define RTCIO_X32N_RDE_S  28

/* RTCIO_X32N_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_X32N_RUE    (BIT(27))
#define RTCIO_X32N_RUE_M  (RTCIO_X32N_RUE_V << RTCIO_X32N_RUE_S)
#define RTCIO_X32N_RUE_V  0x00000001
#define RTCIO_X32N_RUE_S  27

/* RTCIO_X32N_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * 1: use RTC GPIO,0: use digital GPIO
 */

#define RTCIO_X32N_MUX_SEL    (BIT(19))
#define RTCIO_X32N_MUX_SEL_M  (RTCIO_X32N_MUX_SEL_V << RTCIO_X32N_MUX_SEL_S)
#define RTCIO_X32N_MUX_SEL_V  0x00000001
#define RTCIO_X32N_MUX_SEL_S  19

/* RTCIO_X32N_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_X32N_FUN_SEL    0x00000003
#define RTCIO_X32N_FUN_SEL_M  (RTCIO_X32N_FUN_SEL_V << RTCIO_X32N_FUN_SEL_S)
#define RTCIO_X32N_FUN_SEL_V  0x00000003
#define RTCIO_X32N_FUN_SEL_S  17

/* RTCIO_X32N_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 1: enable sleep mode, 0: no sleep mode
 */

#define RTCIO_X32N_SLP_SEL    (BIT(16))
#define RTCIO_X32N_SLP_SEL_M  (RTCIO_X32N_SLP_SEL_V << RTCIO_X32N_SLP_SEL_S)
#define RTCIO_X32N_SLP_SEL_V  0x00000001
#define RTCIO_X32N_SLP_SEL_S  16

/* RTCIO_X32N_SLP_IE : R/W; bitpos: [15]; default: 0;
 * input enable in sleep mode
 */

#define RTCIO_X32N_SLP_IE    (BIT(15))
#define RTCIO_X32N_SLP_IE_M  (RTCIO_X32N_SLP_IE_V << RTCIO_X32N_SLP_IE_S)
#define RTCIO_X32N_SLP_IE_V  0x00000001
#define RTCIO_X32N_SLP_IE_S  15

/* RTCIO_X32N_SLP_OE : R/W; bitpos: [14]; default: 0;
 * output enable in sleep mode
 */

#define RTCIO_X32N_SLP_OE    (BIT(14))
#define RTCIO_X32N_SLP_OE_M  (RTCIO_X32N_SLP_OE_V << RTCIO_X32N_SLP_OE_S)
#define RTCIO_X32N_SLP_OE_V  0x00000001
#define RTCIO_X32N_SLP_OE_S  14

/* RTCIO_X32N_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_X32N_FUN_IE    (BIT(13))
#define RTCIO_X32N_FUN_IE_M  (RTCIO_X32N_FUN_IE_V << RTCIO_X32N_FUN_IE_S)
#define RTCIO_X32N_FUN_IE_V  0x00000001
#define RTCIO_X32N_FUN_IE_S  13

/* RTCIO_PAD_DAC1_REG register
 * DAC1 configuration register
 */

#define RTCIO_PAD_DAC1_REG (DR_REG_RTCIO_BASE + 0xc8)

/* RTCIO_PDAC1_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_PDAC1_DRV    0x00000003
#define RTCIO_PDAC1_DRV_M  (RTCIO_PDAC1_DRV_V << RTCIO_PDAC1_DRV_S)
#define RTCIO_PDAC1_DRV_V  0x00000003
#define RTCIO_PDAC1_DRV_S  29

/* RTCIO_PDAC1_RDE : R/W; bitpos: [28]; default: 0;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_PDAC1_RDE    (BIT(28))
#define RTCIO_PDAC1_RDE_M  (RTCIO_PDAC1_RDE_V << RTCIO_PDAC1_RDE_S)
#define RTCIO_PDAC1_RDE_V  0x00000001
#define RTCIO_PDAC1_RDE_S  28

/* RTCIO_PDAC1_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_PDAC1_RUE    (BIT(27))
#define RTCIO_PDAC1_RUE_M  (RTCIO_PDAC1_RUE_V << RTCIO_PDAC1_RUE_S)
#define RTCIO_PDAC1_RUE_V  0x00000001
#define RTCIO_PDAC1_RUE_S  27

/* RTCIO_PDAC1_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * 1: use RTC GPIO, 0: use digital GPIO
 */

#define RTCIO_PDAC1_MUX_SEL    (BIT(19))
#define RTCIO_PDAC1_MUX_SEL_M  (RTCIO_PDAC1_MUX_SEL_V << RTCIO_PDAC1_MUX_SEL_S)
#define RTCIO_PDAC1_MUX_SEL_V  0x00000001
#define RTCIO_PDAC1_MUX_SEL_S  19

/* RTCIO_PDAC1_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * DAC_1 function selection.
 */

#define RTCIO_PDAC1_FUN_SEL    0x00000003
#define RTCIO_PDAC1_FUN_SEL_M  (RTCIO_PDAC1_FUN_SEL_V << RTCIO_PDAC1_FUN_SEL_S)
#define RTCIO_PDAC1_FUN_SEL_V  0x00000003
#define RTCIO_PDAC1_FUN_SEL_S  17

/* RTCIO_PDAC1_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 1: enable sleep mode, 0: no sleep mode
 */

#define RTCIO_PDAC1_SLP_SEL    (BIT(16))
#define RTCIO_PDAC1_SLP_SEL_M  (RTCIO_PDAC1_SLP_SEL_V << RTCIO_PDAC1_SLP_SEL_S)
#define RTCIO_PDAC1_SLP_SEL_V  0x00000001
#define RTCIO_PDAC1_SLP_SEL_S  16

/* RTCIO_PDAC1_SLP_IE : R/W; bitpos: [15]; default: 0;
 * input enable in sleep mode
 */

#define RTCIO_PDAC1_SLP_IE    (BIT(15))
#define RTCIO_PDAC1_SLP_IE_M  (RTCIO_PDAC1_SLP_IE_V << RTCIO_PDAC1_SLP_IE_S)
#define RTCIO_PDAC1_SLP_IE_V  0x00000001
#define RTCIO_PDAC1_SLP_IE_S  15

/* RTCIO_PDAC1_SLP_OE : R/W; bitpos: [14]; default: 0;
 * output enable in sleep mode
 */

#define RTCIO_PDAC1_SLP_OE    (BIT(14))
#define RTCIO_PDAC1_SLP_OE_M  (RTCIO_PDAC1_SLP_OE_V << RTCIO_PDAC1_SLP_OE_S)
#define RTCIO_PDAC1_SLP_OE_V  0x00000001
#define RTCIO_PDAC1_SLP_OE_S  14

/* RTCIO_PDAC1_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_PDAC1_FUN_IE    (BIT(13))
#define RTCIO_PDAC1_FUN_IE_M  (RTCIO_PDAC1_FUN_IE_V << RTCIO_PDAC1_FUN_IE_S)
#define RTCIO_PDAC1_FUN_IE_V  0x00000001
#define RTCIO_PDAC1_FUN_IE_S  13

/* RTCIO_PDAC1_DAC_XPD_FORCE : R/W; bitpos: [12]; default: 0;
 * 1: use RTCIO_PDAC1_XPD_DAC to control DAC_1 output; 0: use SAR ADC FSM to
 * control DAC_1 output.
 */

#define RTCIO_PDAC1_DAC_XPD_FORCE    (BIT(12))
#define RTCIO_PDAC1_DAC_XPD_FORCE_M  (RTCIO_PDAC1_DAC_XPD_FORCE_V << RTCIO_PDAC1_DAC_XPD_FORCE_S)
#define RTCIO_PDAC1_DAC_XPD_FORCE_V  0x00000001
#define RTCIO_PDAC1_DAC_XPD_FORCE_S  12

/* RTCIO_PDAC1_XPD_DAC : R/W; bitpos: [11]; default: 0;
 * When RTCIO_PDAC1_DAC_XPD_FORCE is set to 1, 1: enable DAC_1 output; 0:
 * disable DAC_1 output.
 */

#define RTCIO_PDAC1_XPD_DAC    (BIT(11))
#define RTCIO_PDAC1_XPD_DAC_M  (RTCIO_PDAC1_XPD_DAC_V << RTCIO_PDAC1_XPD_DAC_S)
#define RTCIO_PDAC1_XPD_DAC_V  0x00000001
#define RTCIO_PDAC1_XPD_DAC_S  11

/* RTCIO_PDAC1_DAC : R/W; bitpos: [10:3]; default: 0;
 * Configure DAC_1 output when RTCIO_PDAC1_DAC_XPD_FORCE is set to 1.
 */

#define RTCIO_PDAC1_DAC    0x000000FF
#define RTCIO_PDAC1_DAC_M  (RTCIO_PDAC1_DAC_V << RTCIO_PDAC1_DAC_S)
#define RTCIO_PDAC1_DAC_V  0x000000FF
#define RTCIO_PDAC1_DAC_S  3

/* RTCIO_PAD_DAC2_REG register
 * DAC2 configuration register
 */

#define RTCIO_PAD_DAC2_REG (DR_REG_RTCIO_BASE + 0xcc)

/* RTCIO_PDAC2_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_PDAC2_DRV    0x00000003
#define RTCIO_PDAC2_DRV_M  (RTCIO_PDAC2_DRV_V << RTCIO_PDAC2_DRV_S)
#define RTCIO_PDAC2_DRV_V  0x00000003
#define RTCIO_PDAC2_DRV_S  29

/* RTCIO_PDAC2_RDE : R/W; bitpos: [28]; default: 0;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_PDAC2_RDE    (BIT(28))
#define RTCIO_PDAC2_RDE_M  (RTCIO_PDAC2_RDE_V << RTCIO_PDAC2_RDE_S)
#define RTCIO_PDAC2_RDE_V  0x00000001
#define RTCIO_PDAC2_RDE_S  28

/* RTCIO_PDAC2_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_PDAC2_RUE    (BIT(27))
#define RTCIO_PDAC2_RUE_M  (RTCIO_PDAC2_RUE_V << RTCIO_PDAC2_RUE_S)
#define RTCIO_PDAC2_RUE_V  0x00000001
#define RTCIO_PDAC2_RUE_S  27

/* RTCIO_PDAC2_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * 1: use RTC GPIO, 0: use digital GPIO
 */

#define RTCIO_PDAC2_MUX_SEL    (BIT(19))
#define RTCIO_PDAC2_MUX_SEL_M  (RTCIO_PDAC2_MUX_SEL_V << RTCIO_PDAC2_MUX_SEL_S)
#define RTCIO_PDAC2_MUX_SEL_V  0x00000001
#define RTCIO_PDAC2_MUX_SEL_S  19

/* RTCIO_PDAC2_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * DAC_2 function selection.
 */

#define RTCIO_PDAC2_FUN_SEL    0x00000003
#define RTCIO_PDAC2_FUN_SEL_M  (RTCIO_PDAC2_FUN_SEL_V << RTCIO_PDAC2_FUN_SEL_S)
#define RTCIO_PDAC2_FUN_SEL_V  0x00000003
#define RTCIO_PDAC2_FUN_SEL_S  17

/* RTCIO_PDAC2_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 1: enable sleep mode, 0: no sleep mode
 */

#define RTCIO_PDAC2_SLP_SEL    (BIT(16))
#define RTCIO_PDAC2_SLP_SEL_M  (RTCIO_PDAC2_SLP_SEL_V << RTCIO_PDAC2_SLP_SEL_S)
#define RTCIO_PDAC2_SLP_SEL_V  0x00000001
#define RTCIO_PDAC2_SLP_SEL_S  16

/* RTCIO_PDAC2_SLP_IE : R/W; bitpos: [15]; default: 0;
 * input enable in sleep mode
 */

#define RTCIO_PDAC2_SLP_IE    (BIT(15))
#define RTCIO_PDAC2_SLP_IE_M  (RTCIO_PDAC2_SLP_IE_V << RTCIO_PDAC2_SLP_IE_S)
#define RTCIO_PDAC2_SLP_IE_V  0x00000001
#define RTCIO_PDAC2_SLP_IE_S  15

/* RTCIO_PDAC2_SLP_OE : R/W; bitpos: [14]; default: 0;
 * output enable in sleep mode
 */

#define RTCIO_PDAC2_SLP_OE    (BIT(14))
#define RTCIO_PDAC2_SLP_OE_M  (RTCIO_PDAC2_SLP_OE_V << RTCIO_PDAC2_SLP_OE_S)
#define RTCIO_PDAC2_SLP_OE_V  0x00000001
#define RTCIO_PDAC2_SLP_OE_S  14

/* RTCIO_PDAC2_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_PDAC2_FUN_IE    (BIT(13))
#define RTCIO_PDAC2_FUN_IE_M  (RTCIO_PDAC2_FUN_IE_V << RTCIO_PDAC2_FUN_IE_S)
#define RTCIO_PDAC2_FUN_IE_V  0x00000001
#define RTCIO_PDAC2_FUN_IE_S  13

/* RTCIO_PDAC2_DAC_XPD_FORCE : R/W; bitpos: [12]; default: 0;
 * 1: use RTCIO_PDAC2_XPD_DAC to control DAC_2 output; 0: use SAR ADC FSM to
 * control DAC_2 output.
 */

#define RTCIO_PDAC2_DAC_XPD_FORCE    (BIT(12))
#define RTCIO_PDAC2_DAC_XPD_FORCE_M  (RTCIO_PDAC2_DAC_XPD_FORCE_V << RTCIO_PDAC2_DAC_XPD_FORCE_S)
#define RTCIO_PDAC2_DAC_XPD_FORCE_V  0x00000001
#define RTCIO_PDAC2_DAC_XPD_FORCE_S  12

/* RTCIO_PDAC2_XPD_DAC : R/W; bitpos: [11]; default: 0;
 * When RTCIO_PDAC2_DAC_XPD_FORCE is set to 1, 1: enable DAC_2 output; 0:
 * disable DAC_2 output.
 */

#define RTCIO_PDAC2_XPD_DAC    (BIT(11))
#define RTCIO_PDAC2_XPD_DAC_M  (RTCIO_PDAC2_XPD_DAC_V << RTCIO_PDAC2_XPD_DAC_S)
#define RTCIO_PDAC2_XPD_DAC_V  0x00000001
#define RTCIO_PDAC2_XPD_DAC_S  11

/* RTCIO_PDAC2_DAC : R/W; bitpos: [10:3]; default: 0;
 * Configure DAC_2 output when RTCIO_PDAC2_DAC_XPD_FORCE is set to 1.
 */

#define RTCIO_PDAC2_DAC    0x000000FF
#define RTCIO_PDAC2_DAC_M  (RTCIO_PDAC2_DAC_V << RTCIO_PDAC2_DAC_S)
#define RTCIO_PDAC2_DAC_V  0x000000FF
#define RTCIO_PDAC2_DAC_S  3

/* RTCIO_RTC_PAD19_REG register
 * Touch pad 19 configuration register
 */

#define RTCIO_RTC_PAD19_REG (DR_REG_RTCIO_BASE + 0xd0)

/* RTCIO_RTC_PAD19_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_RTC_PAD19_DRV    0x00000003
#define RTCIO_RTC_PAD19_DRV_M  (RTCIO_RTC_PAD19_DRV_V << RTCIO_RTC_PAD19_DRV_S)
#define RTCIO_RTC_PAD19_DRV_V  0x00000003
#define RTCIO_RTC_PAD19_DRV_S  29

/* RTCIO_RTC_PAD19_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_RTC_PAD19_RDE    (BIT(28))
#define RTCIO_RTC_PAD19_RDE_M  (RTCIO_RTC_PAD19_RDE_V << RTCIO_RTC_PAD19_RDE_S)
#define RTCIO_RTC_PAD19_RDE_V  0x00000001
#define RTCIO_RTC_PAD19_RDE_S  28

/* RTCIO_RTC_PAD19_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_RTC_PAD19_RUE    (BIT(27))
#define RTCIO_RTC_PAD19_RUE_M  (RTCIO_RTC_PAD19_RUE_V << RTCIO_RTC_PAD19_RUE_S)
#define RTCIO_RTC_PAD19_RUE_V  0x00000001
#define RTCIO_RTC_PAD19_RUE_S  27

/* RTCIO_RTC_PAD19_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * 1: use RTC GPIO, 0: use digital GPIO
 */

#define RTCIO_RTC_PAD19_MUX_SEL    (BIT(19))
#define RTCIO_RTC_PAD19_MUX_SEL_M  (RTCIO_RTC_PAD19_MUX_SEL_V << RTCIO_RTC_PAD19_MUX_SEL_S)
#define RTCIO_RTC_PAD19_MUX_SEL_V  0x00000001
#define RTCIO_RTC_PAD19_MUX_SEL_S  19

/* RTCIO_RTC_PAD19_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_RTC_PAD19_FUN_SEL    0x00000003
#define RTCIO_RTC_PAD19_FUN_SEL_M  (RTCIO_RTC_PAD19_FUN_SEL_V << RTCIO_RTC_PAD19_FUN_SEL_S)
#define RTCIO_RTC_PAD19_FUN_SEL_V  0x00000003
#define RTCIO_RTC_PAD19_FUN_SEL_S  17

/* RTCIO_RTC_PAD19_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 1: enable sleep mode, 0: no sleep mode
 */

#define RTCIO_RTC_PAD19_SLP_SEL    (BIT(16))
#define RTCIO_RTC_PAD19_SLP_SEL_M  (RTCIO_RTC_PAD19_SLP_SEL_V << RTCIO_RTC_PAD19_SLP_SEL_S)
#define RTCIO_RTC_PAD19_SLP_SEL_V  0x00000001
#define RTCIO_RTC_PAD19_SLP_SEL_S  16

/* RTCIO_RTC_PAD19_SLP_IE : R/W; bitpos: [15]; default: 0;
 * input enable in sleep mode
 */

#define RTCIO_RTC_PAD19_SLP_IE    (BIT(15))
#define RTCIO_RTC_PAD19_SLP_IE_M  (RTCIO_RTC_PAD19_SLP_IE_V << RTCIO_RTC_PAD19_SLP_IE_S)
#define RTCIO_RTC_PAD19_SLP_IE_V  0x00000001
#define RTCIO_RTC_PAD19_SLP_IE_S  15

/* RTCIO_RTC_PAD19_SLP_OE : R/W; bitpos: [14]; default: 0;
 * output enable in sleep mode
 */

#define RTCIO_RTC_PAD19_SLP_OE    (BIT(14))
#define RTCIO_RTC_PAD19_SLP_OE_M  (RTCIO_RTC_PAD19_SLP_OE_V << RTCIO_RTC_PAD19_SLP_OE_S)
#define RTCIO_RTC_PAD19_SLP_OE_V  0x00000001
#define RTCIO_RTC_PAD19_SLP_OE_S  14

/* RTCIO_RTC_PAD19_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_RTC_PAD19_FUN_IE    (BIT(13))
#define RTCIO_RTC_PAD19_FUN_IE_M  (RTCIO_RTC_PAD19_FUN_IE_V << RTCIO_RTC_PAD19_FUN_IE_S)
#define RTCIO_RTC_PAD19_FUN_IE_V  0x00000001
#define RTCIO_RTC_PAD19_FUN_IE_S  13

/* RTCIO_RTC_PAD20_REG register
 * Touch pad 20 configuration register
 */

#define RTCIO_RTC_PAD20_REG (DR_REG_RTCIO_BASE + 0xd4)

/* RTCIO_RTC_PAD20_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_RTC_PAD20_DRV    0x00000003
#define RTCIO_RTC_PAD20_DRV_M  (RTCIO_RTC_PAD20_DRV_V << RTCIO_RTC_PAD20_DRV_S)
#define RTCIO_RTC_PAD20_DRV_V  0x00000003
#define RTCIO_RTC_PAD20_DRV_S  29

/* RTCIO_RTC_PAD20_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_RTC_PAD20_RDE    (BIT(28))
#define RTCIO_RTC_PAD20_RDE_M  (RTCIO_RTC_PAD20_RDE_V << RTCIO_RTC_PAD20_RDE_S)
#define RTCIO_RTC_PAD20_RDE_V  0x00000001
#define RTCIO_RTC_PAD20_RDE_S  28

/* RTCIO_RTC_PAD20_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_RTC_PAD20_RUE    (BIT(27))
#define RTCIO_RTC_PAD20_RUE_M  (RTCIO_RTC_PAD20_RUE_V << RTCIO_RTC_PAD20_RUE_S)
#define RTCIO_RTC_PAD20_RUE_V  0x00000001
#define RTCIO_RTC_PAD20_RUE_S  27

/* RTCIO_RTC_PAD20_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * 1: use RTC GPIO, 0: use digital GPIO
 */

#define RTCIO_RTC_PAD20_MUX_SEL    (BIT(19))
#define RTCIO_RTC_PAD20_MUX_SEL_M  (RTCIO_RTC_PAD20_MUX_SEL_V << RTCIO_RTC_PAD20_MUX_SEL_S)
#define RTCIO_RTC_PAD20_MUX_SEL_V  0x00000001
#define RTCIO_RTC_PAD20_MUX_SEL_S  19

/* RTCIO_RTC_PAD20_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_RTC_PAD20_FUN_SEL    0x00000003
#define RTCIO_RTC_PAD20_FUN_SEL_M  (RTCIO_RTC_PAD20_FUN_SEL_V << RTCIO_RTC_PAD20_FUN_SEL_S)
#define RTCIO_RTC_PAD20_FUN_SEL_V  0x00000003
#define RTCIO_RTC_PAD20_FUN_SEL_S  17

/* RTCIO_RTC_PAD20_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 1: enable sleep mode, 0: no sleep mode
 */

#define RTCIO_RTC_PAD20_SLP_SEL    (BIT(16))
#define RTCIO_RTC_PAD20_SLP_SEL_M  (RTCIO_RTC_PAD20_SLP_SEL_V << RTCIO_RTC_PAD20_SLP_SEL_S)
#define RTCIO_RTC_PAD20_SLP_SEL_V  0x00000001
#define RTCIO_RTC_PAD20_SLP_SEL_S  16

/* RTCIO_RTC_PAD20_SLP_IE : R/W; bitpos: [15]; default: 0;
 * input enable in sleep mode
 */

#define RTCIO_RTC_PAD20_SLP_IE    (BIT(15))
#define RTCIO_RTC_PAD20_SLP_IE_M  (RTCIO_RTC_PAD20_SLP_IE_V << RTCIO_RTC_PAD20_SLP_IE_S)
#define RTCIO_RTC_PAD20_SLP_IE_V  0x00000001
#define RTCIO_RTC_PAD20_SLP_IE_S  15

/* RTCIO_RTC_PAD20_SLP_OE : R/W; bitpos: [14]; default: 0;
 * output enable in sleep mode
 */

#define RTCIO_RTC_PAD20_SLP_OE    (BIT(14))
#define RTCIO_RTC_PAD20_SLP_OE_M  (RTCIO_RTC_PAD20_SLP_OE_V << RTCIO_RTC_PAD20_SLP_OE_S)
#define RTCIO_RTC_PAD20_SLP_OE_V  0x00000001
#define RTCIO_RTC_PAD20_SLP_OE_S  14

/* RTCIO_RTC_PAD20_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_RTC_PAD20_FUN_IE    (BIT(13))
#define RTCIO_RTC_PAD20_FUN_IE_M  (RTCIO_RTC_PAD20_FUN_IE_V << RTCIO_RTC_PAD20_FUN_IE_S)
#define RTCIO_RTC_PAD20_FUN_IE_V  0x00000001
#define RTCIO_RTC_PAD20_FUN_IE_S  13

/* RTCIO_RTC_PAD21_REG register
 * Touch pad 21 configuration register
 */

#define RTCIO_RTC_PAD21_REG (DR_REG_RTCIO_BASE + 0xd8)

/* RTCIO_RTC_PAD21_DRV : R/W; bitpos: [30:29]; default: 2;
 * Select the drive strength of the pad. 0: ~5 mA: 1: ~10 mA: 2: ~20 mA; 3:
 * ~40 mA.
 */

#define RTCIO_RTC_PAD21_DRV    0x00000003
#define RTCIO_RTC_PAD21_DRV_M  (RTCIO_RTC_PAD21_DRV_V << RTCIO_RTC_PAD21_DRV_S)
#define RTCIO_RTC_PAD21_DRV_V  0x00000003
#define RTCIO_RTC_PAD21_DRV_S  29

/* RTCIO_RTC_PAD21_RDE : R/W; bitpos: [28]; default: 1;
 * Pull-up enable of the pad. 1: internal pull-up enabled; 0: internal
 * pull-up disabled.
 */

#define RTCIO_RTC_PAD21_RDE    (BIT(28))
#define RTCIO_RTC_PAD21_RDE_M  (RTCIO_RTC_PAD21_RDE_V << RTCIO_RTC_PAD21_RDE_S)
#define RTCIO_RTC_PAD21_RDE_V  0x00000001
#define RTCIO_RTC_PAD21_RDE_S  28

/* RTCIO_RTC_PAD21_RUE : R/W; bitpos: [27]; default: 0;
 * Pull-down enable of the pad. 1: internal pull-down enabled, 0: internal
 * pull-down disabled.
 */

#define RTCIO_RTC_PAD21_RUE    (BIT(27))
#define RTCIO_RTC_PAD21_RUE_M  (RTCIO_RTC_PAD21_RUE_V << RTCIO_RTC_PAD21_RUE_S)
#define RTCIO_RTC_PAD21_RUE_V  0x00000001
#define RTCIO_RTC_PAD21_RUE_S  27

/* RTCIO_RTC_PAD21_MUX_SEL : R/W; bitpos: [19]; default: 0;
 * 1: use RTC GPIO, 0: use digital GPIO
 */

#define RTCIO_RTC_PAD21_MUX_SEL    (BIT(19))
#define RTCIO_RTC_PAD21_MUX_SEL_M  (RTCIO_RTC_PAD21_MUX_SEL_V << RTCIO_RTC_PAD21_MUX_SEL_S)
#define RTCIO_RTC_PAD21_MUX_SEL_V  0x00000001
#define RTCIO_RTC_PAD21_MUX_SEL_S  19

/* RTCIO_RTC_PAD21_FUN_SEL : R/W; bitpos: [18:17]; default: 0;
 * Function selection.
 */

#define RTCIO_RTC_PAD21_FUN_SEL    0x00000003
#define RTCIO_RTC_PAD21_FUN_SEL_M  (RTCIO_RTC_PAD21_FUN_SEL_V << RTCIO_RTC_PAD21_FUN_SEL_S)
#define RTCIO_RTC_PAD21_FUN_SEL_V  0x00000003
#define RTCIO_RTC_PAD21_FUN_SEL_S  17

/* RTCIO_RTC_PAD21_SLP_SEL : R/W; bitpos: [16]; default: 0;
 * 1: enable sleep mode, 0: no sleep mode
 */

#define RTCIO_RTC_PAD21_SLP_SEL    (BIT(16))
#define RTCIO_RTC_PAD21_SLP_SEL_M  (RTCIO_RTC_PAD21_SLP_SEL_V << RTCIO_RTC_PAD21_SLP_SEL_S)
#define RTCIO_RTC_PAD21_SLP_SEL_V  0x00000001
#define RTCIO_RTC_PAD21_SLP_SEL_S  16

/* RTCIO_RTC_PAD21_SLP_IE : R/W; bitpos: [15]; default: 0;
 * input enable in sleep mode
 */

#define RTCIO_RTC_PAD21_SLP_IE    (BIT(15))
#define RTCIO_RTC_PAD21_SLP_IE_M  (RTCIO_RTC_PAD21_SLP_IE_V << RTCIO_RTC_PAD21_SLP_IE_S)
#define RTCIO_RTC_PAD21_SLP_IE_V  0x00000001
#define RTCIO_RTC_PAD21_SLP_IE_S  15

/* RTCIO_RTC_PAD21_SLP_OE : R/W; bitpos: [14]; default: 0;
 * output enable in sleep mode
 */

#define RTCIO_RTC_PAD21_SLP_OE    (BIT(14))
#define RTCIO_RTC_PAD21_SLP_OE_M  (RTCIO_RTC_PAD21_SLP_OE_V << RTCIO_RTC_PAD21_SLP_OE_S)
#define RTCIO_RTC_PAD21_SLP_OE_V  0x00000001
#define RTCIO_RTC_PAD21_SLP_OE_S  14

/* RTCIO_RTC_PAD21_FUN_IE : R/W; bitpos: [13]; default: 0;
 * Input enable in normal execution.
 */

#define RTCIO_RTC_PAD21_FUN_IE    (BIT(13))
#define RTCIO_RTC_PAD21_FUN_IE_M  (RTCIO_RTC_PAD21_FUN_IE_V << RTCIO_RTC_PAD21_FUN_IE_S)
#define RTCIO_RTC_PAD21_FUN_IE_V  0x00000001
#define RTCIO_RTC_PAD21_FUN_IE_S  13

/* RTCIO_EXT_WAKEUP0_REG register
 * External wake up configuration register
 */

#define RTCIO_EXT_WAKEUP0_REG (DR_REG_RTCIO_BASE + 0xdc)

/* RTCIO_EXT_WAKEUP0_SEL : R/W; bitpos: [31:27]; default: 0;
 * GPIO[0-17] can be used to wake up the chip when the chip is in the sleep
 * mode. This register prompts the pad source to wake up the chip when the
 * latter is indeep/light sleep mode.
 * 0: select GPIO0; 1: select GPIO2, etc
 */

#define RTCIO_EXT_WAKEUP0_SEL    0x0000001F
#define RTCIO_EXT_WAKEUP0_SEL_M  (RTCIO_EXT_WAKEUP0_SEL_V << RTCIO_EXT_WAKEUP0_SEL_S)
#define RTCIO_EXT_WAKEUP0_SEL_V  0x0000001F
#define RTCIO_EXT_WAKEUP0_SEL_S  27

/* RTCIO_XTL_EXT_CTR_REG register
 * Crystal power down enable GPIO source
 */

#define RTCIO_XTL_EXT_CTR_REG (DR_REG_RTCIO_BASE + 0xe0)

/* RTCIO_XTL_EXT_CTR_SEL : R/W; bitpos: [31:27]; default: 0;
 * Select the external crystal power down enable source to get into sleep
 * mode. 0: select GPIO0; 1: select GPIO1, etc. The input value on this pin
 * XOR RTC_CNTL_EXT_XTL_CONF_REG[30] is the crystal power down enable signal.
 */

#define RTCIO_XTL_EXT_CTR_SEL    0x0000001F
#define RTCIO_XTL_EXT_CTR_SEL_M  (RTCIO_XTL_EXT_CTR_SEL_V << RTCIO_XTL_EXT_CTR_SEL_S)
#define RTCIO_XTL_EXT_CTR_SEL_V  0x0000001F
#define RTCIO_XTL_EXT_CTR_SEL_S  27

/* RTCIO_SAR_I2C_IO_REG register
 * RTC I²C pad selection
 */

#define RTCIO_SAR_I2C_IO_REG (DR_REG_RTCIO_BASE + 0xe4)

/* RTCIO_SAR_I2C_SDA_SEL : R/W; bitpos: [31:30]; default: 0;
 * Selects a pad the RTC I2C SDA signal connects to. 0: use TOUCH PAD1; 1:
 * use TOUCH PAD3.
 */

#define RTCIO_SAR_I2C_SDA_SEL    0x00000003
#define RTCIO_SAR_I2C_SDA_SEL_M  (RTCIO_SAR_I2C_SDA_SEL_V << RTCIO_SAR_I2C_SDA_SEL_S)
#define RTCIO_SAR_I2C_SDA_SEL_V  0x00000003
#define RTCIO_SAR_I2C_SDA_SEL_S  30

/* RTCIO_SAR_I2C_SCL_SEL : R/W; bitpos: [29:28]; default: 0;
 * Selects a pad the RTC I2C SCL signal connects to. 0: use TOUCH PAD0; 1:
 * use TOUCH PAD2.
 */

#define RTCIO_SAR_I2C_SCL_SEL    0x00000003
#define RTCIO_SAR_I2C_SCL_SEL_M  (RTCIO_SAR_I2C_SCL_SEL_V << RTCIO_SAR_I2C_SCL_SEL_S)
#define RTCIO_SAR_I2C_SCL_SEL_V  0x00000003
#define RTCIO_SAR_I2C_SCL_SEL_S  28

/* RTCIO_SAR_DEBUG_BIT_SEL : R/W; bitpos: [27:23]; default: 0; */

#define RTCIO_SAR_DEBUG_BIT_SEL    0x0000001F
#define RTCIO_SAR_DEBUG_BIT_SEL_M  (RTCIO_SAR_DEBUG_BIT_SEL_V << RTCIO_SAR_DEBUG_BIT_SEL_S)
#define RTCIO_SAR_DEBUG_BIT_SEL_V  0x0000001F
#define RTCIO_SAR_DEBUG_BIT_SEL_S  23

/* RTCIO_RTC_IO_TOUCH_CTRL_REG register
 * Touch Control register
 */

#define RTCIO_RTC_IO_TOUCH_CTRL_REG (DR_REG_RTCIO_BASE + 0xe8)

/* RTCIO_IO_TOUCH_BUFMODE : R/W; bitpos: [4]; default: 0; */

#define RTCIO_IO_TOUCH_BUFMODE    (BIT(4))
#define RTCIO_IO_TOUCH_BUFMODE_M  (RTCIO_IO_TOUCH_BUFMODE_V << RTCIO_IO_TOUCH_BUFMODE_S)
#define RTCIO_IO_TOUCH_BUFMODE_V  0x00000001
#define RTCIO_IO_TOUCH_BUFMODE_S  4

/* RTCIO_IO_TOUCH_BUFSEL : R/W; bitpos: [3:0]; default: 0; */

#define RTCIO_IO_TOUCH_BUFSEL    0x0000000F
#define RTCIO_IO_TOUCH_BUFSEL_M  (RTCIO_IO_TOUCH_BUFSEL_V << RTCIO_IO_TOUCH_BUFSEL_S)
#define RTCIO_IO_TOUCH_BUFSEL_V  0x0000000F
#define RTCIO_IO_TOUCH_BUFSEL_S  0

/* RTCIO_RTC_IO_DATE_REG register
 * Version control register
 */

#define RTCIO_RTC_IO_DATE_REG (DR_REG_RTCIO_BASE + 0x1fc)

/* RTCIO_IO_DATE : R/W; bitpos: [27:0]; default: 28'h2101180;
 * Version control register
 */

#define RTCIO_IO_DATE    0x0FFFFFFF
#define RTCIO_IO_DATE_M  (RTCIO_IO_DATE_V << RTCIO_IO_DATE_S)
#define RTCIO_IO_DATE_V  0x0FFFFFFF
#define RTCIO_IO_DATE_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_RTC_IO_H */
