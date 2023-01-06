/****************************************************************************
 * arch/risc-v/src/esp32c6/esp32c6_attr.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C6_ESP32C6_ATTR_H
#define __ARCH_RISCV_SRC_ESP32C6_ESP32C6_ATTR_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Forces code into IRAM instead of flash */

#define IRAM_ATTR __attribute__((section(".iram1")))

/* Forces data into DRAM instead of flash */

#define DRAM_ATTR __attribute__((section(".dram1")))

/* Forces code into RTC fast memory */

#define RTC_IRAM_ATTR __attribute__((section(".rtc.text")))

/* Forces data into RTC slow memory
 * Any variable marked with this attribute will keep its value
 * during a deep sleep / wake cycle.
 */

#define RTC_DATA_ATTR __attribute__((section(".rtc.data")))

/* Forces read-only data into RTC slow memory
 * Makes constant data available to RTC wake stubs.
 */

#define RTC_RODATA_ATTR __attribute__((section(".rtc.rodata")))

#endif /* __ARCH_RISCV_SRC_ESP32C6_ESP32C6_ATTR_H */
