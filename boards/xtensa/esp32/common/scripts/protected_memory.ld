/****************************************************************************
 * boards/xtensa/esp32/common/scripts/protected_memory.ld
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * ESP32 Linker Script Memory Layout for Protected Mode
 *
 * This file describes the memory layout (memory blocks) as virtual
 * memory addresses.
 *
 * kernel-space.ld and user-space.ld contain output sections to link compiler
 * output into these memory blocks for the Kernel and User images,
 * respectively.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include "esp32_aliases.ld"

#define RTC_TIMER_RESERVE_RTC       (24)
#define RESERVE_RTC_MEM             (RTC_TIMER_RESERVE_RTC)
#define ESP_BOOTLOADER_RESERVE_RTC  (0x0)

MEMORY
{
  metadata (RX) :   org = 0x0, len = 0x18
  ROM (RX) :        org = ORIGIN(metadata) + LENGTH(metadata),
                    len = 0x100000 - ORIGIN(ROM)

  /* Below values assume the flash cache is on, and have the blocks this
   * uses subtracted from the length of the various regions. The 'data access
   * port' dram/drom regions map to the same iram/irom regions but are
   * connected to the data port of the CPU and e.g. allow bytewise access.
   */

  /* Physically located in SRAM0 */
  KIRAM_0 (RWX) : org = 0x40080000, len = 0x2000
  UIRAM (RWX)   : org = 0x40082000, len = 0xe000
  KIRAM_1 (RWX) : org = 0x40090000, len = 0x10000

  /* Flash mapped instruction data. */

  /* The 0x20 offset for the KIROM region is a convenience for the Kernel
   * binary image generation in Espressif Application Image format.
   * Flash cache has 64KB pages. The .bin file which is flashed to the chip
   * has a 0x18 byte file header, and each segment has a 0x08 byte segment
   * header. Setting this offset makes it simple to meet the flash cache MMU's
   * constraint that (paddr % 64KB == vaddr % 64KB).
   */

  KIROM (RX) : org = 0x400d0020, len = 0x330000 - 0x20
  UIROM (RX) : org = 0x40400000, len = 0x400000

  /* Shared data RAM, excluding memory reserved for ROM bss/data/stack.
   * Enabling Bluetooth & Trace Memory features in menuconfig will decrease
   * the amount of RAM available to the NuttX Kernel.
   */

#ifdef CONFIG_ESP32_USER_DATA_EXTMEM
  /* Physically located in External RAM */
  UDRAM (RW)   : org = 0x3f800000, len = 0x400000

  /* Physically located in SRAM2 */
  KDRAM_0 (RW) : org = 0x3ffb0000 + CONFIG_ESP32_BT_RESERVE_DRAM,
                 len = 0x30000 - CONFIG_ESP32_BT_RESERVE_DRAM

  /* Physically located in SRAM1 */
  KDRAM_1 (RW) : org = 0x3ffe0000, len = 0x20000
#else
  /* Physically located in SRAM2 */
  KDRAM_0 (RW) : org = 0x3ffb0000 + CONFIG_ESP32_BT_RESERVE_DRAM,
                 len = 0x18000 - CONFIG_ESP32_BT_RESERVE_DRAM
  UDRAM (RW)   : org = 0x3ffc8000, len = 0x18000

  /* Physically located in SRAM1 */
  KDRAM_1 (RW) : org = 0x3ffe0000, len = 0x20000
#endif

  /* Flash mapped constant data */

  /* See KIROM region documentation above for the meaning of the 0x20 offset.
   *
   * The 0x18 offset for the UDROM region is a convenience for the User
   * binary image generation following a custom image format, which defines
   * a "metadata" output section containing some information that the Kernel
   * needs for properly configuring the External Flash MMU when loading the
   * User application image.
   */

  KDROM (R) : org = 0x3f400020, len = 0x80000 - 0x20
  UDROM (R) : org = 0x3f480018, len = 0x80000 - ORIGIN(ROM)

  /* RTC fast memory (executable). Persists over deep sleep. */

  rtc_iram_seg (RWX) :   org = 0x400c0000, len = 0x2000

  /* RTC fast memory (same block as above, rtc_iram_seg), viewed from
   * data bus.
   */

  rtc_data_seg(RW) :     org = 0x3ff80000, len = 0x2000 - RESERVE_RTC_MEM

  /* RTC slow memory (data accessible). Persists over deep sleep.
   * Start of RTC slow memory is reserved for ULP co-processor code + data,
   * if enabled.
   */

  /* We reduced the size of rtc_iram_seg and rtc_data_seg by
   * ESP_BOOTLOADER_RESERVE_RTC value. It reserves the amount of RTC fast
   * memory that we use for this memory segment.
   * This segment is intended for keeping bootloader rtc data
   * (s_bootloader_retain_mem, when a Kconfig option is on). The aim of this
   * is to keep data that will not be moved around and have a fixed address.
   * org = 0x3ff80000 + 0x2000 - ESP_BOOTLOADER_RESERVE_RTC == SOC_RTC_DRAM_HIGH - sizeof(rtc_retain_mem_t)
   */

  rtc_fast_reserved_seg(RW) :        org = 0x3ff80000 + 0x2000 - ESP_BOOTLOADER_RESERVE_RTC, len = ESP_BOOTLOADER_RESERVE_RTC

  rtc_slow_seg (RW) :    org = 0x50000000 + CONFIG_ESP32_ULP_COPROC_RESERVE_MEM,
                         len = 0x1000 - CONFIG_ESP32_ULP_COPROC_RESERVE_MEM

  rtc_slow_reserved_seg(RW) : org = 0x50000000 + 0x2000 - RESERVE_RTC_MEM, len = RESERVE_RTC_MEM
}
