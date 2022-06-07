/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_region.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <stdint.h>

#include "riscv_internal.h"
#include "hardware/esp32c3_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
  /* BUILD_PROTECTED also makes use of the ESP32-C3 PMP (MPU) for isolating
   * the Kernel from the Userspace.
   */

#  error "ESP32C3_REGION_PROTECTION shall not be enabled with Protected Mode"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_region_protection
 *
 * Description:
 *   Configure the MPU to disable access to invalid memory regions.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Notes:
 * 1) ESP32-C3 CPU doesn't support overlapping PMP regions.
 * 2) Therefore, we use TOR (top of range) entries to map the whole address
 *    space, bottom to top.
 * 3) There are not enough entries to describe all the memory regions 100%
 *    accurately.
 * 4) This means some gaps (invalid memory) are accessible. Priority for
 *    extending regions to cover gaps is to extend read-only or read-execute
 *    regions or read-only regions only (executing unmapped addresses should
 *    always fault with invalid instruction, read-only means stores will
 *    correctly fault even if reads may return some invalid value).
 * 5) Entries are grouped in order with some static asserts to try and verify
 *    everything is correct.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_REGION_PROTECTION
void esp32c3_region_protection(void)
{
  const uintptr_t R   = PMPCFG_L | PMPCFG_R;
  const uintptr_t RW  = PMPCFG_L | PMPCFG_R | PMPCFG_W;
  const uintptr_t RX  = PMPCFG_L | PMPCFG_R | PMPCFG_X;
  const uintptr_t RWX = PMPCFG_L | PMPCFG_R | PMPCFG_W | PMPCFG_X;

  /* 1. Gap at bottom of address space */

  riscv_config_pmp_region(0, PMPCFG_A_TOR, SOC_DEBUG_LOW, 0);

  /* 2. Debug region */

  riscv_config_pmp_region(1, PMPCFG_A_TOR | RWX, SOC_DEBUG_HIGH, 0);
  static_assert(SOC_DEBUG_LOW < SOC_DEBUG_HIGH, "Invalid CPU debug region");

  /* 3. Gap between debug region & DROM (flash cache) */

  riscv_config_pmp_region(2, PMPCFG_A_TOR, SOC_DROM_LOW, 0);
  static_assert(SOC_DEBUG_HIGH < SOC_DROM_LOW, "Invalid PMP entry order");

  /* 4. DROM (flash cache)
   * 5. Gap between DROM & DRAM
   * Note: To save PMP entries these two are merged into one read-only region
   */

  riscv_config_pmp_region(3, PMPCFG_A_TOR | R, SOC_DRAM_LOW, 0);
  static_assert(SOC_DROM_LOW < SOC_DROM_HIGH, "Invalid DROM region");
  static_assert(SOC_DROM_HIGH < SOC_DRAM_LOW, "Invalid PMP entry order");

  /* 6. DRAM */

  riscv_config_pmp_region(4, PMPCFG_A_TOR | RW, SOC_DRAM_HIGH, 0);
  static_assert(SOC_DRAM_LOW < SOC_DRAM_HIGH, "Invalid DRAM region");

  /* 7. Gap between DRAM and Mask DROM
   * 8. Mask DROM
   * Note: to save PMP entries these two are merged into one read-only region
   */

  riscv_config_pmp_region(5, PMPCFG_A_TOR | R, SOC_DROM_MASK_HIGH, 0);
  static_assert(SOC_DRAM_HIGH < SOC_DROM_MASK_LOW,
                "Invalid PMP entry order");
  static_assert(SOC_DROM_MASK_LOW < SOC_DROM_MASK_HIGH,
                "Invalid mask DROM region");

  /* 9. Gap between mask DROM and mask IROM
   * 10. Mask IROM
   * Note: to save PMP entries these two are merged into one RX region
   */

  riscv_config_pmp_region(6, PMPCFG_A_TOR | RX, SOC_IROM_MASK_HIGH, 0);
  static_assert(SOC_DROM_MASK_HIGH < SOC_IROM_MASK_LOW,
                "Invalid PMP entry order");
  static_assert(SOC_IROM_MASK_LOW < SOC_IROM_MASK_HIGH,
                "Invalid mask IROM region");

  /* 11. Gap between mask IROM & IRAM */

  riscv_config_pmp_region(7, PMPCFG_A_TOR, SOC_IRAM_LOW, 0);
  static_assert(SOC_IROM_MASK_HIGH < SOC_IRAM_LOW,
                "Invalid PMP entry order");

  /* 12. IRAM */

  riscv_config_pmp_region(8, PMPCFG_A_TOR | RWX, SOC_IRAM_HIGH, 0);
  static_assert(SOC_IRAM_LOW < SOC_IRAM_HIGH, "Invalid IRAM region");

  /* 13. Gap between IRAM and IROM
   * 14. IROM (flash cache)
   * Note: to save PMP entries these two are merged into one RX region
   */

  riscv_config_pmp_region(9, PMPCFG_A_TOR | RX, SOC_IROM_HIGH, 0);
  static_assert(SOC_IRAM_HIGH < SOC_IROM_LOW, "Invalid PMP entry order");
  static_assert(SOC_IROM_LOW < SOC_IROM_HIGH, "Invalid IROM region");

  /* 15. Gap between IROM & RTC slow memory */

  riscv_config_pmp_region(10, PMPCFG_A_TOR, SOC_RTC_RAM_LOW, 0);
  static_assert(SOC_IROM_HIGH < SOC_RTC_RAM_LOW, "Invalid PMP entry order");

  /* 16. RTC fast memory */

  riscv_config_pmp_region(11, PMPCFG_A_TOR | RWX, SOC_RTC_RAM_HIGH, 0);
  static_assert(SOC_RTC_RAM_LOW < SOC_RTC_RAM_HIGH,
                "Invalid RTC IRAM region");

  /* 17. Gap between RTC fast memory & peripheral addresses */

  riscv_config_pmp_region(12, PMPCFG_A_TOR, SOC_PERIPHERAL_LOW, 0);
  static_assert(SOC_RTC_RAM_HIGH < SOC_PERIPHERAL_LOW,
                "Invalid PMP entry order");

  /* 18. Peripheral addresses */

  riscv_config_pmp_region(13, PMPCFG_A_TOR | RW, SOC_PERIPHERAL_HIGH, 0);
  static_assert(SOC_PERIPHERAL_LOW < SOC_PERIPHERAL_HIGH,
                "Invalid peripheral region");

  /* 19. End of address space */

  riscv_config_pmp_region(14, PMPCFG_A_TOR, UINT32_MAX, 0);
  riscv_config_pmp_region(15, PMPCFG_A_NA4, UINT32_MAX, 0);
}
#endif
