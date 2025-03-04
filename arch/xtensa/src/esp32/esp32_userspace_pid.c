/****************************************************************************
 * arch/xtensa/src/esp32/esp32_userspace_pid.c
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

#include "esp_attr.h"

#include "hal/mmu_hal.h"
#include "hal/mmu_ll.h"
#include "hal/cache_ll.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_userspace_init_for_pid
 *
 * Description:
 *   A helper for esp32_userspace.
 *   Separated to avoid conflicts between esp hal vs nuttx.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void
esp32_userspace_init_for_pid(void)
{
  int i;

  /* Xtensa CPU does speculative load/store on VAddr1/2/3 when connected
   * to cache. Hence it requires all the pages of VAddr2/3 to be set valid
   * to any physical page.
   * Marking any page invalid would stall the CPU.
   */

  for (i = 64; i < 256; i++)
    {
      if (!mmu_ll_check_entry_valid(0, i))
        {
          mmu_ll_write_entry(0, i, 0, MMU_TARGET_FLASH0);
        }

      if (!mmu_ll_check_entry_valid(1, i))
        {
          mmu_ll_write_entry(1, i, 0, MMU_TARGET_FLASH0);
        }
    }

  cache_ll_l1_enable_bus(0, CACHE_BUS_IBUS1);
#ifdef CONFIG_SMP
  cache_ll_l1_enable_bus(1, CACHE_BUS_IBUS1);
#endif
}
