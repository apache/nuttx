/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_wcl.c
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

#include <nuttx/nuttx.h>

#include "chip.h"
#include "xtensa.h"
#include "hardware/esp32s3_sensitive.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_wcl_core.h"

#include "esp32s3_wcl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum number of supported entry addresses */

#define WCL_ENTRY_MAX       13

/* Last value of the agreed sequence to be written to the address configured
 * in WCL_CORE_0_MESSAGE_ADDR register.
 */

#define WCL_SEQ_LAST_VAL    6

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_wcl_set_vecbase
 *
 * Description:
 *   Set the vector table a world uses, and make the World Controller take
 *      the value from these registers instead of the reset default.
 *
 * Input Parameters:
 *   world    - The world the table belongs to.
 *   vecbase  - The address of the table.  It must be aligned to 1 KB.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_wcl_set_vecbase(enum esp32s3_pms_world_e world,
                             uintptr_t vecbase)
{
  switch (world)
    {
      case PMS_WORLD_0:
        {
          modifyreg32(SENSITIVE_CORE_0_VECBASE_OVERRIDE_1_REG,
                      SENSITIVE_CORE_0_VECBASE_OVERRIDE_WORLD0_VALUE_M,
                      VALUE_TO_FIELD(vecbase >> 10,
                            SENSITIVE_CORE_0_VECBASE_OVERRIDE_WORLD0_VALUE));
        }
        break;
      case PMS_WORLD_1:
        {
          modifyreg32(SENSITIVE_CORE_0_VECBASE_OVERRIDE_2_REG,
                      SENSITIVE_CORE_0_VECBASE_OVERRIDE_WORLD1_VALUE_M,
                      VALUE_TO_FIELD(vecbase >> 10,
                            SENSITIVE_CORE_0_VECBASE_OVERRIDE_WORLD1_VALUE));
        }
        break;
      default:
        {
          PANIC();
        }
        break;
    }

  modifyreg32(SENSITIVE_CORE_0_VECBASE_OVERRIDE_1_REG,
              SENSITIVE_CORE_0_VECBASE_OVERRIDE_SEL_M,
              VALUE_TO_FIELD(0x3, SENSITIVE_CORE_0_VECBASE_OVERRIDE_SEL));

  modifyreg32(SENSITIVE_CORE_0_VECBASE_OVERRIDE_0_REG,
              SENSITIVE_CORE_0_VECBASE_WORLD_MASK_M, 0);
}

/****************************************************************************
 * Name: esp32s3_wcl_set_world0_entry
 *
 * Description:
 *   Register an address that returns the CPU to World 0 when it is fetched.
 *      This is how an exception taken in World 1 reaches a kernel handler.
 *
 * Input Parameters:
 *   entry    - Which entry to set, from 1 to WCL_ENTRY_MAX.
 *   addr     - The address that switches the world.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_wcl_set_world0_entry(uint32_t entry, uintptr_t addr)
{
  ASSERT(entry > 0 && entry <= WCL_ENTRY_MAX);

  /* Configure registers required for cleaning the World Controller write
   * buffer upon World0 entry.
   *
   * Refer to ESP32-S3 Technical Reference Manual, section 16.4.3, for a
   * detailed description of the write buffer clearing process.
   */

  putreg32(SOC_RTC_DATA_LOW, WCL_CORE_0_MESSAGE_ADDR_REG);
  putreg32(WCL_SEQ_LAST_VAL, WCL_CORE_0_MESSAGE_MAX_REG);

  uint32_t reg = WCL_CORE_0_ENTRY_1_ADDR_REG + ((entry - 1) * 4);

  /* Write ENTRY address */

  putreg32(addr, reg);

  /* Enable check for that particular address. When fetched, World will
   * switch to World 0.
   */

  modifyreg32(WCL_CORE_0_ENTRY_CHECK_REG, 0, BIT(entry));
}
