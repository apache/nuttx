/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_chip_rev.c
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
 * Included Files
 ****************************************************************************/

#include "hal/efuse_hal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32P4_MIN_FULLY_SUPPORTED_VERSION 300

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * ROM Function Prototypes
 ****************************************************************************/

extern int ets_printf(const char *fmt, ...) printf_like(1, 2);

/****************************************************************************
 * Name: esp_chip_revision_check
 *
 * Description:
 *   Checks if the current  chip revision is greater than the minimum
 *   revision supported by NuttX. If the chip revision is not supported, an
 *   error or warning message is printed, and the system may halt unless
 *   the configuration allows ignoring the chip revision check.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_chip_revision_check(void)
{
  uint32_t chip_rev = efuse_hal_chip_revision();

  if (chip_rev < ESP32P4_MIN_FULLY_SUPPORTED_VERSION)
    {
#ifndef ESP32P4_IGNORE_CHIP_REVISION_CHECK
      ets_printf("ERROR: NuttX supports ESP32-P4 chip revision > v%d.%01d"
                 " (chip revision is v%" PRIu32 ".%" PRIu32 ")\n",
                 ESP32P4_MIN_FULLY_SUPPORTED_VERSION / 100,
                 ESP32P4_MIN_FULLY_SUPPORTED_VERSION % 100,
                 chip_rev / 100, chip_rev % 100);
      PANIC();
#endif
      ets_printf("WARNING: NuttX supports ESP32-P4 chip revision > v%d.%01d"
                 " (chip revision is v%" PRIu32 ".%" PRIu32 ").\n"
                 "Ignoring this error and continuing because "
                 "`ESP32P4_IGNORE_CHIP_REVISION_CHECK` is set...\n"
                 "THIS MAY NOT WORK! DON'T USE THIS CHIP IN PRODUCTION!\n",
                 ESP32P4_MIN_FULLY_SUPPORTED_VERSION / 100,
                 ESP32P4_MIN_FULLY_SUPPORTED_VERSION % 100,
                 chip_rev / 100, chip_rev % 100);
    }
}
