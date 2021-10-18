/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_wdt.c
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

#include "xtensa.h"
#include "hardware/esp32s2_rtccntl.h"

#include "esp32s2_wdt.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_wdt_early_deinit
 *
 * Description:
 *   Disable the WDT(s) that was/were enabled by the bootloader.
 *
 ****************************************************************************/

void esp32s2_wdt_early_deinit(void)
{
  uint32_t regval;
  putreg32(RTC_CNTL_WDT_WKEY_VALUE, RTC_CNTL_WDTWPROTECT_REG);
  regval  = getreg32(RTC_CNTL_WDTCONFIG0_REG);
  regval &= ~RTC_CNTL_WDT_EN;
  putreg32(regval, RTC_CNTL_WDTCONFIG0_REG);
  putreg32(0, RTC_CNTL_WDTWPROTECT_REG);
}
