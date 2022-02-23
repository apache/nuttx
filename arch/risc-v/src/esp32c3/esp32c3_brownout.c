/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_brownout.c
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

#include "esp32c3.h"
#include "hardware/esp32c3_rtccntl.h"
#include "hardware/regi2c_ctrl.h"
#include "hardware/regi2c_brownout.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32C3_BROWNOUT_VAL        CONFIG_ESP32C3_BROWNOUT_DET_LVL
#define ESP32C3_BROWNOUT_RST_SEL    1
#define ESP32C3_BROWNOUT_RST_WAIT   0x3ff

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  esp32c3_brownout_init
 *
 * Description:
 *   Initialize hardware brownout check and reset.
 *
 ****************************************************************************/

void esp32c3_brownout_init(void)
{
  uint32_t regval;

  REGI2C_WRITE_MASK(I2C_BOD, I2C_BOD_THRESHOLD, ESP32C3_BROWNOUT_VAL);

  regval = RTC_CNTL_BROWN_OUT_ENA |
           (ESP32C3_BROWNOUT_RST_SEL << RTC_CNTL_BROWN_OUT_RST_SEL_S) |
           RTC_CNTL_BROWN_OUT_RST_ENA |
           (ESP32C3_BROWNOUT_RST_WAIT << RTC_CNTL_BROWN_OUT_RST_WAIT_S) |
           RTC_CNTL_BROWN_OUT_PD_RF_ENA |
           RTC_CNTL_BROWN_OUT_CLOSE_FLASH_ENA;

  putreg32(regval, RTC_CNTL_BROWN_OUT_REG);
}
