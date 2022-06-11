/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_resetcause.c
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

#include <stdint.h>

#include <nuttx/arch.h>

#include "hardware/esp32c3_rtccntl.h"

#include "esp32c3.h"
#include "esp32c3_resetcause.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_resetcause
 *
 * Description:
 *   Get the cause of the last reset.
 *
 ****************************************************************************/

enum esp32c3_resetcause_e esp32c3_resetcause(void)
{
  uint32_t regmask;
  uint32_t regshift;
  uint32_t regval;

  regval = getreg32(RTC_CNTL_RESET_STATE_REG);
  regmask = RTC_CNTL_RESET_CAUSE_PROCPU_M;
  regshift = RTC_CNTL_RESET_CAUSE_PROCPU_S;

  return (regval & regmask) >> regshift;
}
