/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_reset_reasons.c
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
#include <nuttx/board.h>

#include "xtensa.h"
#include "hardware/esp32s3_rtccntl.h"

#include "esp32s3_reset_reasons.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_reset_reasons
 *
 * Description:
 *   Get the cause of the last reset of the given CPU
 *
 ****************************************************************************/

soc_reset_reason_t esp32s3_reset_reasons(int cpu)
{
  uint32_t regmask;
  uint32_t regshift;
  uint32_t regval;

  regval = getreg32(RTC_CNTL_RTC_RESET_STATE_REG);

#ifdef CONFIG_SMP
  if (cpu != 0)
    {
      regmask = RTC_CNTL_RESET_CAUSE_APPCPU_M;
      regshift = RTC_CNTL_RESET_CAUSE_APPCPU_S;
    }
  else
#endif
    {
      regmask = RTC_CNTL_RESET_CAUSE_PROCPU_M;
      regshift = RTC_CNTL_RESET_CAUSE_PROCPU_S;
    }

  return (regval & regmask) >> regshift;
}
