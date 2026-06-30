/****************************************************************************
 * arch/tricore/src/tc4x/tc4x_watchdog.c
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

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/bits.h>

#include "tricore_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TC4X_WDTCPU0_BASE   0xf0000018
#define TC4X_WDTSYS_BASE    0xf0000184
#define TC4X_WDT_CPU_STRIDE 0x30
#define TC4X_WDT_CTRLA_OFF  0x24
#define TC4X_WDT_CTRLB_OFF  0x28
#define TC4X_WDT_CTRLA_LCK  BIT(0)
#define TC4X_WDT_CTRLA_PW   (0x7fu << 1)
#define TC4X_WDT_CTRLB_DR   BIT(0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void tc4x_wdt_block_disable(uintptr_t ctrla, uintptr_t ctrlb)
{
  uint32_t v = getreg32(ctrla);

  if (v & TC4X_WDT_CTRLA_LCK)
    {
      v &= ~TC4X_WDT_CTRLA_LCK;
      v ^= TC4X_WDT_CTRLA_PW;
      putreg32(v, ctrla);
    }

  putreg32(TC4X_WDT_CTRLB_DR, ctrlb);

  v = getreg32(ctrla);
  v |= TC4X_WDT_CTRLA_LCK;
  putreg32(v, ctrla);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void tricore_wdt_disable(void)
{
  uint32_t cpu_id;
  uintptr_t base;

  TRICORE_MFCR(TRICORE_CPU_CORE_ID, cpu_id);
  cpu_id &= TRICORE_CPU_CORE_ID_MASK;

  base = TC4X_WDTCPU0_BASE + cpu_id * TC4X_WDT_CPU_STRIDE;
  tc4x_wdt_block_disable(base + TC4X_WDT_CTRLA_OFF,
                          base + TC4X_WDT_CTRLB_OFF);

  if (cpu_id == 0)
    {
      tc4x_wdt_block_disable(TC4X_WDTSYS_BASE + TC4X_WDT_CTRLA_OFF,
                              TC4X_WDTSYS_BASE + TC4X_WDT_CTRLB_OFF);
    }
}
