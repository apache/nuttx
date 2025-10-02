/****************************************************************************
 * arch/arm/src/rp2040/rp2040_syscfg.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "rp2040_syscfg.h"
#include "hardware/rp2040_syscfg.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_set_proc_dap_instid
 *
 * Description:
 *   Configure proc{0,1} DAP instance ID.
 *   Recommend that this is NOT changed until you require debug access in
 *   multichip environment. WARNING: do not set to 15 as this is reserved for
 *   RescueDP.
 *
 ****************************************************************************/

void rp2040_set_proc_dap_instid(uint8_t proc, uint8_t dap_instid)
{
  /* Proc number must be either 0 or 1 */

  ASSERT(proc <= 1);
  const uint32_t shift = (proc == 0)
                        ? RP2040_SYSCFG_PROC_CONFIG_PROC0_DAP_INSTID_SHIFT
                        : RP2040_SYSCFG_PROC_CONFIG_PROC1_DAP_INSTID_SHIFT;
  const uint32_t mask = (proc == 0)
                        ? RP2040_SYSCFG_PROC_CONFIG_PROC0_DAP_INSTID_MASK
                        : RP2040_SYSCFG_PROC_CONFIG_PROC1_DAP_INSTID_MASK;

  /* ID must be 4 bits and different from 15 (0xf) */

  ASSERT(dap_instid < 0xf);

  /* Set the configurable ID bits */

  clrbits_reg32(mask, RP2040_SYSCFG_PROC_CONFIG);
  setbits_reg32(dap_instid << shift, RP2040_SYSCFG_PROC_CONFIG);
}
