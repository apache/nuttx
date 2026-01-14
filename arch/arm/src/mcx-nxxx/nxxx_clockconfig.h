/****************************************************************************
 * arch/arm/src/mcx-nxxx/nxxx_clockconfig.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_NXXX_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_MCX_NXXX_NXXX_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include "hardware/nxxx_clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxxx_clockconfig
 *
 * Description:
 *   Called to initialize the clocks for MCX-NXXX. This does whatever setup
 *   is needed to put the SoC in a usable state. This includes the
 *   initialization of clocking using the settings in board.h.
 *
 ****************************************************************************/

void nxxx_clockconfig(void);

/****************************************************************************
 * Name: nxxx_set_periphclock
 *
 * Description:
 *   This function sets the clock frequency of the specified peripheral
 *   functional clock.
 *
 * Input Parameters:
 *   clock - Identifies the peripheral clock of interest
 *   sel   - Selected clock source (every peripheral has its own set of
 *           possible sources)
 *   div   - Divider for the clock
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  -ENODEV is returned if the clock is not enabled or is not
 *   being clocked.
 *
 ****************************************************************************/

int nxxx_set_periphclock(struct clock_regs_s clock, uint32_t sel,
                         uint32_t div);

/****************************************************************************
 * Name: nxxx_set_clock_gate
 *
 * Description:
 *   Open or close a specific clock gate.
 *
 * Input Parameters:
 *   gate    - Identifies the peripheral clock gate of interest.
 *   enabled - True enables the clock; false disables it.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  -ENODEV is returned if the clock is not enabled or is not
 *   being clocked.
 *
 ****************************************************************************/

int nxxx_set_clock_gate(struct clock_gate_reg_s gate, bool enabled);

/****************************************************************************
 * Name: nxxx_get_coreclk
 *
 * Description:
 *   Return the current value of the CORE clock frequency.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   The current value of the CORE clock frequency.  Zero is returned on any
 *   failure.
 *
 ****************************************************************************/

uint32_t nxxx_get_coreclk(void);

#endif /* __ARCH_ARM_SRC_MCX_NXXX_NXXX_CLOCKCONFIG_H */
