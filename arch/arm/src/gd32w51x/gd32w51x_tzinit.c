/****************************************************************************
 * arch/arm/src/gd32w51x/gd32w51x_tzinit.c
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

#include "arm_internal.h"
#include "chip.h"
#include "gd32w51x.h"
#include "hardware/gd32w51x_gpio.h"
#include "hardware/gd32w51x_rcu.h"
#include "gd32w51x_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Every GPIO port has a secure configuration register: one bit per pin,
 * all set (secure) out of reset.  A secure pin cannot be reached through
 * the non-secure alias of the port.
 */

#define GD32W51X_GPIO_SCFG_OFFSET  0x0030

#define RCU_AHB1EN_GPIOEN          (RCU_AHB1EN_PAEN | RCU_AHB1EN_PBEN | \
                                    RCU_AHB1EN_PCEN)

/* The RCU hides the enable bits of the secure peripherals from its
 * non-secure alias (they read as zero and ignore writes), so the clocks of
 * the still-secure ports must be switched through the secure alias.
 */

#define GD32W51X_RCU_AHB1EN_S      (GD32W51X_RCU_AHB1EN + \
                                    GD32W51X_SECURE_ALIAS)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32w51x_tzinit
 *
 * Description:
 *   NuttX runs in secure state but reaches every peripheral through its
 *   non-secure alias (see gd32w51x_memorymap.h).  The bus peripherals
 *   (TZSPC) are already non-secure out of reset and the RCU accepts
 *   non-secure accesses (RCU_SECP_CFG = 0); only the GPIO pins need to be
 *   released.
 *
 *   The SRAM blocks stay secure (TZBMPC): the core accesses them in
 *   secure state.
 *
 ****************************************************************************/

void gd32w51x_tzinit(void)
{
  uint32_t enabled;

  /* The SCFG registers are only reachable with the port clocked; the
   * secure alias is the only one that can write them.
   */

  enabled = getreg32(GD32W51X_RCU_AHB1EN_S);
  putreg32(enabled | RCU_AHB1EN_GPIOEN, GD32W51X_RCU_AHB1EN_S);

  putreg32(0, GD32W51X_GPIOA_BASE + GD32W51X_SECURE_ALIAS +
              GD32W51X_GPIO_SCFG_OFFSET);
  putreg32(0, GD32W51X_GPIOB_BASE + GD32W51X_SECURE_ALIAS +
              GD32W51X_GPIO_SCFG_OFFSET);
  putreg32(0, GD32W51X_GPIOC_BASE + GD32W51X_SECURE_ALIAS +
              GD32W51X_GPIO_SCFG_OFFSET);

  putreg32(enabled, GD32W51X_RCU_AHB1EN_S);
}

/****************************************************************************
 * Name: gd32w51x_gpio_setsecure
 *
 * Description:
 *   Mark one pin secure or non-secure.  The alternate functions of the
 *   secure peripherals (e.g. the QSPI) only reach secure pins.  Once a pin
 *   is secure, its port registers can no longer be reached through the
 *   non-secure alias, so configure the pin first.
 *
 ****************************************************************************/

void gd32w51x_gpio_setsecure(uint32_t pinset, bool secure)
{
  uint32_t port = (pinset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;
  uint32_t regaddr;

  regaddr = GD32W51X_GPIOA_BASE + port * 0x400 + GD32W51X_SECURE_ALIAS +
            GD32W51X_GPIO_SCFG_OFFSET;

  modifyreg32(regaddr, secure ? 0 : (1 << pin), secure ? (1 << pin) : 0);
}
