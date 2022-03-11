/****************************************************************************
 * boards/arm/lpc43xx/lpc4370-link2/src/lpc43_spifilib_init.c
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
#include <stdbool.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "lpc43_spifi.h"
#include "lpc43_cgu.h"

#include "lpc4370-link2.h"

#include "spifi/inc/spifilib_api.h"

#ifdef CONFIG_SPIFI_LIBRARY

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Local memory, 32-bit aligned that will be used for driver context
 * (handle).
 */

static uint32_t lmem[21];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void board_spifi_initialize(void)
{
  irqstate_t flags = enter_critical_section();
  uint32_t regval;

  flags = enter_critical_section();

  /* Initial frequency is set by boot ROM in IDIVE */

  /* Pin configuration */

  lpc43_pin_config(PINCONF_SPIFI_CS);
  lpc43_pin_config(PINCONF_SPIFI_MISO);
  lpc43_pin_config(PINCONF_SPIFI_MOSI);
  lpc43_pin_config(PINCONF_SPIFI_SCK);
  lpc43_pin_config(PINCONF_SPIFI_SIO2);
  lpc43_pin_config(PINCONF_SPIFI_SIO3);

  /* Initialize LPCSPIFILIB library, reset the interface */

  spifiInit(LPC43_SPIFI_CTRL, true);

  /* Register the family for the device */

  spifiRegisterFamily(spifi_REG_FAMILY_CommonCommandSet);

  /* Initialize and detect a device and get device context */

  SPIFI_HANDLE_T *pSpifi = spifiInitDevice(&lmem, sizeof(lmem),
                                           LPC43_SPIFI_CTRL,
                                           LPC43_LOCSRAM_SPIFI_BASE);

  /* Enable quad.  If not supported it will be ignored */

  spifiDevSetOpts(pSpifi, SPIFI_OPT_USE_QUAD, true);

  /* Enter memMode */

  spifiDevSetMemMode(pSpifi, true);

  /* Configure divider as the input to the SPIFI */

  regval  = getreg32(LPC43_BASE_SPIFI_CLK);
  regval &= ~BASE_SPIFI_CLK_CLKSEL_MASK;
  regval |= BASE_SPIFI_CLKSEL_IDIVE;
  putreg32(regval, LPC43_BASE_SPIFI_CLK);

  regval  = getreg32(LPC43_IDIVE_CTRL);
  regval &= ~(IDIVE_CTRL_CLKSEL_MASK | IDIVE_CTRL_IDIV_MASK);
  regval |= BASE_SPIFI_CLKSEL_PLL1 | IDIVE_CTRL_AUTOBLOCK |
            IDIVE_CTRL_IDIV(SPIFI_DEVICE_REQUENCY_DIVIDER);
  putreg32(regval, LPC43_IDIVE_CTRL);

  leave_critical_section(flags);
}

#endif /* CONFIG_SPIFI_LIBRARY */
