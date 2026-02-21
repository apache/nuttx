/****************************************************************************
 * arch/arm/src/am67/am67_pinmux.c
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
#include <assert.h>
#include <stdint.h>
#include <sys/types.h>

#include "am67_pinmux.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

static struct pinmux_conf_s g_am67_pinmux_conf[] =
{
  /* UART1_RXD -> MCASP0_AFSR (C27) */

  {
    PIN_MCASP0_AFSR,
    (PIN_MODE(2) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE)
  },

  /* UART1_TXD -> MCASP0_ACLKR (F24) */

  {
    PIN_MCASP0_ACLKR,
    (PIN_MODE(2) | PIN_PULL_DISABLE)
  },

  /* RED LED -> OLDI0_A0N (AF23) */

  {
    PIN_OLDI0_A0N,
    (PIN_MODE(7) | PIN_PULL_DISABLE)
  },

  /* GREEN LED -> OLDI0_A0P (AG24) */

  {
    PIN_OLDI0_A0P,
    (PIN_MODE(7) | PIN_PULL_DISABLE)
  },
  {PINMUX_END, PINMUX_END}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_pinmux_unlock
 *
 * Description:
 *   Unlock the pinmux configuration registers by writing unlock values to
 *   both lock registers (Lock0 and Lock1) kick registers.
 *
 ****************************************************************************/

static void am67_pinmux_unlock(void)
{
  uint32_t base_addr;
  uint32_t kick_addr;

  base_addr = CSL_PADCFG_CTRL0_CFG0_BASE;

  /* Lock 0 */

  kick_addr = base_addr + CSL_MAIN_PADCONFIG_LOCK0_KICK0_OFFSET;
  putreg32(KICK0_UNLOCK_VAL, kick_addr);
  kick_addr += 4;
  putreg32(KICK1_UNLOCK_VAL, kick_addr);

  /* Lock 1 */

  kick_addr = base_addr + CSL_MAIN_PADCONFIG_LOCK1_KICK0_OFFSET;
  putreg32(KICK0_UNLOCK_VAL, kick_addr);
  kick_addr += 4;
  putreg32(KICK1_UNLOCK_VAL, kick_addr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_pinmux_config
 *
 * Description:
 *   Configure pin multiplexing settings by writing configuration values to
 *   pad configuration registers after unlocking the pinmux registers.
 *
 ****************************************************************************/

void am67_pinmux_config(const struct pinmux_conf_s *pinmux_conf)
{
  if (pinmux_conf != NULL)
    {
      uint32_t base_addr = CSL_PADCFG_CTRL0_CFG0_BASE + PADCFG_PMUX_OFFSET;

      am67_pinmux_unlock();

      while (pinmux_conf->offset != PINMUX_END)
        {
          /* Set all the configuration fields */

          putreg32(pinmux_conf->setting, base_addr + pinmux_conf->offset);
          pinmux_conf++;
        }
    }
}

/****************************************************************************
 * Name: am67_pinmux_init
 *
 * Description:
 *   Initialize pin multiplexing using the global pinmux configuration array.
 *
 ****************************************************************************/

void am67_pinmux_init(void)
{
  am67_pinmux_config(g_am67_pinmux_conf);
}
