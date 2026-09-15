/****************************************************************************
 * arch/arm/src/n32h7/n32_pwr.c
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
#include <nuttx/arch.h>
#include <arch/barriers.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "arm_internal.h"
#include "n32_pwr.h"

#if defined(CONFIG_N32H7_PWR)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint16_t g_bkp_writable_counter = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t n32_pwr_getreg(uint32_t offset)
{
  return getreg32(N32_PWR_BASE + offset);
}

static inline void n32_pwr_putreg(uint32_t offset, uint32_t value)
{
  putreg32(value, N32_PWR_BASE + offset);
}

static inline void n32_pwr_modifyreg(uint32_t offset, uint32_t clearbits,
                                      uint32_t setbits)
{
  modifyreg32(N32_PWR_BASE + offset, clearbits, setbits);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_pwr_initbkp
 ****************************************************************************/

void n32_pwr_initbkp(bool writable)
{
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  regval = n32_pwr_getreg(N32_PWR_SYS_PWR_CR_OFFSET);
  regval &= ~PWR_SYSCTRL1_DBKP;
  n32_pwr_putreg(N32_PWR_SYS_PWR_CR_OFFSET, regval);

  g_bkp_writable_counter = 0;

  leave_critical_section(flags);

  n32_pwr_enablebkp(writable);
}

/****************************************************************************
 * Name: n32_pwr_enablebkp
 ****************************************************************************/

void n32_pwr_enablebkp(bool writable)
{
  irqstate_t flags;
  uint32_t regval;
  bool waswritable;

  flags = enter_critical_section();
  UP_DSB();

  regval = n32_pwr_getreg(N32_PWR_C1_PWR_CR_OFFSET);
  waswritable = ((regval & PWR_SYSCTRL1_DBKP) != 0);

  if (writable)
    {
      DEBUGASSERT(g_bkp_writable_counter < UINT16_MAX);
      g_bkp_writable_counter++;
    }
  else if (g_bkp_writable_counter > 0)
    {
      g_bkp_writable_counter--;
    }

  if (waswritable && g_bkp_writable_counter == 0)
    {
      regval &= ~PWR_SYSCTRL1_DBKP;
      n32_pwr_putreg(N32_PWR_C1_PWR_CR_OFFSET, regval);
    }
  else if (!waswritable && g_bkp_writable_counter > 0)
    {
      regval |= PWR_SYSCTRL1_DBKP;
      n32_pwr_putreg(N32_PWR_C1_PWR_CR_OFFSET, regval);
      up_udelay(4);
    }

  UP_DSB();
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: n32_pwr_setpvd
 ****************************************************************************/

void n32_pwr_setpvd(uint32_t pls)
{
  uint32_t regval;

  regval  = n32_pwr_getreg(N32_PWR_C1_PWR_CR_OFFSET);
  regval &= ~PWR_PVD_LEVEL_MASK;
  regval |= (pls & PWR_PVD_LEVEL_MASK);
  n32_pwr_putreg(N32_PWR_C1_PWR_CR_OFFSET, regval);
}

/****************************************************************************
 * Name: n32_pwr_enablepvd
 ****************************************************************************/

void n32_pwr_enablepvd(void)
{
  n32_pwr_modifyreg(N32_PWR_C1_PWR_CR_OFFSET, 0, PWR_SYSCTRL1_PVDEN);
}

/****************************************************************************
 * Name: n32_pwr_disablepvd
 ****************************************************************************/

void n32_pwr_disablepvd(void)
{
  n32_pwr_modifyreg(N32_PWR_C1_PWR_CR_OFFSET, PWR_SYSCTRL1_PVDEN, 0);
}

/****************************************************************************
 * Name: n32_pwr_configurewkup
 ****************************************************************************/

void n32_pwr_configurewkup(uint32_t pin, bool en, bool rising, uint32_t pull)
{
  irqstate_t flags;
  uint32_t regval;

  DEBUGASSERT(pin < 6);

  flags = enter_critical_section();

  regval = n32_pwr_getreg(N32_PWR_C1_PWR_CSR_OFFSET);

  if (en)
    {
      regval |= (PWR_M7CTRLSTS_WKUP0F << pin); /* WKUP0EN -> WKUP5EN */
    }
  else
    {
      regval &= ~(PWR_M7CTRLSTS_WKUP0F << pin);
    }

  if (rising)
    {
      regval &= ~(PWR_M7CTRLSTS_WKUP0POL << pin); /* WKUP0POL -> WKUP5POL */
    }
  else
    {
      regval |= (PWR_M7CTRLSTS_WKUP0POL << pin);
    }

  n32_pwr_putreg(N32_PWR_C1_PWR_CSR_OFFSET, regval);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: n32_pwr_setcorepwrsrc
 ****************************************************************************/

void n32_pwr_setcorepwrsrc(pwr_vcoresrc_e src)
{
  switch (src)
    {
      case PWR_VCORESRC_EXT:
        n32_pwr_modifyreg(N32_PWR_SYS_PWR_CR4_OFFSET, PWR_VCORESRC_MASK,
                          PWR_SYSCTRL4_VCORESRC);
        break;
      case PWR_VCORESRC_LDO:
        n32_pwr_modifyreg(N32_PWR_SYS_PWR_CR4_OFFSET, PWR_VCORESRC_MASK,
                          PWR_SYSCTRL4_MLDOEN);
        break;
      case PWR_VCORESRC_SMPS:
        n32_pwr_modifyreg(N32_PWR_SYS_PWR_CR4_OFFSET, PWR_VCORESRC_MASK,
                          PWR_SYSCTRL4_DCDEN);
        break;
    }
}

#endif /* CONFIG_N32H7_PWR */
