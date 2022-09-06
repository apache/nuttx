/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_syscfg.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "arm_internal.h"

#include "chip.h"
#include "gd32f4xx_syscfg.h"
#include "gd32f4xx_rcu.h"

#ifdef CONFIG_GD32F4_SYSCFG

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_syscfg_bootmode_config
 *
 * Description:
 *   This function configure the boot mode.
 *
 ****************************************************************************/

void gd32_syscfg_bootmode_config(uint8_t syscfg_bootmode)
{
  uint32_t regval;

  regval = getreg32(GD32_SYSCFG_CFG0);
  regval &= ~ SYSCFG_CFG0_BOOT_MODE_MASK;
  regval |= (uint32_t)syscfg_bootmode;
  putreg32(regval, GD32_SYSCFG_CFG0);
}

/****************************************************************************
 * Name: gd32_syscfg_fmc_swap_config
 *
 * Description:
 *   This function configure FMC memory mapping swap.
 *
 ****************************************************************************/

void gd32_syscfg_fmc_swap_config(uint32_t syscfg_fmc_swap)
{
  uint32_t regval;
  regval = getreg32(GD32_SYSCFG_CFG0);
  regval &= ~SYSCFG_CFG0_FMC_SWP_MASK;
  regval |= syscfg_fmc_swap;
  putreg32(regval, GD32_SYSCFG_CFG0);
}

/****************************************************************************
 * Name: gd32_syscfg_exmc_swap_config
 *
 * Description:
 *   This function configure EXMC memory mapping swap.
 *
 ****************************************************************************/

void gd32_syscfg_exmc_swap_config(uint32_t syscfg_exmc_swap)
{
  uint32_t regval;

  regval = getreg32(GD32_SYSCFG_CFG0);
  regval &= ~SYSCFG_CFG0_EXMC_SWP_MASK;
  regval |= syscfg_exmc_swap;
  putreg32(regval, GD32_SYSCFG_CFG0);
}

/****************************************************************************
 * Name: gd32_syscfg_exti_line_config
 *
 * Description:
 *   This function configure the GPIO pin as EXTI Line.
 *
 ****************************************************************************/

void gd32_syscfg_exti_line_config(uint8_t exti_port, uint8_t exti_pin)
{
  uint32_t regval;
  uint32_t regaddr;

  switch (exti_pin / SYSCFG_EXTI_SS_JSTEP)
    {
    /* EXTI source line(0..3) */

    case SYSCFG_EXTISS0:
      regaddr = GD32_SYSCFG_EXTISS0;
      break;

    /* EXTI source line(4..7) */

    case SYSCFG_EXTISS1:
      regaddr = GD32_SYSCFG_EXTISS1;
      break;

    /* EXTI source line(8..11) */

    case SYSCFG_EXTISS2:
      regaddr = GD32_SYSCFG_EXTISS2;
      break;

    /* EXTI source line(12..15) */

    case SYSCFG_EXTISS3:
      regaddr = GD32_SYSCFG_EXTISS3;
      break;
    default:
      break;
    }

  regval = getreg32(regaddr);
  regval &= ~(SYSCFG_EXTI_SS_MASK << (SYSCFG_EXTI_SS_MSTEP(exti_pin)));
  regval |= ((exti_port) << (SYSCFG_EXTI_SS_MSTEP(exti_pin)));
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_syscfg_enet_phy_interface_config
 *
 * Description:
 *   This function configure the PHY interface for the ethernet MAC.
 *
 ****************************************************************************/

void
gd32_syscfg_enet_phy_interface_config(uint32_t syscfg_enet_phy_interface)
{
  uint32_t regval;

  regval = getreg32(GD32_SYSCFG_CFG1);
  regval &= ~SYSCFG_CFG1_ENET_PHY_SEL_MASK;
  regval |= syscfg_enet_phy_interface;
  putreg32(regval, GD32_SYSCFG_CFG1);
}

/****************************************************************************
 * Name: gd32_syscfg_compensation_config
 *
 * Description:
 *   This function configure the I/O compensation cell.
 *
 ****************************************************************************/

void gd32_syscfg_compensation_config(uint32_t syscfg_compensation)
{
  uint32_t regval;

  regval = getreg32(GD32_SYSCFG_CPSCTL);
  regval &= ~SYSCFG_CPSCTL_CPS_EN_MASK;
  regval |= syscfg_compensation;
  putreg32(regval, GD32_SYSCFG_CPSCTL);
}

/****************************************************************************
 * Name: gd32_syscfg_flag_get
 *
 * Description:
 *   This function checks whether the I/O compensation cell ready flag
 *   is set or not.
 *
 ****************************************************************************/

bool gd32_syscfg_flag_get(void)
{
  bool regval;

  regval = getreg32(GD32_SYSCFG_CPSCTL);
  return (bool)(regval & SYSCFG_CPSCTL_CPS_RDY_MASK);
}

/****************************************************************************
 * Name: gd32_syscfg_clock_enable
 *
 * Description:
 *   Enable SYSCFG clock
 ****************************************************************************/

void gd32_syscfg_clock_enable(void)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  rcu_en = RCU_APB2EN_SYSCFGEN;

  regaddr = GD32_RCU_APB2EN;

  /* Check clock if alreay enable. */

  if (rcu_en != (rcu_en & getreg32(regaddr)))
    {
      /* Enable/disable APB2 clock for SYSCFG */

      modifyreg32(regaddr, 0, rcu_en);
    }
}

/****************************************************************************
 * Name: gd32_syscfg_clock_disable
 *
 * Description:
 *   Dinable SYSCFG clock
 ****************************************************************************/

void gd32_syscfg_clock_disable(void)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  rcu_en = RCU_APB2EN_SYSCFGEN;

  regaddr = GD32_RCU_APB2EN;

  /* Disable APB2 clock for SYSCFG */

  modifyreg32(regaddr, rcu_en, 0);
}

#endif /* CONFIG_GD32F4_SYSCFG */
