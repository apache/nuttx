/****************************************************************************
 * arch/arm/src/am335x/am335x_clockconfig.c
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

#include "arm_internal.h"
#include "hardware/am335x_prcm.h"
#include "am335x_config.h"
#include "am335x_clockconfig.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_power_domain_transition_enable
 ****************************************************************************/

static void am335x_interface_clocks_enable(void)
{
  /* Enable all the Interconnect Modules */

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_L3_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_L3_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_L4LS_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_L4LS_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_L4FW_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_L4FW_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_WKUP_L4WKUP_CLKCTRL);
  while ((getreg32(AM335X_CM_WKUP_L4WKUP_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_L3_INSTR_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_L3_INSTR_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_L4HS_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_L4HS_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_OCPWP_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_OCPWP_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }
}

/****************************************************************************
 * Name: am335x_power_domain_transition_enable
 ****************************************************************************/

static void am335x_power_domain_transition_enable(void)
{
  putreg32(CM_CLKSTCTRL_CLKTRCTRL_SW_WKUP, AM335X_CM_PER_L3_CLKSTCTRL);

  putreg32(CM_CLKSTCTRL_CLKTRCTRL_SW_WKUP, AM335X_CM_PER_L4LS_CLKSTCTRL);

  putreg32(CM_CLKSTCTRL_CLKTRCTRL_SW_WKUP, AM335X_CM_WKUP_CLKSTCTRL);

  putreg32(CM_CLKSTCTRL_CLKTRCTRL_SW_WKUP, AM335X_CM_PER_L4FW_CLKSTCTRL);

  putreg32(CM_CLKSTCTRL_CLKTRCTRL_SW_WKUP, AM335X_CM_PER_L3S_CLKSTCTRL);

  putreg32(CM_CLKSTCTRL_CLKTRCTRL_SW_WKUP, AM335X_CM_PER_OCPWP_L3_CLKSTCTRL);

#if defined(CONFIG_AM335X_LCDC)
  putreg32(CM_CLKSTCTRL_CLKTRCTRL_SW_WKUP, AM335X_CM_PER_LCDC_CLKSTCTRL);
#endif

#if defined(CONFIG_AM335X_CPSW)
  putreg32(CM_CLKSTCTRL_CLKTRCTRL_SW_WKUP, AM335X_CM_PER_CPSW_CLKSTCTRL);
#endif
}

/****************************************************************************
 * Name: am335x_peripheral_enable
 ****************************************************************************/

static void am335x_peripheral_enable(void)
{
  uint32_t wkup = CM_WKUP_CLKSTCTRL_L4_WKUP_GCLK;
  uint32_t per_ocpwp_l3 = CM_PER_OCPWP_L3_CLKSTCTRL_OCPWP_L3_GCLK;
  uint32_t per_l3 = CM_PER_L3_CLKSTCTRL_L3_GCLK;
  uint32_t per_l3s = CM_PER_L3S_CLKSTCTRL_L3S_GCLK;
  uint32_t per_l4ls = CM_PER_L4LS_CLKSTCTRL_L4LS_GCLK;
  uint32_t per_lcdc = 0;
  uint32_t per_cpsw = 0;

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_WKUP_GPIO0_CLKCTRL);
  while ((getreg32(AM335X_CM_WKUP_GPIO0_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_WKUP_TIMER1_CLKCTRL);
  while ((getreg32(AM335X_CM_WKUP_TIMER1_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  wkup |= CM_WKUP_CLKSTCTRL_TIMER1_GCLK;

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_WKUP_WDT1_CLKCTRL);
  while ((getreg32(AM335X_CM_WKUP_WDT1_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  wkup |= CM_WKUP_CLKSTCTRL_WDT1_GCLK;

#ifdef CONFIG_AM335X_TIMER0
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_WKUP_TIMER0_CLKCTRL);
  while ((getreg32(AM335X_CM_WKUP_TIMER0_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  wkup |= CM_WKUP_CLKSTCTRL_TIMER0_GCLK;
#endif

#ifdef CONFIG_AM335X_UART0
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_WKUP_UART0_CLKCTRL);
  while ((getreg32(AM335X_CM_WKUP_UART0_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  wkup |= CM_WKUP_CLKSTCTRL_UART0_GFCLK;
#endif

#ifdef CONFIG_AM335X_I2C0
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_WKUP_I2C0_CLKCTRL);
  while ((getreg32(AM335X_CM_WKUP_I2C0_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  wkup |= CM_WKUP_CLKSTCTRL_I2C0_GFCLK;
#endif

#ifdef CONFIG_AM335X_TSC
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_WKUP_ADC_TSC_CLKCTRL);
  while ((getreg32(AM335X_CM_WKUP_ADC_TSC_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  wkup |= CM_WKUP_CLKSTCTRL_ADC_FCLK;
#endif

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_GPMC_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_GPMC_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_ELM_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_ELM_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_GPIO1_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_GPIO1_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_GPIO2_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_GPIO2_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_GPIO3_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_GPIO3_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

#ifdef CONFIG_AM335X_TIMER2
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_TIMER2_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_TIMER2_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_TIMER2_GCLK;
#endif

#ifdef CONFIG_AM335X_TIMER3
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_TIMER3_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_TIMER3_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_TIMER3_GCLK;
#endif

#ifdef CONFIG_AM335X_TIMER4
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_TIMER4_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_TIMER4_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_TIMER4_GCLK;
#endif

#ifdef CONFIG_AM335X_TIMER5
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_TIMER5_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_TIMER5_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_TIMER5_GCLK;
#endif

#ifdef CONFIG_AM335X_TIMER6
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_TIMER6_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_TIMER6_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_TIMER6_GCLK;
#endif

#ifdef CONFIG_AM335X_TIMER7
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_TIMER7_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_TIMER7_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_TIMER7_GCLK;
#endif

#ifdef CONFIG_AM335X_UART1
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_UART1_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_UART1_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_UART_GCLK;
#endif

#ifdef CONFIG_AM335X_UART2
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_UART2_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_UART2_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_UART_GCLK;
#endif

#ifdef CONFIG_AM335X_UART3
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_UART3_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_UART3_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_UART_GCLK;
#endif

#ifdef CONFIG_AM335X_UART4
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_UART4_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_UART4_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_UART_GCLK;
#endif

#ifdef CONFIG_AM335X_UART5
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_UART5_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_UART5_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_UART_GCLK;
#endif

#if defined(CONFIG_AM335X_CAN0)
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_DCAN0_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_DCAN0_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_CAN_CLK;
#endif

#if defined(CONFIG_AM335X_CAN1)
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_DCAN1_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_DCAN1_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_CAN_CLK;
#endif

#if defined(CONFIG_AM335X_SPI0)
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_SPI0_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_SPI0_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_SPI_GCLK;
#endif

#if defined(CONFIG_AM335X_SPI1)
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_SPI1_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_SPI1_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_SPI_GCLK;
#endif

#if defined(CONFIG_AM335X_I2C1)
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_I2C1_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_I2C1_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_I2C_FCLK;
#endif

#if defined(CONFIG_AM335X_I2C2)
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_I2C2_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_I2C2_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_I2C_FCLK;
#endif

#if defined(CONFIG_AM335X_USB0) || defined(CONFIG_AM335X_USB1)
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_USB0_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_USB0_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

#endif

#if defined(CONFIG_AM335X_MMC0)
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_MMC0_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_MMC0_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L3_CLKSTCTRL_MMC_FCLK;
#endif

#if defined(CONFIG_AM335X_MMC1)
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_MMC1_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_MMC1_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L3_CLKSTCTRL_MMC_FCLK;
#endif

#if defined(CONFIG_AM335X_MMC2)
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_MMC2_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_MMC2_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_l4ls |= CM_PER_L3_CLKSTCTRL_MMC_FCLK;
#endif

#if defined(CONFIG_AM335X_LCDC)
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_LCDC_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_LCDC_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_lcdc |= CM_PER_LCDC_CLKSTCTRL_LCDC_L4_OCP_GCLK |
              CM_PER_LCDC_CLKSTCTRL_LCDC_L3_OCP_GCLK;
  per_l4ls |= CM_PER_L4LS_CLKSTCTRL_LCDC_GCLK;
#endif

#if defined(CONFIG_AM335X_CPSW)
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_CPGMAC0_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_CPGMAC0_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  per_cpsw |= CM_PER_CPSW_CLKSTCTRL_CPSW_125MHZ_GCLK;
#endif

#if defined(CONFIG_AM335X_DMA)
  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_TPCC_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_TPCC_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_TPTC0_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_TPTC0_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_TPTC1_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_TPTC1_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_PER_TPTC2_CLKCTRL);
  while ((getreg32(AM335X_CM_PER_TPTC2_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }
#endif

  putreg32(CM_CLKCTRL_MODULEMODE_ENABLE, AM335X_CM_WKUP_CONTROL_CLKCTRL);
  while ((getreg32(AM335X_CM_WKUP_CONTROL_CLKCTRL) &
         (CM_CLKCTRL_MODULEMODE_MASK | CM_CLKCTRL_IDLEST_MASK))
         != (CM_CLKCTRL_MODULEMODE_ENABLE | CM_CLKCTRL_IDLEST_FUNC))
    {
    }

  while ((getreg32(AM335X_CM_PER_LCDC_CLKSTCTRL) & per_lcdc) != per_lcdc)
    {
    }

  while ((getreg32(AM335X_CM_PER_CPSW_CLKSTCTRL) & per_cpsw) != per_cpsw)
    {
    }

  while ((getreg32(AM335X_CM_WKUP_CLKSTCTRL) & wkup) != wkup)
    {
    }

  while ((getreg32(AM335X_CM_PER_L3S_CLKSTCTRL) & per_l3s) != per_l3s)
    {
    }

  while ((getreg32(AM335X_CM_PER_L3_CLKSTCTRL) & per_l3) != per_l3)
    {
    }

  while ((getreg32(AM335X_CM_PER_OCPWP_L3_CLKSTCTRL) & per_ocpwp_l3) !=
          per_ocpwp_l3)
    {
    }

  while ((getreg32(AM335X_CM_PER_L4LS_CLKSTCTRL) & per_l4ls) != per_l4ls)
    {
    }
}

/****************************************************************************
 * Name: am335x_dmtimer1ms_clockconfig
 ****************************************************************************/

static void am335x_dmtimer1ms_clockconfig(void)
{
  putreg32(CM_DPLL_DMTIMER1_CLKSEL_CLK_M_OSC,
           AM335X_CM_DPLL_CLKSEL_TIMER1MS_CLK);

  while ((getreg32(AM335X_CM_DPLL_CLKSEL_TIMER1MS_CLK) &
          CM_DPLL_DMTIMER1MS_CLKSEL_MASK)
          != CM_DPLL_DMTIMER1_CLKSEL_CLK_M_OSC)
    {
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_clockconfig
 *
 * Description:
 *   Called to initialize the AM335X.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void am335x_clockconfig(void)
{
  /* Don't change the current basic clock configuration if we are running
   * from SDRAM.  In this case, some bootloader logic has already configured
   * clocking and SDRAM.  We are pretty much committed to using things the
   * way that the bootloader has left them.
   *
   * Clocking will be configured at 792 MHz initially when started via
   * U-Boot.  The Linux kernel will uses the CPU frequency scaling code
   * which will switch the processor frequency between 400 MHz and 1GHz based
   * on load and temperature.  For now, NuttX simply leaves the clocking at
   * 792MHz.
   */

  am335x_interface_clocks_enable();

  am335x_power_domain_transition_enable();

  am335x_peripheral_enable();

  am335x_dmtimer1ms_clockconfig();

#ifndef CONFIG_AM335X_BOOT_SDRAM
#  warning Missing logic
#endif
}
