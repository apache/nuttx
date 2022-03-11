/****************************************************************************
 * boards/arm/lpc17xx_40xx/lx_cpu/src/lpc17_40_ulan.c
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

#include <nuttx/serial/serial.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "chip.h"
#include "hardware/lpc17_40_uart.h"
#include "hardware/lpc17_40_pinconfig.h"
#include "hardware/lpc17_40_syscon.h"
#include "lpc17_40_gpio.h"
#include "arm_internal.h"

#include <arch/board/board.h>

#include "lx_cpu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nuttx_ulan_get_chip_data
 *
 * Description:
 *  Retrieve board specific data for uLAN driver
 *
 ****************************************************************************/

int nuttx_ulan_get_chip_data(int minor,
                             struct nuttx_ulan_chip_data_s *chip_data)
{
  uint32_t   regval;
  irqstate_t flags;

  if (minor > 0)
    {
      return 0;
    }

  /* Step 1: Enable power on UART1 */

  flags   = enter_critical_section();
  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCUART1;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Step 2: Enable clocking on UART */

#ifdef LPC176x
  regval = getreg32(LPC17_40_SYSCON_PCLKSEL0);
  regval &= ~SYSCON_PCLKSEL0_UART1_MASK;
  regval |= ((uint32_t)g_uart1priv.cclkdiv << SYSCON_PCLKSEL0_UART1_SHIFT);
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL0);
#endif

  lpc17_40_configgpio(GPIO_UART1_TXD);
  lpc17_40_configgpio(GPIO_UART1_RXD);
  lpc17_40_configgpio(GPIO_UART1_RTS);
  lpc17_40_configgpio(GPIO_UART1_DSR);
  lpc17_40_configgpio(GPIO_UART1_CTS);

  chip_data->chip = UL_DRV_SYSLESS_CHIP;
  chip_data->my_adr = UL_DRV_SYSLESS_MY_ADR_DEFAULT;
  chip_data->baud = UL_DRV_SYSLESS_BAUD;
  chip_data->baudbase = UL_DRV_SYSLESS_BAUDBASE;
  chip_data->irq = UL_DRV_SYSLESS_IRQ;
  chip_data->port = UL_DRV_SYSLESS_PORT;

  leave_critical_section(flags);

  return 1;
}
