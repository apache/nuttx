/************************************************************************************
 * boards/lpc17xx_40xx/src/lpc17_40_ulan.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Pavel Pisa <ppisa@pikron.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <nuttx/serial/serial.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "chip.h"
#include "hardware/lpc17_40_uart.h"
#include "hardware/lpc17_40_pinconfig.h"
#include "hardware/lpc17_40_syscon.h"
#include "lpc17_40_gpio.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include <arch/board/board.h>

#include "lx_cpu.h"

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: nuttx_ulan_get_chip_data
 *
 * Description:
 *  Retrieve board specific data for uLAN driver
 *
 ************************************************************************************/

int nuttx_ulan_get_chip_data(int minor, struct nuttx_ulan_chip_data_s *chip_data)
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
