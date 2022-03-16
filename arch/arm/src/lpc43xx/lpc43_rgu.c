/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_rgu.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "nvic.h"
#include "arm_internal.h"
#include "chip.h"
#include "lpc43_rgu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_softreset
 *
 * Description:
 *   Reset as many of the LPC43 peripherals as possible. This is necessary
 *   because the LPC43 does not provide any way of performing a full system
 *   reset under debugger control.  So, if CONFIG_DEBUG_FEATURES is set
 *   (indicating that a debugger is being used?), the boot logic will call
 *   this function on all restarts.
 *
 * Assumptions:
 *   Since this function is called early in the boot sequence, it cannot
 *   depend on anything such as initialization of .bss or .data.  It can
 *   only assume that it has a stack.
 *
 ****************************************************************************/

void lpc43_softreset(void)
{
  irqstate_t flags;

  /* Disable interrupts */

  flags = enter_critical_section();

  /* Reset all of the peripherals that we can (safely) */

  putreg32((RGU_CTRL0_LCD_RST     | RGU_CTRL0_USB0_RST     |
            RGU_CTRL0_USB1_RST    | RGU_CTRL0_DMA_RST      |
            RGU_CTRL0_SDIO_RST    | RGU_CTRL0_ETHERNET_RST |
            RGU_CTRL0_GPIO_RST), LPC43_RGU_CTRL0);
  putreg32((RGU_CTRL1_TIMER0_RST  | RGU_CTRL1_TIMER1_RST   |
            RGU_CTRL1_TIMER2_RST  | RGU_CTRL1_TIMER3_RST   |
            RGU_CTRL1_RITIMER_RST | RGU_CTRL1_SCT_RST      |
            RGU_CTRL1_MCPWM_RST   | RGU_CTRL1_QEI_RST      |
            RGU_CTRL1_ADC0_RST    | RGU_CTRL1_ADC1_RST     |
            RGU_CTRL1_USART0_RST  | RGU_CTRL1_UART1_RST    |
            RGU_CTRL1_USART2_RST  | RGU_CTRL1_USART3_RST   |
            RGU_CTRL1_I2C0_RST    | RGU_CTRL1_I2C1_RST     |
            RGU_CTRL1_SSP0_RST    | RGU_CTRL1_SSP1_RST     |
            RGU_CTRL1_I2S_RST     | RGU_CTRL1_CAN1_RST     |
            RGU_CTRL1_CAN0_RST    | RGU_CTRL1_M0APP_RST),
            LPC43_RGU_CTRL1);

  /* A delay seems to be necessary somewhere around here */

  up_mdelay(20);

  /* Clear all pending interrupts */

  putreg32(0xffffffff, NVIC_IRQ0_31_CLRPEND);
  putreg32(0xffffffff, NVIC_IRQ32_63_CLRPEND);
  leave_critical_section(flags);
}
