/****************************************************************************
 * boards/arm/stm32h7/stm32h735g-dk/src/stm32_hyperram.c
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

#include <sys/param.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "stm32_gpio.h"
#include "hardware/stm32_octospi.h"
#include "hardware/stm32_rcc.h"
#include "stm32h735g-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HYPERRAM_GPIO (GPIO_ALT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t g_hyperram_pins[] =
{
  HYPERRAM_GPIO | GPIO_AF3 | GPIO_PULLUP | GPIO_PORTG | GPIO_PIN12, /* NCS */
  HYPERRAM_GPIO | GPIO_AF9 | GPIO_PORTF | GPIO_PIN4,                /* CLK */
  HYPERRAM_GPIO | GPIO_AF9 | GPIO_PORTF | GPIO_PIN12,               /* RWDS */
  HYPERRAM_GPIO | GPIO_AF9 | GPIO_PORTF | GPIO_PIN0,                /* D0 */
  HYPERRAM_GPIO | GPIO_AF9 | GPIO_PORTF | GPIO_PIN1,                /* D1 */
  HYPERRAM_GPIO | GPIO_AF9 | GPIO_PORTF | GPIO_PIN2,                /* D2 */
  HYPERRAM_GPIO | GPIO_AF9 | GPIO_PORTF | GPIO_PIN3,                /* D3 */
  HYPERRAM_GPIO | GPIO_AF9 | GPIO_PORTG | GPIO_PIN0,                /* D4 */
  HYPERRAM_GPIO | GPIO_AF9 | GPIO_PORTG | GPIO_PIN1,                /* D5 */
  HYPERRAM_GPIO | GPIO_AF3 | GPIO_PORTG | GPIO_PIN10,               /* D6 */
  HYPERRAM_GPIO | GPIO_AF9 | GPIO_PORTG | GPIO_PIN11                /* D7 */
};

static bool g_hyperram_initialized;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hyperram_initialize
 *
 * Description:
 *   Map the S70KL1281 HyperRAM at 0x70000000 for the LCD framebuffer.  Keep
 *   its reset configuration (six-clock fixed latency); use linear bursts.
 *   The HyperRAM RESET# signal is connected to the board reset.
 *
 ****************************************************************************/

int stm32_hyperram_initialize(void)
{
  uint32_t ccr;
  unsigned int i;

  if (g_hyperram_initialized)
    {
      return OK;
    }

  for (i = 0; i < nitems(g_hyperram_pins); i++)
    {
      stm32_configgpio(g_hyperram_pins[i]);
    }

  /* OCTOSPI kernel clock is HCLK (200 MHz), divided by four for 50 MHz.
   * The clock mux is shared with OCTOSPI1; neither NOR nor QSPI is used.
   */

  modifyreg32(STM32_RCC_D1CCIPR, RCC_D1CCIPR_QSPISEL_MASK,
              RCC_D1CCIPR_QSPISEL_HCLK);
  modifyreg32(STM32_RCC_AHB3ENR, 0,
              RCC_AHB3ENR_OSPI2EN | RCC_AHB3ENR_IOMNGREN);
  modifyreg32(STM32_RCC_AHB3RSTR, 0, RCC_AHB3RSTR_OSPI2RST);
  modifyreg32(STM32_RCC_AHB3RSTR, RCC_AHB3RSTR_OSPI2RST, 0);

  putreg32(OCTOSPIM_PCR_CLKEN | OCTOSPIM_PCR_CLKSRC_OSPI2 |
           OCTOSPIM_PCR_DQSEN | OCTOSPIM_PCR_DQSSRC_OSPI2 |
           OCTOSPIM_PCR_NCSEN | OCTOSPIM_PCR_NCSSRC_OSPI2 |
           OCTOSPIM_PCR_IOLEN | OCTOSPIM_PCR_IOLSRC_OSPI2 |
           OCTOSPIM_PCR_IOHEN | OCTOSPIM_PCR_IOHSRC_OSPI2,
           STM32_OCTOSPIM_P2CR);

  /* 16 MiB device, four-cycle CS high time, no delay-block calibration.
   * Split transactions at each 8 MiB die boundary and every four us to
   * allow the HyperRAM to refresh.
   */

  putreg32(OCTOSPI_DCR1_MTYP_HYPERBUS | OCTOSPI_DCR1_DEVSIZE(24) |
           OCTOSPI_DCR1_CSHT(4),
           STM32_OCTOSPI2_DCR1);
  putreg32(OCTOSPI_DCR2_PRESCALER(4), STM32_OCTOSPI2_DCR2);
  putreg32(OCTOSPI_DCR3_CSBOUND(23), STM32_OCTOSPI2_DCR3);
  putreg32(200, STM32_OCTOSPI2_DCR4);
  putreg32(OCTOSPI_TCR_DHQC, STM32_OCTOSPI2_TCR);
  putreg32(OCTOSPI_HLCR_LM | OCTOSPI_HLCR_TACC(6) |
           OCTOSPI_HLCR_TRWR(4), STM32_OCTOSPI2_HLCR);

  ccr = OCTOSPI_CCR_DQSE | OCTOSPI_CCR_DDTR |
        OCTOSPI_CCR_DMODE_8LINES | OCTOSPI_CCR_ADSIZE_32BITS |
        OCTOSPI_CCR_ADDTR | OCTOSPI_CCR_ADMODE_8LINES;
  putreg32(ccr, STM32_OCTOSPI2_CCR);
  putreg32(ccr, STM32_OCTOSPI2_WCCR);
  putreg32(OCTOSPI_CR_EN | OCTOSPI_CR_FTHRES(4) |
           OCTOSPI_CR_FMODE_MEMORYMAP, STM32_OCTOSPI2_CR);

  g_hyperram_initialized = true;
  return OK;
}
