/****************************************************************************
 * arch/risc-v/src/ab32vg1/ab32vg1_clockconfig.c
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

#include "riscv_internal.h"
#include "chip.h"
#include "ab32vg1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System clock = PLL0 (240 MHz) / (SPLLDIV + 1) */

#define SPLL_DIV    (AB32VG1_PLL0_FREQUENCY / AB32VG1_SYSCLK_FREQUENCY - 1)

#if (AB32VG1_PLL0_FREQUENCY % AB32VG1_SYSCLK_FREQUENCY) != 0
#  error CONFIG_AB32VG1_SYSCLK_MHZ must divide 240 MHz
#endif

/* The internal SPI flash clock must stay at or below 24 MHz:
 * SPI0BAUD = sysclk / 24 MHz, rounded up, minus 1.
 */

#define SPI0_BAUD   ((AB32VG1_SYSCLK_FREQUENCY + 23999999) / 24000000 - 1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ab32vg1_busywait(volatile uint32_t loops)
{
  while (loops-- > 0)
    {
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ab32vg1_clockconfig
 *
 * Description:
 *   Switch the system clock to PLL0 / n.  Follows set_sysclk() from the
 *   RT-Thread AB32VG1 BSP (system_ab32vgx.c, Apache-2.0).  Must run before
 *   interrupts are enabled.
 *
 ****************************************************************************/

void ab32vg1_clockconfig(void)
{
  /* Start PLL0 at 240 MHz if the boot code did not */

  if ((getreg32(AB32VG1_PLL0CON) & PLL0CON_LDO_EN) == 0)
    {
      modifyreg32(AB32VG1_PLL0CON, PLL0CON_VCO_MASK, PLL0CON_VCO_HIGH);
      modifyreg32(AB32VG1_PLL0CON, 0, PLL0CON_LDO_EN);
      ab32vg1_busywait(1000);                   /* >= 100 us */
      putreg32(AB32VG1_PLL0DIV_240M, AB32VG1_PLL0DIV);
      modifyreg32(AB32VG1_PLL0CON, 0, PLL0CON_DIV_UPDATE);
      modifyreg32(AB32VG1_PLL0CON, 0, PLL0CON_ANALOG_EN);
      modifyreg32(AB32VG1_PLL0CON, 0, PLL0CON_SDM_EN);
      ab32vg1_busywait(10000);                  /* >= 1 ms to lock */
    }

  /* Run from the 2 MHz RC oscillator while the dividers change */

  modifyreg32(AB32VG1_CLKCON0, CLKCON0_SYSCLK_MASK, CLKCON0_SYSCLK_RC2M);
  modifyreg32(AB32VG1_CLKCON2, CLKCON2_SPLLDIV_MASK, 0);
  modifyreg32(AB32VG1_CLKCON0, CLKCON0_SPLL_MASK, CLKCON0_SPLL_PLL0);

  if (getreg32(AB32VG1_PLL0DIV) != AB32VG1_PLL0DIV_240M)
    {
      putreg32(AB32VG1_PLL0DIV_240M, AB32VG1_PLL0DIV);
      modifyreg32(AB32VG1_PLL0CON, PLL0CON_VCO_MASK, PLL0CON_VCO_HIGH);
      modifyreg32(AB32VG1_PLL0CON, 0, PLL0CON_DIV_UPDATE);
    }

  modifyreg32(AB32VG1_CLKCON2, CLKCON2_SPLLDIV_MASK,
              SPLL_DIV << CLKCON2_SPLLDIV_SHIFT);
  putreg32(SPI0_BAUD, AB32VG1_SPI0BAUD);
  modifyreg32(AB32VG1_CLKCON0, CLKCON0_SYSCLK_MASK, CLKCON0_SYSCLK_SPLL);
}
