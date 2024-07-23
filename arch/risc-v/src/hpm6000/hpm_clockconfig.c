/****************************************************************************
 * arch/risc-v/src/hpm6000/hpm_clockconfig.c
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

#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "hardware/hpm6300/hpm6300_sysctl.h"
#include "hardware/hpm_memorymap.h"
#include "hardware/hpm_pllctl.h"
#include "riscv_internal.h"
#include "chip.h"
#include "hpm.h"
#include "hpm_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EXT_OSC 24000000

#define PLLCTLV2_PLL_MFN_FACTOR  (10U)               /* PLLCTLV2 PLL MFN Factor */
#define PLLCTLV2_PLL_MFD_DEFAULT (240UL * 1000000UL) /* PLLCTLV2 PLL Default MFD value */

#define PLLCTLV2_PLL_MFI_MIN     (16U)
#define PLLCTLV2_PLL_MFI_MAX     (42U)
#define PLLCTLV2_PLL_XTAL_FREQ   (24000000UL)

#define PLLCTLV2_PLL_FREQ_MIN    (PLLCTLV2_PLL_MFI_MIN * PLLCTLV2_PLL_XTAL_FREQ)
#define PLLCTLV2_PLL_FREQ_MAX    ((PLLCTLV2_PLL_MFI_MAX + 1U) * PLLCTLV2_PLL_XTAL_FREQ)

#define BUS_FREQ_MAX             (166000000UL)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm6750_get_osc_freq
 ****************************************************************************/

uint32_t hpm_get_osc_freq(void)
{
  return EXT_OSC;
}

/****************************************************************************
 * Name: hpm6750_clockconfig
 ****************************************************************************/

void hpm_clockconfig(void)
{
  uint32_t value;

  value = getreg32(0xf40c4010);
  value &= ~0xfff;
  value |= 0x1044c;
  putreg32(value, 0xf40c4010);

  /* uart should configure pin function before opening clock */

  value = 0xffffffff &
          ~(SYSCTL_GROUP0_LINK0_UART0 | SYSCTL_GROUP0_LINK0_UART1 |
            SYSCTL_GROUP0_LINK0_UART2 | SYSCTL_GROUP0_LINK0_UART3 |
            SYSCTL_GROUP0_LINK0_UART4 | SYSCTL_GROUP0_LINK0_UART5 |
            SYSCTL_GROUP0_LINK0_UART6);
  putreg32(0x01f7ffff, HPM_SYSCTL_GROUP0_LINK0_VALUE);

  value = 0xffffffff & ~SYSCTL_GROUP0_LINK1_UART7;
  putreg32(0x3dfffffe, HPM_SYSCTL_GROUP0_LINK1_VALUE);

  /* Connect Group0 to CPU0 */

  putreg32(1, HPM_SYSCTL_AFFILIATE_CPU0_SET);

  value = getreg32(HPM_SYSCTL_CLOCK_CPU_CLK_TOP_CPU0);
  if ((value & 0xff) == 1)
    {
      value = SYSCTL_CLOCK_CPU_MUX_PLL1_CLK0 |
              SYSCTL_CLOCK_CPU_DIV(CPU_DIV) |
              SYSCTL_CLOCK_CPU_SUB0_DIV(AXI_SUB_DIV - 1) |
              SYSCTL_CLOCK_CPU_SUB1_DIV(AHB_SUB_DIV - 1);
      putreg32(value, HPM_SYSCTL_CLOCK_CPU_CLK_TOP_CPU0);
      while (getreg32(HPM_SYSCTL_CLOCK_CPU_CLK_TOP_CPU0) & 0x80000000);
    }

  /* Configure CPU0 clock & AXI Sub-clock & AHB Sub-clock */

  value = SYSCTL_CLOCK_CPU_MUX_PLL1_CLK0 |
          SYSCTL_CLOCK_CPU_DIV(CPU_DIV - 1) |
          SYSCTL_CLOCK_CPU_SUB0_DIV(AXI_SUB_DIV - 1) |
          SYSCTL_CLOCK_CPU_SUB1_DIV(AHB_SUB_DIV - 1);
  putreg32(value, HPM_SYSCTL_CLOCK_CPU_CLK_TOP_CPU0);
  while (getreg32(HPM_SYSCTL_CLOCK_CPU_CLK_TOP_CPU0) & 0x80000000);

#if defined (CONFIG_ARCH_FAMILY_HPM6300)
  /* Configure PLL1_CLK0 Post Divider */

  value = (getreg32(HPM_PLLCTLV2_PLL1_DIV0) &
          ~HPM_PLLCTLV2_PLL_DIV_DIV_MASK) |
          HPM_PLLCTLV2_PLL_DIV_DIV(PLL1_DIV) |
          HPM_PLLCTLV2_PLL_DIV_ENABLE;
  putreg32(value, HPM_PLLCTLV2_PLL1_DIV0);
  while (getreg32(HPM_PLLCTLV2_PLL1_DIV0) & 0x80000000);
#endif
  /* Configure PLL1 clock frequencey */

  value = PLL1_FREQ / PLLCTLV2_PLL_XTAL_FREQ - 1;
  putreg32(value, HPM_PLLCTLV2_PLL1_MFI);
  while (getreg32(HPM_PLLCTLV2_PLL1_MFI) & 0x80000000);
  putreg32(value + 1, HPM_PLLCTLV2_PLL1_MFI);
  while (getreg32(HPM_PLLCTLV2_PLL1_MFI) & 0x80000000);

  value = PLL1_FREQ % PLLCTLV2_PLL_XTAL_FREQ * PLLCTLV2_PLL_MFN_FACTOR;
  putreg32(value, HPM_PLLCTLV2_PLL1_MFN);
  while (getreg32(HPM_PLLCTLV2_PLL1_MFN) & 0x80000000);

  value = (getreg32(HPM_SYSCTL_CLOCK_CLK_TOP_MCT0) &
          ~(SYSCTL_CLOCK_MUX_MASK | SYSCTL_CLOCK_DIV_MASK)) |
          (SYSCTL_CLOCK_MUX_OSC0_CLK0 | SYSCTL_CLOCK_DIV(1));
  putreg32(value, HPM_SYSCTL_CLOCK_CLK_TOP_MCT0);
  while (getreg32(HPM_SYSCTL_CLOCK_CLK_TOP_MCT0) & 0x40000000);
}

void hpm_uart_clockconfig(void)
{
  uint32_t value;

  value = getreg32(HPM_SYSCTL_GROUP0_LINK0_VALUE);

#ifdef CONFIG_HPM_UART0
  putreg32(SYSCTL_CLOCK_MUX_OSC0_CLK0 | SYSCTL_CLOCK_DIV(1),
           HPM_SYSCTL_CLOCK_CLK_TOP_UART0);
  while (getreg32(HPM_SYSCTL_CLOCK_CLK_TOP_UART0) & 0x40000000);
  value |= SYSCTL_GROUP0_LINK0_UART0;
#endif

#ifdef CONFIG_HPM_UART1
  putreg32(SYSCTL_CLOCK_MUX_OSC0_CLK0 | SYSCTL_CLOCK_DIV(1),
           HPM_SYSCTL_CLOCK_CLK_TOP_UART1);
  value |= SYSCTL_GROUP0_LINK0_UART1;
#endif

#if defined CONFIG_HPM_UART2
  putreg32(SYSCTL_CLOCK_MUX_OSC0_CLK0 | SYSCTL_CLOCK_DIV(1),
           HPM_SYSCTL_CLOCK_CLK_TOP_UART2);
  value |= SYSCTL_GROUP0_LINK0_UART2;
#endif

#if defined CONFIG_HPM_UART3
  putreg32(SYSCTL_CLOCK_MUX_OSC0_CLK0 | SYSCTL_CLOCK_DIV(1),
           HPM_SYSCTL_CLOCK_CLK_TOP_UART3);
  value |= SYSCTL_GROUP0_LINK0_UART3;
#endif

#if defined CONFIG_HPM_UART4
  putreg32(SYSCTL_CLOCK_MUX_OSC0_CLK0 | SYSCTL_CLOCK_DIV(1),
           HPM_SYSCTL_CLOCK_CLK_TOP_UART4);
  value |= SYSCTL_GROUP0_LINK0_UART4;
#endif

#if defined CONFIG_HPM_UART5
  putreg32(SYSCTL_CLOCK_MUX_OSC0_CLK0 | SYSCTL_CLOCK_DIV(1),
           HPM_SYSCTL_CLOCK_CLK_TOP_UART5);
  value |= SYSCTL_GROUP0_LINK0_UART5;
#endif

#if defined CONFIG_HPM_UART6
  putreg32(SYSCTL_CLOCK_MUX_OSC0_CLK0 | SYSCTL_CLOCK_DIV(1),
           HPM_SYSCTL_CLOCK_CLK_TOP_UART6);
  value |= SYSCTL_GROUP0_LINK0_UART6;
#endif

  putreg32(value, HPM_SYSCTL_GROUP0_LINK0_SET);

#ifdef CONFIG_HPM_UART7
  putreg32(SYSCTL_CLOCK_MUX_OSC0_CLK0 | SYSCTL_CLOCK_DIV(1),
           HPM_SYSCTL_CLOCK_CLK_TOP_UART7);
  value = getreg32(HPM_SYSCTL_GROUP0_LINK1_VALUE);
  value |= SYSCTL_GROUP0_LINK1_UART7;
  putreg32(value, HPM_SYSCTL_GROUP0_LINK1_SET);
#endif
}
