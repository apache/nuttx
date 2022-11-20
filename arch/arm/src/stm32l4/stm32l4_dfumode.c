/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_dfumode.c
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

#include <unistd.h>
#include <debug.h>

#include <nuttx/signal.h>
#include "stm32l4_rcc.h"
#include "hardware/stm32l4_syscfg.h"
#include "stm32l4_dfumode.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (*boot_call_t)(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_STM32L4_STM32L4X6) || defined(CONFIG_STM32L4_STM32L4XR)
static inline void rcc_reset(void)
{
  uint32_t regval;

  /* Enable the MSI clock */

  regval = getreg32(STM32L4_RCC_CR);
  regval |= RCC_CR_MSION;
  putreg32(regval, STM32L4_RCC_CR);

  while (!(getreg32(STM32L4_RCC_CR) & RCC_CR_MSIRDY));

  /* Set MSI to 4MHz */

  regval = getreg32(STM32L4_RCC_CR);
  regval &= ~RCC_CR_MSIRANGE_MASK;
  regval |= RCC_CR_MSIRANGE_4M | RCC_CR_MSIRGSEL;
  putreg32(regval, STM32L4_RCC_CR);

  /* Reset CFGR register */

  putreg32(0x00000000, STM32L4_RCC_CFGR);

  /* Reset enable bits for other clocks than MSI */

  regval  = getreg32(STM32L4_RCC_CR);
  regval &= ~(RCC_CR_HSION | RCC_CR_HSIKERON | RCC_CR_HSEON |
              RCC_CR_HSIASFS | RCC_CR_CSSON | RCC_CR_PLLON |
              RCC_CR_PLLSAI1ON | RCC_CR_PLLSAI2ON);
  putreg32(regval, STM32L4_RCC_CR);

  /* Reset PLLCFGR register to reset default */

  putreg32(RCC_PLLCFG_RESET, STM32L4_RCC_PLLCFG);

  putreg32(0, STM32L4_RCC_PLLSAI1CFG);
  putreg32(RCC_PLLSAI1CFG_PLLN(16), STM32L4_RCC_PLLSAI1CFG);

  putreg32(0, STM32L4_RCC_PLLSAI2CFG);
  putreg32(RCC_PLLSAI2CFG_PLLN(16), STM32L4_RCC_PLLSAI1CFG);

  /* Reset HSEBYP bit */

  regval  = getreg32(STM32L4_RCC_CR);
  regval &= ~RCC_CR_HSEBYP;
  putreg32(regval, STM32L4_RCC_CR);

  /* Disable all interrupts */

  putreg32(0x00000000, STM32L4_RCC_CIER);
}

static inline void apb_reset(void)
{
  putreg32(0xffffffff, STM32L4_RCC_APB1RSTR1);
  putreg32(0xffffffff, STM32L4_RCC_APB1RSTR2);
  putreg32(0xffffffff, STM32L4_RCC_APB2RSTR);
  putreg32(0xffffffff, STM32L4_RCC_AHB1RSTR);
  putreg32(0xffffffff, STM32L4_RCC_AHB2RSTR);
  putreg32(0xffffffff, STM32L4_RCC_AHB3RSTR);

  putreg32(0, STM32L4_RCC_APB1RSTR1);
  putreg32(0, STM32L4_RCC_APB1RSTR2);
  putreg32(0, STM32L4_RCC_APB2RSTR);
  putreg32(0, STM32L4_RCC_AHB1RSTR);
  putreg32(0, STM32L4_RCC_AHB2RSTR);
  putreg32(0, STM32L4_RCC_AHB3RSTR);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  stm32l4_dfumode
 *
 * Description:
 *   Reboot the part in DFU mode (GCC only).
 *
 ****************************************************************************/

#if defined(CONFIG_STM32L4_STM32L4X6) || defined(CONFIG_STM32L4_STM32L4XR)
void stm32l4_dfumode(void)
{
  uint32_t regval;
  boot_call_t boot;

#ifdef CONFIG_DEBUG_WARN
  _warn("Entering DFU mode...\n");
#endif

  /* disable all peripherals, interrupts and switch to MSI */

  rcc_reset();
  apb_reset();

  /* remap ROM at address zero */

  regval = getreg32(STM32L4_RCC_APB2ENR);
  regval |= RCC_APB2ENR_SYSCFGEN;
  putreg32(regval, STM32L4_RCC_APB2ENR);
  putreg32(SYSCFG_MEMRMP_SYSTEM, STM32L4_SYSCFG_MEMRMP);

  /* set stack pointer and program-counter to ROM values */

  asm("dsb\n\t"
      "isb\n\t"
      "ldr r0, =0x1fff0000\n\t"    /* ROM base */
      "ldr sp,[r0, #0]\n\t"        /* SP @ 0 */
      );

  /* jump into ROM */

  boot = (boot_call_t)(*(uint32_t *)0x1fff0004);
  boot();

  while (true);

  __builtin_unreachable();         /* Tell compiler we will not return */
}
#endif
