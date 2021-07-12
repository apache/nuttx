/****************************************************************************
 * arch/arm/src/stm32/stm32_dfumode.c
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

#include "stm32_dfumode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  stm32_dfumode
 *
 * Description:
 *   Reboot the part in DFU mode (GCC only).
 *
 *   https://community.st.com/s/question/
 *                 0D50X00009XkhAzSAJ/calling-stm32429ieval-bootloader
 *
 *  REVISIT:  STM32_SYSMEM_BASE is not 0x1fff000 for all STM32's.  For F3's
 *  The SYSMEM base is at 0x1fffd800
 *
 *  REVISIT:  RCC_APB2ENR_SYSCFGEN is not bit 14 for all STM32's.  For F3's
 *  and L15's, it is bit 0.
 *
 *  REVISIT:  STM32 F3's do not support the SYSCFG_MEMRMP register.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
void stm32_dfumode(void)
{
#ifdef CONFIG_DEBUG_WARN
  _warn("Entering DFU mode...\n");
  nxsig_sleep(1);
#endif

  asm("ldr r0, =0x40023844\n\t"    /* RCC_APB2ENR */
      "ldr r1, =0x00004000\n\t"    /* Enable SYSCFG clock */
      "str r1, [r0, #0]\n\t"
      "ldr r0, =0x40013800\n\t"    /* SYSCFG_MEMRMP */
      "ldr r1, =0x00000001\n\t"    /* Map ROM at zero */
      "str r1, [r0, #0]\n\t"
      "ldr r0, =0x1fff0000\n\t"    /* ROM base */
      "ldr sp,[r0, #0]\n\t"        /* SP @ 0 */
      "ldr r0,[r0, #4]\n\t"        /* PC @ 4 */
      "bx r0\n");

  __builtin_unreachable();         /* Tell compiler we will not return */
}
#endif /* CONFIG_STM32_STM32F20XX || CONFIG_STM32_STM32F4XXX */
