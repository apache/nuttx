/****************************************************************************
 * arch/arm/src/stm32/stm32_dfumode.c
 *
 *   Copyright (C) 2019 Bill Gatliff. All rights reserved.
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Bill Gatliff <bgat@billgatliff.com>
 *           Gregory Nutt <gnutt@nuttx.org>
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
 *   https://community.st.com/s/question/0D50X00009XkhAzSAJ/calling-stm32429ieval-bootloader
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
