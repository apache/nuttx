/****************************************************************************
 * arch/arm/src/armv8-m/arm_stackcheck.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2013, 2014 PX4 Development Team.
 * All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

#ifdef CONFIG_ARMV8M_STACKCHECK

/* Support per function call stack checking.
 * This code uses R10 to check for a stack overflow within function calls.
 * This has a performance impact, but will be able to catch hard to find
 * stack overflows.
 */

#include <stdint.h>

#include <nuttx/instrument.h>
#include "arm_internal.h"
#include "nvic.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void stack_check_enter(void *func, void *caller, void *arg)
                              naked_function;
void __stack_overflow_trap(void) naked_function;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct instrument_s g_stack_check =
{
  .enter = stack_check_enter,
};

/****************************************************************************
 * Name: __stack_overflow_trap
 ****************************************************************************/

void __stack_overflow_trap(void)
{
  /* if we get here, the stack has overflowed */

  uint32_t regval;

#ifdef CONFIG_ARMV8M_STACKCHECK_BREAKPOINT
  regval = getreg32(NVIC_DHCSR);
  if (regval & NVIC_DHCSR_C_DEBUGEN)
    {
      __asm("bkpt 1");
    }
#endif

  /* force hard fault */

  regval  = getreg32(NVIC_INTCTRL);
  regval |= NVIC_INTCTRL_NMIPENDSET;
  putreg32(regval, NVIC_INTCTRL);

  /* XXX no need to trap it here, the fault handler gets to it */
}

/****************************************************************************
 * Name: stack_check_enter
 ****************************************************************************/

static void stack_check_enter(void *func, void *caller, void *arg)
{
    asm volatile (
            "   mrs r2, ipsr        \n" /* Check whether we are in interrupt mode */
            "   cmp r2, #0          \n" /* since we don't switch r10 on interrupt entry, we */
            "   bne 2f              \n" /* can't detect overflow of the interrupt stack. */
            "                       \n"
            "   sub r2, sp, #68     \n" /* compute stack pointer as though we just stacked a full frame */
            "   mrs r1, control     \n" /* Test CONTROL.FPCA to see whether we also need room for the FP */
            "   tst r1, #4          \n" /* context. */
            "   beq 1f              \n"
            "   sub r2, r2, #136    \n" /* subtract FP context frame size */
            "1:                     \n"
            "   cmp r2, r10         \n" /* compare stack with limit */
            "   bgt 2f              \n" /* stack is above limit and thus OK */
            "   mov r11, r2         \n" /* fault with R11 set to calculated low */
            "   b __stack_overflow_trap\n"
            "2:                     \n"
            "   bx lr               \n"
    );
}

/****************************************************************************
 * Name: arm_stack_check_init
 ****************************************************************************/

void noinstrument_function arm_stack_check_init(void)
{
  instrument_register(&g_stack_check);
}
#endif
