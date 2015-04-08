/************************************************************************************
 * arch/arm/src/efm32/efm32_vectors.S
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <arch/irq.h>

#include "chip.h"
#include "exc_return.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
  /* In kernel mode without an interrupt stack, this interrupt handler will set the
   * MSP to the stack pointer of the interrupted thread.  If the interrupted thread
   * was a privileged thread, that will be the MSP otherwise it will be the PSP.  If
   * the PSP is used, then the value of the MSP will be invalid when the interrupt
   * handler returns because it will be a pointer to an old position in the
   * unprivileged stack.  Then when the high priority interrupt occurs and uses this
   * stale MSP, there will most likely be a system failure.
   *
   * If the interrupt stack is selected, on the other hand, then the interrupt
   * handler will always set the the MSP to the interrupt stack.  So when the high
   * priority interrupt occurs, it will either use the MSP of the last privileged
   * thread to run or, in the case of the nested interrupt, the interrupt stack if
   * no privileged task has run.
   */

#  if defined(CONFIG_NUTTX_KERNEL) && CONFIG_ARCH_INTERRUPTSTACK < 4
#    error Interrupt stack must be used with high priority interrupts in kernel mode
#  endif

  /* Use the the BASEPRI to control interrupts is required if nested, high
   * priority interrupts are supported.
   */

#  ifndef CONFIG_ARMV7M_USEBASEPRI
#    error CONFIG_ARMV7M_USEBASEPRI must be used with CONFIG_ARCH_HIPRI_INTERRUPT
#  endif
#endif

#define IDLE_STACK		(_ebss+CONFIG_IDLETHREAD_STACKSIZE-4)
#define HEAP_BASE		(_ebss+CONFIG_IDLETHREAD_STACKSIZE)

/************************************************************************************
 * Global Symbols
 ************************************************************************************/

	.syntax		unified
	.thumb
	.file		"efm32_vectors.S"

/* The EFM32 chips all use the common ARMv7 interrupt vectoring.
 * (see arch/arm/src/armv7-m/up_vectors.S)
 */

/************************************************************************************
 * .rodata
 ************************************************************************************/

	.section	.rodata, "a"

/* Variables: _sbss is the start of the BSS region (see ld.script) _ebss is the end
 * of the BSS regsion (see ld.script). The idle task stack starts at the end of BSS
 * and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread is the thread that
 * the system boots on and, eventually, becomes the idle, do nothing task that runs
 * only when there is nothing else to run.  The heap continues from there until the
 * end of memory.  See g_idle_topstack below.
 */

	.globl	g_idle_topstack
	.type	g_idle_topstack, object
g_idle_topstack:
	.word	HEAP_BASE
	.size	g_idle_topstack, .-g_idle_topstack

	.end
