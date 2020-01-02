/****************************************************************************
 * arch/arm/src/armv7-m/up_hardfault.c
 *
 *   Copyright (C) 2009, 2013, 2016, 2018 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/userspace.h>
#include <arch/irq.h>

#include "up_arch.h"
#include "nvic.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If CONFIG_ARMV7M_USEBASEPRI=n, then debug output from this file may
 * interfere with context switching!
 */

#ifdef CONFIG_DEBUG_HARDFAULT_ALERT
# define hfalert(format, ...)  _alert(format, ##__VA_ARGS__)
#else
# define hfalert(x...)
#endif

#ifdef CONFIG_DEBUG_HARDFAULT_INFO
# define hfinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
# define hfinfo(x...)
#endif

#define INSN_SVC0        0xdf00 /* insn: svc 0 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_hardfault
 *
 * Description:
 *   This is Hard Fault exception handler.  It also catches SVC call
 *   exceptions that are performed in bad contexts.
 *
 ****************************************************************************/

int up_hardfault(int irq, FAR void *context, FAR void *arg)
{
  /* Get the value of the program counter where the fault occurred */

#ifndef CONFIG_ARMV7M_USEBASEPRI
  uint32_t *regs = (uint32_t *)context;
  uint16_t *pc = (uint16_t *)regs[REG_PC] - 1;

  /* Check if the pc lies in known FLASH memory.
   * REVISIT:  What if the PC lies in "unknown" external memory?  Best
   * use the BASEPRI register if you have external memory.
   */

#ifdef CONFIG_BUILD_PROTECTED
  /* In the kernel build, SVCalls are expected in either the base, kernel
   * FLASH region or in the user FLASH region.
   */

  if (((uintptr_t)pc >= (uintptr_t)_START_TEXT &&
       (uintptr_t)pc <  (uintptr_t)_END_TEXT) ||
      ((uintptr_t)pc >= (uintptr_t)USERSPACE->us_textstart &&
       (uintptr_t)pc <  (uintptr_t)USERSPACE->us_textend))
#else
  /* SVCalls are expected only from the base, kernel FLASH region */

  if ((uintptr_t)pc >= (uintptr_t)_START_TEXT &&
      (uintptr_t)pc <  (uintptr_t)_END_TEXT)
#endif
    {
      /* Fetch the instruction that caused the Hard fault */

      uint16_t insn = *pc;
      hfinfo("  PC: %p INSN: %04x\n", pc, insn);

      /* If this was the instruction 'svc 0', then forward processing
       * to the SVCall handler
       */

      if (insn == INSN_SVC0)
        {
          hfinfo("Forward SVCall\n");
          return up_svcall(irq, context, arg);
        }
    }
#endif

  /* Dump some hard fault info */

  hfalert("Hard Fault:\n");
  hfalert("  IRQ: %d regs: %p\n", irq, context);
  hfalert("  BASEPRI: %08x PRIMASK: %08x IPSR: %08x CONTROL: %08x\n",
          getbasepri(), getprimask(), getipsr(), getcontrol());
  hfalert("  CFAULTS: %08x HFAULTS: %08x DFAULTS: %08x BFAULTADDR: %08x AFAULTS: %08x\n",
          getreg32(NVIC_CFAULTS), getreg32(NVIC_HFAULTS),
          getreg32(NVIC_DFAULTS), getreg32(NVIC_BFAULT_ADDR),
          getreg32(NVIC_AFAULTS));

  up_irq_save();
  _alert("PANIC!!! Hard fault: %08x\n", getreg32(NVIC_HFAULTS));
  PANIC();
  return OK;
}
