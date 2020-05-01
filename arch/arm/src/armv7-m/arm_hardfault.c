/****************************************************************************
 * arch/arm/src/armv7-m/arm_hardfault.c
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
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/userspace.h>
#include <arch/irq.h>

#include "arm_arch.h"
#include "nvic.h"
#include "arm_internal.h"

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
 * Name: arm_hardfault
 *
 * Description:
 *   This is Hard Fault exception handler.  It also catches SVC call
 *   exceptions that are performed in bad contexts.
 *
 ****************************************************************************/

int arm_hardfault(int irq, FAR void *context, FAR void *arg)
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
          return arm_svcall(irq, context, arg);
        }
    }
#endif

  /* Dump some hard fault info */

  hfalert("Hard Fault:\n");
  hfalert("  IRQ: %d regs: %p\n", irq, context);
  hfalert("  BASEPRI: %08x PRIMASK: %08x IPSR: %08x CONTROL: %08x\n",
          getbasepri(), getprimask(), getipsr(), getcontrol());
  hfalert("  CFAULTS: %08x HFAULTS: %08x DFAULTS: %08x BFAULTADDR: %08x "
          "AFAULTS: %08x\n",
          getreg32(NVIC_CFAULTS), getreg32(NVIC_HFAULTS),
          getreg32(NVIC_DFAULTS), getreg32(NVIC_BFAULT_ADDR),
          getreg32(NVIC_AFAULTS));

  up_irq_save();
  _alert("PANIC!!! Hard fault: %08x\n", getreg32(NVIC_HFAULTS));
  PANIC();
  return OK;
}
