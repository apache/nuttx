/****************************************************************************
 * arch/z80/src/ez80/ez80_registerdump.c
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

#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "chip/switch.h"
#include "z80_internal.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Data
 ****************************************************************************/

static chipreg_t s_last_regs[XCPTCONTEXT_REGS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z80_registerdump
 ****************************************************************************/

void ez80_registerdump(void)
{
  volatile chipreg_t *regs = g_current_regs;

  if (regs == NULL)
    {
      ez80_saveusercontext(s_last_regs);
      regs = s_last_regs;
    }

#ifdef CONFIG_EZ80_Z80MODE
  _alert("AF: %04x  I: %04x\n",
        regs[XCPT_AF], regs[XCPT_I]);
  _alert("BC: %04x DE: %04x HL: %04x\n",
        regs[XCPT_BC], regs[XCPT_DE], regs[XCPT_HL]);
  _alert("IX: %04x IY: %04x\n",
        regs[XCPT_IX], regs[XCPT_IY]);
  _alert("SP: %04x PC: %04x\n"
        regs[XCPT_SP], regs[XCPT_PC]);
#else
  _alert("AF: %06x  I: %06x\n",
        regs[XCPT_AF], regs[XCPT_I]);
  _alert("BC: %06x DE: %06x HL: %06x\n",
        regs[XCPT_BC], regs[XCPT_DE], regs[XCPT_HL]);
  _alert("IX: %06x IY: %06x\n",
        regs[XCPT_IX], regs[XCPT_IY]);
  _alert("SP: %06x PC: %06x\n",
        regs[XCPT_SP], regs[XCPT_PC]);
#endif
}

#endif /* CONFIG_ARCH_STACKDUMP */
