/****************************************************************************
 * arch/z80/src/z8/z8_registerdump.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "chip/switch.h"
#include "z80_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void z8_dumpregs(FAR chipret_t *regs)
{
  _alert("REGS: %04x %04x %04x %04x %04x %04x %04x %04x\n",
         regs[XCPT_RR0], regs[XCPT_RR2], regs[XCPT_RR4], regs[XCPT_RR6],
         regs[XCPT_RR8], regs[XCPT_RR10], regs[XCPT_RR12], regs[XCPT_RR14]);
}

static inline void z8_dumpstate(chipreg_t sp, chipreg_t pc, uint8_t irqctl,
                                chipreg_t rpflags)
{
  _alert("SP: %04x PC: %04x IRQCTL: %02x RP: %02x FLAGS: %02x\n",
         sp, pc, irqctl & 0xff, rpflags >> 8, rpflags & 0xff);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

uintptr_t up_getusrsp(void)
{
  FAR chipreg_t *regs = g_z8irqstate.regs;
  return regs[XCPT_SP];
}

/****************************************************************************
 * Name: up_dump_register
 ****************************************************************************/

void up_dump_register(FAR void *dumpregs)
{
  FAR chipret_t *regs;
  chipreg_t      sp;
  uint16_t       rp;

  switch (g_z8irqstate.state)
    {
      case Z8_IRQSTATE_ENTRY:

        /* Calculate the source address based on the saved RP value */

        rp   = g_z8irqstate.regs[Z8_IRQSAVE_RPFLAGS] >> 8;
        regs = (FAR uint16_t *)(rp & 0xf0);

        /* Then dump the register values */

        z8_dumpregs(regs);

        /* Dump the saved machine state:
         * The g_z8irqstate.regs pointer is the value of the stack pointer at
         * the time that z80_doirq() was called.  Therefore, we can calculate
         * the correct value for the stack pointer on return from interrupt:
         */

        sp = ((chipreg_t)g_z8irqstate.regs) + Z8_IRQSAVE_SIZE;
        z8_dumpstate(sp, g_z8irqstate.regs[Z8_IRQSAVE_PC], 0x80,
                     g_z8irqstate.regs[Z8_IRQSAVE_RPFLAGS]);
        break;

      case Z8_IRQSTATE_SAVED:
        regs = g_z8irqstate.regs;
        z8_dumpregs(regs);
        z8_dumpstate(regs[XCPT_SP], regs[XCPT_PC],
                     regs[XCPT_IRQCTL], regs[XCPT_RPFLAGS]);
        break;

      case Z8_IRQSTATE_NONE:
      default:
        up_saveusercontext(s_last_regs);
        regs = s_last_regs;
        z8_dumpregs(regs);
        z8_dumpstate(regs[XCPT_SP], regs[XCPT_PC],
                     regs[XCPT_IRQCTL], regs[XCPT_RPFLAGS]);
        break;
    }
}
