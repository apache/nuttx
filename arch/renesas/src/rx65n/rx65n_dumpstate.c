/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_dumpstate.c
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

#include "up_arch.h"
#include "up_internal.h"
#include "sched/sched.h"
#include "chip.h"
#include "arch/rx65n/irq.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t s_last_regs[XCPTCONTEXT_REGS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_getusersp
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
static inline uint16_t rx65n_getusersp(void)
{
  uint8_t *ptr = (uint8_t *) g_current_regs;
  return (uint16_t)ptr[REG_SP] << 8 | ptr[REG_SP + 1]; /* check */
}
#endif

/****************************************************************************
 * Name: rx65n_stackdump
 ****************************************************************************/

static void rx65n_stackdump(uint16_t sp, uint16_t stack_base)
{
  uint16_t stack;

  for (stack = sp & ~7; stack < stack_base; stack += 8) /* check */

    {
      uint8_t *ptr = (uint8_t *)&stack;
      _alert("%04x: %02x %02x %02x %02x %02x %02x %02x %02x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

/****************************************************************************
 * Name: rx65n_registerdump
 ****************************************************************************/

static inline void rx65n_registerdump(void)
{
  uint8_t *ptr = (uint8_t *) g_current_regs;
  uint32_t regs[XCPTCONTEXT_SIZE];

  /* Are user registers available from interrupt processing? */

  if (ptr == NULL)
    {
      /* No.. capture user registers by hand */

      up_saveusercontext((uint32_t *)s_last_regs);
       *regs = *s_last_regs;
    }

  /* Dump the interrupt registers */

  _alert("PC: %08x PSW=%08x\n",
        ptr[REG_PC], ptr[REG_PSW]);

  _alert("FPSW: %08x ACC0LO: %08x ACC0HI: %08x ACC0GU: %08x"
         "ACC1LO: %08x ACC1HI: %08x ACC1GU: %0.8x\n",
         ptr[REG_FPSW], ptr[REG_ACC0LO], ptr[REG_ACC0HI],
         ptr[REG_ACC0GU], ptr[REG_ACC1LO],
         ptr[REG_ACC1HI], ptr[REG_ACC1GU]);

  _alert("R%d:%08x %08x %08x %08x %08x %08x %08x\n", 0,
        ptr[REG_R1], ptr[REG_R2], ptr[REG_R3],
        ptr[REG_R4], ptr[REG_R5], ptr[REG_R6], ptr[REG_R7]);

  _alert("R%d: %08x %08x %08x %08x %08x %08x %08x %08x\n", 8,
        ptr[REG_R8], ptr[REG_R9], ptr[REG_R10], ptr[REG_R11],
        ptr[REG_R12], ptr[REG_R13], ptr[REG_R14], ptr[REG_R15]);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dumpstate
 ****************************************************************************/

void up_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uint32_t sp = renesas_getsp();
  uint32_t ustackbase;
  uint32_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  uint32_t istackbase;
  uint32_t istacksize;
#endif

  /* Dump the registers (if available) */

  rx65n_registerdump();

  /* Get the limits on the user stack memory */

  ustackbase = (uint32_t)rtcb->adj_stack_ptr;
  ustacksize = (uint16_t)rtcb->adj_stack_size;

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  istackbase = ebss; /* check how to declare ebss, as of now declared in chip.h */

  istacksize = CONFIG_ARCH_INTERRUPTSTACK;

  /* Show interrupt stack info */

  _alert("sp:     %04x\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %04x\n", istackbase);
  _alert("  size: %04x\n", istacksize);

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp <= istackbase && sp > istackbase - istacksize)
    {
      /* Yes.. dump the interrupt stack */

      rx65n_stackdump(sp, istackbase);

      /* Extract the user stack pointer from the register area */

      sp = rx65n_getusersp();
      _alert("sp:     %04x\n", sp);
    }
  else if (g_current_regs)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      rx65n_stackdump(istackbase - istacksize, istackbase);
    }

  /* Show user stack info */

  _alert("User stack:\n");
  _alert("  base: %04x\n", ustackbase);
  _alert("  size: %04x\n", ustacksize);
#else
  _alert("sp:         %04x\n", sp);
  _alert("stack base: %04x\n", ustackbase);
  _alert("stack size: %04x\n", ustacksize);
#endif

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp > ustackbase || sp <= ustackbase - ustacksize)
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      rx65n_stackdump(ustackbase - ustacksize, ustackbase);
    }
  else
    {
      rx65n_stackdump(sp, ustackbase);
    }
}

#endif /* CONFIG_ARCH_STACKDUMP */
