/************************************************************************************
 * arch/z80/src/ez80/switch.h
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
 ************************************************************************************/

#ifndef __ARCH_Z80_SRC_EZ80_SWITCH_H
#define __ARCH_Z80_SRC_EZ80_SWITCH_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
#  include <nuttx/sched.h>
#  include <nuttx/arch.h>
#endif
#include "z80_internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Macros for portability ***********************************************************
 *
 * Common logic in arch/z80/src/common is customized for the z8 context switching
 * logic via the following macros.
 */

/* Initialize the IRQ state */

#define INIT_IRQCONTEXT()        g_current_regs = NULL

/* IN_INTERRUPT returns true if the system is currently operating in the interrupt
 * context.  IN_INTERRUPT is the inline equivalent of up_interrupt_context().
 */

#define IN_INTERRUPT()           (g_current_regs != NULL)

/* The following macro is used when the system enters interrupt handling logic
 *
 * NOTE: Nested interrupts are not supported in this implementation.  If you want
 * to implement nested interrupts, you would have to change the way that
 * g_current_regs is handled.  The savestate variable would not work for
 * that purpose as implemented here because only the outermost nested
 * interrupt can result in a context switch (it can probably be deleted).
 */

#define DECL_SAVESTATE() \
  FAR chipreg_t *savestate

#define IRQ_ENTER(irq, regs) \
  do { \
    savestate    = (FAR chipreg_t *)g_current_regs; \
    g_current_regs = (regs); \
  } while (0)

/* The following macro is used when the system exits interrupt handling logic */

#define IRQ_LEAVE(irq)           g_current_regs = savestate

/* The following macro is used to sample the interrupt state (as a opaque handle) */

#define IRQ_STATE()              (g_current_regs)

/* Save the current IRQ context in the specified TCB */

#define SAVE_IRQCONTEXT(tcb)     ez80_copystate((tcb)->xcp.regs, (FAR chipreg_t*)g_current_regs)

/* Set the current IRQ context to the state specified in the TCB */

#define SET_IRQCONTEXT(tcb)      ez80_copystate((FAR chipreg_t*)g_current_regs, (tcb)->xcp.regs)

/* Save the user context in the specified TCB.  User context saves can be simpler
 * because only those registers normally saved in a C called need be stored.
 */

#define SAVE_USERCONTEXT(tcb)    ez80_saveusercontext((tcb)->xcp.regs)

/* Restore the full context -- either a simple user state save or the full,
 * IRQ state save.
 */

#define RESTORE_USERCONTEXT(tcb) ez80_restorecontext((tcb)->xcp.regs)

/* Dump the current machine registers */

#define _REGISTER_DUMP()         ez80_registerdump()

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
/* This holds a references to the current interrupt level register storage structure.
 * If is non-NULL only during interrupt processing.
 */

extern volatile chipreg_t *g_current_regs;
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

/* Defined in ez80_copystate.c */

void ez80_copystate(FAR chipreg_t *dest, FAR const chipreg_t *src);

/* Defined in ez80_saveusercontext.asm */

int ez80_saveusercontext(FAR chipreg_t *regs);

/* Defined in ez80_restorecontext.asm */

void ez80_restorecontext(FAR chipreg_t *regs);

/* Defined in ez80_sigsetup.c */

void ez80_sigsetup(FAR struct tcb_s *tcb, sig_deliver_t sigdeliver, chipreg_t *regs);

/* Defined in ez80_registerdump.c */

void ez80_registerdump(void);

#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_Z80_SRC_EZ80_SWITCH_H */
