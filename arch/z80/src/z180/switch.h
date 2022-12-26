/************************************************************************************
 * arch/z80/src/z180/switch.h
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

#ifndef __ARCH_Z80_SRC_Z180_SWITCH_H
#define __ARCH_Z80_SRC_Z180_SWITCH_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/sched.h>

#include <nuttx/arch.h>
#include <arch/io.h>

#include "z180_iomap.h"
#include "z80_internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Macros for portability ***********************************************************
 *
 * Common logic in arch/z80/src/common is customized for the z180 context switching
 * logic via the following macros.
 */

/* Initialize the IRQ state */

#define INIT_IRQCONTEXT() \
  g_current_regs = NULL

/* IN_INTERRUPT returns true if the system is currently operating in the interrupt
 * context.  IN_INTERRUPT is the inline equivalent of up_interrupt_context().
 */

#define IN_INTERRUPT() \
  (g_current_regs != NULL)

/* The following macro declares the variables need by IRQ_ENTER and IRQ_LEAVE.
 * These variables are used to support nested interrupts.
 *
 * - savestate holds the previous value of current_state.
 * - savecpr holds the previous value of current_cpr.
 *
 * NOTE: Nested interrupts are not supported in this implementation.  If you want
 * to implement nested interrupts, you would have to change the way that
 * g_current_regs/cbr is handled.  The savestate/savecbr variables would not work
 * for that purpose as implemented here because only the outermost nested
 * interrupt can result in a context switch (they can probably be deleted).
 */

#define DECL_SAVESTATE() \
  FAR chipreg_t *savestate; \
  uint8_t savecbr;

/* The following macro is used when the system enters interrupt handling logic.
 * The entry values of g_current_regs and current_cbr and stored in local variables.
 * Then g_current_regs and current_cbr are set to the values of the interrupted
 * task.
 */

#define IRQ_ENTER(irq, regs) \
  do \
    { \
      savestate    = (FAR chipreg_t *)g_current_regs; \
      savecbr      = current_cbr; \
      g_current_regs = (regs); \
      current_cbr  = inp(Z180_MMU_CBR); \
    } \
  while (0)

/* The following macro is used when the system exits interrupt handling logic.
 * The value of g_current_regs is restored.  If we are not processing a nested
 * interrupt (meaning that we going to return to the user task), then also
 * set the MMU's CBR register.
 */

#define IRQ_LEAVE(irq) \
  do \
    { \
      g_current_regs = savestate; \
      if (g_current_regs) \
        { \
          current_cbr = savecbr; \
        } \
      else \
        { \
          outp(Z180_MMU_CBR, savecbr); \
        } \
    } \
  while (0)

/* The following macro is used to sample the interrupt state (as a opaque handle) */

#define IRQ_STATE() \
  (g_current_regs)

/* Save the current IRQ context in the specified TCB */

#define SAVE_IRQCONTEXT(tcb) \
  z180_copystate((tcb)->xcp.regs, (FAR chipreg_t*)g_current_regs)

/* Set the current IRQ context to the state specified in the TCB */

#define SET_IRQCONTEXT(tcb) \
  do \
    { \
      if ((tcb)->xcp.cbr) \
        { \
          current_cbr = (tcb)->xcp.cbr->cbr; \
        } \
      z180_copystate((FAR chipreg_t*)g_current_regs, (tcb)->xcp.regs); \
    } \
  while (0)

/* Save the user context in the specified TCB.  User context saves can be simpler
 * because only those registers normally saved in a C called need be stored.
 */

#define SAVE_USERCONTEXT(tcb)  \
  up_saveusercontext((tcb)->xcp.regs)

/* Restore the full context -- either a simple user state save or the full,
 * IRQ state save.
 */

#define RESTORE_USERCONTEXT(tcb) \
  do \
    { \
      if ((tcb)->xcp.cbr) \
        { \
          outp(Z180_MMU_CBR, (tcb)->xcp.cbr->cbr); \
        } \
      z180_restoreusercontext((tcb)->xcp.regs); \
    } \
  while (0)

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

/* This holds the value of the MMU's CBR register.  This value is set to the
 * interrupted tasks's CBR on interrupt entry, changed to the new task's CBR if
 * an interrupt level context switch occurs, and restored on interrupt exit.  In
 * this way, the CBR is always correct on interrupt exit.
 */

extern uint8_t current_cbr;
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

/* Defined in z180_copystate.c */

void z180_copystate(FAR chipreg_t *dest, FAR const chipreg_t *src);

/* Defined in z180_saveusercontext.asm */

int up_saveusercontext(FAR chipreg_t *regs);

/* Defined in z180_restoreusercontext.asm */

void z180_restoreusercontext(FAR chipreg_t *regs);

/* Defined in z180_sigsetup.c */

void z180_sigsetup(FAR struct tcb_s *tcb, sig_deliver_t sigdeliver,
                   FAR chipreg_t *regs);

#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_Z80_SRC_Z180_SWITCH_H */
