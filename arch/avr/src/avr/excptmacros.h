/****************************************************************************
 * arch/avr/src/avr/excptmacros.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_AVR_SRC_AVR_EXCPTMACROS_H
#define __ARCH_AVR_SRC_AVR_EXCPTMACROS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef __ASSEMBLY__

#include <arch/irq.h>

#include <avr/io.h>
#include <avr/sfr_defs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  .global g_intstackalloc
  .global g_intstacktop
  .global g_nestlevel
#endif

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

/****************************************************************************
 * General Exception Handling Example:
 *
 *  HANDLER IRQ_X, my_exception
 *  ...
 * my_exception:
 *  EXCPT_PROLOGUE            - Save registers on stack
 *  in r22, _SFR_IO_ADDR(SPL) - Pass register save structure as
 *                              the parameter 2
 *  in r23, _SFR_IO_ADDR(SPH) - (Careful, push post-decrements)
 *  USE_INTSTACK rx, ry, rz   - Switch to the interrupt stack
 *  call handler              - Handle the exception IN=old regs OUT=new regs
 *  RESTORE_STACK rx, ry      - Undo the operations of USE_INTSTACK
 *  EXCPT_EPILOGUE            - Return to the context returned by handler()
 *  reti                      - Return from interrupt
 *
 ****************************************************************************/

/****************************************************************************
 * Name: HANDLER
 *
 * Description:
 *  This macro provides the exception entry logic.  It is called with the
 *  PC already on the stack.  It simply saves one register on the  stack
 *  (r24) and passes the IRQ number to common logic (see EXCPT_PROLOGUE).
 *
 * On Entry:
 *  sp - Points to the top of the stack.  The PC is already on the stack.
 *  Only the stack is available for storage
 *
 *   PC1
 *   PC0
 *   --- <- SP
 *
 * At completion:
 *  Stack pointer is incremented by one, the saved r24 is on the stack,
 *  r24 now contains the IRQ number
 *
 *   PC1
 *   PC0
 *   R0
 *   --- <- SP
 *
 ****************************************************************************/

  .macro HANDLER, label, irqno, common
  .global \label
\label:
  push r24
  ldi r24, \irqno
  rjmp \common
  .endm

/****************************************************************************
 * Name: EXCPT_PROLOGUE
 *
 * Description:
 *  Provides the common "prologue" logic that should appear at the beginning
 *  of the exception handler.
 *
 * On Entry:
 *  r24 - has already been pushed onto the stack and now holds the IRQ number
 *  sp - Points to the top of the stack
 *  Only the stack is available for storage
 *
 *   PC1
 *   PC0
 *   R24
 *   --- <- SP
 *
 * At completion:
 *  Register state is saved on the stack; All registers are available for
 *  usage except sp and r24 which still contains the IRQ number as set by the
 *  HANDLER macro.
 *
 ****************************************************************************/

  .macro EXCPT_PROLOGUE

  /* Save R25  */

  push r25

  /* Save the status register on the stack */

  in r25, _SFR_IO_ADDR(SREG) /* Save the status register */
  cli                        /* Disable interrupts (not necessary) */
  ori r25, (1 << SREG_I)     /* Interrupts re-enabled on restore */
  push r25

  /* Save RAMPZ if the MCU has it */

#ifdef AVR_HAS_RAMPZ
  in r25, _SFR_IO_ADDR(RAMPZ)
  push r25
#endif

  /* Save R0 -- the scratch register and the zero register
   * (which may not be zero).  R1 must be zero for our purposes
   */

  push r0
  push r1
  clr r1

  /* Save r2-r17 - Call-saved, "static" registers */

  push r2
  push r3
  push r4
  push r5
  push r6
  push r7
  push r8
  push r9
  push r10
  push r11
  push r12
  push r13
  push r14
  push r15
  push r16
  push r17

  /* Save r18-r27 - Call-used, "volatile" registers (r24 was saved by
   * HANDLER, r15 was saved above, and r26-r27 saved later, out of sequence)
   */

  push r18
  push r19
  push r20
  push r21
  push r22
  push r23

  /* Save r28-r29 - Call-saved, "static" registers */

  push r28
  push r29

  /* Save r30-r31 - Call-used, "volatile" registers */

  push r30
  push r31

  /* Now save r26-r27 */

  push r26
  push r27

  /* Finally, save the stack pointer.  BUT we want the value of the stack
   * pointer as it was just BEFORE the exception.  We'll have to add to get
   * that value.  The value to add is the size of the register save area
   * including the bytes pushed by the interrupt handler (2), by the HANDLER
   * macro (1), and the 32 registers pushed above.  That is, the entire size
   * of the register save structure MINUS two bytes for the stack pointer
   * which has not yet been saved.
   */

  in r26, _SFR_IO_ADDR(SPL)
  in r27, _SFR_IO_ADDR(SPH)
  adiw r26, XCPTCONTEXT_REGS-2

  push r26 /* SPL then SPH */
  push r27
  .endm

/****************************************************************************
 * Name: EXCPT_EPILOGUE
 *
 * Description:
 *   Provides the "epilogue" logic that should appear at the end of every
 *   exception handler.
 *
 * On input:
 *  sp points to the address of the register save area (just as left by
 *   EXCPT_PROLOGUE).
 *  All registers are available for use.
 *  Interrupts are disabled.
 *
 * On completion:
 *  All registers restored except the PC which remains on the stack so
 *  that a return via reti can be performed.
 *
 ****************************************************************************/

  .macro EXCPT_EPILOGUE, regs

  /* We don't need to restore the stack pointer */

  pop r27 /* Discard SPH */
  pop r26 /* Discard SPL */

  /* Restore r26-r27 */

  pop r27
  pop r26

  /* Restore r30-r31 - Call-used, "volatile" registers */

  pop r31
  pop r30

  /* Restore r28-r29 - Call-saved, "static" registers */

  pop r29
  pop r28

  /* Restore r18-r27 - Call-used, "volatile" registers (r26-r27 already
   * restored, r24 and r25 will be restored later)
   */

  pop r23
  pop r22
  pop r21
  pop r20
  pop r19
  pop r18

  /* Restore r2-r17 - Call-saved, "static" registers */

  pop r17
  pop r16
  pop r15
  pop r14
  pop r13
  pop r12
  pop r11
  pop r10
  pop r9
  pop r8
  pop r7
  pop r6
  pop r5
  pop r4
  pop r3
  pop r2

  /* Restore r0 - the scratch register and  r1- the "zero" register
   * (that may not be zero)
   */

  pop r1
  pop r0

  /* Restore RAMPZ if the MCU has it */

#ifdef AVR_HAS_RAMPZ
  pop r24
  out _SFR_IO_ADDR(RAMPZ), r24
#endif

  /* Restore the status register (probably enabling interrupts) */

  pop r24                  /* Restore the status register */
  andi r24, ~(1 << SREG_I) /* but keeping interrupts disabled until the reti */
  out _SFR_IO_ADDR(SREG), r24

  /* Finally, restore r24-r25 - the temporary and IRQ number registers */

  pop r25
  pop r24
  .endm

/****************************************************************************
 * Name: USER_SAVE
 *
 * Description:
 *  Similar to EXPCT_PROLOGUE except that (1) this saves values into a
 *  register save data structure instead of on the stack, (2) the pointer
 *  is in r26;r27, and (3) Call-used registers are not saved.
 *
 * On Entry:
 *  X [r26:r27] - Points to the register save structure.
 *  Return address is already on the stack
 *  (due to CALL or RCALL instruction).
 * Interrupts are disabled.
 *
 * At completion:
 *   Register state is saved on the stack; All registers are available for
 *   usage except sp.
 *
 ****************************************************************************/

  .macro USER_SAVE

  /* Pop the return address from the stack (PC0 then PC1).
   *  R18:19 are Call-used
   *
   * NOTE: up_saveusercontext in avr_saveusercontext.S uses this macro
   * and needs to reverse this by pushing the return address back
   * to the stack. Contents of these two/three registers must not change
   * throughout whole macro and all changes in registers used and/or
   * instruction ordering need to be reflected in up_saveusercontext.
   */

#if AVR_PC_SIZE > 16
  pop r20
#endif /* AVR_PC_SIZE */
  pop  r19
  pop  r18

  /* Save the current stack pointer as it would be after the return
   * (SPH then SPL).
   */

  in r25, _SFR_IO_ADDR(SPH)
  st x+, r25
  in r24, _SFR_IO_ADDR(SPL)
  st x+, r24

  /* Skip over r26-r27 and r30-r31 - Call-used, "volatile" registers */

  adiw r26, 4 /* Four registers: r26-r27 and r30-r31 */

  /* Save r28-r29 - Call-saved, "static" registers */

  st x+, r29
  st x+, r28

  /* Skip over r18-r27 - Call-used, "volatile" registers (r26-r27 have
   * already been skipped, r24 and r25 are saved elsewhere)
   */

  adiw r26, 6 /* Seven registers: r18-23 */

  /* Save r2-r17 - Call-saved, "static" registers */

  st x+, r17
  st x+, r16
  st x+, r15
  st x+, r14
  st x+, r13
  st x+, r12
  st x+, r11
  st x+, r10
  st x+, r9
  st x+, r8
  st x+, r7
  st x+, r6
  st x+, r5
  st x+, r4
  st x+, r3
  st x+, r2

  /* Set r1  to zero - Function calls must return with r1=0 */

  clr r1
  st x+, r1

  /* Skip over r0 -- the scratch register */

  adiw r26, 1

#ifdef AVR_HAS_RAMPZ
  /* Save RAMPZ if the MCU has it */

  in r0, _SFR_IO_ADDR(RAMPZ)
  st X+, r0
#endif

  /* Save the status register
   * (probably not necessary since interrupts are disabled)
   */

  in r0, _SFR_IO_ADDR(SREG)
  st x+, r0

  /* Skip r24-r25 - These are scratch register and Call-used,
   * "volatile" registers
   */

  adiw r26, 2 /* Two registers: r24-r25 */

  /* Save the return address that we have saved in r18:19 */

  /* Note - this seems backwards for AVR DA/DB family which
   * states in the docs, chapter 6.4.4: The return address
   * consists of two bytes and the Least Significant Byte (LSB)
   * is pushed on the stack first (at the higher address).
   *
   * First value is popped into r19 and holds the MSB, store
   * ordering is reversed here.
   *
   * Considering this value is just stored and then restored
   * with the same discrepancy, it cancels out. No need to branch
   * the code with ifdefs.
   */

#if AVR_PC_SIZE > 16
  st x+, r20
#endif /* AVR_PC_SIZE */
  st x+, r19
  st x+, r18
  .endm

/****************************************************************************
 * Name: TCB_RESTORE
 *
 * Description:
 *  Functionally equivalent to EXCPT_EPILOGUE excetp that register save area
 *  is not on the stack but is held in a data structure.
 *
 * On input:
 *  X [r26:r27] points to the data structure.
 *  All registers are available for use.
 *  Interrupts are disabled.
 *
 * On completion:
 *  All registers restored except for the PC with now resides at the top of
 *  the new stack so that ret can be used to switch to the new context. (ret,
 *  not reti, because ret will preserve the restored interrupt state).
 *
 ****************************************************************************/

  .macro TCB_RESTORE, regs

  /* X [r26:27] points to the register save block.
   *    Get an offset pointer to the PC in
   * Y [r28:29]
   */

  movw r28, r26 /* Get a pointer to the PC0/PC1 storage location */
#if AVR_PC_SIZE <= 16
  adiw r28, REG_PC0
#else
  adiw r28, (REG_PC2+1) /* Will pre-decrement this on use */
#endif

  /* Fetch and set the new stack pointer */

  ld r25, x+                 /* Fetch stack pointer (post-incrementing) */
  out _SFR_IO_ADDR(SPH), r25 /* (SPH then SPL) */
  ld r24, x+
  out _SFR_IO_ADDR(SPL), r24

  /* Fetch the return address and save it at the bottom of the new stack so
   * that we can iret to switch contexts.  The new stack is now:
   *
   *  PC2 (for 24-bit PC arch)
   *  PC1
   *  PC0
   *  --- <- SP
   */

#if AVR_PC_SIZE <= 16
  ld r25, y+ /* Load PC0 (r25) then PC1 (r24) */
  ld r24, y+
  push r24 /* Push PC0 and PC1 on the stack (PC1 then PC0) */
  push r25
#else
  ld r25, -y /* Load PC2 (r25) */
  push r25
  ld r25, -y /* Load PC1 (r25) */
  push r25
  ld r25, -y /* Load PC0 (r25) */
  push r25
#endif

  /* Then get value of X [r26:r27].  Save X on the new stack where we can
   * recover it later.  The new stack is now:
   *
   *  PC2 (for 24-bit PC arch)
   *  PC1
   *  PC0
   *  R26
   *  R27
   *  --- <- SP
   */

  ld r25, x+ /* Fetch r26-r27 and save to the new stack */
  ld r24, x+
  push r24   /* r26 then r27 */
  push r25

  /* Restore r30-r31 - Call-used, "volatile" registers */

  ld r31, x+
  ld r30, x+

  /* Restore r28-r29 - Call-saved, "static" registers */

  ld r29, x+
  ld r28, x+

  /* Restore r18-r27 - Call-used, "volatile" registers (r26-r27 have been
   * moved and r24-r25 will be restore later)
   */

  ld r23, x+
  ld r22, x+
  ld r21, x+
  ld r20, x+
  ld r19, x+
  ld r18, x+

  /* Restore r2-r17 - Call-saved, "static" registers */

  ld r17, x+
  ld r16, x+
  ld r15, x+
  ld r14, x+
  ld r13, x+
  ld r12, x+
  ld r11, x+
  ld r10, x+
  ld r9, x+
  ld r8, x+
  ld r7, x+
  ld r6, x+
  ld r5, x+
  ld r4, x+
  ld r3, x+
  ld r2, x+

  /* Restore r1 - zero register (which may not be zero) */

  ld r1, x+

  /* Restore r0 - the scratch register */

  ld r0, x+

#ifdef AVR_HAS_RAMPZ
  /* Restore RAMPZ if the MCU has it */

  ld r24, X+
  out _SFR_IO_ADDR(RAMPZ), r24
#endif

  /* The following control flow split is required to eliminate non-atomic
   * interrupt_enable - return sequence.
   *
   * NOTE: since actual returning is handled by this macro it has been
   * removed from avr_fullcontextrestore function (avr_doswitch.S)
   */

  /* If interrupts shall be enabled go to 'restore remaining and reti' code
   * otherwise just do 'restore remaining and ret'
   *
   * See Documentation/platforms/avr/context-switch-notes.rst
   * for information about context creation and how it interacts
   * with ways the context is restored
   */

  ld r24, x+
  bst r24, SREG_I
  brts go_reti

#ifdef CONFIG_ARCH_CHIP_AVRDX
  /* Older chips do not care for internal MCU state here but for AVR Dx
   * family, we need to do different things based on if we came here
   * from interrupt handler or not.
   *
   * We do all branching now because at this point we are about to restore
   * SREG, don't have free registers to preserve it anymore and branching
   * would clobber it
   *
   * Branch away if we came here from interrupt handler
   */

  lds r25, CPUINT_STATUS
  and r25, r25
  brne .tcbr_avrdx_ret_reti
#endif

  /* Restore the status register, interrupts are disabled */

  out _SFR_IO_ADDR(SREG), r24

  /* Restore r24-r25 - The temporary and IRQ number registers */

  ld r25, x+
  ld r24, x+

  /* Finally, recover X [r26-r27] from the new stack.  The PC remains on the
   * new stack so that the user of this macro can return with ret (not reti,
   * ret will preserve the restored interrupt state).
   */

  pop r27 /* R27 then R26 */
  pop r26
  ret

#ifdef CONFIG_ARCH_CHIP_AVRDX
.tcbr_avrdx_ret_reti:

  /* See branch above for more detailed comments */

  out _SFR_IO_ADDR(SREG), r24 /* SREG with I-flag cleared */

  /* Restore r24-r25 - The temporary and IRQ number registers */

  ld r25, x+  /* restore r24-r25 */
  ld r24, x+
  pop r27     /* recovering R27 then R26  */
  pop r26

  /* Returning with I-flag cleared (reti does not change that),
   * but clearing internal "running in interrupt context" state
   */

  reti
#endif

go_reti:
#ifdef CONFIG_ARCH_CHIP_AVRDX
  /* In this case, branch away if we did NOT come here
   * from an interrupt handler
   */

  lds r25, CPUINT_STATUS
  and r25, r25
  breq .tcbr_avrdx_reti_ret
#endif

#ifdef CONFIG_ARCH_CHIP_AVRDX
  /* restore Status Register with interrupt enabled. Hardware
   * knows it is processing an interrupt and will not interrupt
   * us even with "I"-flag set (With the exception of high
   * priority interrupts but those are not supported.)
   */
#else

  /* restore the Status Register with interrupts disabled
   * and exit with reti (that will set the Interrupt Enable)
   */

  andi r24, ~(1 << SREG_I)
#endif

  out _SFR_IO_ADDR(SREG), r24

  ld r25, x+
  ld r24, x+

  pop r27
  pop r26

  reti

#ifdef CONFIG_ARCH_CHIP_AVRDX
.tcbr_avrdx_reti_ret:

  /* restore the Status Register with interrupts disabled,
   * we don't want to be interrupted until done
   */

  andi r24, ~(1 << SREG_I)
  out _SFR_IO_ADDR(SREG), r24

  ld r25, x+
  ld r24, x+

  pop r27
  pop r26

  /* Now enable interrupts and exit with ret, not messing with internal
   * state of the interrupt controller because from its point of view,
   * we are not returning from interrupt (despite the fact that the restored
   * context was created in an interrupt)
   *
   * AVR Instruction Set Manual DS40002198 states: "The instruction
   * following SEI will be executed before any pending interrupts."
   */

  sei
  ret
#endif

  .endm

/****************************************************************************
 * Name: USE_INTSTACK
 *
 * Description:
 *  Switch to the interrupt stack (if enabled in the configuration) and if
 *  the nesting level is equal to 0.
 *  Increment the nesting level in any event.
 *
 * On Entry:
 *  sp - Current value of the user stack pointer
 *  tmp1, tmp2, and tmp3 are registers that can be used temporarily.
 *  All interrupts should still be disabled.
 *
 * At completion:
 *  If the nesting level is 0, then (1) the user stack pointer is saved at
 *  the base of the interrupt stack and sp points to the interrupt stack.
 *  The values of tmp1, tmp2, tmp3, and sp have been altered
 *
 ****************************************************************************/

  .macro USE_INTSTACK, tmp1, tmp2, tmp3
#if CONFIG_ARCH_INTERRUPTSTACK > 0
# warning "Not implemented"
#endif
  .endm

/****************************************************************************
 * Name: RESTORE_STACK
 *
 * Description:
 * Restore the user stack.  Not really.. actually only decrements the
 * nesting level.
 * We always get the new stack pointer for the register save array.
 *
 * On Entry:
 *  tmp1 and tmp2 are registers that can be used temporarily.
 *  All interrupts must be disabled.
 *
 * At completion:
 *  Current nesting level is decremented
 *  The values of tmp1 and  tmp2 have been altered
 *
 ****************************************************************************/

  .macro RESTORE_STACK, tmp1, tmp2
#if CONFIG_ARCH_INTERRUPTSTACK > 0
#  warning "Not implemented"
#endif
  .endm

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_AVR_EXCPTMACROS_H */
