/****************************************************************************
 * arch/xtensa/src/common/xtensa_inthandlers.S
 *
 * Adapted from use in NuttX by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from logic originally provided by Cadence Design Systems Inc.
 *
 *   Copyright (c) 2006-2015 Cadence Design Systems Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ****************************************************************************/

	.file	"xtensa_inthandlers.S"

/* NOTES on the use of 'call0' for long jumps instead of 'j':
 *
 *  1. This file should be assembled with the -mlongcalls option to xt-xcc.
 *
 *  2. The -mlongcalls compiler option causes 'call0 dest' to be expanded to
 *     a sequence 'l32r a0, dest' 'callx0 a0' which works regardless of the
 *     distance from the call to the destination. The linker then relaxes
 *     it back to 'call0 dest' if it determines that dest is within range.
 *     This allows more flexibility in locating code without the performance
 *     overhead of the 'l32r' literal data load in cases where the destination
 *     is in range of 'call0'. There is an additional benefit in that 'call0'
 *     has a longer range than 'j' due to the target being word-aligned, so
 *     the 'l32r' sequence is less likely needed.
 *
 *  3. The use of 'call0' with -mlongcalls requires that register a0 not be
 *     live at the time of the call, which is always the case for a function
 *     call but needs to be ensured if 'call0' is used as a jump in lieu of 'j'.
 *
 *  4. This use of 'call0' is independent of the C function call ABI.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_specregs.h>

#include "xtensa_macros.h"
#include "xtensa_timer.h"

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

/****************************************************************************
 * Macro extract_msb - return the input with only the highest bit set.
 *
 * Entry Conditions/Side Effects:
 *   Input  : "ain"  - Input value, clobbered.
 *   Output : "aout" - Output value, has only one bit set, MSB of "ain".
 *
 * The two arguments must be different AR registers.
 *
 ****************************************************************************/

	.macro	extract_msb	aout ain
1:
	addi	\aout, \ain, -1				/* aout = ain - 1 */
	and		\ain, \ain, \aout			/* ain  = ain & aout */
	bnez	\ain, 1b					/* Repeat until ain == 0 */
	addi	\aout, \aout, 1				/* Return aout + 1 */
	.endm

/****************************************************************************
 * Macro dispatch_c_isr level mask
 *
 * Description:
 *
 * This will dispatch to user handlers (if any) that are registered in the
 * XTOS dispatch table (_xtos_interrupt_table). These handlers would have
 * been registered by calling _xtos_set_interrupt_handler(). There is one
 * exception - the timer interrupt used by the OS will not be dispatched
 * to a user handler - this must be handled by the caller of this macro.
 *
 * Level triggered and software interrupts are automatically deasserted by
 * this code.
 *
 * Assumptions:
 *   - PS.INTLEVEL is set to "level" at entry
 *   - PS.EXCM = 0, C calling enabled
 *
 * Entry Conditions/Side Effects:
 *   This macro will use registers a0 and a2-a6. The arguments are:
 *     level - interrupt level
 *     mask  - interrupt bitmask for this level
 *
 ****************************************************************************/

	.macro	dispatch_c_isr	level mask

	/* Get mask of pending, enabled interrupts at this level into a2. */

.L_xt_user_int_&level&:
	rsr		a2, INTENABLE
	rsr		a3, INTERRUPT
	movi	a4, \mask
	and		a2, a2, a3
	and		a2, a2, a4
	beqz	a2, 5f						/* Nothing to do */

	/* This bit of code provides a nice debug backtrace in the debugger.
	 * It does take a few more instructions, so undef XT_DEBUG_BACKTRACE
	 * if you want to save the cycles.
	* /

#if XT_DEBUG_BACKTRACE
#ifndef CONFIG_XTENSA_CALL0_ABI
	rsr		a0, EPC_1 + \level - 1		/* Return address */
	movi	a4, 0xC0000000				/* Constant with top 2 bits set (call size) */
	or		a0, a0, a4					/* Set top 2 bits */
	addx2	a0, a4, a0					/* Clear top bit -- simulating call4 size	*/
#endif
#endif

#ifdef XT_INTEXC_HOOKS
	/* Call interrupt hook if present to (pre)handle interrupts. */

	movi	a4, _xt_intexc_hooks
	l32i	a4, a4, \level << 2
	beqz	a4, 2f

#ifdef CONFIG_XTENSA_CALL0_ABI
	callx0	a4
	beqz	a2, 5f
#else
	mov		a6, a2
	callx4	a4
	beqz	a6, 5f
	mov		a2, a6
#endif
2:
#endif

	/* If multiple bits are set then MSB has highest priority. */

	extract_msb	a4, a2					/* a4 = MSB of a2, a2 trashed */

	movi	a3, XT_TIMER_INTEN			/* a3 = timer interrupt bit */
	wsr		a4, INTCLEAR				/* Clear sw or edge-triggered interrupt */
	beq		a3, a4, 4f					/* If timer interrupt then skip table */

	/* Call xtensa_int_decode with, passing that address of the register save
	 * area as a parameter (A2).
	 */

	mov		a2, sp						/* Argument: Top of stack = register save area */
	call0	xtensa_int_decode			/* Call xtensa_int_decode */

	/* On return from xtensa_int_decode, A2 will contain the address of the new
	 * register save area.  Usually this would be the same as the current SP.
	 * But in the event of a context switch, A2 will instead refer to the TCB
	 * register save area.
	 */

	beq		a2, sp, 3f					/* If timer interrupt then skip table */

	/* Switch stacks */
#warning REVIST: Should use register save are on stack.

	l32i	a4, sp, (4 * REG_A1)		/* Retrieve stack ptr and replace */
	addi	sp,  a4, -(4 * XCPTCONTEXT_SIZE)

3:
	j		.L_xt_user_int_&level&		/* Check for more interrupts */

4:

	.ifeq XT_TIMER_INTPRI - \level

	/* Interrupt handler for the RTOS tick timer if at this level.
	 * We'll be reading the interrupt state again after this call
	 * so no need to preserve any registers except a6 (vpri_mask).
	 */

	mov		a12, a6						/* Preserve a6 */
	movi	a2, XTENSA_IRQ_TIMER&level& /* Arg 1: IRQ number */
	mov		a3, sp						/* Arg 2: Top of stack = register save area */
	call0	xtensa_irq_dispatch			/* Call xtensa_int_decode */
	mov		a6, a12						/* Preserve a6 */
	.endif

	j		.L_xt_user_int_&level&		/* Check for more interrupts */

5:
	/* done */

	.endm

/****************************************************************************
 * Macro: ps_setup
 *
 * Description:
 *   Set up PS for C, enable interrupts above this level and clear EXCM.
 *
 * Entry Conditions:
 *   level - interrupt level
 *   tmp   - scratch register
 *
 * Side Effects:
 *   PS and scratch register modified
 *
 * Assumptions:
 *   - PS.EXCM = 1, C calling disabled
 *
 ****************************************************************************/

	.macro	ps_setup	level tmp

#if 0 /* Nested interrupts no yet supported */
#  ifdef CONFIG_XTENSA_CALL0_ABI
	/* Disable interrupts at level and below */

	movi	\tmp, PS_INTLEVEL(\level) | PS_UM
#  else
	movi	\tmp, PS_INTLEVEL(\level) | PS_UM | PS_WOE
#  endif
#else
#  ifdef CONFIG_XTENSA_CALL0_ABI
	/* Disable all low- and medium-priority interrupts.  Nested are not yet
	 * supported.
	 */

	movi	\tmp, PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM
#  else
	movi	\tmp, PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM | PS_WOE
#  endif
#endif

	wsr		\tmp, PS
	rsync

	.endm

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * LOW PRIORITY (LEVEL 1) LOW LEVEL HANDLER.
 ****************************************************************************/

#warning REVISIT level 1 interrupt handlers

/****************************************************************************
 * MEDIUM PRIORITY (LEVEL 2+) INTERRUPT LOW LEVEL HANDLERS.
 *
 * C Prototype:
 *   void _xtensa_levelN_handler(void)
 *
 * Description:
 *   Medium priority interrupts are by definition those with priority greater
 *   than 1 and not greater than XCHAL_EXCM_LEVEL. These are disabled
 *   by setting PS.EXCM and therefore can easily support a C environment for
 *   handlers in C, and interact safely with NuttX.
 *
 *   Each vector goes at a predetermined location according to the Xtensa
 *   hardware configuration, which is ensured by its placement in a special
 *   section known to the NuttX linker script.  The vector logic performs
 *   the minimum necessary operations before jumping to the handler via
 *   a CALL0 instruction.  See "NOTES on the use of call0 ..." above.
 *
 *   The corresponding handler sets up the appropriate stack frame, saves
 *   a few vector-specific registers and calls _xtensa_context_save()
 *   to save the rest of the interrupted context.  It then calls the C
 *   logic to decode the specific interrupt source and dispatch to the
 *   appropriate C interrupt handler.
 *
 ****************************************************************************/

#if XCHAL_EXCM_LEVEL >= 2
	.section .iram1,"ax"
	.type	_xtensa_level2_handler,@function
	.align	4

_xtensa_level2_handler:

	mov		a0, sp							/* sp == a1 */
	addi	sp, sp, -(4 * XCPTCONTEXT_SIZE)	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_2						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_2						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_2					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)

	/* Save rest of interrupt context. */

	s32i	a2, sp, (4 * REG_A2)
	movi	a2, sp							/* Address of state save on stack */
	call0	_xtensa_context_save			/* Save full register state */

	/* Set up PS for C, enable interrupts above this level and clear EXCM. */

	ps_setup	2 a0

	/* Decode and dispatch the interrupt.  In the event of an interrupt
	 * level context dispatch_c_isr() will switch stacks to the new task's
	 * context save area.
	 */

	dispatch_c_isr	2 XCHAL_INTLEVEL2_MASK

	/* Restore registers in preparation to return from interrupt */

	call0	_xtensa_context_restore

	/* Restore only level-specific regs (the rest were already restored) */

	l32i	a0, sp, (4 * REG_PS)			/* Retrieve interruptee's PS */
	wsr		a0, EPS_2
	l32i	a0, sp, (4 * REG_PC)			/* Retrieve interruptee's PC */
	wsr		a0, EPC_2
	l32i	a0, sp, (4 * REG_A0)			/* Retrieve interruptee's A0 */
	l32i	a2, sp, (4 * REG_A2)			/* Retrieve interruptee's A2 */
	l32i	sp, sp, (4 * REG_A1)			/* Remove interrupt stack frame */
	rsync									/* Ensure EPS and EPC written */

	/* Return from interrupt.  RFI  restores the PS from EPS_2 and jumps to
	 * the address in EPC_2.
	 */

	rfi		2

#endif /* XCHAL_EXCM_LEVEL >= 2 */

#if XCHAL_EXCM_LEVEL >= 3
	.section .iram1,"ax"
	.type	_xtensa_level3_handler,@function
	.align	4

_xtensa_level3_handler:

	mov		a0, sp							/* sp == a1 */
	addi	sp, sp, -(4 * XCPTCONTEXT_SIZE)	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_3						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_3						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_3					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)

	/* Save rest of interrupt context. */

	s32i	a2, sp, (4 * REG_A2)
	movi	a2, sp							/* Address of state save on stack */
	call0	_xtensa_context_save			/* Save full register state */

	/* Set up PS for C, enable interrupts above this level and clear EXCM. */

	ps_setup	3 a0

	/* Decode and dispatch the interrupt.  In the event of an interrupt
	 * level context dispatch_c_isr() will switch stacks to the new task's
	 * context save area.
	 */

	dispatch_c_isr	3 XCHAL_INTLEVEL3_MASK

	/* Restore registers in preparation to return from interrupt */

	call0	_xtensa_context_restore

	/* Restore only level-specific regs (the rest were already restored) */

	l32i	a0, sp, (4 * REG_PS)			/* Retrieve interruptee's PS */
	wsr		a0, EPS_3
	l32i	a0, sp, (4 * REG_PC)			/* Retrieve interruptee's PC */
	wsr		a0, EPC_3
	l32i	a0, sp, (4 * REG_A0)			/* Retrieve interruptee's A0 */
	l32i	a2, sp, (4 * REG_A2)			/* Retrieve interruptee's A2 */
	l32i	sp, sp, (4 * REG_A1)			/* Remove interrupt stack frame */
	rsync									/* Ensure EPS and EPC written */

	/* Return from interrupt.  RFI  restores the PS from EPS_3 and jumps to
	 * the address in EPC_3.
	 */

	rfi		3

#endif /* XCHAL_EXCM_LEVEL >= 3 */

#if XCHAL_EXCM_LEVEL >= 4
	.section .iram1,"ax"
	.type	_xtensa_level4_handler,@function
	.align	4

_xtensa_level4_handler:

	mov		a0, sp							/* sp == a1 */
	addi	sp, sp, -(4 * XCPTCONTEXT_SIZE)	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_4						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_4						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_4					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)

	/* Save rest of interrupt context. */

	s32i	a2, sp, (4 * REG_A2)
	movi	a2, sp							/* Address of state save on stack */
	call0	_xtensa_context_save			/* Save full register state */

	/* Set up PS for C, enable interrupts above this level and clear EXCM. */

	ps_setup	4 a0

	/* Decode and dispatch the interrupt.  In the event of an interrupt
	 * level context dispatch_c_isr() will switch stacks to the new task's
	 * context save area.
	 */

	dispatch_c_isr	4 XCHAL_INTLEVEL4_MASK

	/* Restore registers in preparation to return from interrupt */

	call0	_xtensa_context_restore

	/* Restore only level-specific regs (the rest were already restored) */

	l32i	a0, sp, (4 * REG_PS)			/* Retrieve interruptee's PS */
	wsr		a0, EPS_4
	l32i	a0, sp, (4 * REG_PC)			/* Retrieve interruptee's PC */
	wsr		a0, EPC_4
	l32i	a0, sp, (4 * REG_A0)			/* Retrieve interruptee's A0 */
	l32i	a2, sp, (4 * REG_A2)			/* Retrieve interruptee's A2 */
	l32i	sp, sp, (4 * REG_A1)			/* Remove interrupt stack frame */
	rsync									/* Ensure EPS and EPC written */

	/* Return from interrupt.  RFI  restores the PS from EPS_4 and jumps to
	 * the address in EPC_4.
	 */

	rfi		4

#endif /* XCHAL_EXCM_LEVEL >= 4 */

#if XCHAL_EXCM_LEVEL >= 5
	.section .iram1,"ax"
	.type	_xtensa_level5_handler,@function
	.align	4

_xtensa_level5_handler:

	mov		a0, sp							/* sp == a1 */
	addi	sp, sp, -(4 * XCPTCONTEXT_SIZE)	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_5						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_5						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_5					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)

	/* Save rest of interrupt context. */

	s32i	a2, sp, (4 * REG_A2)
	movi	a2, sp							/* Address of state save on stack */
	call0	_xtensa_context_save			/* Save full register state */

	/* Set up PS for C, enable interrupts above this level and clear EXCM. */

	ps_setup	5 a0

	/* Decode and dispatch the interrupt.  In the event of an interrupt
	 * level context dispatch_c_isr() will switch stacks to the new task's
	 * context save area.
	 */

	dispatch_c_isr	5 XCHAL_INTLEVEL5_MASK

	/* Restore registers in preparation to return from interrupt */

	call0	_xtensa_context_restore

	/* Restore only level-specific regs (the rest were already restored) */

	l32i	a0, sp, (4 * REG_PS)			/* Retrieve interruptee's PS */
	wsr		a0, EPS_5
	l32i	a0, sp, (4 * REG_PC)			/* Retrieve interruptee's PC */
	wsr		a0, EPC_5
	l32i	a0, sp, (4 * REG_A0)			/* Retrieve interruptee's A0 */
	l32i	a2, sp, (4 * REG_A2)			/* Retrieve interruptee's A2 */
	l32i	sp, sp, (4 * REG_A1)			/* Remove interrupt stack frame */
	rsync									/* Ensure EPS and EPC written */

	/* Return from interrupt.  RFI  restores the PS from EPS_5 and jumps to
	 * the address in EPC_5.
	 */

	rfi		5

#endif /* XCHAL_EXCM_LEVEL >= 2 */

#if XCHAL_EXCM_LEVEL >= 6
	.section .iram1,"ax"
	.type	_xtensa_level6_handler,@function
	.align	4

_xtensa_level6_handler:

	mov		a0, sp							/* sp == a1 */
	addi	sp, sp, -(4 * XCPTCONTEXT_SIZE)	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_6						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_6						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_6					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)

	/* Save rest of interrupt context. */

	s32i	a2, sp, (4 * REG_A2)
	movi	a2, sp							/* Address of state save on stack */
	call0	_xtensa_context_save			/* Save full register state */

	/* Set up PS for C, enable interrupts above this level and clear EXCM. */

	ps_setup	6 a0

	/* Decode and dispatch the interrupt.  In the event of an interrupt
	 * level context dispatch_c_isr() will switch stacks to the new task's
	 * context save area.
	 */

	dispatch_c_isr	6 XCHAL_INTLEVEL6_MASK

	/* Restore registers in preparation to return from interrupt */

	call0	_xtensa_context_restore

	/* Restore only level-specific regs (the rest were already restored) */

	l32i	a0, sp, (4 * REG_PS)			/* Retrieve interruptee's PS */
	wsr		a0, EPS_6
	l32i	a0, sp, (4 * REG_PC)			/* Retrieve interruptee's PC */
	wsr		a0, EPC_6
	l32i	a0, sp, (4 * REG_A0)			/* Retrieve interruptee's A0 */
	l32i	a2, sp, (4 * REG_A2)			/* Retrieve interruptee's A2 */
	l32i	sp, sp, (4 * REG_A1)			/* Remove interrupt stack frame */
	rsync									/* Ensure EPS and EPC written */

	/* Return from interrupt.  RFI  restores the PS from EPS_6 and jumps to
	 * the address in EPC_6.
	 */

	rfi		6

#endif /* XCHAL_EXCM_LEVEL >= 6 */

/****************************************************************************
 * HIGH PRIORITY (LEVEL > XCHAL_EXCM_LEVEL) LOW-LEVEL HANDLERS
 *
 * High priority interrupts are by definition those with priorities greater
 * than XCHAL_EXCM_LEVEL. This includes non-maskable (NMI). High priority
 * interrupts cannot interact with the RTOS, that is they must save all regs
 * they use and not call any RTOS function.
 *
 * A further restriction imposed by the Xtensa windowed architecture is that
 * high priority interrupts must not modify the stack area even logically
 * "above" the top of the interrupted stack (they need to provide their
 * own stack or static save area).
 *
 * Cadence Design Systems recommends high priority interrupt handlers be
 * coded in assembly and used for purposes requiring very short service
 * times.
 *
 * Here are templates for high priority (level 2+) interrupt vectors.
 * They assume only one interrupt per level to avoid the burden of
 * identifying which interrupts at this level are pending and enabled. This
 * allows for  minimum latency and avoids having to save/restore a2 in
 * addition to a0.  If more than one interrupt per high priority level is
 * configured, this burden is on the handler which in any case must provide
 * a way to save and restore registers it uses without touching the
 * interrupted stack.
 *
 * Each vector goes at a predetermined location according to the Xtensa
 * hardware configuration, which is ensured by its placement in a special
 * section known to the Xtensa linker support package (LSP). It performs
 * the minimum necessary before jumping to the handler in the .text section.
 *
 ****************************************************************************/

/*
Currently only shells for high priority interrupt handlers are provided
here. However a template and example can be found in the Cadence Design Systems tools
documentation: "Microprocessor Programmer's Guide".
*/

#if XCHAL_INT_NLEVELS >=2 && XCHAL_EXCM_LEVEL < 2 && XCHAL_DEBUGLEVEL !=2
	.section	.iram1,"ax"
	.type		_xtensa_level2_handler, @function
	.align		4

_xtensa_level2_handler:

#ifdef XT_INTEXC_HOOKS
	/* Call interrupt hook if present to (pre)handle interrupts. */

	movi	a0, _xt_intexc_hooks
	l32i	a0, a0, 2<<2
	beqz	a0, 1f

.Ln_xtensa_level2_handler_call_hook:

	callx0	a0							/* Must NOT disturb stack! */

1:
#endif

	/* USER_EDIT:
	 * ADD HIGH PRIORITY LEVEL 2 INTERRUPT HANDLER CODE HERE.
	 */

	.align	4

	rsr		a0, EXCSAVE_2				/* Restore a0 */
	rfi		2

#endif  /* XCHAL_INT_NLEVELS >=2 && XCHAL_EXCM_LEVEL < 2 && XCHAL_DEBUGLEVEL !=2 */

#if XCHAL_INT_NLEVELS >=3 && XCHAL_EXCM_LEVEL < 3 && XCHAL_DEBUGLEVEL !=3
	.section	.iram1,"ax"
	.type		_xtensa_level3_handler, @function
	.align		4

_xtensa_level3_handler:

#ifdef XT_INTEXC_HOOKS
	/* Call interrupt hook if present to (pre)handle interrupts. */

	movi	a0, _xt_intexc_hooks
	l32i	a0, a0, 3<<2
	beqz	a0, 1f

.Ln_xtensa_level3_handler_call_hook:

	callx0	a0							/* Must NOT disturb stack! */
1:
#endif

	/* USER_EDIT:
	 * ADD HIGH PRIORITY LEVEL 3 INTERRUPT HANDLER CODE HERE.
	 */

	.align	4

	rsr		a0, EXCSAVE_3				/* Restore a0 */
	rfi		3

#endif /* XCHAL_INT_NLEVELS >=3 && XCHAL_EXCM_LEVEL < 3 && XCHAL_DEBUGLEVEL !=3 */

#if XCHAL_INT_NLEVELS >=4 && XCHAL_EXCM_LEVEL < 4 && XCHAL_DEBUGLEVEL !=4
	.section	.iram1,"ax"
	.type		_xtensa_level4_handler,@function
	.align		4

_xtensa_level4_handler:

#ifdef XT_INTEXC_HOOKS
	/* Call interrupt hook if present to (pre)handle interrupts. */

	movi	a0, _xt_intexc_hooks
	l32i	a0, a0, 4 << 2
	beqz	a0, 1f

.Ln_xtensa_level4_handler_call_hook:

	callx0	a0							/* Must NOT disturb stack! */
1:
#endif

	/* USER_EDIT:
	 * ADD HIGH PRIORITY LEVEL 4 INTERRUPT HANDLER CODE HERE.
	 */

	.align	4

	rsr		a0, EXCSAVE_4				/* Restore a0 */
	rfi		4

#endif /* XCHAL_INT_NLEVELS >=4 && XCHAL_EXCM_LEVEL < 4 && XCHAL_DEBUGLEVEL !=4 */

#if XCHAL_INT_NLEVELS >=5 && XCHAL_EXCM_LEVEL < 5 && XCHAL_DEBUGLEVEL !=5
	.section	.iram1,"ax"
	.type		_xtensa_level5_handler,@function
	.align		4

_xtensa_level5_handler:

#ifdef XT_INTEXC_HOOKS
	/* Call interrupt hook if present to (pre)handle interrupts. */

	movi	a0, _xt_intexc_hooks
	l32i	a0, a0, 5 << 2
	beqz	a0, 1f

.Ln_xtensa_level5_handler_call_hook:

	callx0	a0							/* Must NOT disturb stack! */
1:
#endif

	/* USER_EDIT:
	 * ADD HIGH PRIORITY LEVEL 5 INTERRUPT HANDLER CODE HERE.
	 */

	rsr		a0, EXCSAVE_5				/* Restore a0 */
	rfi		5

#endif /* XCHAL_INT_NLEVELS >=5 && XCHAL_EXCM_LEVEL < 5 && XCHAL_DEBUGLEVEL !=5 */

#if XCHAL_INT_NLEVELS >=6 && XCHAL_EXCM_LEVEL < 6 && XCHAL_DEBUGLEVEL !=6
	.section	.iram1,"ax"
	.type		_xtensa_level6_handler, @function
	.align		4

_xtensa_level6_handler:

#ifdef XT_INTEXC_HOOKS
	/* Call interrupt hook if present to (pre)handle interrupts. */

	movi	a0, _xt_intexc_hooks
	l32i	a0, a0, 6 << 2
	beqz	a0, 1f

.Ln_xtensa_level6_handler_call_hook:

	callx0	a0							/* Must NOT disturb stack! */
1:
#endif

	/* USER_EDIT:
	 * ADD HIGH PRIORITY LEVEL 6 INTERRUPT HANDLER CODE HERE.
	 */

	rsr		a0, EXCSAVE_6				/* Restore a0 */
	rfi		6

#endif /* XCHAL_INT_NLEVELS >=6 && XCHAL_EXCM_LEVEL < 6 && XCHAL_DEBUGLEVEL !=6 */
