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

#include <arch/chip/core-isa.h>

#include "xtensa_specregs.h"
#include "xtensa_macros.h"
#include "xtensa_timer.h"

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

/* Macro extract_msb - return the input with only the highest bit set.
 *
 * Input  : "ain"  - Input value, clobbered.
 * Output : "aout" - Output value, has only one bit set, MSB of "ain".
 * The two arguments must be different AR registers.
 */

	.macro	extract_msb	aout ain
1:
	addi	\aout, \ain, -1				/* aout = ain - 1 */
	and		\ain, \ain, \aout			/* ain  = ain & aout */
	bnez	\ain, 1b					/* Repeat until ain == 0 */
	addi	\aout, \aout, 1				/* Return aout + 1 */
	.endm

/* Macro dispatch_c_isr - dispatch interrupts to user ISRs.
 * This will dispatch to user handlers (if any) that are registered in the
 * XTOS dispatch table (_xtos_interrupt_table). These handlers would have
 * been registered by calling _xtos_set_interrupt_handler(). There is one
 * exception - the timer interrupt used by the OS will not be dispatched
 * to a user handler - this must be handled by the caller of this macro.
 *
 * Level triggered and software interrupts are automatically deasserted by
 * this code.
 *
 * ASSUMPTIONS:
 *   - PS.INTLEVEL is set to "level" at entry
 *   - PS.EXCM = 0, C calling enabled
 *
 * NOTE: This macro will use registers a0 and a2-a6. The arguments are:
 *   level -- interrupt level
 *   mask  -- interrupt bitmask for this level
 */

	.macro	dispatch_c_isr	level mask

	/* Get mask of pending, enabled interrupts at this level into a2. */

.L_xt_user_int_&level&:
	rsr		a2, INTENABLE
	rsr		a3, INTERRUPT
	movi	a4, \mask
	and		a2, a2, a3
	and		a2, a2, a4
	beqz	a2, 9f						/* Nothing to do */

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
	beqz	a2, 9f
#else
	mov		a6, a2
	callx4	a4
	beqz	a6, 9f
	mov		a2, a6
#endif
2:
#endif

	/* If multiple bits are set then MSB has highest priority. */

	extract_msb	a4, a2					/* a4 = MSB of a2, a2 trashed */

#ifdef CONFIG_XTENSA_USE_SWPRI
	/* Enable all interrupts at this level that are numerically higher
	 * than the one we just selected, since they are treated as higher
	 * priority.
	 */

	movi	a3, \mask					/* a3 = all interrupts at this level */
	add		a2, a4, a4					/* a2 = a4 << 1 */
	addi	a2, a2, -1					/* a2 = mask of 1's <= a4 bit */
	and		a2, a2, a3					/* a2 = mask of all bits <= a4 at this level */
	movi	a3, _xt_intdata
	l32i	a6, a3, 4					/* a6 = _xt_vpri_mask */
	neg		a2, a2
	addi	a2, a2, -1					/* a2 = mask to apply */
	and		a5, a6, a2					/* mask off all bits <= a4 bit */
	s32i	a5, a3, 4					/* update _xt_vpri_mask */
	rsr		a3, INTENABLE
	and		a3, a3, a2					/* mask off all bits <= a4 bit */
	wsr		a3, INTENABLE
	rsil	a3, \level - 1				/* lower interrupt level by 1 */
#endif

	movi	a3, XT_TIMER_INTEN			/* a3 = timer interrupt bit */
	wsr		a4, INTCLEAR				/* Clear sw or edge-triggered interrupt */
	beq		a3, a4, 7f					/* If timer interrupt then skip table */

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
#ifdef CONFIG_XTENSA_USE_SWPRI
	j		8f
#else
	j		.L_xt_user_int_&level&		/* Check for more interrupts */
#endif

7:

	.ifeq XT_TIMER_INTPRI - \level
.L_xt_user_int_timer_&level&:
	/* Interrupt handler for the RTOS tick timer if at this level.
	We'll be reading the interrupt state again after this call
	so no need to preserve any registers except a6 (vpri_mask).
	*/

#ifdef CONFIG_XTENSA_CALL0_ABI
	mov		a12, a6
	call0	XT_RTOS_TIMER_INT
	mov	a2, a12
#else
	mov	a2, a6
	call4	XT_RTOS_TIMER_INT
#endif
	.endif

#ifdef CONFIG_XTENSA_USE_SWPRI
	j		8f
#else
	j		.L_xt_user_int_&level&		/* check for more interrupts */
#endif

#ifdef CONFIG_XTENSA_USE_SWPRI
8:
	/* Restore old value of _xt_vpri_mask from a2. Also update INTENABLE from
		virtual _xt_intenable which _could_ have changed during interrupt
		processing. */

	movi	a3, _xt_intdata
	l32i	a4, a3, 0					/* a4 = _xt_intenable */
	s32i	a2, a3, 4					/* update _xt_vpri_mask */
	and		a4, a4, a2					/* a4 = masked intenable */
	wsr		a4, INTENABLE				/* update INTENABLE */
#endif

9:
	/* done */

	.endm

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * MEDIUM PRIORITY (LEVEL 2+) INTERRUPT VECTORS AND LOW LEVEL HANDLERS.
 *
 * C Prototype:
 *   void _xtensa_levelN_handler(void)
 *
 * Description:
 *   Medium priority interrupts are by definition those with priority greater
 *   than 1 and not greater than XTENSA_EXCM_LEVEL. These are disabled
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
 *   a few vector-specific registers and calls xtensa_full_context_save()
 *   to save the rest of the interrupted context.  It then calls the C
 *   logic to decode the specific interrupt source and dispatch to the
 *   appropriate C interrupt handler.
 *
 ****************************************************************************/

#if XTENSA_EXCM_LEVEL >= 2
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

	/* Save rest of interrupt context and enter RTOS. */

	call0	xtensa_full_context_save		/* Save full register state */

	/* Set up PS for C, enable interrupts above this level and clear EXCM. */

#ifdef CONFIG_XTENSA_CALL0_ABI
	movi	a0, PS_INTLEVEL(2) | PS_UM
	#else
	movi	a0, PS_INTLEVEL(2) | PS_UM | PS_WOE
#endif
	wsr		a0, PS
	rsync

	/* Decode and dispatch the interrupt.  In the event of an interrupt
	 * level context dispatch_c_isr() will switch stacks to the new task's
	 * context save area.
	 */

	dispatch_c_isr	2 XTENSA_INTLEVEL2_MASK

	/* Restore registers in preparation to return from interrupt */

	call0	_xtensa_int_exit

	/* Restore only level-specific regs (the rest were already restored) */

	l32i	a0, sp, (4 * REG_PS)			/* retrieve interruptee's PS */
	wsr		a0, EPS_2
	l32i	a0, sp, (4 * REG_PC)			/* retrieve interruptee's PC */
	wsr		a0, EPC_2
	l32i	a0, sp, (4 * REG_A0)			/* retrieve interruptee's A0 */
	l32i	sp, sp, (4 * REG_A1)			/* remove interrupt stack frame */
	rsync									/* Ensure EPS and EPC written */

	/* Return from interrupt.  RFI  restores the PS from EPS_2 and jumps to
	 * the address in EPC_2.
	 */

	rfi		2

#endif /* XTENSA_EXCM_LEVEL >= 2 */

/****************************************************************************
 *
 * HIGH PRIORITY (LEVEL > XTENSA_EXCM_LEVEL) INTERRUPT VECTORS AND HANDLERS
 *
 * High priority interrupts are by definition those with priorities greater
 * than XTENSA_EXCM_LEVEL. This includes non-maskable (NMI). High priority
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

#if XTENSA_INT_NLEVELS >=2 && XTENSA_EXCM_LEVEL < 2 && XTENSA_DEBUGLEVEL !=2
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
.L_xtensa_level2_handler_exit:
	rsr		a0, EXCSAVE_2				/* Restore a0 */
	rfi		2

#endif  /* XTENSA_INT_NLEVELS >=2 && XTENSA_EXCM_LEVEL < 2 && XTENSA_DEBUGLEVEL !=2 */

#if XTENSA_INT_NLEVELS >=3 && XTENSA_EXCM_LEVEL < 3 && XTENSA_DEBUGLEVEL !=3
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
.L_xtensa_level3_handler_exit:
	rsr		a0, EXCSAVE_3				/* Restore a0 */
	rfi		3

#endif /* XTENSA_INT_NLEVELS >=3 && XTENSA_EXCM_LEVEL < 3 && XTENSA_DEBUGLEVEL !=3 */

#if XTENSA_INT_NLEVELS >=4 && XTENSA_EXCM_LEVEL < 4 && XTENSA_DEBUGLEVEL !=4
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
.L_xtensa_level4_handler_exit:
	rsr		a0, EXCSAVE_4				/* Restore a0 */
	rfi		4

#endif /* XTENSA_INT_NLEVELS >=4 && XTENSA_EXCM_LEVEL < 4 && XTENSA_DEBUGLEVEL !=4 */

#if XTENSA_INT_NLEVELS >=5 && XTENSA_EXCM_LEVEL < 5 && XTENSA_DEBUGLEVEL !=5
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

	.align	4
.L_xtensa_level5_handler_exit:
	rsr		a0, EXCSAVE_5				/* restore a0 */
	rfi		5

#endif /* XTENSA_INT_NLEVELS >=5 && XTENSA_EXCM_LEVEL < 5 && XTENSA_DEBUGLEVEL !=5 */

#if XTENSA_INT_NLEVELS >=6 && XTENSA_EXCM_LEVEL < 6 && XTENSA_DEBUGLEVEL !=6
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

	.align	4
.L_xtensa_level6_handler_exit:
	rsr		a0, EXCSAVE_6				/* Restore a0 */
	rfi		6

#endif /* XTENSA_INT_NLEVELS >=6 && XTENSA_EXCM_LEVEL < 6 && XTENSA_DEBUGLEVEL !=6 */
