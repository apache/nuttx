/****************************************************************************
 * arch/xtensa/src/common/xtensa_user_handler.S
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
 *
 ****************************************************************************/

	.file	"xtensa_user_handler.S"

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
#include <arch/xtensa/core.h>
#include <arch/xtensa/xtensa_specregs.h>

#include "xtensa_macros.S"

#include "chip.h"

/****************************************************************************
 * Waypoints
 ****************************************************************************/

/* Insert some waypoints for jumping beyond the signed 8-bit range of
 * conditional branch instructions, so the conditional branches to specific
 * exception handlers are not taken in the mainline. Saves some cycles in the
 * mainline.
 */

	.section HANDLER_SECTION, "ax"

	.align	4
_xtensa_to_level1_handler:
	call0	_xtensa_level1_handler			/* Jump to level1 interrupt handler */

#if XCHAL_HAVE_WINDOWED
	.align	4
_xtensa_to_alloca_handler:
	call0	_xtensa_alloca_handler			/* Jump to window vectors section */
#endif

	.align	4
_xtensa_to_syscall_handler:
	call0	_xtensa_syscall_handler			/* Jump to syscall exception handler */

#ifdef CONFIG_XTENSA_CP_LAZY
#if XCHAL_CP_NUM > 0
	.align	4
_xtensa_to_coproc_handler:
	call0	_xtensa_coproc_handler			/* Jump to copressor exception handler */
#endif
#endif /* CONFIG_XTENSA_CP_LAZY */

/****************************************************************************
 * Name: _xtensa_user_handler
 *
 * Description:
 *   User exception handler.
 *
 * Entry Conditions:
 *   A0 saved in EXCSAVE_1.  All other register as upon exception.
 *
 ****************************************************************************/

	.type	_xtensa_user_handler, @function
	.global	_xtensa_user_handler
	.align	4

_xtensa_user_handler:

	/* If level 1 interrupt then jump to the dispatcher */

	rsr		a0, EXCCAUSE
	beqi	a0, EXCCAUSE_LEVEL1INTERRUPT, _xtensa_to_level1_handler

#ifdef CONFIG_XTENSA_CP_LAZY
#if XCHAL_CP_NUM > 0
	/* Handle any coprocessor exceptions. Rely on the fact that exception
	 * numbers above EXCCAUSE_CP0_DISABLED all relate to the coprocessors.
	 */

	bgeui	a0, EXCCAUSE_CP0_DISABLED, _xtensa_to_coproc_handler
#endif
#endif /* CONFIG_XTENSA_CP_LAZY */

	/* Handle alloca and syscall exceptions */

#if XCHAL_HAVE_WINDOWED
	beqi	a0, EXCCAUSE_ALLOCA, _xtensa_to_alloca_handler
#endif
	beqi	a0, EXCCAUSE_SYSCALL, _xtensa_to_syscall_handler

	/* Handle all other exceptions. All can have user-defined handlers. */
	/* NOTE: we'll stay on the user stack for exception handling. */

	/* Allocate exception frame and save minimal context. */

	mov		a0, sp							/* Save SP in A0 */
	addi	sp, sp, -XCPTCONTEXT_SIZE	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, PS							/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_1						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_1					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)
	s32i	a2, sp, (4 * REG_A2)

	/* Save EXCCAUSE and EXCVADDR into the user frame */

	rsr		a0, EXCCAUSE
	s32i	a0, sp, (4 * REG_EXCCAUSE)
	rsr		a0, EXCVADDR
	s32i	a0, sp, (4 * REG_EXCVADDR)

	/* Save rest of interrupt context. */

	call0	_xtensa_context_save			/* Save full register state */

	/* Save current SP before (possibly) overwriting it,
	 * it's the register save area.
	 */

	mov	a12, sp

	/* Switch to an interrupt stack if we have one */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
	setintstack a13 a14
#endif

	/* Set up PS for C, re-enable hi-pri interrupts, and clear EXCM. */

	ps_setup	1 a0

	/* Create pseudo base save area. At this point, a12 points to the
	 * allocated and filled exception stack frame (old value of SP in case of
	 * an interrupt stack).
	 */

#ifdef CONFIG_XTENSA_INTBACKTRACE
	l32i    a3, a12, (4 * REG_A0)     /* Copy pre-exception a0 (return address) */
	s32e    a3, sp, -16
	l32i    a3, a12, (4 * REG_A1)     /* Copy pre-exception a1 (stack pointer) */
	s32e    a3, sp, -12

	rsr     a0, EPC_1                /* return address for debug backtrace */
	movi    a4, 0xc0000000           /* constant with top 2 bits set (call size) */
	or      a0, a0, a4               /* set top 2 bits */
	addx2   a0, a4, a0               /* clear top bit -- thus simulating call4 size */
#endif

	/* Call xtensa_user, passing both the EXCCAUSE and a pointer to the
	 * beginning of the register save area.
	 */

#ifdef __XTENSA_CALL0_ABI__
	rsr		a2, EXCCAUSE					/* Argument 1 (a2) = EXCCAUSE */
	mov		a3, a12							/* Argument 2 (a3) = pointer to register save area */
	calx0	xtensa_user						/* Call xtensa_user */
#else
	rsr		a6, EXCCAUSE					/* Argument 1 (a6) = EXCCAUSE */
	mov		a7, a12							/* Argument 2 (a7) = pointer to register save area */
	call4	xtensa_user						/* Call xtensa_user */
	mov		a12, a6
#endif

	/* Restore registers in preparation to return from interrupt */

	mov		a2, a12							/* a2 = address of new state save area */
	call0	_xtensa_context_restore			/* (Preserves a2) */

	/* Restore only level-specific regs (the rest were already restored) */

	l32i	a0, a2, (4 * REG_PS)			/* Retrieve interruptee's PS */
	wsr		a0, PS
	l32i	a0, a2, (4 * REG_PC)			/* Retrieve interruptee's PC */
	wsr		a0, EPC_1
	l32i	a0, a2, (4 * REG_A0)			/* Retrieve interruptee's A0 */
	l32i	sp, a2, (4 * REG_A1)			/* Retrieve interrupt stack frame */
	l32i	a2, a2, (4 * REG_A2)			/* Retrieve interruptee's A2 */
	rsync									/* Ensure PS and EPC written */

	/* Return from exception. RFE returns from either the UserExceptionVector
	 * or the KernelExceptionVector.  RFE sets PS.EXCM back to 0, and then
	 * jumps to the address in EPC[1]. PS.UM and PS.WOE are left unchanged.
	 */

	rfe

/****************************************************************************
 * Name: _xtensa_syscall_handler
 *
 * Description:
 *   Syscall Exception Handler (jumped to from User Exception Handler).
 *   Syscall 0 is required to spill the register windows (no-op in Call 0 ABI).
 *   Only syscall 0 is handled here. Other syscalls return -1 to caller in a2.
 *
 * Entry Conditions:
 *   A0 saved in EXCSAVE_1.  All other register as upon exception.
 *
 ****************************************************************************/

	.section HANDLER_SECTION, "ax"
	.type       _xtensa_syscall_handler, @function
	.align      4

_xtensa_syscall_handler:

	/* Allocate stack frame and save A0, A1, and PS */

	mov		a0, sp							/* Save SP in A0 */
	addi	sp, sp, -XCPTCONTEXT_SIZE	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, PS							/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EXCSAVE_1					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)
	s32i	a2, sp, (4 * REG_A2)

	/* Save rest of interrupt context. */

	call0	_xtensa_context_save

	/* Save EPC */

#if XCHAL_HAVE_LOOPS != 0
	/* Get the interruptee's PC and skip over the 'syscall' instruction.
	 * If it's at the end of a zero-overhead loop and it's not on the last
	 * iteration, decrement loop counter and skip to beginning of loop.
	 */

	rsr		a2, EPC_1						/* a2 = PC of 'syscall' */
	addi	a3, a2, 3						/* Increment PC */

	rsr		a0, LEND						/* Skip if PC != LEND */
	bne		a3, a0, 1f

	rsr		a0, LCOUNT						/* Skip if LCOUNT == 0 */
	beqz	a0, 1f

	addi	a0, a0, -1						/* Decrement LCOUNT */
	rsr		a3, LBEG						/* Set PC = LBEG */
	wsr		a0, LCOUNT						/* Save the new LCOUNT */

1:
	wsr		a3, EPC_1						/* Update PC */
	s32i	a3, sp, (4 * REG_PC)

#else
	/* Get the interruptee's PC and skip over the 'syscall' instruction. */

	rsr		a1, EPC_1						/* a2 = PC of 'syscall' */
	addi	a0, a1, 3						/* ++PC */

	wsr		a0, EPC_1						/* Update PC */
	s32i	a0, sp, (4 * REG_PC)
#endif

	/* Dispatch the sycall as with other interrupts. */

	mov		a12, sp							/* a12 = address of register save area */

	/* Switch to an interrupt stack if we have one */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
	setintstack a13 a14
#endif

	/* Set up PS for C, enable interrupts above this level and clear EXCM. */

	ps_setup	1 a0

#ifdef __XTENSA_CALL0_ABI__
	movi	a2, XTENSA_IRQ_SYSCALL			/* Argument 1: IRQ number */
	mov		a3, a12							/* Argument 2: Top of stack = register save area */
	call0	xtensa_irq_dispatch				/* Call xtensa_int_decode */

	/* On return from xtensa_irq_dispatch, A2 will contain the address of the new
	 * register save area.  Usually this would be the same as the current SP.
	 * But in the event of a context switch, A2 will instead refer to the TCB
	 * register save area.
	 */

#else
	movi	a6, XTENSA_IRQ_SYSCALL			/* Argument 1: IRQ number */
	mov		a7, a12							/* Argument 2: Top of stack = register save area */
	call4	xtensa_irq_dispatch				/* Call xtensa_int_decode */

	/* On return from xtensa_irq_dispatch, A6 will contain the address of the new
	 * register save area.  Usually this would be the same as the current SP.
	 * But in the event of a context switch, A6 will instead refer to the TCB
	 * register save area.
	 */

	mov		a2, a6							/* Switch to the new register save area */
#endif

	/* Restore registers in preparation to return from interrupt */

	call0	_xtensa_context_restore			/* (Preserves a2) */

	/* Restore only level-specific regs (the rest were already restored) */

	l32i	a0, a2, (4 * REG_PS)			/* Retrieve interruptee's PS */
	wsr		a0, PS
	l32i	a0, a2, (4 * REG_PC)			/* Retrieve interruptee's PC */
	wsr		a0, EPC_1
	l32i	a0, a2, (4 * REG_A0)			/* Retrieve interruptee's A0 */
	l32i	sp, a2, (4 * REG_A1)			/* Retrieve interrupt stack frame */
	l32i	a2, a2, (4 * REG_A2)			/* Retrieve interruptee's A2 */
	rsync									/* Ensure PS and EPC written */

	/* Return from exception. RFE returns from either the UserExceptionVector
	 * or the KernelExceptionVector.  RFE sets PS.EXCM back to 0, and then
	 * jumps to the address in EPC[1]. PS.UM and PS.WOE are left unchanged.
	 */

	rfe

/****************************************************************************
 * Name: _xtensa_coproc_handler
 *
 * Description:
 *   Co-Processor Exception Handler (jumped to from User Exception Handler).
 *   This logic handlers handles the User Coprocessor[n]Disabled exceptions,
 *   n=0-7.  A User Coprocessor[n]Disabled exception occurs when if logic
 *   executes a co-processor n instruction while coprocessor n is disabled.
 *
 *   This exception allows for lazy context switch of co-processor state:
 *   CPENABLE can be cleared on each context switch.  When logic on the
 *   thread next accesses the co-processor, this exception will occur and
 *   the exception handler may then enable the co-processor on behalf of
 *   the thread.
 *
 *   NuttX does not currently implement this lazy co-process enable.  Rather,
 *   NuttX follows the model:
 *
 *   1. A set of co-processors may be enable when each thread starts as
 *      determined by CONFIG_XTENSA_CP_INITSET.
 *   2. Additional co-processors may be enabled for the thread by explicitly
 *      setting the CPENABLE register when the thread starts.
 *   3. Co-processor state, including CPENABLE, is saved an restored on each
 *      context switch.
 *   4. Any Coprocessor[n]Disabled exceptions result in a system PANIC.
 *
 *   These exceptions are generated by co-processor instructions, which are
 *   only allowed in thread code (not in interrupts or kernel code).  This
 *   restriction is deliberately imposed to reduce the burden of state-save/
 *   restore in interrupts.
 *
 * Entry Conditions:
 *   A0 saved in EXCSAVE_1.  All other register as upon exception.
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_CP_LAZY
/* Lazy co-processor restoration is not implemented.  Below, the logic simply
 * calls xtensa_user() which will crash the system with an unhandled error
 * Duplicates logic above.
 */

#error Lazy co-processor restoration is not implemented

#if XCHAL_CP_NUM > 0
	.type	_xtensa_coproc_handler, @function
	.align	4

_xtensa_coproc_handler:

	/* For now, just panic */

	mov		a0, sp							/* Save SP in A0 */
	addi	sp, sp, -XCPTCONTEXT_SIZE	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, PS							/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_1						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_1					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)
	s32i	a2, sp, (4 * REG_A2)

	/* Save rest of interrupt context. */

	call0	_xtensa_context_save			/* Save full register state */

	/* Switch to an interrupt stack if we have one */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
	setintstack a13 a14
#endif

	/* Save exc cause and vaddr into exception frame */

	rsr		a0, EXCCAUSE
	s32i	a0, sp, (4 * REG_EXCCAUSE)
	rsr		a0, EXCVADDR
	s32i	a0, sp, (4 * REG_EXCVADDR)

	/* Set up PS for C, re-enable hi-pri interrupts, and clear EXCM. */

	ps_setup	1 a0

	/* Call xtensa_user_panic, passing both the EXCCAUSE and a pointer to the
	 * beginning of the register save area.
	 */

#ifdef __XTENSA_CALL0_ABI__
	rsr		a2, EXCCAUSE					/* Argument 1 (a2) = EXCCAUSE */
	mov		a3, sp							/* Argument 2 (a2) = pointer to register save area */
	calx0	xtensa_user_panic				/* Call xtensa_user_panic */
#else
	rsr		a6, EXCCAUSE					/* Argument 1 (a2) = EXCCAUSE */
	mov		a7, sp							/* Argument 2 (a2) = pointer to register save area */
	call4	xtensa_user_panic				/* Call xtensa_user_panic */
#endif

	/* xtensa_user_panic should not return */

1:	j		1b

#endif /* XCHAL_CP_NUM */
#endif /* CONFIG_XTENSA_CP_LAZY */
