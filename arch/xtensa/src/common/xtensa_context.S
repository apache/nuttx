/****************************************************************************
 * arch/xtensa/src/common/xtensa_context.S
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

	.file	"xtensa_context.S"

/* XTENSA CONTEXT SAVE AND RESTORE ROUTINES
 *
 * Low-level Call0 functions for handling generic context save and restore
 * of registers not specifically addressed by the interrupt vectors and
 * handlers.  Those registers (not handled by these functions) are PC, PS,
 * A0, A1 (SP).
 *
 * Note that in Call0 ABI, interrupt handlers are expected to preserve the callee-
 * save regs (A12-A15), which is always the case if the handlers are coded in C.
 * However A12, A13 are made available as scratch registers for interrupt dispatch
 * code, so are presumed saved anyway, and are always restored even in Call0 ABI.
 * Only A14, A15 are truly handled as callee-save regs.
 *
 * Because Xtensa is a configurable architecture, this port supports all user
 * generated configurations (except restrictions stated in the release notes).
 * This is accomplished by conditional compilation using macros and functions
 * defined in the Xtensa HAL (hardware adaptation layer) for your configuration.
 * Only the processor state included in your configuration is saved and restored,
 * including any processor state added by user configuration options or TIE.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include <arch/chip/core-isa.h>
#include <arch/chip/tie.h>
#include <arch/xtensa/xtensa_abi.h>
#include <arch/xtensa/xtensa_specregs.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: _xtensa_context_save
 *
 * Description:
 *
 *   NOTE: MUST BE CALLED ONLY BY 'CALL0' INSTRUCTION!
 *
 *   This function saves Xtensa processor state:  xtensa_context_save
 *   saves all registers except PC, PS, A0, A1 (SP), and A2
 *
 *   This function is called directly by interrupt handling logic and from
 *   xtensa_context_save() below with interrupts disabled.  In either calling
 *   context, caller saves PC, PS, A0, A1 (SP), and A2.  This
 *   logic also executes indirectly from xtensa_context_save() by falling
 *   through from above.
 *
 *   The counterpart to this function is _xtensa_context_restore().
 *
 * Entry Conditions:
 *   - A0  = Return address to caller.
 *   - A2  = Pointer to the processor state save area
 *   - Other processor state except PC, PS, A0, A1 (SP), and A2 are as at
 *     the point of interruption.
 *
 * Exit conditions:
 *   - A0  = Return address in caller.
 *   - A2, A12-A15 as at entry (preserved).
 *
 * Assumptions:
 *   - Caller is expected to have saved PC, PS, A0, A1 (SP), and A2.
 *   - If windowed ABI, PS.EXCM = 1 (exceptions disabled).
 *
 ****************************************************************************/

	.global	_xtensa_context_save
	.type	_xtensa_context_save, @function

	.align	4
	.literal_position
	.align	4

_xtensa_context_save:

	s32i	a3,  a2, (4 * REG_A3)
	s32i	a4,  a2, (4 * REG_A4)
	s32i	a5,  a2, (4 * REG_A5)
	s32i	a6,  a2, (4 * REG_A6)
	s32i	a7,  a2, (4 * REG_A7)
	s32i	a8,  a2, (4 * REG_A8)
	s32i	a9,  a2, (4 * REG_A9)
	s32i	a10, a2, (4 * REG_A10)
	s32i	a11, a2, (4 * REG_A11)

	/* Call0 ABI callee-saved regs a12-15 do not need to be saved here */

#ifndef __XTENSA_CALL0_ABI__
	s32i	a12, a2, (4 * REG_A12)
	s32i	a13, a2, (4 * REG_A13)
	s32i	a14, a2, (4 * REG_A14)
	s32i	a15, a2, (4 * REG_A15)
#endif

	rsr		a3, SAR
	s32i	a3, a2, (4 * REG_SAR)

#if XCHAL_HAVE_LOOPS != 0
	rsr		a3, LBEG
	s32i	a3, a2, (4 * REG_LBEG)
	rsr		a3, LEND
	s32i	a3, a2, (4 * REG_LEND)
	rsr		a3, LCOUNT
	s32i	a3, a2, (4 * REG_LCOUNT)
#endif

#ifndef __XTENSA_CALL0_ABI__
	/* To spill the reg windows, temp. need pre-interrupt stack ptr and
	 * a4-15.  Interrupts need to be disabled below XCHAL_EXCM_LEVEL and
	 * window overflow and underflow exceptions disabled (assured by
	 * PS.EXCM == 1).
	 */

#ifdef CONFIG_XTENSA_USE_OVLY
	/* Save the overlay state if we are supporting overlays. Since we just
	 * saved three registers, we can conveniently use them here. Note that
	 * as of now, overlays only work for windowed calling ABI.
	 */

#error Overlay support is not implemented
#endif

	s32i	a0, a2, (4 * REG_TMP0)			/* Save return address */
	s32i	sp, a2, (4 * REG_TMP1)			/* Save current stack pointer */
	wsr		a2, EXCSAVE_1					/* Preserve register save area */

	l32i	sp, a2, (4 * REG_A1)			/* Restore the interruptee's SP */
	call0	_xtensa_window_spill			/* Preserves only a4-a5, a8-a9, a12-a13 */

	rsr		a2, EXCSAVE_1					/* Save interruptee's a0 */
	l32i	a0, a2, (4 * REG_TMP0)			/* Save return address */
	l32i	sp, a2, (4 * REG_TMP1)			/* Save current stack pointer */
#endif

	ret

	.size	_xtensa_context_save, . - _xtensa_context_save

/****************************************************************************
 * Name: xtensa_context_save
 *
 * Description:
 *
 *   This functions implements the moral equivalent of setjmp().  It is
 *   called from user code (with interrupts disabled) to save the current
 *   state of the running thread.  This function always returns zero.
 *   However, it sets the saved value of the return address (A2) to 1.
 *   If the thread is s via _xtensa_context_restore or
 *   xtensa_context_restore, it will appear as a second return from
 *   xtensa_context_save but with the returned value of 1 to distinguish
 *   the two cases.
 *
 *   The counterpart to this function is xtensa_context_restore().
 *
 * Entry Conditions:
 *   - A0  = Return address to caller.
 *   - A2  = Pointer to the processor state save area
 *
 * Exit conditions:
 *   - A0  = Return address in caller.
 *   - A2  = 0
 *
 * Assumptions:
 *   - Interrupts are disabled.
 *
 ****************************************************************************/

#ifdef __XTENSA_CALL0_ABI__

/****************************************************************************
 * Name: xtensa_context_save:
 *
 * Description:
 *   This implementation of xtensa_context_save for the case of the CALL0 ABI
 *
 * Input State:
 *   a0 = The return value to the caller.
 *   a2 = The address of the register state structure
 *
 * Return state:
 *   a0 = The return value to the caller.
 *   a2, a12-a15 preserved as at entry
 *
 ****************************************************************************/

	.global	xtensa_context_save
	.type	xtensa_context_save, @function

	.align	4
	.literal_position
	.align	4

xtensa_context_save:
	ENTRY(16)

	/* Set up for (potential) call to _xtensa_context_save() */

	s32i	a3,  a2, (4 * REG_A3)			/* Get  scratch register */
	rsr		a3, PS							/* Save callee's PS */
	s32i	a3, a2, (4 * REG_PS)
	s32i	a0, a2, (4 * REG_PC)			/* Save Return address as PC */

	s32i	a0, a2, (4 * REG_A0)			/* Save callee's a0 */
	s32i	sp, a2, (4 * REG_A1)			/* Save callee's SP */
	movi	a3, 1							/* Set saved A2 to 1 */
	s32i	a3, a2, (4 * REG_A2)

	/* Save the rest of the processor state.  For the CALL0 ABI, we can use
	 * _xtensa_context_save(),  Otherwise we duplicate the context save here
	 * to avoid the window spill.
	 */

	l32i	a3, a2, (4 * REG_A3)			/* Recover original a3 */
	call0	_xtensa_context_save			/* Save full register state */

	/* Recover the return address and return zero */

	l32i	a0, a2, (4 * REG_A0)			/* Recover return address */
	movi	a2, 0							/* Return zero */
	RET(16)

	.size	xtensa_context_save, . - xtensa_context_save
#endif

/****************************************************************************
 * This implementation of xtensa_context_save for the case of the window ABI.
 * This case is more complex.  For the Window ABI, there is a "hook" that
 * performs the low level state.  xtensa_context_save() is simply a
 * trampoline function that performs the window operations in that
 * configuration.
 ****************************************************************************/

#ifndef __XTENSA_CALL0_ABI__

/****************************************************************************
 * Name: _xtensa_save_hook:
 *
 * Input State:
 *   True return value has already been saved
 *   a0 = The return value into xtensa_context_save()
 *   a2 = The address of the register state structure
 *
 * Return state:
 *   a0, a3 modified.
 *   Other values as on entry
 *   Returned value is in a3 (non-stanadard)
 *
 ****************************************************************************/

	.type	_xtensa_save_hook, @function

	.align	4
	.literal_position
	.align	4

_xtensa_save_hook:

	/* Save the return value of 1 that will be used when returning from a
	 * context switch.  NOTE that the returned value from this function is
	 * expected in a3 (not the usual a2). This also frees up a3 for a use
	 * as a scratch register.
	 */

	movi	a3,  1							/* Set saved a3 to 1 */
	s32i	a3,  a2, (4 * REG_A3)

	/* Save the rest of the processor state.
	 *
	 * REVISIT: We could save a lot here.  It should not be necessary to
	 * preserve all of these registers.  The ABI permits volatile, callee-
	 * saved, registers to be clobbered on function calls.  We save the
	 * whole tamale here mostly for debug purposes.
	 *
	 * NOTE that a3 was saved above.  The true a0 return value was saved
	 * in xtensa_context_save.  The a0 value saved below is the return into
	 * xtensa_context_save.
	 */

	rsr		a3,  PS							/* Save callee's PS */
	s32i	a3,  a2, (4 * REG_PS)
	s32i	a0,  a2, (4 * REG_PC)			/* Save Return address as PC */

	s32i	sp,  a2, (4 * REG_A1)			/* Save callee's SP */
	s32i	a2,  a2, (4 * REG_A2)
	s32i	a4,  a2, (4 * REG_A4)			/* Save remaining registers */
	s32i	a5,  a2, (4 * REG_A5)
	s32i	a6,  a2, (4 * REG_A6)
	s32i	a7,  a2, (4 * REG_A7)
	s32i	a8,  a2, (4 * REG_A8)
	s32i	a9,  a2, (4 * REG_A9)
	s32i	a10, a2, (4 * REG_A10)
	s32i	a11, a2, (4 * REG_A11)

	/* Call0 ABI callee-saved regs a12-15 */

	s32i	a12, a2, (4 * REG_A12)
	s32i	a13, a2, (4 * REG_A13)
	s32i	a14, a2, (4 * REG_A14)
	s32i	a15, a2, (4 * REG_A15)

	rsr		a3, SAR
	s32i	a3, a2, (4 * REG_SAR)

#if XCHAL_HAVE_S32C1I != 0
	rsr		a3, SCOMPARE1
	s32i	a3, a2, (4 * REG_SCOMPARE1)
#endif

#if XCHAL_HAVE_LOOPS != 0
	rsr		a3, LBEG
	s32i	a3, a2, (4 * REG_LBEG)
	rsr		a3, LEND
	s32i	a3, a2, (4 * REG_LEND)
	rsr		a3, LCOUNT
	s32i	a3, a2, (4 * REG_LCOUNT)
#endif

	/* NOTE that the returned value is through a3 */

	movi	a3, 0							/* Return zero, no context switch */
	ret

	.size	_xtensa_save_hook, . - _xtensa_save_hook

/****************************************************************************
 * Name: xtensa_context_save:
 *
 * Description:
 *   This is the implementation of xtensa_context_save for the case of the
 *   window ABI.  In the window ABI configuration, xtensa_context_save is a
 *   thin  "trampoline" layer.  It performs the ENTRY window operations on
 *   entry and the exit.  A call0 is used to force the return from the context
 *   switch to the window return within this trampoline.
 *
 * Input State:
 *   a0 = The true return value to the caller.
 *   a2 = The address of the register state structure
 *
 * Return state:
 *   a0, a2, and a3 modified.
 *   Returned value is in a2
 *
 ****************************************************************************/

	.global	xtensa_context_save
	.type	xtensa_context_save, @function

	.align	4
	.literal_position
	.align	4

xtensa_context_save:
	ENTRY(16)

	/* Save the true return address in the register save structure (a0). */

	s32i	a0, a2, (4 * REG_A0)		/* Save true return address (a0) */

	/* Then perform the actual state save in _xtensa_save_hook.  The saved
	 * EPC will be set to the return from this function then we will do the
	 * RET(16) window fix-up.
	 */

	call0	_xtensa_save_hook			/* Save full register state */

	/* a0 and a2 will be automatically restored in the context switch case
	 * with  a3=1.  In the non-context switch return with a2=0, a2 will still
	 * be valid, but we have to restore	 a0 ourself.  The following should
	 * work in either case.
	 */

	l32i	a0, a2, (4 * REG_A0)		/* Recover the true return address (a0) */
	mov		a2, a3						/* Move a3 to the correct register for return */
	RET(16)

	.size	xtensa_context_save, . - xtensa_context_save
#endif

/****************************************************************************
 * Name: _xtensa_context_restore
 *
 * Description:
 *
 *   NOTE: MUST BE CALLED ONLY BY 'CALL0' INSTRUCTION!
 *
 *   These functions restores Xtensa processor state and differ in which
 *   registers are saved: _xtensa_context_restore() restores all registers
 *   except PC, PS, A0, and A2
 *
 *   The caller is responsible for restoring PC, PS, A0, and A2.
 *
 *   _xtensa_context_save is the counterpart to this function.
 *
 * Entry Conditions:
 *   - A0  = Return address in caller.
 *   - A2  = Pointer to the processor state save area
 *
 * Exit conditions:
 *   - A0  = Return address in caller.
 *   - Other registers are restored as detailed above
 *   - A2 is preserved
 *
 ****************************************************************************/

	.global _xtensa_context_restore
	.type	xtensa_context_restore,@function

	.align  4
	.literal_position
	.align  4

_xtensa_context_restore:

#if XCHAL_HAVE_LOOPS != 0
	l32i	a3, a2, (4 * REG_LBEG)
	l32i	a4, a2, (4 * REG_LEND)
	wsr		a3, LBEG
	l32i	a3, a2, (4 * REG_LCOUNT)
	wsr		a4, LEND
	wsr		a3, LCOUNT
#endif

#ifdef CONFIG_XTENSA_USE_OVLY
	/* If we are using overlays, this is a good spot to check if we need
	 * to restore an overlay for the incoming task. Here we have a bunch
	 * of registers to spare. Note that this step is going to use a few
	 * bytes of storage below SP (SP-20 to SP-32) if an overlay is going
	 * to be restored.
	 */

#error Overly support is not implemented
#endif

#if XCHAL_HAVE_S32C1I != 0
	l32i  a3, a2, (4 * REG_SCOMPARE1)
	wsr   a3, SCOMPARE1
#endif

	l32i	a3, a2, (4 * REG_SAR)
	wsr		a3, SAR
	l32i	sp, a2, (4 * REG_A1)
	l32i	a3, a2, (4 * REG_A3)
	l32i	a4, a2, (4 * REG_A4)
	l32i	a5, a2, (4 * REG_A5)
	l32i	a6, a2, (4 * REG_A6)
	l32i	a7, a2, (4 * REG_A7)
	l32i	a8, a2, (4 * REG_A8)
	l32i	a9, a2, (4 * REG_A9)
	l32i	a10, a2, (4 * REG_A10)
	l32i	a11, a2, (4 * REG_A11)

	/* Call0 ABI callee-saved regs a12-15 */

#ifndef __XTENSA_CALL0_ABI__
	l32i	a12, a2, (4 * REG_A12)
	l32i	a13, a2, (4 * REG_A13)
	l32i	a14, a2, (4 * REG_A14)
	l32i	a15, a2, (4 * REG_A15)
#endif

	ret

	.size	_xtensa_context_restore, . - _xtensa_context_restore

/****************************************************************************
 * Name: xtensa_context_restore
 *
 * Description:
 *
 *   This functions implements the moral equivalent of longjmp().  It is
 *   called from user code (with interrupts disabled) to restore the current
 *   state of the running thread.  This function always appears to be a
 *   second return from xtensa_context_save except that that it returns the
 *   value 1 (because the saved value of A2 was set to 1
 *   inxtensa_context_save()).
 *
 *   The counterpart to this function is xtensa_context_save().
 *
 * Entry Conditions:
 *   - A0  = Return address to caller.
 *   - A2  = Pointer to the processor state save area
 *
 * Exit conditions:
 *   NOTE: That this function does NOT return to the caller but rather
 *   to a new threading context.  It is not necessary to save any of the
 *   caller's registers.
 *
 * Assumptions:
 *   - Interrupts are disabled.
 *
 ****************************************************************************/

	.global	xtensa_context_restore
	.type	xtensa_context_restore, @function

	.align	4
	.literal_position
	.align	4

xtensa_context_restore:
	ENTRY(16)

#ifndef __XTENSA_CALL0_ABI__
	/* Force a spill of the live registers of the thread that has been
	 * suspended.
	 *
	 * _xtensa_window_spill return state:
	 *   a2, a3: clobbered
	 *   a4,a5,a8,a9,a12,a13: preserved
	 *   a6,a7,a10,a11,a14,a15 clobbered if they were part of window(s)
	 *      to be spilled, otherwise they are the same as on entry
	 *   loop registers: Preserved
	 *   SAR: clobbered
	 *
	 * We need to preserve only a2 for _xtensa_context_restore
	 */

	mov		a4, a2							/* Save a2 in a preserved register */
	rsr		a5, PS							/* Save PS in preserved register */

	movi	a3, ~(PS_WOE_MASK | PS_INTLEVEL_MASK)
	and		a2, a5, a3						/* Clear WOE, INTLEVEL */
	addi	a2, a2, XCHAL_EXCM_LEVEL		/* Set INTLEVEL = XCHAL_EXCM_LEVEL */
	wsr		a2, PS							/* Apply to PS */
	rsync

	call0	_xtensa_window_spill
	wsr		a5, PS							/* Restore PS */
	rsync

	mov		a2, a4							/* Recover a2 */
#endif

	/* Restore the processor state for the newly started thread */

	call0	_xtensa_context_restore			/* Restore full register state */

	/* Restore PC, PS, A0, and A2. */

	l32i	a0, a2, (4 * REG_PS)			/* Restore PS */
	wsr		a0, PS
	l32i	a0, a2, (4 * REG_PC)			/* Set up for RFE */
	wsr		a0, EPC_1
	l32i	a0, a2, (4 * REG_A0)			/* Restore a0 */
	l32i	a2, a2, (4 * REG_A2)			/* Restore A2 */

	/* Return from exception. RFE returns from either the UserExceptionVector
	 * or the KernelExceptionVector.  RFE sets PS.EXCM back to 0, and then
	 * jumps to the address in EPC[1]. PS.UM and PS.WOE are left unchanged.
	 */

	rfe										/* And return from "exception" */

	.size	xtensa_context_restore, . - xtensa_context_restore
