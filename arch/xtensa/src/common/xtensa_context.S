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
#include <arch/xtensa/xtensa_specregs.h>

#include "xtensa_abi.h"

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
 *   context, caller saves saves PC, PS, A0, A1 (SP), and A2.  This
 *   logic also executes indirectly from xtena_context_save() by falling
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

	s32i	a2,  a2, (4 * REG_A2)
	s32i	a3,  a2, (4 * REG_A3)
	s32i	a4,  a2, (4 * REG_A4)
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

#ifdef XCHAL_HAVE_LOOPS
	rsr		a3, LBEG
	s32i	a3, a2, (4 * REG_LBEG)
	rsr		a3, LEND
	s32i	a3, a2, (4 * REG_LEND)
	rsr		a3, LCOUNT
	s32i	a3, a2, (4 * REG_LCOUNT)
#endif

#ifndef __XTENSA_CALL0_ABI__
	mov		a9,  a0							/* Preserve ret addr */
#endif

#ifndef __XTENSA_CALL0_ABI__
	/* To spill the reg windows, temp. need pre-interrupt stack ptr and
	 * a4-15.  Need to save a9,12,13 temporarily (in frame temps) and
	 * recover originals.  Interrupts need to be disabled below
	 * XCHAL_EXCM_LEVEL and window overflow and underflow exceptions
	 * disabled (assured by PS.EXCM == 1).
	 */

	s32i	a12, a2, (4 * REG_TMP0)			/* Temp. save stuff in stack frame */
	s32i	a13, a2, (4 * REG_TMP1)
	s32i	a9,  a2, (4 * REG_TMP2)

#ifdef CONFIG_XTENSA_USE_OVLY
	/* Save the overlay state if we are supporting overlays. Since we just
	 * saved three registers, we can conveniently use them here. Note that
	 * as of now, overlays only work for windowed calling ABI.
	 */

#error Overly support is not implemented
#endif

	l32i	a12, a2, (4 * REG_A12)			/* Recover original a9,12,13 */
	l32i	a13, a2, (4 * REG_A13)
	l32i	a9,  a2, (4 * REG_A9)

#warning REVISIT: The following is probably not correct due to changes in registers
	addi	sp,  sp, (4 * XCPTCONTEXT_SIZE)	/* Restore the interruptee's SP */
	call0	_xtensa_window_spill			/* Preserves only a4,5,8,9,12,13 */
	addi	sp,  sp, -(4 * XCPTCONTEXT_SIZE)
	l32i	a12, sp, (4 * REG_TMP0)			/* Recover stuff from stack frame */
	l32i	a13, sp, (4 * REG_TMP1)
	l32i	a9,  sp, (4 * REG_TMP2)
#endif

	ret

	.size	_xtensa_context_save, . - _xtensa_context_save

/****************************************************************************
 * Name: xtensa_context_save
 *
 * Description:
 *
 *   NOTE: MUST BE CALLED ONLY BY 'CALL0' INSTRUCTION!
 *
 *   This functions implements the moral equivalent of setjmp().  It is
 *   called from user code (with interrupts disabled) to save the current
 *   state of the running thread.  This function always returns zero.
 *   However, it sets the saved value of the return address (A2) to 1.
 *   If the thread is restarted via _xtensa_contest_restore or
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

	.global	xtensa_context_save
	.type	xtensa_context_save, @function

	.align	4
	.literal_position
	.align	4

xtensa_context_save:

	/* Set up for call to _xtensa_context_save() */

	rsr		a12, PS							/* Save callee's PS */
	s32i	a12, a2, (4 * REG_PS)
	s32i	a0, a2, (4 * REG_PC)			/* Save Return address as PC */

	s32i	a0, a2, (4 * REG_A0)			/* Save callee's a0 */
	s32i	sp, a2, (4 * REG_A1)			/* Save callee's SP */
	movi	a12, 1							/* Set saved A2 to 1 */
	s32i	a12, a2, (4 * REG_A2)

	/* Save the rest of the processor state */

	call0	_xtensa_context_save			/* Save full register state */

	/* Recover the return address and return zero */

	l32i	a0, a2, (4 * REG_A0)			/* Recover return addess */
	movi	a2, 0							/* Return zero */

	ret

	.size	xtensa_context_save, . - xtensa_context_save

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
 *   - Other registers are restored as detailed above (including A1 and A2).
 *
 ****************************************************************************/

	.global _xtensa_context_restore
	.type   _xtensa_context_restore,@function

	.align  4
	.literal_position
	.align  4

_xtensa_context_restore:

#ifdef XCHAL_HAVE_LOOPS
	l32i	a2, a2, (4 * REG_LBEG)
	l32i	a3, a2, (4 * REG_LEND)
	wsr		a2, LBEG
	l32i	a2, a2, (4 * REG_LCOUNT)
	wsr		a3, LEND
	wsr		a2, LCOUNT
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

	l32i	a3, a2, (4 * REG_SAR)
	l32i	sp, a2, (4 * REG_A1)
	wsr		a3, SAR
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

	l32i	a12, a2, (4 * REG_A12)
	l32i	a13, a2, (4 * REG_A13)
	l32i	a14, a2, (4 * REG_A14)
	l32i	a15, a2, (4 * REG_A15)

	/* Finally, restore A2 with the correct value */

	l32i	a2, a2, (4 * REG_A2)

	ret

	.size	_xtensa_context_restore, . - _xtensa_context_restore

/****************************************************************************
 * Name: xtensa_context_restore
 *
 * Description:
 *
 *   NOTE: MUST BE CALLED ONLY BY 'CALL0' INSTRUCTION!
 *
 *   This functions implements the moral equivalent of longjmp().  It is
 *   called from user code (with interrupts disabled) to restor the current
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

	/* Restore the processor state */

	call0	_xtensa_context_restore			/* Restore full register state */

	/* Restore PC, PS, A0, and A2. */

	l32i	a0, a2, (4 * REG_PS)			/* Restore PS */
	wsr		a0, PS
	l32i	a0, a2, (4 * REG_PC)			/* Set up for RFE */
	rsr		a0, EPC
	l32i	a0, a2, (4 * REG_A0)			/* Restore a0 */
	l32i	a2, a2, (4 * REG_A2)			/* Restore A2 */

	rfe										/* And return from "exception" */

	.size	xtensa_context_restore, . - xtensa_context_restore
