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

#include "chip.h"
#include "syscall.h"
#include "xtensa_asm_utils.h"
#include "xtensa_coproc.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _xtensa_context_save
 *
 * Description:
 *
 *   NOTE: MUST BE CALLED ONLY BY 'CALL0' INSTRUCTION!
 *
 *   This function saves Xtensa processor state.
 *   It is called directly by interrupt handling logic with interrupts
 *   disabled.  Registers PC, PS, A0, A1 (SP), A2 and A3 are saved before
 *   calling this function.
 *
 *   The counterpart to this function is _xtensa_context_restore().
 *
 * Entry Conditions:
 *   - A0  = Return address to caller.
 *   - A2  = Pointer to the processor state save area
 *   - Other processor state except PC, PS, A0, A1 (SP), A2 and A3 are as at
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

	.section HANDLER_SECTION, "ax"
	.global	_xtensa_context_save
	.type	_xtensa_context_save, @function

	.align	4
	.literal_position
	.align	4

_xtensa_context_save:

	s32i	a3,  sp, (4 * REG_A3)
	s32i	a4,  sp, (4 * REG_A4)
	s32i	a5,  sp, (4 * REG_A5)
	s32i	a6,  sp, (4 * REG_A6)
	s32i	a7,  sp, (4 * REG_A7)
	s32i	a8,  sp, (4 * REG_A8)
	s32i	a9,  sp, (4 * REG_A9)
	s32i	a10, sp, (4 * REG_A10)
	s32i	a11, sp, (4 * REG_A11)

	/* Call0 ABI callee-saved regs a12-15 do not need to be saved here */

#ifndef __XTENSA_CALL0_ABI__
	s32i	a12, sp, (4 * REG_A12)
	s32i	a13, sp, (4 * REG_A13)
	s32i	a14, sp, (4 * REG_A14)
	s32i	a15, sp, (4 * REG_A15)
#endif

	rsr		a3, SAR
	s32i	a3, sp, (4 * REG_SAR)

#if XCHAL_HAVE_LOOPS != 0
	rsr		a3, LBEG
	s32i	a3, sp, (4 * REG_LBEG)
	rsr		a3, LEND
	s32i	a3, sp, (4 * REG_LEND)
	rsr		a3, LCOUNT
	s32i	a3, sp, (4 * REG_LCOUNT)
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

	/* SPILL_ALL_WINDOWS macro requires window overflow exceptions to be enabled,
	 * i.e. PS.EXCM cleared and PS.WOE set.
	 * Since we are going to clear PS.EXCM, we also need to increase INTLEVEL
	 * at least to XCHAL_EXCM_LEVEL. This matches that value of effective INTLEVEL
	 * at entry (CINTLEVEL=max(PS.INTLEVEL, XCHAL_EXCM_LEVEL) when PS.EXCM is set.
	 * Since WindowOverflow exceptions will trigger inside SPILL_ALL_WINDOWS,
	 * we need to save/restore EPC1 as well.
	 * NOTE: Even though a4-a15 are saved into the exception frame, we should not
	 * clobber them until after SPILL_ALL_WINDOWS. This is because these registers
	 * may contain live windows belonging to previous frames in the call stack.
	 * These frames will be spilled by SPILL_ALL_WINDOWS, and if the register was
	 * used as a temporary by this code, the temporary value would get stored
	 * onto the stack, instead of the real value.
	 */

	s32i    a0, sp, (4 * REG_TMP0)     /* Save return address */
	rsr     a2, PS                     /* To be restored after SPILL_ALL_WINDOWS */
	movi    a0, PS_INTLEVEL_MASK
	and     a3, a2, a0                 /* Get the current INTLEVEL */
	bgeui   a3, XCHAL_EXCM_LEVEL, 1f   /* Calculate max(INTLEVEL, XCHAL_EXCM_LEVEL) */
	movi    a3, XCHAL_EXCM_LEVEL
1:
	movi    a0, PS_UM | PS_WOE         /* Clear EXCM, enable window overflow, set new INTLEVEL */
	or      a3, a3, a0
	wsr     a3, ps
	rsync
	rsr     a0, EPC1                   /* To be restored after SPILL_ALL_WINDOWS */

	addi    sp,  sp, XCPTCONTEXT_SIZE  /* Go back to spill register region */
	SPILL_ALL_WINDOWS                  /* Place the live register windows there */
	addi    sp,  sp, -XCPTCONTEXT_SIZE /* Return the current stack pointer and proceed with context save*/

	wsr     a2, PS                     /* Restore PS to the value at entry */
	wsr     a0, EPC1                   /* Restore EPC1 to the value at entry */
	rsync
	l32i	a0, sp, (4 * REG_TMP0)       /* Restore return address */

#endif

#if XCHAL_CP_NUM > 0
	xtensa_coproc_savestate
#endif

	ret

	.size	_xtensa_context_save, . - _xtensa_context_save

/****************************************************************************
 * Name: _xtensa_context_restore
 *
 * Description:
 *
 *   NOTE: MUST BE CALLED ONLY BY 'CALL0' INSTRUCTION!
 *
 *   This function restores Xtensa processor state.
 *   It is called directly by interrupt handling logic with interrupts
 *   disabled.  It restores all registers except PC, PS, A0, and A2
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

	.section HANDLER_SECTION, "ax"
	.global _xtensa_context_restore
	.type	_xtensa_context_restore,@function

	.align  4
	.literal_position
	.align  4

_xtensa_context_restore:

#if XCHAL_CP_NUM > 0
	xtensa_coproc_restorestate
#endif

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

