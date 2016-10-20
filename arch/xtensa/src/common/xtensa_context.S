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

#ifdef CONFIG_XTENSA_USE_OVLY
#  include <xtensa/overlay_os_asm.h>
#endif

#warning REVIST XCHAL_EXTRA_SA_SIZE is not yet provided
#define XCHAL_EXTRA_SA_SIZE 0 /* REMOVE ME */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: _xtensa_context_save
 *       _xtensa_full_context_save
 *
 * Description:
 *
 *   NOTE: MUST BE CALLED ONLY BY 'CALL0' INSTRUCTION!
 *
 *   These functions save Xtensa processor state:
 *
 *   - _xtensa_full_context_save saves all registers except PC, PS, A0, and
 *     A1 (SP)
 *   - _xtensa_context_save, in addition, does not save A12-A15 which
 *     are preserved by the callee).
 *
 *   Caller is expected to have saved PC, PS, A0, and A1 (SP).
 *
 *   The counterparts to these functions are _xtensa_context_restore() and
 *   xtensa_full_context_restore().
 *
 * Entry Conditions:
 *   - A0  = Return address in caller.
 *   - A1  = Stack pointer of calling thread or interrupt handler.
 *   - Other processor state except PC, PS, A0, and A1 (SP) are as at
 *     the point of interruption.
 *   - If windowed ABI, PS.EXCM = 1 (exceptions disabled).
 *
 *   Exit conditions:
 *   - A0  = Return address in caller.
 *   - A1  = Stack pointer of interrupted thread or handler ("interruptee").
 *   - A12-A15 as at entry (preserved).
 *   - If windowed ABI, PS.EXCM = 1 (exceptions disabled).
 *
 ****************************************************************************/

	.global	_xtensa_full_context_save
	.global	_xtensa_context_save
	.type	_xtensa_full_context_save, @function
	.type	_xtensa_context_save, @function

	.align	4
	.literal_position
	.align	4

_xtensa_full_context_save:

#ifndef CONFIG_XTENSA_CALL0_ABI
	/* Save Call0 ABI callee-saved regs a12-15. */

	s32i	a12, sp, (4 * REG_A12)
	s32i	a13, sp, (4 * REG_A13)
	s32i	a14, sp, (4 * REG_A14)
	s32i	a15, sp, (4 * REG_A15)
#endif

_xtensa_context_save:

	s32i	a2,  sp, (4 * REG_A2)
	s32i	a3,  sp, (4 * REG_A3)
	s32i	a4,  sp, (4 * REG_A4)
	s32i	a5,  sp, (4 * REG_A5)
	s32i	a6,  sp, (4 * REG_A6)
	s32i	a7,  sp, (4 * REG_A7)
	s32i	a8,  sp, (4 * REG_A8)
	s32i	a9,  sp, (4 * REG_A9)
	s32i	a10, sp, (4 * REG_A10)
	s32i	a11, sp, (4 * REG_A11)

	/* Call0 ABI callee-saved regs a12-15 do not need to be saved here.
	 * a12-13 are the caller's responsibility so it can use them as scratch.
	 * So only need to save a14-a15 here for Windowed ABI (not Call0).
	 */

#ifndef CONFIG_XTENSA_CALL0_ABI
	s32i	a12, sp, (4 * REG_A12)
	s32i	a13, sp, (4 * REG_A13)
	s32i	a14, sp, (4 * REG_A14)
	s32i	a15, sp, (4 * REG_A15)
#endif

	rsr		a3, SAR
	s32i	a3, sp, (4 * REG_SAR)

#ifdef CONFIG_XTENSA_HAVE_LOOPS
	rsr		a3, LBEG
	s32i	a3, sp, (4 * REG_LBEG)
	rsr		a3, LEND
	s32i	a3, sp, (4 * REG_LEND)
	rsr		a3, LCOUNT
	s32i	a3, sp, (4 * REG_LCOUNT)
#endif

#ifdef CONFIG_XTENSA_USE_SWPRI
	/* Save virtual priority mask */

	movi	a3, _xtensa_vprimask
	l32i	a3, a3, 0
	s32i	a3, sp, (4 * REG_VPRI)
#endif

#if XCHAL_EXTRA_SA_SIZE > 0 || !defined(CONFIG_XTENSA_CALL0_ABI)
	mov		a9,  a0						/* Preserve ret addr */
	#endif

#ifndef CONFIG_XTENSA_CALL0_ABI
	/* To spill the reg windows, temp. need pre-interrupt stack ptr and
	 * a4-15.  Need to save a9,12,13 temporarily (in frame temps) and
	 * recover originals.  Interrupts need to be disabled below
	 * CONFIG_XTENSA_EXCM_LEVEL and window overflow and underflow exceptions
	 * disabled (assured by PS.EXCM == 1).
	 */

	s32i	a12, sp, (4 * REG_TMP0)		/* Temp. save stuff in stack frame */
	s32i	a13, sp, (4 * REG_TMP1)
	s32i	a9,  sp, (4 * REG_TMP2)

	/* Save the overlay state if we are supporting overlays. Since we just
	 * saved three registers, we can conveniently use them here. Note that
	 * as of now, overlays only work for windowed calling ABI.
	 */

#ifdef CONFIG_XTENSA_USE_OVLY
	l32i	a9,  sp, (4 * REG_PC)		/* Recover saved PC */
	_xt_overlay_get_state	a9, a12, a13
	s32i	a9,  sp, (4 * REG_OVLY)		/* Save overlay state */
#endif

	l32i	a12, sp, (4 * REG_A12)			/* Recover original a9,12,13 */
	l32i	a13, sp, (4 * REG_A13)
	l32i	a9,  sp, (4 * REG_A9)
	addi	sp,  sp, (4 * REG_FRMSZ)		/* Restore the interruptee's SP */
	call0   xthal_window_spill_nw			/* Preserves only a4,5,8,9,12,13 */
	addi	sp,  sp, -(4 * REG_FRMSZ)
	l32i	a12, sp, (4 * REG_TMP0)			/* Recover stuff from stack frame */
	l32i	a13, sp, (4 * REG_TMP1)
	l32i	a9,  sp, (4 * REG_TMP2)
#endif

#if XCHAL_EXTRA_SA_SIZE > 0
	/* NOTE: Normally the xthal_save_extra_nw macro only affects address
	 * registers a2-a5. It is theoretically possible for Xtensa processor
	 * designers to write TIE that causes more address registers to be
	 * affected, but it is generally unlikely. If that ever happens,
	 * more registers need to be saved/restored around this macro invocation.
	 * Here we assume a9,12,13 are preserved.
	 * Future Xtensa tools releases might limit the regs that can be affected.
	 */

	addi	a2,  sp, (4 * REG_EXTRA)		/* Where to save it */
#if XCHAL_EXTRA_SA_ALIGN > 16
	movi	a3, -XCHAL_EXTRA_SA_ALIGN
	and		a2, a2, a3						/* Align dynamically >16 bytes */
#endif
	call0   xthal_save_extra_nw				/* Destroys a0,2,3,4,5 */
#endif

#if XCHAL_EXTRA_SA_SIZE > 0 || !defined(CONFIG_XTENSA_CALL0_ABI)
	mov		a0, a9							/* Retrieve ret addr */
#endif

	ret

/****************************************************************************
 * Name: _xtensa_context_restore
 *       _xtensa_full_context_restore
 *
 * Description:
 *
 *   NOTE: MUST BE CALLED ONLY BY 'CALL0' INSTRUCTION!
 *
 *   These functions restores Xtensa processor state and differ in which
 *   registers are saved:
 *
 *   - _xtensa_full_context_restore restores all registers except PC, PS,
 *     A0, and A1 (SP)
 *   - _xtensa_context_restore, in addition, does not restore A12-A15 which
 *     are preserved by the callee).
 *
 *   The caller is responsible for restoring PC, PS, A0, A1 (SP) in both cases.
 *
 *   _xtensa_context_save and _xtensa_full_context_save are the counterparts
 *   to these functions.
 *
 * Entry Conditions:
 *   - A0  = Return address in caller.
 *   - A1  = Stack pointer of calling thread or interrupt handler.
 *
 * Exit conditions:
 *   - A0  = Return address in caller.
 *   - A1  = Stack pointer of calling thread or interrupt handler.
 *   - Other registers are restored as detailed above.
 *
 ****************************************************************************/

	.global _xtensa_full_context_restore
	.global _xtensa_context_restore
	.type   _xtensa_full_context_restore,@function
	.type   _xtensa_context_restore,@function

	.align  4
	.literal_position
	.align  4

_xtensa_full_context_restore:

#ifndef CONFIG_XTENSA_CALL0_ABI
	/* Restore Call0 ABI callee-saved regs a12-15. */

	l32i	a12, sp, (4 * REG_A12)
	l32i	a13, sp, (4 * REG_A13)
	l32i	a14, sp, (4 * REG_A14)
	l32i	a15, sp, (4 * REG_A15)
#endif

_xtensa_context_restore:

#if XCHAL_EXTRA_SA_SIZE > 0
	/* NOTE: Normally the xthal_restore_extra_nw macro only affects address
	 * registers a2-a5. It is theoretically possible for Xtensa processor
	 * designers to write TIE that causes more address registers to be
	 * affected, but it is generally unlikely. If that ever happens,
	 * more registers need to be saved/restored around this macro invocation.
	 * Here we only assume a13 is preserved.
	 * Future Xtensa tools releases might limit the regs that can be affected.
	 */

	mov		a13, a0							/* Preserve ret addr */
	addi	a2, sp, (4 * REG_EXTRA)			/* Where to find it */

#if XCHAL_EXTRA_SA_ALIGN > 16
	movi	a3, -XCHAL_EXTRA_SA_ALIGN
	and		a2, a2, a3						/* Align dynamically >16 bytes */
#endif
	call0   xthal_restore_extra_nw			/* Destroys a0,2,3,4,5 */
	mov		a0, a13							/* Retrieve ret addr */
#endif

#ifdef CONFIG_XTENSA_HAVE_LOOPS
	l32i	a2, sp, (4 * REG_LBEG)
	l32i	a3, sp, (4 * REG_LEND)
	wsr		a2, LBEG
	l32i	a2, sp, (4 * REG_LCOUNT)
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

	l32i	a2, sp, (4 * REG_PC)			/* Retrieve PC */
	l32i	a3, sp, (4 * REG_PS)			/* Retrieve PS */
	l32i	a4, sp, (4 * REG_OVLY)			/* Retrieve overlay state */
	l32i	a5, sp, (4 * REG_A1)			/* Retrieve stack ptr */
	_xt_overlay_check_map	a2, a3, a4, a5, a6
	s32i	a2, sp, (4 * REG_PC)			/* Save updated PC */
	s32i	a3, sp, (4 * REG_PS)			/* Save updated PS */
#endif

#ifdef CONFIG_XTENSA_USE_SWPRI
	/* Restore virtual interrupt priority and interrupt enable */

	movi	a3, _xtensa_intdata
	l32i	a4, a3, 0						/* a4 = _xtensa_intenable */
	l32i	a5, sp, (4 * REG_VPRI)			/* a5 = saved _xtensa_vprimask */
	and		a4, a4, a5
	wsr		a4, INTENABLE					/* Update INTENABLE */
	s32i	a5, a3, 4						/* Restore _xtensa_vprimask */
	#endif

	l32i	a3, sp, (4 * REG_SAR)
	l32i	a2, sp, (4 * REG_A2)
	wsr		a3, SAR
	l32i	a3, sp, (4 * REG_A3)
	l32i	a4, sp, (4 * REG_A4)
	l32i	a5, sp, (4 * REG_A5)
	l32i	a6, sp, (4 * REG_A6)
	l32i	a7, sp, (4 * REG_A7)
	l32i	a8, sp, (4 * REG_A8)
	l32i	a9, sp, (4 * REG_A9)
	l32i	a10, sp, (4 * REG_A10)
	l32i	a11, sp, (4 * REG_A11)

	/* Call0 ABI callee-saved regs a12-15 do not need to be restored here.
	 * However a12-13 were saved for scratch before XT_RTOS_INT_ENTER(),
	 * so need to be restored anyway, despite being callee-saved in Call0.
	 */

#ifndef CONFIG_XTENSA_CALL0_ABI
	l32i	a12, sp, (4 * REG_A12)
	l32i	a13, sp, (4 * REG_A13)
	l32i	a14, sp, (4 * REG_A14)
	l32i	a15, sp, (4 * REG_A15)
#endif

	ret
