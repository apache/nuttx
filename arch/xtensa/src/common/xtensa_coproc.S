/****************************************************************************
 * arch/xtensa/src/common/xtensa_coproc.S
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

  .file	"xtensa_coproc.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/xtensa/core.h>
#include <arch/xtensa/xtensa_abi.h>
#include <arch/xtensa/xtensa_coproc.h>
#include <arch/xtensa/xtensa_specregs.h>
#include <arch/chip/core-isa.h>
#include <arch/chip/tie.h>
#include <arch/chip/tie-asm.h>

#if XCHAL_CP_NUM > 0

/****************************************************************************
 * Public Data
 ****************************************************************************/

  .section .rodata, "a"

  /* Offset to CP n save area in thread's CP save area. */

  .global	_xtensa_coproc_saoffsets
  .type	_xtensa_coproc_saoffsets, @object
  .align	16                      /* Minimize crossing cache boundaries */

_xtensa_coproc_saoffsets:

  .word	XTENSA_CP0_SA, XTENSA_CP1_SA, XTENSA_CP2_SA, XTENSA_CP3_SA
  .word	XTENSA_CP4_SA, XTENSA_CP5_SA, XTENSA_CP6_SA, XTENSA_CP7_SA

  .size	_xtensa_coproc_saoffsets, . - _xtensa_coproc_saoffsets

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .text

/****************************************************************************
 * Name: _xtensa_coproc_savestate
 *
 * Description:
 *   If there is a current thread and it has a coprocessor state save area,
 *   then save all callee-saved state into this area. This function is
 *   called from the solicited context switch handler. It calls a system-
 *   specific function to get the coprocessor save area base address.
 *
 *   It is also called from xtensa_coproc_savestate() for synchronous
 *   context switches.  xtensa_coproc_savestate() is simply a C wrapper
 *   around the assembly language call to _xtensa_coproc_savestate.
 *
 * Entry Conditions:
 *   - A2 holds the address of the co-processor state save area
 *   - The thread being switched out is still the current thread.
 *   - CPENABLE state reflects which coprocessors are active.
 *   - Registers have been saved/spilled already.
 *
 * Exit conditions:
 *   - All necessary CP callee-saved state has been saved.
 *   - Registers a2-a7, a13-a15 have been trashed.
 *
 * Must be called from assembly code only, using CALL0.
 *
 ****************************************************************************/

  .global	_xtensa_coproc_savestate
  .type	_xtensa_coproc_savestate, @function

  .align	4
  .literal_position
  .align	4

_xtensa_coproc_savestate:

	/* Move the address of the thread state save area to R15 */

	mov		a15, a2							/* A15 is now the address of the save area */

	/* CPENABLE should show which CPs are enabled. */

	rsr		a2, CPENABLE					/* a2 = which CPs are enabled */
	beqz	a2, .Ldone1						/* Quick exit if none */

	s16i	a2, a15, XTENSA_CPSTORED		/* Save mask of CPs being stored */
	movi	a13, _xtensa_coproc_saoffsets	/* Array of CP save offsets */
	l32i	a15, a15, XTENSA_CPASA			/* a15 = base of aligned save area */

#if XCHAL_CP0_SA_SIZE > 0
	bbci.l	a2, 0, 2f						/* CP 0 not enabled */
	l32i	a14, a13, 0						/* a14 = _xtensa_coproc_saoffsets[0] */
	add		a3, a14, a15					/* a3 = save area for CP 0 */
	xchal_cp0_store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XCHAL_CP1_SA_SIZE > 0
	bbci.l	a2, 1, 2f						/* CP 1 not enabled */
	l32i	a14, a13, 4						/* a14 = _xtensa_coproc_saoffsets[1] */
	add		a3, a14, a15					/* a3 = save area for CP 1 */
	xchal_cp1_store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XCHAL_CP2_SA_SIZE > 0
	bbci.l	a2, 2, 2f
	l32i	a14, a13, 8
	add		a3, a14, a15
	xchal_cp2_store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XCHAL_CP3_SA_SIZE > 0
	bbci.l	a2, 3, 2f
	l32i	a14, a13, 12
	add		a3, a14, a15
	xchal_cp3store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XCHAL_CP4_SA_SIZE > 0
	bbci.l	a2, 4, 2f
	l32i	a14, a13, 16
	add		a3, a14, a15
	xchal_cp4store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XCHAL_CP5_SA_SIZE > 0
	bbci.l	a2, 5, 2f
	l32i	a14, a13, 20
	add		a3, a14, a15
	xchal_cp5store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XCHAL_CP6_SA_SIZE > 0
	bbci.l	a2, 6, 2f
	l32i	a14, a13, 24
	add		a3, a14, a15
	xchal_cp6store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XCHAL_CP7_SA_SIZE > 0
	bbci.l	a2, 7, 2f
	l32i	a14, a13, 28
	add		a3, a14, a15
	xchal_cp7store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

.Ldone1:
	ret

	.size	_xtensa_coproc_savestate, . - _xtensa_coproc_savestate

/****************************************************************************
 * Name: xtensa_coproc_savestate
 *
 * Description:
 *   If there is a current thread and it has a coprocessor state save area,
 *   then save all callee-saved state into this area. xtensa_coproc_savestate()
 *   is simply a C wrapper around the assembly language call to
 *   _xtensa_coproc_savestate.
 *
 * Input Parameters:
 *   A2 - Address of co-processor save area
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with interrupts disabled.
 *
 ****************************************************************************/

	.global	xtensa_coproc_savestate
	.type	xtensa_coproc_savestate, @function

	.align	4
	.literal_position
	.align	4

xtensa_coproc_savestate:

#ifdef __XTENSA_CALL0_ABI__

	/* Need to preserve a8-11.  _xtensa_coproc_savestate modifies a2-a7,
   * a13-a15. a12-a15 are callee saved registers so a13-a14 must be
   * preserved.
   */

	ENTRY(16)
	s32i	a13, sp, LOCAL_OFFSET(1)		/* Save clobbered registers */
	s32i	a14, sp, LOCAL_OFFSET(2)
	s32i	a15, sp, LOCAL_OFFSET(3)

	/* Call _xtensa_coproc_savestate() with A2=address of co-processor
   * save area.
   */

	call0 _xtensa_coproc_savestate

	/* Restore a13-15 and return */

	l32i	a13, sp, LOCAL_OFFSET(1)		/* Restore clobbered registers */
	l32i	a14, sp, LOCAL_OFFSET(2)
	l32i	a15, sp, LOCAL_OFFSET(3)
	RET(16)

#else
	/* Need to preserve a8-15.  _xtensa_coproc_savestate modifies a2-a7,
   * a13-a15.  So a13-a15 may need to be preserved.
   */

	ENTRY(32 /*16*/)						/* REVISIT: Why 32? */
	s32i	a0,  sp, LOCAL_OFFSET(1)		/* Save return address */

	/* Call _xtensa_coproc_savestate() with A2=address of co-processor
   * save area.
   */

	call0 _xtensa_coproc_savestate

	/* Restore a0 and return */

	l32i	a0,  sp, LOCAL_OFFSET(1)		/* Recover return address */
	RET(32 /*16*/)							/* REVISIT: Why 32? */

#endif

	.size	xtensa_coproc_savestate, . - xtensa_coproc_savestate

/****************************************************************************
 * Name: _xtensa_coproc_restorestate
 *
 * Description:
 *   Restore any callee-saved coprocessor state for the incoming thread.
 *   This function is called from coprocessor exception handling, when
 *   giving ownership to a thread that solicited a context switch earlier.
 *   It calls a system-specific function to get the coprocessor save area
 *   base address.
 *
 *   It is also called from xtensa_coproc_restorestate() for synchronous
 *   context switches.  xtensa_coproc_restorestate() is simply a C wrapper
 *   around the assembly language call to _xtensa_coproc_restorestate.
 *
 * Entry Conditions:
 *   - A2 holds the address of the co-processor state save area
 *   - The incoming thread is set as the current thread.
 *
 * Exit conditions:
 *   - All necessary CP callee-saved state has been restored.
 *   - CPENABLE - Set up correctly for the current thread.
 *   - Registers a2-a7, a13-a15 have been trashed.
 *
 * Must be called from assembly code only, using CALL0.
 *
 ****************************************************************************/

	.global	_xtensa_coproc_restorestate
	.type	_xtensa_coproc_restorestate, @function

	.align	4
	.literal_position
	.align	4

_xtensa_coproc_restorestate:

	/* Move the address of the thread state save area to R15 */

	mov		a15, a2							/* A15 is now the address of the save area */

#ifdef CONFIG_XTENSA_CP_LAZY
	movi	a2, 0							/* a2 = Will disable all coprocessors */
#else
	l16ui	a2, a15, XTENSA_CPENABLE		/* a2 = Which CPs have been enable for this thread? */
#endif
	wsr		a2, CPENABLE					/* Set CPENABLE correctly for this thread */
	l16ui	a2, a15, XTENSA_CPSTORED		/* a2 = Which CPs have been saved for this thread? */
	movi	a3, 0							/* Clear the ones being restored (all of them) */
	s16i	a3, a15, XTENSA_CPSTORED		/* Clear saved CP mask */

	movi	a13, _xtensa_coproc_saoffsets	/* Array of CP save offsets */
	l32i	a15, a15, XTENSA_CPASA			/* a15 = base of aligned save area */

#if XCHAL_CP0_SA_SIZE
	bbci.l	a2, 0, 2f						/* CP 0 not enabled */
	l32i	a14, a13, 0						/* a14 = _xtensa_coproc_saoffsets[0] */
	add		a3, a14, a15					/* a3 = save area for CP 0 */
	xchal_cp0_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XCHAL_CP1_SA_SIZE
	bbci.l	a2, 1, 2f						/* CP 1 not enabled */
	l32i	a14, a13, 4						/* a14 = _xtensa_coproc_saoffsets[1] */
	add		a3, a14, a15					/* a3 = save area for CP 1 */
	xchal_cp1_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XCHAL_CP2_SA_SIZE
	bbci.l	a2, 2, 2f
	l32i	a14, a13, 8
	add		a3, a14, a15
	xchal_cp2_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XCHAL_CP3_SA_SIZE
	bbci.l	a2, 3, 2f
	l32i	a14, a13, 12
	add		a3, a14, a15
	xchal_cp3_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XCHAL_CP4_SA_SIZE
	bbci.l	a2, 4, 2f
	l32i	a14, a13, 16
	add		a3, a14, a15
	xchal_cp4_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XCHAL_CP5_SA_SIZE
	bbci.l	a2, 5, 2f
	l32i	a14, a13, 20
	add		a3, a14, a15
	xchal_cp5_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XCHAL_CP6_SA_SIZE
	bbci.l	a2, 6, 2f
	l32i	a14, a13, 24
	add		a3, a14, a15
	xchal_cp6_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XCHAL_CP7_SA_SIZE
	bbci.l	a2, 7, 2f
	l32i	a14, a13, 28
	add		a3, a14, a15
	xchal_cp7_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif
	/* Ensure wsr.CPENABLE has completed. */

	rsync
	ret

	.size	_xtensa_coproc_restorestate, . - _xtensa_coproc_restorestate

/****************************************************************************
 * Name: xtensa_coproc_restorestate
 *
 * Description:
 *   Restore any callee-saved coprocessor state for the incoming thread.
 *   xtensa_coproc_restorestate() is simply a C wrapper around the assembly
 *   language call to _xtensa_coproc_restorestate.
 *
 * Input Parameters:
 *   - A2 holds the address of the threads state save area
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with interrupts disabled.
 *
 ****************************************************************************/

	.global	xtensa_coproc_restorestate
	.type	xtensa_coproc_restorestate, @function

	.align	4
	.literal_position
	.align	4

xtensa_coproc_restorestate:

#ifdef __XTENSA_CALL0_ABI__

	/* Need to preserve a8-11.  _xtensa_coproc_restorestate modifies a2-a7,
   * a13-a15. a12-a15 are callee saved registers so a13-a14 must be
   * preserved.
   */

	ENTRY(16)
	s32i	a13, sp, LOCAL_OFFSET(1)		/* Save clobbered values */
	s32i	a14, sp, LOCAL_OFFSET(2)
	s32i	a15, sp, LOCAL_OFFSET(3)

	/* Call _xtensa_coproc_restorestate() with A2=address of co-processor
   * save area.   Registers a0, a2-a7, a13-a15 have been trashed.
   */

	call0 _xtensa_coproc_restorestate

	/* Restore a13-a15 and return */

	l32i	a13, sp, LOCAL_OFFSET(1)		/* Restore clobbered registers */
	l32i	a14, sp, LOCAL_OFFSET(2)
	l32i	a15, sp, LOCAL_OFFSET(3)
	RET(16)

#else
	/* Need to preserve a8-15.  _xtensa_coproc_savestate modifies a2-a7,
   * a13-a15.  So a13-a15 may need to be preserved.
   */

	ENTRY(32 /*16*/)						/* REVISIT: Why 32? */
	s32i	a0,  sp, LOCAL_OFFSET(1)		/* Save return address */

	/* Call _xtensa_coproc_restorestate() with A2=address of co-processor
   * save area.   Registers a0, a2-a7, a13-a15 have been trashed.
   */

  call0 _xtensa_coproc_restorestate

	/* Restore a0 and return */

  l32i	a0,  sp, LOCAL_OFFSET(1)		/* Recover return address */
  RET(32 /*16*/)							/* REVISIT: Why 32? */

#endif

  .size	xtensa_coproc_restorestate, . - xtensa_coproc_restorestate

#endif /* XCHAL_CP_NUM > 0 */
