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

#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_specregs.h>

#if XCHAL_CP_NUM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: xtensa_coproc_init
 *
 * Description:
 *   Initializes.global	co-processor management data, setting all co-
 *   processors to "unowned".  Leaves CPENABLE as it found it (does NOT clear
 *   it).
 *
 *   Called during initialization of the NuttX, before any threads run.
 *
 *   This may be called from normal Xtensa single-threaded application code
 *   which might use co-processors. The Xtensa run-time initialization
 *   enables all co-processors. They must remain enabled here, else a co-
 *   processor exception might occur outside of a thread, which the exception
 *   handler doesn't expect.
 *
 * Entry Conditions:
 *   Xtensa single-threaded run-time environment is in effect.
 *   No thread is yet running.
 *
 * Exit conditions:
 *   None.
 *
 * Obeys ABI conventions per prototype:
 *    void xtensa_coproc_init(void)
 *
 ****************************************************************************/

	.global	xtensa_coproc_init
	.type	xtensa_coproc_init, @function

	.align	4
	.literal_position
	.align	4

xtensa_coproc_init:

	/* Initialize thread co-processor ownerships to 0 (unowned). */

	movi	a2, _xt_coproc_owner_sa			/* a2 = base of owner array */
	addi	a3, a2, (XTENSA_CP_MAX*portNUM_PROCESSORS) << 2		/* a3 = top+1 of owner array */
	movi	a4, 0							/* a4 = 0 (unowned) */
1:	s32i	a4, a2, 0
	addi	a2, a2, 4
	bltu	a2, a3, 1b

	ret

	.size	xtensa_coproc_init, . - xtensa_coproc_init

/****************************************************************************
 * Name: xtensa_coproc_release
 *
 * Description:
 *   Releases any and all co-processors owned by a given thread. The thread
 *   is identified by it's co-processor state save area defined in
 *   xtensa_context.h .
 *
 *   Must be called before a thread's co-proc save area is deleted to avoid
 *   memory corruption when the exception handler tries to save the state.
 *   May be called when a thread terminates or completes but does not delete
 *   the co-proc save area, to avoid the exception handler having to save the
 *   thread's co-proc state before another thread can use it (optimization).
 *
 * Entry Conditions:
 *   A2 = Pointer to base of co-processor state save area.
 *
 * Exit conditions:
 *   None.
 *
 * Obeys ABI conventions per prototype:
 *   void xtensa_coproc_release(void * coproc_sa_base)
 *
 ****************************************************************************/

#if 0 /* Not used in current design */
	.global	xtensa_coproc_release
	.type	xtensa_coproc_release, @function

	.align	4
	.literal_position
	.align	4

xtensa_coproc_release:
	getcoreid a5
	movi	a3, XTENSA_CP_MAX << 2
	mull	a5, a5, a3
	movi	a3, _xt_coproc_owner_sa			/* a3 = base of owner array */
	add		a3, a3, a5

	addi	a4, a3, XTENSA_CP_MAX << 2		/* a4 = top+1 of owner array */
	movi	a5, 0							/* a5 = 0 (unowned) */

	rsil	a6, XCHAL_EXCM_LEVEL			/* Lock interrupts */

1:  l32i	a7, a3, 0						/* a7 = owner at a3 */
	bne		a2, a7, 2f						/* if (coproc_sa_base == owner) */
	s32i	a5, a3, 0						/*   owner = unowned */
2:  addi	a3, a3, 1<<2					/* a3 = next entry in owner array */
	bltu	a3, a4, 1b						/* repeat until end of array */

3:  wsr		a6, PS							/* Restore interrupts */

	ret

	.size	xtensa_coproc_release, . - xtensa_coproc_release
#endif

/****************************************************************************
 * Name: _xtensa_coproc_savestate
 *
 * Description:
 *   If there is a current thread and it has a coprocessor state save area,
 *   then save all callee-saved state into this area. This function is
 *   called from the solicited context switch handler. It calls a system-
 *   specific function to get the coprocessor save area base address.
 *
 * Entry Conditions:
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

	.extern	_xt_coproc_sa_offset		/* External reference */

	.global	_xtensa_coproc_savestate
	.type	_xtensa_coproc_savestate, @function

	.align	4
	.literal_position
	.align	4

_xtensa_coproc_savestate:

	/* At entry, CPENABLE should be showing which CPs are enabled. */

	rsr		a2, CPENABLE				/* a2 = which CPs are enabled */
	beqz	a2, .Ldone					/* Quick exit if none */
	mov		a14, a0						/* Save return address */
	call0	XT_RTOS_CP_STATE			/* Get address of CP save area */
	mov		a0, a14						/* Restore return address */
	beqz	a15, .Ldone					/* if none then nothing to do */
	s16i	a2, a15, XT_CP_CS_ST		/* Save mask of CPs being stored */
	movi	a13, _xt_coproc_sa_offset   /* Array of CP save offsets */
	l32i	a15, a15, XT_CP_ASA			/* a15 = base of aligned save area */

#if XTENSA_CP0_SA_SIZE > 0
	bbci.l	a2, 0, 2f					/* CP 0 not enabled */
	l32i	a14, a13, 0					/* a14 = _xt_coproc_sa_offset[0] */
	add		a3, a14, a15				/* a3 = save area for CP 0 */
	xchal_cp0_store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XTENSA_CP1_SA_SIZE > 0
	bbci.l	a2, 1, 2f					/* CP 1 not enabled */
	l32i	a14, a13, 4					/* a14 = _xt_coproc_sa_offset[1] */
	add		a3, a14, a15				/* a3 = save area for CP 1 */
	xchal_cp1_store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XTENSA_CP2_SA_SIZE > 0
	bbci.l	a2, 2, 2f
	l32i	a14, a13, 8
	add		a3, a14, a15
	xchal_cp2_store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XTENSA_CP3_SA_SIZE > 0
	bbci.l	a2, 3, 2f
	l32i	a14, a13, 12
	add		a3, a14, a15
	xchal_cp3store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XTENSA_CP4_SA_SIZE > 0
	bbci.l	a2, 4, 2f
	l32i	a14, a13, 16
	add		a3, a14, a15
	xchal_cp4store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XTENSA_CP5_SA_SIZE > 0
	bbci.l	a2, 5, 2f
	l32i	a14, a13, 20
	add		a3, a14, a15
	xchal_cp5store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XTENSA_CP6_SA_SIZE > 0
	bbci.l	a2, 6, 2f
	l32i	a14, a13, 24
	add		a3, a14, a15
	xchal_cp6store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XTENSA_CP7_SA_SIZE > 0
	bbci.l	a2, 7, 2f
	l32i	a14, a13, 28
	add		a3, a14, a15
	xchal_cp7store	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

.Ldone:
	ret

	.size	_xtensa_coproc_savestate, . - _xtensa_coproc_savestate

/****************************************************************************
 * Name: _xtensa_coproc_restorestate
 *
 * Description:
 *   Restore any callee-saved coprocessor state for the incoming thread.
 *   This function is called from coprocessor exception handling, when
 *   giving ownership to a thread that solicited a context switch earlier.
 *   It calls a system-specific function to get the coprocessor save area base address.
 *
 * Entry Conditions:
 *   - The incoming thread is set as the current thread.
 *   - CPENABLE is set up correctly for all required coprocessors.
 *   - a2 = mask of coprocessors to be restored.
 *
 * Exit conditions:
 *   - All necessary CP callee-saved state has been restored.
 *   - CPENABLE - unchanged.
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

	mov		a14, a0						/* Save return address */
	call0	XT_RTOS_CP_STATE			/* Get address of CP save area */
	mov		a0, a14						/* Restore return address */
	beqz	a15, .Ldone2				/* if none then nothing to do */
	l16ui	a3, a15, XT_CP_CS_ST		/* a3 = which CPs have been saved */
	xor		a3, a3, a2					/* Clear the ones being restored */
	s32i	a3, a15, XT_CP_CS_ST		/* Update saved CP mask */
	movi	a13, _xt_coproc_sa_offset   /* Array of CP save offsets */
	l32i	a15, a15, XT_CP_ASA			/* a15 = base of aligned save area */

#if XTENSA_CP0_SA_SIZE
	bbci.l	a2, 0, 2f					/* CP 0 not enabled */
	l32i	a14, a13, 0					/* a14 = _xt_coproc_sa_offset[0] */
	add		a3, a14, a15				/* a3 = save area for CP 0 */
	xchal_cp0_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XTENSA_CP1_SA_SIZE
	bbci.l	a2, 1, 2f					/* CP 1 not enabled */
	l32i	a14, a13, 4					/* a14 = _xt_coproc_sa_offset[1] */
	add		a3, a14, a15				/* a3 = save area for CP 1 */
	xchal_cp1_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XTENSA_CP2_SA_SIZE
	bbci.l	a2, 2, 2f
	l32i	a14, a13, 8
	add		a3, a14, a15
	xchal_cp2_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XTENSA_CP3_SA_SIZE
	bbci.l	a2, 3, 2f
	l32i	a14, a13, 12
	add		a3, a14, a15
	xchal_cp3_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XTENSA_CP4_SA_SIZE
	bbci.l	a2, 4, 2f
	l32i	a14, a13, 16
	add		a3, a14, a15
	xchal_cp4_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XTENSA_CP5_SA_SIZE
	bbci.l	a2, 5, 2f
	l32i	a14, a13, 20
	add		a3, a14, a15
	xchal_cp5_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XTENSA_CP6_SA_SIZE
	bbci.l	a2, 6, 2f
	l32i	a14, a13, 24
	add		a3, a14, a15
	xchal_cp6_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

#if XTENSA_CP7_SA_SIZE
	bbci.l	a2, 7, 2f
	l32i	a14, a13, 28
	add		a3, a14, a15
	xchal_cp7_load	a3, a4, a5, a6, a7 continue=0 ofs=-1 select=XTHAL_SAS_TIE|XTHAL_SAS_NOCC|XTHAL_SAS_CALE alloc=XTHAL_SAS_ALL
2:
#endif

.Ldone2:
	ret

	.size	_xtensa_coproc_restorestate, . - _xtensa_coproc_restorestate

#endif /* XCHAL_CP_NUM > 0 */
