/****************************************************************************
 * arch/xtensa/src/common/xtensa_windowspill.S
 * Register window spill routine
 *
 * Adapted from use in NuttX by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from logic originally provided by Tensilica Inc.
 *
 *   $Id: //depot/rel/Eaglenest/Xtensa/OS/hal/windowspill_asm.S#1 $
 *   Copyright (c) 1999-2010 Tensilica Inc.
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

	.file   "xtensa_windowspill.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/esp32/core-isa.h>
#include <arch/xtensa/xtensa_specregs.h>
#include "xtensa_abi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _xtensa_window_spill
 *
 * Description:
 *   Spill live register windows to the stack.
 *
 *   All non-spilled register windows will be spilled.  Beware that this may
 *   include a4..a15 of the current window, so generally these should not
 *   have been clobbered by the caller if it is at all possible that these
 *   registers are part of an unspilled window (it often is possible)
 *   (otherwise the spilled stack would be invalid).
 *
 *   THIS MEANS: the caller is responsible for saving a0-a15 but the caller
 *   must leave a4-a15 intact when control is transferred here.
 *
 *   It may be reentrant (but stack pointer is invalid during execution due
 *   to window rotations, so can't take interrupts and exceptions in the
 *   usual manner, so ... what does reentrancy really mean here?).
 *
 * Required entry conditions:
 *   PS.WOE = 0
 *   PS.INTLEVEL >= XCHAL_EXCM_LEVEL
 *   a1 = valid stack pointer (note: some regs may be spilled at a1-16)
 *   a0 = return PC (usually set by call0 or callx0 when calling this function)
 *   a2,a3 undefined
 *   a4 thru a15 valid, if they are part of window(s) to be spilled
 *     (Current window a0..a15 saved if necessary.)
 *   WINDOWSTART[WINDOWBASE] = 1
 *
 * Exit conditions:
 *   PS.WOE, PS.INTLEVEL = same as on entry
 *   WINDOWBASE = same as on entry
 *   WINDOWSTART updated to reflect spilled windows
 *     (equals 1<<WINDOWBASE if successful return)
 *   a0 = return PC
 *   a1 = same as on entry
 *   a2 = error code:
 *        0 --> successful
 *              (WINDOWSTART = 1<<WINDOWBASE)
 *        1 --> invalid WINDOWSTART (WINDOWBASE bit not set)
 *              (WINDOWSTART unchanged)
 *        2 --> invalid window size (not 4, 8 or 12 regs)
 *              (WINDOWSTART bits of successfully spilled
 *              windows are cleared, others left intact)
 *   a3 clobbered
 *   a4,a5,a8,a9,a12,a13 = same as on entry
 *   a6,a7,a10,a11,a14,a15 clobbered if they were part of window(s)
 *     to be spilled, otherwise they are the same as on entry
 *   loop registers (LCOUNT,LBEG,LEND) are NOT affected (they were in
 *     earlier versions)
 *   SAR clobbered
 *
 ****************************************************************************/

	.text
	.align	4
	.global	_xtensa_window_spill

_xtensa_window_spill:

#ifndef XCHAL_HAVE_WINDOWED
	/* Nothing to do -- window option was not selected. */

	movi	a2, 0						/* Always report success */
	ret

#else /* XCHAL_HAVE_WINDOWED */
#  define WSBITS  (XCHAL_NUM_AREGS / 4)			/* Width of WINDOWSTART register in bits */
#  define WBBITS  (XCHAL_NUM_AREGS_LOG2 - 2)	/* Width of WINDOWBASE register in bits */

	/*
	 * Rearrange (rotate) window start bits relative to the current
	 * window (WINDOWBASE).  WINDOWSTART currently looks like this:
	 *
	 *          a15-a0
	 * NAREG-1   |  |    0
	 *    |      vvvv    |
	 *    xxxxxxxxxx1yyyyy
	 *              ^
	 *              |
	 *              WINDOWBASE
	 *
	 * The start bit pointed to by WINDOWBASE must be set
	 * (we return an error if it isn't), as it corresponds
	 * to the start of the current window (shown as a0-a15).
	 *
	 * We want the window start bits rotated to look like this:
	 *              1yyyyyxxxxxxxxxx
	 *
	 * Note that there is one start bit for every four registers;
	 * and the total number of registers (NAREG) can be 32 or 64;
	 * so the number of start bits in WINDOWSTART is NAREG/4,
	 * and the size of WINDOWSTART can be 8 or 16.
	 */

	rsr		a2, WINDOWBASE
	addi	a2, a2, 1
	ssr		a2						/* sar = WINDOWBASE + 1 */
	rsr		a3, WINDOWSTART
	srl		a2, a3					/* a2 is 0... | 000000xxxxxxxxxx = WINDOWSTART >> sar */
	sll		a3, a3					/* a3 is 1yyyyy0000000000 | 0... = WINDOWSTART << (32 - sar) */
	bgez	a3, .Linvalid_ws		/* verify that msbit is indeed set */

	srli	a3, a3, 32-WSBITS		/* a3 is 0... | 1yyyyy0000000000 = a3 >> (32-NAREG/4) */
	or		a2, a2, a3				/* a2 is 0... | 1yyyyyxxxxxxxxxx */

	/* FIND THE FIRST ONE
	 *
	 * Now we have (in a2) the window start bits rotated in order
	 * from oldest (closest to lsbit) to current (msbit set).
	 * Each start bit (that is set), other than the current one,
	 * corresponds to a window frame to spill.
	 *
	 * Now find the first start bit, ie. the first frame to spill,
	 * by looking for the first bit set in a2 (from lsbit side).
	 */

#if XCHAL_HAVE_NSA
	neg		a3, a2					/* Keep only the least-significant bit set of a2 ... */
	and		a3, a3, a2				/* ... in a3 */
	nsau	a3, a3					/* Get index of that bit, numbered from msbit (32 if absent) */
	ssl		a3						/* Set sar = 32 - a3 = bit index numbered from lsbit + 1 */
#else /* XCHAL_HAVE_NSA */
	wsr		a2, WINDOWSTART			/* temporarily save rotated start bits
									 * (we can use WINDOWSTART because WOE=0) */

	/* NOTE:  this could be optimized a bit, by explicit coding rather than the macro.
	 */

	find_ls_one	a3, a2				/* Set a3 to index of lsmost bit set in a2 (a2 clobbered) */

	addi	a2, a3, 1				/* Index+1 */
	ssr		a2						/* Set sar = index + 1 */
	rsr		a2, WINDOWSTART			/* Restore a2 (rotated start bits) */
#endif /* XCHAL_HAVE_NSA */

	srl		a2, a2					/* Right-justify the rotated start bits (dropping lsbit set) */
	wsr		a2, WINDOWSTART			/* Save rotated + justified window start bits,
									 * because a2 will disappear when modifying WINDOWBASE
									 * again, we can use WINDOWSTART because WOE=0 */

	/* Rotate WindowBase so that a0 of the next window to spill is in a4
	 * (ie. leaving us with a2 and a3 to play with, because a0 and a1
	 * may be those of the original window which we must preserve).
	 */

	rsr	a2, WINDOWBASE

#if XCHAL_HAVE_NSA
	addi	a2, a2, 31
	sub		a3, a2, a3				/* a3 = WINDOWBASE + index = WINDOWBASE + (31 - msbit_index) */
#else /* XCHAL_HAVE_NSA */
	add		a3, a2, a3				/* a3 = WINDOWBASE + index */
#endif /* XCHAL_HAVE_NSA */

	wsr		a3, WINDOWBASE			/* Effectively do:  rotw index */
	rsync							/* Wait for write to WINDOWBASE to complete */

	/* Now our registers have changed! */

	rsr		a2, WINDOWSTART			/* Restore a2 (rotated + justified window start bits) */

	/* We are now ready to start the window spill loop.
	 * Relative to the above, a2 and WINDOWBASE are now as follows:
	 *
	 *        1yyyyyxxxxxxxxxx = rotated start bits as shown above
	 *        1yyyyyxxxx100000 = actual rotated start bits (example)
	 *  0000001yyyyyxxxx ^     = a2 = rotated + justified start bits
	 *        ^      xxx1^     = window being spilled
	 *        ^          ^
	 *        |          |
	 *    original    current
	 *   WINDOWBASE  WINDOWBASE
	 *
	 * The first window to spill (save) starts at what is now a4.
	 * The spill loop maintains the adjusted start bits in a2,
	 * shifting them right as each window is spilled.
	 */

.Lspill_loop:
	/* Top of save loop. */
	/* Find the size of this call and branch to the appropriate save routine. */

	beqz	a2, .Ldone				/* If no start bit remaining, we're done */
	bbsi.l	a2, 0, .Lspill4			/* If next start bit is set, it's a call4 */
	bbsi.l	a2, 1, .Lspill8			/* If 2nd next bit set, it's a call8 */
	bbsi.l	a2, 2, .Lspill12		/* If 3rd next bit set, it's a call12 */
	j		.Linvalid_window		/* Else it's an invalid window! */

	/* SAVE A CALL4 */

.Lspill4:
	addi	a3, a9, -16				/* a3 gets call[i+1]'s sp - 16 */
	s32i	a4, a3, 0				/* Store call[i]'s a0 */
	s32i	a5, a3, 4				/* Store call[i]'s a1 */
	s32i	a6, a3, 8				/* Store call[i]'s a2 */
	s32i	a7, a3, 12				/* Store call[i]'s a3 */

	srli	a6, a2, 1				/* Move and shift the start bits */
	rotw	1						/* Rotate the window */

	j		.Lspill_loop

	/* SAVE A CALL8 */

.Lspill8:
	addi	a3, a13, -16			/* a0 gets call[i+1]'s sp - 16 */
	s32i	a4, a3, 0				/* Store call[i]'s a0 */
	s32i	a5, a3, 4				/* Store call[i]'s a1 */
	s32i	a6, a3, 8				/* Store call[i]'s a2 */
	s32i	a7, a3, 12				/* Store call[i]'s a3 */

	addi	a3, a5, -12				/* Call[i-1]'s sp address */
	l32i	a3, a3, 0				/* a3 is call[i-1]'s sp
									 * (load slot) */
	addi	a3, a3, -32				/* a3 points to our spill area */

	s32i	a8, a3, 0				/* Store call[i]'s a4 */
	s32i	a9, a3, 4				/* Store call[i]'s a5 */
	s32i	a10, a3, 8				/* Store call[i]'s a6 */
	s32i	a11, a3, 12				/* Store call[i]'s a7 */

	srli	a10, a2, 2				/* Move and shift the start bits */
	rotw	2						/* Rotate the window */

	j		.Lspill_loop

	/* SAVE A CALL12 */

.Lspill12:
	rotw	1						/* Rotate to see call[i+1]'s sp */

	addi	a13, a13, -16			/* Set to the reg save area */
	s32i	a0, a13, 0				/* Store call[i]'s a0 */
	s32i	a1, a13, 4				/* Store call[i]'s a1 */
	s32i	a2, a13, 8				/* Store call[i]'s a2 */
	s32i	a3, a13, 12				/* Store call[i]'s a3 */

	addi	a3, a1, -12				/* Call[i-1]'s sp address */
	l32i	a3, a3, 0				/* a3 has call[i-1]'s sp */
	addi	a13, a13, 16			/* Restore call[i+1]'s sp (here to fill load slot) */
	addi	a3, a3, -48				/* a3 points to our save area */

	s32i	a4, a3, 0				/* Store call[i]'s a4 */
	s32i	a5, a3, 4				/* Store call[i]'s a5 */
	s32i	a6, a3, 8				/* Store call[i]'s a6 */
	s32i	a7, a3, 12				/* Store call[i]'s a7 */
	s32i	a8, a3, 16				/* Store call[i]'s a4 */
	s32i	a9, a3, 20				/* Store call[i]'s a5 */
	s32i	a10, a3, 24				/* Store call[i]'s a6 */
	s32i	a11, a3, 28				/* Store call[i]'s a7 */

	rotw	-1						/* Rotate to see start bits (a2) */
	srli	a14, a2, 3				/* Move and shift the start bits */
	rotw	3						/* Rotate to next window */

	j		.Lspill_loop

.Ldone:
	rotw	1						/* Back to the original window */
	rsr		a2, WINDOWBASE			/* Get (original) window base */
	ssl		a2						/* Setup for shift left by WINDOWBASE */
	movi	a2, 1
	sll		a2, a2					/* Compute new WINDOWSTART = 1<<WINDOWBASE */
	wsr		a2, WINDOWSTART			/* And apply it */
	rsync
	movi	a2, 0					/* Done! */
	ret
	/* jx	a0 */

	/* Invalid WINDOWSTART register. */

.Linvalid_ws:
	movi	a2, 1					/* Indicate invalid WINDOWSTART */
	ret								/* Return from subroutine */

	/* Invalid window size!
	 * The three bits following the start bit are all clear, so
	 * we have an invalid window state (can't determine a window size).
	 *
	 * So we exit with an error, but to do that we must first restore
	 * the original WINDOWBASE.  We also compute a sensible
	 * WINDOWSTART that has the start bits of spilled windows
	 * cleared, but all other start bits intact, so someone debugging
	 * the failure can look at WINDOWSTART to see which window
	 * failed to spill.
	 */

.Linvalid_window:
	slli	a2, a2, 1				/* Space for missing start bit */
	addi	a2, a2, 1				/* Add missing start bit */
	rsr		a3, WINDOWBASE			/* Get current WINDOWBASE */
	bbsi.l	a2, WSBITS-1, 2f		/* Branch if current WINDOWBASE==original */
1:	addi	a3, a3, -1				/* Decrement towards original WINDOWBASE */
	slli	a2, a2, 1				/* Shift towards original WINDOWSTART alignment */
	bbci.l	a2, WSBITS-1, 1b		/* Repeat until ms start bit set */
	extui	a3, a3, 0, WBBITS		/* Mask out upper base bits, in case of carry-over */

	/* Here, a3 = original WINDOWBASE;
	 * and msbit of start bits in a2 is set, and no other bits above it.
	 * Now rotate a2 to become the correct WINDOWSTART.
	 */

2:
	ssl		a3						/* Set shift left ... (sar = 32 - orig WB) */
	slli	a3, a2, 32-WSBITS		/* Left-justify start bits */
	src		a2, a2, a3				/* Rotate left by original WINDOWBASE */
	extui	a2, a2, 0, WSBITS		/* Keep only significant start bits */
	wsr		a2, WINDOWSTART			/* We've cleared only start bits of spilled windows */
	rsr		a3, SAR					/* Retrieve 32 - original WINDOWBASE */
	movi	a2, 32
	sub		a3, a2, a3				/* Restore original WINDOWBASE */
	wsr		a3, WINDOWBASE			/* Back to original WINDOWBASE */
	rsync

	movi	a2, 2					/* Indicate invalid window size */
	ret

#endif /* XCHAL_HAVE_WINDOWED */

	.size	_xtensa_window_spill, . - _xtensa_window_spill

/****************************************************************************
 * Name: xtensa_window_spill
 *
 * Description:
 *   Spill live register windows to the stack.
 *
 *   This will spill all register windows except this function's window, and
 *   possibly that of its caller (Currently, the caller's window is spilled
 *   and reloaded when this function returns.  This may change with future
 *   optimisations.)
 *
 *   Another, simpler way to implement this might be to use an appropriate
 *   sequence of call/entry/retw instructions to force overflow of any live
 *   windows.
 *
 *   Assumes that PS.INTLEVEL=0 and PS.WOE=1 on entry/exit.
 *
 *   C callable as:
 *     void  xtensa_window_spill (void);
 *
 ****************************************************************************/

#if 0 /* Not used */
	.text
	.global	xtensa_window_spill
	.type	xtensa_window_spill, @function
	.align	4

xtensa_window_spill:

	ENTRY(16)

#if XCHAL_HAVE_WINDOWED
	movi	a6, ~(PS_WOE_MASK|PS_INTLEVEL_MASK)	/* (using a6 ensures any window
												 * using this a4..a7 is spilled) */
	rsr		a5, PS
	mov		a4, a0					/* Save a0 */
	and		a2, a5, a6				/* Clear WOE, INTLEVEL */
	addi	a2, a2, XCHAL_EXCM_LEVEL /* Set INTLEVEL = XCHAL_EXCM_LEVEL */
	wsr		a2, PS					/* Apply to PS */
	rsync
	call0	_xtensa_window_spill
	mov		a0, a4					/* Restore a0 */
	wsr		a5, PS					/* Restore PS */
	rsync
#endif /* XCHAL_HAVE_WINDOWED */

	RET(16)

	.size	xtensa_window_spill, . - xtensa_window_spill
#endif
