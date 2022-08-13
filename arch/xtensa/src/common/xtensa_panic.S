/****************************************************************************
 * arch/xtensa/src/common/xtensa_panic.S
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

	.file	"xtensa_panic.S"

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
#include <arch/xtensa/xtensa_abi.h>
#include <arch/xtensa/xtensa_specregs.h>

#include "xtensa_macros.S"

#include "chip.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _xtensa_panic
 *
 * Description:
 *   Should be reached by call0 (preferable) or jump only. If call0, a0 says
 *   where from. If on simulator, display panic message and abort, else loop
 *   indefinitely.
 *
 * Entry Conditions:
 *   - A1 = Stack frame already allocated.  SP points to beginning of the
 *          register frame.
 *   - A0, A1, A2, PC and PS  = Already saved in the stack frame
 *   - A2  = Exception code
 *
 * Exit conditions:
 *   Does not return.
 *
 ****************************************************************************/

	.section HANDLER_SECTION, "ax"
	.global	_xtensa_panic
	.type	_xtensa_panic, @function

	.align	4
	.literal_position
	.align	4

_xtensa_panic:
	/* Save the exception code */

	wsr		a2, EXCSAVE_1

	/* Save rest of interrupt context (A2=address of state save area on
	 * stack.
	 */

	call0	_xtensa_context_save			/* Save full register state */

	/* Save exception cause and vaddr into the user frame */

	rsr		a0, EXCCAUSE
	s32i	a0, sp, (4 * REG_EXCCAUSE)
	rsr		a0, EXCVADDR
	s32i	a0, sp, (4 * REG_EXCVADDR)

	/* Dispatch the sycall as with other interrupts. */

	mov		a12, sp							/* a12 = address of register save area */

	/* Switch to an interrupt stack if we have one */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
	setintstack a13 a14
#endif

	/* Set up PS for C, re-enable hi-pri interrupts, and clear EXCM. */

	ps_setup XCHAL_EXCM_LEVEL a0

	/* Call C panic handler: 
	 *  Arg1 = Exception code.
	 *  Arg2 = Start of the register save area.
	 */

	rsr		ARG1, EXCSAVE_1
	mov		ARG2, a12
	CALL	xtensa_panic					/* Call xtensa_panic. Should not return */

1:	j		1b								/* loop infinitely */
	retw
