/****************************************************************************
 * arch/xtensa/src/common/xtensa_nmi_handler.S
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

	.file	"xtensa_nmi_handler.S"

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
#include <arch/xtensa/xtensa_specregs.h>

#include "chip_macros.h"
#include "xtensa_abi.h"

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * HIGH PRIORITY NMI LOW-LEVEL HANDLER
 *
 * High priority interrupts are by definition those with priorities greater
 * than XCHAL_EXCM_LEVEL.  This includes non-maskable (NMI).  High priority
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
 * The NMI vector goes at a predetermined location according to the Xtensa
 * hardware configuration, which is ensured by its placement in a special
 * section known to the Xtensa linker support package (LSP).  It performs
 * the minimum necessary before jumping to the NMI andler.
 *
 * Below is a template for the high priority NMI interrupt handler.
 * A template and example can be found in the Cadence Design Systems tools
 * documentation: "Microprocessor Programmer's Guide".
 *
 ****************************************************************************/

#if XCHAL_HAVE_NMI
	.section	HANDLER_SECTION, "ax"
	.type		_xt_nmi, @function
	.align		4
_xt_nmi:

#ifdef XT_INTEXC_HOOKS
	/* Call interrupt hook if present to (pre)handle interrupts. */

	movi	a0, _xt_intexc_hooks
	l32i	a0, a0, XCHAL_NMILEVEL << 2
	beqz	a0, 1f
	callx0	a0                              /* Must NOT disturb stack! */
1:
#endif

	/* Add high priority non-maskable interrupt (NMI) handler code here. */

	rsr		a0, EXCSAVE + XCHAL_NMILEVEL    /* Restore a0 */
	rfi		XCHAL_NMILEVEL

#endif /* XCHAL_HAVE_NMI */
