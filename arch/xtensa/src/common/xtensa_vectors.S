/****************************************************************************
 * arch/xtensa/src/common/xtensa_vectors.S
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

	.file	"xtensa_vectors.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_specregs.h>

#include "xtensa.h"
#include "xtensa_abi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _xtensa_level[n]_vector, n=2..6
 *
 * Description:
 *   Xtensa medium/nigh priority interrupt vectors.  Each vector goes at a
 *   predetermined location according to the Xtensa hardware configuration,
 *   which is ensured by its placement in a special section known to the
 *   NuttX linker script.  The vector logic performs the minimum necessary
 *   operations before jumping to the handler.
 *
 ****************************************************************************/

#if XCHAL_EXCM_LEVEL >= 2
	.begin		literal_prefix .xtensa_level2_vector
	.section	.xtensa_level2_vector.text, "ax"
	.global		_xtensa_level2_vector
	.global		_xtensa_level2_handler
	.type		_xtensa_level2_vector, @function
	.align		4

_xtensa_level2_vector:
	wsr			a0, EXCSAVE_2			/* Preserve a0 */
	call0		_xtensa_level2_handler	/* Call level 2 interrupt handling */

	/* Never returns here - call0 is used as a jump */

	.end		literal_prefix

	.size	_xtensa_level2_vector, . - _xtensa_level2_vector
#endif

#if XCHAL_EXCM_LEVEL >= 3
	.begin		literal_prefix .xtensa_level3_vector
	.section	.xtensa_level3_vector.text, "ax"
	.global		_xtensa_level3_vector
	.global		_xtensa_level3_handler
	.type		_xtensa_level3_vector, @function
	.align		4

_xtensa_level3_vector:
	wsr			a0, EXCSAVE_3			/* Preserve a0 */
	call0		_xtensa_level3_handler	/* Call level 3 interrupt handling */

	/* Never returns here - call0 is used as a jump */

	.end		literal_prefix

	.size	_xtensa_level3_vector, . - _xtensa_level3_vector
#endif

#if XCHAL_EXCM_LEVEL >= 4
	.begin		literal_prefix .xtensa_level4_vector
	.section	.xtensa_level4_vector.text, "ax"
	.global		_xtensa_level4_vector
	.global		_xtensa_level4_handler
	.type		_xtensa_level4_vector, @function
	.align		4

_xtensa_level4_vector:
	wsr			a0, EXCSAVE_4			/* Preserve a0 */
	call0		_xtensa_level4_handler	/* Call level 5 interrupt handling */

	/* Never returns here - call0 is used as a jump */

	.end		literal_prefix

	.size	_xtensa_level5_vector, . - _xtensa_level5_vector
#endif

#if XCHAL_EXCM_LEVEL >= 5
	.begin		literal_prefix .xtensa_level5_vector
	.section	.xtensa_level5_vector.text, "ax"
	.global		_xtensa_level5_vector
	.global		_xtensa_level5_handler
	.type		_xtensa_level5_vector, @function
	.align		4

_xtensa_level5_vector:
	wsr			a0, EXCSAVE_5			/* Preserve a0 */
	call0		_xtensa_level5_handler	/* Call level 5 interrupt handling */

	/* Never returns here - call0 is used as a jump */

	.size		_xtensa_level5_vector, . - _xtensa_level5_vector
	.end		literal_prefix
#endif

#if XCHAL_EXCM_LEVEL >= 6
	.begin		literal_prefix .xtensa_level6_vector
	.section	.xtensa_level6_vector.text, "ax"
	.global		_xtensa_level6_vector
	.global		_xtensa_level6_handler
	.type		_xtensa_level6_vector, @function
	.align		4

_xtensa_level6_vector:
	wsr			a0, EXCSAVE_6			/* Preserve a0 */
	call0		_xtensa_level6_handler	/* Call level 6 interrupt handling */

	/* Never returns here - call0 is used as a jump */

	.size		_xtensa_level6_vector, . - _xtensa_level6_vector
	.end		literal_prefix
#endif

/****************************************************************************
 * Exception Vectors (except User, Co-processor and window exception
 * vectors).
 *
 * Each vector goes at a predetermined location according to the Xtensa
 * hardware configuration, which is ensured by its placement in a special
 * section known to the Xtensa linker support package (LSP). It performs
 * the minimum necessary before jumping to the handler in the .text section.
 *
 * The corresponding handler goes in the normal .text section. It sets up
 * the appropriate stack frame, saves a few vector-specific registers and
 * calls _xtensa_panic() to save the rest of the interrupted context
 * and enter the NuttX panic handler
 *
 ****************************************************************************/

/****************************************************************************
 * Name: _xtensa_nmi_vector
 *
 * Description:
 *   NMI Exception
 *
 ****************************************************************************/

#if XCHAL_HAVE_NMI
	.begin		literal_prefix .nmi_vector
	.section	.nmi_vector.text, "ax"
	.global		_xtensa_nmi_vector
	.type		_xtensa_nmi_vector, @function
	.align		4

_xtensa_nmi_vector:

#if 1
	/* For now, just panic */

	wsr		a0, EXCSAVE + XCHAL_NMILEVEL	/* Preserve a0 */

	mov		a0, sp							/* sp == a1 */
	addi	sp, sp, -(4 * XCPTCONTEXT_SIZE)	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS + XCHAL_NMILEVEL		/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_2						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE + XCHAL_NMILEVEL	/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)

	s32i	a2, sp, (4 * REG_A2)
	movi	a2, XTENSA_NMI_EXCEPTION		/* Argument 1: Error code */
	call0	_xtensa_panic					/* Does not return */

#else
	/* Add high priority non-maskable interrupt (NMI) handler code here. */

	rfi		XCHAL_NMILEVEL

#endif

	.size		_xtensa_nmi_vector, . - _xtensa_nmi_vector
	.end		literal_prefix

#endif /* XCHAL_HAVE_NMI */

/****************************************************************************
 * Name: _debug_exception_vector
 *
 * Description:
 *   Debug exception vector
 *
 ****************************************************************************/

#if XCHAL_HAVE_DEBUG
	.begin		literal_prefix .debug_exception_vector
	.section	.debug_exception_vector.text, "ax"
	.global		_debug_exception_vector
	.align		4

_debug_exception_vector:

	mov		a0, sp							/* sp == a1 */
	addi	sp, sp, -(4 * XCPTCONTEXT_SIZE)	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS + XCHAL_DEBUGLEVEL		/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC + XCHAL_DEBUGLEVEL		/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE + XCHAL_DEBUGLEVEL	/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)

	s32i	a2, sp, (4 * REG_A2)
	movi	a2, XTENSA_DEBUG_EXCEPTION		/* Argument 1: Error code */
	call0	_xtensa_panic					/* Does not return */

	.end	literal_prefix

#endif /* XCHAL_HAVE_DEBUG */

/****************************************************************************
 * Name: _double_exception_vector
 *
 * Description:
 *   Double Exception Vector. Double exceptions are not a normal occurrence.
 *   They indicate a bug of some kind.
 *
 ****************************************************************************/

#ifdef XCHAL_DOUBLEEXC_VECTOR_VADDR
	.begin		literal_prefix .double_exception_vector
	.section	.double_exception_vector.text, "ax"
	.global		_double_exception_vector
	.align		4

_double_exception_vector:

#if XCHAL_HAVE_DEBUG
	break	1, 4							/* Unhandled double exception */
#endif

	mov		a0, sp							/* sp == a1 */
	addi	sp, sp, -(4 * XCPTCONTEXT_SIZE)	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS							/* Save interruptee's PS -- REVISIT */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, DEPC						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE						/* Save interruptee's a0 -- REVISIT */
	s32i	a0, sp, (4 * REG_A0)

	rsr		a0, EXCCAUSE					/* Save the EXCCAUSE register */
	s32i	a0, sp, (4 * REG_EXCCAUSE)
	rsr		a0, EXCVADDR					/* Save the EXCVADDR register */
	s32i	a0, sp, (4 * REG_EXCVADDR)

	s32i	a2, sp, (4 * REG_A2)
	movi	a2, XTENSA_DOUBLE_EXCEPTION		/* Argument 1: Error code */
	call0	_xtensa_panic					/* Does not return */

	.end	literal_prefix

#endif /* XCHAL_DOUBLEEXC_VECTOR_VADDR */

/****************************************************************************
 * Name: _kernel_exception_vector
 *
 * Description:
 *   Kernel Exception (including Level 1 Interrupt from kernel mode).
 *
 ****************************************************************************/

	.begin		literal_prefix .kernel_exception_vector
	.section	.kernel_exception_vector.text, "ax"
	.global		_kernel_exception_vector
	.align		4

_kernel_exception_vector:

#if XCHAL_HAVE_DEBUG
	break	1, 0							/* Unhandled kernel exception */
#endif

	mov		a0, sp							/* sp == a1 */
	addi	sp, sp, -(4 * XCPTCONTEXT_SIZE)	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS							/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_1						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_1					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)

	s32i	a2, sp, (4 * REG_A2)
	movi	a2, XTENSA_KERNEL_EXCEPTION		/* Argument 1: Error code */
	call0	_xtensa_panic					/* Does not return */

	.end	literal_prefix

/****************************************************************************
 * Name: _user_exception_vector
 *
 * Description:
 *   User Exception (including Level 1 Interrupt from user mode).
 *
 ****************************************************************************/

	.begin		literal_prefix .user_exception_vector
	.section	.user_exception_vector.text, "ax"
	.global		_user_exception_vector
	.type		_user_exception_vector, @function
	.align		4

_user_exception_vector:

	wsr		a0, EXCSAVE_1					/* Preserve a0 */
	call0	_xtensa_user_handler			/* And jump to user exception handler */

	.end	literal_prefix
