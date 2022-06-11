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

#include <arch/irq.h>
#include <arch/xtensa/core.h>
#include <arch/xtensa/xtensa_abi.h>
#include <arch/xtensa/xtensa_coproc.h>
#include <arch/xtensa/xtensa_specregs.h>
#include <arch/chip/core-isa.h>
#include <arch/chip/tie.h>
#include <arch/chip/tie-asm.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

  .section .rodata, "a"

  /* Offset to CP n save area in thread's CP save area. */

  .global	_xtensa_coproc_saoffsets
  .type		_xtensa_coproc_saoffsets, @object
  .align	16                      /* Minimize crossing cache boundaries */

_xtensa_coproc_saoffsets:

  .word	XTENSA_CP0_SA, XTENSA_CP1_SA, XTENSA_CP2_SA, XTENSA_CP3_SA
  .word	XTENSA_CP4_SA, XTENSA_CP5_SA, XTENSA_CP6_SA, XTENSA_CP7_SA

  .size	_xtensa_coproc_saoffsets, . - _xtensa_coproc_saoffsets

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Macro: xtensa_coproc_savestate
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
 *   - Registers have been saved/spilled already.
 *
 * Exit conditions:
 *   - All necessary CP callee-saved state has been saved.
 *   - Registers a2-a7, a13-a14 have been trashed.
 *
 ****************************************************************************/

	.macro xtensa_coproc_savestate

	/* The stack when interrupt happened
	* ----------------------------------------------------
	* | Reserve area (0x20)                              |
	* ----------------------------------------------------
	* | Coproc context                                   |
	* ----------------------------------------------------
	* | Xtensa common regs                               |
	* ---------------------------------------------------| <- SP
	*/

	addi		a3, sp, (4 * XCPTCONTEXT_REGS)

	/* CPENABLE should show which CPs are enabled. */

	rsr		a2, CPENABLE				/* a2 = which CPs are enabled */
	beqz		a2, Ldone1				/* Quick exit if none */

	movi		a13, _xtensa_coproc_saoffsets		/* Array of CP save offsets */

#if XCHAL_CP0_SA_SIZE > 0
	bbci.l		a2,  0,   2f				/* CP 0 not enabled */
	l32i		a14, a13, 0				/* a14 = _xtensa_coproc_saoffsets[0] */
	add		a3,  a14, a3				/* a3 = save area for CP 0 */
	xchal_cp0_store	a3,  a4,  a5, a6, a7
2:
#endif

#if XCHAL_CP1_SA_SIZE > 0
	bbci.l		a2,  1,   2f				/* CP 1 not enabled */
	l32i		a14, a13, 4				/* a14 = _xtensa_coproc_saoffsets[1] */
	add		a3,  a14, a3				/* a3 = save area for CP 1 */
	xchal_cp1_store	a3,  a4,  a5, a6, a7
2:
#endif

#if XCHAL_CP2_SA_SIZE > 0
	bbci.l		a2,  2,   2f
	l32i		a14, a13, 8
	add		a3,  a14, a3
	xchal_cp2_store	a3,  a4,  a5, a6, a7
2:
#endif

#if XCHAL_CP3_SA_SIZE > 0
	bbci.l		a2,  3,   2f
	l32i		a14, a13, 12
	add		a3,  a14, a3
	xchal_cp3_store	a3,  a4,  a5, a6, a7
2:
#endif

#if XCHAL_CP4_SA_SIZE > 0
	bbci.l		a2,  4,   2f
	l32i		a14, a13, 16
	add		a3,  a14, a3
	xchal_cp4_store	a3,  a4,  a5, a6, a7
2:
#endif

#if XCHAL_CP5_SA_SIZE > 0
	bbci.l		a2,  5,   2f
	l32i		a14, a13, 20
	add		a3,  a14, a3
	xchal_cp5_store	a3,  a4,  a5, a6, a7
2:
#endif

#if XCHAL_CP6_SA_SIZE > 0
	bbci.l		a2,  6,   2f
	l32i		a14, a13, 24
	add		a3,  a14, a3
	xchal_cp6_store	a3,  a4,  a5, a6, a7
2:
#endif

#if XCHAL_CP7_SA_SIZE > 0
	bbci.l		a2,  7,   2f
	l32i		a14, a13, 28
	add		a3,  a14, a3
	xchal_cp7_store	a3,  a4,  a5, a6, a7
2:
#endif

Ldone1:

.endm

/****************************************************************************
 * Macro: xtensa_coproc_restorestate
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
 *
 * Exit conditions:
 *   - All necessary CP callee-saved state has been restored.
 *   - Registers a3-a8, a13-a14 have been trashed.
 *
 ****************************************************************************/

	.macro xtensa_coproc_restorestate

	/* The stack when interrupt happened (the register A2)
	* ----------------------------------------------------
	* | Reserve area (0x20)                              |
	* ----------------------------------------------------
	* | Coproc context                                   |
	* ----------------------------------------------------
	* | Xtensa common regs                               |
	* ---------------------------------------------------| <- SP
	*/

	addi		a3, a2, (4 * XCPTCONTEXT_REGS)

	rsr		a8, CPENABLE			/* a8 = which CPs are enabled */
	beqz		a8, Ldone2			/* Quick exit if none */

	movi		a13, _xtensa_coproc_saoffsets	/* Array of CP save offsets */

#if XCHAL_CP0_SA_SIZE
	bbci.l		a8,  0,   2f			/* CP 0 not enabled */
	l32i		a14, a13, 0			/* a14 = _xtensa_coproc_saoffsets[0] */
	add		a3,  a14, a3			/* a3 = save area for CP 0 */
	xchal_cp0_load	a3,  a4,  a5, a6, a7
2:
#endif

#if XCHAL_CP1_SA_SIZE
	bbci.l		a8,  1,   2f			/* CP 1 not enabled */
	l32i		a14, a13, 4			/* a14 = _xtensa_coproc_saoffsets[1] */
	add		a3,  a14, a3			/* a3 = save area for CP 1 */
	xchal_cp1_load	a3,  a4,  a5, a6, a7
2:
#endif

#if XCHAL_CP2_SA_SIZE
	bbci.l		a8,  2,   2f
	l32i		a14, a13, 8
	add		a3,  a14, a3
	xchal_cp2_load	a3,  a4,  a5, a6, a7
2:
#endif

#if XCHAL_CP3_SA_SIZE
	bbci.l		a8,  3,   2f
	l32i		a14, a13, 12
	add		a3,  a14, a3
	xchal_cp3_load	a3,  a4,  a5, a6, a7
2:
#endif

#if XCHAL_CP4_SA_SIZE
	bbci.l		a8,  4,   2f
	l32i		a14, a13, 16
	add		a3,  a14, a3
	xchal_cp4_load	a3,  a4,  a5, a6, a7
2:
#endif

#if XCHAL_CP5_SA_SIZE
	bbci.l		a8,  5,   2f
	l32i		a14, a13, 20
	add		a3,  a14, a3
	xchal_cp5_load	a3,  a4,  a5, a6, a7
2:
#endif

#if XCHAL_CP6_SA_SIZE
	bbci.l		a8,  6,   2f
	l32i		a14, a13, 24
	add		a3,  a14, a3
	xchal_cp6_load	a3,  a4,  a5, a6, a7
2:
#endif

#if XCHAL_CP7_SA_SIZE
	bbci.l		a8,  7,   2f
	l32i		a14, a13, 28
	add		a3,  a14, a3
	xchal_cp7_load	a3,  a4,  a5, a6, a7
2:
#endif
	/* Ensure wsr.CPENABLE has completed. */

	rsync

Ldone2:

.endm
