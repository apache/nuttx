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

#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_specregs.h>

#include "xtensa_abi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * C Prototype:
 *   void _xtensa_levelN_vector(void)
 *
 * Description:
 *   Xtensa interrupt vectors.  Each vector goes at a predetermined location
 *   according to the Xtensa hardware configuration, which is ensured by its
 *   placement in a special section known to the NuttX linker script.  The
 *   vector logic performs the minimum necessary operations before jumping
 *   to the handler.
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

	.end        literal_prefix

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

	.end        literal_prefix

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

	.end        literal_prefix

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

	.size	_xtensa_level5_vector, . - _xtensa_level5_vector
	.end        literal_prefix
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
 * C Prototype:
 *   void _xtensa_nmi_vector(void)
 *
 * Description:
 *   Xtensa NMI  vectors.  This vector goes at a predetermined location
 *   according to the Xtensa hardware configuration, which is ensured by its
 *   placement in a special section known to the NuttX linker script.  The
 *   vector logic performs the minimum necessary operations before jumping
 *   to the handler.
 *
 ****************************************************************************/

#if XCHAL_HAVE_NMI
	.begin		literal_prefix .nmi_vector
	.section	.nmi_vector.text, "ax"
	.global		_xtensa_nmi_vector
	.type		_xtensa_nmi_vector, @function
	.align		4

_xtensa_nmi_vector:

	wsr			a0, EXCSAVE + XCHAL_NMILEVEL	/* Preserve a0 */
	call0		_xtensa_nmi						/* Load interrupt handler */

	/* Never returns here - call0 is used as a jump */

	.size		_xtensa_nmi_vector, . - _xtensa_nmi_vector
	.end		literal_prefix
#endif
