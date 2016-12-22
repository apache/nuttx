/****************************************************************************
 * arch/xtensa/src/common/xtensa_sigtramp.S
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

	.file	"xtensa_sigtramp.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "xtensa_abi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _xtensa_sig_trampoline
 *
 * Description:
 *   Just sets up a proper window call to xtensa_sig_deliver().  We get
 *   here via a context switch setup in up_schedule_signaction.  Which
 *   re-vectors the context switch to this location.
 *
 *   Here we just call xtensa_sig_deliver() using the proper ABI.  NOTE
 *   that this function cannot return and depends on the fact that
 *   xtensa_sig_deliver() does not return.
 *
 *
 ****************************************************************************/

	.text
	.global	_xtensa_sig_trampoline
	.type	_xtensa_sig_trampoline, @function
	.align	4

_xtensa_sig_trampoline:
	ENTRY(16)

#ifdef __XTENSA_CALL0_ABI__
	cali0	xtensa_sig_deliver				/* Call xtensa_sig_deliver */
#else
	call4	xtensa_sig_deliver				/* Call xtensa_sig_deliver */
#endif

1:	j		1b

	.size	_xtensa_sig_trampoline, . - _xtensa_sig_trampoline
