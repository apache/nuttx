/****************************************************************************
 * arch/xtensa/src/common/xtensa_sigtramp.S
 *
 * Adapted from use in NuttX by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
	ENTRY(16)						/* REVISIT: This should not be here */

#ifdef __XTENSA_CALL0_ABI__
	cali0	xtensa_sig_deliver		/* Call xtensa_sig_deliver */
#else
	call4	xtensa_sig_deliver		/* Call xtensa_sig_deliver */
#endif

1:	j		1b						/* xtensa_sig_deliver does not return */

	.size	_xtensa_sig_trampoline, . - _xtensa_sig_trampoline
