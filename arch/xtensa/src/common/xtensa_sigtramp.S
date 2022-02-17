/****************************************************************************
 * arch/xtensa/src/common/xtensa_sigtramp.S
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

	.file	"xtensa_sigtramp.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/xtensa/xtensa_abi.h>

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
