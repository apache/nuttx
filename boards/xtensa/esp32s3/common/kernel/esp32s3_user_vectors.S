/****************************************************************************
 * boards/xtensa/esp32s3/common/kernel/esp32s3_user_vectors.S
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

	.file	"esp32s3_user_vectors.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_abi.h>
#include <arch/xtensa/xtensa_specregs.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Medium-/High-priority interrupt vectors
 ****************************************************************************/

/****************************************************************************
 * Name: _xtensa_level2_vector
 ****************************************************************************/

	.begin		literal_prefix .xtensa_level2_vector
	.section	.xtensa_level2_vector.text, "ax"
	.global		_xtensa_level2_vector
	.type		_xtensa_level2_vector, @function
	.align		4

_xtensa_level2_vector:
	j			__kernel_vector_table + 0x180

	.size		_xtensa_level2_vector, . - _xtensa_level2_vector
	.end		literal_prefix

/****************************************************************************
 * Name: _xtensa_level3_vector
 ****************************************************************************/

	.begin		literal_prefix .xtensa_level3_vector
	.section	.xtensa_level3_vector.text, "ax"
	.global		_xtensa_level3_vector
	.type		_xtensa_level3_vector, @function
	.align		4

_xtensa_level3_vector:
	j			__kernel_vector_table + 0x1c0

	.size		_xtensa_level3_vector, . - _xtensa_level3_vector
	.end		literal_prefix

/****************************************************************************
 * Name: _xtensa_level4_vector
 ****************************************************************************/

	.begin		literal_prefix .xtensa_level4_vector
	.section	.xtensa_level4_vector.text, "ax"
	.global		_xtensa_level4_vector
	.type		_xtensa_level4_vector, @function
	.align		4

_xtensa_level4_vector:
	j			__kernel_vector_table + 0x200

	.size		_xtensa_level4_vector, . - _xtensa_level4_vector
	.end		literal_prefix

/****************************************************************************
 * Name: _xtensa_level5_vector
 ****************************************************************************/

	.begin		literal_prefix .xtensa_level5_vector
	.section	.xtensa_level5_vector.text, "ax"
	.global		_xtensa_level5_vector
	.type		_xtensa_level5_vector, @function
	.align		4

_xtensa_level5_vector:
	j			__kernel_vector_table + 0x240

	.size		_xtensa_level5_vector, . - _xtensa_level5_vector
	.end		literal_prefix

/****************************************************************************
 * Name: _debug_exception_vector
 ****************************************************************************/

	.begin		literal_prefix .debug_exception_vector
	.section	.debug_exception_vector.text, "ax"
	.global		_debug_exception_vector
	.type		_debug_exception_vector, @function
	.align		4

_debug_exception_vector:
	j			__kernel_vector_table + 0x280

	.size		_debug_exception_vector, . - _debug_exception_vector
	.end		literal_prefix

/****************************************************************************
 * Name: _nmi_exception_vector
 ****************************************************************************/

	.begin		literal_prefix .nmi_exception_vector
	.section	.nmi_exception_vector.text, "ax"
	.global		_nmi_exception_vector
	.type		_nmi_exception_vector, @function
	.align		4

_nmi_exception_vector:
	j			__kernel_vector_table + 0x2c0

	.size		_nmi_exception_vector, . - _nmi_exception_vector
	.end		literal_prefix

/****************************************************************************
 * General exception vectors
 ****************************************************************************/

/****************************************************************************
 * Name: _kernel_exception_vector
 ****************************************************************************/

	.begin		literal_prefix .kernel_exception_vector
	.section	.kernel_exception_vector.text, "ax"
	.global		_kernel_exception_vector
	.type		_kernel_exception_vector, @function
	.align		4

_kernel_exception_vector:
	j			__kernel_vector_table + 0x300

	.size		_kernel_exception_vector, . - _kernel_exception_vector
	.end		literal_prefix

/****************************************************************************
 * Name: _user_exception_vector
 ****************************************************************************/

	.begin		literal_prefix .user_exception_vector
	.section	.user_exception_vector.text, "ax"
	.global		_user_exception_vector
	.type		_user_exception_vector, @function
	.align		4

_user_exception_vector:
	j			__kernel_vector_table + 0x340

	.size		_user_exception_vector, . - _user_exception_vector
	.end		literal_prefix

/****************************************************************************
 * Name: _double_exception_vector
 ****************************************************************************/

	.begin		literal_prefix .double_exception_vector
	.section	.double_exception_vector.text, "ax"
	.global		_double_exception_vector
	.type		_double_exception_vector, @function
	.align		4

_double_exception_vector:
	j			__kernel_vector_table + 0x3c0

	.size		_double_exception_vector, . - _double_exception_vector
	.end		literal_prefix

/****************************************************************************
 * Window exception vectors
 ****************************************************************************/

	.section .window_vectors.text, "ax"

/****************************************************************************
 * Name: _window_overflow4
 ****************************************************************************/

	.org    0x0
	.global _window_overflow4
_window_overflow4:
	s32e    a0, a5, -16     /* save a0 to call[j+1]'s stack frame */
	s32e    a1, a5, -12     /* save a1 to call[j+1]'s stack frame */
	s32e    a2, a5,  -8     /* save a2 to call[j+1]'s stack frame */
	s32e    a3, a5,  -4     /* save a3 to call[j+1]'s stack frame */
	rfwo                    /* rotates back to call[i] position */

/****************************************************************************
 * Name: _window_underflow4
 ****************************************************************************/

	.org    0x40
	.global _window_underflow4
_window_underflow4:
	l32e    a0, a5, -16     /* restore a0 from call[i+1]'s stack frame */
	l32e    a1, a5, -12     /* restore a1 from call[i+1]'s stack frame */
	l32e    a2, a5,  -8     /* restore a2 from call[i+1]'s stack frame */
	l32e    a3, a5,  -4     /* restore a3 from call[i+1]'s stack frame */
	rfwu

/****************************************************************************
 * Name: _window_overflow8
 ****************************************************************************/

	.org    0x80
	.global _window_overflow8
_window_overflow8:
	s32e    a0, a9, -16     /* save a0 to call[j+1]'s stack frame */
	l32e    a0, a1, -12     /* a0 <- call[j-1]'s sp
                               (used to find end of call[j]'s frame) */
	s32e    a1, a9, -12     /* save a1 to call[j+1]'s stack frame */
	s32e    a2, a9,  -8     /* save a2 to call[j+1]'s stack frame */
	s32e    a3, a9,  -4     /* save a3 to call[j+1]'s stack frame */
	s32e    a4, a0, -32     /* save a4 to call[j]'s stack frame */
	s32e    a5, a0, -28     /* save a5 to call[j]'s stack frame */
	s32e    a6, a0, -24     /* save a6 to call[j]'s stack frame */
	s32e    a7, a0, -20     /* save a7 to call[j]'s stack frame */
	rfwo                    /* rotates back to call[i] position */

/****************************************************************************
 * Name: _window_underflow8
 ****************************************************************************/

	.org    0xc0
	.global _window_underflow8
_window_underflow8:
	l32e    a0, a9, -16     /* restore a0 from call[i+1]'s stack frame */
	l32e    a1, a9, -12     /* restore a1 from call[i+1]'s stack frame */
	l32e    a2, a9,  -8     /* restore a2 from call[i+1]'s stack frame */
	l32e    a7, a1, -12     /* a7 <- call[i-1]'s sp
                               (used to find end of call[i]'s frame) */
	l32e    a3, a9,  -4     /* restore a3 from call[i+1]'s stack frame */
	l32e    a4, a7, -32     /* restore a4 from call[i]'s stack frame */
	l32e    a5, a7, -28     /* restore a5 from call[i]'s stack frame */
	l32e    a6, a7, -24     /* restore a6 from call[i]'s stack frame */
	l32e    a7, a7, -20     /* restore a7 from call[i]'s stack frame */
	rfwu

/****************************************************************************
 * Name: _window_overflow12
 ****************************************************************************/

	.org    0x100
	.global _window_overflow12
_window_overflow12:
	s32e    a0,  a13, -16   /* save a0 to call[j+1]'s stack frame */
	l32e    a0,  a1,  -12   /* a0 <- call[j-1]'s sp
	                           (used to find end of call[j]'s frame) */
	s32e    a1,  a13, -12   /* save a1 to call[j+1]'s stack frame */
	s32e    a2,  a13,  -8   /* save a2 to call[j+1]'s stack frame */
	s32e    a3,  a13,  -4   /* save a3 to call[j+1]'s stack frame */
	s32e    a4,  a0,  -48   /* save a4 to end of call[j]'s stack frame */
	s32e    a5,  a0,  -44   /* save a5 to end of call[j]'s stack frame */
	s32e    a6,  a0,  -40   /* save a6 to end of call[j]'s stack frame */
	s32e    a7,  a0,  -36   /* save a7 to end of call[j]'s stack frame */
	s32e    a8,  a0,  -32   /* save a8 to end of call[j]'s stack frame */
	s32e    a9,  a0,  -28   /* save a9 to end of call[j]'s stack frame */
	s32e    a10, a0,  -24   /* save a10 to end of call[j]'s stack frame */
	s32e    a11, a0,  -20   /* save a11 to end of call[j]'s stack frame */
	rfwo                    /* rotates back to call[i] position */

/****************************************************************************
 * Name: _window_underflow12
 ****************************************************************************/

	.org    0x140
	.global _window_underflow12
_window_underflow12:
	l32e    a0,  a13, -16   /* restore a0 from call[i+1]'s stack frame */
	l32e    a1,  a13, -12   /* restore a1 from call[i+1]'s stack frame */
	l32e    a2,  a13,  -8   /* restore a2 from call[i+1]'s stack frame */
	l32e    a11, a1,  -12   /* a11 <- call[i-1]'s sp
	                           (used to find end of call[i]'s frame) */
	l32e    a3,  a13,  -4   /* restore a3 from call[i+1]'s stack frame */
	l32e    a4,  a11, -48   /* restore a4 from end of call[i]'s stack frame */
	l32e    a5,  a11, -44   /* restore a5 from end of call[i]'s stack frame */
	l32e    a6,  a11, -40   /* restore a6 from end of call[i]'s stack frame */
	l32e    a7,  a11, -36   /* restore a7 from end of call[i]'s stack frame */
	l32e    a8,  a11, -32   /* restore a8 from end of call[i]'s stack frame */
	l32e    a9,  a11, -28   /* restore a9 from end of call[i]'s stack frame */
	l32e    a10, a11, -24   /* restore a10 from end of call[i]'s stack frame */
	l32e    a11, a11, -20   /* restore a11 from end of call[i]'s stack frame */
	rfwu
