/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_world1_vectors.S
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

/* The vector table used while the CPU executes in WORLD1, the unprivileged
 * world, in a kernel build.  The World Controller gives each world its own
 * vector table base, and this one exists so that WORLD1 needs no execute
 * permission on the kernel's own vectors.
 *
 * Window overflow and underflow are handled here, in WORLD1, because they
 * only ever touch the interrupted task's own stack and because there is no
 * way back: entering the kernel table switches the CPU to WORLD0, and only
 * the level 1 and level 3 exception paths restore the interruptee's world
 * on the way out (see exception_exit_hook in chip_macros.h).  A window spill
 * returns with RFWO/RFWU, which would leave the task running privileged.
 *
 * Every other vector jumps to its counterpart in the kernel table.  Those
 * kernel entry points are registered with the World Controller as WORLD0
 * entry addresses, so fetching one is what switches the CPU to WORLD0 -- the
 * jump instruction itself still executes in WORLD1, out of this table.
 *
 * The layout must match the kernel table in esp32s3_sections.ld, since the
 * hardware picks the slot by exception type and vector base alone.
 */

	.file	"esp32s3_world1_vectors.S"

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

	.section	.world1_vectors.text, "ax"
	.global		_world1_vectors
	.type		_world1_vectors, @function
	.align		1024

_world1_vectors:

/****************************************************************************
 * Window exception vectors
 ****************************************************************************/

/****************************************************************************
 * Name: _world1_window_overflow4
 ****************************************************************************/

	.org	0x0
	.global	_world1_window_overflow4
_world1_window_overflow4:
	s32e	a0, a5, -16     /* save a0 to call[j+1]'s stack frame */
	s32e	a1, a5, -12     /* save a1 to call[j+1]'s stack frame */
	s32e	a2, a5,  -8     /* save a2 to call[j+1]'s stack frame */
	s32e	a3, a5,  -4     /* save a3 to call[j+1]'s stack frame */
	rfwo                    /* rotates back to call[i] position */

/****************************************************************************
 * Name: _world1_window_underflow4
 ****************************************************************************/

	.org	0x40
	.global	_world1_window_underflow4
_world1_window_underflow4:
	l32e	a0, a5, -16     /* restore a0 from call[i+1]'s stack frame */
	l32e	a1, a5, -12     /* restore a1 from call[i+1]'s stack frame */
	l32e	a2, a5,  -8     /* restore a2 from call[i+1]'s stack frame */
	l32e	a3, a5,  -4     /* restore a3 from call[i+1]'s stack frame */
	rfwu

/****************************************************************************
 * Name: _world1_window_overflow8
 ****************************************************************************/

	.org	0x80
	.global	_world1_window_overflow8
_world1_window_overflow8:
	s32e	a0, a9, -16     /* save a0 to call[j+1]'s stack frame */
	l32e	a0, a1, -12     /* a0 <- call[j-1]'s sp
	                           (used to find end of call[j]'s frame) */
	s32e	a1, a9, -12     /* save a1 to call[j+1]'s stack frame */
	s32e	a2, a9,  -8     /* save a2 to call[j+1]'s stack frame */
	s32e	a3, a9,  -4     /* save a3 to call[j+1]'s stack frame */
	s32e	a4, a0, -32     /* save a4 to call[j]'s stack frame */
	s32e	a5, a0, -28     /* save a5 to call[j]'s stack frame */
	s32e	a6, a0, -24     /* save a6 to call[j]'s stack frame */
	s32e	a7, a0, -20     /* save a7 to call[j]'s stack frame */
	rfwo                    /* rotates back to call[i] position */

/****************************************************************************
 * Name: _world1_window_underflow8
 ****************************************************************************/

	.org	0xc0
	.global	_world1_window_underflow8
_world1_window_underflow8:
	l32e	a0, a9, -16     /* restore a0 from call[i+1]'s stack frame */
	l32e	a1, a9, -12     /* restore a1 from call[i+1]'s stack frame */
	l32e	a2, a9,  -8     /* restore a2 from call[i+1]'s stack frame */
	l32e	a7, a1, -12     /* a7 <- call[i-1]'s sp
	                           (used to find end of call[i]'s frame) */
	l32e	a3, a9,  -4     /* restore a3 from call[i+1]'s stack frame */
	l32e	a4, a7, -32     /* restore a4 from call[i]'s stack frame */
	l32e	a5, a7, -28     /* restore a5 from call[i]'s stack frame */
	l32e	a6, a7, -24     /* restore a6 from call[i]'s stack frame */
	l32e	a7, a7, -20     /* restore a7 from call[i]'s stack frame */
	rfwu

/****************************************************************************
 * Name: _world1_window_overflow12
 ****************************************************************************/

	.org	0x100
	.global	_world1_window_overflow12
_world1_window_overflow12:
	s32e	a0,  a13, -16   /* save a0 to call[j+1]'s stack frame */
	l32e	a0,  a1,  -12   /* a0 <- call[j-1]'s sp
	                           (used to find end of call[j]'s frame) */
	s32e	a1,  a13, -12   /* save a1 to call[j+1]'s stack frame */
	s32e	a2,  a13,  -8   /* save a2 to call[j+1]'s stack frame */
	s32e	a3,  a13,  -4   /* save a3 to call[j+1]'s stack frame */
	s32e	a4,  a0,  -48   /* save a4 to end of call[j]'s stack frame */
	s32e	a5,  a0,  -44   /* save a5 to end of call[j]'s stack frame */
	s32e	a6,  a0,  -40   /* save a6 to end of call[j]'s stack frame */
	s32e	a7,  a0,  -36   /* save a7 to end of call[j]'s stack frame */
	s32e	a8,  a0,  -32   /* save a8 to end of call[j]'s stack frame */
	s32e	a9,  a0,  -28   /* save a9 to end of call[j]'s stack frame */
	s32e	a10, a0,  -24   /* save a10 to end of call[j]'s stack frame */
	s32e	a11, a0,  -20   /* save a11 to end of call[j]'s stack frame */
	rfwo                    /* rotates back to call[i] position */

/****************************************************************************
 * Name: _world1_window_underflow12
 ****************************************************************************/

	.org	0x140
	.global	_world1_window_underflow12
_world1_window_underflow12:
	l32e	a0,  a13, -16   /* restore a0 from call[i+1]'s stack frame */
	l32e	a1,  a13, -12   /* restore a1 from call[i+1]'s stack frame */
	l32e	a2,  a13,  -8   /* restore a2 from call[i+1]'s stack frame */
	l32e	a11, a1,  -12   /* a11 <- call[i-1]'s sp
	                           (used to find end of call[i]'s frame) */
	l32e	a3,  a13,  -4   /* restore a3 from call[i+1]'s stack frame */
	l32e	a4,  a11, -48   /* restore a4 from end of call[i]'s stack frame */
	l32e	a5,  a11, -44   /* restore a5 from end of call[i]'s stack frame */
	l32e	a6,  a11, -40   /* restore a6 from end of call[i]'s stack frame */
	l32e	a7,  a11, -36   /* restore a7 from end of call[i]'s stack frame */
	l32e	a8,  a11, -32   /* restore a8 from end of call[i]'s stack frame */
	l32e	a9,  a11, -28   /* restore a9 from end of call[i]'s stack frame */
	l32e	a10, a11, -24   /* restore a10 from end of call[i]'s stack frame */
	l32e	a11, a11, -20   /* restore a11 from end of call[i]'s stack frame */
	rfwu

/****************************************************************************
 * Medium-/High-priority interrupt vectors
 ****************************************************************************/

	.org	0x180
	j	_xtensa_level2_vector

	.org	0x1c0
	j	_xtensa_level3_vector

/* 0x200 and 0x240 are the level 4 and level 5 vectors.  This configuration
 * has no interrupt above level 3, and the kernel table leaves those slots
 * empty as well.
 */

	.org	0x280
	j	_debug_exception_vector

	.org	0x2c0
	j	_xtensa_nmi_vector

/****************************************************************************
 * General exception vectors
 ****************************************************************************/

/* A WORLD1 task always takes the user exception vector, since it runs with
 * PS.UM set.  The kernel slot is jumped to anyway, so that a table entry is
 * never a hole.
 */

	.org	0x300
	j	_kernel_exception_vector

	.org	0x340
	j	_user_exception_vector

	.org	0x3c0
	j	_double_exception_vector

	.org	0x400

	.size	_world1_vectors, . - _world1_vectors
