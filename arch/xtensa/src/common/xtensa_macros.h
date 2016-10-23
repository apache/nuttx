/****************************************************************************
 * arch/xtensa/src/common/xtensa_macros.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_XTENSA_MACROS_H
#define __ARCH_XTENSA_SRC_COMMON_XTENSA_MACROS_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip_macros.h"

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__
/* Macros to handle ABI specifics of function entry and return.
 *
 * Convenient where the frame size requirements are the same for both ABIs.
 *    ENTRY(sz), RET(sz) are for framed functions (have locals or make calls).
 *    ENTRY0,    RET0    are for frameless functions (no locals, no calls).
 *
 * where size = size of stack frame in bytes (must be >0 and aligned to 16).
 * For framed functions the frame is created and the return address saved at
 * base of frame (Call0 ABI) or as determined by hardware (Windowed ABI).
 * For frameless functions, there is no frame and return address remains in a0.
 * Note: Because CPP macros expand to a single line, macros requiring multi-line
 * expansions are implemented as assembler macros.
 */

#ifdef CONFIG_XTENSA_CALL0_ABI
  /* Call0 */

	.macro	entry1 size=0x10
	addi	sp, sp, -\size
	s32i	a0, sp, 0
	.endm

	.macro	ret1 size=0x10
	l32i	a0, sp, 0
	addi	sp, sp, \size
	ret
	.endm

#  define ENTRY(sz)   entry1  sz
#  define ENTRY0
#  define RET(sz)     ret1    sz
#  define RET0        ret

#else
  /* Windowed */

#  define ENTRY(sz)   entry   sp, sz
#  define ENTRY0      entry   sp, 0x10
#  define RET(sz)     retw
#  define RET0        retw

#endif /* CONFIG_XTENSA_CALL0_ABI */

#endif /* __ASSEMBLY */
#endif /* __ARCH_XTENSA_SRC_COMMON_XTENSA_MACROS_H */
