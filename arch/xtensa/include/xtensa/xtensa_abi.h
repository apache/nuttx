/****************************************************************************
 * arch/xtensa/include/xtensa/xtensa_abi.h
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

#ifndef __ARCH_XTENSA_INCLUDE_XTENSA_XTENSA_ABI_H
#define __ARCH_XTENSA_INCLUDE_XTENSA_XTENSA_ABI_H

/* Windowed ABI
 *
 *   The Windowed Register Option replaces the simple 16-entry AR register
 *   file with a larger register file from which a window of 16 entries is
 *   visible at any given time.  The window is rotated on subroutine entry
 *   and exit, automatically saving and restoring some registers.  When the
 *   window is rotated far enough to require registers to be saved to or
 *   restored from the program stack, an exception is raised to move some
 *   of the register values between the register file and the program stack.
 *
 *   Windowed Register Usage:
 *     ---------------- ----------------------------------
 *     Callee Register  Usage
 *     Register Name
 *     ---------------- ----------------------------------
 *     a0               Return address
 *     a1/sp            Stack pointer
 *     a2..a7           In, out, inout, and return values
 *     ---------------- ----------------------------------
 *
 *   Calls to routines that use only a2..a3 as parameters may use the CALL4,
 *   CALL8, or CALL12 instructions to save 4, 8, or 12 live registers. Calls
 *   to routines that use a2..a7 for parameters may use only CALL8 or CALL12.
 *
 *   Arguments are passed in both registers and memory. The first six
 *   incoming arguments are stored in registers a2 through a7, and additional
 *   arguments are stored on the stack starting at the current stack pointer
 *   a1.  Because Xtensa uses register windows that rotate during a function
 *   call, outgoing arguments that will become the incoming arguments must be
 *   stored to different register numbers. Depending on the call instruction
 *   and, thus, the rotation of the register window, the arguments are passed
 *   starting starting with register a(2+N), where N is the size of the
 *   window rotation.
 *   Therefore, the first argument in case of a call4 instruction is placed
 *   into a6, and for a call8 instruction into a10. Large arguments (8-bytes)
 *   are always passed in an even/odd register pair even if that means to
 *   omit a register for alignment. The return values are stored in a2
 *   through a7.
 *
 *          return addr  stack ptr  arg0, arg1, arg2, arg3, arg4, arg5
 *          -----------  ---------  ----------------------------------
 *            a0           a1         a2,   a3,   a4,   a5,   a6,   a7
 *
 *   call4    a4           a5         a6,   a7,   a8,   a9,  a10,  a11
 *   call8    a8           a9        a10,  a11,  a12,  a13,  a14,  a15
 *   call12  a12          a13        a14,  a15   ---   ---   ---   ---
 *
 *   The stack pointer SP should only be modified by ENTRY and MOVSP
 *   instructions (except for initialization and restoration). If some other
 *   instruction modifies SP, any values in the register-spill area will not
 *   be moved.
 *
 * Call 0 ABI
 *
 *   CALL0 AR Register Usage
 *     ---------------- ----------------------------------
 *     Callee Register  Usage
 *     Register Name
 *     ---------------- ----------------------------------
 *     a0               Return Address
 *     a1/sp            Stack pointer
 *     a2..a7           In, out, inout, and return values
 *     a8               Static Chain
 *     a12..a15         Callee-saved
 *     a15              Stack-Frame Pointer (optional)
 *
 *     a0, a2-a11       Caller-saved
 *     a1, a12..a15     Callee-saved
 *     ---------------- ----------------------------------
 *
 *   CALL0 is used.  The return address is placed in A0 and the CPU simply
 *   jumps to the CALL0 function entry point.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MACROS TO HANDLE ABI SPECIFICS OF FUNCTION ENTRY AND RETURN
 *
 * Convenient where the frame size requirements are the same for both ABIs.
 *   ENTRY(sz), RET(sz) are for framed functions (have locals or make calls).
 *   ENTRY0,    RET0    are for frameless functions (no locals, no calls).
 *
 * where size = size of stack frame in bytes (must be >0 and aligned to 16).
 * For framed functions the frame is created and the return address saved at
 * base of frame (Call0 ABI) or as determined by hardware (Windowed ABI).
 * For frameless functions, there is no frame and return address remains in
 * a0.
 * Note: Because CPP macros expand to a single line, macros requiring
 * multi-line expansions are implemented as assembler macros.
 */

#ifdef __ASSEMBLY__
/* Function prologues and epilogues */

#ifdef __XTENSA_CALL0_ABI__
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

#  define ENTRY(sz)     entry1  sz
#  define ENTRY0
#  define RET(sz)       ret1    sz
#  define RET0          ret

#else
  /* Windowed */

#  define ENTRY(sz)     entry   sp, sz
#  define ENTRY0        entry   sp, 0x10
#  define RET(sz)       retw
#  define RET0          retw

#endif

/* Index into stack frame.
 * For CALL0 ABI the argument "n" should be greater than 0 to avoid
 * corrupting the saved A0 if ENTRY was used.
 */

#define LOCAL_OFFSET(n) ((n) << 2)  /* n = 0/1 .. ((size >> 2) - 1) */

#endif /* __ASSEMBLY_ */

#endif /* __ARCH_XTENSA_INCLUDE_XTENSA_XTENSA_ABI_H */
