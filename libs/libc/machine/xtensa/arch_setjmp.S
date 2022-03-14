/****************************************************************************
 * libs/libc/machine/xtensa/arch_setjmp.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/xtensa/core.h>
#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_abi.h>

#include <arch/syscall.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if XCHAL_HAVE_WINDOWED && !defined(__XTENSA_CALL0_ABI___)

/* Windowed ABI:

   This implementation relies heavily on the Xtensa register window
   mechanism.  Setjmp flushes all the windows except its own to the
   stack and then copies registers from the save areas on the stack
   into the jmp_buf structure, along with the return address of the call
   to setjmp.  Longjmp invalidates all the windows except its own, and
   then sets things up so that it will return to the right place,
   using a window underflow to automatically restore the registers.

   Note that it would probably be sufficient to only copy the
   registers from setjmp's caller into jmp_buf.  However, we also copy
   the save area located at the stack pointer of setjmp's caller.
   This save area will typically remain intact until the longjmp call.
   The one exception is when there is an intervening alloca in
   setjmp's caller.  This is certainly an unusual situation and is
   likely to cause problems in any case (the storage allocated on the
   stack cannot be safely accessed following the longjmp).  As bad as
   it is, on most systems this situation would not necessarily lead to
   a catastrophic failure.  If we did not preserve the extra save area
   on Xtensa, however, it would.  When setjmp's caller returns after a
   longjmp, there will be a window underflow; an invalid return
   address or stack pointer in the save area will almost certainly
   lead to a crash.  Keeping a copy of the extra save area in the
   jmp_buf avoids this with only a small additional cost.  If setjmp
   and longjmp are ever time-critical, this could be removed.
*/

  .text
  .align  4
  .literal_position
  .global setjmp
  .type setjmp, @function
setjmp:

# if XCHAL_HAVE_XEA3
/*
  a2 points to the jmp_buf structure of 68 bytes length:
  8 * 4 to save the regester save area of setjmp that contains the callers resgisters
  8 * 4 to save the caller's register save area which is pottentally
  clobbered by an alloca() in the caller
*/

  entry sp, 32

  /* Flush all registers.  */
  ssai  0
  spillw

  addi  a7, a1, -32   # find the destination save area
  s32i  a0, a2, 64

/* Copy the callee register save area to jmp_buf */
  l32i  a3, a7, 0
  l32i  a4, a7, 4
  s32i  a3, a2, 0
  s32i  a4, a2, 4
  l32i  a3, a7, 8
  l32i  a4, a7, 12
  s32i  a3, a2, 8
  s32i  a4, a2, 12
  l32i  a3, a7, 16
  l32i  a4, a7, 20
  s32i  a3, a2, 16
  s32i  a4, a2, 20
  l32i  a3, a7, 24
  l32i  a4, a7, 28
  s32i  a3, a2, 24
  s32i  a4, a2, 28

/* keep copy of callee register save area to protect against an
   alloca() (after the setjmp) clobbering the registers needed to return from
   the caller of setjmp */

  l32i  a3, a1, 0
  l32i  a4, a1, 4
  s32i  a3, a2, 32
  s32i  a4, a2, 36
  l32i  a3, a1, 8
  l32i  a4, a1, 12
  s32i  a3, a2, 40
  s32i  a4, a2, 44
  l32i  a3, a1, 16
  l32i  a4, a1, 20
  s32i  a3, a2, 48
  s32i  a4, a2, 52
  l32i  a3, a1, 24
  l32i  a4, a1, 28
  s32i  a3, a2, 56
  s32i  a4, a2, 60
# else
  entry sp, 16

  /* Flush registers.  */
  mov a4, a2      # save a2 (jmp_buf)
  movi  a2, SYS_flush_context
  movi a3, XCHAL_SWINT_CALL
  wsr  a3, intset
  isync
  mov a2, a4      # restore a2

  /* Copy the register save area at (sp - 16).  */
  addi  a5, a1, -16
  l32i  a3, a5, 0
  l32i  a4, a5, 4
  s32i  a3, a2, 0
  s32i  a4, a2, 4
  l32i  a3, a5, 8
  l32i  a4, a5, 12
  s32i  a3, a2, 8
  s32i  a4, a2, 12

  /* Copy 0-8 words from the register overflow area.  */
  extui a3, a0, 30, 2
  blti  a3, 2, .Lendsj
  l32i  a7, a1, 4
  slli  a4, a3, 4
  sub a5, a7, a4
  addi  a6, a2, 16
  addi  a7, a7, -16   # a7 = end of register overflow area
.Lsjloop:
  l32i  a3, a5, 0
  l32i  a4, a5, 4
  s32i  a3, a6, 0
  s32i  a4, a6, 4
  l32i  a3, a5, 8
  l32i  a4, a5, 12
  s32i  a3, a6, 8
  s32i  a4, a6, 12
  addi  a5, a5, 16
  addi  a6, a6, 16
  blt a5, a7, .Lsjloop
.Lendsj:

  /* Copy the register save area at sp.  */
  l32i  a3, a1, 0
  l32i  a4, a1, 4
  s32i  a3, a2, 48
  s32i  a4, a2, 52
  l32i  a3, a1, 8
  l32i  a4, a1, 12
  s32i  a3, a2, 56
  s32i  a4, a2, 60

  /* Save the return address, including the window size bits.  */
  s32i  a0, a2, 64
# endif

  movi  a2, 0
  retw
  .size setjmp, . - setjmp

/* void longjmp (jmp_buf env, int val) */

  .align  4
  .literal_position
  .global longjmp
  .type longjmp, @function
longjmp:
  /*  a2 == &env, a3 == val  */
#if XCHAL_HAVE_XEA3
  entry sp, 32
  ssai  0
  tossw

  l32i a0, a2, 64

  addi  a7, a1, -32   # find the destination save area
  l32i  a4, a2, 0
  l32i  a5, a2, 4
  s32i  a4, a7, 0
  s32i  a5, a7, 4
  l32i  a4, a2, 8
  l32i  a5, a2, 12
  s32i  a4, a7, 8
  s32i  a5, a7, 12
  l32i  a4, a2, 16
  l32i  a5, a2, 20
  s32i  a4, a7, 16
  s32i  a5, a7, 20
  l32i  a4, a2, 24
  l32i  a5, a2, 28
  s32i  a4, a7, 24
  s32i  a5, a7, 28

  /* The 8 words saved from the register save area at the target's
     sp are copied back to the target procedure's save area.  The
     only point of this is to prevent a catastrophic failure in
     case the contents were moved by an alloca after calling
     setjmp.  This is a bit paranoid but it doesn't cost much.
   */

  l32i  a7, a2, 4 /* get the stack pointer as it was at the call to setjmp() ...
                before any changed due to alloca() */
      addi  a7, a7, -32
  l32i  a4, a2, 32 /* copy the register values from the jmp_buf to the
                possibly clobbered register save area */
  l32i  a5, a2, 36
  s32i  a4, a7, 0
  s32i  a5, a7, 4
  l32i  a4, a2, 40
  l32i  a5, a2, 44
  s32i  a4, a7, 8
  s32i  a5, a7, 12
  l32i  a4, a2, 48
  l32i  a5, a2, 52
  s32i  a4, a7, 16
  s32i  a5, a7, 20
  l32i  a4, a2, 56
  l32i  a5, a2, 60
  s32i  a4, a7, 24
  s32i  a5, a7, 28

#else
  entry sp, 16

# if XCHAL_MAYHAVE_ERRATUM_XEA1KWIN
  /* Using this register triggers early any overflow that a kernel-mode
     level-one interrupt might otherwise cause.  */
#  define AR_WB a15
# else
  /* Using this register is more efficient; it triggers less overflows.  */
#  define AR_WB a5
# endif
  /* Invalidate all but the current window;
     set WindowStart to (1 << WindowBase).  */
  rsr AR_WB, WINDOWBASE
  movi  a4, 1
  ssl AR_WB
  sll a4, a4
  wsr a4, WINDOWSTART
  rsync

  /* Return to the return address of the setjmp, using the
     window size bits from the setjmp call so that the caller
     will be able to find the return value that we put in a2.  */

  l32i  a0, a2, 64

  /* Copy the first 4 saved registers from jmp_buf into the save area
     at the current sp so that the values will be restored to registers
     when longjmp returns.  */

  addi  a7, a1, -16
  l32i  a4, a2, 0
  l32i  a5, a2, 4
  s32i  a4, a7, 0
  s32i  a5, a7, 4
  l32i  a4, a2, 8
  l32i  a5, a2, 12
  s32i  a4, a7, 8
  s32i  a5, a7, 12

  /* Copy the remaining 0-8 saved registers.  */
  extui a7, a0, 30, 2
  blti  a7, 2, .Lendlj
  l32i  a8, a2, 52
  slli  a4, a7, 4
  sub a6, a8, a4
  addi  a5, a2, 16
  addi  a8, a8, -16   # a8 = end of register overflow area
.Lljloop:
  l32i  a7, a5, 0
  l32i  a4, a5, 4
  s32i  a7, a6, 0
  s32i  a4, a6, 4
  l32i  a7, a5, 8
  l32i  a4, a5, 12
  s32i  a7, a6, 8
  s32i  a4, a6, 12
  addi  a5, a5, 16
  addi  a6, a6, 16
  blt a6, a8, .Lljloop
.Lendlj:

  /* The 4 words saved from the register save area at the target's
     sp are copied back to the target procedure's save area.  The
     only point of this is to prevent a catastrophic failure in
     case the contents were moved by an alloca after calling
     setjmp.  This is a bit paranoid but it doesn't cost much.  */

  l32i  a7, a2, 4   # load the target stack pointer
  addi  a7, a7, -16   # find the destination save area
  l32i  a4, a2, 48
  l32i  a5, a2, 52
  s32i  a4, a7, 0
  s32i  a5, a7, 4
  l32i  a4, a2, 56
  l32i  a5, a2, 60
  s32i  a4, a7, 8
  s32i  a5, a7, 12
#endif

  /* Return val ? val : 1.  */
  movi  a2, 1
  movnez  a2, a3, a3

  retw
  .size longjmp, . - longjmp

#else

  /*
   Call0 ABI:
   Much like other ABIs, this version just saves the necessary registers
   to the stack and restores them later.  Much less needs to be done.
  */

  .text
  .align  4
  .literal_position
  .global setjmp
  .type setjmp, @function
setjmp:
  s32i  a0, a2, 0
  s32i  a1, a2, 4
  s32i  a12, a2, 8
  s32i  a13, a2, 12
  s32i  a14, a2, 16
  s32i  a15, a2, 20
  movi  a2, 0
  ret
  .size setjmp, . - setjmp

  .align  4
  .literal_position
  .global longjmp
  .type longjmp, @function
longjmp:
  l32i  a0, a2, 0
  l32i  a12, a2, 8
  l32i  a13, a2, 12
  l32i  a14, a2, 16
  l32i  a15, a2, 20
  l32i  a1, a2, 4
  /* Return val ? val : 1.  */
  movi  a2, 1
  movnez  a2, a3, a3

  ret
  .size longjmp, .-longjmp

#endif /* CALL0 ABI */
