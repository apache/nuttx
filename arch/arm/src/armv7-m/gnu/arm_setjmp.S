/************************************************************************************
 * arch/arm/src/armv7-m/gnu/arm_setjmp.S
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: David S. Alessio <David@DSA.Consulting>
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
 ************************************************************************************/

/* When this file is assembled, it will require the following GCC options:
 *
 * -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -meabi=5 -mthumb
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

    .globl      setjmp
    .globl      longjmp

    .syntax     unified
    .thumb
    .file       "setjmp.S"

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: setjmp
 *
 * Description:
 *   Given the pointer to a register save area (in R0), save the state of the
 *   all callee-saved registers
 *
 * C Function Prototype:
 *   int setjmp(jmp_buf env);
 *
 * Input Parameters:
 *   env - A pointer to the register save area in which to save the floating point
 *     registers and core registers.  Since setjmp() can not be inlined, we
 *     only need to save the ABI-specified callee-saved registers.
 *
 * Returned Value:
 *   0      setjmp called directly
 *   non-0  we justed returned from a longjmp()
 *
 ************************************************************************************/

    .thumb_func
    .type   setjmp, function
setjmp:

    /* Store callee-saved Core registers */

    mov     ip, sp                  /* move sp to ip so we can save it */
    stmia   r0!, {r4-r11, ip, lr}

#ifdef CONFIG_ARCH_FPU
    vstmia  r0!, {s16-s31}          /* Save the callee-saved FP registers */

    /* Store the floating point control and status register.  At the end of the
     * vstmia, r0 will point to the FPCSR storage location.
     */

    vmrs    r1, fpscr               /* Fetch the FPCSR */
    str     r1, [r0], #4            /* Save the floating point control and status register */
                                    // DSA: don't need to inc r0
#endif /* CONFIG_ARCH_FPU */

    /* we're done, we're out of here */

    mov     r0, #0
    bx      lr

    .size   setjmp, .-setjmp

/************************************************************************************
 * Name: longjmp
 *
 * Description:
 *   The longjmp() function used the information saved in env to transfer control
 *   control back to the point where setjmp() was called and to restore ("rewind")
 *   the stack to its state at the time of the setjmp() call.  When control is
 *   passed back to where setjmp() had been called, setjmp() will return with
 *   'val', the second parameter passed to longjmp().
 *
 * C Function Prototype:
 *   void longjmp(jmp_buf env, int val);
 *
 * Input Parameters:
 *   jmp_buf env
 *   int     val
 *
 * Returned Value:
 *   This function does not return anything explicitly.
 *
 ************************************************************************************/

    .thumb_func
    .type   longjmp, function
longjmp:

    /* Load callee-saved Core registers */

    ldmia   r0!, {r4-r11, ip, lr}
    mov     sp, ip                  /* restore sp */

#ifdef CONFIG_ARCH_FPU
    /* Load callee-saved floating point registers.  */

    vldmia  r0!, {s16-s31}          /* Restore FP context */

    /* Load the floating point control and status register. */

    ldr     r2, [r0], #4            /* Fetch the floating point control and status register */
                                    /* DSA: don't need to inc r0 */
    vmsr    fpscr, r2               /* Restore the FPCSR */
#endif /* CONFIG_ARCH_FPU */

    mov     r0, r1                  /* return val */
    bx      lr

    .size   longjmp, .-longjmp
    .end
