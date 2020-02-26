/**************************************************************************
 * arch/x86/src/intel64/intel64_saveusercontext.S
 *
 *   Copyright (C) 2011 Gregory Nutt,
 *                 2020 Chung-Fan Yang.
 *   All rights reserved.

 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Chung-Fan Yang <sonic.tw.tp@gmail.com>
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>
#include "up_internal.h"

    .file    "intel64_saveusercontext.S"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/****************************************************************************
 * Macros
 ****************************************************************************/

/* Trace macros, use like trace 'i' to print char to serial port. */

    .macro    chout, addr, ch
#ifdef CONFIG_DEBUG_FEATURES
    mov        $\addr, %dx
    mov        $\ch, %al
    out        %al, %dx
#endif
    .endm

    .macro    trace, ch
#ifdef CONFIG_DEBUG_FEATURES
    push    %eax
    push    %edx
    chout    0x3f8, \ch
    pop        %edx
    pop        %eax
#endif
    .endm

/**************************************************************************
 * .text
 **************************************************************************/

    .text
.code64
/**************************************************************************
 * Name: up_saveusercontext
 *
 * Full C prototype:
 *  int up_saveusercontext(uint32_t *regs);
 *
 * Description:
 *  Save the "user" context.  It is not necessary to save all of the
 *  registers because it is acceptable for certain registers to be
 *  modified upon return from a subroutine call.  On a context switch
 *  back to user mode, it will appear as a return from this function.
 *
 *  According to the SysV x86_64 ABI, the RAX, RDI, RSI, RDX, RCX, r8 and r9 
 *  are to be free for use within a procedure or function, and need not 
 *  be preserved.
 *
 *  On entry,
 *    sp points to the return address
 *    rdi points to register save array
 *
 **************************************************************************/

    .globl    up_saveusercontext
    .type    up_saveusercontext, @function
up_saveusercontext:
    
    // callee saved regs
    movq    %rbx, (8*REG_RBX)(%rdi)
    movq    %r12, (8*REG_R12)(%rdi)
    movq    %r13, (8*REG_R13)(%rdi)
    movq    %r14, (8*REG_R14)(%rdi)
    movq    %r15, (8*REG_R15)(%rdi)

    // save xmm registers
    fxsaveq (%rdi)

    /* Save the value of SP as will be at the time of the IRET that will
     * appear to be the return from this function.
     *
     *         CURRENT STACK           IRET STACK
     *        ------------------------------        -----------------
     *                              RIP
     *                              CS
     *                              RFLAGS
     *                              RSP
     * ESP->Return address          SS
     * Argument                     Alignment (16bytes)
     *
     */

    leaq    8(%rsp), %rcx
    movq    %rcx, (8*REG_RSP)(%rdi)

    /* Fetch the PC from the stack and save it in the save block */

    movq    0(%rsp), %rcx
    movq    %rcx, (8*REG_RIP)(%rdi)

    /* Save the framepointer */

    movq    %rbp, (8*REG_RBP)(%rdi)

    /* Save EAX=1.  This will be the "apparent" return value from this
     * function when context is switch back to this thread.  The non-zero
     * return value is the indication that we have been resumed.
     */

    movq    $1, (8*REG_RAX)(%rdi)

    /* Get and save the interrupt state */

    pushf
    pop     %rcx
    movq    %rcx, (8*REG_RFLAGS)(%rdi)

    /* And return 0 -- The zero return value is the indication that that
     * this is the original, "true" return from the function.
     *
     * 'ret' will remove the RIP from the top of the stack.
     */

    xor     %rax, %rax
    ret
    .size    up_saveusercontext, . - up_saveusercontext
    .end
