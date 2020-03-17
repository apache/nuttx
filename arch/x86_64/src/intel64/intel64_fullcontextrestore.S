/**************************************************************************
 * arch/x86_64/src/intel64/intel64_fullcontextrestore.S
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>
#include "up_internal.h"

    .file    "intel64_fullcontextrestore.S"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/**************************************************************************
 * Public Data
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
 * Public Functions
 **************************************************************************/

    .text

/**************************************************************************
 * Name: up_fullcontextrestore
 *
 * Full C prototype:
 *  void up_fullcontextrestore(uint32_t *regs) noreturn_function;
 *
 **************************************************************************/

    .globl    up_fullcontextrestore
    .type    up_fullcontextrestore, @function
up_fullcontextrestore:
    /* The pointer to the register save array in RDI. */

    /* Disable interrupts now (the correct RFLAGS will be restored before we
     * return
     */

    cli

    /* Create an interrupt stack frame for the final iret.
     *
     *
     *                  IRET STACK
     *               ---------------
     * RSP Before ->
     *                  SS
     *                  RSP
     *                  RFLAGS
     *                  CS
     * RSP After  ->    RIP
     *
     */

    movq    (8*REG_SS)(%rdi), %rbx
    push    %rbx
    movq    (8*REG_RSP)(%rdi), %rbx
    push    %rbx

    movq    (8*REG_RFLAGS)(%rdi), %rbx
    push    %rbx
    movq    (8*REG_CS)(%rdi), %rbx
    push    %rbx
    movq    (8*REG_RIP)(%rdi), %rbx
    push    %rbx

    /* Save the value of RDI on the stack too */

    movq    (8*REG_RDI)(%rdi), %rbx
    push    %rbx

    /* Now restore the remaining registers */
    movq    (8*REG_RSI)(%rdi), %rsi
    movq    (8*REG_RDX)(%rdi), %rdx
    movq    (8*REG_RCX)(%rdi), %rcx
    movq    (8*REG_R8 )(%rdi), %r8
    movq    (8*REG_R9 )(%rdi), %r9

    movq    (8*REG_R15)(%rdi), %r15
    movq    (8*REG_R14)(%rdi), %r14
    movq    (8*REG_R13)(%rdi), %r13
    movq    (8*REG_R12)(%rdi), %r12
    movq    (8*REG_R11)(%rdi), %r11
    movq    (8*REG_R10)(%rdi), %r10
    movq    (8*REG_RBP)(%rdi), %rbp
    movq    (8*REG_RBX)(%rdi), %rbx
    movq    (8*REG_RAX)(%rdi), %rax

    /* Restore the data segment register.  I think there is an issue that will
     * need to be address here at some time:  If the register save area is in
     * one data segment and the stack is in another, then the above would not
     * work (and, conversely, if they are in the same data segment, the
     * following is unnecessary and redundant).
     */

    mov        (8*REG_DS)(%rdi), %ds
    // mov        (8*REG_ES)(%rdi), %es // Not used in 64 bit
    // mov        (8*REG_GS)(%rdi), %gs // Disabled, otherwise we will destroy MSR_GS_BASE
    // mov        (8*REG_FS)(%rdi), %fs // Disabled, otherwise we will destroy MSR_FS_BASE
    // XXX: Should use wrgsbase and wrfsbase to restore the gs and fs register

    /* restore xmm registers */
    fxrstorq (%rdi)

    /* Restore the correct value of EAX and then return */

    popq    %rdi
    iretq
    .size up_fullcontextrestore, . - up_fullcontextrestore
    .end
