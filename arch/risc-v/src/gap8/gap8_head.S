/****************************************************************************
 * arch/risc-v/src/gapuino/gap8_head.S
 * Startup file for FC of GAP8
 *  Interrupt vector and reset handler
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: hhuysqt <1020988872@qq.com>
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Exception context size: EPC + 31 common regs + 6 loop regs */

#define EXCEPTION_STACK_SIZE 4*38

/****************************************************************************
 * Assembler Macro Definitions
 ****************************************************************************/

/* save all the registers on interrupt entry */

  .macro SAVE_REGS
    addi sp, sp, -EXCEPTION_STACK_SIZE
    sw  x1,  1*4(sp)  /* ra */
    sw  x3,  3*4(sp)  /* gp */
    sw  x4,  4*4(sp)  /* tp */
    sw  x5,  5*4(sp)  /* t0 */
    sw  x6,  6*4(sp)  /* t1 */
    sw  x7,  7*4(sp)  /* t2 */
    sw  x8,  8*4(sp)  /* s0 */
    sw  x9,  9*4(sp)  /* s1 */
    sw x10, 10*4(sp)  /* a0 */
    sw x11, 11*4(sp)  /* a1 */
    sw x12, 12*4(sp)  /* a2 */
    sw x13, 13*4(sp)  /* a3 */
    sw x14, 14*4(sp)  /* a4 */
    sw x15, 15*4(sp)  /* a5 */
    sw x16, 16*4(sp)  /* a6 */
    sw x17, 17*4(sp)  /* a7 */
    sw x18, 18*4(sp)  /* s2 */
    sw x19, 19*4(sp)  /* s3 */
    sw x20, 20*4(sp)  /* s4 */
    sw x21, 21*4(sp)  /* s5 */
    sw x22, 22*4(sp)  /* s6 */
    sw x23, 23*4(sp)  /* s7 */
    sw x24, 24*4(sp)  /* s8 */
    sw x25, 25*4(sp)  /* s9 */
    sw x26, 26*4(sp)  /* s10 */
    sw x27, 27*4(sp)  /* s11 */
    sw x28, 28*4(sp)  /* t3 */
    sw x29, 29*4(sp)  /* t4 */
    sw x30, 30*4(sp)  /* t5 */
    sw x31, 31*4(sp)  /* t6 */
    csrr x28, 0x7B0
    csrr x29, 0x7B1
    csrr x30, 0x7B2
    sw x28, 32*4(sp)  /* lpstart[0] */
    sw x29, 33*4(sp)  /* lpend[0] */
    sw x30, 34*4(sp)  /* lpcount[0] */
    csrr x28, 0x7B4
    csrr x29, 0x7B5
    csrr x30, 0x7B6
    sw x28, 35*4(sp)  /* lpstart[1] */
    sw x29, 36*4(sp)  /* lpend[1] */
    sw x30, 37*4(sp)  /* lpcount[1] */
    addi s0, sp, EXCEPTION_STACK_SIZE
    sw  s0,  2*4(sp)   /* original SP */
  .endm

/* restore regs and `mret` */

  .macro RESTORE_REGS
    lw x28, 35*4(sp)  /* lpstart[1] */
    lw x29, 36*4(sp)  /* lpend[1] */
    lw x30, 37*4(sp)  /* lpcount[1] */
    csrrw x0, 0x7B4, x28
    csrrw x0, 0x7B5, x29
    csrrw x0, 0x7B6, x30
    lw x28, 32*4(sp)  /* lpstart[0] */
    lw x29, 33*4(sp)  /* lpend[0] */
    lw x30, 34*4(sp)  /* lpcount[0] */
    csrrw x0, 0x7B0, x28
    csrrw x0, 0x7B1, x29
    csrrw x0, 0x7B2, x30
    li s0, 0x1890     /* machine mode, UPIE & MPIE enabled */
    csrrw x0, mstatus, s0
    lw  x3,  3*4(sp)  /* gp */
    lw  x4,  4*4(sp)  /* tp */
    lw  x5,  5*4(sp)  /* t0 */
    lw  x6,  6*4(sp)  /* t1 */
    lw  x7,  7*4(sp)  /* t2 */
    lw  x8,  8*4(sp)  /* s0 */
    lw  x9,  9*4(sp)  /* s1 */
    lw x10, 10*4(sp)  /* a0 */
    lw x11, 11*4(sp)  /* a1 */
    lw x12, 12*4(sp)  /* a2 */
    lw x13, 13*4(sp)  /* a3 */
    lw x14, 14*4(sp)  /* a4 */
    lw x15, 15*4(sp)  /* a5 */
    lw x16, 16*4(sp)  /* a6 */
    lw x17, 17*4(sp)  /* a7 */
    lw x18, 18*4(sp)  /* s2 */
    lw x19, 19*4(sp)  /* s3 */
    lw x20, 20*4(sp)  /* s4 */
    lw x21, 21*4(sp)  /* s5 */
    lw x22, 22*4(sp)  /* s6 */
    lw x23, 23*4(sp)  /* s7 */
    lw x24, 24*4(sp)  /* s8 */
    lw x25, 25*4(sp)  /* s9 */
    lw x26, 26*4(sp)  /* s10 */
    lw x27, 27*4(sp)  /* s11 */
    lw x28, 28*4(sp)  /* t3 */
    lw x29, 29*4(sp)  /* t4 */
    lw x30, 30*4(sp)  /* t5 */
    lw x31, 31*4(sp)  /* t6 */

    lw  x1,  1*4(sp)  /* ra */

    lw  sp,  2*4(sp)  /* restore original sp */
  .endm

/* wrapper for IRQ vector */

  .macro WRAP_IRQ Routine, IRQn
  wrap_irq_\Routine :
    SAVE_REGS

    csrr s0, mepc
    sw  s0,  0(sp)   /* exception PC */

    li a0, \IRQn     /* irq = IRQn */
    mv a1, sp        /* context = sp */
    jal x1, gap8_dispatch_irq

    /* If context switch is needed, return
     * a new sp
     */

    mv sp, a0

    lw  s0, 0(sp)    /* restore ePC */
    csrw mepc, s0

    RESTORE_REGS

    mret
  .endm


/*******************************************************************************
 * External Variables and Functions
 *******************************************************************************/

  .extern _sbss
  .extern _ebss
  .extern _idle_stack_end

  .extern gap8_dispatch_irq
  .extern nx_start
  .extern gapuino_sysinit

/*******************************************************************************
 * Reset handler
 *******************************************************************************/

reset_handler:
#if 0
  csrr a0, 0xf14 /* Cluster ID */
  srli a0, a0, 5
#endif

  li a0, 0x1800 /* Set MSTATUS : Machine Mode */
  csrw mstatus, a0

  li a0, 0x1C000000 /* Set MTVEC */
  csrw mtvec, a0

  /* Stack initialization */

  la  x2, _idle_stack_end

  /* Clear BSS */

  la x26, _sbss
  la x27, _ebss

  bge x26, x27, zero_loop_end

zero_loop:
  sw x0, 0(x26)
  addi x26, x26, 4
  ble x26, x27, zero_loop

zero_loop_end:

  /* TODO: initialize data section */

  /* Initialize cache and clock */

  jal x1, gapuino_sysinit

  /* Directly call NuttX nx_start() */

  jal x1, nx_start

  /* If it ever returns, spin here forever... */

dead_loop:
  jal x0, dead_loop


/* IRQ wrappers
 *  IRQn are identical to gap8_interrupt.h
 */

WRAP_IRQ sw_evt0,   0
WRAP_IRQ sw_evt1,   1
WRAP_IRQ sw_evt2,   2
WRAP_IRQ sw_evt3,   3
WRAP_IRQ sw_evt4,   4
WRAP_IRQ sw_evt5,   5
WRAP_IRQ sw_evt6,   6
WRAP_IRQ sw_evt7,   7

WRAP_IRQ timer_lo,  10
WRAP_IRQ timer_hi,  11

WRAP_IRQ udma,      27
WRAP_IRQ mpu,       28
WRAP_IRQ udma_err,  29
WRAP_IRQ fc_hp0,    30
WRAP_IRQ fc_hp1,    31

WRAP_IRQ reserved,  60

/* RISCV exceptions */

illegal_insn_handler:
  csrr s0, mepc
  sw  s0, 0*4(sp)   /* exception PC */

  /* Spin here so that debugger would read `s0` */

1:
  j 1b

/* Systemcall handler */

ecall_insn_handler:
  SAVE_REGS

  /* Point to the next instruction of `ecall` */

  csrr s0, mepc
  addi s0, s0, 4
  sw  s0,  0(sp)   /* exception PC */

  li a0, 34        /* irq = 34 */
  mv a1, sp        /* context = sp */
  jal x1, gap8_dispatch_irq

  /* If context switch is needed, return
   * a new sp
   */

  mv sp, a0

  lw  s0, 0(sp)    /* restore ePC */
  csrw mepc, s0

  RESTORE_REGS

  mret

/*******************************************************************************
 *  INTERRUPT VECTOR TABLE
 *******************************************************************************/

  /* This section has to be down here, since we have to disable rvc for it  */

  .section .vectors_M, "ax"
  .option norvc;

  j wrap_irq_sw_evt0    /* 0 */
  j wrap_irq_sw_evt1    /* 1 */
  j wrap_irq_sw_evt2    /* 2 */
  j wrap_irq_sw_evt3    /* 3 */
  j wrap_irq_sw_evt4    /* 4 */
  j wrap_irq_sw_evt5    /* 5 */
  j wrap_irq_sw_evt6    /* 6 */
  j wrap_irq_sw_evt7    /* 7 */
  j wrap_irq_reserved   /* 8 */
  j wrap_irq_reserved   /* 9 */
  j wrap_irq_timer_lo   /* 10 */
  j wrap_irq_timer_hi   /* 11 */
  j wrap_irq_reserved   /* 12 */
  j wrap_irq_reserved   /* 13 */
  j wrap_irq_reserved   /* 14 */
  j wrap_irq_reserved   /* 15 */
  j wrap_irq_reserved   /* 16 */
  j wrap_irq_reserved   /* 17 */
  j wrap_irq_reserved   /* 18 */
  j wrap_irq_reserved   /* 19 */
  j wrap_irq_reserved   /* 20 */
  j wrap_irq_reserved   /* 21 */
  j wrap_irq_reserved   /* 22 */
  j wrap_irq_reserved   /* 23 */
  j wrap_irq_reserved   /* 24 */
  j wrap_irq_reserved   /* 25 */
  j wrap_irq_reserved   /* 26 */
  j wrap_irq_udma       /* 27 */
  j wrap_irq_mpu        /* 28 */
  j wrap_irq_udma_err   /* 29 */
  j wrap_irq_fc_hp0     /* 30 */
  j wrap_irq_fc_hp1     /* 31 */

  j reset_handler       /* 32 */
  j illegal_insn_handler/* 33 */
  j ecall_insn_handler  /* 34 */

/****************************************************************************
 * This variable is pointed to the structure containing all information
 * exchanged with the platform loader. It is using a fixed address so that
 * the loader can also find it and then knows the address of the debug
 * structure.
 ****************************************************************************/

  .section .dbg_struct, "ax"
  .option norvc;
  .org 0x90
  .global __rt_debug_struct_ptr
__rt_debug_struct_ptr:
  .word Debug_Struct

/****************************************************************************
 * This global variable is unsigned int g_idle_topstack and is exported here
 * only because of its coupling to idle thread stack.
 ****************************************************************************/

  .section .data
  .global  g_idle_topstack
g_idle_topstack:
  .word  _idle_stack_end
