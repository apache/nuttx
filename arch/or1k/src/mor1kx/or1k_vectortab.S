/****************************************************************************
 * arch/or1k/src/mor1kx/or1k_vectortab.S
 *
 *   Copyright (C) 2018 Extent3D. All rights reserved.
 *   Author: Matt Thompson <matt@extent3d.com>
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>
#include <arch/spr.h>
#include <arch/arch.h>

#define EXCEPTION_STACK_SIZE (4*XCPTCONTEXT_REGS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LED_BASE (0x91000000)
#define LED_DIR (LED_BASE+1)

/* For debugging, we'll use a dedicated IRQ stack at 8MB */

#define IRQ_STACK (0x800000)

/* For now, we'll store interrupt context in RAM
 * at the following offsets. This must fit before 0x100.
 */

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

#define _load_symbol(gpr, symbol) \
  l.movhi gpr,hi(symbol); \
  l.ori   gpr,gpr,lo(symbol)

#if 0
.macro _led_init
  l.movhi r3,hi(LED_BASE);
  l.addi  r4,r0,0x80;
  l.sb    0(r3),r4;
  l.addi  r4,r0,0x80;
  l.sb    4(r3),r4
.endm
#endif

/* Disable the programmable interrupt controller */

.macro _pic_disable
  l.mtspr r0,r0,SPR_PIC_MR;
  l.mtspr r0,r0,SPR_PIC_SR;
.endm

.macro _save_registers r
  l.sw    4*REG_R1(\r), r1;
  l.sw    4*REG_R2(\r), r2;
  l.sw    4*REG_R3(\r), r3;
  l.sw    4*REG_R4(\r), r4;
  l.sw    4*REG_R5(\r), r5;
  l.sw    4*REG_R6(\r), r6;
  l.sw    4*REG_R7(\r), r7;
  l.sw    4*REG_R8(\r), r8;
  l.sw    4*REG_R9(\r), r9;
  l.sw    4*REG_R10(\r), r10;
  l.sw    4*REG_R11(\r), r11;
  l.sw    4*REG_R12(\r), r12;
  l.sw    4*REG_R13(\r), r13;
  l.sw    4*REG_R14(\r), r14;
  l.sw    4*REG_R15(\r), r15;
  l.sw    4*REG_R16(\r), r16;
  l.sw    4*REG_R17(\r), r17;
  l.sw    4*REG_R18(\r), r18;
  l.sw    4*REG_R19(\r), r19;
  l.sw    4*REG_R20(\r), r20;
  l.sw    4*REG_R21(\r), r21;
  l.sw    4*REG_R22(\r), r22;
  l.sw    4*REG_R23(\r), r23;
  l.sw    4*REG_R24(\r), r24;
  l.sw    4*REG_R25(\r), r25;
  l.sw    4*REG_R26(\r), r26;
  l.sw    4*REG_R27(\r), r27;
  l.sw    4*REG_R28(\r), r28;
  l.sw    4*REG_R29(\r), r29;
  l.sw    4*REG_R30(\r), r30;
  l.sw    4*REG_R31(\r), r31;
.endm

.macro _restore_registers r
  l.lwz   r2, 4*REG_R2(\r);
  l.lwz   r3, 4*REG_R3(\r);
  l.lwz   r4, 4*REG_R4(\r);
  l.lwz   r5, 4*REG_R5(\r);
  l.lwz   r6, 4*REG_R6(\r);
  l.lwz   r7, 4*REG_R7(\r);
  l.lwz   r8, 4*REG_R8(\r);
  l.lwz   r9, 4*REG_R9(\r);
  l.lwz   r10, 4*REG_R10(\r);
  l.lwz   r11, 4*REG_R11(\r);
  l.lwz   r12, 4*REG_R12(\r);
  l.lwz   r13, 4*REG_R13(\r);
  l.lwz   r14, 4*REG_R14(\r);
  l.lwz   r15, 4*REG_R15(\r);
  l.lwz   r16, 4*REG_R16(\r);
  l.lwz   r17, 4*REG_R17(\r);
  l.lwz   r18, 4*REG_R18(\r);
  l.lwz   r19, 4*REG_R19(\r);
  l.lwz   r20, 4*REG_R20(\r);
  l.lwz   r21, 4*REG_R21(\r);
  l.lwz   r22, 4*REG_R22(\r);
  l.lwz   r23, 4*REG_R23(\r);
  l.lwz   r24, 4*REG_R24(\r);
  l.lwz   r25, 4*REG_R25(\r);
  l.lwz   r26, 4*REG_R26(\r);
  l.lwz   r27, 4*REG_R27(\r);
  l.lwz   r28, 4*REG_R28(\r);
  l.lwz   r29, 4*REG_R29(\r);
  l.lwz   r30, 4*REG_R30(\r);
  l.lwz   r31, 4*REG_R31(\r);
  l.lwz   r1, 4*REG_R1(\r);
.endm

.macro _grow_stack
  l.sw    -8(r1),r2;
  l.addi  r2,r1,0;
  l.sw    -4(r1),r9;
  l.addi  r1,r1,-8;
  l.addi  r1,r1,-EXCEPTION_STACK_SIZE;
.endm

#if 0
.macro _disable_interrupts
  l.mfspr r30,r0,SPR_SYS_SR;
  l.movhi r31,hi(~(SPR_SR_IEE|SPR_SR_TEE));
  l.ori   r31,r31,lo(~(SPR_SR_IEE|SPR_SR_TEE));
  l.and   r30,r30,r31;
  l.mtspr r0,r30,SPR_SYS_SR;
.endm

.macro _enable_interrupts
  l.mfspr r30,r0,SPR_SYS_SR;
  l.ori   r30,r30,lo(SPR_SR_IEE|SPR_SR_TEE);
  l.mtspr r0,r30,SPR_SYS_SR;
.endm
#endif

/****************************************************************************
 * Name: _vectors
 *
 * Description:
 *   Exception vectors
 ****************************************************************************/

  .section  .vectors, "ax"
  .align    2
  .global   _vectors
  .type     _vectors, function

_vectors:
_vector_start:

  /* Reset Exception
   *
   * The CPU will begin execution here.
   */

  .org 0x100
_reset_vector:
  l.movhi r0, 0x0;
  l.movhi r1, 0x0;
  l.movhi r2, 0x0;
  l.movhi r3, 0x0;
  l.movhi r4, 0x0;
  l.movhi r5, 0x0;
  l.movhi r6, 0x0;
  l.movhi r7, 0x0;
  l.movhi r8, 0x0;
  l.movhi r9, 0x0;
  l.movhi r10, 0x0;
  l.movhi r11, 0x0;
  l.movhi r12, 0x0;
  l.movhi r13, 0x0;
  l.movhi r14, 0x0;
  l.movhi r15, 0x0;
  l.movhi r16, 0x0;
  l.movhi r17, 0x0;
  l.movhi r18, 0x0;
  l.movhi r19, 0x0;
  l.movhi r20, 0x0;
  l.movhi r21, 0x0;
  l.movhi r22, 0x0;
  l.movhi r23, 0x0;
  l.movhi r24, 0x0;
  l.movhi r25, 0x0;
  l.movhi r26, 0x0;
  l.movhi r27, 0x0;
  l.movhi r28, 0x0;
  l.movhi r29, 0x0;
  l.movhi r30, 0x0;
  l.movhi r31, 0x0;

  l.ori   r1,r0,SPR_SR_SM;
  l.mtspr r0,r1,SPR_SYS_SR;

  /* Disable external interrupts */

  _pic_disable;

  /* Initialize LED GPIO early to get some feedback */

  //_led_init;

  /* Clear Tick Timer SPR */

  l.mtspr r0,r0,SPR_TICK_TTMR;
  l.mtspr r0,r0,SPR_TICK_TTCR;

  /* Set the stack pointer */

  l.movhi r1,hi(_ebss);
  l.ori   r1,r1,lo(_ebss);
  l.addi  r1,r1,CONFIG_IDLETHREAD_STACKSIZE;
  l.ori   r2,r1,0;
  l.nop

  /* Jump to __start */

  _load_symbol(r13, __start);
  l.jr    r13;
  l.nop;

  /* Should never reach here */

  l.j 0;
  l.nop;

  /* Bus exception */

  .org 0x200
_bus_fault:
  l.nop;
  l.j 0;
  l.nop;

  /* Data page fault */

  .org 0x300
_data_page_fault:
  l.nop;
  l.j 0;
  l.nop;

  /* Instruction page fault */

  .org 0x400
_instruction_page_fault:
  l.nop;
  l.j 0;
  l.nop;

  /* Tick Timer */

  .org 0x500
_tick_timer:
  //_grow_stack;
  _save_registers r0;

  l.movhi r1,hi(IRQ_STACK);
  l.ori   r1,r1,lo(IRQ_STACK);
  l.ori   r2,r1,0;

  l.ori r3,r0,OR1K_IRQ_TICK
  l.j _handler_entry;
  l.nop;

  /* Alignment exception */

  .org 0x600
_alignment_fault:
  l.nop;
  l.j 0;
  l.nop;

  /* Illegal Instruction exception */

  .org 0x700
_illegal_instruction:
  l.nop;
  l.j 0;
  l.nop;

  /* External interrupt */

  .org 0x800
_external_interrupt:
  //_grow_stack;
  _save_registers r0;

  l.movhi r1,hi(IRQ_STACK);
  l.ori   r1,r1,lo(IRQ_STACK);
  l.ori   r2,r1,0;

  l.ori r3,r0,OR1K_IRQ_EXT2;
  l.j _handler_entry;
  l.nop;

  /* Data TLB Miss */

  .org 0x900
_dtlb_miss:
  l.nop;
  l.j 0;
  l.nop;

  /* Instruction TLB Miss */

  .org 0xA00
_itlb_miss:
  l.nop;
  l.j 0;
  l.nop;

  /* Range exception */

  .org 0xB00
_range_exception:
  l.nop;
  l.j 0;
  l.nop;

  /* System Call */

  .org 0xC00
_syscall_vector:
  l.nop;
  l.j 0;
  l.nop;

  /* Floating Point */

  .org 0xD00
_fpu_vector:
  l.nop;
  l.j 0;
  l.nop;

  /* Trap */

  .org 0xE00
_trap_vector:
  l.nop;
  l.j 0;
  l.nop;

_vector_end:

_handler_entry:

  /* Pointer to registers */

  l.addi  r4,r1,0;

  l.jal   or1k_doirq;
  l.nop;

  //l.ori   r1,r2,0;
  //l.lwz   r2, -4(r1);

  /* Restore saved registers from RAM */

  _restore_registers r0;

  /* Return from exception */

  l.rfe;

  .end
