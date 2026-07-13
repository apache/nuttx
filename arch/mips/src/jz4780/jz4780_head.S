/****************************************************************************
 * arch/mips/src/jz4780/jz4780_head.S
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/config.h>

#include <arch/mips32/registers.h>
#include <arch/jz4780/cp0.h>

#include "jz4780_excptmacros.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define JZ4780_STACK_BASE      _ebss
#define JZ4780_STACK_TOP       _ebss + CONFIG_IDLETHREAD_STACKSIZE - 8

#define JZ4780_HEAP_BASE       JZ4780_STACK_TOP

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file       "jz4780_head.S"

    /* Exported symbols */

    .global     __reset
    .global     __start
    .global     halt
    .global     g_idle_topstack

    /* Imported symbols */

    .global     nx_start
    .global     jz4780_exception
    .global     jz4780_decodeirq
    .global     jz4780_swint0

    /* This file contains 32-bit assembly code */

    .set        nomips16


/****************************************************************************
 * Name: _gen_exception
 *
 * Description:
 *   General Exception Vector Handler.  Jumps to _exception_handler.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

    .section    .gen_excpt,"ax",@progbits
    .set        noreorder
    .ent        _gen_exception

_gen_exception:
    la          k0, _exception_handler
    jr          k0
    nop
    .end        _gen_exception

/****************************************************************************
 * Name: _ebase_exception
 *
 * Description:
 *   Interrupt Exception Vector Handlers.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

    .section    .ebase_excpt,"ax",@progbits
    .set        noreorder
    .ent        _ebase_exception

_ebase_exception:
    la          k0, _swint_handler
    jr          k0
    nop
    .end        _ebase_exception


    .section    .ebase_excpt1,"ax",@progbits
    .set        noreorder
    .ent        _ebase_exception1

_ebase_exception1:
    la          k0, _int_handler
    jr          k0
    nop
    .end        _ebase_exception1


    .section    .ebase_excpt2,"ax",@progbits
    .set        noreorder
    .ent        _ebase_exception2

_ebase_exception2:
    la          k0, _int_handler
    jr          k0
    nop
    .end        _ebase_exception2


    .section    .ebase_excpt3,"ax",@progbits
    .set        noreorder
    .ent        _ebase_exception3

_ebase_exception3:
    la          k0, _int_handler
    jr          k0
    nop
    .end        _ebase_exception3


/****************************************************************************
 * Name: _bev_exception
 *
 * Description:
 *   Boot Exception Vector Handler.  Jumps to _exception_handler.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

    .section    .bev_excpt,"ax",@progbits
    .set        noreorder
    .ent        _bev_exception

_bev_exception:
    la          k0, _exception_handler
    jr          k0
    nop
    .end        _bev_exception

/****************************************************************************
 * Name: _int_exception
 *
 * Description:
 *   Interrupt Exception Vector Handler.  Jumps to _int_handler.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

    .section    .int_excpt,"ax",@progbits
    .set        noreorder
    .ent        _int_exception

_int_exception:
    la          k0, _int_handler
    jr          k0
    nop
    .end        _int_exception

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   This is the KSEG0 startup code.  It receives control from the reset
 *   entry point.  This prepares the processor to execute C code, performs
 *   some very low-level initialization, then starts NuttX (via __start_nuttx)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

    .section    .start, "ax", @progbits
    .set        noreorder
    .ent        __start

__start:

    /* Initialize the stack pointer */

    la          sp, JZ4780_STACK_TOP

    /* Initialize the global pointer (gp).  _gp is initialized by the linker
     * script to point to the "middle" of the small variables region.
     */

    la          gp, _gp

    /* Clear uninitialized data sections */

    la          t0, _sbss
    la          t1, _ebss
    b           .Lbsscheck
    nop

.Lbssloop:
    sw          zero, 0x0(t0)
    sw          zero, 0x4(t0)
    sw          zero, 0x8(t0)
    sw          zero, 0xc(t0)
    addu        t0, 16

.Lbsscheck:
    bltu        t0, t1, .Lbssloop
    nop

    /* Copy initialized data from program flash to data memory */

    la          t0, _data_loaddr
    la          t1, _sdata
    la          t2, _edata
    b           .Ldatacheck
    nop

.Ldataloop:
    lw          t3, (t0)
    sw          t3, (t1)
    addu        t0, 4
    addu        t1, 4

.Ldatacheck:
    bltu        t1, t2, .Ldataloop
    nop

    /* Start NuttX. We do this via a thunk in the text section so that
     * a normal jump and link can be used, enabling the startup code
     * to work properly whether main is written in MIPS16 or MIPS32
     * code. I.e., the linker will correctly adjust the JAL to JALX if
     * necessary
     */

    la          t0, __start_nuttx
    jr          t0
    nop
    .end        __start

/****************************************************************************
 * Name: _exception_handler
 *
 * Description:
 *   BEV/General exception handler.  Calls jz4780_exception()
 *
 ****************************************************************************/

    .section    .bev_handler, "ax", @progbits
    .set        noreorder
    .ent        _exception_handler

_exception_handler:
    EXCPT_PROLOGUE t0
    move        a0, sp
    la          t0, jz4780_exception
    jalr        ra, t0
    nop
    EXCPT_EPILOGUE v0
    .end        _exception_handler

/****************************************************************************
 * Name: _swint_handler
 *
 * Description:
 *   Interrupt exception handler.  Calls up_decodeirq()
 *
 ****************************************************************************/

    .section    .swint_handler, "ax", @progbits
    .set        noreorder
    .ent        _swint_handler

_swint_handler:
    EXCPT_PROLOGUE t0
    move        a0, sp
    la          t0, jz4780_swint0
    jalr        ra, t0
    nop
    EXCPT_EPILOGUE v0
    .end        _swint_handler


/****************************************************************************
 * Name: _int_handler
 *
 * Description:
 *   Interrupt exception handler.  Calls up_decodeirq()
 *
 ****************************************************************************/

    .section    .int_handler, "ax", @progbits
    .set        noreorder
    .ent        _int_handler

_int_handler:
    EXCPT_PROLOGUE t0
    move        a0, sp
    la          t0, jz4780_decodeirq
    jalr        ra, t0
    nop
    EXCPT_EPILOGUE v0
    .end        _int_handler

/****************************************************************************
 * Name: __start_nuttx
 *
 * Description:
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

    .text
    .ent        __start_nuttx

__start_nuttx:

    /* Perform low level initialization */

    la          t0, jz4780_lowinit
    jalr        ra, t0
    nop

    /* Call nx_start */

    la          t0, nx_start
    jalr        ra, t0
    nop

    /* Just in case main returns, go into an infinite loop */

halt:
1:
    b           1b
    nop
    .end        __start_nuttx

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This global variable is unsigned int g_idle_topstack and is exported here only
 * because of its coupling to idle thread stack.
 */

    .sdata
    .type       g_idle_topstack, object
g_idle_topstack:
    .long       JZ4780_HEAP_BASE
    .size       g_idle_topstack, .-g_idle_topstack
