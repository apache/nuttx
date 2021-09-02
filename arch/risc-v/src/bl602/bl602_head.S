/****************************************************************************
 * arch/risc-v/src/bl602/bl602_head.S
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
#include <nuttx/irq.h>

    .section .init
    .globl bl602_start
    .globl __start
    .type bl602_start,@function

__start:
bl602_start:
    .cfi_startproc
    .cfi_undefined ra
    .option push
    .option norelax

  /*disable IRQ*/

    li t0, MSTATUS_MIE
    csrc mstatus, t0

    la gp, __global_pointer$
.option pop
    la sp, g_idle_stack
    add s11, sp, zero

    li t0, CONFIG_IDLETHREAD_STACKSIZE
    add sp, sp, t0
    andi sp, sp, ~0xF

#ifndef RUN_IN_RAM

    /* Load boot2 partition address */

    la a0, __boot2_pt_addr_src
    la a1, __boot2_pt_addr_start
    la a2, __boot2_pt_addr_end
    bgeu a1, a2, bl602_entry_load_boot2_partiton_address_end

bl602_entry_load_boot2_partiton_address_loop:
    lw t0, (a0)
    sw t0, (a1)
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a1, a2, bl602_entry_load_boot2_partiton_address_loop

bl602_entry_load_boot2_partiton_address_end:

    /* Load boot2 flashCfg address */

    jal boot2_get_flash_addr
    la a1, __boot2_flash_cfg_start
    la a2, __boot2_flash_cfg_end
    bgeu a1, a2, bl602_entry_load_boot2_flashcfg_address_end

bl602_entry_load_boot2_flashcfg_address_loop:
    lw t0, (a0)
    sw t0, (a1)
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a1, a2, bl602_entry_load_boot2_flashcfg_address_loop

bl602_entry_load_boot2_flashcfg_address_end:
#endif

    /* Load data section */

    la a0, _data_load
    la a1, _data_run
    la a2, _data_run_end
    bgeu a1, a2, bl602_entry_load_data_section_end

bl602_entry_load_data_section_loop:
    lw t0, (a0)
    sw t0, (a1)
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a1, a2, bl602_entry_load_data_section_loop

bl602_entry_load_data_section_end:

    /* Clear bss section */

    la a0, __bss_start
    la a1, __bss_end
    bgeu a0, a1, bl602_entry_zero_bss_end

bl602_entry_zero_bss_loop:
    sw zero, (a0)
    addi a0, a0, 4
    bltu a0, a1, bl602_entry_zero_bss_loop
bl602_entry_zero_bss_end:

    /* Clear bss section */

    la a0, __wifi_bss_start
    la a1, __wifi_bss_end
    bgeu a0, a1, bl602_entry_zero_wifi_bss_end

bl602_entry_zero_wifi_bss_loop:
    sw zero, (a0)
    addi a0, a0, 4
    bltu a0, a1, bl602_entry_zero_wifi_bss_loop
bl602_entry_zero_wifi_bss_end:

    auipc ra, 0
    addi sp, sp, -16
    sw ra, 8(sp)

    /* argc = argv = 0 */

    li a0, 0
    li a1, 0
    call bfl_main

bl602_entry_deadloop:
    j bl602_entry_deadloop

    .cfi_endproc

