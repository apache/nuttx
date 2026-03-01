/****************************************************************************
 * boards/arm/am67a/t3-gem-o1/scripts/sdram.ld
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

MEMORY
{
  atcm         (rwx): ORIGIN = 0x00000000, LENGTH = 32K      /* Instruction TCM for vectors/boot code */
  btcm         (rwx): ORIGIN = 0x41010000, LENGTH = 32K      /* Data TCM, ideal for stack */
  ddr_rsctable (rwx): ORIGIN = 0xA2100000, LENGTH = 1K       /* Resource Table for RemoteProc/IPC */
  ddr          (rwx): ORIGIN = 0xA2200000, LENGTH = 0xE00000 /* Main DDR RAM */
}

EXTERN(_vector_start)
ENTRY(_vector_start)

SECTIONS
{
    .vectors :
    {
        _stext = ABSOLUTE(.);
        KEEP(*(.vectors))
        KEEP(*arm_head*.o*(.text))
        KEEP(*(.text.arm_boot))
        KEEP(*(.text.am67_mpu_init))
        KEEP(*(.text.mpu_control))
        KEEP(*(.text.cp15_rdsctlr))
        KEEP(*(.text.am67_mpu_reset))
        KEEP(*(.text.mpu_set_region_zero))
        KEEP(*(.text.am67_mpu_disable_br))
        KEEP(*(.text.mpu_configure_region))
        KEEP(*(.text.mpu_allocregion))
        KEEP(*(.text.mpu_modify_region))
        KEEP(*(.text.mpu_set_rgnr))
        KEEP(*(.text.mpu_set_drbar))
        KEEP(*(.text.mpu_log2regionceil))
        KEEP(*(.text.mpu_subregion))
        KEEP(*(.text.mpu_subregion_ms))
        KEEP(*(.text.mpu_subregion_ls))
        KEEP(*(.text.mpu_set_drsr))
        KEEP(*(.text.mpu_set_dracr))
        KEEP(*(.text.cp15_wrsctlr))
        KEEP(*(.text.nx_start))
        KEEP(*(.tick_timer))
        KEEP(*(.text.nxsched_process_timer))
        KEEP(*(.text.clock_timer))
        KEEP(*(.text.wd_timer))
        KEEP(*(.text.clock_systime_ticks))
    } > atcm

    .resource_table :
    {
        KEEP(*(.resource_table))
        . = ALIGN(1024);
    } > ddr_rsctable

    .text :
    {
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.glue_7)
        *(.glue_7t)
        *(.gcc_except_table)
    } > ddr

    .rodata :
    {
        *(.rodata .rodata.*)
        *(.gnu.linkonce.r.*)
        *(.got)
    } > ddr

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > ddr

    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > ddr

    _etext = ABSOLUTE(.); /* End of code/read-only sections */
    _sidata = _etext;    /* Start of initial values for .data (LMA) */

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > ddr

    .data : AT ( _sidata )
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > ddr

    .noinit (NOLOAD) :
    {
        _snoinit = ABSOLUTE(.);
        *(.noinit*)
        _enoinit = ABSOLUTE(.);
    } > ddr

    .bss (NOLOAD) :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > ddr

    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        _sheap = ABSOLUTE(.);
        _end = _sheap;
        . = . + 64K;
        _eheap = ABSOLUTE(.);
    } > ddr

    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        _sstack = ABSOLUTE(.);
        . = . + 8K;
        _estack = ABSOLUTE(.);
    } > btcm

    /* Stabs debugging sections. */

    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}
