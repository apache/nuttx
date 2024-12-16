/****************************************************************************
 * boards/arm/imx95/imx95-evk/scripts/itcm.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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
 *
 ****************************************************************************/

/*
 * Flash controller
 *    1) 32 MB of FlexSPI_1 at 0x0200:0000
 * The on-chip SRAM is split in these parts:
 *    1) 256Kb of program memory located at 0x0000:0000
 *    2) 256Kb of data memory beginning at the address 0x2000:0000
 *    3) 352Kb of OCRAM beginning at the address 0x2048:0000
 *    4) 4Mb of SDRAM located at 0x9000:0000
 */

MEMORY
{
  m_interrupts (rx)  : ORIGIN = 0x90000000, LENGTH = 0x00000800
  flash (rx)  : ORIGIN = 0x90000800, LENGTH = 0x003FF800
  sram  (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
  ocram (rwx) : ORIGIN = 0x20480000, LENGTH = 352K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

SECTIONS
{
    .interrupts :
    {
        __VECTOR_TABLE = .;
        __Vectors = .;
        . = ALIGN(4);
        KEEP(*(.vectors))     /* Startup code */
        . = ALIGN(4);
    } > m_interrupts

    .text : {
        _stext = ABSOLUTE(.);
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } > flash

    .resource_table :
    {
        . = ALIGN(8);
        KEEP(*(.resource_table))
        . = ALIGN(4);
    } > flash

    .init_section : {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array .ctors))
        _einit = ABSOLUTE(.);
    } > flash

    .ARM.extab : {
        *(.ARM.extab*)
    } > flash

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx : {
        *(.ARM.exidx*)
    } > flash
    __exidx_end = ABSOLUTE(.);

    _eronly = ABSOLUTE(.);

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram AT > flash

    .bss : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > sram

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

    _ram_start = ORIGIN(sram);
    _ram_size = LENGTH(sram);
    _ram_end = ORIGIN(sram) + LENGTH(sram);
    _ocram_start = ORIGIN(ocram);
    _ocram_size = LENGTH(ocram);
    _ocram_end = ORIGIN(ocram) + LENGTH(ocram);
}
