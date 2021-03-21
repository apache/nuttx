/****************************************************************************
 * boards/arm/sama5/sama5d3x-ek/scripts/nor-isram.ld
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

/* The SAMA5D3x-EK has 128MB of NOR flash at CS0 (0x1000:0000).  In this
 * configuration, the .text and a copy of the .data section will be loaded
 * into NOR flash.  NuttX .data, .bss, the IDLE stack, and the primary
 * heap will be allocated in SRAM.  The SAMA5D3 has 128 KB of ISRAM beginning
 * at virtual address 0x0030:0000.
 *
 * Vectors in low memory are assumed to reside at the beginning of NOR flash
 * 16KB of ISRAM is reserved at the beginning of ISRAM for the page table.
 */

MEMORY
{
    norflash (W!RX) : ORIGIN = 0x10000000, LENGTH = 128M
    isram    (WRX)  : ORIGIN = 0x00304000, LENGTH = 128K - 16K
}

ENTRY(entry)
ENTRY(_stext)

SECTIONS
{
    .text :
    {
        _stext = ABSOLUTE(.);
        *(.vectors)
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
        *(.ARM.extab*)
        *(.gnu.linkonce.armextab.*)
        _etext = ABSOLUTE(.);
    } > norflash

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        *(.init_array .init_array.*)
        _einit = ABSOLUTE(.);
    } > norflash

    .ARM.extab :
    {
        *(.ARM.extab*)
    } > norflash

    /* .ARM.exidx is sorted, so has to go in its own output section.  */

    .ARM.exidx :
    {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = ABSOLUTE(.);
    } > norflash

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > isram AT > norflash

    _eronly = LOADADDR(.data);

    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > isram

    .ramfunc ALIGN(4):
    {
        _sramfuncs = ABSOLUTE(.);
        *(.ramfunc  .ramfunc.*)
        _eramfuncs = ABSOLUTE(.);
    } > isram AT > norflash

     _framfuncs = LOADADDR(.ramfunc);

    /* Uninitialized data */

    .noinit :
    {
        _snoinit = ABSOLUTE(.);
        *(.noinit*)
        _enoinit = ABSOLUTE(.);
    } > isram

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
