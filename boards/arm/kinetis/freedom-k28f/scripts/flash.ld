/****************************************************************************
 * boards/arm/kinetis/freedom-k28f/scripts/flash.ld
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

/* The MK28FN2M0VMI15 has 2Mb of FLASH beginning at address 0x0000:0000 and
 * 1024Kb of SRAM as follows.

 * This device contains TCRAM that is tightly coupled to the ARM Cortex-M4
 * core as well as OCRAM that is not tightly coupled to ARM Cortex-M4 core.
 * The TCRAM is split into SRAM_L and SRAM_U regions where the SRAM_L and
 * SRAM_U ranges form a contiguous block in the memory map anchored at
 * address 0x20000000. As such:
 *
 * - SRAM_L is anchored at address 0x20000000 and occupies the space below
 *   this address, that is, addresses less than 0x20000000.
 * - SRAM_U is anchored at address 0x2000_0000 and occupies the space at and
 *   above this beginning address, that is, addresses greater than or equal
 *   to 0x20000000.
 *
 * FLASH is arranged as 4 blocks of 512 kB of program flash each block
 * consisting of 4Kb sectors.  The first part of the K28F FLASH region is
 * reserved for interrupt vectflash.
 */

MEMORY
{
  vectflash (rx)  : ORIGIN = 0x00000000, LENGTH = 1K
  cfmprotect (rx) : ORIGIN = 0x00000400, LENGTH = 16
  progflash (rx)  : ORIGIN = 0x00000800, LENGTH = 1M - 2K
  tcm (rwx)       : ORIGIN = 0x1ffc0000, LENGTH = 512K
  ocram (rwx)     : ORIGIN = 0x34000000, LENGTH = 512K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)
EXTERN(__flashconfigbytes)
SECTIONS
{
    .vectors :
    {
        _svectors = ABSOLUTE(.);
        *(.vectors)
        _evectors = ABSOLUTE(.);
    } > vectflash

    .cfmprotect :
    {
        KEEP(*(.cfmconfig))
    } > cfmprotect

    .text :
    {
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
    } > progflash

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        KEEP(*(.init_array .init_array.*))
        _einit = ABSOLUTE(.);
    } > progflash

    .ARM.extab :
    {
        *(.ARM.extab*)
    } > progflash

    .ARM.exidx :
    {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
    } > progflash

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > ocram AT > progflash

    _eronly = LOADADDR(.data);

    .ramfunc ALIGN(4):
    {
        _sramfuncs = ABSOLUTE(.);
        *(.ramfunc  .ramfunc.*)
        _eramfuncs = ABSOLUTE(.);
    } > ocram AT > progflash

    _framfuncs = LOADADDR(.ramfunc);

    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > ocram

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
