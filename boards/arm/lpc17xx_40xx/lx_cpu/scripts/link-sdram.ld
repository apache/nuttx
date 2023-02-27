/****************************************************************************
 * boards/arm/lpc17xx_40xx/lx_cpu/scripts/link-sdram.ld
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

/* The LPC1788 has 512Kb of FLASH beginning at address 0x0000:0000 and
 * 96Kb of total SRAM: 64Kb of SRAM in the CPU block beginning at address
 * 0x10000000 and 32Kb of Peripheral SRAM in two banks, 8Kb at addresses
 * 0x20000000 bank0 first and 8kb at 0x20002000 at bank0 second. And 16Kb
 * at 0x20004000 on bank1.
 *
 * Here we assume that .data and .bss will all fit into the 64Kb CPU SRAM
 * address range.
 */

MEMORY
{
    FLASHBOOT (rx) : ORIGIN = 0x00000000, LENGTH = 0x00007000
    KEYVAL (rx) : ORIGIN = 0x00007000, LENGTH = 0x00002000
    FLASH (rx) : ORIGIN = 0x00009000, LENGTH = 0x00077000
    SRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
    AHBRAM8_B0A(rwx): ORIGIN = 0x20000000, LENGTH = 8K
    AHBRAM8_B0B(rwx): ORIGIN = 0x20002000, LENGTH = 8K
    AHBRAM16(rwx): ORIGIN = 0x20004000, LENGTH = 16K
    /* External SDRAM - 32MB for LX_CPU */
    SDRAM (rwx) : ORIGIN = 0xA0000000, LENGTH = 32M

}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)
SECTIONS
{
    .text : {
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
        _etext = ABSOLUTE(.);
    } > SDRAM

    .init_section : ALIGN(16) {
        _sinit = ABSOLUTE(.);
        KEEP(*(.init_array .init_array.*))
        _einit = ABSOLUTE(.);
    } > SDRAM

    .ARM.extab : ALIGN(16) {
        *(.ARM.extab*)
    } > SDRAM

    .ARM.exidx : ALIGN(16) {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
    } > SDRAM

    .data : ALIGN(16) {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > SRAM AT > SDRAM

    _eronly = LOADADDR(.data);

    .bss : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > SRAM

    /* Stabs debugging sections */

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
