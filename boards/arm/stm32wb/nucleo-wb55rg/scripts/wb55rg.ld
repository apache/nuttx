/****************************************************************************
 * boards/arm/stm32wb/nucleo-wb55rg/scripts/wb55rg.ld
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

/* The STM32WB55RG has 1024Kb of FLASH beginning at address 0x0800:0000,
 * and 256Kb SRAM, split in three blocks:
 *   SRAM1: 192Kb mapped at address 0x2000:0000
 *   SRAM2a: 32Kb located at address 0x2003:0000 (contiguous to SRAM1)
 * also mirrored at 0x1000:0000 with hardware parity check and can be
 * retained in Standby mode.
 *   SRAM2b: 32Kb located at address 0x2003:8000 (contiguous with SRAM2a)
 * also mirrored at 0x1000:8000 with hardware parity check.
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point
 * in the 0x0800:0000 address range.
 */

MEMORY
{
  flash (rx) : ORIGIN = 0x08000000, LENGTH = 512K
  sram1 (rwx) : ORIGIN = 0x20000000, LENGTH = 192K
  sram2a (rwx) : ORIGIN = 0x20030000, LENGTH = 32K
  sram2b (rwx) : ORIGIN = 0x20038000, LENGTH = 32K
}

OUTPUT_ARCH(arm)
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
    } > flash

    .init_section : {
        _sinit = ABSOLUTE(.);
        KEEP(*(.init_array .init_array.*))
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

    /* The STM32WB55RG has 192Kb of SRAM1 beginning at the following address */

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram1 AT > flash

    .bss : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > sram1

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
