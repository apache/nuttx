/****************************************************************************
 * boards/arm/stm32/stm32f429i-disco/scripts/ofloader.ld
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

/* The STM32F429ZIT6 has 2048Kb of sram beginning at address 0x0800:0000 and
 * 256Kb of SRAM. SRAM is split up into four blocks:
 *
 * 1) 112Kb of SRAM beginning at address 0x2000:0000
 * 2)  16Kb of SRAM beginning at address 0x2001:c000
 * 3)  64Kb of SRAM beginning at address 0x2002:0000
 * 4)  64Kb of CCM SRAM beginning at address 0x1000:0000
 *
 * When booting from sram, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0800:0000 address
 * range.
 */

MEMORY
{
  sram (rwx) : ORIGIN = 0x20000000, LENGTH = 112K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)
SECTIONS
{
      PrgCode : ALIGN(4) {
        KEEP(*(PrgCode PrgCode.*));
        . = ALIGN(4);
      } > sram

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
    } > sram

    .init_section : ALIGN(4) {
        _sinit = ABSOLUTE(.);
        KEEP(*(.init_array .init_array.*))
        _einit = ABSOLUTE(.);
    } > sram

    .ARM.extab : ALIGN(4) {
        *(.ARM.extab*)
    } > sram

    .ARM.exidx : ALIGN(4) {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
    } > sram

    .tdata : {
        _stdata = ABSOLUTE(.);
        *(.tdata .tdata.* .gnu.linkonce.td.*);
        _etdata = ABSOLUTE(.);
    } > sram

    .tbss : {
        _stbss = ABSOLUTE(.);
        *(.tbss .tbss.* .gnu.linkonce.tb.* .tcommon);
        _etbss = ABSOLUTE(.);
    } > sram

    PrgData : ALIGN(4) {
        KEEP(*(PrgData PrgData.*))
        . = ALIGN(4);
    } > sram

    _eronly = ABSOLUTE(.);

    /* The RAM vector table (if present) should lie at the beginning of SRAM */

    .ram_vectors : {
        *(.ram_vectors)
    } > sram

    .data : ALIGN(4) {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram

    DevDscr : ALIGN(4) {
        KEEP(*(DevDscr DevDscr.*));
        KEEP(*(DevStack DevStack.*))
        . = ALIGN(4);
        /* StackBuff are the stack used to run opeflashloder,
         * and the location of the transmitted buffer.
         */
    } > sram

    .bss : ALIGN(4) {
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
}
