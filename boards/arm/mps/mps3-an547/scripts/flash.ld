/****************************************************************************
 * boards/arm/mps/mps3-an547/scripts/flash.ld
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

#include "nuttx/config.h"

MEMORY
{
  flash (rx)  : ORIGIN = 0x00000000, LENGTH = 512K
  sram1 (rwx) : ORIGIN = 0x01000000, LENGTH = 2M
  sram2 (rwx) : ORIGIN = 0x22000000, LENGTH = 2M
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(__start)

SECTIONS
{
    .text : {
        _stext = ABSOLUTE(.);
#ifndef CONFIG_BUILD_PIC /* We need change vectors use pic */
        *(.vectors)
#endif
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } > flash

    .init_section : ALIGN(4) {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > flash

    __llvm_prf_names : {
        __start__llvm_prf_names = .;
        KEEP (*(__llvm_prf_names))
        __end__llvm_prf_names = .;
    } > flash

    .ARM.extab : ALIGN(4) {
        *(.ARM.extab*)
    } > flash

    .ARM.exidx : ALIGN(4) {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
    } > flash

    .tdata : {
        _stdata = ABSOLUTE(.);
        *(.tdata .tdata.* .gnu.linkonce.td.*);
        _etdata = ABSOLUTE(.);
    } > flash

    .tbss : {
        _stbss = ABSOLUTE(.);
        *(.tbss .tbss.* .gnu.linkonce.tb.* .tcommon);
        _etbss = ABSOLUTE(.);
    } > flash

    _eronly = ABSOLUTE(.);

    /* The RAM vector table (if present) should lie at the beginning of SRAM */

    .ram_vectors : {
        *(.ram_vectors)
    } > sram1

    .data : ALIGN(4) {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
    } > sram1 AT > flash

    __llvm_prf_data : {
        __start__llvm_prf_data = .;
        KEEP (*(__llvm_prf_data))
        __end__llvm_prf_data = .;
    } > sram1 AT > flash

    __llvm_prf_vnds : {
        __start__llvm_prf_vnds = .;
        KEEP (*(__llvm_prf_vnds))
        __end__llvm_prf_vnds = .;
    } > sram1 AT > flash

    __llvm_prf_cnts : {
        __start__llvm_prf_cnts = .;
        KEEP (*(__llvm_prf_cnts))
        __end__llvm_prf_cnts = .;
        _edata = ABSOLUTE(.);
    } > sram1 AT > flash

    .got :
      {
        *(.got*)
      } > sram1

    .bss : ALIGN(4) {
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
