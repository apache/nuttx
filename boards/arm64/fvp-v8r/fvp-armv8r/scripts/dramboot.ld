/****************************************************************************
 * boards/arm64/fvp-v8r/fvp-armv8r/scripts/dramboot.ld
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

#include "memory.ld"

OUTPUT_ARCH(aarch64)

ENTRY(__start)

SECTIONS
{
  /* where the global variable out-of-bounds detection information located */
  .text : {
      . = ALIGN(64);
      _stext = .;            /* Text section */
      *(.start .start.*)
      *(.vectors*)
      *(.exc_vector_table*)
      *(.text*)
      *(.fixup)
      *(.gnu.warning)
      . = ALIGN(64);
      _etext = .; /* End_1 of .text */
      _sztext = _etext - _stext;
  } > ksram

  .ARM.extab :
  {
      *(.ARM.extab*)
  } > ksram

  /* .ARM.exidx is sorted, so has to go in its own output section.  */

  PROVIDE_HIDDEN (__exidx_start = .);
  .ARM.exidx :
  {
      *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } > ksram
  PROVIDE_HIDDEN (__exidx_end = .);

  .bss : {                     /* BSS */
       . = ALIGN(64);
       _sbss = . ;
       *(.bss*)
       . = ALIGN(4096);
       _ebss = .;
  } > ksram

  _szbss = _ebss - _sbss;

  .init_section : {
      _sinit = ABSOLUTE(.);
      KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
      KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
      _einit = ABSOLUTE(.);
      . = ALIGN(4096);
  } > ksram

  .rodata : {
      _srodata = .;          /* Read-only data */
      *(.rodata*)
      *(.data.rel.ro*)
      KEEP(*(SORT(.scattered_array*)));
      . = ALIGN(4096);
      _erodata = .;
  } > ksram

  .data : {                    /* Data */
      _sdata = .;
      *(.data*)
      . = ALIGN(8);
      __start_impls = .;
      *(.impls)
      KEEP(*(.impls))
      . = ALIGN(4);
      __stop_impls = .;
      _edata = .;
  } > ksram

  .noinit : {
      _snoinit = ABSOLUTE(.);
      *(.noinit*)
      _enoinit = ABSOLUTE(.);
      . =  + 60;
  } > ksram

 . = ALIGN(4096);
  .initstack : {             /* INIT STACK */
       _s_initstack = .;
       *(.initstack)
       . = ALIGN(16);
  } > ksram
  . = ALIGN(4096);
  _e_initstack = . ;
  g_idle_topstack = . ;

  /* Sections to be discarded */
  /DISCARD/ : {
       *(.exit.text)
       *(.exit.data)
       *(.exitcall.exit)
  }

  /* Stabs debugging sections.  */
  .stab 0 : { *(.stab) }
  .stabstr 0 : { *(.stabstr) }
  .stab.excl 0 : { *(.stab.excl) }
  .stab.exclstr 0 : { *(.stab.exclstr) }
  .stab.index 0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment 0 : { *(.comment) }

    /* DWARF debug sections.
         Symbols in the DWARF debugging sections are relative to the beginning
         of the section so we begin them at 0.  */

    /* DWARF 1 */

    .debug 0            : { *(.debug) }
    .line 0             : { *(.line) }

    /* GNU DWARF 1 extensions */

    .debug_srcinfo 0    : { *(.debug_srcinfo) }
    .debug_sfnames 0    : { *(.debug_sfnames) }

    /* DWARF 1.1 and DWARF 2 */

    .debug_aranges 0    : { *(.debug_aranges) }
    .debug_pubnames 0   : { *(.debug_pubnames) }

    /* DWARF 2 */

    .debug_info 0       : { *(.debug_info) *(.gnu.linkonce.wi.*) }
    .debug_abbrev 0     : { *(.debug_abbrev) }
    .debug_line 0       : { *(.debug_line) }
    .debug_frame 0      : { *(.debug_frame) }
    .debug_str 0        : { *(.debug_str) }
    .debug_loc 0        : { *(.debug_loc) }
    .debug_macinfo 0    : { *(.debug_macinfo) }
}
