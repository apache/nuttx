/****************************************************************************
 * boards/arm64/rk3588/orangepi-5-pro/scripts/dramboot.ld
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
 * Orange Pi 5 Pro linker script for the initial boot image.  The kernel is
 * linked at the address selected by U-Boot after ARM64 Image relocation.
 *
 ****************************************************************************/

#include <nuttx/config.h>

OUTPUT_ARCH(aarch64)
ENTRY(__start)

PHDRS
{
  text PT_LOAD ;
}

SECTIONS
{
  . = CONFIG_RAM_START;
  _start = .;

  .text :
  {
    _stext = .;
    *(.start .start.*)
    *(.text) *(.text.*) *(.text.cold) *(.text.unlikely)
    *(.fixup) *(.gnu.warning)
  } :text = 0x9090

  . = ALIGN(4096);

  .init_section :
  {
    _sinit = ABSOLUTE(.);
    KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
    KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
    _einit = ABSOLUTE(.);
  } :text

  . = ALIGN(4096);

  .vector :
  {
    _vector_start = .;
    KEEP(*(.exc_vector_table))
    KEEP(*(".exc_vector_table.*"))
    KEEP(*(.vectors))
    _vector_end = .;
  } :text

  . = ALIGN(4096);
  _etext = .;
  _sztext = _etext - _stext;

  . = ALIGN(4096);
  .rodata :
  {
    _srodata = .;
    *(.rodata)
    *(.rodata.*)
    *(.data.rel.ro)
    *(.data.rel.ro.*)
  } :text
  . = ALIGN(4096);
  _erodata = .;
  _szrodata = _erodata - _srodata;
  _eronly = .;

  . = ALIGN(4096);
  .data :
  {
    _sdata = .;
    *(.data.page_aligned)
    *(.data) *(.data.*)
    . = ALIGN(8);
    *(.data.rel) *(.data.rel.*)
    CONSTRUCTORS
  } :text
  _edata = .;

  .bss :
  {
    . = ALIGN(8);
    _sbss = .;
    *(.bss) *(.bss.*) *(COMMON)
    . = ALIGN(8);
  } :text
  . = ALIGN(4096);
  _ebss = .;
  _szbss = _ebss - _sbss;

  .initstack :
  {
    _s_initstack = .;
    *(.initstack)
    . = ALIGN(16);
  } :text
  . = ALIGN(4096);
  _e_initstack = .;
  g_idle_topstack = .;
  _szdata = _e_initstack - _sdata;

  /DISCARD/ :
  {
    *(.exit.text) *(.exit.data) *(.exitcall.exit) *(.eh_frame)
  }

  .stab 0 : { *(.stab) }
  .stabstr 0 : { *(.stabstr) }
  .comment 0 : { *(.comment) }
}
