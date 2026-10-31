/****************************************************************************
 * boards/rh850/u2c/qemu-rh850-u2c/scripts/qemu-rh850-u2c.ld
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

#include <nuttx/config.h>

/* GCC prefixes C symbols with "_": a symbol used by C is defined both as
 * _name (C name) and as __name (C _name).
 */

MEMORY
{
  flash (rx)  : ORIGIN = 0x00000000, LENGTH = 8M
  lram  (rwx) : ORIGIN = 0xfdc00000, LENGTH = 128K
}

OUTPUT_ARCH(v850:rh850)
ENTRY(__start)

SECTIONS
{
  .text :
  {
    _stext = ABSOLUTE(.);
    __stext = ABSOLUTE(_stext);
    KEEP(*(.vectors))
    *(.text .text.*)
    *(.gnu.linkonce.t.*)
    _etext = ABSOLUTE(.);
    __etext = ABSOLUTE(_etext);
  } > flash

  .rodata :
  {
    *(.rodata .rodata.*)
    *(.rosdata .rosdata.*)
    *(.rozdata .rozdata.*)
    *(.gnu.linkonce.r.*)
  } > flash

  .init_section :
  {
    _sinit = ABSOLUTE(.);
    __sinit = ABSOLUTE(_sinit);
    KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
    KEEP(*(.init_array .ctors))
    _einit = ABSOLUTE(.);
    __einit = ABSOLUTE(_einit);
  } > flash

  _eronly = LOADADDR(.data);
  __eronly = ABSOLUTE(_eronly);

  .data : ALIGN(4)
  {
    _sdata = ABSOLUTE(.);
    __sdata = ABSOLUTE(_sdata);
    *(.data .data.*)
    *(.sdata .sdata.*)
    *(.tdata .tdata.*)
    *(.zdata .zdata.*)
    *(.gnu.linkonce.d.*)
    . = ALIGN(4);
    _edata = ABSOLUTE(.);
    __edata = ABSOLUTE(_edata);
  } > lram AT > flash

  .bss (NOLOAD) : ALIGN(4)
  {
    _sbss = ABSOLUTE(.);
    __sbss = ABSOLUTE(_sbss);
    *(.bss .bss.*)
    *(.sbss .sbss.*)
    *(.tbss .tbss.*)
    *(.zbss .zbss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = ABSOLUTE(.);
    __ebss = ABSOLUTE(_ebss);
  } > lram AT > lram

  /* Idle thread stack above .bss, the heap above it.  NOLOAD sections
   * keep LMA = VMA so that loaders do not place them in flash.
   */

  .idle_stack (NOLOAD) : ALIGN(4)
  {
    . += CONFIG_IDLETHREAD_STACKSIZE;
    _g_idle_topstack = ABSOLUTE(.);
  } > lram AT > lram

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
