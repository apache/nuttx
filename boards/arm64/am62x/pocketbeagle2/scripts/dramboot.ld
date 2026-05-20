/****************************************************************************
 * boards/arm64/am62x/pocketbeagle2/scripts/dramboot.ld
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

/* PocketBeagle 2 — NuttX loaded into DDR by U-Boot.
 *
 * Memory layout:
 *   0x8000_0000  DDR start (512 MB physically installed)
 *   0x8200_0000  NuttX load address (avoids U-Boot reserved region)
 *   0x9FFF_FFFF  DDR end
 *
 * U-Boot loads the raw binary and jumps to the first instruction
 * (_start in arm64_head.S which is placed in the .start section).
 *
 * The heap begins at _e_initstack and extends upward; its size is
 * determined by CONFIG_RAM_SIZE minus the image footprint.
 */

OUTPUT_ARCH(aarch64)

ENTRY(__start)

PHDRS
{
  text PT_LOAD ;
}

SECTIONS
{
  /* NuttX is loaded at the kernel_addr_r address */

  . = 0x82000000;

  _start = .;

  /* -----------------------------------------------------------------------
   * Read-only / executable sections
   * --------------------------------------------------------------------- */

  .text :
  {
    _stext = .;
    *(.start .start.*)          /* arm64_head.S __start must be first      */
    *(.text)
    *(.text.*)
    *(.text.cold)
    *(.text.unlikely)
    *(.fixup)
    *(.gnu.warning)
  } :text = 0x9090

  . = ALIGN(4096);

  /* C++ constructor / destructor init arrays */

  .init_section :
  {
    _sinit = ABSOLUTE(.);
    KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*)
           SORT_BY_INIT_PRIORITY(.ctors.*)))
    KEEP(*(.init_array
           EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o)
           .ctors))
    _einit = ABSOLUTE(.);
  } :text

  . = ALIGN(4096);

  /* Exception vector table — must be 4 KB aligned per AArch64 spec */

  .vector :
  {
    _vector_start = .;
    KEEP(*(.exc_vector_table))
    KEEP(*(.exc_vector_table.*))
    KEEP(*(.vectors))
    _vector_end = .;
  } :text

  . = ALIGN(4096);

  _etext    = .;
  _sztext   = _etext - _stext;

  /* -----------------------------------------------------------------------
   * Read-only data
   * --------------------------------------------------------------------- */

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

  _erodata  = .;
  _szrodata = _erodata - _srodata;
  _eronly   = .;

  /* -----------------------------------------------------------------------
   * Initialised data  (copied from flash / loaded image at boot)
   * --------------------------------------------------------------------- */

  . = ALIGN(4096);

  .data :
  {
    _sdata = .;
    *(.data.page_aligned)
    *(.data)
    *(.data.*)
    . = ALIGN(8);
    *(.data.rel)
    *(.data.rel.*)
    CONSTRUCTORS
  } :text

  _edata    = .;

  /* -----------------------------------------------------------------------
   * Zero-initialised data (BSS)
   * --------------------------------------------------------------------- */

  .bss :
  {
    . = ALIGN(8);
    _sbss = .;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(8);
  } :text

  . = ALIGN(4096);

  _ebss   = .;
  _szbss  = _ebss - _sbss;

  /* -----------------------------------------------------------------------
   * Boot/idle stack  (placed immediately after BSS)
   * --------------------------------------------------------------------- */

  .initstack :
  {
    _s_initstack = .;
    *(.initstack)
    . = ALIGN(16);
  } :text

  . = ALIGN(4096);

  _e_initstack  = .;
  g_idle_topstack = .;          /* arm64 common layer uses this symbol     */

  _szdata       = _e_initstack - _sdata;

  /* -----------------------------------------------------------------------
   * Discarded sections
   * --------------------------------------------------------------------- */

  /DISCARD/ :
  {
    *(.exit.text)
    *(.exit.data)
    *(.exitcall.exit)
    *(.eh_frame)
  }

  /* Debugging sections */

  .stab        0 : { *(.stab) }
  .stabstr     0 : { *(.stabstr) }
  .stab.excl   0 : { *(.stab.excl) }
  .stab.exclstr 0 : { *(.stab.exclstr) }
  .stab.index  0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment     0 : { *(.comment) }
}
