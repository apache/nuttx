/****************************************************************************
 * boards/hc/m9s12/ne64badge/scripts/ne64badge-nonbanked.ld
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

/* The NE64 Badge has 64Kb of FLASH and 8Kb of SRAM that are assumed to be
 * paged and positioned as below:
 */

MEMORY
{
  /* The register space resides at address 0x0000-0x03ff.  The following
   * address, 0x0400-0x1fff are unused.
   *
   * The 8Kb SRAM is mapped to 0x2000-0x2fff.
   */

  sram    (rwx) : ORIGIN = 0x2000, LENGTH = 8K

  /* Three fixed text flash pages (corresponding to page 3e, 3d, and 3f) at
   * 16Kb each (minus 256 bytes at the end of page 3f that is reserved for
   * interrupt vectors).  Notice that this is linked as a single contiguous;
   * Post-processing is planned to make the binary to the appropriate flash
   * pages.
   */

  text    (rx)  : ORIGIN = 0x4000, LENGTH = 48K-256     /* Page 3e, 3d, and 3f */

  /* Vectors */

  vectors (rx)  : ORIGIN = 0xff80, LENGTH = 256
}

ENTRY(_stext)
SECTIONS
{
    .text : {
        _stext = ABSOLUTE(.);
        *(nonbanked)
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
    } > text

    _eronly = ABSOLUTE(.);

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        _edata = ABSOLUTE(.);
    } > sram AT > text

    .vectors : {
        *(vectors)
    } > vectors

    .bss : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        _ebss = ABSOLUTE(.);
    } > sram

    .vectors : {
        _svectors = ABSOLUTE(.);
        *(.vectors)
        _evectors = ABSOLUTE(.);
    } > vectors

    /* Stabs debugging sections.    */
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
