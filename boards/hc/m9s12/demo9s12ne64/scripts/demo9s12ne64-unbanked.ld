/****************************************************************************
 * boards/hc/m9s12/demo9s12ne64/scripts/demo9s12ne64-unbanked.ld
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

/* The DEMO9S12NE64 has 64Kb of FLASH and 8Kb of SRAM that are assumed to be
 * paged and positioned as below:
 */

MEMORY
{
  /* The 8Kb SRAM is mapped to 0x2000-0x2fff.  The top 256 bytes are reserved
   * for the serial monitor stack space.
   */

  sram    (rwx) : ORIGIN = 0x2000, LENGTH = 8K-256

  /* Three fixed text flash pages (corresponding to page 3e, 3d, and 3f) at
   * 16Kb each (minus 2Kb at the end of page 3f that is reserved at the top for
   * the serial boot loader
   */

  text    (rx)  : ORIGIN = 0x4000, LENGTH = 48K-2k      /* Page 3e, 3d, and 3f */

  /* Vectors.  These get relocated to 0xf780-f7ff by the serial loader */

  vectors (rx)  : ORIGIN = 0xff80, LENGTH = 256
}

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
    } > text

    _eronly = ABSOLUTE(.);      /* See below                    */

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        _edata = ABSOLUTE(.);
    } > sram AT > text

    .bss : {            /* BSS              */
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
