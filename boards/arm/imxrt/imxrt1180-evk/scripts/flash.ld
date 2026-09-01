/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/scripts/flash.ld
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

/* The Cortex-M7 image is a raw XIP payload; it carries no FCB and no AHAB
 * container of its own.  The M33 image (the "bl" bootloader config, or any
 * other M33 build) owns the first 512 KB of FlexSPI1 NOR and releases the
 * M7 with INITVTOR pointing at this payload.
 *
 * FlexSPI1 NOR layout:
 *
 *   0x2800_0000 .. 0x2807_FFFF   M33 image (512 KB reserved)
 *   0x2808_0000 .. 0x287F_FFFF   M7 image  (this script, 7.5 MB window)
 *
 * NuttX's stack, .data, .ramfunc and .bss live in OCRAM1, which the M33
 * grants the M7 R/W/X through TRDC2 MRC_W3 before releasing it.  DTCM and
 * ITCM are declared above but are not used by any section; they are
 * tightly-coupled to the M7 core and have no TRDC in the access path, so
 * they remain available for opt-in placement.
 */

MEMORY
{
  flash (rx)  : ORIGIN = 0x28080000, LENGTH = 4M      /* Cortex-M7 XIP window (after the 512K M33/bl reserve) */
  ocram (rwx) : ORIGIN = 0x20484000, LENGTH = 480K    /* OCRAM1 minus ROM patch (0x20480000..0x20483fff) and M33 reserve (0x204fc000..0x204fffff) */
  dtcm  (rwx) : ORIGIN = 0x20000000, LENGTH = 256K    /* M7 DTCM (opt-in) */
  itcm  (rwx) : ORIGIN = 0x00000000, LENGTH = 256K    /* M7 ITCM (opt-in) */
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

SECTIONS
{
    .text : ALIGN(4)
    {
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

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        *(.init_array .init_array.*)
        _einit = ABSOLUTE(.);
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab*)
    } > flash

    .ARM.exidx :
    {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
    } > flash

    _eronly = ABSOLUTE(.);

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > ocram AT > flash

    .ramfunc ALIGN(4):
    {
        _sramfuncs = ABSOLUTE(.);
        *(.ramfunc  .ramfunc.*)
        _eramfuncs = ABSOLUTE(.);
    } > ocram AT > flash

    _framfuncs = LOADADDR(.ramfunc);

    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > ocram

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

    _boot_loadaddr = ORIGIN(flash);
    _boot_size = LENGTH(flash);
    _ram_size = LENGTH(ocram);
}
