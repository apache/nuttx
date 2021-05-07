/****************************************************************************
 * boards/arm/kinetis/teensy-3.x/scripts/mk20dx256vlh7.ld
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

/* The MK20DX256VLH7 has 256Kb of FLASH beginning at address 0x0000:0000 and
 * 64Kb of SRAM beginning at address 0x1c00:0000 (SRAM_L) and 0x2000:000
 * bitband region (SRAM_U).
 *
 * NOTE: that the first part of the K20 FLASH region is reserved for
 *       interrupt vectflash and, following that, is a region from 0x0000:0400
 *       to 0x0000:040f that is reserved for the FLASH control fields (FCF).
 *
 * NOTE: The on-chip RAM is split evenly among SRAM_L and SRAM_U. The RAM is
 *       also implemented such that the SRAM_L and SRAM_U ranges form a
 *       contiguous block in the memory map.
 */

MEMORY
{
  vectflash (rx)  : ORIGIN = 0x00000000, LENGTH = 1K
  cfmprotect (rx) : ORIGIN = 0x00000400, LENGTH = 16
  progflash (rx)  : ORIGIN = 0x00000800, LENGTH = 256K - 2K
  datasram (rwx)  : ORIGIN = 0x1fff8000, LENGTH = 64K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)
EXTERN(__flashconfigbytes)
SECTIONS
{
    .vectors : {
        _svectors = ABSOLUTE(.);
        *(.vectors)
        _evectors = ABSOLUTE(.);
    } > vectflash

    .cfmprotect : {
        KEEP(*(.cfmconfig))
    } > cfmprotect

    .text : {
        _stext = ABSOLUTE(.);
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
    } > progflash

    .init_section : {
        _sinit = ABSOLUTE(.);
        *(.init_array .init_array.*)
        _einit = ABSOLUTE(.);
    } > progflash

    .ARM.extab : {
        *(.ARM.extab*)
    } > progflash

    .ARM.exidx : {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
    } > progflash

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > datasram AT > progflash

    _eronly = LOADADDR(.data);

    .ramfunc ALIGN(4): {
        _sramfuncs = ABSOLUTE(.);
        *(.ramfunc  .ramfunc.*)
        _eramfuncs = ABSOLUTE(.);
    } > datasram AT > progflash

    _framfuncs = LOADADDR(.ramfunc);

    .bss : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > datasram

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
