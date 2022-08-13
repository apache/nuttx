/****************************************************************************
 * boards/arm/kl/teensy-lc/scripts/teensy-lc.ld
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

/* The Teensy LC has 64kB of flash and 8 kB of RAM.  2 kB of flash is */
/* reserved for the bootloader.
 */

MEMORY
{
  vectflash (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00c0
  cfmprotect (rx) : ORIGIN = 0x00000400, LENGTH = 0x10
  progflash (rx)  : ORIGIN = 0x00000410, LENGTH = 64K - 2K - 0x410
  datasram (rwx)  : ORIGIN = 0x1ffff800, LENGTH = 8K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
EXTERN(_cfmconfig)
ENTRY(_stext)
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
