/****************************************************************************
 * boards/avr/atmega/amber/scripts/amber.ld
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

/* Memory Regions ***********************************************************/

/* -------------- ------ ------+------- -- ------ -- ------ --- ------ ----+------- ---
 *                FLASH        | REGISTERS I/O       EXT I/O    ISRAM      | EEPROM
 *                             |           REGISTERS REGISTERS             |
 * -------------- ------ ------+------- -- ------ -- ------ --- ------ ----+------- ---
 * ATMEGA128*     0x0000 128Kb | 0x0000 32 0x0020 64 0x0060 160 0x0100 4Kb | 0x0000 4Kb
 * -------------- ------ ------+------- -- ------ -- ------ --- ------ ----+------- ---
 * *Memory configuration A
 */

MEMORY
{
  flash    (rx) : ORIGIN = 0, LENGTH = 128K
  sram   (rw!x) : ORIGIN = 0x800100, LENGTH = 4K
  eeprom (rw!x) : ORIGIN = 0x810000, LENGTH = 4K
}

ENTRY(__start)

SECTIONS
{
    /* Read-only sections, merged into text segment: */

    .hash           : { *(.hash) }
    .dynsym         : { *(.dynsym) }
    .dynstr         : { *(.dynstr) }
    .gnu.version    : { *(.gnu.version) }
    .gnu.version_d  : { *(.gnu.version_d) }
    .gnu.version_r  : { *(.gnu.version_r) }
    .rel.init       : { *(.rel.init) }
    .rela.init      : { *(.rela.init) }

    .rel.text       :
        {
            *(.rel.text)
            *(.rel.text.*)
            *(.rel.gnu.linkonce.t*)
        }

    .rela.text      :
        {
            *(.rela.text)
            *(.rela.text.*)
            *(.rela.gnu.linkonce.t*)
        }

    .rel.fini       : { *(.rel.fini) }
    .rela.fini      : { *(.rela.fini) }

    .rel.rodata     :
        {
            *(.rel.rodata)
            *(.rel.rodata.*)
            *(.rel.gnu.linkonce.r*)
        }

    .rela.rodata    :
        {
            *(.rela.rodata)
            *(.rela.rodata.*)
            *(.rela.gnu.linkonce.r*)
        }

    .rel.data       :
        {
            *(.rel.data)
            *(.rel.data.*)
            *(.rel.gnu.linkonce.d*)
        }

    .rela.data      :
        {
            *(.rela.data)
            *(.rela.data.*)
            *(.rela.gnu.linkonce.d*)
        }

    .rel.ctors      : { *(.rel.ctors) }
    .rela.ctors     : { *(.rela.ctors) }
    .rel.dtors      : { *(.rel.dtors) }
    .rela.dtors     : { *(.rela.dtors) }
    .rel.got        : { *(.rel.got) }
    .rela.got       : { *(.rela.got) }
    .rel.bss        : { *(.rel.bss) }
    .rela.bss       : { *(.rela.bss) }
    .rel.plt        : { *(.rel.plt) }
    .rela.plt       : { *(.rela.plt) }

    .text           :
    {
        _stext = . ;
        *(.vectors)
        *(.init)
        *(.handlers)
        *(.text)
        *(.text.*)
        _etext = . ;
    } > flash

    _eronly = ABSOLUTE(.);

    .data           :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        _edata = ABSOLUTE(.);
    } > sram AT > flash

    .bss            :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        _ebss = ABSOLUTE(.);
    } > sram

    /* Global data not cleared after reset. */

    .noinit         :
    {
        _snoinit = ABSOLUTE(.);
        *(.noinit*)
        _enoinit = ABSOLUTE(.);
    }   > sram

    .eeprom         :
    {
        _seeprom = ABSOLUTE(.);
        *(.eeprom*)
        _eeeprom = ABSOLUTE(.);
    }   > eeprom

    /* Stabs debugging sections.    */

    .stab 0             : { *(.stab) }
    .stabstr 0          : { *(.stabstr) }
    .stab.excl 0        : { *(.stab.excl) }
    .stab.exclstr 0     : { *(.stab.exclstr) }
    .stab.index 0       : { *(.stab.index) }
    .stab.indexstr 0    : { *(.stab.indexstr) }
    .comment 0          : { *(.comment) }

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
