/****************************************************************************
 * boards/avr/avrdx/breadxavr/scripts/breadxavr.ld
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

/* Memory Regions ***********************************************************/
/* -------------- ------ ------+------- -- ------ -- ------ --- ------ ----+------- ---
 *                FLASH        | REGISTERS I/O       EXT I/O    ISRAM      | EEPROM
 *                             |           REGISTERS REGISTERS             |
 * -------------- ------ ------+------- -- ------ -- ------ --- ------ ----+------- ---
 * ATMEGA1284P    0x0000 128Kb | 0x0000 32 0x0020 64 0x0060 160 0x0100 16Kb| 0x0000 4Kb
 * -------------- ------ ------+------- -- ------ -- ------ --- ------ ----+------- ---
 * *Memory configuration A
 */

/* Use this instead of repeated magical number */
__CHIP_FLASH_SIZE__ = 128K;

/* VMA above 0x800000, must not collide with anything else,
 * arbitrary value otherwise.
 */
__RODATA_VMA__ = 0xa00000;
/* Application configuration is supposed to pass this value
 * to linker so the default should not be used.
 */
__RODATA_SIZE__ = DEFINED(__RODATA_SIZE__) ? __RODATA_SIZE__ : 4K;
/* Size increment matches chip's memory organization (boot, application
 * code and application data can be sized with 512B increments.)
 */
ASSERT (__RODATA_SIZE__ % 512 == 0,
        "__RODATA_SIZE__ must be a multiple of 512")
/* Only 32kB of flash is mapped to data memory */
ASSERT (__RODATA_SIZE__ <= 32K,
        "__RODATA_SIZE__ must not be larger than 32kB")

/* Same as for __RODATA_SIZE__ */
__RODATA_OFFSET__ = DEFINED(__RODATA_OFFSET__) ? __RODATA_OFFSET__ : 0;
/* Size increment matches chip's memory organization (boot, application
 * code and application data can be sized with 512B increments.)
 */
ASSERT (__RODATA_OFFSET__ % 512 == 0,
        "__RODATA_OFFSET__ must be a multiple of 512")
/* Only 32kB of flash is mapped to data memory */
ASSERT (__RODATA_OFFSET__ <= 31K,
        "__RODATA_OFFSET__ must not be larger than 31kB")

/* Also verify sum of size and offset */
ASSERT (__RODATA_SIZE__ + __RODATA_OFFSET__ <= 32K,
        "Sum of __RODATA_SIZE__ and __RODATA_OFFSET__ must not be larger than 32kB")

/* Set the origin to the very end of flash. 64K is a magical number
 * that constitutes of 32K (start of memory-mapped flash in data
 * memory address space) and 32K (end of the area, size of the rodata
 * section is subtracted from it.)
 */
__RODATA_ORIGIN__ = __RODATA_VMA__ + 64K - __RODATA_SIZE__ - __RODATA_OFFSET__;
__RODATA_FLASH_START = __CHIP_FLASH_SIZE__ - __RODATA_SIZE__ - __RODATA_OFFSET__;

MEMORY
{
  flash    (rx) : ORIGIN = 0, LENGTH = __CHIP_FLASH_SIZE__
  sram   (rw!x) : ORIGIN = 0x804000, LENGTH = 16K
  eeprom (rw!x) : ORIGIN = 0x801400, LENGTH = 512
  rodata  (r!x) : ORIGIN = __RODATA_ORIGIN__, LENGTH = __RODATA_SIZE__
}

ENTRY(__start)

SECTIONS
{
  /* Read-only sections, merged into text segment: */

  .hash     : { *(.hash) }
  .dynsym     : { *(.dynsym) }
  .dynstr     : { *(.dynstr) }
  .gnu.version  : { *(.gnu.version) }
  .gnu.version_d  : { *(.gnu.version_d) }
  .gnu.version_r  : { *(.gnu.version_r) }
  .rel.init   : { *(.rel.init) }
  .rela.init    : { *(.rela.init) }

  .rel.text   :
    {
      *(.rel.text)
      *(.rel.text.*)
      *(.rel.gnu.linkonce.t*)
    }

  .rela.text    :
    {
      *(.rela.text)
      *(.rela.text.*)
      *(.rela.gnu.linkonce.t*)
    }

  .rel.fini   : { *(.rel.fini) }
  .rela.fini    : { *(.rela.fini) }

  .rel.rodata   :
    {
      *(.rel.rodata)
      *(.rel.rodata.*)
      *(.rel.gnu.linkonce.r*)
    }

  .rela.rodata  :
    {
      *(.rela.rodata)
      *(.rela.rodata.*)
      *(.rela.gnu.linkonce.r*)
    }

  .rel.data   :
    {
      *(.rel.data)
      *(.rel.data.*)
      *(.rel.gnu.linkonce.d*)
    }

  .rela.data    :
    {
      *(.rela.data)
      *(.rela.data.*)
      *(.rela.gnu.linkonce.d*)
    }

  .rel.ctors    : { *(.rel.ctors) }
  .rela.ctors   : { *(.rela.ctors) }
  .rel.dtors    : { *(.rel.dtors) }
  .rela.dtors   : { *(.rela.dtors) }
  .rel.got    : { *(.rel.got) }
  .rela.got   : { *(.rela.got) }
  .rel.bss    : { *(.rel.bss) }
  .rela.bss   : { *(.rela.bss) }
  .rel.plt    : { *(.rel.plt) }
  .rela.plt   : { *(.rela.plt) }

  .text     :
  {
    _stext = . ;
    *(.vectors)
    KEEP(*(.vectors))
    *(.progmem.gcc*)
    *(.progmem*)
    . = ALIGN(2);
    *(.init)
    *(.handlers)
    *(.text)
    *(.text.*)
    _etext = . ;
  } > flash

  _eronly = ABSOLUTE(.);

  .data     :
  {
    _sdata = ABSOLUTE(.);
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    CONSTRUCTORS
    _edata = ABSOLUTE(.);
  } > sram AT > flash

  .bss      :
  {
    _sbss = ABSOLUTE(.);
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    _ebss = ABSOLUTE(.);
  } > sram

  /* Global data not cleared after reset. */

  .noinit     :
  {
    _snoinit = ABSOLUTE(.);
    *(.noinit*)
    _enoinit = ABSOLUTE(.);
  } > sram

  .rodata ABSOLUTE(__RODATA_ORIGIN__) : AT (ABSOLUTE(__RODATA_FLASH_START))
  {
    *(.rodata)
    *(.rodata*)
    *(.gnu.linkonce.r*)
    _rodata_end = ABSOLUTE(.);
  } > rodata

  .eeprom     :
  {
    _seeprom = ABSOLUTE(.);
    *(.eeprom*)
    _eeeprom = ABSOLUTE(.);
  } > eeprom

  /* Stabs debugging sections.  */

  .stab 0       : { *(.stab) }
  .stabstr 0      : { *(.stabstr) }
  .stab.excl 0    : { *(.stab.excl) }
  .stab.exclstr 0   : { *(.stab.exclstr) }
  .stab.index 0   : { *(.stab.index) }
  .stab.indexstr 0  : { *(.stab.indexstr) }
  .comment 0      : { *(.comment) }

  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */

  /* DWARF 1 */

  .debug 0      : { *(.debug) }
  .line 0       : { *(.line) }

  /* GNU DWARF 1 extensions */

  .debug_srcinfo 0  : { *(.debug_srcinfo) }
  .debug_sfnames 0  : { *(.debug_sfnames) }

  /* DWARF 1.1 and DWARF 2 */

  .debug_aranges 0  : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }

  /* DWARF 2 */

  .debug_info 0   : { *(.debug_info) *(.gnu.linkonce.wi.*) }
  .debug_abbrev 0   : { *(.debug_abbrev) }
  .debug_line 0   : { *(.debug_line) }
  .debug_frame 0    : { *(.debug_frame) }
  .debug_str 0    : { *(.debug_str) }
  .debug_loc 0    : { *(.debug_loc) }
  .debug_macinfo 0  : { *(.debug_macinfo) }
}
