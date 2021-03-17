/****************************************************************************
 * boards/arm/s32k1xx/s32k144evb/scripts/sram.ld
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

/* The S32K144 has 512Kb of FLASH beginning at address 0x0000:0000 and
 * 60Kb of SRAM beginning at address 0x1fff:8000 (plus 4Kb of FlexRAM)
 *
 * The on-chip RAM is split in two regions: SRAM_L and SRAM_U. The RAM is
 * implemented such that the SRAM_L and SRAM_U ranges form a contiguous
 * block in the memory map
 *
 *   SRAM_L 1fff8000 - 1fffffff 32Kb
 *   SRAM_U 20000000 - 20006fff 28Kb
 */

MEMORY
{
  flash (rx)  : ORIGIN = 0x00000000, LENGTH = 512K
  sram  (rwx) : ORIGIN = 0x1fff8000, LENGTH = 60K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

SECTIONS
{
  .text :
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
  } > sram

  .init_section :
  {
    _sinit = ABSOLUTE(.);
    KEEP(*(.init_array .init_array.*))
    _einit = ABSOLUTE(.);
  } > sram

  .ARM.extab :
  {
    *(.ARM.extab*)
  } >sram

  .ARM.exidx :
  {
    __exidx_start = ABSOLUTE(.);
    *(.ARM.exidx*)
    __exidx_end = ABSOLUTE(.);
  } >sram

  .data :
  {
    _sdata = ABSOLUTE(.);
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    CONSTRUCTORS
    . = ALIGN(4);
    _edata = ABSOLUTE(.);
  } > sram

  .bss :
  {
    _sbss = ABSOLUTE(.);
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = ABSOLUTE(.);
  } > sram

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
