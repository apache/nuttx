/****************************************************************************
 * boards/arm/stm32h7/nucleo-h745zi/scripts/flash.ld
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

#ifndef CONFIG_STM32H7_CORTEXM4_ENABLED
MEMORY
{
  itcm  (rwx) : ORIGIN = 0x00000000, LENGTH = 64K
  flash (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  dtcm1 (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
  dtcm2 (rwx) : ORIGIN = 0x20010000, LENGTH = 64K
  sram  (rwx) : ORIGIN = 0x24000000, LENGTH = 512K
  sram1 (rwx) : ORIGIN = 0x30000000, LENGTH = 128K
  sram2 (rwx) : ORIGIN = 0x30020000, LENGTH = 128K
  sram3 (rwx) : ORIGIN = 0x30040000, LENGTH = 32K
  sram4 (rwx) : ORIGIN = 0x38000000, LENGTH = 64K
  bbram (rwx) : ORIGIN = 0x38800000, LENGTH = 4K
}
#else
MEMORY
{
  itcm  (rwx) : ORIGIN = 0x00000000, LENGTH = 64K
  flash (rx)  : ORIGIN = 0x08000000, LENGTH = CONFIG_STM32H7_CORTEXM7_FLASH_SIZE
  dtcm1 (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
  dtcm2 (rwx) : ORIGIN = 0x20010000, LENGTH = 64K
  sram  (rwx) : ORIGIN = 0x24000000, LENGTH = 512K

  /* shared memory on SRAM3 */

  shmem (rwx) : ORIGIN = 0x30040000, LENGTH = 32K
  sram4 (rwx) : ORIGIN = 0x38000000, LENGTH = 64K
  bbram (rwx) : ORIGIN = 0x38800000, LENGTH = 4K
}
#endif

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
    } > flash

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab*)
    } > flash

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx :
    {
        *(.ARM.exidx*)
    } > flash
    __exidx_end = ABSOLUTE(.);

    _eronly = ABSOLUTE(.);

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram AT > flash

    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > sram

    .shmem :
    {
        . = ALIGN(4);
        *(.shmem);
        KEEP(*(.shmem))
    } > shmem

    /* Emit the the D3 power domain section for locating BDMA data
     *
     * Static data with locate_data(".sram4") will be located
     * at start of SRAM4; the rest of SRAM4 will be added to the heap.
     */

    .sram4_reserve (NOLOAD) :
    {
        *(.sram4)
        . = ALIGN(4);
        _sram4_heap_start = ABSOLUTE(.);
    } > sram4

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
