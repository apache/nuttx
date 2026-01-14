/****************************************************************************
 * boards/arm/mps/mps2-an500/scripts/ghs-flash.ld
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

MEMORY
{
  flash (rwx) : ORIGIN = 0x00000000, LENGTH = 4M
}

SECTIONS
{
    .text ALIGN(4) : {
        _stext = ABSOLUTE(.);
        *(.vectors)
        *(.text)
        "*(.text.*)"
        *(.fixup)
        *(.gnu.warning)
        *(.rodata)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        _etext = ABSOLUTE(.);
    } > flash

    .syscall : > .

    .init_section ALIGN(4) : {
        _sinit = ABSOLUTE(.);
        *(.init_array)
        *(.ctors)
        _einit = ABSOLUTE(.);
    } > flash

    .ARM.extab ALIGN(4) : {
        "*(.ARM.extab*)"
    } > flash

    .ARM.exidx ALIGN(4) : {
        __exidx_start = ABSOLUTE(.);
        "*(.ARM.exidx*)"
        __exidx_end = ABSOLUTE(.);
    } > flash

    .tdata ALIGN(4) : {
        _stdata = ABSOLUTE(.);
        *(.tdata)
        "*(.tdata.*)"
        _etdata = ABSOLUTE(.);
    } > flash

    .tbss : {
        _stbss = ABSOLUTE(.);
        *(.tbss)
        *(.tcommon)
        _etbss = ABSOLUTE(.);
    } > flash

    _eronly = ENDADDR(.tbss);

    .ram_vectors : {
        *(.ram_vectors)
    } > flash

    .data ALIGN(4) : {
        _sdata = ABSOLUTE(.);
        *(.data)
        "*(.data.*)"
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > flash

    .bss ALIGN(4) : {
        _sbss = ABSOLUTE(.);
        *(.bss)
        "*(.bss.*)"
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > flash
}
