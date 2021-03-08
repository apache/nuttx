/****************************************************************************
 * boards/x86_64/intel64/qemu-intel64/scripts/qemu.ld
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

OUTPUT_ARCH(i386:x86-64)
ENTRY(__pmode_entry)
SECTIONS
{

    . = 0x0;

    .realmode : {
        . = ALIGN(8);
        KEEP(*(.multiboot))
        *(.realmode)
    }

    . = 0x1M;
    _kernel_physical_start = .;

    .loader.text : {
        . = ALIGN(8);
        *(.loader.text)
    }

    .loader.rodata : {
        *(.loader.rodata)
    }

    .loader.data : {
        *(.loader.data)
    }

    .loader.bss : {
        *(.loader.bss)
    }

    . = ALIGN(0x1000);
      _boot_end = .;

    . += 0x100000000;
    _kernel_virtual_start = .;

    .text : AT(_boot_end)
    {
        _stext = ABSOLUTE(.);
        . = ALIGN(8);
        KEEP(*(.multiboot))
        *(.text .text.*)
        *(.gnu.linkonce.t.*)
        _etext = ABSOLUTE(.);
    }

    .rodata ALIGN(0x1000) : AT ( (LOADADDR (.text) + SIZEOF (.text) + 0xFFF) & 0xFFFFFFFFFFFFF000 )
    {
        _srodata = ABSOLUTE(.);
        *(.rodata .rodata.*)
        *(.fixup)
        *(.gnu.warning)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        *(.eh_frame)
        _erodata = ABSOLUTE(.);
    }

    .data ALIGN(0x1000) : AT ( (LOADADDR (.rodata) + SIZEOF (.rodata) + 0xFFF) & 0xFFFFFFFFFFFFF000 )
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    }

    .bss ALIGN(0x1000) : AT ( (LOADADDR (.data) + SIZEOF (.data) + 0xFFF) & 0xFFFFFFFFFFFFF000 )
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    }

    _kernel_virtual_end = .;

    _kernel_physical_end = (LOADADDR (.bss) + SIZEOF (.bss) + 0xFFF) & 0xFFFFFFFFFFFFF000;

    /* Stabs debugging sections */
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
