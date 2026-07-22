/****************************************************************************
 * boards/mips/jz4780/ci20/scripts/mips-debug.ld
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

MEMORY
{
    kseg0_progmem  (rx) : ORIGIN = 0x80020000, LENGTH = 0x0F7E0000

    kseg0_genexcpt (rx) : ORIGIN = 0x80000180, LENGTH = 128
    kseg0_ebexcpt0 (rx) : ORIGIN = 0x80000200, LENGTH = 32
    kseg0_ebexcpt1 (rx) : ORIGIN = 0x80000220, LENGTH = 32
    kseg0_ebexcpt2 (rx) : ORIGIN = 0x80000240, LENGTH = 32
    kseg0_ebexcpt3 (rx) : ORIGIN = 0x80000260, LENGTH = 32
    kseg0_ebexcpt4 (rx) : ORIGIN = 0x80000280, LENGTH = 32
    kseg0_ebexcpt5 (rx) : ORIGIN = 0x800002A0, LENGTH = 32
    kseg0_ebexcpt6 (rx) : ORIGIN = 0x800002C0, LENGTH = 32
    kseg0_ebexcpt7 (rx) : ORIGIN = 0x800002E0, LENGTH = 32
    kseg0_bevexcpt (rx) : ORIGIN = 0x80000380, LENGTH = 128
    kseg0_intexcpt (rx) : ORIGIN = 0x80000400, LENGTH = 128
    kseg0_dbgexcpt (rx) : ORIGIN = 0x80000480, LENGTH = 16
    kseg0_bootmem  (rx) : ORIGIN = 0x800004ac, LENGTH = 8192-1196
}

OUTPUT_FORMAT("elf32-tradlittlemips")
OUTPUT_ARCH(mips)
ENTRY(__start)

SECTIONS
{
    .gen_excpt :
    {
        KEEP (*(.gen_excpt))
    } > kseg0_genexcpt

    .ebase_excpt :
    {
        KEEP (*(.ebase_excpt))
    } > kseg0_ebexcpt0

    .ebase_excpt1 :
    {
        KEEP (*(.ebase_excpt1))
    } > kseg0_ebexcpt1

    .ebase_excpt2 :
    {
        KEEP (*(.ebase_excpt2))
    } > kseg0_ebexcpt2

    .bev_excpt :
    {
        KEEP (*(.bev_excpt))
    } > kseg0_bevexcpt

    .int_excpt :
    {
        KEEP (*(.int_excpt))
    } > kseg0_intexcpt

    .dbg_excpt = ORIGIN(kseg0_dbgexcpt);

    .start :
    {
        /* KSEG0 Reset startup logic */

        *(.start)

        /* KSEG0 exception handlers */

        *(.nmi_handler)
        *(.bev_handler)
        *(.int_handler)
        *(.swint_handler)
    } > kseg0_bootmem

    /* Program FLASH sections */

    .text :
    {
        _stext = ABSOLUTE(.);
        *(.text .text.*)
        *(.stub)
        KEEP (*(.text.*personality*))
        *(.gnu.linkonce.t.*)
        *(.gnu.warning)
        *(.mips16.fn.*)
        *(.mips16.call.*)

        /* Read-only data is included in the text section */

        *(.rodata .rodata.*)
        *(.rodata1)
        *(.gnu.linkonce.r.*)

        /* Small initialized constant global and static data */

        *(.sdata2 .sdata2.*)
        *(.gnu.linkonce.s2.*)

        /* Uninitialized constant global and static data */

        *(.sbss2 .sbss2.*)
        *(.gnu.linkonce.sb2.*)
        _etext = ABSOLUTE(.);
    } > kseg0_progmem

    /* Initialization data begins here in progmem */

    _data_loaddr = LOADADDR(.data);

    .eh_frame_hdr : { *(.eh_frame_hdr) *(.eh_frame_entry .eh_frame_entry.*) }
    .eh_frame : ONLY_IF_RO { KEEP (*(.eh_frame)) }

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        KEEP (*(.gnu.linkonce.d.*personality*))
        *(.data1)
    } > kseg0_progmem

    .eh_frame : ONLY_IF_RW { KEEP (*(.eh_frame)) }
    _gp = ALIGN(16) + 0x7FF0 ;

    .got :
    {
        *(.got.plt) *(.got)
    } > kseg0_progmem

    .sdata :
    {
        *(.sdata .sdata.* .gnu.linkonce.s.*)
        _edata = ABSOLUTE(.);
    } > kseg0_progmem

    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.dynsbss)
        *(.sbss .sbss.* .gnu.linkonce.sb.*)
        *(.scommon)
        *(.dynbss)
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(0x8);
        _ebss = ABSOLUTE(.);
    } > kseg0_progmem

    /* Stabs debugging sections */

    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }

    /* DWARF debug sections */
    /* DWARF 1 */

    .debug 0 : { *(.debug) }
    .line 0 : { *(.line) }

    /* GNU DWARF 1 extensions */

    .debug_srcinfo 0 : { *(.debug_srcinfo) }
    .debug_sfnames 0 : { *(.debug_sfnames) }

    /* DWARF 1.1 and DWARF 2 */

    .debug_aranges 0 : { *(.debug_aranges) }
    .debug_pubnames 0 : { *(.debug_pubnames) }

    /* DWARF 2 */

    .debug_info 0 : { *(.debug_info .gnu.linkonce.wi.*) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_line 0 : { *(.debug_line) }
    .debug_frame 0 : { *(.debug_frame) }
    .debug_str 0 : { *(.debug_str) }
    .debug_loc 0 : { *(.debug_loc) }
    .debug_macinfo 0 : { *(.debug_macinfo) }

    /* SGI/MIPS DWARF 2 extensions */

    .debug_weaknames 0 : { *(.debug_weaknames) }
    .debug_funcnames 0 : { *(.debug_funcnames) }
    .debug_typenames 0 : { *(.debug_typenames) }
    .debug_varnames 0 : { *(.debug_varnames) }

    /DISCARD/ : { *(.note.GNU-stack) }
}
