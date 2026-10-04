/****************************************************************************
 * boards/mips/pic32mz/ev49n51a/scripts/mips-debug.ld
 *
 * Adapted from boards/mips/pic32mz/flipnclick-pic32mz/scripts/mips-debug.ld
 * for the WFI32E01 (PIC32MZ-W1).  Memory region addresses/sizes were
 * cross-checked against Microchip's PIC32MZ-W_DFP (WFI32E01.atdf) and its
 * reference linker script (xc32/WFI32E01/pWFI32E01.ld) - WFI32E01's
 * program flash lives at a different physical base (0x10000000) than
 * EC/EF (0x1d000000); do not assume EC/EF addresses apply.
 *
 * The MIPS32 microAptiv exception vector layout (reset/TLB-refill/cache
 * error/general/interrupt/EJTAG offsets from EBase) is core-architecture
 * defined, not PIC32-family specific, so that part is unchanged from
 * EC/EF.
 *
 * The boot-flash configuration words (.devcfg at 0xbfc55f88, plus FCPN0
 * and FSIGN0) are emitted by pic32mz_head.S; see
 * hardware/pic32mzw1_features.h.
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
    /* WFI32E01 has 1024Kb (1MB) of program FLASH at physical address
     * 0x10000000, always accessed at KSEG0 address 0x9000:0000.
     */

    kseg0_progmem  (rx) : ORIGIN = 0x90000000, LENGTH = 1024K

    /* Exception vectors and startup trampoline, in boot flash
     * (0xbfc00000, KSEG1).  Offsets below the vectors are defined by the
     * MIPS32 microAptiv core architecture (EBase-relative), same as on
     * PIC32MZ-EC/EF:
     *
     *   REGION        PHYSICAL   KSEG   SIZE
     *   DESCRIPTION   START ADDR        (BYTES)
     *   ------------- ---------- ------ ----------------------
     *   Reset         0x1fc00000 KSEG1  512
     *   TLB Refill    0x1fc00200 KSEG1  256
     *   Cache Error   0x1fc00300 KSEG1  128
     *   Others        0x1fc00380 KSEG1  128
     *   Interrupt     0x1fc00400 KSEG1  128
     *   JTAG          0x1fc00480 KSEG1  16
     *
     * Exceptions assume STATUS:BEV=0/1,EXL=0; CAUSE:IV=1; multi-vector
     * support disabled - same configuration pic32mz_head.S sets up for
     * EC/EF.
     */

    kseg1_reset    (rx) : ORIGIN = 0xbfc00000, LENGTH = 384
    kseg1_genexcpt (rx) : ORIGIN = 0xbfc00180, LENGTH = 128
    kseg1_ebexcpt  (rx) : ORIGIN = 0xbfc00200, LENGTH = 128
    kseg1_bevexcpt (rx) : ORIGIN = 0xbfc00380, LENGTH = 128
    kseg1_intexcpt (rx) : ORIGIN = 0xbfc00400, LENGTH = 128
    kseg1_dbgexcpt (rx) : ORIGIN = 0xbfc00480, LENGTH = 16
    kseg0_bootmem  (rx) : ORIGIN = 0x9fc004ac, LENGTH = 8192-1196
    kseg1_dbgcode  (rx) : ORIGIN = 0xbfc02000, LENGTH = 4096-16

    /* Boot-flash configuration words (primary copy) */

    kseg1_devcfg   (r)  : ORIGIN = 0xbfc55f88, LENGTH = 24
    kseg1_devcp    (r)  : ORIGIN = 0xbfc55fbc, LENGTH = 4
    kseg1_devsign  (r)  : ORIGIN = 0xbfc55fdc, LENGTH = 4

    /* WFI32E01 has 256Kb of data memory at physical address 0x00000000,
     * accessed through KSEG0 so that it goes through the D-Cache when the
     * cache is enabled (KSEG0 is uncached otherwise).  640 bytes are set
     * aside at the start, matching the margin EC/EF boards reserve.
     */

    kseg0_datamem (rw!x) : ORIGIN = 0x80000200, LENGTH = 256K - 640
}

/* No OUTPUT_FORMAT: XC32 names its little-endian ELF target
 * elf32-tradlittlemips while Pinguino's binutils only know
 * elf32-littlemips.  Make.defs passes -EL instead, so each linker
 * picks its own little-endian format.
 */

OUTPUT_ARCH(mips)
ENTRY(__start)

SECTIONS
{
    /* Boot FLASH sections */

    .reset :
    {
        KEEP (*(.reset))
    } > kseg1_reset

    /* KSEG1 exception handler "trampolines" */

    .gen_excpt :
    {
        KEEP (*(.gen_excpt))
    } > kseg1_genexcpt

    .ebase_excpt :
    {
        KEEP (*(.ebase_excpt))
    } > kseg1_ebexcpt

    .bev_excpt :
    {
        KEEP (*(.bev_excpt))
    } > kseg1_bevexcpt

    .int_excpt :
    {
        KEEP (*(.int_excpt))
    } > kseg1_intexcpt

    .dbg_excpt = ORIGIN(kseg1_dbgexcpt);

    .start :
    {
        /* KSEG0 Reset startup logic */

        *(.start)

        /* KSEG0 exception handlers */

        *(.nmi_handler)
        *(.bev_handler)
        *(.int_handler)
    } > kseg0_bootmem

    .dbg_code = ORIGIN(kseg1_dbgcode);

    .devcfg :
    {
        KEEP (*(.devcfg))
    } > kseg1_devcfg

    .devcfg_cp :
    {
        KEEP (*(.devcfg_cp))
    } > kseg1_devcp

    .devcfg_sign :
    {
        KEEP (*(.devcfg_sign))
    } > kseg1_devsign

    /* ABI flags note: not needed in the image; discard it so it is not
     * placed as an orphan into the configuration regions above.
     */

    /DISCARD/ : { *(.MIPS.abiflags) }

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
    } > kseg0_datamem AT > kseg0_progmem

    .eh_frame : ONLY_IF_RW { KEEP (*(.eh_frame)) }
    _gp = ALIGN(16) + 0x7FF0 ;

    .got :
    {
        *(.got.plt) *(.got)
    } > kseg0_datamem AT > kseg0_progmem

    .sdata :
    {
        *(.sdata .sdata.* .gnu.linkonce.s.*)
    } > kseg0_datamem AT > kseg0_progmem

    .lit8 :
    {
        *(.lit8)
    } > kseg0_datamem AT > kseg0_progmem

    .lit4 :
    {
        *(.lit4)
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } >kseg0_datamem AT>kseg0_progmem

    .sbss :
    {
        . = ALIGN(16);
        _sbss = ABSOLUTE(.);
        *(.dynsbss)
        *(.sbss .sbss.* .gnu.linkonce.sb.*)
        *(.scommon)
    } >kseg0_datamem

    .bss :
    {
        *(.dynbss)
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(16);
        _ebss = ABSOLUTE(.);
    } > kseg0_datamem

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
