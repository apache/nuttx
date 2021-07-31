/****************************************************************************
 * boards/mips/pic32mz/chipkit-wifire/scripts/mips-debug.ld
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
    /* The PIC32MZ2048ECH144 and PIC32MZ2048ECM144 chips have has 2048Kb of
     * program FLASH at physical address 0x1d000000 but is always accessed
     * at KSEG0 address 0x9d00:0000
     */

    kseg0_program_mem  (rx) : ORIGIN = 0x9d000000, LENGTH = 2048K

    /* The PIC32MZ2048ECH144 and PIC32MZ2048ECM144 chips have 160Kb of boot
     * FLASH:  80Kb at physical address 0x1fc4000 (Boot Flash 1, boot1) and
     * 80Kb at physical address 0x1fc60000 (Boot Flash 2, boot2).  Either
     * may be mapped to the lower boot alias region (0x1fc00000,
     * boolalias1) or the upper boot alias region (0x1fc20000, bootalias2).
     * This linker script assumes that Boot Flash 1 is mapped to the lower
     * alias region and Boot Flash 2 to the upper region.
     *
     * NOTE: This linker script simply writes into the lower boot alias,
     * whichever boot FLASH that may correspond to.  The other boot FLASH
     * is simply ignored.
     *
     * The initial reset vector is in KSEG1, but all other accesses are in
     * KSEG0.
     *
     *   REGION        PHYSICAL   KSEG   SIZE
     *   DESCRIPTION   START ADDR        (BYTES)
     *   ------------- ---------- ------ ----------------------
     *   Exceptions:*
     *     Reset       0x1fc00000 KSEG1  512         512
     *     TLB Refill  0x1fc00200 KSEG1  256         768
     *     Cache Error 0x1fc00300 KSEG1  128         896
     *     Others      0x1fc00380 KSEG1  128        1024 (1Kb)
     *     Interrupt   0x1fc00400 KSEG1  128        1152
     *     JTAG        0x1fc00480 KSEG1  16         1168
     *   Exceptions    0x1fc00490 KSEG0  8192-1168  8192 (4Kb)
     *   Debug code    0x1fc02000 KSEG1  4096-16   12272
     *   ADEVCFG3-0    0x1fc0ff40 KSEG1  16        12288 (12Kb)
     *   DEVCFG3-0     0x1fc0ffc0 KSEG1  16        12288 (12Kb)
     *
     * Exceptions assume:
     *
     *   STATUS: BEV=0/1 and EXL=0
     *   CAUSE:  IV=1
     *   JTAG:   ProbEn=0
     *   And multi-vector support disabled
     */

    kseg1_reset    (rx) : ORIGIN = 0xbfc00000, LENGTH = 384
    kseg1_genexcpt (rx) : ORIGIN = 0xbfc00180, LENGTH = 128
    kseg1_ebexcpt  (rx) : ORIGIN = 0xbfc00200, LENGTH = 128
    kseg1_bevexcpt (rx) : ORIGIN = 0xbfc00380, LENGTH = 128
    kseg1_intexcpt (rx) : ORIGIN = 0xbfc00400, LENGTH = 128
    kseg1_dbgexcpt (rx) : ORIGIN = 0xbfc00480, LENGTH = 16
    kseg0_bootmem  (rx) : ORIGIN = 0x9fc004ac, LENGTH = 8192-1196
    kseg1_dbgcode  (rx) : ORIGIN = 0xbfc02000, LENGTH = 4096-16
    kseg1_adevcfg   (r) : ORIGIN = 0x1fc0ff40, LENGTH = 128
    kseg1_devcfg    (r) : ORIGIN = 0x1fc0ffc0, LENGTH = 128

    /* The The PIC32MZ2048ECH144 and PIC32MZ2048ECM144 chips have has 512Kb
     * of data memory at physical address 0x00000000.  Since the PIC32MZ
     * has no data cache, this memory is always accessed through KSEG1.
     *
     * When used with MPLABX, we need to set aside 512 bytes of memory
     * for use by MPLABX and 128 for DSP register storage.
     */

    kseg1_data_mem (rw!x) : ORIGIN = 0xa0000200, LENGTH = 512K - 640
}

OUTPUT_FORMAT("elf32-tradlittlemips")
OUTPUT_ARCH(pic32mx)
ENTRY(__start)

SECTIONS
{
    /* Boot FLASH sections */

    .reset :
    {
        KEEP (*(.reset))
    } > kseg1_reset

    /* Exception handlers.  The following is assumed:
     *
     *   STATUS: BEV=1 and EXL=0
     *   CAUSE:  IV=1
     *   JTAG:   ProbEn=0
     *   And multi-vector support disabled
     *
     * In that configuration, the vector locations become:
     *
     *   Reset, Soft Reset  bfc0:0000
     *   TLB Refill         bfc0:0200
     *   Cache Error        bfc0:0300
     *   All others         bfc0:0380
     *   Interrupt          bfc0:0400
     *   EJTAG Debug        bfc0:0480
     */

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

    .adevcfg :
    {
        KEEP (*(.adevcfg))
    } > kseg1_adevcfg

    .devcfg :
    {
        KEEP (*(.devcfg))
    } > kseg1_devcfg

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
    } > kseg0_program_mem

    /* Initialization data begins here in progmem */

    _data_loaddr = LOADADDR(.data);

    .eh_frame_hdr : { *(.eh_frame_hdr) *(.eh_frame_entry .eh_frame_entry.*) }
    .eh_frame : ONLY_IF_RO { KEEP (*(.eh_frame)) }

    /* RAM functions are positioned at the beginning of RAM so that
     * they can be guaranteed to satisfy the 2Kb alignment requirement.
     */

/* This causes failures if there are no RAM functions
    .ramfunc ALIGN(2K) :
    {
        _sramfunc =  ABSOLUTE(.);
        *(.ramfunc  .ramfunc.*)
        _eramfunc =  ABSOLUTE(.);
    } > kseg1_data_mem AT > kseg0_program_mem

    _ramfunc_loadaddr = LOADADDR(.ramfunc);
    _ramfunc_sizeof   = SIZEOF(.ramfunc);
    _bmxdkpba_address = _sramfunc - ORIGIN(kseg1_data_mem) ;
    _bmxdudba_address = LENGTH(kseg1_data_mem) ;
    _bmxdupba_address = LENGTH(kseg1_data_mem) ;
*/

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        KEEP (*(.gnu.linkonce.d.*personality*))
        *(.data1)
    } > kseg1_data_mem AT > kseg0_program_mem

    .eh_frame : ONLY_IF_RW { KEEP (*(.eh_frame)) }
    _gp = ALIGN(16) + 0x7FF0 ;

    .got :
    {
        *(.got.plt) *(.got)
    } > kseg1_data_mem AT > kseg0_program_mem

    .sdata :
    {
        *(.sdata .sdata.* .gnu.linkonce.s.*)
    } > kseg1_data_mem AT > kseg0_program_mem

    .lit8 :
    {
        *(.lit8)
    } > kseg1_data_mem AT > kseg0_program_mem

    .lit4 :
    {
        *(.lit4)
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } >kseg1_data_mem AT>kseg0_program_mem

    .sbss :
    {
        . = ALIGN(16);
        _sbss = ABSOLUTE(.);
        *(.dynsbss)
        *(.sbss .sbss.* .gnu.linkonce.sb.*)
        *(.scommon)
    } >kseg1_data_mem

    .bss :
    {
        *(.dynbss)
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(16);
        _ebss = ABSOLUTE(.);
    } > kseg1_data_mem

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
