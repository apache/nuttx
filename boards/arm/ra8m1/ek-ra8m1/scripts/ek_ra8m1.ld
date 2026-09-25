/****************************************************************************
 * boards/arm/ra8m1/ek_ra8m1/scripts/ek_ra8m1.ld
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

/* EK-RA8M1 (R7FA8M1AHECFC): 2016Kb of code flash, 896Kb of SRAM, plus
 * 64Kb ITCM, 64Kb DTCM, 12Kb data flash, and the fixed-address Option
 * Setting Memory registers.
 *
 * This is a flat (no TrustZone) image: it always runs in the secure state
 * and links against the secure aliases of every memory (User's Manual
 * Table 4.1): code flash at 0x0200_0000, SRAM at 0x2200_0000, data flash at
 * 0x2700_0000, ITCM at 0x0000_0000, DTCM at 0x2000_0000.  (The non-secure
 * aliases in Table 4.2 -- 0x1200_0000, 0x3200_0000, ... -- are not used.)
 * The origins and lengths below match Renesas's own FSP-generated linker
 * script for this board (fsp.ld -> memory_regions.ld in the e2 studio
 * project), which is also a flat project.
 *
 * Option Setting Memory: OFS0/OFS1/OFS2 (watchdog, voltage detection,
 * HOCO, ECC, DCDC), DUALSEL/BANKSEL (dual-bank flash select), BPS/PBPS
 * (block write protection), their _SEL security-attribution selectors,
 * plus the data-flash FSBL/SACC/SAMR registers, all live at fixed
 * addresses outside the normal code-flash range.  Only the secure-region
 * instances are reserved (OFS1 is the register at 0x0300_A200, which the
 * manual calls OFS1_SEC); the non-secure copies are not used.  This script
 * only reserves those addresses as MEMORY regions/output sections; the
 * register images come from const, section-tagged C objects, e.g.
 * `__attribute__((section(".option_setting_ofs0")))` (see
 * arch/arm/src/ra8m1/ra_option_setting.c for OFS0/OFS1/OFS2).  Getting
 * these values wrong can affect flash bank switching, write protection or
 * the watchdogs, so nothing else here should be populated without
 * confirming the bit pattern against the RA8M1 User's Manual.
 *
 * These sections sit ~16Mb above the code flash, so the image must be
 * flashed as ELF/HEX/SREC: a raw `objcopy -O binary` would pad that gap.
 */

MEMORY
{
  flash (rx)  : ORIGIN = 0x02000000, LENGTH = 2016K
  sram (rwx)  : ORIGIN = 0x22000000, LENGTH = 896K
  itcm (rwx)  : ORIGIN = 0x00000000, LENGTH = 64K
  dtcm (rwx)  : ORIGIN = 0x20000000, LENGTH = 64K
  dataflash (rx) : ORIGIN = 0x27000000, LENGTH = 12K

  option_setting_ofs0                 (r) : ORIGIN = 0x0300a100, LENGTH = 4
  option_setting_ofs2                 (r) : ORIGIN = 0x0300a104, LENGTH = 4
  option_setting_dualsel              (r) : ORIGIN = 0x0300a110, LENGTH = 4
  option_setting_ofs1                 (r) : ORIGIN = 0x0300a200, LENGTH = 4
  option_setting_banksel              (r) : ORIGIN = 0x0300a210, LENGTH = 4
  option_setting_bps                  (r) : ORIGIN = 0x0300a240, LENGTH = 16
  option_setting_pbps                 (r) : ORIGIN = 0x0300a260, LENGTH = 16
  option_setting_ofs1_sel             (r) : ORIGIN = 0x0300a280, LENGTH = 4
  option_setting_banksel_sel          (r) : ORIGIN = 0x0300a290, LENGTH = 4
  option_setting_bps_sel              (r) : ORIGIN = 0x0300a2c0, LENGTH = 16
  option_setting_data_flash_fsblctrl0 (r) : ORIGIN = 0x27030080, LENGTH = 4
  option_setting_data_flash_fsblctrl1 (r) : ORIGIN = 0x27030084, LENGTH = 4
  option_setting_data_flash_fsblctrl2 (r) : ORIGIN = 0x27030088, LENGTH = 4
  option_setting_data_flash_sacc0     (r) : ORIGIN = 0x2703008c, LENGTH = 4
  option_setting_data_flash_sacc1     (r) : ORIGIN = 0x27030090, LENGTH = 4
  option_setting_data_flash_samr      (r) : ORIGIN = 0x27030094, LENGTH = 4
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)
SECTIONS
{
    .text :
    {
        _stext = .;
        *(.vectors)
        . = _stext + 0x400;
        KEEP(*(.rom_registers*))
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
        _etext = .;
    } > flash

    .init_section :
    {
        _sinit = .;
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array .ctors))
        _einit = .;
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab*)
    } >flash

    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >flash

    .data :
    {
        _sdata = .;
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = .;
    } > sram AT > flash

    _eronly = LOADADDR(.data);

    .ramfunc ALIGN(4):
    {
        _sramfuncs = .;
        *(.ramfunc  .ramfunc.*)
        _eramfuncs = .;
    } > sram AT > flash

    _framfuncs = LOADADDR(.ramfunc);

    .bss :
    {
        _sbss = .;
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(8);
        _ebss = .;
    } > sram

    /* ITCM/DTCM: volatile, zero out of reset -- like .data/.ramfunc,
     * anything placed here (via __attribute__((section(".itcm")))  or
     * ".dtcm") must be copied in from its flash load address by startup
     * code before use; _eitcmonly/_edtcmonly give that code its source.
     */

    .itcm ALIGN(4) :
    {
        _sitcm = .;
        *(.itcm .itcm.*)
        _eitcm = .;
    } > itcm AT > flash

    _eitcmonly = LOADADDR(.itcm);

    .dtcm ALIGN(4) :
    {
        _sdtcm = .;
        *(.dtcm .dtcm.*)
        . = ALIGN(4);
        _edtcm = .;
    } > dtcm AT > flash

    _edtcmonly = LOADADDR(.dtcm);

    /* Data flash: non-volatile, read in place like any other flash --
     * unlike .itcm/.dtcm this needs no startup copy.
     */

    .dataflash (READONLY) :
    {
        _sdataflash = .;
        KEEP(*(.data_flash .data_flash.*))
        _edataflash = .;
    } > dataflash

    /* Option Setting Memory -- see the file header comment: these
     * placeholder sections only reserve the correct fixed hardware
     * address for each register; they carry no content of their own.
     */

    .option_setting_ofs0 (READONLY) :
    {
        KEEP(*(.option_setting_ofs0))
    } > option_setting_ofs0

    .option_setting_ofs2 (READONLY) :
    {
        KEEP(*(.option_setting_ofs2))
    } > option_setting_ofs2

    .option_setting_dualsel (READONLY) :
    {
        KEEP(*(.option_setting_dualsel))
    } > option_setting_dualsel

    .option_setting_ofs1 (READONLY) :
    {
        KEEP(*(.option_setting_ofs1))
    } > option_setting_ofs1

    .option_setting_banksel (READONLY) :
    {
        KEEP(*(.option_setting_banksel))
    } > option_setting_banksel

    .option_setting_bps (READONLY) :
    {
        KEEP(*(.option_setting_bps))
    } > option_setting_bps

    .option_setting_pbps (READONLY) :
    {
        KEEP(*(.option_setting_pbps))
    } > option_setting_pbps

    .option_setting_ofs1_sel (READONLY) :
    {
        KEEP(*(.option_setting_ofs1_sel))
    } > option_setting_ofs1_sel

    .option_setting_banksel_sel (READONLY) :
    {
        KEEP(*(.option_setting_banksel_sel))
    } > option_setting_banksel_sel

    .option_setting_bps_sel (READONLY) :
    {
        KEEP(*(.option_setting_bps_sel))
    } > option_setting_bps_sel

    .option_setting_data_flash_fsblctrl0 (READONLY) :
    {
        KEEP(*(.option_setting_data_flash_fsblctrl0))
    } > option_setting_data_flash_fsblctrl0

    .option_setting_data_flash_fsblctrl1 (READONLY) :
    {
        KEEP(*(.option_setting_data_flash_fsblctrl1))
    } > option_setting_data_flash_fsblctrl1

    .option_setting_data_flash_fsblctrl2 (READONLY) :
    {
        KEEP(*(.option_setting_data_flash_fsblctrl2))
    } > option_setting_data_flash_fsblctrl2

    .option_setting_data_flash_sacc0 (READONLY) :
    {
        KEEP(*(.option_setting_data_flash_sacc0))
    } > option_setting_data_flash_sacc0

    .option_setting_data_flash_sacc1 (READONLY) :
    {
        KEEP(*(.option_setting_data_flash_sacc1))
    } > option_setting_data_flash_sacc1

    .option_setting_data_flash_samr (READONLY) :
    {
        KEEP(*(.option_setting_data_flash_samr))
    } > option_setting_data_flash_samr

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
