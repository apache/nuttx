/****************************************************************************
 * boards/arm/stm32f7/stm32f777zit6-meadow/scripts/flash.ld
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

/* The STM32F777ZIT6 has 2048Kb of main FLASH memory.  This FLASH memory can
 * be accessed from either the AXIM interface at address 0x0800:0000 or from
 * the ITCM interface at address 0x0020:0000.
 *
 * Additional information, including the option bytes, is available at at
 * FLASH at address 0x1ff0:0000.
 *
 * In the STM32F777ZIT6, two different boot spaces can be selected through
 * the BOOT pin and the boot base address programmed in the BOOT_ADD0 and
 * BOOT_ADD1 option bytes:
 *
 *   1) BOOT=0: Boot address defined by user option byte BOOT_ADD0[15:0].
 *      ST programmed value: Flash on ITCM at 0x0020:0000
 *   2) BOOT=1: Boot address defined by user option byte BOOT_ADD1[15:0].
 *      ST programmed value: System bootloader at 0x0010:0000
 *
 * NuttX does not modify these option bytes.  On the unmodified STM32F777ZIT6
 * MEADOW board, the BOOT0 pin is at ground so by default, the STM32 will boot
 * to address 0x0020:0000 in ITCM FLASH.
 *
 * The STM32F777ZIT6 also has 512Kb of data SRAM (in addition to ITCM SRAM).
 * SRAM is split up into three blocks:
 *
 *   1) 128Kb of DTCM SRM beginning at address 0x2000:0000
 *   2) 368Kb of SRAM1 beginning at address 0x2002:0000
 *   3)  16Kb of SRAM2 beginning at address 0x2007:c000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0800:0000 address range.
 */

MEMORY
{
  itcm  (rwx) : ORIGIN = 0x00200000, LENGTH = 2048K
  flash (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  dtcm  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
  sram1 (rwx) : ORIGIN = 0x20020000, LENGTH = 368K
  sram2 (rwx) : ORIGIN = 0x2007c000, LENGTH = 16K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)
SECTIONS
{
    .text : {
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

    .init_section : {
        _sinit = ABSOLUTE(.);
        *(.init_array .init_array.*)
        _einit = ABSOLUTE(.);
    } > flash

    .ARM.extab : {
        *(.ARM.extab*)
    } > flash

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx : {
        *(.ARM.exidx*)
    } > flash
    __exidx_end = ABSOLUTE(.);

    _eronly = ABSOLUTE(.);

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram1 AT > flash

    .bss : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > sram1

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
