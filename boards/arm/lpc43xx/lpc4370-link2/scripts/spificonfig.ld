/****************************************************************************
 * boards/arm/lpc43xx/lpc4370-link2/scripts/spificonfig.ld
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

/*
 * Power-Up Reset Overview
 * -----------------------
 *
 * The ARM core starts executing code on reset with the program counter set
 * to 0x0000 0000.  The LPC43xx contains a shadow pointer register that
 * allows areas of memory to be mapped to address 0x0000 0000. The default,
 * reset value of the shadow pointer is 0x1040 0000 so that on reset code in
 * the boot ROM is always executed first.
 *
 * The boot starts after reset is released.  The IRC is selected as CPU clock
 * and the Cortex-M4 starts the boot loader. By default the JTAG access to the
 * chip is disabled at reset.  The boot ROM determines the boot mode based on
 * the OTP BOOT_SRC value or reset state pins.  For flash-based parts, the part
 * boots from internal flash by default.  Otherwise, the boot ROM copies the
 * image to internal SRAM at location 0x1000 0000, sets the ARM's shadow
 * pointer to 0x1000 0000, and jumps to that location.
 *
 * Of course, using JTAG the executable image can be also loaded directly
 * into and executed from SRAM.
 */

/* The LPC4370 on the LPC4370-Link2 has the following memory resources:
 *
 * 1. 4096Kb of SPIFI FLASH beginning at address 0x1400:0000
 * 2. 264KB of total SRAM:
 *    a. 128KB of SRAM in the CPU block beginning at address 0x1000:0000
 *    b. 72KB beginning at address 0x1008:0000 and
 *    c. 64KB of AHB SRAM in three banks beginning at addresses 0x2000:0000,
 *       0x2000:8000 and 0x2000:C000.
 * 3. No internal FLASH
 *
 * Here we assume that:
 *
 * 1. We will be running out of SPIFI flash at 0x1400:0000, and
 * 2. All .data and .bss will all fit into the 128KB CPU SRAM block.
 */

MEMORY
{
    progmem (rx)  : ORIGIN = 0x14000000, LENGTH = 1024K
    datamem (rwx) : ORIGIN = 0x10000000, LENGTH = 128K
}

OUTPUT_ARCH(arm)
ENTRY(__start)         /* Treat __start as the anchor for dead code stripping */
EXTERN(_vectors)       /* Force the vectors to be included in the output */
SECTIONS
{
    .text : {
        _stext = ABSOLUTE(.);
        *(.vectors)
        *(EXCLUDE_FILE (*spifilib*) .text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(EXCLUDE_FILE (*spifilib*) .rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } > progmem

    .init_section : {
        _sinit = ABSOLUTE(.);
        *(.init_array .init_array.*)
        _einit = ABSOLUTE(.);
    } > progmem

    .ARM.extab : {
        *(.ARM.extab*)
    } > progmem

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx : {
        *(.ARM.exidx*)
    } > progmem
    __exidx_end = ABSOLUTE(.);

    _eronly = ABSOLUTE(.);

    .data : {
        _sdata = ABSOLUTE(.);
        *spifilib*(.text .text.*)
        *spifilib*( .rodata .rodata.*)
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > datamem AT > progmem

    .bss : {            /* BSS              */
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > datamem

    /* Stabs debugging sections.    */
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
