/****************************************************************************
 * boards/arm/imxrt/teensy-4.x/scripts/flash-ocram.ld
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

/* Specify the memory areas */

/* The imxrt162-evk has 8MiB of QSPI FLASH beginning at address,
 * 0x0060:0000, Upto 512Kb of DTCM RAM beginning at 0x2000:0000, and 1MiB OCRAM
 * beginning at 0x2020:0000.  Neither DTCM or SDRAM are used in this
 * configuration.
 *
 * The default flexram setting on the iMXRT 1062 is
 * 256Kib to OCRRAM, 128Kib ITCM and 128Kib DTCM.
 * This can be changed by using a dcd by minipulating
 * IOMUX GPR16 and GPR17.
 * The configuration we will use is 384Kib to OCRRAM, 0Kib ITCM and
 * 128Kib DTCM.
 *
 * This is the OCRAM inker script.
 * The NXP ROM bootloader will move the FLASH image to OCRAM.
 * We must reserve 32K for the bootloader' OCRAM usage from  the OCRAM Size
 * and an additinal 8K for the ivt_s which is IVT_SIZE(8K) This 40K can be
 * reused once the application is running.
 *
 *      0x2020:A000 to 0x202d:ffff - The application Image's vector table
 *      0x2020:8000 to 0x2020:A000 - IVT
 *      0x2020:0000 to 0x2020:7fff - NXP ROM bootloader.
 *
 * We artificially split the FLASH to allow locating sections that we do not
 * want loaded inoto OCRAM. This is to save on OCRAM where the speen of the
 * code does not matter.
 *
 */

MEMORY
{
    flash    (rx)  : ORIGIN = 0x60000000, LENGTH = 7M
    flashxip (rx)  : ORIGIN = 0x60700000, LENGTH = 1M
    /*                     Vectors @ boot+ivt   OCRAM2   Flex RAM     Boot  IVT */
    sram     (rwx) : ORIGIN = 0x2020A000, LENGTH = 512K + 256K + 128K - (32K + 8K)
    itcm     (rwx) : ORIGIN = 0x00000000, LENGTH = 0K
    dtcm     (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
EXTERN(g_flash_config)
EXTERN(g_image_vector_table)
EXTERN(g_boot_data)
EXTERN(g_dcd_data)

ENTRY(_stext)

SECTIONS
{
	/* Image Vector Table and Boot Data for booting from external flash */

	.boot_hdr : ALIGN(4)
	{
		FILL(0xff)
		__boot_hdr_start__ = ABSOLUTE(.) ;
		KEEP(*(.boot_hdr.conf))
		. = 0x1000 ;
		KEEP(*(.boot_hdr.ivt))
		. = 0x1020 ;
		KEEP(*(.boot_hdr.boot_data))
		. = 0x1030 ;
		KEEP(*(.boot_hdr.dcd_data))
		__boot_hdr_end__ = ABSOLUTE(.) ;
		. = 0x2000 ;
	} > flash

	/* Catch all the section we want not in OCRAM so that the *(.text .text.*) in flash does not */

	.flashxip : ALIGN(4)
	{
		FILL(0xff)

		/* Order matters */

		imxrt_start.o(.text)
		imxrt_boot.o(.text)

		*(.slow_memory)
		*(.rodata .rodata.*)
		KEEP(*(__param*))
		*(.fixup)
		*(.gnu.warning)
		*(.gnu.linkonce.t.*)
		*(.glue_7)
		*(.glue_7t)
		*(.got)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)

	} > flashxip

	.text :
	{
		_stext = ABSOLUTE(.);
		*(.vectors)
		*(.text .text.*)
		_etext = ABSOLUTE(.);
	} > sram AT > flash

	.init_section :
	{
		_sinit = ABSOLUTE(.);
		KEEP(*(.init_array .init_array.*))
		_einit = ABSOLUTE(.);
	} > flash

	.ARM.extab :
	{
		*(.ARM.extab*)
	} > flash

	.ARM.exidx :
	{
		__exidx_start = ABSOLUTE(.);
		*(.ARM.exidx*)
		__exidx_end = ABSOLUTE(.);
	} > flash

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

	.ramfunc ALIGN(4):
	{
		_sramfuncs = ABSOLUTE(.);
		*(.ramfunc  .ramfunc.*)
		_eramfuncs = ABSOLUTE(.);
	} > sram AT > flash

	_framfuncs = LOADADDR(.ramfunc);

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
