/****************************************************************************
 * arch/tricore/src/common/tricore.ld
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

OUTPUT_FORMAT("elf32-tricore")
OUTPUT_ARCH("tricore")
ENTRY(__start)
EXTERN(__start)

__USTACK_SIZE = 2K;
CONFIG_TRICORE_CSA_COUNT = 1024;
CONFIG_TRICORE_INITIAL_STACK_OFFSET = 0xEB00;
CONFIG_TRICORE_INITIAL_STACK_SIZE = 512;

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC4X)
__HEAP_SIZE = 512K;
#else
__HEAP_SIZE = 64K;
#endif

MEMORY
{
	dsram0 (w!xp): ORIGIN = 0x70000000, LENGTH = 240K
	psram0 (wx!p): ORIGIN = 0x70100000, LENGTH = 64K
#if defined(CONFIG_ARCH_CHIP_FAMILY_TC4X)
	dlmu   (wr!xp): ORIGIN = 0x90000000, LENGTH = 512K
#else
	dlmu   (wr!xp): ORIGIN = 0x90000000, LENGTH = 96K
#endif
	pflash0 (rx!p): ORIGIN = 0x80000000, LENGTH = 4M
}

REGION_ALIAS("CODE_MEM", pflash0)
REGION_ALIAS("DATA_MEM", dsram0)
REGION_ALIAS("BSS_MEM", dsram0)
REGION_ALIAS("CSA_MEM", dsram0)
REGION_ALIAS("HEAP_MEM", dlmu)

SECTIONS
{
	.start :
	{
		KEEP(*(.start))
		. = ALIGN(4);
	} > CODE_MEM

	.traptab :
	{
		KEEP(*(.traptab))
		. = ALIGN(4);
	} > CODE_MEM

	.ivtab :
	{
		KEEP(*(.ivtab))
		. = ALIGN(4);
	} > CODE_MEM

	.text :
	{
		KEEP(*(.text))
		*(.text .text.*)
		*(.gnu.linkonce.t.*)
		*(.rodata .rodata.*)
		*(.gnu.linkonce.r.*)
		. = ALIGN(2);
	} > CODE_MEM

	.data :
	{
		. = ALIGN(4);
		_sdata = .;
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		. = ALIGN(4);
		_edata = .;
	} > DATA_MEM AT> CODE_MEM
	_data_load = LOADADDR(.data);

	.bss . :
	{
		. = ALIGN(4);
		_sbss = .;
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	} > BSS_MEM
	_data_flash = LOADADDR(.data);

	.ustack : ALIGN(8) FLAGS(aw)
	{
		. += __USTACK_SIZE;
		__USTACK = .;
	} > BSS_MEM

	.istack : ALIGN(8) FLAGS(aw)
	{
		. += CONFIG_ARCH_INTERRUPTSTACK;
		__istack = .;
	} > BSS_MEM

	.csa : ALIGN(64) FLAGS(aw)
	{
		__CSA_BEGIN = .;
		. += CONFIG_TRICORE_CSA_COUNT * 64;
		__CSA_END = .;
	} > CSA_MEM
	__CSA_SIZE = __CSA_END - __CSA_BEGIN;

	.psram_base (NOLOAD) : ALIGN(8)
	{
		. += CONFIG_TRICORE_INITIAL_STACK_OFFSET;
		__PSRAM_USTACK = .;
		. += CONFIG_TRICORE_INITIAL_STACK_SIZE;
		__PSRAM_BOOT_CSA = .;
	} > psram0

	.heap : ALIGN(8) FLAGS(aw)
	{
		_sheap = .;
		. += __HEAP_SIZE;
		_eheap = .;
	} > HEAP_MEM
	_heap_size = _sheap - _eheap;

	/DISCARD/ : { *(.comment) }
	/DISCARD/ : { *(.callinfo) }
}
