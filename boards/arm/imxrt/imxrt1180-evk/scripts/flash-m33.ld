/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/scripts/flash-m33.ld
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

/* Linker script for the imxrt1180-evk NuttX build targeting the Cortex-M33
 * core (nsh-m33 config).  This is a fully independent image; there is no
 * M33 stub in front of it — the ROM's AHAB flow loads NuttX itself onto
 * the M33.
 *
 * Layout of flash.bin (assembled by tools/imxrt1180/build_flash_image.sh):
 *
 *   0x2800_0400  FCB (baked into this image via .boot_hdr.conf)
 *   0x2800_1000  NXP ELE FW AHAB container (injected by build script)
 *   0x2800_?000  Our AHAB container header (injected by build script)
 *   0x2800_B000  This image's .vectors and .text (XIP)
 *
 * The M33 image is confined to the first 512 KB of FlexSPI1 NOR; the
 * Cortex-M7 image (scripts/flash.ld) starts at 0x2808_0000.
 *
 * Data/BSS live in the M33 System TCM at 0x2000_0000 (128 KB).  Code TCM
 * (0x0FFE_0000, 128 KB) is left alone for possible ramfuncs.
 *
 * USB DMA buffers cannot live in System TCM, since it is not reachable
 * by the USB controller's DMA engine.  These buffers are placed in the
 * `.dmamemory` section in OCRAM2 instead (see imxrt_dma_alloc.c).
 */

MEMORY
{
  flash (rx)  : ORIGIN = 0x28000000, LENGTH = 512K   /* FlexSPI XIP window reserved for the M33 image */
  systcm (rwx): ORIGIN = 0x20000000, LENGTH = 128K   /* CM33 private System TCM */
  ocram2 (rw) : ORIGIN = 0x20500000, LENGTH = 16K    /* OCRAM2 (DMA/TRDC accessible); USB DMA buffers only */
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
EXTERN(g_flash_config)
ENTRY(_stext)

SECTIONS
{
    /* FCB slot at the ROM-mandated offset. */

    .fcb 0x28000400 : ALIGN(4)
    {
        KEEP(*(.boot_hdr.conf))
        . = ALIGN(4);
    } > flash

    /* Vector table + code at the fixed XIP address the ROM's AHAB flow
     * will jump to after loading the ELE FW and validating our container.
     */

    .text 0x2800B000 : ALIGN(4)
    {
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

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        *(.init_array .init_array.*)
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
    } > systcm AT > flash

    .ramfunc ALIGN(4):
    {
        _sramfuncs = ABSOLUTE(.);
        *(.ramfunc  .ramfunc.*)
        _eramfuncs = ABSOLUTE(.);
    } > systcm AT > flash

    _framfuncs = LOADADDR(.ramfunc);

    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > systcm

    /* USB DMA-reachable buffers (endpoint queue heads/transfer
     * descriptors, etc).  See the OCRAM2 rationale above.
     */

    .dmamemory (NOLOAD) :
    {
        _sdmamemory = ABSOLUTE(.);
        *(.dmamemory .dmamemory.*)
        . = ALIGN(4);
        _edmamemory = ABSOLUTE(.);
    } > ocram2

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

    _boot_loadaddr = ORIGIN(flash);
    _boot_size = LENGTH(flash);
    _ram_size = LENGTH(systcm);
}

/* The M7 image starts at 0x2808_0000.  Fail the link rather than silently
 * producing an M33 image that runs into it.
 */

ASSERT(ORIGIN(flash) + LENGTH(flash) <= 0x28080000,
       "M33 flash region overlaps the M7 image at 0x28080000")
